/**
 * @file Wheel_Leg_about.c
 * @brief 轮腿控制核心算法：LQR 增益拟合、防劈叉、横滚补偿、腿长控制、
 *        速度/距离误差计算、车身速度融合及惯性导航解算。
 *
 * 本文件提供：
 *   - LQR_Get_K(): 二维多项式拟合 LQR 反馈增益矩阵
 *   - AntiSplit_Get_K(): 防劈叉 PID 增益随腿长的一维二次拟合
 *   - Roll_Comp(): 横滚角补偿控制
 *   - Leg_L0_Control(): 带斜坡的腿长 PD 控制
 *   - Speed_Error_Set(): 前进速度误差计算（支持小陀螺模式）
 *   - Distance_Error_Set(): 车身位移误差累加
 *   - Body_Speed_Coculate(): 基于轮速和腿摆动融合的车身速度估计
 *   - INS_Coculate(): 惯性导航数据解算（pitch/yaw 和角速度）
 */

#include "Wheel_Leg_about.h"
#include "imu_temp_ctrl.h"
#include "user_pid.h"
#include "chassis_behavior_tree.h"
#include "remoter.h"
#include "PowerCtrl.h"
#include "VMC.h"
#include "observe_task.h"
#include "Motor_Drv.h"
#include "Gimbal.h"
#include "Maths_about.h"
#include "body_speed_state.h"
#include <math.h>

/*============================ 轮腿相关算法 ================================*/

/* 速度斜坡句柄（非小陀螺路径） */
static RampStep_Handle target_speed_ramp;
static uint8_t target_speed_ramp_inited = 0;


/**
 * @brief 二维多项式拟合 LQR 反馈增益矩阵 K(L0_l, L0_r)。
 *
 * 对于 LQR 的 4×12 增益矩阵 K，每个元素 K[i][j] 通过二次多项式插值获得：
 *   K[i][j] = p00 + p10·L0_l + p01·L0_r + p20·L0_l² + p11·L0_l·L0_r + p02·L0_r²
 * 其中系数 p00~p02 由预拟合表 K_Fit_Coefficients[48][6] 提供
 * （48 = 4行 × 12列）。
 *
 * @param[out] LQR                 4×12 反馈增益矩阵，填充结果。
 * @param[in]  K_Fit_Coefficients  48×6 拟合系数表（行优先）。
 * @param[in]  L0_l               左腿当前腿长，单位 m。
 * @param[in]  L0_r               右腿当前腿长，单位 m。
 */
void LQR_Get_K(float LQR[4][12], float K_Fit_Coefficients[48][6], float L0_l, float L0_r)
{
    for (uint8_t i = 0; i < 4; i++) {
        for (uint8_t j = 0; j < 12; j++) {
            uint8_t pos = i * 12 + j;

            float p00 = K_Fit_Coefficients[pos][0];
            float p10 = K_Fit_Coefficients[pos][1];
            float p01 = K_Fit_Coefficients[pos][2];
            float p20 = K_Fit_Coefficients[pos][3];
            float p11 = K_Fit_Coefficients[pos][4];
            float p02 = K_Fit_Coefficients[pos][5];

            LQR[i][j] = p00
                      + p10 * L0_l
                      + p01 * L0_r
                      + p20 * L0_l * L0_l
                      + p11 * L0_l * L0_r
                      + p02 * L0_r * L0_r;
        }
    }
}

/**
 * @brief 防劈叉 PID 增益的 1D 二次拟合：K(L0_avg) = p0 + p1·L0_avg + p2·L0_avg²。
 *
 * 根据平均腿长 L0_avg 在线计算防劈叉 PD 控制器的 Kp 和 Kd。
 *
 * @param[out] K      输出增益数组，K[0]=Kp, K[1]=Kd。
 * @param[in]  Coef   2×3 系数表，行 0 为 Kp 系数，行 1 为 Kd 系数，
 *                    每行：[p0, p1, p2]。
 * @param[in]  L0_avg 当前左右腿长的均值，单位 m。
 */
void AntiSplit_Get_K(float K[2], float Coef[2][3], float L0_avg)
{
    for (uint8_t i = 0; i < 2; i++) {
        K[i] = Coef[i][0]
             + Coef[i][1] * L0_avg
             + Coef[i][2] * L0_avg * L0_avg;
    }
}

/**
 * @brief 小陀螺模式下的平移方向倍率计算。
 *
 * 传入当前车身 yaw 角 a、窗口半宽度 tol 和方向偏置 offset，
 * 返回车身朝向与目标方向的匹配倍率：
 *   - |a - offset| ≤ tol            → ratio 从 0 线性降至 -1（方向重合时 -1）
 *   - |a - offset ± π| ≤ tol        → ratio 从 0 线性升至 +1（反向时 +1）
 *   - 其余                          → ratio = 0
 *
 * offset 用于补偿小陀螺平移方向与遥控器方向之间的安装偏置。
 *
 * @param[in] a      当前车身 yaw 角，单位 rad。
 * @param[in] tol    窗口半宽度，单位 rad。
 * @param[in] offset 方向偏置补偿角，单位 rad。
 * @return 平移倍率 ratio，范围 [-1, 1]。
 */
static float Spin_Translation_Ratio(float a, float tol, float offset)
{
    if (tol <= 1e-6f) return 0.0f;
    a -= offset;
    while (a >  PI) a -= 2.0f * PI;
    while (a < -PI) a += 2.0f * PI;
    if (a >= -tol && a <= tol)         return -(tol - fabsf(a)) / tol;
    if (a >=  PI - tol)                return  (tol - fabsf(a - PI)) / tol;
    if (a <= -PI + tol)                return  (tol - fabsf(a + PI)) / tol;
    return 0.0f;
}

/**
 * @brief 计算前进速度误差 speed_error。
 *
 * 根据当前腿长状态和功率限制设定速度上限 speed_limit（小陀螺模式提高上限），
 * 从遥控器/云台头目标速度中提取 target_body_speed，并叠加小陀螺方向倍率或
 * yaw 误差衰减系数，最终与卡尔曼滤波速度 kalman_body_speed 做差得到 speed_error。
 *
 * speed_error 最终限幅为 speed_limit 的 70%。
 *
 * @note 小陀螺模式下使用矢量模长作为速度目标，通过 Spin_Translation_Ratio
 *       确定沿车身前向的投影方向。
 */
void Speed_Error_Set()
{
    if (Foot_Chassis.Target_Leg_State == 1) {
        speed_limit = 2.0f;
    } else {
        // 长腿：speed_limit 随 base_power_limit 在 [60W, 100W] 区间从 2.1 线性升到 2.5
        if (base_power_limit <= 60.0f)         speed_limit = 2.1f;
        else if (base_power_limit >= 100.0f)   speed_limit = 2.5f;
        else                                   speed_limit = 2.1f + (base_power_limit - 60.0f) * 0.01f;
    }
    if (spinning_flag == 1) {
        speed_limit = 4.0f;     // 小陀螺时限制平移最大速度
    }

    float temp;
    if (spinning_flag == 1) {
        // 小陀螺：(Vx,Vy) 是云台头坐标系下的速度向量（Vy=前，Vx=右）
        float v_mag = sqrtf(Foot_Chassis.Target_Vx * Foot_Chassis.Target_Vx
                          + Foot_Chassis.Target_Vy * Foot_Chassis.Target_Vy);
        float v_dir = atan2f(Foot_Chassis.Target_Vx, Foot_Chassis.Target_Vy);

        if (v_mag > speed_limit) v_mag = speed_limit;
        target_body_speed = PowerCtrl_LimitTargetSpeed(v_mag, speed_limit);

        // 开窗中心=目标方向+偏置；车体朝向接近时 ratio 给出沿车体前向的符号
        temp = Spin_Translation_Ratio(yaw_angle_PI, spin_speed_tol_angle,
                                      v_dir + spin_speed_angle_offset);
    } else {
        // 常态：车体跟随云台头，只用 Vy（前后）
        {
            float raw_target = Foot_Chassis.Target_Vy;
            raw_target = PowerCtrl_LimitTargetSpeed(raw_target, speed_limit);

            /* 根据腿长插值斜坡参数：短腿(0.23m) → 阶跃1.2 速率0.5，长腿(0.39m) → 阶跃0.3 速率0.2 */
            float leg_ratio = (target_Leg_L0 - LEG_MIN_LENTH) / (LEG_MAX_LENTH - LEG_MIN_LENTH);
            if (leg_ratio < 0.0f) leg_ratio = 0.0f;
            if (leg_ratio > 1.0f) leg_ratio = 1.0f;
            float step_accel = 1.2f + leg_ratio * (0.3f - 1.2f);
            float step_decel = 0.8f + leg_ratio * (0.3f - 0.8f);
            float ramp_rate  = 1.0f + leg_ratio * (0.2f - 1.0f);

            if (!target_speed_ramp_inited) {
                RampStep_Init(&target_speed_ramp, target_body_speed);
                target_speed_ramp_inited = 1;
            }
            target_body_speed = RampStep_Update(&target_speed_ramp, raw_target,
                                                 step_accel, step_decel, ramp_rate, 0.002f);
        }

        // yaw 误差越大，目标速度越小
        temp = 1.0f - fabsf(yaw_error) / 0.7f;
        if (temp < 0.0f) temp = 0.0f;
    }
    

    target_body_speed = target_body_speed * temp;
    speed_error = target_body_speed - kalman_body_speed;

    if (speed_error >= speed_limit * 0.7f)
        speed_error = speed_limit * 0.7f;
    if (speed_error <= -speed_limit * 0.7f)
        speed_error = -speed_limit * 0.7f;
}

/**
 * @brief 计算车身位移误差 body_distance_error。
 *
 * 以 0.002s 为周期，对 kalman_body_speed 积分得到实际位移 body_distance，
 * 对 (kalman_body_speed + speed_error) 积分得到目标位移 target_body_distance，
 * 两者之差即为 body_distance_error。
 *
 * 调用 BodySpeedState_Get() 判断当前车身速度状态：
 * - ACCEL / STATIONARY：正常积分累加位移
 * - DECEL（减速滑行）：将 body_distance 和 target_body_distance 清零，
 *   避免减速期间位移积分干扰下一段加速。
 *
 * @warning speed_error 是经限幅后的值，而 target_body_speed 未经限幅，
 *          因此 kalman_body_speed + speed_error ≠ target_body_speed。
 */
void Distance_Error_Set()
{
    /* 同步更新车身速度状态，后续逻辑据此决定是否积分位移 */
    BodySpeedState_t speed_state = BodySpeedState_Get();

    /* 减速滑行期间不积分位移，防止位移误差累积影响下一段加速 */
    if (speed_state == BODY_SPEED_DECEL) {
        body_distance = 0.0f;
        target_body_distance = 0.0f;
        body_distance_error = 0.0f;
        return;
    }

    /* ACCEL 或 STATIONARY 状态：正常积分累加 */
    body_distance += kalman_body_speed * 0.002f;
    target_body_distance += target_body_speed * 0.002f;
    body_distance_error = target_body_distance - body_distance;
}

/**
 * @brief 基于轮速和腿摆动融合的车身速度估计。
 *
 * 分别对左/右轮：
 *   1) 轮速 Wl/Wr = α·(电机转速 + 腿角速度 d_b_phi0) + (1-α)·上一周期值
 *   2) 单侧车身速度 = α·(W × R + d_b_phi0 × L0 × cos(b_phi0)) + (1-α)·上一周期值
 *   3) 车身速度 body_speed = (body_speed_L + body_speed_R) / 2
 *
 * 其中 α_W 和 α_body_speed 为低通滤波系数。
 */
void Body_Speed_Coculate()
{
    // 算单侧轮子速度
    Wl = alpha_W * (-L_DJ3508.Rx_Data.Velocity + VMC_L.d_b_phi0) + (1 - alpha_W) * Wl;
    // 算单侧车身速度
    body_speed_L = alpha_body_speed * ((Wl * WHEEL_RADIUS)
                   + VMC_L.d_b_phi0 * VMC_L.L0 * arm_cos_f32(VMC_L.b_phi0))
                   + (1 - alpha_body_speed) * body_speed_L;

    Wr = alpha_W * (R_DJ3508.Rx_Data.Velocity + VMC_R.d_b_phi0) + (1 - alpha_W) * Wr;
    body_speed_R = alpha_body_speed * ((Wr * WHEEL_RADIUS)
                   + VMC_R.d_b_phi0 * VMC_R.L0 * arm_cos_f32(VMC_R.b_phi0))
                   + (1 - alpha_body_speed) * body_speed_R;

    body_speed = (body_speed_L + body_speed_R) / 2.0f;
}

/**
 * @brief 惯性导航系统数据处理。
 *
 * 计算 pitch 和 yaw 的角度及角速度：
 *   - d_pitch: 低通滤波后的俯仰角速度
 *   - d_yaw:   低通滤波后的偏航角速度（处理角度环绕）
 *
 * 采样周期假设为 0.002s（500 Hz）。
 */
void INS_Coculate()
{
    task_Pitch_Coculate();

    yaw_trans[1] = yaw_trans[0];
    yaw_trans[0] = (yaw / 180.0f) * PI;
    d_pitch = alpha_d_pitch * ((pitch_trans[0] - pitch_trans[1]) / 0.002f)
            + (1 - alpha_d_pitch) * d_pitch;
    float temp = yaw_trans[0] - yaw_trans[1];
    if (temp > PI)
        temp -= 2 * PI;
    else if (temp < -PI)
        temp += 2 * PI;
    d_yaw = alpha_d_yaw * (temp / 0.002f) + (1 - alpha_d_yaw) * d_yaw;
}