/**
 * @file Lqr_Error_Calculate.c
 * @brief LQR 误差计算模块：速度误差、位移误差、Yaw 误差。
 *
 * 在 LQR 计算前统一更新三个状态误差：
 *  - Speed_Error_Set():   前进速度误差 speed_error（支持小陀螺模式）
 *  - Distance_Error_Set(): 车身位移积分误差 body_distance_error
 *  - Yaw_Error_Coculate():  Yaw 角度误差 yaw_error（含速度相关限幅并触发 Speed_Error_Set）
 */

#include "Lqr_Error_Calculate.h"
#include "chassis_behavior_tree.h"
#include "PowerCtrl.h"
#include "observe_task.h"
#include "ramp_generator.h"
#include "body_speed_state.h"
#include "Motor_Drv.h"
#include "user_pid.h"
#include "Maths_about.h"
#include "Gimbal.h"
#include <math.h>

/* ---- 本模块内部状态（原 Wheel_Leg_about.c / yaw_error.c 搬迁） ---- */

/** @brief 非小陀螺路径的斜坡发生器句柄 */
static RampStep_Handle target_speed_ramp;
static uint8_t target_speed_ramp_inited = 0;

/* ---- 内部辅助函数 ---- */

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

/* ---- 对外接口 ---- */

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

//region[rgb(0, 50, 0)] 速度最大值判断
    // 长短腿判断
    if (Foot_Chassis.Target_Leg_State == 1) // 长腿
    {
        speed_limit = 2.0f;
    } 
    else // 短腿
    {
        // 长腿：speed_limit 随 base_power_limit 在 [60W, 100W] 区间从 2.1 线性升到 2.5
        if (base_power_limit <= 60.0f)         speed_limit = 2.1f;
        else if (base_power_limit >= 100.0f)   speed_limit = 2.5f;
        else                                   speed_limit = 2.1f + (base_power_limit - 60.0f) * 0.01f;
    }
    if (spinning_flag == 1) {
        speed_limit = 4.0f;     // 小陀螺时限制平移最大速度
    }
//endregion

    float temp;
    if (spinning_flag == 1) // 小陀螺
    {
    //!云台头坐标系下的速度向量（Vy=前，Vx=右）

        // 目标速度模长，单位 m/s
        float v_mag = sqrtf(Foot_Chassis.Target_Vx * Foot_Chassis.Target_Vx
                          + Foot_Chassis.Target_Vy * Foot_Chassis.Target_Vy);
        // 目标速度方向，单位 rad
        float v_dir = atan2f(Foot_Chassis.Target_Vx, Foot_Chassis.Target_Vy);

        // 限速限功率
        if (v_mag > speed_limit) v_mag = speed_limit;
        target_body_speed = PowerCtrl_LimitTargetSpeed(v_mag, speed_limit);

        // 开窗中心=目标方向+偏置 

        // 计算当前小陀螺平移方向倍率，yaw误差越大，倍率越小
        temp = Spin_Translation_Ratio(yaw_angle_PI, spin_speed_tol_angle,
                                      v_dir + spin_speed_angle_offset);
    } 
    else 
    {
        // 计算目标速度 target_body_speed，带斜坡
        {
            float raw_target = Foot_Chassis.Target_Vy;
            raw_target = PowerCtrl_LimitTargetSpeed(raw_target, speed_limit);

            /* 根据腿长插值斜坡参数（此时不计算斜坡）：短腿(0.23m) → 阶跃1.2 速率0.5，长腿(0.39m) → 阶跃0.3 速率0.2 */

            // 计算腿长比例，范围 [0, 1]
            float leg_ratio = (target_Leg_L0 - LEG_MIN_LENTH) / (LEG_MAX_LENTH - LEG_MIN_LENTH);
            if (leg_ratio < 0.0f) leg_ratio = 0.0f;
            if (leg_ratio > 1.0f) leg_ratio = 1.0f;

            float step_accel = 1.2f + leg_ratio * (0.3f - 1.2f);
            float step_decel = 0.8f + leg_ratio * (0.3f - 0.8f);
            float ramp_rate  = 1.0f + leg_ratio * (0.2f - 1.0f);

            // 计算斜坡后的值
            if (!target_speed_ramp_inited) 
            {
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
 * 对 target_body_speed 积分得到目标位移 target_body_distance（不是用带限幅后的 speed_error），
 * 两者之差即为 body_distance_error。
 *
 * 调用 BodySpeedState_Get() 判断当前车身速度状态：
 * - ACCEL / STATIONARY：正常积分累加位移
 * - DECEL（减速滑行）：将 body_distance 和 target_body_distance 清零，
 *   避免减速期间位移积分干扰下一段加速。
 */
void Distance_Error_Set()
{
    /* 同步更新车身速度状态，后续逻辑据此决定是否积分位移 */
    int speed_state = BodySpeedState_Get();

    /* 减速滑行期间不积分位移，防止位移误差累积影响下一段加速 */
    if (speed_state == BODY_SPEED_DECEL || spinning_flag == 1) 
    {
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
 * @brief 计算 Yaw 误差 yaw_error 并触发速度误差更新。
 *
 * 1) 读取云台 Yaw 电机位置，减去零点偏置 head_forward_angle 得到原始 yaw_error
 * 2) 根据车身速度 kalman_body_speed 动态计算 yaw_error_max（速度越快允差越小）
 * 3) 调用 Speed_Error_Set() 更新前进速度误差（此时 yaw_error 尚未限幅，保证速度衰减正确）
 * 4) 对 yaw_error 做限幅（±yaw_error_max），防止大角度误差导致 LQR 过度补偿
 *
 * @note Speed_Error_Set() 中用到 yaw_error 做速度衰减，因此必须在限幅前调用。
 */
void Yaw_Error_Coculate()
{
    float Yaw_motor_position;
    Yaw_motor_position = Yaw_DM4310.Rx_Data.Position - (head_forward_angle);//减的是零点

    //套圈处理
    if(Yaw_motor_position > PI)
    {
        Yaw_motor_position -= 2 * PI;
    }
    if(Yaw_motor_position < -PI)
    {
        Yaw_motor_position += 2 * PI;
    }
    yaw_error = Yaw_motor_position;

    float yaw_error_max = 0;
    yaw_error_max = 2.5f - fabsf(kalman_body_speed);//速度越快，允许的yaw误差越小，最大为5度，最小为0.05度
    if(yaw_error_max <= 0.05f)
    {
        yaw_error_max = 0.05f;
    }

    //这里Speed_Error_Set要用的是原生yaw_error，所以要写在yaw_error_max之上
    Speed_Error_Set();

    if(yaw_error > yaw_error_max)
        yaw_error = yaw_error_max;
    if(yaw_error < -yaw_error_max)
        yaw_error = -yaw_error_max;
}