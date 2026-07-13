/**
 * @file Wheel_Leg_about.c
 * @brief 轮腿控制核心算法：LQR 增益拟合、防劈叉、横滚补偿、腿长控制、
 *        车身速度融合及惯性导航解算。
 *
 * 本文件提供：
 *   - LQR_Get_K(): 二维多项式拟合 LQR 反馈增益矩阵
 *   - AntiSplit_Get_K(): 防劈叉 PID 增益随腿长的一维二次拟合
 *   - Roll_Comp(): 横滚角补偿控制
 *   - Leg_L0_Control(): 带斜坡的腿长 PD 控制
 *   - Body_Speed_Coculate(): 基于轮速和腿摆动融合的车身速度估计
 *   - INS_Coculate(): 惯性导航数据解算（pitch/yaw 和角速度）
 *
 * Speed_Error_Set()、Distance_Error_Set()、Yaw_Error_Coculate() 已迁移至
 * Lqr_Error_Calculate.c，统一管理 LQR 状态误差计算。
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
 * @brief 防劈叉 PID 增益的一次函数计算：K(L) = a·L + b（线性插值）。
 *
 * 根据平均腿长 L0_avg 在线计算防劈叉 PD 控制器的 Kp 和 Kd。
 * 系数硬编码为两点插值结果（零次项 + 一次项），不再由外部传入。
 *
 * @param[out] AntiSplit_Kp 输出 Kp 增益。
 * @param[out] AntiSplit_Kd 输出 Kd 增益。
 * @param[in]  L0_avg       当前左右腿长的均值，单位 m。
 */
void AntiSplit_Get_K(float *AntiSplit_Kp, float *AntiSplit_Kd, float L0_avg)
{
    // 最短腿(0.23m)和最长腿(0.39m)对应的 Kp、Kd 端点值
    const float Kp_short = 300.0f,  Kp_long = 700.0f;
    const float Kd_short = 10.0f,   Kd_long = 200.0f;

    float t = (L0_avg - LEG_MIN_LENTH) / (LEG_MAX_LENTH - LEG_MIN_LENTH);
    *AntiSplit_Kp = Kp_short + (Kp_long - Kp_short) * t;
    *AntiSplit_Kd = Kd_short + (Kd_long - Kd_short) * t;
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