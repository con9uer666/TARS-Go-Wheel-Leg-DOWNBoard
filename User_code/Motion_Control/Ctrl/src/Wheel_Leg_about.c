/**
 * @file Wheel_Leg_about.c
 * @brief 轮腿控制核心算法：横滚补偿、腿长控制、车身速度融合及惯性导航解算。
 *
 * 本文件提供：
 *   - Roll_Comp(): 横滚角补偿控制
 *   - Leg_L0_Control(): 带斜坡的腿长 PD 控制
 *   - Body_Speed_Coculate(): 基于轮速和腿摆动融合的车身速度估计
 *   - INS_Coculate(): 惯性导航数据解算（pitch/yaw 和角速度）
 *
 * Yaw_Error_Calculate()、Distance_Error_Set() 已迁移至
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
