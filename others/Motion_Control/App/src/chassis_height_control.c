/**
 * @file chassis_height_control.c
 * @brief 机身高度控制：横滚补偿 + 腿长 PD 控制
 */

#include "chassis_height_control.h"
#include "imu_temp_ctrl.h"
#include "user_pid.h"
#include "chassis_behavior_tree.h"
#include "remoter.h"
#include "VMC.h"
#include "observe_task.h"
#include "Motor_Drv.h"
#include "Maths_about.h"
#include "body_speed_state.h"
#include <math.h>

/**
 * @brief 横滚角补偿控制。
 *
 * 通过 PID 计算横滚补偿量以保持车身水平。
 * 当速度误差 (speed_error) 较大时，不再更新目标横滚角
 * （即保持当前目标值以避免剧烈姿态变化）。
 *
 * @note 目标横滚角来源于遥控器 CH1 通道映射（含低通滤波），
 *       并叠加 2° 固定偏置。
 */
void Roll_Comp()
{
    if (speed_error <= 0.3f && speed_error >= -0.3f)
        target_roll = alpha_target_roll * (-((SBUS_CH.CH1 - 992.0f) / 800.0f) * 12.0f)
                    + (1 - alpha_target_roll) * target_roll;
    else
        target_roll = alpha_target_roll * target_roll + (1 - alpha_target_roll) * target_roll;

    PID_Set_Error(&Roll_Comp_PID, roll, target_roll + 2);
    PID_coculate(&Roll_Comp_PID);
}

/** @brief 伸腿（短→长）的斜坡速率，每次 Leg_L0_Control() 调用最大增量，单位 m */
float ramp_target_L0_up   = 0.00085f;
/** @brief 缩腿（长→短）的斜坡速率，每次 Leg_L0_Control() 调用最大增量，单位 m */
float ramp_target_L0_down = 0.0010f;

/**
 * @brief 带双速率斜坡的腿长 PD 控制。
 *
 * 根据 Target_Leg_State（0~1 归一化）映射目标腿长范围 [LEG_MIN_LENTH, LEG_MAX_LENTH]，
 * 通过不对称斜坡（伸腿慢、缩腿快）平滑过渡，最后对左/右腿分别施加 PID 控制。
 *
 * @note 伸腿/缩腿速率由 ramp_target_L0_up 和 ramp_target_L0_down 控制，
 *       单位 m/次调用，需根据控制周期标定。
 */
void Leg_L0_Control()
{
    if (leg_state_count > 0) {
        leg_state_count--;
    }

    // 双速率斜坡
    float target_L0_input = ((Foot_Chassis.Target_Leg_State / 1.0f) * (LEG_MAX_LENTH - LEG_MIN_LENTH))
                          + LEG_MIN_LENTH;
    // 根据目标腿长选择斜坡速率
    float ramp_L0 = (target_L0_input > target_Leg_L0) ? ramp_target_L0_up : ramp_target_L0_down;
    target_Leg_L0 = RAMP_float(target_L0_input, target_Leg_L0, ramp_L0);

    if (target_Leg_L0 >= LEG_MAX_LENTH)
        target_Leg_L0 = LEG_MAX_LENTH;
    if (target_Leg_L0 <= LEG_MIN_LENTH)
        target_Leg_L0 = LEG_MIN_LENTH;

    target_L_Leg_L0 = target_Leg_L0;
    target_R_Leg_L0 = target_Leg_L0;

    if (target_L_Leg_L0 >= LEG_MAX_LENTH)
        target_L_Leg_L0 = LEG_MAX_LENTH;
    if (target_L_Leg_L0 <= LEG_MIN_LENTH)
        target_L_Leg_L0 = LEG_MIN_LENTH;

    if (target_R_Leg_L0 >= LEG_MAX_LENTH)
        target_R_Leg_L0 = LEG_MAX_LENTH;
    if (target_R_Leg_L0 <= LEG_MIN_LENTH)
        target_R_Leg_L0 = LEG_MIN_LENTH;

    PID_Set_Error(&L_Leg_L0_PID, VMC_L.L0, target_L_Leg_L0);
    PID_Set_Error(&R_Leg_L0_PID, VMC_R.L0, target_R_Leg_L0);

    PID_coculate(&L_Leg_L0_PID);
    PID_coculate(&R_Leg_L0_PID);
}