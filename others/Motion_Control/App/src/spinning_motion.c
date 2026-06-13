/**
 * @file spinning_motion.c
 * @brief 小陀螺动作组：加速(spinning_up) 与 统一退出(spinning_exit)。
 *        加速段闭环 d_yaw 到目标转速；退出段随转速降低平滑引入角度归位，均受功率门控。
 *        小陀螺时叠加在 Standing 上的 phi0 归中 PID 也在此定义（L/R_Spin_Phi0_PID）。
 */

#include "chassis_behavior_tree.h"
#include "user_pid.h"
#include "Motor_Drv.h"
#include "Gimbal.h"
#include "User_State.h"
#include "State.h"
#include "arm_math.h"
#include "USER_CAN.h"
#include "VMC.h"
#include "observe_task.h"
#include "Leg_Control.h"
#include "Self_Righting.h"
#include "Board2Board.h"
#include "Slope.h"
#include "Wheel_Leg_about.h"
#include "controller.h"
#include "remoter.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "fdcan.h"
#include <math.h>
#include <stdint.h>
#include "imu_temp_ctrl.h"
#include "Angle_about.h"
#include "PowerCtrl.h"
#include "Gas_Spring.h"
#include "buzzer.h"
#include "Wheel_End_Velocity.h"

user_pid_t spinning_pid;//小陀螺PID
user_pid_t spinning_speed_pid;//小陀螺减速PID

user_pid_t L_Spin_Phi0_PID, R_Spin_Phi0_PID;
float target_spin_phi0 = PI / 2.0f;
float spinning_target_d_yaw_cmd = 0.0f;
float spinning_d_yaw_feedback = 0.0f;
float alpha_spinning_target_d_yaw = 0.02f;
float alpha_spinning_d_yaw = 0.2f;
float alpha_spinning_down_target_d_yaw = 0.08f;
float alpha_spinning_stop_target_d_yaw = 0.05f;

uint8_t spinning_flag = 0; // 1：小陀螺运行中 0：小陀螺停止
uint8_t spinning_usable = 1; // 小陀螺是否可用，0为不可用，1为可用

float target_spinning_d_yaw = 8.0f; // 目标小陀螺yaw速度，单位为弧度每秒
float centrifugal_comp_gain = 0.8f;  // spin离心补偿系数
// 小陀螺时允许触发平移的yaw_angle_PI误差窗口(rad)：
// |yaw_angle_PI| <= 此值时，速度倍率从0线性升至+1（正向）；
// |yaw_angle_PI - ±PI| <= 此值时，倍率从0线性降至-1（反向）；
// 其余角度倍率为0。建议0.2~0.5rad
float spin_speed_tol_angle = 1.0f;
// 小陀螺平移方向偏置(rad)：补偿"拨杆向前-实际方向"的安装/解算偏差。
// 正负与 yaw_angle_PI 同号系：调一调正负看车的实际响应方向。建议先±0.1rad尝试。
float spin_speed_angle_offset = -0.9f;

//小陀螺加速
void spinning_up()
{
    float spinning_setpoint = (g_filtered_power > power_limit)
                            ? target_spinning_d_yaw * g_power_obs_lambda
                            : target_spinning_d_yaw;
    spinning_target_d_yaw_cmd = alpha_spinning_target_d_yaw * spinning_setpoint
                              + (1.0f - alpha_spinning_target_d_yaw) * spinning_target_d_yaw_cmd;
    spinning_d_yaw_feedback = alpha_spinning_d_yaw * d_yaw
                            + (1.0f - alpha_spinning_d_yaw) * spinning_d_yaw_feedback;
    PID_Set_Error(&spinning_pid, spinning_d_yaw_feedback, spinning_target_d_yaw_cmd);
    yaw_error = PID_coculate(&spinning_pid);
    Speed_Error_Set();
}

//统一小陀螺退出：转速降低过程中平滑引入角度修正，同时受功率门控
void spinning_exit()
{
    // 角度修正量：P-only，Kp=-6，即 -6 * yaw_angle_PI
    PID_Set_Error(&spinning_speed_pid, yaw_angle_PI, 0);
    float angle_correction = PID_coculate(&spinning_speed_pid);

    // 混合权重：转速越高 weight 越小，专注减速；转速越低 weight 越大，角度归位
    float speed_ratio = fabsf(d_yaw) / target_spinning_d_yaw;
    if (speed_ratio > 1.0f) speed_ratio = 1.0f;
    float angle_weight = 1.0f - speed_ratio;

    float blended_target = 1.0 * angle_correction;

    // 功率门控：超功率时缩放目标，与 spinning_up 一致
    if (g_filtered_power > power_limit)
    {
        blended_target *= g_power_obs_lambda;
    }

    PID_Set_Error(&spinning_pid, d_yaw, blended_target);
    yaw_error = PID_coculate(&spinning_pid);
    Speed_Error_Set();
}
