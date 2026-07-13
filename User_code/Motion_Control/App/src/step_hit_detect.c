/**
 * @file step_hit_detect.c
 * @brief 磕台阶检测：长腿姿态下两腿同时出现高腿力矩 + 高腿长，连续命中即触发上台阶
 *        (upstairs_flag=1)。含切到长腿后的解禁延时与离地冷却联动。
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

static uint8_t step_hit_count = 0;
static float prev_L_Ground_F0 = 0.0f;
static float prev_R_Ground_F0 = 0.0f;

void Step_Hit_Detect(void)
{
    const float leg_torque_threshold = 6.0f;        // 经验值，可再微调
    const float leg_length_threshold = LEG_MAX_LENTH - 0.03f; // 高腿长门槛
    const int step_hit_count_target = 2;
    const int step_hit_cooldown_target = motor_HZ; // 1s 冷却
    const int long_leg_arm_delay_target = motor_HZ / 4; // 切到长腿后 250ms 才允许检测

    // 切换到长腿(Target_Leg_State==1)的上升沿后，延迟 250ms 才解禁台阶检测
    // 防止刚抬腿瞬间因伸腿动作本身产生的高力矩+高腿长被误判成磕台阶
    static uint8_t prev_target_leg_state = 0;
    static int     long_leg_arm_delay = 0;
    if (Foot_Chassis.Target_Leg_State == 1 && prev_target_leg_state != 1)
    {
        long_leg_arm_delay = long_leg_arm_delay_target;
    }
    if (Foot_Chassis.Target_Leg_State != 1)
    {
        long_leg_arm_delay = 0;
    }
    else if (long_leg_arm_delay > 0)
    {
        long_leg_arm_delay--;
    }
    prev_target_leg_state = Foot_Chassis.Target_Leg_State;

    float left_leg_torque_cmd = fabsf(VMC_L.T_actual);
    float right_leg_torque_cmd = fabsf(VMC_R.T_actual);

    int left_leg_torque_high = (left_leg_torque_cmd > leg_torque_threshold);
    int right_leg_torque_high = (right_leg_torque_cmd > leg_torque_threshold);

    int left_leg_high = (VMC_L.L0 > leg_length_threshold);
    int right_leg_high = (VMC_R.L0 > leg_length_threshold);

    int left_step_hit = left_leg_torque_high && left_leg_high;
    int right_step_hit = right_leg_torque_high && right_leg_high;

    if (step_hit_cooldown > 0)
    {
        step_hit_cooldown--;
    }

    if (step_hit_cooldown == 0 && long_leg_arm_delay == 0 && (left_step_hit && right_step_hit) && Foot_Chassis.Target_Leg_State == 1 && start_mode == 1 && upstares_mode == 0)
    {
        if (step_hit_count < step_hit_count_target * 2)
            step_hit_count++;
    }
    else if (step_hit_count > 0)
    {
        step_hit_count--;
    }

    if (step_hit_count >= step_hit_count_target)
    {
        upstairs_flag = 1;
        step_hit_count = 0;
        step_hit_cooldown = step_hit_cooldown_target;
    }

    prev_L_Ground_F0 = L_Ground_F0;
    prev_R_Ground_F0 = R_Ground_F0;
}
