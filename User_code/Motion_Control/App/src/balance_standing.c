/**
 * @file balance_standing.c
 * @brief 站起 / 正常运行动作组 Standing()：
 *        倾覆保护 → INS/VMC/车速解算 → yaw误差(常态/小陀螺) → 距离误差 →
 *        横滚补偿 + 腿长PID + 防劈叉 → LQR_Update_K + 功率门控 + LQR_calculate →
 *        防劈叉力矩合成 → 跳跃状态更新 → Standing 选择并写入 VMC 目标 →
 *        离地检测 + 磕台阶检测 → 上台阶/坐地模式切换。
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
#include "chassis_height_control.h"
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
#include "Tip_Protect.h"
#include "anti_split_control.h"

//站起
void Standing()
{
    // 判断倾覆保护
    if (Tip_Protect_Detect())
    {
        // 执行倾覆保护
        Tip_Protect_Action();
        return;
    }

    // 计算 LQR 使用的速度、位移和 yaw 误差
    Error_Calculate();

    // 横滚补偿和PD单环腿长控制
    Roll_Comp();
    Leg_L0_Control();

    // 100hz算K值，毕竟K值的计算比较耗时
    LQR_Update_K();

	// 更新功率门控缩放系数（不直接限制扭矩，只作用于观测量）。
	PowerCtrl();

    LQR_calculate();

    float L_leg_T_cmd;
    float R_leg_T_cmd;
    AntiSplit_Control(&L_leg_T_cmd, &R_leg_T_cmd);

    // 跳跃状态机只更新状态和跳跃腿长 PID，VMC 目标由 Standing 统一选择。
    uint8_t jump_active = Jump_Motion_Update();
    
// region [rgb(0, 70, 70)] VMC 目标赋值
    if (jump_active)
    {
        VMC_Chassis_Target.L_F0 = 200.0f + (mg / arm_cos_f32(VMC_L.b_phi0)) + Roll_Comp_PID.output;
        VMC_Chassis_Target.R_F0 = 200.0f + (mg / arm_cos_f32(VMC_R.b_phi0)) - Roll_Comp_PID.output;
    }
    else
    {
        VMC_Chassis_Target.L_F0 = L_Leg_L0_PID.output + (mg / arm_cos_f32(VMC_L.b_phi0)) + Roll_Comp_PID.output;
        VMC_Chassis_Target.R_F0 = R_Leg_L0_PID.output + (mg / arm_cos_f32(VMC_R.b_phi0)) - Roll_Comp_PID.output;
    }
    VMC_Chassis_Target.L_T = L_leg_T_cmd;
    VMC_Chassis_Target.R_T = R_leg_T_cmd;
// endregion

    off_ground_detect();
    Step_Hit_Detect();
    // 注：跳跃上锁统一在 jump_motion 锁存退出时处理（离地 / 0.5s 到期），此处不再需要兜底

    if(upstairs_flag == 1)
    {
        start_mode = 2;
        upstairs_flag = 0;    // 已消费触发信号，避免动作完成后残留导致距离闭环被永久压0
    }

    if(sit_mode_enable == 1)
    {
        start_mode = 3;
    }
}
