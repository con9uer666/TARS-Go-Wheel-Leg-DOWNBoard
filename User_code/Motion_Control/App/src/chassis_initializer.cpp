/**
 * @file chassis_initializer.cpp
 * @brief 底盘初始化：电机参数、VMC、PID 初始化，以及 pitch 前后帧计算。
 */

#include "arm_math.h"

extern "C"
{
#include "chassis_behavior_tree.h"
#include "user_pid.h"
#include "Motor_Drv.h"
#include "Gimbal.h"
#include "User_State.h"
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
}

#include "chassis_initializer.hpp"

/** @brief 初始化底盘关节、Yaw、拨弹电机参数并启用气弹簧补偿。 */
void chassis::ChassisInitializer::InitializeMotors()
{
    DM_Joint_Motor_Init(&L_DM8009[0], 40.0f, 3.14159265f, 45.0f, 0x01);
    DM_Joint_Motor_Init(&L_DM8009[1], 40.0f, 3.14159265f, 45.0f, 0x02);

    DM_Joint_Motor_Init(&R_DM8009[0], 40.0f, 3.14159265f, 45.0f, 0x01);
    DM_Joint_Motor_Init(&R_DM8009[1], 40.0f, 3.14159265f, 45.0f, 0x02);

    DM_Joint_Motor_Init(&Yaw_DM4310, 10.0f, 3.14159265f, 30.0f, 0x10);
    DM_Joint_Motor_Init(&Shooter_DM2325, 10.0f, 3.14159265f, 200.0f, 0x11);

    // 功率控制模块初始化（仅初始化参数，不改变现有控制流）。
    // PowerCtralInit(&whell_power);

    // gas_spring_enable = 1;  // 氮气弹簧已拆除，禁用补偿
}

/** @brief 初始化左右五连杆 VMC 的连杆长度和机构镜像方向。 */
void chassis::ChassisInitializer::InitializeVmc()
{
    VMC_Init(&VMC_L, 0.210f, 0.250f, 0.250f, 0.210f, 0.0f, 1);
    VMC_Init(&VMC_R, 0.210f, 0.250f, 0.250f, 0.210f, 0.0f, 0);
}

/** @brief 使用迁移前完全相同的增益、限幅和积分参数初始化全部共享 PID。 */
void chassis::ChassisInitializer::InitializePids()
{
    PID_INIT(&L_Leg_L0_PID, 200, 0, 1000, 200, 0, 0, 0, 0);
    PID_INIT(&R_Leg_L0_PID, 200, 0, 1000, 200, 0, 0, 0, 0);
    PID_INIT(&Leg_AntiSplit_PID, 200, 0, 10, 150, 0, 0, 0, 0);   // Kp/Kd 为占位，每周期由 AntiSplitController 按腿长覆盖
    PID_INIT(&L_Spin_Phi0_PID, 80, 0, 8, 40, 0, 0, 0, 0);
    PID_INIT(&R_Spin_Phi0_PID, 80, 0, 8, 40, 0, 0, 0, 0);
    PID_INIT(&Roll_Comp_PID, 0, 0.0, 0, 150, 80, 0, 10000, 0);
    PID_INIT(&L_Leg_Middle_PID, 15, 0.1, 0.1, 5.0, 4.0, 0, 0, 0);
    PID_INIT(&R_Leg_Middle_PID, 15, 0.1, 0.1, 5.0, 4.0, 0, 0, 0);
    PID_INIT(&L_Leg_dphi0_PID, 3, 0.1, 1, 150, 150, 0, 2000, 0);
    PID_INIT(&R_Leg_dphi0_PID, 3, 0.1, 1, 150,150, 0, 2000, 0);

    PID_INIT(&L_Leg_L0_POS_PID, 15, 0.001, 0.1, 3.0, 2.0, 0, 200, 0);
    PID_INIT(&R_Leg_L0_POS_PID, 15, 0.001, 0.1, 3.0, 2.0, 0, 200, 0);
    PID_INIT(&L_Leg_L0_SPD_PID, 50, 3, 400, 80, 80, 0, 2000, 0);
    PID_INIT(&R_Leg_L0_SPD_PID, 50, 3, 400, 80, 80, 0, 2000, 0);

    //小陀螺pid
    PID_INIT(&spinning_pid, 0.0005, 0.001f, 0.001, 6.0f, 6.0f, 0.005f, 20.0f, 0);

    //云台pid
    PID_INIT(&gimbal_pitch_pid, 10, 0.002, 100, 150, 80, 0, 10000, 0);
    PID_INIT(&spinning_speed_pid, -6, 0, -500, 6, 0, 0, 0, 0);

    // PID_INIT(&gimbal_follow_error_pid, 3, 0.002, 100, 150, 80, 10000, 0);

    // 目标腿长初值与 LEG_MIN_LENTH 同步
    target_Leg_L0   = LEG_MIN_LENTH;
    target_L_Leg_L0 = LEG_MIN_LENTH;
    target_R_Leg_L0 = LEG_MIN_LENTH;
}

/** @brief 推进俯仰角前后帧历史，并把 IMU 度数转换为弧度。 */
void chassis::ChassisInitializer::UpdatePitchHistory()
{
    pitch_trans[1] = pitch_trans[0];
    pitch_trans[0] = (pitch/180.0f) * PI;
}

/**
 * @brief 未迁移 C 状态解算代码使用的俯仰历史兼容入口。
 *
 * Wheel_Leg_about.c 的 INS_Coculate() 仍调用该符号；内部立即委托 C++ 初始化器。
 */
extern "C" void task_Pitch_Coculate(void)
{
    chassis::ChassisInitializer::UpdatePitchHistory();
}
