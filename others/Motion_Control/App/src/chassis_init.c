/**
 * @file chassis_init.c
 * @brief 底盘初始化：电机参数、VMC、PID 初始化，以及 pitch 前后帧计算。
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

//电机初始化参数及结构体
void task_Motor_Init()
{
    DM_Joint_Motor_Init(&L_DM8009[0], 40.0f, 3.14159265f, 45.0f, 0x01);
    DM_Joint_Motor_Init(&L_DM8009[1], 40.0f, 3.14159265f, 45.0f, 0x02);

    DM_Joint_Motor_Init(&R_DM8009[0], 40.0f, 3.14159265f, 45.0f, 0x01);
    DM_Joint_Motor_Init(&R_DM8009[1], 40.0f, 3.14159265f, 45.0f, 0x02);

    DM_Joint_Motor_Init(&Yaw_DM4310, 10.0f, 3.14159265f, 30.0f, 0x10);
    DM_Joint_Motor_Init(&Shooter_DM2325, 10.0f, 3.14159265f, 200.0f, 0x11);

    // 功率控制模块初始化（仅初始化参数，不改变现有控制流）。
    // PowerCtralInit(&whell_power);

    gas_spring_enable = 1;
}

//VMC赋值与初始化结构体
void task_VMC_Init()
{
    VMC_Init(&VMC_L, 0.210f, 0.250f, 0.250f, 0.210f, 0.0f, 1);
    VMC_Init(&VMC_R, 0.210f, 0.250f, 0.250f, 0.210f, 0.0f, 0);
}

//PID赋值与初始化结构体
void task_PID_Init()
{
    PID_INIT(&L_Leg_L0_PID, 2500, 0, 30000, 200, 0, 0, 0, 0);
    PID_INIT(&R_Leg_L0_PID, 2500, 0, 30000, 200, 0, 0, 0, 0);
    PID_INIT(&Leg_AntiSplit_PID, 300, 0, 10, 150, 0, 0, 0, 0);   //Kp/Kd为占位，每周期由 AntiSplit_Get_K 覆盖
    PID_INIT(&L_Spin_Phi0_PID, 80, 0, 8, 40, 0, 0, 0, 0);
    PID_INIT(&R_Spin_Phi0_PID, 80, 0, 8, 40, 0, 0, 0, 0);
    PID_INIT(&Roll_Comp_PID, 20, 0.002, 100, 150, 80, 0, 10000, 0);
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

//机身pitch计算，记录前一帧的pitch值，单位为弧度，-PI到PI之间
void task_Pitch_Coculate()
{
    pitch_trans[1] = pitch_trans[0];
    pitch_trans[0] = (pitch/180.0f) * PI;
}
