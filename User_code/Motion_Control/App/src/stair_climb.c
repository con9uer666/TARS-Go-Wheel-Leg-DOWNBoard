/**
 * @file stair_climb.c
 * @brief 上台阶动作组：
 *        Upstair_NotStairRetract() —— 磕台阶后伸长腿、外摆腿角顶上台阶，两腿到位置 upstares_mode=1。
 *        StairRetract()           —— 上台阶后收腿起立（含转角卡住反向绕长路），完成回正常模式。
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

void Upstair_NotStairRetract()
{

    // 磕台阶过程中双环腿长控制
    PID_Set_Error(&L_Leg_L0_POS_PID, VMC_L.L0, LEG_MAX_LENTH);   //TODO: 写一个最大腿长的宏定义
    PID_Set_Error(&R_Leg_L0_POS_PID, VMC_R.L0, LEG_MAX_LENTH);
    PID_coculate(&L_Leg_L0_POS_PID);
    PID_coculate(&R_Leg_L0_POS_PID);

    PID_Set_Error(&L_Leg_L0_SPD_PID, VMC_L.d_L0, L_Leg_L0_POS_PID.output);
    PID_Set_Error(&R_Leg_L0_SPD_PID, VMC_R.d_L0, R_Leg_L0_POS_PID.output);
    PID_coculate(&L_Leg_L0_SPD_PID);
    PID_coculate(&R_Leg_L0_SPD_PID);

    //磕台阶过程中双环腿角度控制
    PID_Set_AngleError(&L_Leg_Middle_PID, VMC_L.phi0, PI/2 + 1.5f);
    PID_coculate(&L_Leg_Middle_PID);
    PID_Set_Error(&L_Leg_dphi0_PID, VMC_L.d_phi0, L_Leg_Middle_PID.output);
    PID_coculate(&L_Leg_dphi0_PID);

    PID_Set_AngleError(&R_Leg_Middle_PID, VMC_R.phi0, PI/2 - 1.5f);
    PID_coculate(&R_Leg_Middle_PID);
    PID_Set_Error(&R_Leg_dphi0_PID, -VMC_R.d_phi0, -R_Leg_Middle_PID.output);
    PID_coculate(&R_Leg_dphi0_PID);

    //上台阶过程中VMC解算vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F%E4%B8%8A%E5%8F%B0%E9%98%B6%E8%BF%87%E7%A8%8B%E4%B8%ADVMC%E8%A7%A3%E7%AE%97
    VMC_Chassis_Target.L_F0 = L_Leg_L0_SPD_PID.output;
    VMC_Chassis_Target.L_T = L_Leg_dphi0_PID.output;
    VMC_Chassis_Target.R_F0 = R_Leg_L0_SPD_PID.output;
    VMC_Chassis_Target.R_T = -R_Leg_dphi0_PID.output;

    //上台阶收腿过程中判断腿长和腿角度是否都到位了
    if(L_Leg_State == 0 && fabsf(L_Leg_L0_POS_PID.error) <= 0.15 && fabsf(L_Leg_Middle_PID.error) <= 0.15)
    {
        L_Ready_Count ++;
    }
    if(L_Leg_State == 0 && L_Ready_Count >= 60)
    {
        L_Leg_State = 2;
        L_Ready_Count = 0;
    }
    if(R_Leg_State == 0 && fabsf(R_Leg_L0_POS_PID.error) <= 0.15 && fabsf(R_Leg_Middle_PID.error) <= 0.15)
    {
        R_Ready_Count ++;
    }
    if(R_Leg_State == 0 && R_Ready_Count >= 60)
    {
        R_Leg_State = 2;
        R_Ready_Count = 0;
    }
    if(R_Leg_State == 2 && L_Leg_State == 2)
    {
        upstares_mode = 1;
        R_Leg_State = 0;
        L_Leg_State = 0;
        //初始化 StairRetract 的子状态机，避免 Self_Righting 或上一次残留计数
        L_stair_sub = STAIR_SUB_TURN_FWD;
        R_stair_sub = STAIR_SUB_TURN_FWD;
        L_sub_dwell = 0;
        R_sub_dwell = 0;
        leg_turn_stuck_reset(&VMC_L);
        leg_turn_stuck_reset(&VMC_R);
    }
}

void StairRetract()
{

    //收腿起立的腿长双环控制（State=0/1 都跑）
    PID_Set_Error(&L_Leg_L0_POS_PID, VMC_L.L0, 0.12f);
    PID_Set_Error(&R_Leg_L0_POS_PID, VMC_R.L0, 0.12f);
    PID_coculate(&L_Leg_L0_POS_PID);
    PID_coculate(&R_Leg_L0_POS_PID);

    PID_Set_Error(&L_Leg_L0_SPD_PID, VMC_L.d_L0, L_Leg_L0_POS_PID.output);
    PID_Set_Error(&R_Leg_L0_SPD_PID, VMC_R.d_L0, R_Leg_L0_POS_PID.output);
    PID_coculate(&L_Leg_L0_SPD_PID);
    PID_coculate(&R_Leg_L0_SPD_PID);

    //腿角度控制（State>=1 启用，含卡住反向绕长路）
    float L_T = 0.0f, R_T = 0.0f;
    int L_near = 0, R_near = 0;
    if(L_Leg_State >= 1)
    {
        L_near = turn_ctrl_with_stuck_flip(
            &VMC_L, 0, PI/2.0f - 0.1f,
            &L_Leg_Middle_PID, &L_Leg_dphi0_PID,
            &L_stair_sub, &L_sub_dwell,
            &L_rev_dir, &L_rev_long_remain, &L_rev_traveled, &L_T);
    }
    if(R_Leg_State >= 1)
    {
        R_near = turn_ctrl_with_stuck_flip(
            &VMC_R, 1, PI/2.0f + 0.1f,
            &R_Leg_Middle_PID, &R_Leg_dphi0_PID,
            &R_stair_sub, &R_sub_dwell,
            &R_rev_dir, &R_rev_long_remain, &R_rev_traveled, &R_T);
    }

    //映射到电机力矩
    VMC_Chassis_Target.L_F0 = L_Leg_L0_SPD_PID.output;
    VMC_Chassis_Target.L_T = L_T;
    VMC_Chassis_Target.R_F0 = R_Leg_L0_SPD_PID.output;
    VMC_Chassis_Target.R_T = R_T;

    //轮力矩：运行中 0.5，State=2 完成后 0
    VMC_Chassis_Target.L_Wheel_Torque = 0.0f;
    VMC_Chassis_Target.R_Wheel_Torque = 0.0f;

    //State=0 → 1（腿长到位后进入转角阶段）
    if(L_Leg_State == 0 && fabsf(L_Leg_L0_POS_PID.error) <= 0.05f) L_Ready_Count ++;
    else if(L_Leg_State == 0) L_Ready_Count = 0;
    if(L_Leg_State == 0 && L_Ready_Count >= 50)
    {
        L_Leg_State = 1;
        L_Ready_Count = 0;
        L_stair_sub = STAIR_SUB_TURN_FWD;
        L_sub_dwell = 0;
        leg_turn_stuck_reset(&VMC_L);
    }
    if(R_Leg_State == 0 && fabsf(R_Leg_L0_POS_PID.error) <= 0.05f) R_Ready_Count ++;
    else if(R_Leg_State == 0) R_Ready_Count = 0;
    if(R_Leg_State == 0 && R_Ready_Count >= 50)
    {
        R_Leg_State = 1;
        R_Ready_Count = 0;
        R_stair_sub = STAIR_SUB_TURN_FWD;
        R_sub_dwell = 0;
        leg_turn_stuck_reset(&VMC_R);
    }

    //State=1 → 2（角度到位，由 helper 返回 near 判定）
    if(L_Leg_State == 1 && L_near) L_Ready_Count ++;
    else if(L_Leg_State == 1)      L_Ready_Count = 0;
    if(L_Leg_State == 1 && L_Ready_Count >= 50)
    {
        L_Leg_State = 2;
        L_Ready_Count = 0;
    }
    if(R_Leg_State == 1 && R_near) R_Ready_Count ++;
    else if(R_Leg_State == 1)      R_Ready_Count = 0;
    if(R_Leg_State == 1 && R_Ready_Count >= 50)
    {
        R_Leg_State = 2;
        R_Ready_Count = 0;
    }

    //两腿到位后固定延时再切Standing，跳过NotStanding冗余状态迁移
    static int stair_dwell_cnt = 0;
    if (R_Leg_State == 2 && L_Leg_State == 2)
    {
        stair_dwell_cnt++;
        if (stair_dwell_cnt >= 150) // 300ms
        {
            stair_dwell_cnt = 0;
            upstares_mode = 0;
            start_mode = 1;
            R_Leg_State = 0;
            L_Leg_State = 0;
            leg_state = 0;
            target_Leg_L0 = LEG_MIN_LENTH;
            body_distance = 0;
            target_body_distance = 0.0f;
        }
    }
}
