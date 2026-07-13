/**
 * @file self_righting_retract.c
 * @brief 未站起 + 未上楼收腿：倒地时先跑倒地自起(Self_Righting)，姿态稳定后收腿、
 *        转腿到竖直，两腿到位即切入正常模式(start_mode=1)。
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

//未站起 + 未上楼收腿  函数
void NotStanding_NotStairRetract_for_chassis()
{
    //自起完成蜂鸣音 latch：刚离开自起态时响 100ms 高 si
    static uint8_t sr_was_active = 0;
    static int sr_finish_chime_remain = 0;

    VMC_Coculate();
    Body_Speed_Coculate();//车身速度解算

    //是否姿态稳定在误差20°内的起立态
    if((roll >= 40.0f || roll <= -40.0f || pitch >= 40.0f || pitch <= -40.0f) && first_run == 1)//不稳定且是急停开始第一次运行
    {
        gimbal_follow_flag = 1;//自起期间云台跟随底盘
        //只在进入自起的第一拍锁定方向：|pitch|>90° → 反面倒地 dir=+1；正面倒地 → dir=-1
        //后续 tick 沿用第一拍的 dir，防止自起过程中 pitch 穿越 90° 边界导致方向抖动
        if (sr_was_active == 0)
        {
            g_sr_turn_dir = (pitch > 90.0f || pitch < -90.0f) ? 1 : -1;
        }
        g_tip_recovery_active = 1;//占用蜂鸣器，错误码蜂鸣器必须让位
        Self_Righting_Step();
        sr_was_active = 1;
        return ;
    }

    //倒地自起成功后复位Self_Righting的状态机（stage / sync_from_stuck / VMC输出 / 蜂鸣器 / tick 一次性归零）
    if (sr_was_active)
    {
        Self_Righting_Reset();
        sr_finish_chime_remain = 50;  //50 * 2ms = 100ms
        sr_was_active = 0;
    }
    if (sr_finish_chime_remain > 0)
    {
        g_tip_recovery_active = 1;//完成提示音期间继续独占蜂鸣器
        Buzzer_Tone_Max(1976);
        sr_finish_chime_remain--;
        if (sr_finish_chime_remain == 0)
        {
            Stop_Buzzer();
            g_tip_recovery_active = 0;//完成提示音结束，把蜂鸣器交还给错误码代码
        }
    }
    first_run = 0;//第一次运行完成

    //收腿过程腿长控制
    PID_Set_Error(&L_Leg_L0_POS_PID, VMC_L.L0, 0.12f);//0.19这个值是通过反复试验得来的，目的是让腿在收腿过程中稍微有个前倾，防止完全竖直时不稳定
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

    //腿长判断是否到达目标长度
    if(L_Leg_State == 0 && fabsf(L_Leg_L0_POS_PID.error) <= 0.04)
    {
        L_Ready_Count ++;
    }
    if(L_Leg_State == 0 && L_Ready_Count >= 20)//腿到目标长度
    {
        L_Leg_State = 1;    //收腿完成
        L_Ready_Count = 0;  //归零
        L_stair_sub = STAIR_SUB_TURN_FWD;
        L_sub_dwell = 0;
        leg_turn_stuck_reset(&VMC_L);
    }
    if(R_Leg_State == 0 && fabsf(R_Leg_L0_POS_PID.error) <= 0.04)
    {
        R_Ready_Count ++;
    }
    if(R_Leg_State == 0 && R_Ready_Count >= 20)
    {
        R_Leg_State = 1;
        R_Ready_Count = 0;
        R_stair_sub = STAIR_SUB_TURN_FWD;
        R_sub_dwell = 0;
        leg_turn_stuck_reset(&VMC_R);
    }

    //腿长达标之后，判断腿角度是否到达目标角度（用 helper 返回的 near）
    if(L_Leg_State == 1 && L_near) L_Ready_Count ++;
    else if(L_Leg_State == 1)      L_Ready_Count = 0;
    if(L_Leg_State == 1 && L_Ready_Count >= 20)
    {
        L_Leg_State = 2;
        L_Ready_Count = 0;
    }
    if(R_Leg_State == 1 && R_near) R_Ready_Count ++;
    else if(R_Leg_State == 1)      R_Ready_Count = 0;
    if(R_Leg_State == 1 && R_Ready_Count >= 20)
    {
        R_Leg_State = 2;
        R_Ready_Count = 0;
    }

    if(R_Leg_State == 2 && L_Leg_State == 2)
    {
        start_mode = 1; // 收腿完成，进入正常模式
        //归零
        R_Leg_State = 0;
        L_Leg_State = 0;
        body_distance = 0;
        target_body_distance = 0.0;
    }

    //映射到电机力矩
    VMC_Set_F0_T(&VMC_L, L_Leg_L0_SPD_PID.output, L_T);
    VMC_Set_F0_T(&VMC_R, R_Leg_L0_SPD_PID.output, R_T);
    L_DJ3508.Target_Torque = 0;
    R_DJ3508.Target_Torque = 0;

}
