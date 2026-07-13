/**
 * @file off_ground_detect.c
 * @brief 离地检测：滤波地面支持力 → 判定单腿/双腿离地 → 离地腿归中、轮子脱力。
 *        离地期间持续刷新 step_hit_cooldown，着地后 1s 内禁止上台阶检测。
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

float L_Ground_F0, R_Ground_F0; //地面支持力
int L_off_ground = 0;   //必须是int类型，因为要减去计数器，不能无符号
int R_off_ground = 0;   //必须是int类型，因为要减去计数器，不能无符号

// 提到文件作用域：off_ground_detect 也会写，用作离地保护期；step_hit_detect 共用
int step_hit_cooldown = 0;

void off_ground_detect()
{
    float alpha_G_F0 = 0.1;

    L_Ground_F0 = alpha_G_F0 * VMC_Get_Ground_F0(&VMC_L) + (1 - alpha_G_F0) * L_Ground_F0;
    R_Ground_F0 = alpha_G_F0 * VMC_Get_Ground_F0(&VMC_R) + (1 - alpha_G_F0) * R_Ground_F0;

    //离地检测滤波
    if(L_Ground_F0 <= 50.0f)
    L_off_ground ++;
    else if (L_Ground_F0 >= 10.0f)
    L_off_ground --;
    if(L_off_ground >= 20)
    L_off_ground = 20;
    if(L_off_ground <= 0)
    L_off_ground = 0;

    if(R_Ground_F0 <= 50.0f)
    R_off_ground ++;
    else if (R_Ground_F0 >= 10.0f)
    R_off_ground --;
    if(R_off_ground >= 20)
    R_off_ground = 20;
    if(R_off_ground <= 0)
    R_off_ground = 0;

    //!这段是先算一遍不离地的情况的数，再检测是否离地，如果离地，就再算一次覆盖掉
    if(L_off_ground >= 10 && R_off_ground >= 10)//两腿同时离地：锁定 Target_Leg_State 为短腿，直到上位机重新发短腿(0)才解锁
    {
        leg_state_locked_short = 1;
    }
    if(L_off_ground >= 10)//正常行驶过程离地
    {
        //离地后腿归中，轮子脱力vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F%E7%A6%BB%E5%9C%B0%E5%90%8E%E8%85%BF%E5%BD%92%E4%B8%AD%EF%BC%8C%E8%BD%AE%E5%AD%90%E8%84%B1%E5%8A%9B
        Leg_L_T =
        - LQR_K[2][4] * (VMC_L.b_phi0 - b_phi0_offset)
        - LQR_K[2][5] * VMC_L.d_b_phi0 ;
        Leg_L_T *= 0.7; //收腿力度参数
        L_DJ3508.Target_Torque = 0;//离地轮子脱力
        //正常行驶过程离地VMC解算
        VMC_Set_F0_T(&VMC_L, L_Leg_L0_PID.output * 0.5, Leg_L_T);//VMC解算
        //离地距离相关量归零
        body_distance = 0;
        target_body_distance = 0.0;
    }
    if(R_off_ground >= 10)
    {
        Leg_R_T =
        - LQR_K[3][6] * (VMC_R.b_phi0 - b_phi0_offset)
        - LQR_K[3][7] * VMC_R.d_b_phi0;
        Leg_R_T *= 0.7;
        R_DJ3508.Target_Torque = 0;
        VMC_Set_F0_T(&VMC_R, R_Leg_L0_PID.output * 0.5, -Leg_R_T);
        body_distance = 0;
        target_body_distance = 0.0;
    }

    // 离地保护：任一腿离地期间持续刷新冷却，着地后 1s 内禁止上台阶检测
    if(L_off_ground >= 10 || R_off_ground >= 10)
    {
        step_hit_cooldown = motor_HZ;   // 1s
    }
}
