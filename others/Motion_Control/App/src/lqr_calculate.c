/**
 * @file lqr_calculate.c
 * @brief LQR 力矩计算：由 11 维状态 × 4×11 增益矩阵算出左右轮力矩与左右模拟腿力矩。
 *        并提供 LQR_Update_K()：按 100Hz 节流刷新 K 矩阵（K 值拟合较耗时）。
 *
 * LQR 状态（11维）：
 *   [ body_distance_error, speed_error, yaw_error, d_yaw,
 *     VMC_L.b_phi0, VMC_L.d_b_phi0, VMC_R.b_phi0, VMC_R.d_b_phi0,
 *     pitch_trans[0], d_pitch, int_pitch ]
 *   其中 int_pitch = ∫(pitch_trans[0] - pitch_offset_eff) dt，离地时冻结。
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

float Leg_L_T; //模拟腿力矩
float Leg_R_T;

float PITCH_OFFSET = -0.10;
// 小陀螺时叠加的pitch偏置：用于抵消起转/退出时的反作用俯仰
// 占位值，需现场调试：先试正值再试负值，找到能抵消低头方向的符号后再加大
float PITCH_OFFSET_SPIN  = 0.05f;       //单位 rad，叠加在 PITCH_OFFSET 上
float pitch_offset_ramp  = 0.0003f;    //偏置斜坡速率(rad/调用)，进入/退出共用
float pitch_offset_eff   = -0.0f;     //运行时生效的pitch偏置，初值与PITCH_OFFSET一致避免首拍跳变

float LQR_K[4][11] = {
    -1.9045,  -2.9573,  -6.2325,  -1.2595,  -13.805,  -0.96974,  -3.4901,  -0.45392,  -10.489,  -1.5631,  -0.72426,
     -1.9045,  -2.9573,  6.2325,  1.2595,  -3.4901,  -0.45392,  -13.805,  -0.96974,  -10.489,  -1.5631,  -0.72426,
     4.5806,  6.7741,  -26.254,  -6.699,  29.403,  2.0429,  -5.5104,  0.023442,  -38.831,  -2.861,  -2.4091,
     4.5806,  6.7741,  26.254,  6.699,  -5.5104,  0.023442,  29.403,  2.0429,  -38.831,  -2.861,  -2.4091
};

float K_Fit_Coefficients[44][6] = {
-1.1679,  -10.314,  4.008,  11.281,  0.78322,  -3.874,
     -1.9699,  -14.57,  6.3729,  15.752,  0.0048656,  -5.9403,
     -14.511,  44.489,  -19.441,  -55.154,  12.508,  22.624,
     -5.3486,  19.416,  -9.3243,  -21.934,  6.1317,  10.364,
     -7.6234,  -54.09,  13.754,  37.64,  2.3579,  -16.708,
     -0.20243,  -5.9723,  1.47,  -1.8481,  1.7935,  -2.1121,
     -2.3486,  9.1302,  -27.278,  -11.779,  24.979,  20.45,
     -0.037669,  -0.41102,  -2.9598,  0.65491,  0.89346,  0.95794,
     -17.418,  33.476,  20.415,  -20.666,  -19.565,  -14.415,
     -2.4401,  3.063,  3.9738,  -0.49889,  -3.5197,  -3.1287,
     -3.6968,  6.6018,  4.7477,  -3.743,  -4.3119,  -3.4981,
     -1.1679,  4.008,  -10.314,  -3.874,  0.78322,  11.281,
     -1.9699,  6.3729,  -14.57,  -5.9403,  0.0048656,  15.752,
     14.511,  19.441,  -44.489,  -22.624,  -12.508,  55.154,
     5.3486,  9.3243,  -19.416,  -10.364,  -6.1317,  21.934,
     -2.3486,  -27.278,  9.1302,  20.45,  24.979,  -11.779,
     -0.037669,  -2.9598,  -0.41102,  0.95794,  0.89346,  0.65491,
     -7.6234,  13.754,  -54.09,  -16.708,  2.3579,  37.64,
     -0.20243,  1.47,  -5.9723,  -2.1121,  1.7935,  -1.8481,
     -17.418,  20.415,  33.476,  -14.415,  -19.565,  -20.666,
     -2.4401,  3.9738,  3.063,  -3.1287,  -3.5197,  -0.49889,
     -3.6968,  4.7477,  6.6018,  -3.4981,  -4.3119,  -3.743,
     7.2618,  -5.5115,  -16.474,  -4.0406,  17.823,  8.4908,
     10.84,  -7.7242,  -26.297,  -6.1744,  29.415,  12.717,
     -21.895,  -32.368,  -43.636,  69.179,  -39.358,  52.983,
     -7.4357,  -20.584,  -18.019,  36.213,  -22.307,  20.27,
     32.083,  -18.073,  7.9492,  16.616,  74.59,  -25.073,
     1.3551,  5.8575,  -1.1707,  -1.0142,  7.0351,  -1.0431,
     4.4928,  -43.681,  -44.802,  65.793,  -60.839,  20.148,
     0.5853,  -3.5303,  0.16262,  5.0353,  -6.2887,  -7.5213,
     -25.96,  -160.89,  47.756,  179.74,  18.163,  -51.539,
     -1.2565,  -17.859,  3.8032,  17.152,  5.1681,  -4.2734,
     -4.7131,  -33.094,  8.1191,  36.139,  5.4029,  -9.1425,
     7.2618,  -16.474,  -5.5115,  8.4908,  17.823,  -4.0406,
     10.84,  -26.297,  -7.7242,  12.717,  29.415,  -6.1744,
     21.895,  43.636,  32.368,  -52.983,  39.358,  -69.179,
     7.4357,  18.019,  20.584,  -20.27,  22.307,  -36.213,
     4.4928,  -44.802,  -43.681,  20.148,  -60.839,  65.793,
     0.5853,  0.16262,  -3.5303,  -7.5213,  -6.2887,  5.0353,
     32.083,  7.9492,  -18.073,  -25.073,  74.59,  16.616,
     1.3551,  -1.1707,  5.8575,  -1.0431,  7.0351,  -1.0142,
     -25.96,  47.756,  -160.89,  -51.539,  18.163,  179.74,
     -1.2565,  3.8032,  -17.859,  -4.2734,  5.1681,  17.152,
     -4.7131,  8.1191,  -33.094,  -9.1425,  5.4029,  36.139,
};

float lqr_body_distance_error ;
float lqr_speed_error;
float lqr_yaw_error;
float lqr_d_yaw;
float int_pitch = 0.0f;          // 第11维状态：俯仰角积分 ∫(pitch_trans[0]-pitch_offset_eff)dt，离地冻结
float int_pitch_limit = 5.0f;    // 俯仰积分抗饱和限幅(rad·s)，现场可调
void LQR_calculate()
{
	lqr_body_distance_error = body_distance_error;
	lqr_speed_error = speed_error;
	lqr_yaw_error = yaw_error;
	lqr_d_yaw = d_yaw;
    float leg_yaw_error = lqr_yaw_error;
    float leg_d_yaw = lqr_d_yaw;
    if(spinning_flag == 1)
    {
        leg_yaw_error = 0.0f;
        leg_d_yaw = 0.0f;
        lqr_body_distance_error = 0.0f;   // 小陀螺时禁止距离闭环，避免误差累积干扰平移
    }
    if(upstairs_flag == 1)
    {
        lqr_body_distance_error = 0.0f;   // 上台阶动作组触发后禁止距离闭环
    }
    // float leg_b_phi0_offset = (spinning_flag == 1) ? 0.0f : b_phi0_offset;
    float leg_b_phi0_offset = (spinning_flag == 1) ? 0.0f : speed_error * 0.07f; 

    // 第11维状态：俯仰角积分。任一腿离地时冻结(不累加)，其余(含小陀螺)正常积分，带抗饱和限幅。
    if (L_off_ground < 10 && R_off_ground < 10)
    {
        int_pitch += (pitch_trans[0] - pitch_offset_eff) * 0.002f;
        if (int_pitch >  int_pitch_limit) int_pitch =  int_pitch_limit;
        if (int_pitch < -int_pitch_limit) int_pitch = -int_pitch_limit;
    }

    //算轮子力矩
    L_DJ3508.Target_Torque =
    + LQR_K[0][0] * lqr_body_distance_error
    + LQR_K[0][1] * (lqr_speed_error)
    + LQR_K[0][2] * (lqr_yaw_error)
    - LQR_K[0][3] * lqr_d_yaw
    - LQR_K[0][4] * (VMC_L.b_phi0 - leg_b_phi0_offset)
    - LQR_K[0][5] * VMC_L.d_b_phi0
    - LQR_K[0][6] * (VMC_R.b_phi0 - leg_b_phi0_offset)
    - LQR_K[0][7] * VMC_R.d_b_phi0
    + LQR_K[0][8] * (pitch_trans[0] - pitch_offset_eff)
    + LQR_K[0][9] * d_pitch
    + LQR_K[0][10] * int_pitch;

    R_DJ3508.Target_Torque =
    + LQR_K[1][0] * lqr_body_distance_error
    + LQR_K[1][1] * (lqr_speed_error)
    + LQR_K[1][2] * (lqr_yaw_error)
    - LQR_K[1][3] * lqr_d_yaw
    - LQR_K[1][4] * (VMC_L.b_phi0 - leg_b_phi0_offset)
    - LQR_K[1][5] * VMC_L.d_b_phi0
    - LQR_K[1][6] * (VMC_R.b_phi0 - leg_b_phi0_offset)
    - LQR_K[1][7] * VMC_R.d_b_phi0
    + LQR_K[1][8] * (pitch_trans[0] - pitch_offset_eff)
    + LQR_K[1][9] * d_pitch
    + LQR_K[1][10] * int_pitch;

    //算模拟腿力矩
    Leg_L_T =
    + LQR_K[2][0] * lqr_body_distance_error
    + LQR_K[2][1] * (lqr_speed_error)
    + LQR_K[2][2] * (-leg_yaw_error)
    - LQR_K[2][3] * leg_d_yaw
    - LQR_K[2][4] * (VMC_L.b_phi0 - leg_b_phi0_offset)
    - LQR_K[2][5] * VMC_L.d_b_phi0
    - LQR_K[2][6] * (VMC_R.b_phi0 - leg_b_phi0_offset)
    - LQR_K[2][7] * VMC_R.d_b_phi0
    + LQR_K[2][8] * (pitch_trans[0] - pitch_offset_eff)
    + LQR_K[2][9] * d_pitch
    + LQR_K[2][10] * int_pitch;

    Leg_R_T =
    + LQR_K[3][0] * lqr_body_distance_error
    + LQR_K[3][1] * (lqr_speed_error)
    + LQR_K[3][2] * (-leg_yaw_error)
    - LQR_K[3][3] * leg_d_yaw
    - LQR_K[3][4] * (VMC_L.b_phi0 - leg_b_phi0_offset)
    - LQR_K[3][5] * VMC_L.d_b_phi0
    - LQR_K[3][6] * (VMC_R.b_phi0 - leg_b_phi0_offset)
    - LQR_K[3][7] * VMC_R.d_b_phi0
    + LQR_K[3][8] * (pitch_trans[0] - pitch_offset_eff)
    + LQR_K[3][9] * d_pitch
    + LQR_K[3][10] * int_pitch;
}

// 100Hz 刷新 LQR 增益矩阵 K（K 值计算较耗时，故每 5 拍算一次）
static uint8_t i = 0;
void LQR_Update_K()
{
    i++;
    if(i >= 5)
    {
        i = 0;
        LQR_Get_K(LQR_K, K_Fit_Coefficients, VMC_L.L0, VMC_R.L0);
    }
}
