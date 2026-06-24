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
-1.3141,  -9.9058,  4.2222,  11.006,  0.35742,  -3.917,
     -2.1805,  -13.979,  6.6836,  15.36,  -0.63524,  -5.9955,
     -14.512,  44.36,  -19.303,  -55.006,  12.288,  22.678,
     -5.3494,  19.429,  -9.3322,  -21.98,  6.0731,  10.46,
     -8.0136,  -53.085,  14.315,  36.902,  1.7518,  -17.152,
     -0.2295,  -5.9546,  1.5252,  -1.8162,  1.741,  -2.1481,
     -2.7403,  10.285,  -26.856,  -12.817,  24.169,  20.494,
     -0.065225,  -0.30348,  -2.9922,  0.54398,  0.88881,  1.0154,
     -16.276,  32.401,  19.756,  -21.28,  -18.952,  -14.276,
     -2.6525,  3.6902,  4.2523,  -1.1171,  -3.8061,  -3.3356,
     -1.1219,  2.0536,  1.5048,  -1.2313,  -1.3688,  -1.1321,
     -1.3141,  4.2222,  -9.9058,  -3.917,  0.35742,  11.006,
     -2.1805,  6.6836,  -13.979,  -5.9955,  -0.63524,  15.36,
     14.512,  19.303,  -44.36,  -22.678,  -12.288,  55.006,
     5.3494,  9.3322,  -19.429,  -10.46,  -6.0731,  21.98,
     -2.7403,  -26.856,  10.285,  20.494,  24.169,  -12.817,
     -0.065225,  -2.9922,  -0.30348,  1.0154,  0.88881,  0.54398,
     -8.0136,  14.315,  -53.085,  -17.152,  1.7518,  36.902,
     -0.2295,  1.5252,  -5.9546,  -2.1481,  1.741,  -1.8162,
     -16.276,  19.756,  32.401,  -14.276,  -18.952,  -21.28,
     -2.6525,  4.2523,  3.6902,  -3.3356,  -3.8061,  -1.1171,
     -1.1219,  1.5048,  2.0536,  -1.1321,  -1.3688,  -1.2313,
     6.9745,  -6.1923,  -15.656,  -2.3848,  17.054,  8.1661,
     10.414,  -8.6523,  -25.087,  -3.7848,  28.115,  12.277,
     -21.894,  -32.42,  -43.608,  67.451,  -37.957,  53.378,
     -7.4312,  -20.383,  -18.251,  35.326,  -21.934,  20.835,
     31.27,  -19.993,  9.8388,  20.887,  74.715,  -27.352,
     1.3305,  5.3376,  -0.97979,  -0.43125,  7.2048,  -1.2598,
     3.6957,  -46.341,  -42.275,  68.867,  -59.588,  18.105,
     0.56596,  -3.7675,  0.043366,  5.4714,  -6.4151,  -7.2655,
     -28.755,  -146.03,  47.275,  164.25,  16.844,  -52.048,
     -2.1169,  -19.622,  4.6241,  19.527,  5.1633,  -5.2076,
     -1.6814,  -9.6669,  2.5868,  10.596,  1.6494,  -2.972,
     6.9745,  -15.656,  -6.1923,  8.1661,  17.054,  -2.3848,
     10.414,  -25.087,  -8.6523,  12.277,  28.115,  -3.7848,
     21.894,  43.608,  32.42,  -53.378,  37.957,  -67.451,
     7.4312,  18.251,  20.383,  -20.835,  21.934,  -35.326,
     3.6957,  -42.275,  -46.341,  18.105,  -59.588,  68.867,
     0.56596,  0.043366,  -3.7675,  -7.2655,  -6.4151,  5.4714,
     31.27,  9.8388,  -19.993,  -27.352,  74.715,  20.887,
     1.3305,  -0.97979,  5.3376,  -1.2598,  7.2048,  -0.43125,
     -28.755,  47.275,  -146.03,  -52.048,  16.844,  164.25,
     -2.1169,  4.6241,  -19.622,  -5.2076,  5.1633,  19.527,
     -1.6814,  2.5868,  -9.6669,  -2.972,  1.6494,  10.596,
};

float lqr_body_distance_error ;
float lqr_speed_error;
float lqr_yaw_error;
float lqr_d_yaw;
float int_pitch = 0.0f;          // 第11维状态：俯仰角积分 ∫(pitch_trans[0]-pitch_offset_eff)dt，离地冻结
float int_pitch_limit = 1.0f;    // 俯仰积分抗饱和限幅(rad·s)，现场可调
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
    float leg_b_phi0_offset = (spinning_flag == 1) ? 0.0f : speed_error * 0.05f; 

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
