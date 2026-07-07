/**
 * @file lqr_calculate.c
 * @brief LQR 力矩计算：由 12 维状态 × 4×12 增益矩阵算出左右轮力矩与左右模拟腿力矩。
 *        并提供 LQR_Update_K()：按 100Hz 节流刷新 K 矩阵（K 值拟合较耗时）。
 *
 * LQR 状态（12维）：
 *   [ body_distance_error, speed_error, yaw_error, d_yaw,
 *     VMC_L.b_phi0, VMC_L.d_b_phi0, VMC_R.b_phi0, VMC_R.d_b_phi0,
 *     pitch_trans[0], d_pitch, int_pitch, int_s ]
 *   其中 int_pitch = ∫(pitch_trans[0] - pitch_offset_eff) dt，离地时冻结；
 *       int_s     = ∫(body_distance_error) dt，离地/小陀螺/上台阶时冻结。
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

// 上电初值/备用值；上电 5 拍(约50ms)后即由 LQR_Update_K()→LQR_Get_K() 全量覆盖。
// 第 12 列(int_s 位移积分增益)暂置 0；跑完 12 维 LQR.m 后可用新模型定腿长输出整体替换。
float LQR_K[4][12] = {
    -1.9045,  -2.9573,  -6.2325,  -1.2595,  -13.805,  -0.96974,  -3.4901,  -0.45392,  -10.489,  -1.5631,  -0.72426,  0,
     -1.9045,  -2.9573,  6.2325,  1.2595,  -3.4901,  -0.45392,  -13.805,  -0.96974,  -10.489,  -1.5631,  -0.72426,  0,
     4.5806,  6.7741,  -26.254,  -6.699,  29.403,  2.0429,  -5.5104,  0.023442,  -38.831,  -2.861,  -2.4091,  0,
     4.5806,  6.7741,  26.254,  6.699,  -5.5104,  0.023442,  29.403,  2.0429,  -38.831,  -2.861,  -2.4091,  0
};

// ⚠️ 必须整体替换：列数 11→12 后，LQR_Get_K 的行排布已改为 pos = i*12 + j，
//    下方为旧 11 维(44行)数据，在新排布下已全部错位，上台前务必用 12 维 LQR.m
//    输出的 48 行系数整体覆盖，否则在线拟合出的 K 全错。
float K_Fit_Coefficients[48][6] = {
-3.9469,  -22.599,  13.679,  26.722,  -5.9985,  -9.5519,
     -5.1839,  -24.98,  17.34,  30.788,  -11.412,  -10.94,
     -18.768,  57.477,  -18.683,  -74.999,  17.776,  22.293,
     -6.6933,  25.211,  -10.647,  -28.777,  7.1658,  12.809,
     -17.168,  -64.172,  15.277,  43.633,  1.193,  -18.3,
     -0.43733,  -8.7641,  2.2858,  -3.1408,  2.4129,  -2.7727,
     -3.497,  12.31,  -27.974,  -17.709,  24.821,  23.443,
     -0.12061,  -0.78975,  -3.7305,  1.9165,  -2.2212,  2.7685,
     -19.509,  43.052,  18.799,  -32.873,  -17.404,  -14.729,
     -3.2508,  4.4626,  5.3689,  -1.288,  -4.8608,  -4.3393,
     -1.3944,  2.7133,  1.6948,  -1.8234,  -1.5188,  -1.3444,
     -1.0081,  -6.2791,  3.567,  7.292,  -1.159,  -2.6175,
     -3.9469,  13.679,  -22.599,  -9.5519,  -5.9985,  26.722,
     -5.1839,  17.34,  -24.98,  -10.94,  -11.412,  30.788,
     18.768,  18.683,  -57.477,  -22.293,  -17.776,  74.999,
     6.6933,  10.647,  -25.211,  -12.809,  -7.1658,  28.777,
     -3.497,  -27.974,  12.31,  23.443,  24.821,  -17.709,
     -0.12061,  -3.7305,  -0.78975,  2.7685,  -2.2212,  1.9165,
     -17.168,  15.277,  -64.172,  -18.3,  1.193,  43.633,
     -0.43733,  2.2858,  -8.7641,  -2.7727,  2.4129,  -3.1408,
     -19.509,  18.799,  43.052,  -14.729,  -17.404,  -32.873,
     -3.2508,  5.3689,  4.4626,  -4.3393,  -4.8608,  -1.288,
     -1.3944,  1.6948,  2.7133,  -1.3444,  -1.5188,  -1.8234,
     -1.0081,  3.567,  -6.2791,  -2.6175,  -1.159,  7.292,
     10.189,  -5.0561,  -27.362,  -9.8795,  33.271,  11.777,
     12.293,  -5.7922,  -35.13,  -12.278,  44.348,  14.091,
     -27.427,  -14.131,  -48.619,  39.672,  -25.939,  58.073,
     -9.2435,  -14.003,  -18.743,  24.109,  -15.251,  19.878,
     37.815,  -47.666,  10.004,  67.327,  54.268,  -26.323,
     1.4031,  5.3241,  -1.4046,  1.3498,  5.2347,  -1.0409,
     0.82391,  -26.889,  -49.396,  40.765,  -31.568,  12.073,
     0.48992,  -2.5624,  -0.76471,  2.4266,  -0.63073,  -9.4383,
     -34.976,  -120.02,  52.177,  139.76,  2.4325,  -54.399,
     -2.8581,  -16.018,  4.5296,  15.721,  4.4692,  -5.2024,
     -2.0521,  -7.9844,  2.7432,  8.9252,  0.94965,  -3.0432,
     2.7201,  -1.3937,  -7.0641,  -2.5787,  8.4153,  3.1482,
     10.189,  -27.362,  -5.0561,  11.777,  33.271,  -9.8795,
     12.293,  -35.13,  -5.7922,  14.091,  44.348,  -12.278,
     27.427,  48.619,  14.131,  -58.073,  25.939,  -39.672,
     9.2435,  18.743,  14.003,  -19.878,  15.251,  -24.109,
     0.82391,  -49.396,  -26.889,  12.073,  -31.568,  40.765,
     0.48992,  -0.76471,  -2.5624,  -9.4383,  -0.63073,  2.4266,
     37.815,  10.004,  -47.666,  -26.323,  54.268,  67.327,
     1.4031,  -1.4046,  5.3241,  -1.0409,  5.2347,  1.3498,
     -34.976,  52.177,  -120.02,  -54.399,  2.4325,  139.76,
     -2.8581,  4.5296,  -16.018,  -5.2024,  4.4692,  15.721,
     -2.0521,  2.7432,  -7.9844,  -3.0432,  0.94965,  8.9252,
     2.7201,  -7.0641,  -1.3937,  3.1482,  8.4153,  -2.5787,
};

float lqr_body_distance_error ;
float lqr_speed_error;
float lqr_yaw_error;
float lqr_d_yaw;
float int_pitch = 0.0f;          // 第11维状态：俯仰角积分 ∫(pitch_trans[0]-pitch_offset_eff)dt，离地冻结
float int_pitch_limit = 1.0f;    // 俯仰积分抗饱和限幅(rad·s)，现场可调
float int_s = 0.0f;              // 第12维状态：位移积分 ∫(body_distance_error)dt，离地/小陀螺/上台阶冻结
float int_s_limit = 1.0f;        // 位移积分抗饱和限幅(m·s)，现场可调
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

    float leg_b_phi0_offset ;
    if(speed_error >= 0)
    {
        leg_b_phi0_offset = (spinning_flag == 1) ? 0.0f : speed_error * 0.02f; 
    }
    if(speed_error < 0)
    {
        leg_b_phi0_offset = (spinning_flag == 1) ? 0.0f : speed_error * 0.02f; 
    }

    // 第11维状态：俯仰角积分。任一腿离地时冻结(不累加)，其余(含小陀螺)正常积分，带抗饱和限幅。
    // 第12维状态：位移积分。同样离地冻结；小陀螺/上台阶时 lqr_body_distance_error 已被清零，累加自然冻结。
    if (L_off_ground < 10 && R_off_ground < 10)
    {
        int_pitch += (pitch_trans[0] - pitch_offset_eff) * 0.002f;
        if (int_pitch >  int_pitch_limit) int_pitch =  int_pitch_limit;
        if (int_pitch < -int_pitch_limit) int_pitch = -int_pitch_limit;

        int_s += lqr_body_distance_error * 0.002f;
        if (int_s >  int_s_limit) int_s =  int_s_limit;
        if (int_s < -int_s_limit) int_s = -int_s_limit;
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
    + LQR_K[0][10] * int_pitch
    + LQR_K[0][11] * int_s;

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
    + LQR_K[1][10] * int_pitch
    + LQR_K[1][11] * int_s;

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
    + LQR_K[2][10] * int_pitch
    + LQR_K[2][11] * int_s;

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
    + LQR_K[3][10] * int_pitch
    + LQR_K[3][11] * int_s;
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
