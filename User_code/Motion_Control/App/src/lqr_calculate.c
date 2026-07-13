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
-4.4864,  -26.302,  14.454,  29.963,  -3.407,  -12.172,
     -4.7029,  -24.035,  14.691,  27.182,  -5.6658,  -11.888,
     -18.974,  57.524,  -21.007,  -74.869,  18.272,  25.913,
     -6.8255,  25.419,  -10.519,  -29.8,  8.5295,  12.531,
     -12.152,  -62.876,  16.328,  36.519,  0.19358,  -20.115,
     -0.35443,  -8.2663,  2.2085,  -3.6216,  2.2063,  -2.9825,
     -3.2062,  11.408,  -29.081,  -16.781,  27.676,  23.212,
     -0.093467,  -0.7295,  -3.7893,  1.3988,  -1.0485,  2.0283,
     -18.457,  38.417,  18.895,  -27.548,  -17.402,  -14.531,
     -2.8829,  3.6791,  4.7709,  -0.76297,  -4.1504,  -3.9878,
     -1.0255,  1.925,  1.239,  -1.2518,  -1.0694,  -0.99323,
     -1.6477,  -10.55,  5.4373,  12.059,  -0.74524,  -4.7002,
     -4.4864,  14.454,  -26.302,  -12.172,  -3.407,  29.963,
     -4.7029,  14.691,  -24.035,  -11.888,  -5.6658,  27.182,
     18.974,  21.007,  -57.524,  -25.913,  -18.272,  74.869,
     6.8255,  10.519,  -25.419,  -12.531,  -8.5295,  29.8,
     -3.2062,  -29.081,  11.408,  23.212,  27.676,  -16.781,
     -0.093467,  -3.7893,  -0.7295,  2.0283,  -1.0485,  1.3988,
     -12.152,  16.328,  -62.876,  -20.115,  0.19358,  36.519,
     -0.35443,  2.2085,  -8.2663,  -2.9825,  2.2063,  -3.6216,
     -18.457,  18.895,  38.417,  -14.531,  -17.402,  -27.548,
     -2.8829,  4.7709,  3.6791,  -3.9878,  -4.1504,  -0.76297,
     -1.0255,  1.239,  1.925,  -0.99323,  -1.0694,  -1.2518,
     -1.6477,  5.4373,  -10.55,  -4.7002,  -0.74524,  12.059,
     12.645,  -7.3525,  -31.836,  -8.7246,  36.138,  14.459,
     12.198,  -6.1261,  -32.901,  -8.9592,  38.088,  14.035,
     -26.549,  -15.222,  -48.282,  43.407,  -28.708,  55.869,
     -8.9723,  -14.241,  -19.731,  25.735,  -16.104,  20.506,
     28.058,  -11.758,  8.108,  25.246,  58.248,  -23.553,
     1.2403,  5.8563,  -1.3104,  0.99888,  4.9848,  -0.88311,
     2.3531,  -28.641,  -48.989,  44.307,  -37.543,  12.821,
     0.52278,  -2.7021,  -0.40202,  3.0998,  -1.6884,  -9.3707,
     -29.083,  -115.31,  45.797,  132.09,  3.9433,  -46.48,
     -1.8506,  -14.331,  3.7252,  13.851,  3.9173,  -4.0475,
     -1.3641,  -6.0528,  1.9831,  6.7233,  0.63031,  -2.0865,
     4.918,  -3.1143,  -11.819,  -3.209,  13.228,  5.5791,
     12.645,  -31.836,  -7.3525,  14.459,  36.138,  -8.7246,
     12.198,  -32.901,  -6.1261,  14.035,  38.088,  -8.9592,
     26.549,  48.282,  15.222,  -55.869,  28.708,  -43.407,
     8.9723,  19.731,  14.241,  -20.506,  16.104,  -25.735,
     2.3531,  -48.989,  -28.641,  12.821,  -37.543,  44.307,
     0.52278,  -0.40202,  -2.7021,  -9.3707,  -1.6884,  3.0998,
     28.058,  8.108,  -11.758,  -23.553,  58.248,  25.246,
     1.2403,  -1.3104,  5.8563,  -0.88311,  4.9848,  0.99888,
     -29.083,  45.797,  -115.31,  -46.48,  3.9433,  132.09,
     -1.8506,  3.7252,  -14.331,  -4.0475,  3.9173,  13.851,
     -1.3641,  1.9831,  -6.0528,  -2.0865,  0.63031,  6.7233,
     4.918,  -11.819,  -3.1143,  5.5791,  13.228,  -3.209,
};

float lqr_body_distance_error ;         //lqr内部变量，会被限幅
float lqr_speed_error;                  //lqr内部变量，会被限幅
float lqr_yaw_error;                    //lqr内部变量，会被限幅，弧度制
float lqr_d_yaw;                        //lqr内部变量，会被限幅

float int_pitch = 0.0f;          // 第11维状态：俯仰角积分 ∫(pitch_trans[0]-pitch_offset_eff)dt，离地冻结
float int_pitch_limit = 0.5f;    // 俯仰积分抗饱和限幅(rad·s)，现场可调
float int_s = 0.0f;              // 第12维状态：位移积分 ∫(body_distance_error)dt，离地/小陀螺/上台阶冻结
float int_s_limit = 1.0f;        // 位移积分抗饱和限幅(m·s)，现场可调
void LQR_calculate()
{
	lqr_body_distance_error = body_distance_error;
	lqr_speed_error = speed_error;
	lqr_yaw_error = yaw_error;
	lqr_d_yaw = d_yaw;

    float leg_b_phi0_offset ;
    if(speed_error >= 0)
    {
        leg_b_phi0_offset = speed_error * 0.02f; 
    }
    else
    {
        leg_b_phi0_offset = speed_error * 0.02f; 
    }

    //上台阶动作组和小陀螺时清零位移和位移积分，腿没有前馈摆角
    if(spinning_flag == 1 || upstairs_flag == 1)
    {
        lqr_body_distance_error = 0.0f;
        int_s = 0.0f;
        leg_b_phi0_offset = 0.0f;
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
    VMC_Chassis_Target.L_Wheel_Torque =
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

    VMC_Chassis_Target.R_Wheel_Torque =
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
    + LQR_K[2][2] * (-lqr_yaw_error)
    - LQR_K[2][3] * lqr_d_yaw
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
    + LQR_K[3][2] * (-lqr_yaw_error)
    - LQR_K[3][3] * lqr_d_yaw
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
