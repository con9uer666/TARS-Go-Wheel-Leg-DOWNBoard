/**
 * @file lqr_calculate.c
 * @brief LQR 力矩计算：由 10 维状态 × 4×10 增益矩阵算出左右轮力矩与左右模拟腿力矩。
 *        并提供 LQR_Update_K()：按 100Hz 节流刷新 K 矩阵（K 值拟合较耗时）。
 *
 * LQR 状态（10维）：
 *   [ body_distance_error, speed_error, yaw_error, d_yaw,
 *     VMC_L.b_phi0, VMC_L.d_b_phi0, VMC_R.b_phi0, VMC_R.d_b_phi0,
 *     pitch_trans[0], d_pitch ]
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
float pitch_offset_eff   = -0.15f;     //运行时生效的pitch偏置，初值与PITCH_OFFSET一致避免首拍跳变

float LQR_K[4][10] = {
    -1.5319,  -4.4506,  -4.8607,  -0.84003,  -6.7098,  -0.90493,  -5.4903,  -0.83328,  -9.7772,  -0.90614,
     -1.5319,  -4.4506,  4.8607,  0.84003,  -5.4903,  -0.83328,  -6.7098,  -0.90493,  -9.7772,  -0.90614,
     12.165,  34.823,  -14.353,  -2.4417,  74.246,  7.2706,  4.5678,  3.7992,  -108.66,  -0.62272,
     12.165,  34.823,  14.353,  2.4417,  4.5678,  3.7992,  74.246,  7.2706,  -108.66,  -0.62272
};

float K_Fit_Coefficients[40][6] = {
0.025121,  -9.2416,  2.4065,  9.5832,  1.0396,  -2.403,
     -0.10134,  -20.704,  5.6628,  21.484,  1.5793,  -5.4275,
     -11.551,  42.46,  -20.392,  -38.027,  -0.55309,  23.617,
     -2.2884,  10.232,  -7.0008,  -5.7981,  -1.2154,  7.9753,
     -4.5954,  -67.743,  10.589,  48.606,  16.608,  -15.024,
     0.031271,  -6.338,  1.1749,  -2.2384,  3.5566,  -2.0846,
     -0.066199,  -2.7065,  -22.705,  13.51,  0.78679,  16.545,
     0.13212,  -1.7258,  -1.8516,  3.4299,  -2.7238,  0.34574,
     -12.399,  10.764,  16.666,  8.018,  -16.317,  -10.751,
     -1.8497,  0.34058,  3.7207,  2.4919,  -2.7643,  -2.9296,
     0.025121,  2.4065,  -9.2416,  -2.403,  1.0396,  9.5832,
     -0.10134,  5.6628,  -20.704,  -5.4275,  1.5793,  21.484,
     11.551,  20.392,  -42.46,  -23.617,  0.55309,  38.027,
     2.2884,  7.0008,  -10.232,  -7.9753,  1.2154,  5.7981,
     -0.066199,  -22.705,  -2.7065,  16.545,  0.78679,  13.51,
     0.13212,  -1.8516,  -1.7258,  0.34574,  -2.7238,  3.4299,
     -4.5954,  10.589,  -67.743,  -15.024,  16.608,  48.606,
     0.031271,  1.1749,  -6.338,  -2.0846,  3.5566,  -2.2384,
     -12.399,  16.666,  10.764,  -10.751,  -16.317,  8.018,
     -1.8497,  3.7207,  0.34058,  -2.9296,  -2.7643,  2.4919,
     8.4322,  -1.5359,  -16.278,  -17.863,  19.593,  10.253,
     19.78,  -4.3411,  -40.165,  -41.18,  51.633,  23.407,
     -15.546,  -101.77,  -28.044,  163.67,  -68.414,  48.602,
     -2.3132,  -35.362,  -1.9451,  50.762,  -28.348,  5.9445,
     68.402,  -59.125,  -6.0128,  2.5938,  81.812,  -13.308,
     2.2067,  14.607,  -3.6773,  -15.238,  8.231,  1.4574,
     7.181,  -60.995,  0.41124,  74.597,  -107.67,  17.509,
     0.67787,  -2.3033,  5.6756,  0.74248,  -13,  -4.4736,
     -5.9674,  -216.73,  34.749,  216.2,  52.204,  -51.509,
     1.4529,  -23.145,  -0.49862,  17.601,  12.719,  -2.2251,
     8.4322,  -16.278,  -1.5359,  10.253,  19.593,  -17.863,
     19.78,  -40.165,  -4.3411,  23.407,  51.633,  -41.18,
     15.546,  28.044,  101.77,  -48.602,  68.414,  -163.67,
     2.3132,  1.9451,  35.362,  -5.9445,  28.348,  -50.762,
     7.181,  0.41124,  -60.995,  17.509,  -107.67,  74.597,
     0.67787,  5.6756,  -2.3033,  -4.4736,  -13,  0.74248,
     68.402,  -6.0128,  -59.125,  -13.308,  81.812,  2.5938,
     2.2067,  -3.6773,  14.607,  1.4574,  8.231,  -15.238,
     -5.9674,  34.749,  -216.73,  -51.509,  52.204,  216.2,
     1.4529,  -0.49862,  -23.145,  -2.2251,  12.719,  17.601,
};

float lqr_body_distance_error ;
float lqr_speed_error;
float lqr_yaw_error;
float lqr_d_yaw;
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
    float leg_b_phi0_offset = (spinning_flag == 1) ? 0.0f : b_phi0_offset;

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
    + LQR_K[0][9] * d_pitch;

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
    + LQR_K[1][9] * d_pitch;

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
    + LQR_K[2][9] * d_pitch;

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
    + LQR_K[3][9] * d_pitch;
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
