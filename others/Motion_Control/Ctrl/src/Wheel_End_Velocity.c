/**
 * @file Wheel_End_Velocity.c
 * @brief 轮端速度/加速度前馈计算：由卡尔曼滤波机身速度 + 腿角/腿长解算轮端线速度和加速度。
 *
 * 输入：
 *   - kalman_body_speed, vel_acc[1], d_yaw（来自 observe_task）
 *   - VMC_L/R.b_phi0, L0, d_b_phi0, dd_b_phi0, d_L0
 * 输出：
 *   - v_L/R, a_L/R（世界系下轮端线速度/加速度，单位 m/s, m/s²）
 */

#include "Wheel_End_Velocity.h"
#include "observe_task.h"
#include "motor.h"
#include "arm_math.h"

/**
 * @brief 由卡尔曼滤波机身速度 + 腿角腿长，计算左右轮端在世界系下的线速度和加速度
 *
 * 分两步：
 *   1. 差速模型：由质心线速度/加速度 + 横摆角速度算出左右髋关节各自的线速度/加速度
 *      v_hip_L = kalman_body_speed - d_yaw * HALF_TRACK_WIDTH
 *      v_hip_R = kalman_body_speed + d_yaw * HALF_TRACK_WIDTH
 *      加速度同理（使用上一帧 d_yaw 差分得到 dd_yaw）
 *
 *   2. 腿摆动补偿：髋关节速度加上腿绕髋摆动在水平方向产生的附加线速度
 *      即现有正向运动学中 d_b_phi0 * L0 * cos(b_phi0) 这一项
 *      v_wheel = v_hip + d_b_phi0 * L0 * cos(b_phi0)
 *      a_wheel = a_hip + dd_b_phi0 * L0 * cos(b_phi0)
 *                      + d_b_phi0 * d_L0 * cos(b_phi0)
 *                      - d_b_phi0² * L0 * sin(b_phi0)
 *
 * @param[out] v_L 左轮端线速度 (m/s)
 * @param[out] a_L 左轮端线加速度 (m/s²)
 * @param[out] v_R 右轮端线速度 (m/s)
 * @param[out] a_R 右轮端线加速度 (m/s²)
 */
void Wheel_End_Velocity_Both(float *v_L, float *a_L, float *v_R, float *a_R)
{
    // ---- 计算横摆角加速度 dd_yaw（由 d_yaw 差分得到，控制周期 0.002s）----
    static float last_d_yaw = 0.0f;
    float dd_yaw = (d_yaw - last_d_yaw) / 0.002f;
    last_d_yaw = d_yaw;

    // ---- 差速模型：质心 → 左右髋关节线速度/加速度 ----
    float v_hip_L = kalman_body_speed - d_yaw * HALF_TRACK_WIDTH;
    float v_hip_R = kalman_body_speed + d_yaw * HALF_TRACK_WIDTH;

    float a_hip_L = vel_acc[1] - dd_yaw * HALF_TRACK_WIDTH;
    float a_hip_R = vel_acc[1] + dd_yaw * HALF_TRACK_WIDTH;

    // ---- 左腿：加腿摆动速度/加速度 ----
    float cos_b_phi0_L = arm_cos_f32(VMC_L.b_phi0);
    float sin_b_phi0_L = arm_sin_f32(VMC_L.b_phi0);

    float v_rel_L = VMC_L.d_b_phi0 * VMC_L.L0 * cos_b_phi0_L;
    *v_L = v_hip_L + v_rel_L;

    *a_L = a_hip_L
         + VMC_L.dd_b_phi0 * VMC_L.L0 * cos_b_phi0_L
         + VMC_L.d_b_phi0 * VMC_L.d_L0 * cos_b_phi0_L
         - VMC_L.d_b_phi0 * VMC_L.d_b_phi0 * VMC_L.L0 * sin_b_phi0_L;

    // ---- 右腿：加腿摆动速度/加速度 ----
    float cos_b_phi0_R = arm_cos_f32(VMC_R.b_phi0);
    float sin_b_phi0_R = arm_sin_f32(VMC_R.b_phi0);

    float v_rel_R = VMC_R.d_b_phi0 * VMC_R.L0 * cos_b_phi0_R;
    *v_R = v_hip_R + v_rel_R;

    *a_R = a_hip_R
         + VMC_R.dd_b_phi0 * VMC_R.L0 * cos_b_phi0_R
         + VMC_R.d_b_phi0 * VMC_R.d_L0 * cos_b_phi0_R
         - VMC_R.d_b_phi0 * VMC_R.d_b_phi0 * VMC_R.L0 * sin_b_phi0_R;
}