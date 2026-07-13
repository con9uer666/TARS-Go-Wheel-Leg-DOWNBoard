/**
 * @file Wheel_End_Velocity.h
 * @brief 轮端速度/加速度前馈计算接口。
 *
 * 提供函数 Wheel_End_Velocity_Both()，根据卡尔曼滤波机身状态和
 * 腿角/腿长解算出左右轮端在世界系下的线速度和加速度。
 */

#ifndef WHEEL_END_VELOCITY_H
#define WHEEL_END_VELOCITY_H

#include "main.h"
#include "VMC.h"

/** @brief 左右髋关节间距之半 (m)，即半轮距 */
#define HALF_TRACK_WIDTH 0.19242f

/**
 * @brief 计算左右轮端在世界系下的线速度和加速度。
 *
 * 输入依赖：
 *   - kalman_body_speed, vel_acc[1], d_yaw（全局变量）
 *   - VMC_L/R 的 b_phi0, L0, d_b_phi0, dd_b_phi0, d_L0
 *
 * 输出：
 *   - v_L/R: 轮端线速度 (m/s)
 *   - a_L/R: 轮端线加速度 (m/s²)
 *
 * @param[out] v_L 左轮端线速度。
 * @param[out] a_L 左轮端线加速度。
 * @param[out] v_R 右轮端线速度。
 * @param[out] a_R 右轮端线加速度。
 */
void Wheel_End_Velocity_Both(float *v_L, float *a_L, float *v_R, float *a_R);

#endif // WHEEL_END_VELOCITY_H