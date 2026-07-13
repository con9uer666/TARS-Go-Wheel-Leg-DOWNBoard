/**
 * @file Wheel_Leg_about.h
 * @brief 轮腿控制核心算法接口。
 *
 * 提供 LQR 增益拟合、防劈叉、横滚补偿、腿长控制、
 * 速度/距离误差计算、车身速度估计和惯性导航解算的函数声明。
 */

#ifndef WHEEL_LEG_ABOUT_H
#define WHEEL_LEG_ABOUT_H

#include "main.h"

/**
 * @brief 二维多项式拟合 LQR 反馈增益矩阵。
 * @param[out] LQR                 4×12 反馈增益矩阵。
 * @param[in]  K_Fit_Coefficients  48×6 拟合系数表。
 * @param[in]  L0_l               左腿腿长，单位 m。
 * @param[in]  L0_r               右腿腿长，单位 m。
 */
void LQR_Get_K(float LQR[4][12], float K_Fit_Coefficients[48][6], float L0_l, float L0_r);

/**
 * @brief 防劈叉 PID 增益的一次函数计算：K(L) = p0 + p1·L（线性插值）。
 * @param[out] K      输出增益，K[0]=Kp, K[1]=Kd。
 * @param[in]  L0_avg 平均腿长，单位 m。
 */
void AntiSplit_Get_K(float *Kp, float *Kd, float L0_avg);

void Body_Speed_Coculate();
void INS_Coculate();

#endif // WHEEL_LEG_ABOUT_H
