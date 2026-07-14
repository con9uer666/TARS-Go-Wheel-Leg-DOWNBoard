/**
 * @file Wheel_Leg_about.h
 * @brief 轮腿控制核心算法接口。
 *
 * 提供横滚补偿、腿长控制、车身速度估计和惯性导航解算的函数声明。
 */

#ifndef WHEEL_LEG_ABOUT_H
#define WHEEL_LEG_ABOUT_H

#include "main.h"

void Body_Speed_Coculate();
void INS_Coculate();

#endif // WHEEL_LEG_ABOUT_H
