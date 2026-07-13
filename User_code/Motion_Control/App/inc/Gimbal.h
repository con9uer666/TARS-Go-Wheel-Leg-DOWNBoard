/**
 * @file Gimbal.h
 * @brief 云台控制接口。
 *
 * 提供云台偏航角度环 PID 控制、底盘跟随/小陀螺模式切换所需的全局变量。
 */

#ifndef GIMBAL_H
#define GIMBAL_H

#include "main.h"

/**
 * @brief 标零处理后的 yaw 角度 (rad)，范围 [-PI, PI]。
 *
 * 以 head_forward_angle 为基准，将 Yaw_DM4310 编码器角度归一化到此范围。
 */
extern float yaw_angle_PI;

/**
 * @brief 正视前方的 yaw 电机角度 (rad)。
 *
 * 以该角度为基准进行标零处理。
 */
extern float head_forward_angle;

#endif // GIMBAL_H