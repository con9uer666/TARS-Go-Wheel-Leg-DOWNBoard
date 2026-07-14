/**
 * @file Lqr_Error_Calculate.h
 * @brief LQR 误差计算模块接口：速度误差、位移误差、Yaw 误差。
 *
 * Yaw_Error_Calculate() 统一更新 yaw_error 与 speed_error，
 * Distance_Error_Set() 随后更新位移误差。
 */

#ifndef LQR_ERROR_CALCULATE_H
#define LQR_ERROR_CALCULATE_H

#include <stdint.h>

void Error_Calculate(void);

#endif // LQR_ERROR_CALCULATE_H
