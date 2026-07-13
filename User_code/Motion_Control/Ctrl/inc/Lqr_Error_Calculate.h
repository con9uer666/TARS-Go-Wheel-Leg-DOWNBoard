/**
 * @file Lqr_Error_Calculate.h
 * @brief LQR 误差计算模块接口：速度误差、位移误差、Yaw 误差。
 *
 * 将 Speed_Error_Set()、Distance_Error_Set()、Yaw_Error_Coculate()
 * 三个函数集中于此，供 LQR 计算前统一更新状态误差。
 */

#ifndef LQR_ERROR_CALCULATE_H
#define LQR_ERROR_CALCULATE_H

#include <stdint.h>

void Speed_Error_Set(void);
void Distance_Error_Set(void);
void Yaw_Error_Coculate(void);

#endif // LQR_ERROR_CALCULATE_H