/**
 * @file USER_CAN.h
 * @brief CAN 总线初始化与电机数据传输接口。
 */

#ifndef USER_CAN_H
#define USER_CAN_H

#include "fdcan.h"
#include "main.h"

/** @brief 调试开关：1=强制全部电机零力矩，0=不干预正常输出。 */
extern volatile uint8_t debug_force_all_motor_zero_output;

/** @brief 轮腿输出开关：0=仅关断 3508 和 8009（4310/2325 不受影响）。 */
extern uint8_t wheel_leg_output_enable;

/** @brief 电机使能监督：记录所有 DM 关节电机当前期望状态，1=应使能 0=应失能。 */
extern uint8_t motor_should_enabled;

void CAN_Init(void);
void CAN_Send_DM_Motor_Data(FDCAN_HandleTypeDef *hfdcan, int16_t StdId, uint8_t *Data);
void Error_Buzzer_Tick(void);

#endif
