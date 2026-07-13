/**
 * @file User_State.h
 * @brief 用户调试/状态变量声明。
 *
 * 提供外部可访问的调试接口和模式标志。
 */

#ifndef USER_STATE_H
#define USER_STATE_H

#include "main.h"

/** @brief 用户重力补偿测试功能开关（调试接口） */
extern uint8_t user_Gravity_Compensation_Test_Function_set;

/**
 * @brief 运动模式标志。
 *
 * 0: 刚从急停退出/待机
 * 1: 正常行走（Standing）
 * 2: 正在上楼模式（Upstair）
 * 3: 趴下/坐地模式（Sit_On_Ground）
 */
extern uint8_t start_mode;

#endif // USER_STATE_H