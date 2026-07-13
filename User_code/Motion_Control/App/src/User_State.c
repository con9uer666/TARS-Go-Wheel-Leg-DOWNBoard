/**
 * @file User_State.c
 * @brief 用户调试/状态变量定义。
 */

#include "User_State.h"

/** @brief 用户重力补偿测试功能开关（调试接口）。0: 关闭，1: 开启。 */
uint8_t user_Gravity_Compensation_Test_Function_set = 0;

/**
 * @brief 运动模式标志。
 *
 * 0: 刚从急停退出/待机
 * 1: 正常行走（Standing）
 * 2: 正在上楼模式（Upstair）
 * 3: 趴下/坐地模式（Sit_On_Ground）
 */
uint8_t start_mode = 0;