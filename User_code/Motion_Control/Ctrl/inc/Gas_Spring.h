/**
 * @file Gas_Spring.h
 * @brief 气弹簧力-位移特性拟合接口。
 *
 * 提供根据腿长 L0 计算气弹簧辅助弹力的函数声明，
 * 用于腿长控制中的力补偿。
 *
 * @note 拟合系数及细节参见 Gas_Spring.c。
 */

#ifndef GAS_SPRING_H
#define GAS_SPRING_H

#include <stdint.h>

/** @brief 气弹簧补偿使能标志位。1:启用, 0:关闭（外部定义于 Gas_Spring.c） */
extern uint8_t gas_spring_enable;

/**
 * @brief 根据当前腿长计算气弹簧输出力。
 * @param[in] L0_m 当前腿长，单位 m。
 * @return 气弹簧力，单位 N（未启用时返回 0）。
 */
float Gas_Spring_GetForce(float L0_m);

#endif
