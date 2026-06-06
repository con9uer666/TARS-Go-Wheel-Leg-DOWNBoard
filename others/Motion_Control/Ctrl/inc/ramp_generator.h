/**
 * @file ramp_generator.h
 * @brief 通用一阶斜坡发生器：平滑目标值过渡，避免阶跃。
 */

#ifndef RAMP_GENERATOR_H
#define RAMP_GENERATOR_H

#include "main.h"

/**
 * @brief 一阶斜坡发生器。
 *
 * 用于平滑过渡目标值（如速度/腿长），避免阶跃。
 * 每周期调用 rampIterate() 更新当前值。
 */
typedef struct RampGenerator
{
    float currentValue; /**< 当前值 */
    float targetValue;  /**< 目标值 */
    float step;         /**< 每个控制周期应当改变的数值大小 */
    uint8_t isBusy;     /**< 指示斜坡发生器是否正在调整中 */
} RampGenerator;

void rampIterate(RampGenerator *ramp);
void rampInit(RampGenerator *ramp, float startValue, float targetValue, float time, float cycleTime);

#endif // RAMP_GENERATOR_H
