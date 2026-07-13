/**
 * @file ramp_generator.c
 * @brief 通用一阶斜坡发生器实现。
 */

#include "ramp_generator.h"

// 一阶斜坡发生器更新函数
void rampIterate(RampGenerator *ramp)
{
    if (ramp->isBusy)
    {
        if (ramp->currentValue < ramp->targetValue)
        {                                     // 如果当前值小于目标值
            ramp->currentValue += ramp->step; // 增大当前值
            if (ramp->currentValue > ramp->targetValue)
            { // 避免超调
                ramp->currentValue = ramp->targetValue;
            }
        }
        else if (ramp->currentValue > ramp->targetValue)
        {                                     // 如果当前值大于目标值
            ramp->currentValue -= ramp->step; // 减小当前值
            if (ramp->currentValue < ramp->targetValue)
            { // 避免超调
                ramp->currentValue = ramp->targetValue;
            }
        }

        // 判断是否达到目标
        // if (ramp->currentValue == ramp->targetValue)
        // {
        //     ramp->isBusy = 0; // 达到目标，标记为不忙碌
        // }
    }
}

/**
 * @brief 初始化斜坡发生器
 *
 * @param ramp 斜坡发生器结构体指针
 * @param startValue 开始值
 * @param targetValue 目标值
 * @param time 总时间
 * @param cycleTime 周期时间
 */
void rampInit(RampGenerator *ramp, float startValue, float targetValue, float time, float cycleTime)
{
    ramp->currentValue = startValue;
    ramp->targetValue = targetValue;
    // 计算步进值，这里需要注意的是，确保斜坡时间和周期时间都不为零来避免除以零的错误
    if (time != 0 && cycleTime != 0)
    {
        if(targetValue>= startValue)
        {
            ramp->step = (targetValue - startValue) * (cycleTime / time); // 计算步进值，确保在指定时间内达到目标值
        }
        else
        {
            ramp->step = (startValue - targetValue) * (cycleTime / time); // 计算步进值，确保在指定时间内达到目标值
        }
    }
    else
    {
        ramp->step = 0; // 出错情况下设置为0，避免非法操作
    }
    ramp->isBusy = 1; // 标记为忙碌
}
