/**
 * @file Gas_Spring.c
 * @brief 气弹簧力-位移特性拟合，用于腿长控制中的辅助力补偿。
 *
 * 气弹簧力由三次多项式拟合，根据当前腿长 L0（单位 m）
 * 输出对应的弹力（单位 N）。通过全局标志位 gas_spring_enable
 * 控制是否启用补偿。
 *
 * @note 拟合曲线来源于实验标定数据，多项式系数为：
 *       2.44e-6·y³ - 2.80e-3·y² + 1.043·y - 34.66
 *       其中 y = L0_m * 1000（即腿长 mm）。
 */

#include "Gas_Spring.h"

/** @brief 气弹簧补偿使能标志位。1:启用, 0:关闭 */
uint8_t gas_spring_enable = 0;

/**
 * @brief 根据当前腿长计算气弹簧输出力。
 *
 * 当 gas_spring_enable != 0 时，将腿长 L0_m (m) 转为 mm，
 * 代入三次多项式计算气弹簧弹力 (N)；否则返回 0。
 *
 * @param[in] L0_m 当前腿长，单位 m。
 * @return 气弹簧力，单位 N（未启用时返回 0）。
 */
float Gas_Spring_GetForce(float L0_m)
{
    if (!gas_spring_enable)
        return 0.0f;

    float y = L0_m * 1000.0f;

    return 1.0 * (2.44e-6f * y * y * y - 2.80e-3f * y * y + 1.043f * y - 34.66f);
}
