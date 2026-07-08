#ifndef __MATHS_ABOUT_H__
#define __MATHS_ABOUT_H__

#include <stdint.h>

typedef struct {
    float ramp_value;    // 当前斜坡值
    float last_target;   // 上一次目标值，用于检测变化
    uint8_t step_done;   // 是否已完成当前段斜坡的初始阶跃
} RampStep_Handle;

/**
 * @brief 初始化斜坡阶跃句柄
 * @param h       句柄指针
 * @param current 当前实际值
 */
void RampStep_Init(RampStep_Handle *h, float current);

/**
 * @brief 带初始阶跃的斜坡更新函数
 *
 * 每次目标值变化时，先进行一个瞬时的阶跃跳变，之后再以固定速率朝目标值斜坡。
 * 阶跃方向始终指向目标值，阶跃幅值根据当前值是"加速"还是"减速"分别使用不同参数。
 *
 * 加速/减速判断：目标方向与当前 ramp_value 符号相同为加速，相异为减速。
 *
 * @param h          句柄指针
 * @param target     目标值
 * @param step_accel 加速阶跃绝对值（ramp_value 与 target 同号时使用）
 * @param step_decel 减速阶跃绝对值（ramp_value 与 target 异号时使用）
 * @param ramp_rate  斜坡速率（单位/秒）
 * @param dt         时间步长（秒）
 * @return           当前斜坡值
 */
float RampStep_Update(RampStep_Handle *h, float target,
                      float step_accel, float step_decel,
                      float ramp_rate, float dt);

#endif