#include "Maths_about.h"
#include <math.h>

void RampStep_Init(RampStep_Handle *h, float current)
{
    h->ramp_value = current;
    h->last_target = current;
    h->step_done = 0;
}

float RampStep_Update(RampStep_Handle *h, float target,
                      float step_accel, float step_decel,
                      float ramp_rate, float dt)
{
    /* 目标值变化时重置阶跃状态 */
    if (target != h->last_target) {
        h->step_done = 0;
        h->last_target = target;
    }

    /* 尚未完成初始阶跃 */
    if (!h->step_done) {
        float dir = (target > h->ramp_value) ? 1.0f : -1.0f;
        float step_abs;

        /*
         * 加速/减速判断：
         * 加速：ramp_value 与 target 同号（含 ramp_value==0 的情况）
         * 减速：ramp_value 与 target 异号
         */
        if (h->ramp_value == 0.0f || (h->ramp_value > 0.0f) == (target > 0.0f)) {
            step_abs = step_accel;   /* 加速阶跃 */
        } else {
            step_abs = step_decel;   /* 减速阶跃 */
        }

        h->ramp_value += dir * step_abs;
        h->step_done = 1;

        /* 阶跃后检查是否已越过目标 */
        if (dir > 0.0f) {
            if (h->ramp_value >= target) {
                h->ramp_value = target;
            }
        } else {
            if (h->ramp_value <= target) {
                h->ramp_value = target;
            }
        }
    }

    /* 斜坡逼近 */
    if (h->ramp_value < target) {
        h->ramp_value += ramp_rate * dt;
        if (h->ramp_value > target) {
            h->ramp_value = target;
        }
    } else if (h->ramp_value > target) {
        h->ramp_value -= ramp_rate * dt;
        if (h->ramp_value < target) {
            h->ramp_value = target;
        }
    }

    return h->ramp_value;
}