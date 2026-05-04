#ifndef POWER_OBSERVER_LIMIT_H
#define POWER_OBSERVER_LIMIT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float buffer_min;        /* 缓冲低阈值（J），低于此值 power_limit 降到最小 */
    float buffer_max;        /* 缓冲高阈值（J），高于此值 power_limit = 裁判上限 */

    float lambda_min;        /* lambda 下限，防止完全失控 */
    float lambda_rise_rate;  /* lambda 上升速率（1/s），功率恢复后放开的速度 */
    float lambda_fall_rate;  /* lambda 下降速率（1/s），功率超限时收紧的速度 */

    float lambda_pi_ki;      /* I 增益，补偿 lambda 对功率的跟踪误差 */
} PowerObsCtrlParam;

typedef struct
{
    PowerObsCtrlParam param;

    float lambda;        /* 当前观测量缩放系数 */
    float bias;          /* I 补偿项，超限时为负值 */
    float allowed_power; /* 本周期允许功率（调试用） */
} PowerObsCtrl;

typedef struct
{
    float body_distance_error;
    float speed_error;
    float yaw_error;
    float d_yaw;
} PowerObsInput;

typedef struct
{
    float body_distance_error;
    float speed_error;
    float yaw_error;
    float d_yaw;
    float lambda;
} PowerObsOutput;

void  PowerObsCtrl_DefaultParam(PowerObsCtrlParam *param);
void  PowerObsCtrl_Init(PowerObsCtrl *ctrl, const PowerObsCtrlParam *param);

float PowerObsCtrl_ComputeLambda(PowerObsCtrl *ctrl,
                                 float power_limit,
                                 float measured_power,
                                 float dt_s);

void  PowerObsCtrl_Apply(const PowerObsCtrl *ctrl,
                         const PowerObsInput *in,
                         PowerObsOutput *out);

#ifdef __cplusplus
}
#endif

#endif
