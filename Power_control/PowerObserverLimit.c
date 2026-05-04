#include "PowerObserverLimit.h"
#include <math.h>

static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

void PowerObsCtrl_DefaultParam(PowerObsCtrlParam *param)
{
    if (param == NULL) return;

    // ===== 调参区 =====
    param->buffer_min       = 10.0f;  // 缓冲低于此值时功率上限降到最小（J）
    param->buffer_max       = 60.0f;  // 缓冲高于此值时功率上限 = 裁判上限（J）

    param->lambda_min       = 0.15f;  // lambda 最低值，防止完全失控
    param->lambda_rise_rate = 5.0f;   // 功率恢复后 lambda 上升速率（1/s），越大恢复越快
    param->lambda_fall_rate = 15.0f;  // 功率超限时 lambda 下降速率（1/s），越大收紧越快

    param->lambda_pi_ki     = 0.002f; // I 增益，补偿 lambda 跟踪误差；太大会震荡
    // ==================
}

void PowerObsCtrl_Init(PowerObsCtrl *ctrl, const PowerObsCtrlParam *param)
{
    PowerObsCtrlParam fallback;

    if (ctrl == NULL) return;

    if (param == NULL)
    {
        PowerObsCtrl_DefaultParam(&fallback);
        param = &fallback;
    }

    ctrl->param        = *param;
    ctrl->lambda       = 1.0f;
    ctrl->bias         = 0.0f;
    ctrl->allowed_power = 0.0f;
}

/*
 * 计算并更新 lambda。
 *
 * 逻辑：
 *   若 measured_power > power_limit（功率超限）：
 *     feedforward = power_limit / measured_power  （立即给出比例估计）
 *     bias -= Ki * (measured - limit) * dt        （I 项补偿跟踪误差）
 *     lambda_target = clamp(feedforward + bias, lambda_min, 1)
 *   否则（功率在限内）：
 *     bias 清零，lambda_target = 1
 *   最后对 lambda 做斜坡限制，防止突变。
 */
float PowerObsCtrl_ComputeLambda(PowerObsCtrl *ctrl,
                                 float power_limit,
                                 float measured_power,
                                 float dt_s)
{
    float lambda_target;
    float delta;
    float max_step;

    if (ctrl == NULL) return 1.0f;

    ctrl->allowed_power = power_limit;

    if ((measured_power > power_limit) && (measured_power > 1.0f))
    {
        float feedforward = power_limit / measured_power;
        float error       = measured_power - power_limit;

        ctrl->bias -= ctrl->param.lambda_pi_ki * error * dt_s;
        // bias 只允许为负（进一步压低 lambda），上限为 0
        ctrl->bias = clampf(ctrl->bias, -(1.0f - ctrl->param.lambda_min), 0.0f);

        lambda_target = clampf(feedforward + ctrl->bias,
                               ctrl->param.lambda_min, 1.0f);
    }
    else
    {
        ctrl->bias    = 0.0f;
        lambda_target = 1.0f;
    }

    // 斜坡限制：下降用 fall_rate，上升用 rise_rate
    delta    = lambda_target - ctrl->lambda;
    max_step = (delta < 0.0f) ? ctrl->param.lambda_fall_rate * dt_s
                              : ctrl->param.lambda_rise_rate * dt_s;
    ctrl->lambda = clampf(ctrl->lambda + clampf(delta, -max_step, max_step),
                          ctrl->param.lambda_min, 1.0f);

    return ctrl->lambda;
}

void PowerObsCtrl_Apply(const PowerObsCtrl *ctrl,
                        const PowerObsInput *in,
                        PowerObsOutput *out)
{
    float lambda;
    float lambda_speed;
    float lambda_yaw;

    if ((in == NULL) || (out == NULL)) return;

    lambda = (ctrl == NULL) ? 1.0f : ctrl->lambda;
    lambda = clampf(lambda, 0.0f, 1.0f);

    // 速度通道比姿态通道收缩更快（全速前进时速度误差主导功率）
    lambda_speed = (lambda < 0.45f) ? lambda * lambda : powf(lambda, 1.2f);
    lambda_yaw   = powf(lambda, 1.1f);

    out->body_distance_error = in->body_distance_error * lambda;
    out->speed_error         = in->speed_error * lambda_speed;
    out->yaw_error           = in->yaw_error * lambda_yaw;
    out->d_yaw               = in->d_yaw * lambda_yaw;
    out->lambda              = lambda;
}
