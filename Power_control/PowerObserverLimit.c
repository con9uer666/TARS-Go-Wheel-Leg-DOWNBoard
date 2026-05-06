#include "PowerObserverLimit.h"
#include <math.h>
#include <stddef.h>

/* ======================================================================= */

static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

void PowerObsCtrl_Reset(PowerObsCtrl *ctrl)
{
    if (ctrl == NULL) return;
    ctrl->lambda   = 1.0f;
    ctrl->integral = 0.0f;
}

float PowerObsCtrl_ComputeLambda(PowerObsCtrl *ctrl,
                                 float power_limit,
                                 float measured_power,
                                 float dt_s)
{
    float lambda_target;
    float delta;
    float max_step;

    if (ctrl == NULL) return 1.0f;

    if ((measured_power > power_limit) && (measured_power > 1.0f))
    {
        float error = measured_power - power_limit;

        ctrl->integral += error * dt_s;
        // 积分限幅，防止饱和（最多让 lambda 额外下降 0.5）
        ctrl->integral = clampf(ctrl->integral, 0.0f, 0.5f / obs_lambda_ki);

        float correction = obs_lambda_kp * error + obs_lambda_ki * ctrl->integral;
        lambda_target = clampf(1.0f - correction, obs_lambda_min, 1.0f);
    }
    else
    {
        ctrl->integral = 0.0f;
        lambda_target  = 1.0f;
    }

    // 斜坡限制：下降用 fall_rate，上升用 rise_rate
    delta    = lambda_target - ctrl->lambda;
    max_step = (delta < 0.0f) ? obs_lambda_fall_rate * dt_s
                              : obs_lambda_rise_rate * dt_s;
    ctrl->lambda = clampf(ctrl->lambda + clampf(delta, -max_step, max_step),
                          obs_lambda_min, 1.0f);

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
