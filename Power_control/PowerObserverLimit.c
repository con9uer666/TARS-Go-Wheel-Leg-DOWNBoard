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
                                 float dt_s,
                                 float kp,
                                 float ki,
                                 float alpha_fall,
                                 float alpha_rise,
                                 float lambda_min)
{
    float lambda_target;

    if (ctrl == NULL) return 1.0f;

    if ((measured_power > power_limit) && (measured_power > 1.0f))
    {
        float error = measured_power - power_limit;

        ctrl->integral += error * dt_s;
        ctrl->integral = clampf(ctrl->integral, 0.0f, 0.5f / ki);

        float correction = kp * error + ki * ctrl->integral;
        lambda_target = clampf(1.0f - correction, lambda_min, 1.0f);
    }
    else
    {
        ctrl->integral = 0.0f;
        lambda_target  = 1.0f;
    }

    // 低通滤波
    float alpha = (lambda_target < ctrl->lambda) ? alpha_fall : alpha_rise;
    ctrl->lambda = clampf(alpha * lambda_target + (1.0f - alpha) * ctrl->lambda,
                          lambda_min, 1.0f);

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
