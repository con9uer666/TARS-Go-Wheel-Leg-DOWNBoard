#include "PowerObserverLimit.h"

#include <cmath>
#include <cstddef>

namespace {

float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

float maxf(float a, float b)
{
    return (a > b) ? a : b;
}

} // anonymous namespace

void PowerObsCtrl_DefaultParam(PowerObsCtrlParam *param)
{
    if (param == nullptr)
        return;

    param->buffer_min = 10.0f;
    param->buffer_ref = 22.0f;
    param->buffer_max = 70.0f;

    param->lambda_min = 0.12f;
    param->lambda_rise_rate = 7.0f;
    param->lambda_fall_rate = 10.0f;

    param->power_margin = 2.5f;
    param->buffer_gain = 0.15f;
}

void PowerObsCtrl_Init(PowerObsCtrl *ctrl, const PowerObsCtrlParam *param)
{
    PowerObsCtrlParam fallback;

    if (ctrl == nullptr)
        return;

    if (param == nullptr)
    {
        PowerObsCtrl_DefaultParam(&fallback);
        param = &fallback;
    }

    ctrl->param = *param;
    ctrl->lambda = 1.0f;
    ctrl->predicted_power = 0.0f;
    ctrl->allowed_power = 0.0f;
    ctrl->last_predicted_power = 0.0f;
}

float PowerObsCtrl_ComputeLambda(PowerObsCtrl *ctrl,
                                 float power_limit,
                                 float power_buffer,
                                 float predicted_power,
                                 float dt_s)
{
    constexpr float eps = 1e-6f;

    if (ctrl == nullptr)
        return 1.0f;

    dt_s = maxf(dt_s, 0.0f);
    power_limit = maxf(power_limit, 1.0f);
    predicted_power = maxf(predicted_power, 0.0f);

    float raw_limit = power_limit - ctrl->param.power_margin;
    raw_limit = maxf(raw_limit, 1.0f);

    float allowed = raw_limit + ctrl->param.buffer_gain * (power_buffer - ctrl->param.buffer_ref);
    allowed = clampf(allowed, 1.0f, power_limit + ctrl->param.power_margin);

    float dynamic_lambda_min = ctrl->param.lambda_min;
    float buffer_lambda;

    if (power_buffer <= ctrl->param.buffer_min)
    {
        float ratio = clampf(power_buffer / maxf(ctrl->param.buffer_min, eps), 0.0f, 1.0f);
        dynamic_lambda_min = 0.05f + (ctrl->param.lambda_min - 0.05f) * ratio;
        buffer_lambda = dynamic_lambda_min;
    }
    else if (power_buffer >= ctrl->param.buffer_max)
    {
        buffer_lambda = 1.0f;
    }
    else
    {
        float span = ctrl->param.buffer_max - ctrl->param.buffer_min;
        float ratio = (power_buffer - ctrl->param.buffer_min) / maxf(span, eps);
        buffer_lambda = ctrl->param.lambda_min + (1.0f - ctrl->param.lambda_min) * ratio;
    }

    float power_lambda;
    if (predicted_power <= allowed)
    {
        power_lambda = 1.0f;
    }
    else
    {
        power_lambda = std::pow((allowed + eps) / (predicted_power + eps), 0.65f);
    }

    float target_lambda = buffer_lambda;
    if (power_lambda < target_lambda)
        target_lambda = power_lambda;

    target_lambda = clampf(target_lambda, dynamic_lambda_min, 1.0f);

    float delta = target_lambda - ctrl->lambda;
    float max_step;
    if (delta >= 0.0f)
        max_step = ctrl->param.lambda_rise_rate * dt_s;
    else
        max_step = ctrl->param.lambda_fall_rate * dt_s;

    max_step = maxf(max_step, 0.0f);
    delta = clampf(delta, -max_step, max_step);

    ctrl->lambda = clampf(ctrl->lambda + delta, dynamic_lambda_min, 1.0f);
    ctrl->last_predicted_power = ctrl->predicted_power;
    ctrl->predicted_power = predicted_power;
    ctrl->allowed_power = allowed;

    return ctrl->lambda;
}

void PowerObsCtrl_Apply(const PowerObsCtrl *ctrl,
                        const PowerObsInput *in,
                        PowerObsOutput *out)
{
    if (!in || !out)
        return;

    float lambda = (ctrl == nullptr) ? 1.0f : ctrl->lambda;
    lambda = clampf(lambda, 0.0f, 1.0f);

    float lambda_speed;
    if (lambda < 0.45f)
        lambda_speed = lambda * lambda;
    else
        lambda_speed = std::pow(lambda, 1.2f);

    float lambda_yaw = std::pow(lambda, 1.1f);

    out->body_distance_error = in->body_distance_error * lambda;
    out->speed_error = in->speed_error * lambda_speed;
    out->yaw_error = in->yaw_error * lambda_yaw;
    out->d_yaw = in->d_yaw * lambda_yaw;

    out->lambda = lambda;
}
