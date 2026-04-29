#include "PowerObserverLimit.h"

#include <math.h>
#include <stddef.h>

/*
 * @brief 浮点限幅函数。
 * @param x 输入值。
 * @param lo 下限。
 * @param hi 上限。
 * @return 限幅后的结果。
 */
static float clampf(float x, float lo, float hi)
{
    if (x < lo)
    {
        return lo;
    }
    if (x > hi)
    {
        return hi;
    }
    return x;
}

/*
 * @brief 返回两个浮点数中的较大值。
 * @param a 输入值 a。
 * @param b 输入值 b。
 * @return max(a, b)。
 */
static float maxf(float a, float b)
{
    return (a > b) ? a : b;
}

/*
 * @brief 生成默认门控参数。
 * @param param 输出参数结构体指针。
 * @note 参数含义：
 * 1) buffer_min/ref/max 控制缓冲能量对 lambda 的映射区间；
 * 2) lambda_min 控制最低缩放强度；
 * 3) rise/fall_rate 控制 lambda 的变化速度；
 * 4) power_margin/buffer_gain 控制 allowed_power 的保守程度。
 */
void PowerObsCtrl_DefaultParam(PowerObsCtrlParam *param)
{
    if (param == NULL)
    {
        return;
    }

    param->buffer_min = 10.0f;
    param->buffer_ref = 22.0f;
    param->buffer_max = 70.0f;

    param->lambda_min = 0.12f;
    param->lambda_rise_rate = 7.0f;
    param->lambda_fall_rate = 10.0f;

    param->power_margin = 2.5f;
    param->buffer_gain = 0.15f;
}

/*
 * @brief 初始化观测门控控制器。
 * @param ctrl 控制器对象指针。
 * @param param 参数对象指针；传 NULL 时自动使用默认参数。
 */
void PowerObsCtrl_Init(PowerObsCtrl *ctrl, const PowerObsCtrlParam *param)
{
    /* fallback 用于承接默认参数，避免上层必须显式传参。 */
    PowerObsCtrlParam fallback;

    if (ctrl == NULL)
    {
        return;
    }

    if (param == NULL)
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

/*
 * @brief 计算并更新观测量缩放系数 lambda。
 * @param ctrl 控制器对象指针。
 * @param power_limit 本周期可用功率上限（W）。
 * @param power_buffer 本周期缓冲能量（J）。
 * @param predicted_power 本周期预测总电功率（W）。
 * @param dt_s 控制周期（s）。
 * @return 本周期更新后的 lambda。
 */
float PowerObsCtrl_ComputeLambda(PowerObsCtrl *ctrl,
                                 float power_limit,
                                 float power_buffer,
                                 float predicted_power,
                                 float dt_s)
{
    /* 防止除零的极小常数。 */
    const float eps = 1e-6f;

    /* 可用功率计算中间量。 */
    float raw_limit;
    float allowed;

    /* 两条约束路径得到的 lambda。 */
    float buffer_lambda;
    float power_lambda;
    float target_lambda;
    float dynamic_lambda_min;

    /* 斜率限制相关变量。 */
    float delta;
    float max_step;

    if (ctrl == NULL)
    {
        return 1.0f;
    }

    /* 步骤1：输入清洗，保证后续计算稳定。 */
    dt_s = maxf(dt_s, 0.0f);
    power_limit = maxf(power_limit, 1.0f);
    predicted_power = maxf(predicted_power, 0.0f);

    /* 步骤2：扣除安全裕量，得到基础可用功率。 */
    raw_limit = power_limit - ctrl->param.power_margin;
    raw_limit = maxf(raw_limit, 1.0f);

    /* 步骤3：根据缓冲高低动态修正 allowed_power。 */
    allowed = raw_limit + ctrl->param.buffer_gain * (power_buffer - ctrl->param.buffer_ref);
    allowed = clampf(allowed, 1.0f, power_limit + ctrl->param.power_margin);

    /* 步骤4：缓冲路径，缓冲越低 lambda 越小。 */
    dynamic_lambda_min = ctrl->param.lambda_min;
    if (power_buffer <= ctrl->param.buffer_min)
    {
        /* 缓冲跌入低区时，进一步下探 lambda 下限，增强保缓冲能力。 */
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
        /* 在 [buffer_min, buffer_max] 之间线性插值。 */
        float span = ctrl->param.buffer_max - ctrl->param.buffer_min;
        float ratio = (power_buffer - ctrl->param.buffer_min) / maxf(span, eps);
        buffer_lambda = ctrl->param.lambda_min + (1.0f - ctrl->param.lambda_min) * ratio;
    }

    /* 步骤5：功率路径，超限时按幂函数收缩。 */
    if (predicted_power <= allowed)
    {
        power_lambda = 1.0f;
    }
    else
    {
        power_lambda = powf((allowed + eps) / (predicted_power + eps), 0.65f);
    }

    /* 步骤6：缓冲约束与功率约束取更严格者。 */
    target_lambda = buffer_lambda;
    if (power_lambda < target_lambda)
    {
        target_lambda = power_lambda;
    }

    /* 步骤7：目标 lambda 再做上下限保护。 */
    target_lambda = clampf(target_lambda, dynamic_lambda_min, 1.0f);

    /* 步骤8：计算本周期变化量。 */
    delta = target_lambda - ctrl->lambda;

    /* 步骤9：上升/下降采用不同斜率（下降更快，保护优先）。 */
    if (delta >= 0.0f)
    {
        max_step = ctrl->param.lambda_rise_rate * dt_s;
    }
    else
    {
        max_step = ctrl->param.lambda_fall_rate * dt_s;
    }

    /* 步骤10：斜率限幅，抑制突变。 */
    max_step = maxf(max_step, 0.0f);
    delta = clampf(delta, -max_step, max_step);

    /* 步骤11：更新状态缓存并返回新 lambda。 */
    ctrl->lambda = clampf(ctrl->lambda + delta, dynamic_lambda_min, 1.0f);
    ctrl->last_predicted_power = ctrl->predicted_power;
    ctrl->predicted_power = predicted_power;
    ctrl->allowed_power = allowed;

    return ctrl->lambda;
}

/*
 * @brief 对观测量应用门控缩放。
 * @param ctrl 控制器指针。
 * @param in 门控前观测量。
 * @param out 门控后观测量。
 */
void PowerObsCtrl_Apply(const PowerObsCtrl *ctrl,
                        const PowerObsInput *in,
                        PowerObsOutput *out)
{
    /* 本周期实际使用的缩放系数。 */
    float lambda;
    float lambda_speed;
    float lambda_yaw;

    /* 入参合法性检查。 */
    if ((in == NULL) || (out == NULL))
    {
        return;
    }

    /* 步骤1：允许 ctrl 为空，便于调试时一键旁路。 */
    lambda = (ctrl == NULL) ? 1.0f : ctrl->lambda;
    lambda = clampf(lambda, 0.0f, 1.0f);

    /*
     * 对速度/偏航通道做更强抑制：
     * - 全速前进时主导功率的是速度误差项，应比姿态相关项更快收缩。
     */
    /* 步骤2：根据整体 lambda 生成分量缩放权重。 */
    if (lambda < 0.45f)
    {
        lambda_speed = lambda * lambda;
    }
    else
    {
        lambda_speed = powf(lambda, 1.2f);
    }

    lambda_yaw = powf(lambda, 1.1f);

    /* 步骤3：逐项缩放观测量。 */
    out->body_distance_error = in->body_distance_error * lambda;
    out->speed_error = in->speed_error * lambda_speed;
    out->yaw_error = in->yaw_error * lambda_yaw;
    out->d_yaw = in->d_yaw * lambda_yaw;

    /* 步骤4：输出本周期 lambda 便于上位机调试。 */
    out->lambda = lambda;
}
