/**
 * @file user_pid.c
 * @brief 通用 PID 控制器实现。
 *
 * 提供单轴 PID 控制，支持角度误差归一化、积分死区、
 * 积分变化率限制（I_step）、输出死区等功能。
 */

#include "user_pid.h"
#include "arm_math.h"

/**
 * @brief 初始化 PID 结构体参数。
 *
 * 设置增益、限幅、死区和积分步长，并将积分项清零。
 *
 * @param[in,out] PID     PID 结构体指针。
 * @param[in] Kp          比例增益。
 * @param[in] Ki          积分增益。
 * @param[in] Kd          微分增益。
 * @param[in] out_limit   输出限幅（绝对值）。
 * @param[in] i_limit     积分项限幅（绝对值）。
 * @param[in] I_step      积分变化率限制（每次调用最大增量，<=0 时无限制）。
 * @param[in] Integraldead_zone 积分累加死区，误差超出该范围时停止积分。
 * @param[in] deadzone    输出死区，误差低于该阈值时输出归零。
 */
void PID_INIT(user_pid_t *PID, float Kp, float Ki, float Kd, float out_limit,
              float i_limit, float I_step, float Integraldead_zone, float deadzone)
{
    PID->Kp = Kp;
    PID->Ki = Ki;
    PID->Kd = Kd;
    PID->out_limit = out_limit;
    PID->I_limit = i_limit;
    PID->I_step = I_step;
    PID->Integraldead_zone = Integraldead_zone;
    PID->deadzone = deadzone;

    PID->I = 0;
}

/**
 * @brief 重置 PID 输出限幅。
 *
 * 在线修改输出限幅值，不影响积分及其他状态。
 *
 * @param[in,out] PID       PID 结构体指针。
 * @param[in] new_limit     新的输出限幅（绝对值）。
 */
void PID_Reset_OutLimit(user_pid_t *PID, float new_limit)
{
    PID->out_limit = new_limit;
}

/**
 * @brief 清零 PID 积分项。
 *
 * 通常在切换控制模式或滞后补偿重置时调用。
 *
 * @param[in,out] PID PID 结构体指针。
 */
void PID_Clear(user_pid_t *PID)
{
    PID->I = 0;
}

/**
 * @brief 计算线性误差（目标 - 当前）。
 *
 * 注意：角度控制应使用 PID_Set_AngleError()，该函数会进行角度归一化。
 *
 * @param[in,out] PID   PID 结构体指针。
 * @param[in] now       当前测量值。
 * @param[in] target    目标值。
 */
void PID_Set_Error(user_pid_t *PID, float now, float target)
{
    PID->error = target - now;
}

/**
 * @brief 将角度归一化到 [-π, π] 区间。
 * @param[in] angle 输入角度，单位 rad。
 * @return 归一化后的角度，范围 [-π, π]。
 */
static float NormalizeAngle(float angle)
{
    angle = fmodf(angle, 2.0f * PI);
    if (angle > PI) {
        angle -= 2.0f * PI;
    } else if (angle < -PI) {
        angle += 2.0f * PI;
    }
    return angle;
}

/**
 * @brief 计算从当前角度到目标角度的最小差值（最短路径）。
 *
 * @param[in] target_angle  目标角度，单位 rad。
 * @param[in] current_angle 当前角度，单位 rad。
 * @return 最小转角差值，范围 [-π, π]。
 */
float ShortestAngleDelta(float target_angle, float current_angle)
{
    target_angle = NormalizeAngle(target_angle);
    current_angle = NormalizeAngle(current_angle);
    float delta = target_angle - current_angle;
    return NormalizeAngle(delta);
}

/**
 * @brief 角度控制专用的误差设置函数。
 *
 * 自动计算目标角度与当前角度之间的最短路径差值作为误差，
 * 避免角度环绕问题（如 350°→10° 按 -20° 而非 +340°）。
 *
 * @param[in,out] PID          PID 结构体指针。
 * @param[in] current_angle   当前角度，单位 rad。
 * @param[in] target_angle    目标角度，单位 rad。
 */
void PID_Set_AngleError(user_pid_t *PID, float current_angle, float target_angle)
{
    PID->error = ShortestAngleDelta(target_angle, current_angle);
}

/**
 * @brief 执行 PID 计算，更新积分/微分项并返回控制量。
 *
 * 对于每个计算周期，完成以下处理流程：
 *   1) 输出死区：误差绝对值 <= deadzone 时误差归零，否则从误差中扣除死区。
 *   2) 比例和微分项：P = Kp·error, D = Kd·(error - pre_error)。
 *   3) 积分死区：误差绝对值 <= Integraldead_zone 时才累加积分；
 *      若 I_step > 0 则对积分增量进行变化率限制。
 *   4) 积分限幅：[-I_limit, +I_limit]。
 *   5) 输出限幅：[-out_limit, +out_limit]。
 *
 * @note 仅负责基于已设定的 error 计算输出，不修改 error。
 *
 * @param[in,out] PID PID 结构体指针。
 * @return 控制输出值。
 */
float PID_coculate(user_pid_t *PID)
{
    // 输出死区处理
    if ((PID->error <= PID->deadzone) && (PID->error >= -PID->deadzone)) {
        PID->error = 0;
    } else {
        if (PID->error >= 0) PID->error -= PID->deadzone;
        if (PID->error <= 0) PID->error += PID->deadzone;
    }

    // 基本计算
    float P = PID->Kp * PID->error;
    float D = PID->Kd * (PID->error - PID->pre_error);

    // 积分死区 + 积分变化率限制
    if (PID->error <= PID->Integraldead_zone && PID->error >= -PID->Integraldead_zone) {
        float I_target = PID->I + PID->Ki * PID->error;

        if (PID->I_step <= 0.0f) {
            PID->I = I_target;
        } else {
            float delta = I_target - PID->I;
            if (delta > PID->I_step)
                delta = PID->I_step;
            else if (delta < -PID->I_step)
                delta = -PID->I_step;
            PID->I += delta;
        }
    }

    // 积分限幅
    if (PID->I >= PID->I_limit || PID->I <= -PID->I_limit) {
        PID->I = (PID->I > 0) ? PID->I_limit : -PID->I_limit;
    }

    float out = P + PID->I + D;

    // 输出限幅
    if (out >= PID->out_limit || out <= -PID->out_limit) {
        out = (out > 0) ? PID->out_limit : -PID->out_limit;
    }

    PID->pre_error = PID->error;
    PID->output = out;
    return out;
}