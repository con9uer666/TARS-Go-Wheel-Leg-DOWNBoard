#include "PowerCtrl.h"
#include "main.h"
#include "math.h"
#include <stddef.h>
#include "motor.h"
#include "observe_task.h"
#include "Judge.h"
#include "wattmeter.h"

extern float powerPredict;

/* ------------------------------- 全局对象与开关 ------------------------------- */

ChassisPower whell_power;

uint8_t g_power_ctrl_enable = 1; // 总开关：1=启用功率控制，0=旁路

float g_power_obs_lambda = 1.0f; // 当前门控系数，供上位机观测

/* ================================ 调参区 ================================
 *
 * 功率上限随缓冲能量的映射：
 *   缓冲 <= power_buffer_min  →  power_limit = power_limit_min（最保守）
 *   缓冲 >= power_buffer_max  →  power_limit = 裁判上限（全力）
 *   中间线性插值
 *
 * lambda 是目标速度的缩放系数（0~1），越小限速越猛。
 * 纯 PI 控制 lambda，目标是让 measured_power 跟随 power_limit。
 *
 * ======================================================================= */

float power_buffer_min = 30.0f;  // 缓冲低阈值（J）
float power_buffer_max = 60.0f;  // 缓冲高阈值（J）
float power_limit_min  = 10.0f;  // 功率上限最低值（W）

float obs_lambda_min       = 0.15f;
float obs_lambda_kp        = 0.007f;
float obs_lambda_ki        = 0.001f;
float obs_lambda_rise_rate = 1.0f;
float obs_lambda_fall_rate = 1.0f;

/* ======================================================================= */

static const float POWER_CTRL_DT_S          = 0.002f;
static const float POWER_MEASURE_VALID_MIN_W = 1.0f;
static const float POWER_MEASURE_VALID_MAX_W = 500.0f;

static float g_last_measured_power = 0.0f;
static float g_lambda_integral     = 0.0f;

/* ------------------------------- 工具函数 ------------------------------- */

static float PowerCtrl_Clampf(float x, float lo, float hi)
{
	if (x < lo) return lo;
	if (x > hi) return hi;
	return x;
}

/* ------------------------------- 初始化 ------------------------------- */

void PowerCtralInit(ChassisPower *whell_power_out)
{
	whell_power_out->toque_coefficient = 2.4324e-6f;
	whell_power_out->moto_type         = 3508;
	whell_power_out->UserPowerLimit    = 120;

	g_power_obs_lambda    = 1.0f;
	g_lambda_integral     = 0.0f;
	g_last_measured_power = 0.0f;
}

/* ------------------------------- 内部函数 ------------------------------- */

static float PowerCtrl_GetPowerLimit(void)
{
	float limit = (float)JUDGE_GetChassisPowerLimit();
	if ((limit < 35.0f) || (limit > 300.0f))
	{
		limit = (float)whell_power.UserPowerLimit;
	}
	return limit;
}

static float PowerCtrl_GetMeasuredPower(void)
{
	float p = wattmeter_data.power;
	if ((p < POWER_MEASURE_VALID_MIN_W) || (p > POWER_MEASURE_VALID_MAX_W))
	{
		return -1.0f;
	}
	return p;
}

static float PowerCtrl_ComputePowerLimit(float buffer, float base_limit)
{
	float ratio = (buffer - power_buffer_min) / (power_buffer_max - power_buffer_min);
	ratio = PowerCtrl_Clampf(ratio, 0.0f, 1.0f);
	return power_limit_min + ratio * (base_limit - power_limit_min);
}

/* ------------------------------- 目标速度限幅 ------------------------------- */

float PowerCtrl_LimitTargetSpeed(float raw_target_speed, float speed_limit)
{
	if (g_power_ctrl_enable == 0)
		return raw_target_speed;

	float limited = raw_target_speed * g_power_obs_lambda;
	return PowerCtrl_Clampf(limited, -speed_limit, speed_limit);
}

/* ------------------------------- 主控制函数 ------------------------------- */
float power_buffer;
float power_limit;
float base_power_limit;

void PowerCtrl(void)
{
	float measured_power;
	float lambda_target;
	float delta;
	float max_step;

	if (g_power_ctrl_enable == 0)
	{
		g_power_obs_lambda = 1.0f;
		g_lambda_integral  = 0.0f;
		return;
	}

	// 步骤1：读取功率计
	measured_power = PowerCtrl_GetMeasuredPower();
	whell_power.MeasurePower = measured_power;
	if (measured_power >= POWER_MEASURE_VALID_MIN_W)
	{
		g_last_measured_power = measured_power;
	}
	whell_power.PredictPower = g_last_measured_power;
	powerPredict             = g_last_measured_power;

	// 步骤2：读取裁判上限和缓冲
	if (PowerCtrl_GetPowerLimit() >= 45)
	{
		base_power_limit = PowerCtrl_GetPowerLimit();
	}
	else
	{
		base_power_limit = 60;
	}

	power_buffer = (float)JUDGE_GetPowerBuffer();

	// 步骤3：计算功率上限
	power_limit = PowerCtrl_ComputePowerLimit(power_buffer, base_power_limit);
	whell_power.MaxPowerLimit = (uint16_t)base_power_limit;
	whell_power.InputPower    = power_limit;

	// 步骤4：PI 控制 lambda
	if ((g_last_measured_power > power_limit) && (g_last_measured_power > POWER_MEASURE_VALID_MIN_W))
	{
		float error = g_last_measured_power - power_limit;
		g_lambda_integral += error * POWER_CTRL_DT_S;
		g_lambda_integral = PowerCtrl_Clampf(g_lambda_integral, 0.0f, 0.5f / obs_lambda_ki);
		float correction = obs_lambda_kp * error + obs_lambda_ki * g_lambda_integral;
		lambda_target = PowerCtrl_Clampf(1.0f - correction, obs_lambda_min, 1.0f);
	}
	else
	{
		g_lambda_integral = 0.0f;
		lambda_target     = 1.0f;
	}

	// 斜坡限制
	delta    = lambda_target - g_power_obs_lambda;
	max_step = (delta < 0.0f) ? obs_lambda_fall_rate * POWER_CTRL_DT_S
	                          : obs_lambda_rise_rate * POWER_CTRL_DT_S;
	g_power_obs_lambda = PowerCtrl_Clampf(
		g_power_obs_lambda + PowerCtrl_Clampf(delta, -max_step, max_step),
		obs_lambda_min, 1.0f);
}
