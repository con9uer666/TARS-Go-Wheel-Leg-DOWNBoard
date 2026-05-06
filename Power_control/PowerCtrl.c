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
 * 按裁判功率档位（45~100W，步长5）配置参数，共12档。
 * 索引0对应45W，索引11对应100W。
 *
 * 不随档位变化的参数：
 *   power_limit_min  — 功率上限最低值（W）
 *   obs_lambda_min   — lambda 下限
 *
 * ======================================================================= */

#define POWER_LEVEL_COUNT 12
#define POWER_LEVEL_BASE  45
#define POWER_LEVEL_STEP  5

typedef struct
{
    float power_buffer_min;   // 缓冲低阈值（J）
    float power_buffer_max;   // 缓冲高阈值（J）
    float obs_lambda_kp;
    float obs_lambda_ki;
    float obs_lambda_alpha_fall;
    float obs_lambda_alpha_rise;
} PowerLevelParam;

// 索引 0=45W, 1=50W, ..., 11=100W
static PowerLevelParam g_power_level_params[POWER_LEVEL_COUNT] = {
    /* 45W  */ { 30.0f, 60.0f, 0.007f, 0.001f, 0.01f, 0.01f },
    /* 50W  */ { 30.0f, 60.0f, 0.007f, 0.001f, 0.01f, 0.01f },
    /* 55W  */ { 30.0f, 60.0f, 0.007f, 0.001f, 0.01f, 0.01f },
    /* 60W  */ { 30.0f, 60.0f, 0.007f, 0.001f, 0.01f, 0.01f },
    /* 65W  */ { 30.0f, 60.0f, 0.007f, 0.001f, 0.01f, 0.01f },
    /* 70W  */ { 30.0f, 60.0f, 0.007f, 0.001f, 0.01f, 0.01f },
    /* 75W  */ { 30.0f, 60.0f, 0.007f, 0.001f, 0.01f, 0.01f },
    /* 80W  */ { 30.0f, 60.0f, 0.007f, 0.001f, 0.01f, 0.01f },
    /* 85W  */ { 30.0f, 60.0f, 0.007f, 0.001f, 0.01f, 0.01f },
    /* 90W  */ { 30.0f, 60.0f, 0.007f, 0.001f, 0.01f, 0.01f },
    /* 95W  */ { 30.0f, 60.0f, 0.007f, 0.001f, 0.01f, 0.01f },
    /* 100W */ { 30.0f, 60.0f, 0.007f, 0.001f, 0.01f, 0.01f },
};

float power_limit_min = 10.0f;  // 功率上限最低值（W），不随档位变化
float obs_lambda_min  = 0.15f;  // lambda 下限，不随档位变化

// 当前档位参数指针，由 PowerCtrl_SelectLevelParam() 更新
static const PowerLevelParam *g_cur_param = &g_power_level_params[0];

/* ======================================================================= */

static void PowerCtrl_SelectLevelParam(float base_limit)
{
    int idx = (int)((base_limit - POWER_LEVEL_BASE) / POWER_LEVEL_STEP + 0.5f);
    if (idx < 0) idx = 0;
    if (idx >= POWER_LEVEL_COUNT) idx = POWER_LEVEL_COUNT - 1;
    g_cur_param = &g_power_level_params[idx];
}

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
	float ratio = (buffer - g_cur_param->power_buffer_min) / (g_cur_param->power_buffer_max - g_cur_param->power_buffer_min);
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
		base_power_limit = 45;
	}

	power_buffer = (float)JUDGE_GetPowerBuffer();

	// 步骤2.5：根据档位选参数
	PowerCtrl_SelectLevelParam(base_power_limit);

	// 步骤3：计算功率上限
	power_limit = PowerCtrl_ComputePowerLimit(power_buffer, base_power_limit);
	whell_power.MaxPowerLimit = (uint16_t)base_power_limit;
	whell_power.InputPower    = power_limit;

	// 步骤4：PI 控制 lambda
	if ((g_last_measured_power > power_limit) && (g_last_measured_power > POWER_MEASURE_VALID_MIN_W))
	{
		float error = g_last_measured_power - power_limit;
		g_lambda_integral += error * POWER_CTRL_DT_S;
		g_lambda_integral = PowerCtrl_Clampf(g_lambda_integral, 0.0f, 0.5f / g_cur_param->obs_lambda_ki);
		float correction = g_cur_param->obs_lambda_kp * error + g_cur_param->obs_lambda_ki * g_lambda_integral;
		lambda_target = PowerCtrl_Clampf(1.0f - correction, obs_lambda_min, 1.0f);
	}
	else
	{
		g_lambda_integral = 0.0f;
		lambda_target     = 1.0f;
	}

	// 低通滤波
	float alpha = (lambda_target < g_power_obs_lambda) ? g_cur_param->obs_lambda_alpha_fall : g_cur_param->obs_lambda_alpha_rise;
	g_power_obs_lambda = PowerCtrl_Clampf(
		alpha * lambda_target + (1.0f - alpha) * g_power_obs_lambda,
		obs_lambda_min, 1.0f);
}
