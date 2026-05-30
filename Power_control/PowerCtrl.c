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

// lambda 是目标速度/转速的缩放系数（0~1），由 PID 输出后经低通滤波得到。
// 仅在 g_filtered_power > power_limit 时生效，否则不限制。
float g_power_obs_lambda = 1.0f;

/* ================================ 调参区 ================================
 *
 * 控制链路：
 *   功率计 → 低通滤波(power_measure_alpha) → g_filtered_power
 *   缓冲能量 → ComputePowerLimit → power_limit（本周期允许的最大功率）
 *   (g_filtered_power - power_limit) → PID → lambda_target
 *   lambda_target → 低通滤波(alpha_fall/rise) → g_power_obs_lambda
 *   g_power_obs_lambda × 目标速度/转速（仅超功率时生效）
 *
 * 按裁判功率档位（45~100W，步长5）分档配置，共12档。
 * 索引0对应45W，索引11对应100W。
 *
 * 不随档位变化的参数：
 *   power_limit_min  — 缓冲耗尽时的功率下限（W）
 *   obs_lambda_min   — lambda 硬下限，防止完全停止
 *   power_measure_alpha — 功率计一阶低通系数，越小越平滑但延迟越大
 *
 * ======================================================================= */

#define POWER_LEVEL_COUNT 12
#define POWER_LEVEL_BASE  45
#define POWER_LEVEL_STEP  5

typedef struct
{
    float power_buffer_min;      // 缓冲低阈值（J）：低于此值 power_limit 降至 power_limit_min
    float power_buffer_max;      // 缓冲高阈值（J）：高于此值 power_limit 等于裁判上限
    float obs_lambda_kp;         // PID 比例系数
    float obs_lambda_ki;         // PID 积分系数
    float obs_lambda_kd;         // PID 微分系数
    float obs_lambda_alpha_fall; // lambda 下降时的低通系数（越小越慢，响应越柔和）
    float obs_lambda_alpha_rise; // lambda 上升时的低通系数（越小越慢，恢复越柔和）
} PowerLevelParam;

// 索引 0=45W, 1=50W, ..., 11=100W
// 列顺序：buffer_min, buffer_max, kp, ki, kd, alpha_fall, alpha_rise
static PowerLevelParam g_power_level_params[POWER_LEVEL_COUNT] = {
    /* 45W  */ { 30.0f, 60.0f, 0.01f,  0.000003f, 0.01f, 0.01f, 0.01f },
    /* 50W  */ { 30.0f, 60.0f, 0.009f, 0.000003f, 0.01f, 0.01f, 0.01f },
    /* 55W  */ { 30.0f, 60.0f, 0.008f, 0.000003f, 0.01f, 0.01f, 0.01f },
    /* 60W  */ { 40.0f, 60.0f, 5.0f, 0.1f, 0.5f, 0.01f, 0.01f },//最好的基准
    /* 65W  */ { 40.0f, 60.0f, 4.0f, 0.1f, 0.4f, 0.01f, 0.01f },
    /* 70W  */ { 40.0f, 60.0f, 3.0f, 0.1f, 0.3f, 0.01f, 0.01f },
    /* 75W  */ { 40.0f, 60.0f, 2.0f, 0.1f, 0.2f, 0.01f, 0.01f },
    /* 80W  */ { 40.0f, 60.0f, 1.0f, 0.1f, 0.1f, 0.01f, 0.01f },//基准
    /* 85W  */ { 30.0f, 60.0f, 1.0f, 0.01f, 0.1f, 0.01f, 0.01f },
    /* 90W  */ { 30.0f, 60.0f, 1.0f, 0.01f, 0.1f, 0.01f, 0.01f },
    /* 95W  */ { 30.0f, 60.0f, 1.0f, 0.01f, 0.1f, 0.01f, 0.01f },
    /* 100W */ { 30.0f, 60.0f, 1.0f, 0.01f, 0.1f, 0.01f, 0.01f },//基准
};

float power_limit_min     = 10.0f; // 缓冲耗尽时的功率下限（W）
float obs_lambda_min      = 0.15f; // lambda 硬下限，防止完全停止
float power_measure_alpha = 0.3f;  // 功率计一阶低通系数（越小越平滑）

// 当前档位参数指针，每帧由 PowerCtrl_SelectLevelParam() 根据裁判上限更新
static const PowerLevelParam *g_cur_param = &g_power_level_params[0];

/* ======================================================================= */

static const float POWER_CTRL_DT_S          = 0.002f;  // 控制周期（s）
static const float POWER_MEASURE_VALID_MIN_W = 1.0f;   // 功率计有效读数下限
static const float POWER_MEASURE_VALID_MAX_W = 500.0f; // 功率计有效读数上限

float         g_filtered_power    = 0.0f; // 功率计低通滤波后的值，用于 PID 输入
static float  g_lambda_integral   = 0.0f; // PID 积分项
static float  g_lambda_prev_error = 0.0f; // 上一帧误差，用于微分项

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

	g_power_obs_lambda  = 1.0f;
	g_lambda_integral   = 0.0f;
	g_lambda_prev_error = 0.0f;
	g_filtered_power    = 0.0f;
}

/* ------------------------------- 内部函数 ------------------------------- */

// 读取裁判系统功率上限，异常时回退到用户设定值
static float PowerCtrl_GetPowerLimit(void)
{
	float limit = (float)JUDGE_GetChassisPowerLimit();
	if ((limit < 35.0f) || (limit > 300.0f))
	{
		limit = (float)whell_power.UserPowerLimit;
	}
	return limit;
}

// 读取功率计，范围外返回 -1 表示无效
static float PowerCtrl_GetMeasuredPower(void)
{
	float p = wattmeter_data.power;
	if ((p < POWER_MEASURE_VALID_MIN_W) || (p > POWER_MEASURE_VALID_MAX_W))
	{
		return -1.0f;
	}
	return p;
}

// 根据缓冲能量在 [power_limit_min, base_limit] 之间线性插值得到本周期功率上限
static float PowerCtrl_ComputePowerLimit(float buffer, float base_limit)
{
	float ratio = (buffer - g_cur_param->power_buffer_min) / (g_cur_param->power_buffer_max - g_cur_param->power_buffer_min);
	ratio = PowerCtrl_Clampf(ratio, 0.0f, 1.0f);
	return power_limit_min + ratio * (base_limit - power_limit_min);
}

// 根据裁判功率档位选取对应参数组
static void PowerCtrl_SelectLevelParam(float base_limit)
{
	int idx = (int)((base_limit - POWER_LEVEL_BASE) / POWER_LEVEL_STEP + 0.5f);
	if (idx < 0) idx = 0;
	if (idx >= POWER_LEVEL_COUNT) idx = POWER_LEVEL_COUNT - 1;
	g_cur_param = &g_power_level_params[idx];
}

/* ------------------------------- 目标速度限幅 ------------------------------- */

// 仅在 g_filtered_power > power_limit 时用 lambda 缩放目标速度，否则不限制
float PowerCtrl_LimitTargetSpeed(float raw_target_speed, float speed_limit)
{
	if (g_power_ctrl_enable == 0)
		return raw_target_speed;

	float limited = (g_filtered_power > power_limit)
	              ? raw_target_speed * g_power_obs_lambda
	              : raw_target_speed;
	return PowerCtrl_Clampf(limited, -speed_limit, speed_limit);
}

/* ------------------------------- 主控制函数 ------------------------------- */
float power_buffer;      // 当前缓冲能量（J），供上位机观测
float power_limit;       // 本周期允许的最大功率（W），供上位机观测及应用点判断
float base_power_limit;  // 裁判基础功率上限（W），供上位机观测

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

	// 步骤1：读取功率计，一阶低通滤波抑制噪声
	measured_power = PowerCtrl_GetMeasuredPower();
	whell_power.MeasurePower = measured_power;
	if (measured_power >= POWER_MEASURE_VALID_MIN_W)
	{
		g_filtered_power = power_measure_alpha * measured_power
		                 + (1.0f - power_measure_alpha) * g_filtered_power;
	}
	whell_power.PredictPower = g_filtered_power;
	powerPredict             = g_filtered_power;

	// 步骤2：读取裁判功率上限，异常时回退到 65W
	if (PowerCtrl_GetPowerLimit() >= 45)
	{
		base_power_limit = PowerCtrl_GetPowerLimit();
	}
	else
	{
		base_power_limit = 60;
	}

	power_buffer = (float)JUDGE_GetPowerBuffer();

	// 步骤3：根据档位选参数，再由缓冲能量插值得到本周期功率上限
	PowerCtrl_SelectLevelParam(base_power_limit);
	power_limit = PowerCtrl_ComputePowerLimit(power_buffer, base_power_limit);
	whell_power.MaxPowerLimit = (uint16_t)base_power_limit;
	whell_power.InputPower    = power_limit;

	// 步骤4：PID 计算 lambda_target
	// 超功率时：误差驱动 PID，输出 lambda_target < 1 以缩减目标速度
	// 不超功率时：积分保留（避免积分清零导致 lambda_target 突跳），误差和微分项归零
	if ((g_filtered_power > power_limit) && (g_filtered_power > POWER_MEASURE_VALID_MIN_W))
	{
		float error   = g_filtered_power - power_limit;
		float d_error = (error - g_lambda_prev_error) / POWER_CTRL_DT_S;
		g_lambda_integral += error * POWER_CTRL_DT_S;
		// 积分限幅：最多让 lambda 额外下降 0.5
		g_lambda_integral = PowerCtrl_Clampf(g_lambda_integral, 0.0f, 0.5f / g_cur_param->obs_lambda_ki);
		float correction = g_cur_param->obs_lambda_kp * error
		                 + g_cur_param->obs_lambda_ki * g_lambda_integral
		                 + g_cur_param->obs_lambda_kd * d_error;
		lambda_target       = PowerCtrl_Clampf(1.0f - correction, obs_lambda_min, 1.0f);
		g_lambda_prev_error = error;
	}
	else
	{
		// 积分缓慢衰减而非清零，防止功率在 power_limit 附近反复横跳时 lambda 突变
		g_lambda_integral  *= 0.99f;
		g_lambda_prev_error = 0.0f;
		float correction    = g_cur_param->obs_lambda_ki * g_lambda_integral;
		lambda_target       = PowerCtrl_Clampf(1.0f - correction, obs_lambda_min, 1.0f);
	}

	// 步骤5：对 lambda_target 做一阶低通，避免 lambda 突变冲击运动控制
	// 下降（限速）用 alpha_fall，上升（恢复）用 alpha_rise，两者可独立调节响应速度
	float alpha = (lambda_target < g_power_obs_lambda) ? g_cur_param->obs_lambda_alpha_fall : g_cur_param->obs_lambda_alpha_rise;
	g_power_obs_lambda = PowerCtrl_Clampf(
		alpha * lambda_target + (1.0f - alpha) * g_power_obs_lambda,
		obs_lambda_min, 1.0f);
}
