#include "PowerCtrl.h"
#include "main.h"
#include "arm_math.h"
#include "math.h"
#include "motor.h"
#include "observe_task.h"
#include "Judge.h"
#include "PowerObserverLimit.h"
#include "wattmeter.h"

extern float powerPredict;

/* ------------------------------- 全局对象与开关 ------------------------------- */

ChassisPower whell_power;

uint8_t g_power_ctrl_enable    = 1;
uint8_t g_power_obs_gate_enable = 1;

float g_power_obs_lambda = 1.0f;

static PowerObsCtrl g_power_obs_ctrl;
static uint8_t g_power_obs_inited = 0;

/* ------------------------------- 固定参数区 ------------------------------- */

static const float POWER_CTRL_DT_S          = 0.002f;
static const float POWER_MEASURE_VALID_MIN_W = 1.0f;
static const float POWER_MEASURE_VALID_MAX_W = 500.0f;

/* -------------------------------- 调参区 -------------------------------- */
// buffer_min / buffer_max 控制功率上限随缓冲的映射区间（J）
// 缓冲 <= buffer_min 时 power_limit 降到最低；>= buffer_max 时等于裁判上限
static const float POWER_BUFFER_MIN = 10.0f;
static const float POWER_BUFFER_MAX = 60.0f;

// power_limit 斜坡速率（W/s）：下降快、上升慢，保证安全
static const float POWER_LIMIT_FALL_RATE = 200.0f;
static const float POWER_LIMIT_RISE_RATE = 30.0f;

// power_limit 最低值（W），防止完全断力
static const float POWER_LIMIT_MIN = 10.0f;

/* ------------------------------- 运行时状态 ------------------------------- */

static float g_last_measured_power = 0.0f;
static float g_power_limit_state   = 0.0f; // 带斜坡的 power_limit 当前值

/* ------------------------------- 工具函数 ------------------------------- */

static float PowerCtrl_Clampf(float x, float lo, float hi)
{
	if (x < lo) return lo;
	if (x > hi) return hi;
	return x;
}

/* ------------------------------- 初始化 ------------------------------- */

static void PowerInit3508v1(ChassisPower *p)
{
	p->toque_coefficient = 2.4324e-6f;
	p->moto_type         = 3508;
	p->UserPowerLimit    = 120;
}

void PowerCtralInit(ChassisPower *whell_power_out)
{
	PowerObsCtrlParam obs_param;

	PowerInit3508v1(whell_power_out);

	PowerObsCtrl_DefaultParam(&obs_param);
	PowerObsCtrl_Init(&g_power_obs_ctrl, &obs_param);

	g_power_obs_lambda    = 1.0f;
	g_power_limit_state   = 0.0f;
	g_last_measured_power = 0.0f;
	g_power_obs_inited    = 1;
}

/* ------------------------------- 开关接口 ------------------------------- */

void PowerCtrl_SetEnable(uint8_t enable)
{
	g_power_ctrl_enable = (enable != 0) ? 1 : 0;
}

void PowerCtrl_SetObserverGateEnable(uint8_t enable)
{
	g_power_obs_gate_enable = (enable != 0) ? 1 : 0;
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

/*
 * 根据缓冲能量线性插值得到目标功率上限，再用斜坡限制变化速率。
 * buffer_min → POWER_LIMIT_MIN
 * buffer_max → base_limit
 */
static float PowerCtrl_ComputePowerLimit(float buffer, float base_limit)
{
	float ratio  = (buffer - POWER_BUFFER_MIN) / (POWER_BUFFER_MAX - POWER_BUFFER_MIN);
	ratio        = PowerCtrl_Clampf(ratio, 0.0f, 1.0f);
	float target = POWER_LIMIT_MIN + ratio * (base_limit - POWER_LIMIT_MIN);

	float delta    = target - g_power_limit_state;
	float max_rise = POWER_LIMIT_RISE_RATE * POWER_CTRL_DT_S;
	float max_fall = POWER_LIMIT_FALL_RATE * POWER_CTRL_DT_S;
	g_power_limit_state += PowerCtrl_Clampf(delta, -max_fall, max_rise);

	return g_power_limit_state;
}

/* ------------------------------- 观测门控 ------------------------------- */

void PowerCtrl_ApplyObserverGate(float *body_distance_error,
                                 float *speed_error,
                                 float *yaw_error,
                                 float *d_yaw)
{
	PowerObsInput  in;
	PowerObsOutput out;
	float speed_mag;
	float min_brake_gain;

	if ((body_distance_error == NULL) || (speed_error == NULL) ||
	    (yaw_error == NULL) || (d_yaw == NULL))
	{
		return;
	}

	if ((g_power_ctrl_enable == 0) || (g_power_obs_gate_enable == 0) || (g_power_obs_inited == 0))
	{
		return;
	}

	in.body_distance_error = *body_distance_error;
	in.speed_error         = *speed_error;
	in.yaw_error           = *yaw_error;
	in.d_yaw               = *d_yaw;

	PowerObsCtrl_Apply(&g_power_obs_ctrl, &in, &out);

	// 保刹车：减速工况保留更大速度误差，防止限功率时刹不住
	speed_mag = fabsf(kalman_body_speed);
	if ((in.speed_error * kalman_body_speed < 0.0f) && (speed_mag > 0.2f))
	{
		min_brake_gain = 0.70f + 0.30f * PowerCtrl_Clampf(speed_mag / 2.0f, 0.0f, 1.0f);
		if (fabsf(out.speed_error) < fabsf(in.speed_error) * min_brake_gain)
		{
			out.speed_error = in.speed_error * min_brake_gain;
		}
	}

	*body_distance_error = out.body_distance_error;
	*speed_error         = out.speed_error;
	*yaw_error           = out.yaw_error;
	*d_yaw               = out.d_yaw;

	g_power_obs_lambda = out.lambda;
}

/* ------------------------------- 主控制函数 ------------------------------- */

static void Whellv1PowerCtral(void)
{
	float base_power_limit;
	float power_buffer;
	float power_limit;
	float measured_power;

	if (g_power_obs_inited == 0)
	{
		PowerCtralInit(&whell_power);
	}

	if (g_power_ctrl_enable == 0)
	{
		g_power_obs_ctrl.lambda = 1.0f;
		g_power_obs_lambda      = 1.0f;
		return;
	}

	// 步骤1：读取本帧功率计，更新下一帧预测值
	measured_power = PowerCtrl_GetMeasuredPower();
	whell_power.MeasurePower = measured_power;
	if (measured_power >= POWER_MEASURE_VALID_MIN_W)
	{
		g_last_measured_power = measured_power;
	}
	whell_power.PredictPower = g_last_measured_power;
	powerPredict             = g_last_measured_power;

	// 步骤2：读取裁判上限和缓冲
	base_power_limit = PowerCtrl_GetPowerLimit();
	power_buffer     = (float)JUDGE_GetPowerBuffer();
	if (!JUDGE_IsValid())
	{
		power_buffer = POWER_BUFFER_MAX; // 裁判无效时默认缓冲充足
	}

	// 步骤3：计算带斜坡的功率上限
	power_limit = PowerCtrl_ComputePowerLimit(power_buffer, base_power_limit);
	whell_power.MaxPowerLimit = (uint16_t)base_power_limit;
	whell_power.InputPower    = power_limit;

	// 步骤4：计算门控系数 lambda
	if (g_power_obs_gate_enable != 0)
	{
		g_power_obs_lambda = PowerObsCtrl_ComputeLambda(&g_power_obs_ctrl,
		                                               power_limit,
		                                               g_last_measured_power,
		                                               POWER_CTRL_DT_S);
	}
	else
	{
		g_power_obs_ctrl.lambda = 1.0f;
		g_power_obs_lambda      = 1.0f;
	}
}

void PowerCtrl(void)
{
	Whellv1PowerCtral();
}
