#include "PowerCtrl.h"
#include "RLS.h"
#include "PowerObserverLimit.h"

extern "C" {
#include "main.h"
#include "arm_math.h"
#include "user_pid.h"
#include "motor.h"
#include "observe_task.h"
#include "Motor_Drv.h"
#include "Judge.h"
#include "wattmeter.h"
}

#include <cmath>

extern "C" float powerPredict;

uint16_t SET_WHEELSPEED_MAX = 8000;
ChassisPower whell_power;

uint8_t g_power_ctrl_enable = 1;
uint8_t g_power_obs_gate_enable = 1;
uint8_t g_power_buffer_pid_enable = 1;

float g_power_obs_lambda = 1.0f;
float g_power_buffer_target = 20.0f;
float g_power_buffer_measure = 50.0f;
float g_power_buffer_pid_out = 0.0f;
float g_power_buffer_drop_rate_alpha = 0.85f;

static PowerObsCtrl g_power_obs_ctrl;
static uint8_t g_power_obs_inited = 0;
static user_pid_t g_power_buffer_pid;

static constexpr float POWER_CTRL_DT_S = 0.002f;
static constexpr float POWER_BUFFER_TARGET_MIN = 10.0f;
static constexpr float POWER_BUFFER_TARGET_MAX = 120.0f;
static constexpr float POWER_BUFFER_EMERGENCY_TH = 12.0f;
static constexpr float POWER_BUFFER_CRITICAL_TH = 6.0f;
static constexpr float POWER_BUFFER_DROP_GAIN_W_PER_JS = 0.025f;
static constexpr float POWER_BUFFER_SAMPLE_DT_S = 0.02f;
static constexpr float POWER_MEASURE_VALID_MIN_W = 1.0f;
static constexpr float POWER_MEASURE_VALID_MAX_W = 500.0f;

static float g_power_buffer_last = 50.0f;
static float g_power_buffer_drop_rate = 0.0f;

namespace {

float clampf(float x, float lo, float hi)
{
	if (x < lo) return lo;
	if (x > hi) return hi;
	return x;
}

float torqueToDJICurrent(float torque)
{
	float current = ((((torque / 14.88f) / 0.02f) / 20.0f) * 16384.0f);
	return clampf(current, -16384.0f, 16384.0f);
}

void initBufferPid()
{
	PID_INIT(&g_power_buffer_pid,
	         0.55f, 0.02f, 0.0f,
	         45.0f, 30.0f, 0.0f, 100.0f, 0.0f);
	PID_Clear(&g_power_buffer_pid);
	g_power_buffer_pid.error = 0.0f;
	g_power_buffer_pid.pre_error = 0.0f;
	g_power_buffer_pid.output = 0.0f;
	g_power_buffer_pid_out = 0.0f;
}

float getPowerLimit()
{
	float limit = static_cast<float>(JUDGE_GetChassisPowerLimit());
	if ((limit < 35.0f) || (limit > 300.0f))
		limit = static_cast<float>(whell_power.UserPowerLimit);
	return limit;
}

float getPowerBuffer()
{
	float buffer = static_cast<float>(JUDGE_GetPowerBuffer());

	if (!JUDGE_IsValid())
	{
		g_power_buffer_drop_rate = 0.0f;
		g_power_buffer_last = g_power_buffer_target;
		g_power_buffer_measure = g_power_buffer_target;
		return g_power_buffer_target;
	}

	float delta = g_power_buffer_last - buffer;
	float instant_drop_rate = 0.0f;
	if (delta > 0.5f)
		instant_drop_rate = delta / POWER_BUFFER_SAMPLE_DT_S;

	instant_drop_rate = clampf(instant_drop_rate, 0.0f, 80.0f);
	g_power_buffer_drop_rate = g_power_buffer_drop_rate_alpha * g_power_buffer_drop_rate
	                         + (1.0f - g_power_buffer_drop_rate_alpha) * instant_drop_rate;
	g_power_buffer_drop_rate = clampf(g_power_buffer_drop_rate, 0.0f, 80.0f);

	g_power_buffer_last = buffer;
	g_power_buffer_measure = buffer;
	return buffer;
}

float getMeasuredChassisPower()
{
	float measured_power = wattmeter_data.power;
	if ((measured_power < POWER_MEASURE_VALID_MIN_W) || (measured_power > POWER_MEASURE_VALID_MAX_W))
		return -1.0f;
	return measured_power;
}

float getPidAdjustedLimit(float base_power_limit, float power_buffer)
{
	if (g_power_buffer_pid_enable == 0)
	{
		g_power_buffer_pid_out = 0.0f;
		return base_power_limit;
	}

	float auto_target = clampf(0.5f * base_power_limit + 8.0f, 18.0f, 55.0f);
	float effective_target = g_power_buffer_target;
	if (effective_target > auto_target)
		effective_target = auto_target;

	if ((g_power_buffer_drop_rate < 2.0f) && (power_buffer >= (effective_target - 2.0f)))
		PID_Clear(&g_power_buffer_pid);

	PID_Set_Error(&g_power_buffer_pid, power_buffer, effective_target);
	float pid_output = PID_coculate(&g_power_buffer_pid);
	g_power_buffer_pid_out = pid_output;

	float adjusted_limit = base_power_limit - pid_output;
	adjusted_limit -= POWER_BUFFER_DROP_GAIN_W_PER_JS * g_power_buffer_drop_rate;

	if (power_buffer < POWER_BUFFER_EMERGENCY_TH)
		adjusted_limit -= (POWER_BUFFER_EMERGENCY_TH - power_buffer) * 1.0f;

	if (power_buffer < POWER_BUFFER_CRITICAL_TH)
		return clampf(adjusted_limit, 5.0f, 10.0f);

	return clampf(adjusted_limit, 5.0f, base_power_limit + 20.0f);
}

} // anonymous namespace

static void PowerInit3508v1(ChassisPower* whell_power_out)
{
	whell_power_out->toque_coefficient = 2.4324e-6f;
	whell_power_out->paramVector[0][0] = 1.2158888e-7f;
	whell_power_out->paramVector[1][0] = 1.5822148e-7f;
	whell_power_out->paramVector[2][0] = 3.04855824f;

	whell_power_out->transVector[0][0] = 2.5e-15f;
	whell_power_out->transVector[1][1] = 2.5e-15f;
	whell_power_out->transVector[2][2] = 0.000025f;

	whell_power_out->moto_type = 3508;
	whell_power_out->UserPowerLimit = 120;
}

void PowerCtralInit(ChassisPower* whell_power_out)
{
	PowerObsCtrlParam obs_param;

	PowerInit3508v1(whell_power_out);
	PowerControl_AutoUpdateParamInit(whell_power_out);

	PowerObsCtrl_DefaultParam(&obs_param);
	PowerObsCtrl_Init(&g_power_obs_ctrl, &obs_param);

	initBufferPid();

	g_power_buffer_measure = g_power_buffer_target;
	g_power_buffer_last = g_power_buffer_target;
	g_power_buffer_drop_rate = 0.0f;
	g_power_obs_lambda = 1.0f;
	g_power_obs_inited = 1;
}

void PowerCtrl_SetEnable(uint8_t enable)
{
	g_power_ctrl_enable = (enable != 0) ? 1 : 0;
	if (g_power_ctrl_enable == 0)
	{
		PID_Clear(&g_power_buffer_pid);
		g_power_buffer_pid_out = 0.0f;
	}
}

void PowerCtrl_SetObserverGateEnable(uint8_t enable)
{
	g_power_obs_gate_enable = (enable != 0) ? 1 : 0;
}

void PowerCtrl_SetBufferPidEnable(uint8_t enable)
{
	g_power_buffer_pid_enable = (enable != 0) ? 1 : 0;
	if (g_power_buffer_pid_enable == 0)
	{
		PID_Clear(&g_power_buffer_pid);
		g_power_buffer_pid_out = 0.0f;
	}
}

void PowerCtrl_SetBufferTarget(float target)
{
	g_power_buffer_target = clampf(target, POWER_BUFFER_TARGET_MIN, POWER_BUFFER_TARGET_MAX);
}

void PowerCtrl_ApplyObserverGate(float *body_distance_error,
                                 float *speed_error,
                                 float *yaw_error,
                                 float *d_yaw)
{
	if (!body_distance_error || !speed_error || !yaw_error || !d_yaw)
		return;

	if ((g_power_ctrl_enable == 0) || (g_power_obs_gate_enable == 0) || (g_power_obs_inited == 0))
		return;

	PowerObsInput in;
	in.body_distance_error = *body_distance_error;
	in.speed_error = *speed_error;
	in.yaw_error = *yaw_error;
	in.d_yaw = *d_yaw;

	PowerObsOutput out;
	PowerObsCtrl_Apply(&g_power_obs_ctrl, &in, &out);

	float speed_mag = std::fabs(kalman_body_speed);
	if ((in.speed_error * kalman_body_speed < 0.0f) && (speed_mag > 0.2f))
	{
		float min_brake_gain = 0.70f + 0.30f * clampf(speed_mag / 2.0f, 0.0f, 1.0f);
		if (std::fabs(out.speed_error) < std::fabs(in.speed_error) * min_brake_gain)
			out.speed_error = in.speed_error * min_brake_gain;
	}

	*body_distance_error = out.body_distance_error;
	*speed_error = out.speed_error;
	*yaw_error = out.yaw_error;
	*d_yaw = out.d_yaw;

	g_power_obs_lambda = out.lambda;
}

static void Whellv1PowerCtral()
{
	if (g_power_obs_inited == 0)
		PowerCtralInit(&whell_power);

	if (g_power_ctrl_enable == 0)
	{
		g_power_obs_ctrl.lambda = 1.0f;
		g_power_obs_lambda = 1.0f;
		return;
	}

	float wl = L_DJ3508.Rx_Data.Velocity;
	float wr = R_DJ3508.Rx_Data.Velocity;
	float il = torqueToDJICurrent(L_DJ3508.Target_Torque);
	float ir = torqueToDJICurrent(R_DJ3508.Target_Torque);

	whell_power.SumPowerSpeed = wl * wl + wr * wr;
	whell_power.SumPowerTorque = il * il + ir * ir;
	whell_power.EffetivePower = whell_power.toque_coefficient * (wl * il + wr * ir);

	whell_power.PredictPower = PowerControl_WheelPowerPredict(wl, wr, il, ir, &whell_power);
	powerPredict = whell_power.PredictPower;

	float measured_power = getMeasuredChassisPower();
	whell_power.MeasurePower = measured_power;
	if (measured_power >= POWER_MEASURE_VALID_MIN_W)
	{
		float measured_loss = measured_power - whell_power.EffetivePower;
		measured_loss = clampf(measured_loss, 0.0f, POWER_MEASURE_VALID_MAX_W);

		PowerControl_AutoUpdateParam(whell_power.SumPowerSpeed / 2,
		                            whell_power.SumPowerTorque / 2,
		                            1.0f,
		                            measured_loss / 2,
		                            whell_power);

		whell_power.PredictPower = PowerControl_WheelPowerPredict(wl, wr, il, ir, &whell_power);
		powerPredict = whell_power.PredictPower;
	}

	float base_power_limit = getPowerLimit();
	float power_buffer = getPowerBuffer();

	float power_limit = getPidAdjustedLimit(base_power_limit, power_buffer);
	whell_power.MaxPowerLimit = static_cast<uint16_t>(base_power_limit);
	whell_power.InputPower = power_limit;

	if (g_power_obs_gate_enable != 0)
	{
		g_power_obs_lambda = PowerObsCtrl_ComputeLambda(&g_power_obs_ctrl,
		                                               power_limit,
		                                               power_buffer,
		                                               whell_power.PredictPower,
		                                               POWER_CTRL_DT_S);
	}
	else
	{
		g_power_obs_ctrl.lambda = 1.0f;
		g_power_obs_lambda = 1.0f;
	}
}

void PowerCtrl(void)
{
	Whellv1PowerCtral();
}
