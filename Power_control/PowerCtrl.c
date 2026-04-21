#include "PowerCtrl.h" // 功率控制模块头文件。
#include "main.h" // 主工程头文件。
#include "arm_math.h" // CMSIS-DSP 数学库。
#include "math.h" // 标准数学库。
#include "RLS.h" // 递推最小二乘模块头文件。
#include "user_pid.h" // 通用 PID 模块。
#include "motor.h" // 电机与底盘数据结构头文件。
#include "observe_task.h" // 卡尔曼车速估计。
#include "Motor_Drv.h" // 轮电机实时数据结构。
#include "Judge.h" // 裁判系统功率与缓冲接口。
#include "PowerObserverLimit.h" // 观测量门控模块。

extern float powerPredict; // 外部预测功率变量。

// 轮速上限配置值。 // 轮速最大值。
uint16_t SET_WHEELSPEED_MAX = 8000; // 默认轮速上限。

// 轮电机功率控制对象。 // 全局功率控制实例。
ChassisPower whell_power; // 轮功率控制结构体。

// 运行时开关。
uint8_t g_power_ctrl_enable = 1;
uint8_t g_power_obs_gate_enable = 1;
uint8_t g_power_buffer_pid_enable = 1;

// 当前门控缩放系数（用于观测/调参）。
float g_power_obs_lambda = 1.0f;

// 缓冲能量 PID 闭环调试量。
float g_power_buffer_target = 20.0f;
float g_power_buffer_measure = 50.0f;
float g_power_buffer_pid_out = 0.0f;

// 功率观测量门控内部状态。
static PowerObsCtrl g_power_obs_ctrl;
static uint8_t g_power_obs_inited = 0;
static user_pid_t g_power_buffer_pid;

// 控制周期（s），与 Motor_task 500Hz 对齐。
static const float POWER_CTRL_DT_S = 0.002f;
static const float POWER_BUFFER_TARGET_MIN = 10.0f;
static const float POWER_BUFFER_TARGET_MAX = 120.0f;
static const float POWER_BUFFER_EMERGENCY_TH = 12.0f;
static const float POWER_BUFFER_CRITICAL_TH = 6.0f;
static const float POWER_BUFFER_DROP_GAIN_W_PER_JS = 0.025f;
static const float POWER_BUFFER_SAMPLE_DT_S = 0.02f;

static float g_power_buffer_last = 50.0f;
static float g_power_buffer_drop_rate = 0.0f;

static float PowerCtrl_Clampf(float x, float lo, float hi)
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

static void PowerCtrl_InitBufferPid(void)
{
	/*
	 * 输出单位按“等效可用功率修正值(W)”使用：
	 * output > 0 表示缓冲不足，需要降低可用功率；
	 * output < 0 表示缓冲富余，可适当放开可用功率。
	 */
	PID_INIT(&g_power_buffer_pid, 0.55f, 0.02f, 0.0f, 45.0f, 30.0f, 100.0f, 0.0f);
	PID_Clear(&g_power_buffer_pid);
	g_power_buffer_pid.error = 0.0f;
	g_power_buffer_pid.pre_error = 0.0f;
	g_power_buffer_pid.output = 0.0f;
	g_power_buffer_pid_out = 0.0f;
}

// 自研减速箱 3508。 // 自研 3508 参数初始化。
void PowerInit3508v1(ChassisPower* whell_power) // 初始化自研 3508 参数。
{
	whell_power->toque_coefficient = 2.4324e-6f; // 扭矩系数。
	whell_power->paramVector[0][0] = 1.2158888e-7; // RPM 项参数。
  	whell_power->paramVector[1][0] = 1.5822148e-7; // 扭矩项参数。
  	whell_power->paramVector[2][0] = 3.04855824; // 常数项参数。
	// experience points。 // 经验参数初始化。
  	whell_power->transVector[0][0] = 2.5e-15; // RPM 项协方差。
  	whell_power->transVector[1][1] = 2.5e-15; // 扭矩项协方差。
  	whell_power->transVector[2][2] = 0.000025; // 常数项协方差。
	whell_power->moto_type = 3508; // 电机型号。
	whell_power->UserPowerLimit = 120; // 用户功率上限。
} // 初始化结束。

void PowerCtralInit(ChassisPower* whell_power) // 功率控制初始化入口。
{
	PowerInit3508v1(whell_power); // 载入自研参数。
	PowerControl_AutoUpdateParamInit(whell_power); // 初始化参数更新器。

	PowerObsCtrlParam obs_param;
	PowerObsCtrl_DefaultParam(&obs_param);
	PowerObsCtrl_Init(&g_power_obs_ctrl, &obs_param);
	PowerCtrl_InitBufferPid();
	g_power_buffer_measure = g_power_buffer_target;
	g_power_buffer_last = g_power_buffer_target;
	g_power_buffer_drop_rate = 0.0f;
	g_power_obs_lambda = 1.0f;
	g_power_obs_inited = 1;
} // 初始化入口结束。

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
	g_power_buffer_target = PowerCtrl_Clampf(target, POWER_BUFFER_TARGET_MIN, POWER_BUFFER_TARGET_MAX);
}

static float PowerCtrl_GetPowerLimit(void)
{
	float limit = (float)JUDGE_GetChassisPowerLimit();

	if ((limit < 10.0f) || (limit > 300.0f))
	{
		limit = (float)whell_power.UserPowerLimit;
	}

	return limit;
}

static float PowerCtrl_GetPowerBuffer(void)
{
	float buffer = (float)PowerHeatData.chassis_power_buffer;
	float delta;
	float instant_drop_rate;

	/* 裁判系统数据无效时，退回目标值，避免误触发功率急剧收缩。 */
	if (!JUDGE_IsValid())
	{
		buffer = g_power_buffer_target;
		g_power_buffer_drop_rate = 0.0f;
		g_power_buffer_last = buffer;
		g_power_buffer_measure = buffer;
		return buffer;
	}

	/*
	 * 裁判系统功率帧约 50Hz；直接按 500Hz 微分会把 1J 量化抖动放大成假“快速掉缓冲”。
	 * 这里按 20ms 等效采样计算，并加入死区与低通，避免常态误惩罚。
	 */
	delta = g_power_buffer_last - buffer;
	if (delta > 0.5f)
	{
		instant_drop_rate = delta / POWER_BUFFER_SAMPLE_DT_S;
	}
	else
	{
		instant_drop_rate = 0.0f;
	}

	instant_drop_rate = PowerCtrl_Clampf(instant_drop_rate, 0.0f, 80.0f);
	g_power_buffer_drop_rate = 0.85f * g_power_buffer_drop_rate + 0.15f * instant_drop_rate;
	g_power_buffer_drop_rate = PowerCtrl_Clampf(g_power_buffer_drop_rate, 0.0f, 80.0f);
	g_power_buffer_last = buffer;

	g_power_buffer_measure = buffer;
	return buffer;
}

static float PowerCtrl_GetPidAdjustedLimit(float base_power_limit, float power_buffer)
{
	float pid_output;
	float adjusted_limit;
	float effective_target;
	float auto_target;

#if POWER_CTRL_BUFFER_PID_ENABLE
	if (g_power_buffer_pid_enable == 0)
	{
		g_power_buffer_pid_out = 0.0f;
		return base_power_limit;
	}

	/* 低功率限额场景下自动下调目标缓冲，避免“目标过高导致长期限扭”。 */
	auto_target = PowerCtrl_Clampf(0.5f * base_power_limit + 8.0f, 18.0f, 55.0f);
	effective_target = g_power_buffer_target;
	if (effective_target > auto_target)
	{
		effective_target = auto_target;
	}

	/* 缓冲稳定且接近目标时清积分，避免残余积分让车长期“没劲”。 */
	if ((g_power_buffer_drop_rate < 2.0f) && (power_buffer >= (effective_target - 2.0f)))
	{
		PID_Clear(&g_power_buffer_pid);
	}

	PID_Set_Error(&g_power_buffer_pid, power_buffer, effective_target);
	pid_output = PID_coculate(&g_power_buffer_pid);
	g_power_buffer_pid_out = pid_output;

	/*
	 * 缓冲闭环：
	 * - 缓冲低于目标 -> pid_output > 0 -> 可用功率降低；
	 * - 缓冲高于目标 -> pid_output < 0 -> 可用功率放开。
	 */
	adjusted_limit = base_power_limit - pid_output;

	/* 缓冲掉得越快，额外收紧可用功率。 */
	adjusted_limit -= POWER_BUFFER_DROP_GAIN_W_PER_JS * g_power_buffer_drop_rate;

	/* 低缓冲紧急区，进一步快速收紧。 */
	if (power_buffer < POWER_BUFFER_EMERGENCY_TH)
	{
		adjusted_limit -= (POWER_BUFFER_EMERGENCY_TH - power_buffer) * 1.0f;
	}

	if (power_buffer < POWER_BUFFER_CRITICAL_TH)
	{
		adjusted_limit = PowerCtrl_Clampf(adjusted_limit, 5.0f, 10.0f);
		return adjusted_limit;
	}

	adjusted_limit = PowerCtrl_Clampf(adjusted_limit, 5.0f, base_power_limit + 20.0f);
	return adjusted_limit;
#else
	(void)power_buffer;
	g_power_buffer_pid_out = 0.0f;
	return base_power_limit;
#endif
}

// 轮电机功率控制。 // 轮电机功率控制入口。
void Whellv1PowerCtral() // 轮功率控制函数。
{
#if POWER_CTRL_MODULE_ENABLE
	float wl;
	float wr;
	float tl;
	float tr;
	float base_power_limit;
	float power_limit;
	float power_buffer;

	if (g_power_obs_inited == 0)
	{
		PowerCtralInit(&whell_power);
	}

	if (g_power_ctrl_enable == 0)
	{
		g_power_obs_ctrl.lambda = 1.0f;
		g_power_obs_lambda = 1.0f;
		return;
	}

	// 读取轮电机当前状态与目标扭矩。
	wl = L_LK9025.Rx_Data.Velocity;
	wr = R_LK9025.Rx_Data.Velocity;
	tl = L_LK9025.Target_Torque;
	tr = R_LK9025.Target_Torque;

	// 记录中间量，便于在线观察。
	whell_power.SumPowerSpeed = wl * wl + wr * wr;
	whell_power.SumPowerTorque = tl * tl + tr * tr;
	whell_power.EffetivePower = whell_power.toque_coefficient * (wl * tl + wr * tr);

	// 功率预测统一复用 RLS 模块中的模型实现。
	whell_power.PredictPower = PowerControl_WheelPowerPredict(wl, wr, tl, tr, &whell_power);
	powerPredict = whell_power.PredictPower;

	// 读取裁判系统功率限制与缓冲。
	base_power_limit = PowerCtrl_GetPowerLimit();
	power_buffer = PowerCtrl_GetPowerBuffer();
	power_limit = PowerCtrl_GetPidAdjustedLimit(base_power_limit, power_buffer);

	whell_power.MaxPowerLimit = (uint16_t)base_power_limit;
	whell_power.InputPower = power_limit;

#if POWER_CTRL_OBSERVER_GATE_ENABLE
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
#else
	(void)power_buffer;
	g_power_obs_ctrl.lambda = 1.0f;
	g_power_obs_lambda = 1.0f;
#endif

#else
	g_power_obs_lambda = 1.0f;
#endif
} // 轮功率控制结束。

void PowerCtrl_ApplyObserverGate(float *body_distance_error,
                                 float *speed_error,
                                 float *yaw_error,
                                 float *d_yaw)
{
#if POWER_CTRL_MODULE_ENABLE && POWER_CTRL_OBSERVER_GATE_ENABLE
	PowerObsInput in;
	PowerObsOutput out;
	float speed_mag;
	float min_brake_gain;

	if ((body_distance_error == NULL) || (speed_error == NULL) || (yaw_error == NULL) || (d_yaw == NULL))
	{
		return;
	}

	if ((g_power_ctrl_enable == 0) || (g_power_obs_gate_enable == 0) || (g_power_obs_inited == 0))
	{
		return;
	}

	in.body_distance_error = *body_distance_error;
	in.speed_error = *speed_error;
	in.yaw_error = *yaw_error;
	in.d_yaw = *d_yaw;

	PowerObsCtrl_Apply(&g_power_obs_ctrl, &in, &out);

	/*
	 * 低缓冲时只抑制“继续加速”，但尽量保留“减速刹车”能力：
	 * speed_error = target_speed - current_speed
	 * 当 speed_error 与 current_speed 异号时，代表希望降速到目标。
	 */
	speed_mag = fabsf(kalman_body_speed);
	if ((in.speed_error * kalman_body_speed < 0.0f) && (speed_mag > 0.2f))
	{
		/* 车速越大，刹车保留比例越高，避免“停不下来”。 */
		min_brake_gain = 0.70f + 0.30f * PowerCtrl_Clampf(speed_mag / 2.0f, 0.0f, 1.0f);
		if (fabsf(out.speed_error) < fabsf(in.speed_error) * min_brake_gain)
		{
			out.speed_error = in.speed_error * min_brake_gain;
		}
	}

	*body_distance_error = out.body_distance_error;
	*speed_error = out.speed_error;
	*yaw_error = out.yaw_error;
	*d_yaw = out.d_yaw;
	g_power_obs_lambda = out.lambda;
#else
	(void)body_distance_error;
	(void)speed_error;
	(void)yaw_error;
	(void)d_yaw;
#endif
}

void PowerCtrl() // 功率控制总入口。
{
	Whellv1PowerCtral(); // 执行轮功率控制。
} // 总入口结束。
