#include "PowerCtrl.h" // 功率控制模块对外接口。
#include "main.h" // 工程全局类型与基础定义。
#include "arm_math.h" // CMSIS-DSP 数学函数与矩阵支持。
#include "math.h" // fabsf/powf 等标准数学函数。
#include "RLS.h" // 预测模型与在线更新函数声明。
#include "user_pid.h" // 通用 PID 控制器。
#include "motor.h" // LQR 状态量与目标量定义。
#include "observe_task.h" // kalman_body_speed 状态估计值。
#include "Motor_Drv.h" // 轮电机实时速度与目标电流指令（字段名沿用 Target_Torque）。
#include "Judge.h" // 裁判系统功率限制与缓冲读数。
#include "PowerObserverLimit.h" // 观测量门控逻辑。

extern float powerPredict; // 上位机观测：当前功率预测值（W）。

/* ------------------------------- 全局对象与开关 ------------------------------- */

uint16_t SET_WHEELSPEED_MAX = 8000; // 轮速上限配置（历史保留变量，当前仅作参数记录）。
ChassisPower whell_power; // 轮组功率控制对象（模型参数、观测量、功率预算等均在此结构体中）。

uint8_t g_power_ctrl_enable = 1; // 运行时总开关：1=启用功率控制，0=旁路。
uint8_t g_power_obs_gate_enable = 1; // 运行时门控开关：1=缩放 LQR 观测量，0=不缩放。
uint8_t g_power_buffer_pid_enable = 1; // 运行时缓冲 PID 开关：1=启用缓冲闭环，0=仅按裁判上限。

float g_power_obs_lambda = 1.0f; // 当前观测门控系数（用于上位机调试，不直接参与控制决策）。
float g_power_buffer_target = 20.0f; // 缓冲目标值（J）。
float g_power_buffer_measure = 50.0f; // 最新缓冲测量值（J）。
float g_power_buffer_pid_out = 0.0f; // 缓冲 PID 输出（W，正值代表收紧功率预算）。
float g_power_buffer_drop_rate_alpha = 0.85f; // 缓冲掉速低通滤波系数，越大越平滑。

static PowerObsCtrl g_power_obs_ctrl; // 观测门控控制器状态。
static uint8_t g_power_obs_inited = 0; // 门控模块初始化标志：0=未初始化，1=已初始化。
static user_pid_t g_power_buffer_pid; // 缓冲能量 PID 对象。

/* ------------------------------- 固定参数区 ------------------------------- */

static const float POWER_CTRL_DT_S = 0.002f; // 控制周期（s），与 500Hz 主控周期对齐。
static const float POWER_BUFFER_TARGET_MIN = 10.0f; // 缓冲目标下限（J）。
static const float POWER_BUFFER_TARGET_MAX = 120.0f; // 缓冲目标上限（J）。
static const float POWER_BUFFER_EMERGENCY_TH = 12.0f; // 紧急缓冲阈值（J）。
static const float POWER_BUFFER_CRITICAL_TH = 6.0f; // 临界缓冲阈值（J）。
static const float POWER_BUFFER_DROP_GAIN_W_PER_JS = 0.025f; // 缓冲掉速额外惩罚增益（W/(J/s)）。
static const float POWER_BUFFER_SAMPLE_DT_S = 0.02f; // 裁判缓冲近似采样周期（s，约 50Hz）。
static const float POWER_MEASURE_VALID_MIN_W = 1.0f; // 允许用于 RLS 更新的最小测量功率（W）。
static const float POWER_MEASURE_VALID_MAX_W = 500.0f; // 允许用于 RLS 更新的最大测量功率（W）。

static float g_power_buffer_last = 50.0f; // 上一次缓冲测量值（J）。
static float g_power_buffer_drop_rate = 0.0f; // 当前缓冲掉速估计（J/s）。

/* -------------------------------- 调参区 -------------------------------- */


/*
 * @brief 浮点限幅函数。
 * @param x 输入值。
 * @param lo 下限。
 * @param hi 上限。
 * @return 限幅后的结果。
 */
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

/*
 * @brief 将电机扭矩指令换算为 DJI 电流指令值。
 * @note 该公式与 Motor_Drv.c 中 DJI_Motor_Torque_Ctrl 保持一致。
 */
static float PowerCtrl_TorqueToDJICurrent(float torque)
{
	float current;

	current = ((((torque / 14.88f) / 0.02f) / 20.0f) * 16384.0f);
	return PowerCtrl_Clampf(current, -16384.0f, 16384.0f);
}

/*
 * @brief 初始化缓冲 PID。
 * @note PID 输出单位按 W 解释，正值表示收紧可用功率，负值表示放开可用功率。
 */
static void PowerCtrl_InitBufferPid(void)
{
	PID_INIT(&g_power_buffer_pid,
	         0.55f,
	         0.02f,
	         0.0f,
	         45.0f,
	         30.0f,
			 0.0,
	         100.0f,
	         0.0f);
	PID_Clear(&g_power_buffer_pid);
	g_power_buffer_pid.error = 0.0f;
	g_power_buffer_pid.pre_error = 0.0f;
	g_power_buffer_pid.output = 0.0f;
	g_power_buffer_pid_out = 0.0f;
}

/*
 * @brief 初始化轮组功率模型参数（3508）。
 * @param whell_power_out 轮组功率对象指针。
 */
void PowerInit3508v1(ChassisPower* whell_power_out)
{
	/* 模型参数：loss = a*w^2 + b*i^2 + c。 */
	whell_power_out->toque_coefficient = 2.4324e-6f;//有效做功系数
	whell_power_out->paramVector[0][0] = 1.2158888e-7f;
	whell_power_out->paramVector[1][0] = 1.5822148e-7f;
	whell_power_out->paramVector[2][0] = 3.04855824f;

	/* RLS 协方差初值（决定在线更新初期收敛速度）。 */
	whell_power_out->transVector[0][0] = 2.5e-15f;
	whell_power_out->transVector[1][1] = 2.5e-15f;
	whell_power_out->transVector[2][2] = 0.000025f;

	whell_power_out->moto_type = 3508;
	whell_power_out->UserPowerLimit = 120;
}

/*
 * @brief 功率控制初始化入口。
 * @param whell_power_out 轮组功率对象指针。
 */
void PowerCtralInit(ChassisPower* whell_power_out)
{
	PowerObsCtrlParam obs_param; // 门控参数缓存（先取默认值，再写入控制器）

	PowerInit3508v1(whell_power_out); // 1) 初始化功率模型参数。
	PowerControl_AutoUpdateParamInit(whell_power_out); // 2) 初始化 RLS 内部矩阵对象。

	PowerObsCtrl_DefaultParam(&obs_param); // 3) 初始化门控参数。
	PowerObsCtrl_Init(&g_power_obs_ctrl, &obs_param); // 4) 初始化门控状态。

	PowerCtrl_InitBufferPid(); // 5) 初始化缓冲 PID。

	/* 6) 初始化调试输出变量。 */
	g_power_buffer_measure = g_power_buffer_target;
	g_power_buffer_last = g_power_buffer_target;
	g_power_buffer_drop_rate = 0.0f;
	g_power_obs_lambda = 1.0f;
	g_power_obs_inited = 1;
}

/*
 * @brief 运行时总开关。
 * @param enable 0=关闭；非0=开启。
 */
void PowerCtrl_SetEnable(uint8_t enable)
{
	g_power_ctrl_enable = (enable != 0) ? 1 : 0;
	if (g_power_ctrl_enable == 0)
	{
		PID_Clear(&g_power_buffer_pid);
		g_power_buffer_pid_out = 0.0f;
	}
}

/*
 * @brief 运行时观测门控开关。
 * @param enable 0=关闭门控；非0=开启门控。
 */
void PowerCtrl_SetObserverGateEnable(uint8_t enable)
{
	g_power_obs_gate_enable = (enable != 0) ? 1 : 0;
}

/*
 * @brief 运行时缓冲 PID 开关。
 * @param enable 0=关闭缓冲 PID；非0=开启缓冲 PID。
 */
void PowerCtrl_SetBufferPidEnable(uint8_t enable)
{
	g_power_buffer_pid_enable = (enable != 0) ? 1 : 0;
	if (g_power_buffer_pid_enable == 0)
	{
		PID_Clear(&g_power_buffer_pid);
		g_power_buffer_pid_out = 0.0f;
	}
}

/*
 * @brief 设置缓冲目标值。
 * @param target 目标缓冲（J）。
 */
void PowerCtrl_SetBufferTarget(float target)
{
	g_power_buffer_target = PowerCtrl_Clampf(target, POWER_BUFFER_TARGET_MIN, POWER_BUFFER_TARGET_MAX);
}

/*
 * @brief 获取当前有效功率上限。
 * @return 有效功率上限（W）。
 */
static float PowerCtrl_GetPowerLimit(void)
{
	float limit = (float)JUDGE_GetChassisPowerLimit(); // 裁判系统发布的功率限制。

	/* 异常保护：裁判值越界时回退到用户上限。 */
	if ((limit < 35.0f) || (limit > 300.0f))
	{
		limit = (float)whell_power.UserPowerLimit;
	}

	return limit;
}

/*
 * @brief 获取缓冲能量并估计缓冲掉速。
 * @return 当前缓冲能量（J）。
 */
static float PowerCtrl_GetPowerBuffer(void)
{
	float buffer = (float)JUDGE_GetPowerBuffer(); // 裁判缓冲读数。
	float delta; // 连续两帧缓冲差值。
	float instant_drop_rate; // 本次计算得到的瞬时掉速（J/s）。

	/* 裁判数据无效时，回退为目标值并清掉速。 */
	if (!JUDGE_IsValid())
	{
		buffer = g_power_buffer_target;
		g_power_buffer_drop_rate = 0.0f;
		g_power_buffer_last = buffer;
		g_power_buffer_measure = buffer;
		return buffer;
	}

	/*
	 * delta > 0.5J 才视为有效掉缓冲，避免 1J 量化噪声在 500Hz 下被误放大。
	 * 等效采样周期按 20ms（50Hz 裁判帧率）计算。
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

	/* 瞬时掉速限幅，防止偶发错误帧导致异常收紧。 */
	instant_drop_rate = PowerCtrl_Clampf(instant_drop_rate, 0.0f, 80.0f);

	/* 一阶低通平滑掉速估计，提升控制连续性。 */
	g_power_buffer_drop_rate = g_power_buffer_drop_rate_alpha * g_power_buffer_drop_rate
	                        + (1.0f - g_power_buffer_drop_rate_alpha) * instant_drop_rate;
	g_power_buffer_drop_rate = PowerCtrl_Clampf(g_power_buffer_drop_rate, 0.0f, 80.0f);

	/* 更新状态缓存与调试输出。 */
	g_power_buffer_last = buffer;
	g_power_buffer_measure = buffer;

	return buffer;
}

/*
 * @brief 获取当前可用的实测底盘功率（用于 RLS 在线更新）。
 * @return >=0 表示有效功率（W）；<0 表示无效。
 */
static float PowerCtrl_GetMeasuredChassisPower(void)
{
	float measured_power;

	if (!JUDGE_IsValid())
	{
		return -1.0f;
	}

	/*
	 * 当前协议头文件中该字段名为 reserved_3，
	 * 但按照实际数据含义使用为“实测底盘功率（W）”。
	 */
	measured_power = PowerHeatData.reserved_3;

	if ((measured_power < POWER_MEASURE_VALID_MIN_W) || (measured_power > POWER_MEASURE_VALID_MAX_W))
	{
		return -1.0f;
	}

	return measured_power;
}

/*
 * @brief 根据缓冲 PID 和保护逻辑得到修正后的功率预算。
 * @param base_power_limit 裁判基础上限（W）。
 * @param power_buffer 当前缓冲（J）。
 * @return 修正后的可用功率预算（W）。
 */
static float PowerCtrl_GetPidAdjustedLimit(float base_power_limit, float power_buffer)
{
	float pid_output; // 缓冲 PID 输出（W）。
	float adjusted_limit; // 修正后的可用功率预算（W）。
	float effective_target; // 实际生效的缓冲目标（J）。
	float auto_target; // 按功率上限自动推导的缓冲目标上限（J）。

	/* 开关关闭时直接返回基础上限。 */
	if (g_power_buffer_pid_enable == 0)
	{
		g_power_buffer_pid_out = 0.0f;
		return base_power_limit;
	}

	/*
	 * 自动目标逻辑：低功率上限场景下，不允许缓冲目标设得过高，避免长期限扭。
	 * 公式：auto_target = clamp(0.5 * power_limit + 8, 18, 55)
	 */
	auto_target = PowerCtrl_Clampf(0.5f * base_power_limit + 8.0f, 18.0f, 55.0f);
	effective_target = g_power_buffer_target;
	if (effective_target > auto_target)
	{
		effective_target = auto_target;
	}

	/* 缓冲稳定且接近目标时清积分，避免残余积分长期压制动力。 */
	if ((g_power_buffer_drop_rate < 2.0f) && (power_buffer >= (effective_target - 2.0f)))
	{
		PID_Clear(&g_power_buffer_pid);
	}

	/* PID 输入：now=power_buffer，target=effective_target。 */
	PID_Set_Error(&g_power_buffer_pid, power_buffer, effective_target);
	pid_output = PID_coculate(&g_power_buffer_pid);
	g_power_buffer_pid_out = pid_output;

	/* PID 输出正值代表缓冲偏低，需要收紧功率预算。 */
	adjusted_limit = base_power_limit - pid_output;

	/* 掉缓冲越快，额外惩罚越大。 */
	adjusted_limit -= POWER_BUFFER_DROP_GAIN_W_PER_JS * g_power_buffer_drop_rate;

	/* 紧急缓冲区的线性额外收紧。 */
	if (power_buffer < POWER_BUFFER_EMERGENCY_TH)
	{
		adjusted_limit -= (POWER_BUFFER_EMERGENCY_TH - power_buffer) * 1.0f;
	}

	/* 临界缓冲区硬钳制，优先保供电安全。 */
	if (power_buffer < POWER_BUFFER_CRITICAL_TH)
	{
		return PowerCtrl_Clampf(adjusted_limit, 5.0f, 10.0f);
	}

	/* 常规工况限幅：允许略高于基础上限以提升动态恢复。 */
	return PowerCtrl_Clampf(adjusted_limit, 5.0f, base_power_limit + 20.0f);
}

/*
 * @brief 对 LQR 的关键观测量进行门控缩放。
 * @param body_distance_error 车体位移误差（输入/输出）。
 * @param speed_error 速度误差（输入/输出）。
 * @param yaw_error 偏航误差（输入/输出）。
 * @param d_yaw 偏航角速度（输入/输出）。
 */
void PowerCtrl_ApplyObserverGate(float *body_distance_error,
                                 float *speed_error,
                                 float *yaw_error,
                                 float *d_yaw)
{
	PowerObsInput in; // 门控前观测量。
	PowerObsOutput out; // 门控后观测量。
	float speed_mag; // 当前车速幅值（m/s）。
	float min_brake_gain; // 刹车工况 speed_error 的最小保留比例。

	/* 步骤1：空指针保护。 */
	if ((body_distance_error == NULL) || (speed_error == NULL) || (yaw_error == NULL) || (d_yaw == NULL))
	{
		return;
	}

	/* 步骤2：开关关闭或未初始化时直接旁路。 */
	if ((g_power_ctrl_enable == 0) || (g_power_obs_gate_enable == 0) || (g_power_obs_inited == 0))
	{
		return;
	}

	/* 步骤3：组装门控输入。 */
	in.body_distance_error = *body_distance_error;
	in.speed_error = *speed_error;
	in.yaw_error = *yaw_error;
	in.d_yaw = *d_yaw;

	/* 步骤4：执行门控缩放。 */
	PowerObsCtrl_Apply(&g_power_obs_ctrl, &in, &out);

	/*
	 * 步骤5：低缓冲“保刹车”非对称逻辑（新增核心行为）。
	 * 判据：speed_error = target - current。
	 * 若 speed_error 与 current_speed 异号，表示目标是减速，此时应保留更大速度误差，
	 * 避免由于统一缩放导致“想刹但刹不住”。
	 */
	speed_mag = fabsf(kalman_body_speed);
	if ((in.speed_error * kalman_body_speed < 0.0f) && (speed_mag > 0.2f))
	{
		/* 车速越高，刹车保留比例越高。 */
		min_brake_gain = 0.70f + 0.30f * PowerCtrl_Clampf(speed_mag / 2.0f, 0.0f, 1.0f);
		if (fabsf(out.speed_error) < fabsf(in.speed_error) * min_brake_gain)
		{
			out.speed_error = in.speed_error * min_brake_gain;
		}
	}

	/* 步骤6：写回门控结果。 */
	*body_distance_error = out.body_distance_error;
	*speed_error = out.speed_error;
	*yaw_error = out.yaw_error;
	*d_yaw = out.d_yaw;

	/* 步骤7：同步对外调试量。 */
	g_power_obs_lambda = out.lambda;
}










/**
 * 此函数内部使用的局部变量
 */
float wl; // 左轮速度（rad/s）。
float wr; // 右轮速度（rad/s）。
float il; // 左轮电流指令值（-16384~16384）。
float ir; // 右轮电流指令值（-16384~16384）。
float base_power_limit; // 裁判基础功率上限（W）。
float power_limit; // PID 修正后功率上限（W）。
float power_buffer; // 当前缓冲值（J）。
float measured_power; // 实测底盘功率（W），用于 RLS 更新。
float measured_loss; // 由实测功率反推的损耗功率样本（W）。
/*
 * @brief 轮组功率控制主函数。
 * @note 本函数在每个控制周期更新：预测功率、可用功率预算、观测门控系数。//!说白了最后改的是门控系数，前面都是为了更准确地计算这个系数做的准备工作。
 */
void Whellv1PowerCtral(void)
{
	/* 步骤1：首次进入时执行初始化。 */
	if (g_power_obs_inited == 0)
	{
		PowerCtralInit(&whell_power);
	}

	/* 步骤2：总开关关闭时直接旁路，门控系数复位为 1。 */
	if (g_power_ctrl_enable == 0)
	{
		g_power_obs_ctrl.lambda = 1.0f;
		g_power_obs_lambda = 1.0f;
		return;
	}

	/* 步骤3：采集轮组实时状态（速度与电流指令）。 */
	wl = L_LK9025.Rx_Data.Velocity;
	wr = R_LK9025.Rx_Data.Velocity;
	il = PowerCtrl_TorqueToDJICurrent(L_LK9025.Target_Torque);
	ir = PowerCtrl_TorqueToDJICurrent(R_LK9025.Target_Torque);

	/* 步骤4：更新用于观测与建模的中间量。 */
	whell_power.SumPowerSpeed = wl * wl + wr * wr;
	whell_power.SumPowerTorque = il * il + ir * ir;
	whell_power.EffetivePower = whell_power.toque_coefficient * (wl * il + wr * ir);//有效做功

	/* 步骤5：先用当前模型参数计算预测功率。 */
	whell_power.PredictPower = PowerControl_WheelPowerPredict(wl, wr, il, ir, &whell_power);
	powerPredict = whell_power.PredictPower;

	/*
	 * 步骤6：RLS 在线更新（新增）。
	 * 6.1 读取裁判实测功率；
	 * 6.2 将实测总功率减去速度-电流耦合功率项，得到损耗项样本 y；
	 * 6.3 调用 RLS 更新参数向量；
	 * 6.4 立即用新参数重算预测功率，提升当前周期门控精度。
	 */
	measured_power = PowerCtrl_GetMeasuredChassisPower();
	whell_power.MeasurePower = measured_power;
	if (measured_power >= POWER_MEASURE_VALID_MIN_W)
	{
		measured_loss = measured_power - whell_power.EffetivePower;
		measured_loss = PowerCtrl_Clampf(measured_loss, 0.0f, POWER_MEASURE_VALID_MAX_W);

		//模型拟合的时候就要除2
		PowerControl_AutoUpdateParam(whell_power.SumPowerSpeed / 2,
		                            whell_power.SumPowerTorque / 2,
		                            1.0f,
		                            measured_loss / 2,
		                            whell_power);

		whell_power.PredictPower = PowerControl_WheelPowerPredict(wl, wr, il, ir, &whell_power);
		powerPredict = whell_power.PredictPower;
	}

	/* 步骤7：读取裁判功率上限与缓冲。 */
	base_power_limit = PowerCtrl_GetPowerLimit();
	power_buffer = PowerCtrl_GetPowerBuffer();

	/* 步骤8：用缓冲 PID + 保护逻辑计算 //!最大功率。 */
	power_limit = PowerCtrl_GetPidAdjustedLimit(base_power_limit, power_buffer);
	whell_power.MaxPowerLimit = (uint16_t)base_power_limit;
	whell_power.InputPower = power_limit;

	//上交思路
	/* 步骤9：根据预算/缓冲/预测功率计算门控系数 lambda。 */
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

	//港科思路
	
}



/*
 * @brief 功率控制总入口。
 */
void PowerCtrl(void)
{
	Whellv1PowerCtral();
}
