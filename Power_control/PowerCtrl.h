#ifndef _POWERCTRAL_H_
#define _POWERCTRAL_H_

#include "main.h" // uint8_t/uint16_t 等基础类型。

/* 编译期总开关：0=关闭功率控制逻辑，1=启用。 */
#ifndef POWER_CTRL_MODULE_ENABLE
#define POWER_CTRL_MODULE_ENABLE 1
#endif

/* 编译期观测量门控开关：0=不缩放 LQR 观测量，1=启用。 */
#ifndef POWER_CTRL_OBSERVER_GATE_ENABLE
#define POWER_CTRL_OBSERVER_GATE_ENABLE 1
#endif

/* 编译期缓冲能量 PID 闭环开关：0=关闭，1=启用。 */
#ifndef POWER_CTRL_BUFFER_PID_ENABLE
#define POWER_CTRL_BUFFER_PID_ENABLE 1
#endif

#define TOQUE_CONST  600 // 历史保留系数宏（当前功率控制主链路未直接使用）。

/*
 * @brief 轮组功率控制对象。
 * @note 该结构体同时承载：
 * 1) 功率预测模型参数与中间量；
 * 2) 功率预算与测量值；
 * 3) 调试观察量。
 */
typedef struct
{
	float output; // 历史保留输出字段（当前主链路未使用）。
	float LastOutput[4]; // 历史保留：上周期输出缓存。
	float SumPowerSpeed; // 当前周期轮速平方和：wl^2 + wr^2。
	float SumPowerTorque; // 历史命名，当前语义为轮电流指令平方和：il^2 + ir^2。
	float EffetivePower; // 历史命名，当前语义为速度-电流耦合功率项：k_i * (wl*il + wr*ir)。
	float InitialGivePower[4]; // 历史保留：分配前功率。
	float InitialTotalPower; // 历史保留：分配前总功率。
	float PredictPower; // 当前预测总电功率（W）。
	float MeasurePower; // 当前实测底盘功率（W）。
	float TotalPower; // 历史保留：总功率。
	float scaleFactor; // 历史保留：缩放因子。
	float paramVector[3][1]; // 功率模型参数向量 [a b c]^T。
	float transVector[3][3]; // RLS 协方差矩阵 P。
	float toque_coefficient; // 历史命名，当前语义为速度-电流耦合系数 k_i。
	float a; // 历史保留参数。
	float k2; // 历史保留参数。
	float constant; // 历史保留参数。
	float kp; // 历史保留参数。
	float sdmax; // 历史保留参数。
	uint16_t moto_type; // 电机型号标识。
	uint16_t UserPowerLimit; // 用户默认功率限制（裁判数据异常时作为回退）。
	uint16_t MaxPowerLimit; // 本周期基础功率限制（通常来自裁判）。
	float InputPower; // 本周期 PID+保护修正后的可用功率预算（W）。
}ChassisPower;

extern ChassisPower whell_power; // 全局轮组功率对象。

/* 运行时开关：可在调试中动态启停模块。 */
extern uint8_t g_power_ctrl_enable;
extern uint8_t g_power_obs_gate_enable;
extern uint8_t g_power_buffer_pid_enable;

/* 当前观测量缩放系数，便于上位机观测。 */
extern float g_power_obs_lambda; // 当前门控系数镜像值（用于上位机观测）。

/* 缓冲能量 PID 闭环调试量。 */
extern float g_power_buffer_target; // 缓冲目标值（J）。
extern float g_power_buffer_measure; // 缓冲测量值（J）。
extern float g_power_buffer_pid_out; // 缓冲 PID 输出（W）。
extern float g_power_buffer_drop_rate_alpha; // 缓冲掉速估计低通系数。

/*
 * @brief 功率控制初始化函数。
 * @param whell_power 轮组功率对象指针，初始化模型参数与控制状态。
 */
void PowerCtralInit(ChassisPower* whell_power);

/*
 * @brief 功率控制总入口。
 * @note 建议每个控制周期调用一次。
 */
void PowerCtrl();

/* 运行时开关控制接口。 */
void PowerCtrl_SetEnable(uint8_t enable); // 运行时总开关。
void PowerCtrl_SetObserverGateEnable(uint8_t enable); // 运行时门控开关。
void PowerCtrl_SetBufferPidEnable(uint8_t enable); // 运行时缓冲 PID 开关。
void PowerCtrl_SetBufferTarget(float target); // 运行时调整缓冲目标（J）。

/*
 * @brief 对 LQR 观测量应用功率门控。
 * @param body_distance_error 车体位移误差（输入/输出）。
 * @param speed_error 速度误差（输入/输出）。
 * @param yaw_error 偏航误差（输入/输出）。
 * @param d_yaw 偏航角速度（输入/输出）。
 * @note 建议在 LQR_calculate() 入口处调用。
 */
void PowerCtrl_ApplyObserverGate(float *body_distance_error,
								 float *speed_error,
								 float *yaw_error,
								 float *d_yaw);

#endif