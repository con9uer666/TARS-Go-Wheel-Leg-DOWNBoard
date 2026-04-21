#ifndef _POWERCTRAL_H_
#define _POWERCTRAL_H_

#include "main.h"

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

#define TOQUE_CONST  600 // 电机扭矩系数

typedef struct
{
	float output;
	float LastOutput[4];      // 上一次的输出，也就是当前的电机功率值
	float SumPowerSpeed;      // 转速平方和
	float SumPowerTorque;     // 扭矩平方和
	float EffetivePower;      // 机械功率
	float InitialGivePower[4];// 分配前功率
	float InitialTotalPower;  // 分配前总功率
	float PredictPower;       // 预测功率
	float MeasurePower;       // 测量功率
	float TotalPower;         // 总功率
	float scaleFactor;        // 缩放系数
	float paramVector[3][1];  // 动态模型初始值参数
	float transVector[3][3];  // 动态矩阵（用于参数变化范围）
	float toque_coefficient;  // 单位转速对应的扭矩系数（A/(rad/s)）
	float a;                  // 转速平方系数
	float k2;                 // 转矩平方系数
	float constant;           // 常数项
	float kp;                 // 平步反解系数
	float sdmax;              // 最大限制值（可能是速度/功率上限）
	uint16_t moto_type;       // 电机类型
	uint16_t UserPowerLimit;  // 用户功率限制
	uint16_t MaxPowerLimit;   // 最大功率限制
	float InputPower;         // 输入功率
}ChassisPower;

extern ChassisPower whell_power;

/* 运行时开关：可在调试中动态启停模块。 */
extern uint8_t g_power_ctrl_enable;
extern uint8_t g_power_obs_gate_enable;
extern uint8_t g_power_buffer_pid_enable;

/* 当前观测量缩放系数，便于上位机观测。 */
extern float g_power_obs_lambda;

/* 缓冲能量 PID 闭环调试量。 */
extern float g_power_buffer_target;
extern float g_power_buffer_measure;
extern float g_power_buffer_pid_out;

void PowerCtralInit(ChassisPower* whell_power);
void PowerCtrl();

/* 运行时开关控制接口。 */
void PowerCtrl_SetEnable(uint8_t enable);
void PowerCtrl_SetObserverGateEnable(uint8_t enable);
void PowerCtrl_SetBufferPidEnable(uint8_t enable);
void PowerCtrl_SetBufferTarget(float target);

/*
 * 对 LQR 观测量应用功率门控。
 * 建议在 LQR_calculate() 内调用。
 */
void PowerCtrl_ApplyObserverGate(float *body_distance_error,
								 float *speed_error,
								 float *yaw_error,
								 float *d_yaw);

#endif