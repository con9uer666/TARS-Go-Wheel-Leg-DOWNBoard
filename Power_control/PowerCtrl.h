#ifndef _POWERCTRAL_H_
#define _POWERCTRAL_H_

#include "main.h"

#define TOQUE_CONST  600

typedef struct
{
	float toque_coefficient;   // 有效做功系数
	float PredictPower;        // 本帧预测功率（= 上一帧功率计读数，W）
	float MeasurePower;        // 本帧功率计读数（W）
	uint16_t moto_type;        // 电机型号标识
	uint16_t UserPowerLimit;   // 裁判数据异常时的回退功率上限（W）
	uint16_t MaxPowerLimit;    // 本周期裁判基础功率上限（W）
	float InputPower;          // 本周期计算得到的 power_limit（W）
} ChassisPower;

extern ChassisPower whell_power;

extern uint8_t g_power_ctrl_enable;
extern uint8_t g_power_obs_gate_enable;

extern float g_power_obs_lambda;

void PowerCtralInit(ChassisPower* whell_power);
void PowerCtrl(void);

void PowerCtrl_SetEnable(uint8_t enable);
void PowerCtrl_SetObserverGateEnable(uint8_t enable);

void PowerCtrl_ApplyObserverGate(float *body_distance_error,
								 float *speed_error,
								 float *yaw_error,
								 float *d_yaw);

#endif
