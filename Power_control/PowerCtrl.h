#ifndef _POWERCTRAL_H_
#define _POWERCTRAL_H_

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TOQUE_CONST  600

typedef struct
{
	float output;
	float LastOutput[4];
	float SumPowerSpeed;
	float SumPowerTorque;
	float EffetivePower;
	float InitialGivePower[4];
	float InitialTotalPower;
	float PredictPower;
	float MeasurePower;
	float TotalPower;
	float scaleFactor;
	float paramVector[3][1];
	float transVector[3][3];
	float toque_coefficient;
	float a;
	float k2;
	float constant;
	float kp;
	float sdmax;
	uint16_t moto_type;
	uint16_t UserPowerLimit;
	uint16_t MaxPowerLimit;
	float InputPower;
} ChassisPower;

extern ChassisPower whell_power;

extern uint8_t g_power_ctrl_enable;
extern uint8_t g_power_obs_gate_enable;
extern uint8_t g_power_buffer_pid_enable;

extern float g_power_obs_lambda;
extern float g_power_buffer_target;
extern float g_power_buffer_measure;
extern float g_power_buffer_pid_out;
extern float g_power_buffer_drop_rate_alpha;

void PowerCtralInit(ChassisPower* whell_power);
void PowerCtrl(void);
void PowerCtrl_SetEnable(uint8_t enable);
void PowerCtrl_SetObserverGateEnable(uint8_t enable);
void PowerCtrl_SetBufferPidEnable(uint8_t enable);
void PowerCtrl_SetBufferTarget(float target);
void PowerCtrl_ApplyObserverGate(float *body_distance_error,
								 float *speed_error,
								 float *yaw_error,
								 float *d_yaw);

#ifdef __cplusplus
}
#endif

#endif
