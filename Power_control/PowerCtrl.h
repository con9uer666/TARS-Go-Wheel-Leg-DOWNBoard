#ifndef _POWERCTRAL_H_
#define _POWERCTRAL_H_

#include "main.h"

#define TOQUE_CONST  600

typedef struct
{
	float toque_coefficient;   // 有效做功系数
	float PredictPower;        // 功率计低通滤波后的值（W）
	float MeasurePower;        // 功率计原始读数（W）
	uint16_t moto_type;        // 电机型号标识
	uint16_t UserPowerLimit;   // 裁判数据异常时的回退功率上限（W）
	uint16_t MaxPowerLimit;    // 本周期裁判基础功率上限（W）
	float InputPower;          // 本周期经缓冲插值后的功率上限（W）
} ChassisPower;

extern ChassisPower whell_power;

extern uint8_t g_power_ctrl_enable; // 总开关：1=启用，0=旁路（lambda 固定为 1）

// lambda：目标速度/转速缩放系数（0~1），超功率时由 PID+低通 计算，不超功率时为 1
extern float g_power_obs_lambda;

extern float g_filtered_power; // 功率计低通滤波后的值，用于超功率判断
extern float power_limit;      // 本周期允许的最大功率（W），用于超功率判断

void  PowerCtralInit(ChassisPower *whell_power);
void  PowerCtrl(void);

// 限制目标速度：超功率时乘 lambda，否则直接返回原值（仍做幅值限幅）
float PowerCtrl_LimitTargetSpeed(float raw_target_speed, float speed_limit);

#endif
