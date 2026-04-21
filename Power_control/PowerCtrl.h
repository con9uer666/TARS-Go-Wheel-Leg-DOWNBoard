#ifndef _POWERCTRAL_H_
#define _POWERCTRAL_H_

#include "struct_typedef.h"

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
void PowerCtrl();

#endif