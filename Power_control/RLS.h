#ifndef __RLS__
#define __RLS__
#include "arm_math.h"
#include "arm_math.h"
#include "PowerCtrl.h"
 
void PowerControl_AutoUpdateParamInit(ChassisPower* power);
float PowerControl_AutoUpdateParam(float x1, float x2, float x3, float y,ChassisPower power);

/*
 * 使用当前参数向量计算模型损耗项：p = a*x1 + b*x2 + c*x3。
 */
float PowerControl_ModelLossPredict(float x1, float x2, float x3, const ChassisPower *power);

/*
 * 轮组总电功率预测（含机械功项）。
 * wl/wr: 左右轮角速度(rad/s), tl/tr: 左右轮扭矩(N*m)。
 */
float PowerControl_WheelPowerPredict(float wl, float wr, float tl, float tr, const ChassisPower *power);


#endif
