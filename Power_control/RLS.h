#ifndef __RLS__
#define __RLS__

#include "arm_math.h"
#include "PowerCtrl.h"

#ifdef __cplusplus
extern "C" {
#endif

void PowerControl_AutoUpdateParamInit(ChassisPower* power);
float PowerControl_AutoUpdateParam(float x1, float x2, float x3, float y, ChassisPower power);
float PowerControl_ModelLossPredict(float x1, float x2, float x3, const ChassisPower *power);
float PowerControl_WheelPowerPredict(float wl, float wr, float il, float ir, const ChassisPower *power);
float PowerControl_SingleWheelCurrentFromPower(float w, float power_limit, const ChassisPower *power);

#ifdef __cplusplus
}
#endif

#endif
