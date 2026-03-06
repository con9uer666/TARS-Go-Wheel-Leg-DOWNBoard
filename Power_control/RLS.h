#ifndef __RLS__
#define __RLS__
#include "arm_math.h"
#include "arm_math.h"
#include "PowerCtrl.h"
 
void PowerControl_AutoUpdateParamInit(ChassisPower* power);
float PowerControl_AutoUpdateParam(float x1, float x2, float x3, float y,ChassisPower power);


#endif
