#ifndef WHEEL_LEG_ABOUT_H
#define WHEEL_LEG_ABOUT_H

#include "main.h"

void LQR_Get_K(float LQR[4][10], float K_Fit_Coefficients[40][6], float L0_l, float L0_r);
void AntiSplit_Get_K(float K[2], float Coef[2][3], float L0_avg);
void Roll_Comp();
void Leg_L0_Control();
void Speed_Error_Set();
void Distance_Error_Set();
void Body_Speed_Coculate();
void INS_Coculate();
void Yaw_Error_Coculate();

#endif // WHEEL_LEG_ABOUT_H