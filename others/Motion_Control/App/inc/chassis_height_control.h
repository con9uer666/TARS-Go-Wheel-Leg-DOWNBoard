/**
 * @file chassis_height_control.h
 * @brief 机身高度的控制接口：横滚补偿和腿长控制
 */

#ifndef CHASSIS_HEIGHT_CONTROL_H
#define CHASSIS_HEIGHT_CONTROL_H

#include "main.h"

void Roll_Comp();
void Leg_L0_Control();

extern float ramp_target_L0_up;
extern float ramp_target_L0_down;

#endif // CHASSIS_HEIGHT_CONTROL_H