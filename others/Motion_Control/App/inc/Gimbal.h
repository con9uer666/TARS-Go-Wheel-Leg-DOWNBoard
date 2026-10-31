#ifndef GIMBAL_H
#define GIMBAL_H

#include "main.h"

/* 云台总开关：0=关闭云台控制和4310输出，1=启用云台 */
extern uint8_t gimbal_enable;

extern float yaw_angle_PI;//标零处理后的yaw角度，单位rad，范围在[-PI, PI]内
extern float head_forward_angle;//正视前方的yaw电机角度

#endif // GIMBAL_H
