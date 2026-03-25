#ifndef ANGLE_ABOUT_H
#define ANGLE_ABOUT_H

#include "main.h"
#include <stdint.h>

#define PI 3.1415926f

typedef struct
{
    uint8_t angle_range;    //角度范围，0：-PI到PI，1：0到2PI
    uint8_t angle_unit;     //角度单位，0：弧度，1：度
    float value;            //值
} Angle_Def;

float easy_angle_normalize(float angle_middle, float angle_now);
float easy_angle_wrapping(float raw_angle);
float calculate_angle_diff_single_direction(float left_leg_angle, float right_leg_angle);
float calculate_angle_diff_double_direction(float left_leg_angle, float right_leg_angle);

#endif