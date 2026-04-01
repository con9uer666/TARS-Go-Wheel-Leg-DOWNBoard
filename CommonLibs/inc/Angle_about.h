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

typedef struct
{
    float X;
    float Y;
    float d_X;
    float d_Y;
    float dd_X;
    float dd_Y;
    uint8_t leg_id; // 0为左腿，1为右腿
} Cartesian_Def;

typedef struct
{
    float angle; // 极坐标角度，单位为弧度
    float d_angle; // 角速度，单位为弧度每秒
    float dd_angle; // 角加速度，单位为弧度每秒平方
    float r;     // 极坐标半径，单位为米
    float d_r;   // 半径速度，单位为米每秒
    float dd_r;  // 半径加速度，单位为米每秒平方
    uint8_t leg_id; // 0为左腿，1为右腿
} Polar_Def;

float easy_angle_normalize(float angle_middle, float angle_now);
float easy_angle_wrapping(float raw_angle);
float calculate_angle_diff_single_direction(float left_leg_angle, float right_leg_angle);
float calculate_angle_diff_double_direction(float left_leg_angle, float right_leg_angle);
void Cartesian_to_Polar(Cartesian_Def *cartesian, Polar_Def *polar);
void Polar_to_Cartesian(Polar_Def *polar, Cartesian_Def *cartesian);
void Polar_Get(Polar_Def *polar, float phi0, float L0);

#endif // ANGLE_ABOUT_H