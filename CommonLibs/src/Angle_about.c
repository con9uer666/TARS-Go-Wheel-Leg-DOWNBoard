// Created by Speros987

#include "Angle_about.h"

// -PI到PI范围角度转换，将设定点标定为0点
//
// @angle_middle 位于转换后0°的原始角度
// @angle_now 当前角度值
//
// @return 转换后的角度
float easy_angle_normalize(float angle_middle, float angle_now)
{
    float angle_out = 0.0f;
    if(angle_middle >= 0)
    {
        if(angle_now > -PI && angle_now <= (angle_middle - PI))
        {
            angle_out = angle_now + 2 * PI - angle_middle;
        }
        else
        {
            angle_out = angle_now - angle_middle;
        }

        return angle_out;
    }
    else
    {
        // 负半轴核心逻辑：处理角度漂移到正区间的情况，反向映射回-PI~PI
        if(angle_now < PI && angle_now >= (angle_middle + PI))
        {
            angle_out = angle_now - 2 * PI - angle_middle;
        }
        else
        {
            angle_out = angle_now - angle_middle;
        }

        return angle_out;
    }
}

//[-PI, PI]套圈处理
//
//@raw_angle 原始角度值
//
//@return 转换后的角度值，范围在[-PI, PI]内
float easy_angle_wrapping(float raw_angle)
{
    // 将raw_angle转换到[-PI, PI]范围内
    while (raw_angle > PI)
    {
        raw_angle -= 2.0f * PI;
    }
    while (raw_angle < -PI)
    {
        raw_angle += 2.0f * PI;
    }
    return raw_angle;
}

//弧度制计算两角度角度最小差值，得到的是左腿-右腿的角度差值，范围在[-PI, PI]内
// 
// 正值为坐标系内左腿正转绝对值到右腿，负值为坐标系内左腿负转绝对值到右腿
float calculate_angle_diff_double_direction(float left_leg_angle, float right_leg_angle)
{
    float diff = left_leg_angle - right_leg_angle;
    // 将diff转换到[-PI, PI]范围内
    while (diff > PI)
    {
        diff -= 2.0f * PI;
    }
    while (diff < -PI)
    {
        diff += 2.0f * PI;
    }
    return diff;
}

//弧度制计算单旋转方向下 坐标系正转 两角度差值，得到的是左腿-右腿的角度差值，范围在[0, 2PI]内
float calculate_angle_diff_single_direction(float left_leg_angle, float right_leg_angle)
{
    float diff = left_leg_angle - right_leg_angle;
    // 将diff转换到[0, 2PI]范围内
    while (diff < 0)
    {
        diff += 2.0f * PI;
    }
    while (diff >= 2.0f * PI)
    {
        diff -= 2.0f * PI;
    }
    return diff;
}