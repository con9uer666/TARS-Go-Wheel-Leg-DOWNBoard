#include "Angle_about.h"

// 将漂移的角度转换到-PI到PI的范围内
//
// @angle_middle 位于转换后0°的原始角度
// @angle_now 当前角度值
//
// @return 转换后的角度，范围在-PI到PI之间
float Angle_Normalize(float angle_middle, float angle_now)
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
