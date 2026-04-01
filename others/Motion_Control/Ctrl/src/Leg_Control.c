// 本文件实现了机器人腿部动力学的控制逻辑，包括基于PID的腿长和转动速度控制，
// 以及检测和处理卡住情况的机制。每个函数都设计为与VMC（虚拟模型控制）结构交互。

#include "Leg_Control.h"
#include <math.h>
#include "Angle_about.h"

// 全局变量，用于控制参数和状态。


// float torque_limit = 0.0f;  // 动态调整的转动力矩限制，防止过大力矩。

// // 卡住检测和控制调节的参数。
// float torque_release_rate = 0.1f;  // 卡住解除时转动力矩限制的释放速率。

user_pid_t speed_PID[2]; // 用于转动速度的PID控制器（每条腿一个）。
user_pid_t L0_PID[2];    // 用于腿长的PID控制器（每条腿一个）。
// float s_torque_limit[2] = {0.0f, 0.0f}; // 每条腿的转动力矩限制。
int s_l0_stuck_counter[2] = {0, 0}; // 每条腿的腿长卡住计数器。
int s_turn_stuck_counter[2] = {0, 0}; // 每条腿的转动卡住计数器。
uint8_t s_pid_inited[2] = {0, 0}; // 标志PID控制器是否已初始化（每条腿一个）。

float s_ramped_torque[2] = {0.0f, 0.0f};    // 转矩斜坡限幅
float s_ramped_F0[2] = {0.0f, 0.0f};        // 腿长斜坡限幅



// 辅助函数：获取VMC结构左右腿（左腿为0，右腿为1）。
static int leg_ctrl_get_idx(VMC_t *VMC)
{
    return (VMC == &VMC_R) ? 1 : 0;
}

int get_sign_float(float value)
{
    if (value > 0.0f)
    {
        return 1;
    }
    else if (value < 0.0f)
    {
        return -1;
    }
    else
    {
        return 0;
    }
}

// 腿部PID控制器初始化
static void leg_ctrl_pid_init(VMC_t *VMC)
{
    int idx = leg_ctrl_get_idx(VMC);

    if (s_pid_inited[idx] != 0)
    {
        return; // PID控制器已初始化。
    }

    // 使用默认参数初始化PID控制器。
    PID_INIT(&speed_PID[idx], 5.0f, 0.05f, 1.0f, 10.0f, 20.0f, 2000.0f, 0.0f);    //TODO: 调参
    PID_INIT(&L0_PID[idx], 2000.0f, 0.0f, 25000.0f, 150.0f, 0.0f, 0.0f, 0.0f);       //TODO: 调参
    s_pid_inited[idx] = 1; // 标记PID控制器已初始化。
}

/**
 * @brief 使用PID控制器和可选的斜坡发生器控制腿长，ramp_rate参数控制斜坡发生器的速率，如果为0.0f则不启用斜坡处理
 * 
 * @param VMC 
 * @param target_L0 
 * @param ramp_rate 周期增量，非负数，值为0.0f表示不启用斜坡处理
 * @param F0_max  非负数
 * @return float 
 */
float leg_length_control(VMC_t *VMC, float target_L0, float ramp_rate, float F0_max)
{
    if (ramp_rate < 0.0f)
	{
		ramp_rate = -ramp_rate;
    }
    if (F0_max < 0.0f)
	{
		F0_max = -F0_max;
	}
    int idx = leg_ctrl_get_idx(VMC);
    float f0_raw;// 原始计算的F0值，未经斜坡处理。
    float f0_cmd;

    //控腿函数PID控制器初始化vscode://lirentech.file-ref-tags?filePath=Leg_Control.c&snippet=%2F%2F%E6%8E%A7%E8%85%BF%E5%87%BD%E6%95%B0PID%E6%8E%A7%E5%88%B6%E5%99%A8%E5%88%9D%E5%A7%8B%E5%8C%96
    leg_ctrl_pid_init(VMC);

    // 控腿函数配置PID控制器的限制vscode://lirentech.file-ref-tags?filePath=Leg_Control.c&snippet=%2F%2F+%E6%8E%A7%E8%85%BF%E5%87%BD%E6%95%B0%E9%85%8D%E7%BD%AEPID%E6%8E%A7%E5%88%B6%E5%99%A8%E7%9A%84%E9%99%90%E5%88%B6
    PID_Reset_OutLimit(&L0_PID[idx], F0_max);//输出限制

    // 计算控制误差并计算原始F0值vscode://lirentech.file-ref-tags?filePath=Leg_Control.c&snippet=%2F%2F+%E8%AE%A1%E7%AE%97%E6%8E%A7%E5%88%B6%E8%AF%AF%E5%B7%AE%E5%B9%B6%E8%AE%A1%E7%AE%97%E5%8E%9F%E5%A7%8BF0%E5%80%BC
    PID_Set_Error(&L0_PID[idx], VMC->L0, target_L0);
    f0_raw = PID_coculate(&L0_PID[idx]);

    // 如果启用了斜坡发生器，则应用斜坡处理vscode://lirentech.file-ref-tags?filePath=Leg_Control.c&snippet=%2F%2F+%E5%A6%82%E6%9E%9C%E5%90%AF%E7%94%A8%E4%BA%86%E6%96%9C%E5%9D%A1%E5%8F%91%E7%94%9F%E5%99%A8%EF%BC%8C%E5%88%99%E5%BA%94%E7%94%A8%E6%96%9C%E5%9D%A1%E5%A4%84%E7%90%86
    if (ramp_rate != 0.0f)
    {
        // 斜坡累加vscode://lirentech.file-ref-tags?filePath=Leg_Control.c&snippet=%2F%2F+%E6%96%9C%E5%9D%A1%E7%B4%AF%E5%8A%A0
        if (fabsf(f0_raw) > fabsf(s_ramped_F0[idx]))
        {
            s_ramped_F0[idx] += ramp_rate;
        }
        else
        {
            s_ramped_F0[idx] = fabsf(f0_raw);
        }

        // 斜坡限幅限制在F0_max内 vscode://lirentech.file-ref-tags?filePath=Leg_Control.c&snippet=%2F%2F+%E6%96%9C%E5%9D%A1%E9%99%90%E5%B9%85%E9%99%90%E5%88%B6%E5%9C%A8F0_max%E5%86%85
        if (s_ramped_F0[idx] > F0_max)
        {
            s_ramped_F0[idx] = F0_max;
        }
    }
    else
    {
        if (f0_raw > F0_max)
        {
            f0_raw = F0_max;
        }
        if (f0_raw < -F0_max)
        {
            f0_raw = -F0_max;
        }
        return f0_raw; // 如果不启用斜坡处理，直接返回原始计算的F0值。
    }

    // 限幅得f0_cmd vscode://lirentech.file-ref-tags?filePath=Leg_Control.c&snippet=%2F%2F+%E9%99%90%E5%B9%85%E5%BE%97f0_cmd
    f0_cmd = f0_raw;
    if (f0_cmd > s_ramped_F0[idx])
    {
        f0_cmd = s_ramped_F0[idx];
    }
    else if (f0_cmd < -s_ramped_F0[idx])
    {
        f0_cmd = -s_ramped_F0[idx];
    }

    return f0_cmd;
}

/**
 * @brief 使用PID控制器和可选斜坡发生器控制腿部的转动速度
 * 
 * @param VMC 
 * @param target_speed 
 * @param max_torque 非负数
 * @param ramp_rate 周期增量，非负数，值为0.0f表示不启用斜坡处理
 * @return float 
 */
float leg_turn_speed_control(VMC_t *VMC, float target_speed, float max_torque, float ramp_rate)
{
    if (max_torque < 0.0f)
	{
		max_torque = -max_torque;
    }
    if (ramp_rate < 0.0f)
	{
		ramp_rate = -ramp_rate;
	}
    int idx = leg_ctrl_get_idx(VMC);
    float torque_raw;
    float torque_cmd;

    // 腿部的转动PID控制器初始化vscode://lirentech.file-ref-tags?filePath=Leg_Control.c&snippet=%2F%2F+%E8%85%BF%E9%83%A8%E7%9A%84%E8%BD%AC%E5%8A%A8PID%E6%8E%A7%E5%88%B6%E5%99%A8%E5%88%9D%E5%A7%8B%E5%8C%96
    leg_ctrl_pid_init(VMC);

    // 配置PID控制器的限制。
    PID_Reset_OutLimit(&speed_PID[idx], max_torque); // 输出限制

    // 计算控制误差并计算原始torque值vscode://lirentech.file-ref-tags?filePath=Leg_Control.c&snippet=%2F%2F+%E8%AE%A1%E7%AE%97%E6%8E%A7%E5%88%B6%E8%AF%AF%E5%B7%AE%E5%B9%B6%E8%AE%A1%E7%AE%97%E5%8E%9F%E5%A7%8Btorque%E5%80%BC
    PID_Set_Error(&speed_PID[idx], VMC->d_phi0, target_speed);//target - now
    torque_raw = PID_coculate(&speed_PID[idx]);

    // 是否启用斜坡发生器
    if (ramp_rate != 0.0f)
    {
        // 斜坡累加
        if (fabsf(torque_raw) > s_ramped_torque[idx])
        {
            s_ramped_torque[idx] += ramp_rate;

        }
        else
        {
            s_ramped_torque[idx] = torque_raw;
        }
        // 斜坡限幅限制在max_torque内
        if (s_ramped_torque[idx] > max_torque)
        {
            s_ramped_torque[idx] = max_torque;
        }
    }
    else
    {
        if (torque_raw > max_torque)
        {
            torque_raw = max_torque;
        }
        if (torque_raw < -max_torque)
        {
            torque_raw = -max_torque;
        }
        return torque_raw; // 如果不启用斜坡处理，直接返回原始计算的torque值。
    }

    torque_cmd = torque_raw;
    // 将转动力矩命令限制在最大允许范围内。
    if (torque_cmd > s_ramped_torque[idx])
    {
        torque_cmd = s_ramped_torque[idx];
    }
    else if (torque_cmd < -s_ramped_torque[idx])
    {
        torque_cmd = -s_ramped_torque[idx];
    }

    return torque_cmd;
}

/**
 * @brief 根据当前和上一次的腿长差值检测腿长是否卡住
 * 
 * @param VMC 
 * @param L0_stuck 长度差值的死区，单位m
 * @return int 
 */
int leg_length_stuck_detect(VMC_t *VMC, float L0_stuck, float stuck_counter_time_s)
{
    int idx = leg_ctrl_get_idx(VMC);

    // 检查腿长变化是否低于卡住阈值
    if (fabsf(VMC->L0 - VMC->last_L0) < L0_stuck)
    {
        s_l0_stuck_counter[idx]++;
    }
    else
    {
        s_l0_stuck_counter[idx] = 0;
    }

    return (s_l0_stuck_counter[idx] > (int)(stuck_counter_time_s * HZ));
}

/**
 * @brief 根据当前和上一次的转动角度差值检测转动是否卡住
 * 
 * @param VMC 
 * @param phi0_stuck 转动速度的死区，单位rad/s
 * @param turn_stuck_counter_time_s 转动卡住计数器的时间阈值，单位s
 * @return int 
 */
int leg_turn_stuck_detect(VMC_t *VMC, float d_phi0_stuck, float turn_stuck_counter_time_s)
{
    int idx = leg_ctrl_get_idx(VMC);

    // 检查转动角度变化是否低于卡住阈值。
    if (fabsf(VMC->d_phi0) < d_phi0_stuck)
    {
        s_turn_stuck_counter[idx]++;
    }
    else
    {
        s_turn_stuck_counter[idx] = 0;
    }

    return (s_turn_stuck_counter[idx] > (int)(turn_stuck_counter_time_s * HZ));
}

// // 高级控制函数：结合腿长和转动速度控制的腿部动力学控制
// void leg_control(VMC_t *VMC, float target_speed, float max_torque, float target_L0)
// {
//     int idx = leg_ctrl_get_idx(VMC);
//     float f_cmd;
//     float t_cmd = 0.0f;
//     float l0_error;

//     // 执行腿长控制，启用斜坡处理。
//     f_cmd = leg_length_control(VMC, target_L0, 1);
//     l0_error = target_L0 - VMC->L0;

//     // 根据腿长误差确定转动力矩。
//     if (fabsf(l0_error) <= 0.01f)
//     {
//         t_cmd = leg_turn_speed_control(VMC, target_speed, max_torque);
//         s_torque_limit[idx] = 0.0f;
//     }
//     else if (leg_length_stuck_detect(VMC))
//     {
//         // 如果腿长卡住，逐步释放转动力矩限制。
//         s_torque_limit[idx] += torque_release_rate * HZ;
//         if (s_torque_limit[idx] > max_torque)
//         {
//             s_torque_limit[idx] = max_torque;
//         }

//         t_cmd = leg_turn_speed_control(VMC, target_speed, max_torque);
//         if (t_cmd > s_torque_limit[idx])
//         {
//             t_cmd = s_torque_limit[idx];
//         }
//         else if (t_cmd < -s_torque_limit[idx])
//         {
//             t_cmd = -s_torque_limit[idx];
//         }
//     }
//     else
//     {
//         s_torque_limit[idx] = 0.0f;
//         t_cmd = 0.0f;
//     }

//     // 更新全局变量，并在VMC结构中设置命令的力和力矩。
//     torque_limit = s_torque_limit[idx];
//     torque = t_cmd;
//     VMC_Set_F0_T(VMC, f_cmd, t_cmd);
// }

