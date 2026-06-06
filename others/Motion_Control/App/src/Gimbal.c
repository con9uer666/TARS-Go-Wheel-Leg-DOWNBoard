/**
 * @file Gimbal.c
 * @brief 云台偏航控制任务：执行云台归位 + 底盘跟随/小陀螺模式切换。
 *
 * 在 gimbal_follow_flag==1 时驱动云台回中，到位后切回底盘跟随云台模式。
 */

#include "Gimbal.h"
#include "cmsis_os.h"
#include "User_State.h"
#include "Angle_about.h"
#include "Motor_Drv.h"
#include "chassis_behavior_tree.h"

/** @brief 云台偏航角度环 PID 控制器。 */
user_pid_t gimbal_yaw_angle_pid;

/**
 * @brief 标零处理后的 yaw 角度 (rad)，范围 [-PI, PI]。
 *
 * 以 head_forward_angle 为基准，调用 easy_angle_normalize() 归一化。
 */
float yaw_angle_PI = 0.0f;

/** @brief 正视前方的 yaw 电机角度 (rad)，见 motor.c 定义。 */
float head_forward_angle = -2.773;

/** @brief 云台跟随底盘归位完成计数器，计数满 100 次（约 200ms）即完成。 */
uint16_t gimbal_follow_flag_cnt = 0;

/** @brief 用户调试变量 e：记录归位成功次数。 */
uint16_t user_e = 0;

/** @brief 用户调试变量 f：记录归位控制帧数。 */
uint16_t user_f = 0;

/**
 * @brief 云台控制任务（FreeRTOS 任务）。
 *
 * 以 500 Hz 频率执行：
 *   - 更新 yaw_angle_PI（基于编码器位置标零归一化）
 *   - 若 gimbal_follow_flag==1：PID 驱动云台回中，成功 100 帧后置 gimbal_follow_flag=0
 *   - 若 gimbal_follow_flag==0：直接传递 Yaw_DM4310.Target_Speed 给底盘
 *
 * @param argument 未使用（FreeRTOS 标准接口要求）。
 */
void Gimbal_task(void const * argument)
{
    PID_INIT(&gimbal_yaw_angle_pid, 20, 0.01,1, 10, 0.5, 0, 0.3, 0);
    TickType_t xLastWakeTime = xTaskGetTickCount(); 
    for(;;)
    {
        //计算标零后角度值
        yaw_angle_PI = easy_angle_normalize(head_forward_angle, Yaw_DM4310.Rx_Data.Position);

        if(gimbal_follow_flag == 1)
        {
            //云台归位
            PID_Set_Error(&gimbal_yaw_angle_pid, yaw_angle_PI, 0);
            down_board_yaw_output = PID_coculate(&gimbal_yaw_angle_pid);

            if(fabsf(yaw_angle_PI) <= 0.03f)
            {
                gimbal_follow_flag_cnt ++;

                user_f ++;
            }
            if(gimbal_follow_flag_cnt >= 100)
            {
                gimbal_follow_flag = 0;//云台跟随底盘完成，切换到底盘跟随云台
                gimbal_follow_flag_cnt = 0;

                user_e ++;
            }
        }
        else
        {
            down_board_yaw_output = Yaw_DM4310.Target_Speed;

            user_f = 0;
            user_e = 0;
        }

        osDelayUntil(&xLastWakeTime, 2);//精确延时2毫秒，同时更新xLastWakeTime的值为当前时间
    }
}