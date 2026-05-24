#include "Gimbal.h"
#include "cmsis_os.h"
#include "User_State.h"
#include "Angle_about.h"
#include "Motor_Drv.h"
#include "motor.h"

user_pid_t gimbal_yaw_angle_pid;//云台偏航角度环pid

float yaw_angle_PI = 0.0f;//标零处理后的yaw角度，单位rad，范围在[-PI, PI]内
float head_forward_angle = -0.772610664f;//正视前方的yaw电机角度

uint16_t gimbal_follow_flag_cnt = 0; // 刚站起来云台跟随底盘的计数器

uint16_t user_e = 0; // 用户调试变量e
uint16_t user_f = 0; // 用户调试变量f

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