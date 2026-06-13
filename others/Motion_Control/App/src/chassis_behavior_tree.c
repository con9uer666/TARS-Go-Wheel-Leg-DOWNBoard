/**
 * @file chassis_behavior_tree.c
 * @brief 底盘行为树主循环（原 motor.c 的 Motor_task）。
 *
 * 本文件只负责"行为树"：500Hz 周期里按 start_mode / upstares_mode 调度各动作组，
 * 各动作组 / 各计算的实现已拆分到独立文件（balance_standing / stair_climb /
 * sit_motion / self_righting_retract / spinning_motion / jump_motion /
 * lqr_calculate / off_ground_detect / step_hit_detect / yaw_error / ...）。
 *
 * 状态调度：
 *   start_mode==0 && upstares_mode==0 → NotStanding_NotStairRetract_for_chassis（收腿→起立）
 *   start_mode==1                     → Standing（LQR + 腿长PID + 跳跃）
 *   start_mode==2 && upstares_mode==0 → Upstair_NotStairRetract（上台阶伸腿）
 *   start_mode==3                     → Sit_On_Ground（坐地）
 *   upstares_mode==1                  → StairRetract（上台阶后收腿起立）
 */

#include "chassis_behavior_tree.h"
#include "user_pid.h"
#include "Motor_Drv.h"
#include "Gimbal.h"
#include "User_State.h"
#include "State.h"
#include "arm_math.h"
#include "USER_CAN.h"
#include "VMC.h"
#include "observe_task.h"
#include "Leg_Control.h"
#include "Self_Righting.h"
#include "Board2Board.h"
#include "Slope.h"
#include "Wheel_Leg_about.h"
#include "controller.h"
#include "remoter.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "fdcan.h"
#include <math.h>
#include <stdint.h>
#include "imu_temp_ctrl.h"
#include "Angle_about.h"
#include "PowerCtrl.h"
#include "Gas_Spring.h"
#include "buzzer.h"
#include "Wheel_End_Velocity.h"

float user_gas = 0;

float left_wheel_velocity = 0;
float right_wheel_velocity = 0;
float left_wheel_acceleration = 0;
float right_wheel_acceleration = 0;

uint16_t cnt = 0;

void Motor_task(void const *argument)
{
    task_Motor_Init();
    task_VMC_Init();
    task_PID_Init();
    osDelay(1000);

    task_Pitch_Coculate();
    task_Motor_Enable();

    //精确延时用
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for(;;)
    {
        Wheel_End_Velocity_Both(&left_wheel_velocity, &left_wheel_acceleration, &right_wheel_velocity, &right_wheel_acceleration);

        if(user_Gravity_Compensation_Test_Function_set == 0)
        {
            user_gas = Gas_Spring_GetForce(VMC_L.L0);

            //刚启动收腿过程中
            if(start_mode == 0 && upstares_mode == 0)//未站起 + 未上楼收腿
            {
                NotStanding_NotStairRetract_for_chassis();
                gimbal_follow_flag = 1;
            }

            else if(start_mode == 1)//站起
            {
                Standing();
            }
            else if(start_mode == 2 && upstares_mode == 0)//上楼梯模式 + 未上楼收腿
            {
                Upstair_NotStairRetract();
            }
            else if(start_mode == 3)//坐地模式
            {
                Sit_On_Ground();
            }
            else if(upstares_mode == 1)//收腿起立
            {
                StairRetract();
            }
        }
        if(user_Gravity_Compensation_Test_Function_set == 1)
        {
            if(cnt < 1000)
            {
                cnt++;
            }
            else 
            {
                Gravity_Compensation_Test_Function();
            }

        }

        //错误码蜂鸣器：电机错误时长响低音。倒地自起激活时本函数自动让位，不碰蜂鸣器
        Error_Buzzer_Tick();

        osDelayUntil(&xLastWakeTime, 2);//精确延时2毫秒，同时更新xLastWakeTime的值为当前时间
    }


}


/**
 * 行为树（未启用）
 */
// void Action_Tree(void)
// {
//     起立
//         IF(处于倒地)
//         {
//             倒地自起动作组
//             return
//         }
//         后伸腿动作组

//     站起
//         小陀螺动作组
//         正常开
//         IF(侧翻检测)
//         {
//             底盘脱力动作组
//             IF(持续侧翻)
//             {
//                 返回起立态
//                 return
//             }
//         }

//     上台阶
//         上台阶动作组

//     下台阶
//         下台阶动作组
// }


// 摆头
