/**
 * @file yaw_error.c
 * @brief 常态 Yaw 误差计算：以云台 Yaw 电机相对零点的角度为误差，按车速动态限幅，
 *        并触发速度误差计算 Speed_Error_Set()。
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

float target_yaw;
float raw_yaw_error;
float yaw_error_step = 0.01f;//yaw误差斜坡步长

//speed_error, yaw_error | 算yaw的误差，以及根据yaw误差调整target_body_speed进而调整speed_error()
void Yaw_Error_Coculate()
{
    float Yaw_motor_position;
    Yaw_motor_position = Yaw_DM4310.Rx_Data.Position - (head_forward_angle);//减的是零点

    //套圈处理
    if(Yaw_motor_position > PI)
    {
        Yaw_motor_position -= 2 * PI;
    }
    if(Yaw_motor_position < -PI)
    {
        Yaw_motor_position += 2 * PI;
    }
    yaw_error = Yaw_motor_position;
    // alpha_yaw_error = Yaw_motor_position * Yaw_motor_position * 0.05f;//Yaw_motor_position越大，alpha越大，响应越慢，最大为0.5
    // raw_yaw_error = Yaw_motor_position;


    float yaw_error_max = 0;
    yaw_error_max = 2.5f - fabsf(kalman_body_speed);//速度越快，允许的yaw误差越小，最大为5度，最小为0.05度
    if(yaw_error_max <= 0.05f)
    {
        yaw_error_max = 0.05f;
    }

    //这里Speed_Error_Set要用的是原生yaw_error，所以要写在yaw_error_max之上
    Speed_Error_Set();

    if(yaw_error > yaw_error_max)
        yaw_error = yaw_error_max;
    if(yaw_error < -yaw_error_max)
        yaw_error = -yaw_error_max;


        // yaw_error = easy_Slope(raw_yaw_error, yaw_error, yaw_error_step);

}
