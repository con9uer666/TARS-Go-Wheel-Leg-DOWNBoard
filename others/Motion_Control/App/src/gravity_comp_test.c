/**
 * @file gravity_comp_test.c
 * @brief 重力补偿参数测试：极坐标双环 PID 跟踪目标腿姿，统计均方误差，
 *        误差达标时记录该姿态下的 F0/τ 作为重力补偿标定值。调参用，非常规控制路径。
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

/*====================================== 腿坐标系相关结构体 ========================================*/

typedef struct
{
    Cartesian_Def current;
    Cartesian_Def error;
    Cartesian_Def target;
} Leg_Cartesian_Def;

typedef struct
{
    Polar_Def current;
    Polar_Def error;
    Polar_Def target;
} Leg_Polar_Def;

Leg_Cartesian_Def Cartesian_left_leg;
Leg_Cartesian_Def Cartesian_right_leg;

Leg_Polar_Def Polar_left_leg;
Leg_Polar_Def Polar_right_leg;

typedef struct
{
    user_pid_t L0_speed;
    user_pid_t L0_distance;
    user_pid_t phi0_speed;
    user_pid_t phi0_angle;
} Leg_user_pid_t;

Leg_user_pid_t left_Leg_PID;
Leg_user_pid_t right_Leg_PID;

float left_square_error;
float right_square_error;

float left_mean_square_error;
float right_mean_square_error;

uint8_t count;

float final_left_F0;
float final_left_tao;
float final_right_F0;
float final_right_tao;

void Gravity_Compensation_Test_Function(void)
{
    VMC_Coculate();
    Polar_Get(&Polar_left_leg.current, VMC_L.phi0, VMC_L.L0);
    Polar_Get(&Polar_right_leg.current, VMC_R.phi0, VMC_R.L0);

    //计算方差
    left_square_error += Polar_left_leg.error.r * Polar_left_leg.error.r + Polar_left_leg.error.angle * Polar_left_leg.error.angle;
    right_square_error += Polar_right_leg.error.r * Polar_right_leg.error.r + Polar_right_leg.error.angle * Polar_right_leg.error.angle;

    left_mean_square_error = left_square_error / count;
    right_mean_square_error = right_square_error / count;

    //极坐标误差计算
    Polar_left_leg.error.angle = Polar_left_leg.target.angle - Polar_left_leg.current.angle;
    Polar_left_leg.error.r = Polar_left_leg.target.r - Polar_left_leg.current.r;
    Polar_right_leg.error.angle = Polar_right_leg.target.angle - Polar_right_leg.current.angle;
    Polar_right_leg.error.r = Polar_right_leg.target.r - Polar_right_leg.current.r;

    //pid双环
    PID_Set_Error(&left_Leg_PID.L0_distance, Polar_right_leg.current.r, Polar_right_leg.target.r);
    PID_coculate(&left_Leg_PID.L0_distance);
    PID_Set_Error(&left_Leg_PID.L0_speed, Polar_right_leg.current.d_r, left_Leg_PID.L0_distance.output);
    PID_coculate(&left_Leg_PID.L0_speed);

    PID_Set_Error(&left_Leg_PID.phi0_angle, Polar_left_leg.current.angle, Polar_left_leg.target.angle);
    PID_coculate(&left_Leg_PID.phi0_angle);
    PID_Set_Error(&left_Leg_PID.phi0_speed, Polar_left_leg.current.d_angle, left_Leg_PID.phi0_angle.output);
    PID_coculate(&left_Leg_PID.phi0_speed);

    PID_Set_Error(&right_Leg_PID.L0_distance, Polar_right_leg.current.r, Polar_right_leg.target.r);
    PID_coculate(&right_Leg_PID.L0_distance);
    PID_Set_Error(&right_Leg_PID.L0_speed, Polar_right_leg.current.d_r, right_Leg_PID.L0_distance.output);
    PID_coculate(&right_Leg_PID.L0_speed);

    PID_Set_Error(&right_Leg_PID.phi0_angle, Polar_left_leg.current.angle, Polar_left_leg.target.angle);
    PID_coculate(&right_Leg_PID.phi0_angle);
    PID_Set_Error(&right_Leg_PID.phi0_speed, Polar_left_leg.current.d_angle, right_Leg_PID.phi0_angle.output);
    PID_coculate(&right_Leg_PID.phi0_speed);

    VMC_Set_F0_T(&VMC_L, left_Leg_PID.L0_speed.output, left_Leg_PID.phi0_speed.output);
    VMC_Set_F0_T(&VMC_R, right_Leg_PID.L0_speed.output, right_Leg_PID.phi0_speed.output);

    if(count == 255)
    {
        if(left_mean_square_error <= 0.01f)
        {
            //重力补偿参数测试通过
            final_left_F0 = left_Leg_PID.L0_speed.output;
            final_left_tao = left_Leg_PID.phi0_speed.output;
        }
        if(right_mean_square_error <= 0.01f)
        {
            //重力补偿参数测试通过
            final_right_F0 = right_Leg_PID.L0_speed.output;
            final_right_tao = right_Leg_PID.phi0_speed.output;
        }

        count = 0;
        left_square_error = 0;
        right_square_error = 0;
    }

    count ++;
}
