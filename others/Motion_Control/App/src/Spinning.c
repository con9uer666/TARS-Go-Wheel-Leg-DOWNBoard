#include "Spinning.h"
#include "motor.h"
#include "Variable.h"
#include "Wheel_Leg_about.h"

//?调参
float target_spinning_d_yaw = 8.0f; // 目标小陀螺yaw速度，单位为弧度每秒
extern float d_yaw;
//小陀螺加速
void spinning_up()
{
    PID_Set_Error(&spinning_pid, d_yaw, target_spinning_d_yaw);
    yaw_error = PID_coculate(&spinning_pid);
    Speed_Error_Set();
}

//小陀螺减速
void spinning_down()
{
    PID_Set_Error(&spinning_pid, d_yaw, 0);
    yaw_error = PID_coculate(&spinning_pid);
    Speed_Error_Set();
}

//小陀螺急停
void spinning_stop()
{
    PID_Set_Error(&spinning_speed_pid, yaw_angle_PI, 0);
    float spinning_speed_output = PID_coculate(&spinning_speed_pid);
    PID_Set_Error(&spinning_pid, d_yaw, spinning_speed_output);
    yaw_error = PID_coculate(&spinning_pid);
    Speed_Error_Set();
}