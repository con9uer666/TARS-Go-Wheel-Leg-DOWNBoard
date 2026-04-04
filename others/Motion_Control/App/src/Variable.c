#include "Variable.h"

// PID控制器定义
user_pid_t L_Leg_L0_PID;     //常态
user_pid_t R_Leg_L0_PID;     //

user_pid_t L_Leg_L0_POS_PID; //收腿
user_pid_t R_Leg_L0_POS_PID; //
user_pid_t L_Leg_L0_SPD_PID; //
user_pid_t R_Leg_L0_SPD_PID; //
user_pid_t L_Leg_L0_POS_PID; //收腿
user_pid_t R_Leg_L0_POS_PID; //
user_pid_t L_Leg_L0_SPD_PID; //
user_pid_t R_Leg_L0_SPD_PID; //

user_pid_t spinning_pid;//小陀螺PID
user_pid_t spinning_speed_pid;//小陀螺减速PID

user_pid_t Roll_Comp_PID;    //ROLL补偿pid

user_pid_t Leg_Phi0_PID;     //防劈叉pid

user_pid_t gimbal_pitch_pid;//云台俯仰pid

user_pid_t gimbal_yaw_speed_pid;//云台偏航速度环pid
user_pid_t gimbal_yaw_angle_pid;//云台偏航角度环pid

float PITCH_OFFSET = -0.08;
float pitch_trans[2];
float d_pitch;//pitch速度，单位为弧度每秒

float yaw_angle_PI = 0.0f;//标零处理后的yaw角度，单位rad，范围在[-PI, PI]内