#ifndef VARIABLE_H
#define VARIABLE_H

#include "user_pid.h"

//过会拆耦合
extern user_pid_t L_Leg_L0_PID;
extern user_pid_t R_Leg_L0_PID;
extern user_pid_t L_Leg_L0_POS_PID;
extern user_pid_t R_Leg_L0_POS_PID;
extern user_pid_t L_Leg_L0_SPD_PID;
extern user_pid_t R_Leg_L0_SPD_PID;
extern user_pid_t L_Leg_L0_POS_PID;
extern user_pid_t R_Leg_L0_POS_PID;
extern user_pid_t L_Leg_L0_SPD_PID;
extern user_pid_t R_Leg_L0_SPD_PID;
extern user_pid_t spinning_pid;
extern user_pid_t spinning_speed_pid;
extern user_pid_t Roll_Comp_PID;
extern user_pid_t Leg_Phi0_PID;
extern user_pid_t gimbal_pitch_pid;
extern user_pid_t gimbal_yaw_speed_pid;
extern user_pid_t gimbal_yaw_angle_pid;

extern float PITCH_OFFSET;
extern float pitch_trans[2];
extern float d_pitch;//pitch速度，单位为弧度每秒
extern float yaw_angle_PI;//标零处理后的yaw角度，单位rad，范围在[-PI, PI]内
extern float yaw_error;

#endif // VARIABLE_H