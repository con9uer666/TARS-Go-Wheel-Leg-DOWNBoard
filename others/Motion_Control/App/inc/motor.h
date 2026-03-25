#ifndef MOTOR_H
#define MOTOR_H

#include "user_pid.h"
#include "main.h"
#include "Motor_Drv.h"

#define LEG_MIN_LENTH 0.19f
#define WHEEL_RADIUS 0.061f

typedef struct RampGenerator
{
    float currentValue; // 当前值
    float targetValue;  // 目标值
    float step;         // 每个控制周期应当改变的数值大小
    uint8_t isBusy;        // 指示斜坡发生器是否正在调整中
}RampGenerator;

typedef struct Leg_Info
{
	float Current_L0;//腿当前长度 单位m
}Leg_Info_t;

typedef struct Foot_Chassis_Info
{
	float Yaw_Motor_Angle;//Yaw电机角度
	float Current_Speed;//底盘当前的速度 m/s
	Leg_Info_t L_Leg, R_Leg;//腿信息
}Foot_Chassis_Info_t;

typedef struct Foot_Chassis
{
	float Target_Vx, Target_Vy;//云台坐标系下的目标速度 单位m/s
	uint8_t Target_Leg_State;//目标腿长，0短腿 1长腿
	uint8_t Chassis_Mode;//0跟随 1小陀螺 2静止趴下
	Foot_Chassis_Info_t	Info;//底盘信息
}Foot_Chassis_t;



extern Foot_Chassis_t Foot_Chassis;
extern uint8_t gimbal_follow_flag; // 1：刚站起来，云台跟随底盘 0：底盘跟随云台
extern float down_board_yaw_output; // 下板yaw输出

extern float speed_error; 
extern float target_roll;
extern float alpha_target_roll;
extern user_pid_t Roll_Comp_PID;
extern int leg_state_count;
extern float target_Leg_L0;
extern float alpha_target_L0;
extern user_pid_t L_Leg_L0_PID;
extern user_pid_t R_Leg_L0_PID; 
extern float target_L_Leg_L0;
extern float target_R_Leg_L0;
extern float speed_limit;
extern float target_body_speed;
extern float yaw_error;
extern float body_distance;
extern float target_body_distance;
extern float body_speed;
extern uint8_t start_mode;
extern float Wr, Wl;
extern float alpha_W;
extern float body_speed_L, body_speed_R, body_speed;
extern float pitch_trans[2];
extern float yaw_trans[2];
extern float d_pitch;
extern float alpha_d_pitch;
extern float d_yaw;
extern float alpha_d_yaw;
extern float LQR_K[4][10];
extern float body_speed;
extern uint8_t start_mode;
extern float body_distance_error;
extern float alpha_body_speed;

/*====================================== 运动控制相关 =========================================== */
void task_Motor_Init();
void task_VMC_Init();
void task_PID_Init();
void task_Pitch_Coculate();
void task_Motor_Enable();
void NotStanding_NotStairRetract();
void Standing();
void Upstair_NotStairRetract();
void StairRetract();


#endif

