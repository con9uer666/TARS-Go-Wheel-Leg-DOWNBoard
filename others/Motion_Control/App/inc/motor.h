#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"
#include "user_pid.h"

extern float LEG_MIN_LENTH;
extern float LEG_MAX_LENTH;  
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
extern uint8_t spinning_flag;      // 1：小陀螺运行中 0：小陀螺停止
extern float spin_speed_tol_angle; // 小陀螺时允许触发平移的yaw_angle_PI误差窗口(rad)
extern float spin_speed_angle_offset; // 小陀螺平移方向偏置(rad)
extern uint8_t jump_mode;            // 只读：当前是否处于跳跃中，由 Standing 内部根据 jump_cmd / jump_locked 计算
extern uint8_t jump_cmd;             // B2B byte51 原始跳跃指令：1=请求跳跃，0=解除跳跃锁
extern uint8_t jump_enable;          // 跳跃使能：=1 允许跳跃，=0 一票否决
extern uint8_t g_jump_buzzer_active; // 跳跃蜂鸣器独占标志，Error_Buzzer_Tick 看到=1 必须让位
extern float jump_F0;                // 跳跃时两腿沿腿杆向外的固定虚拟力(N)，最大约250N
extern float jump_leg_change_threshold; // 跳跃失败阈值(m)：锁存到期时任一腿变化<此值判失败
extern uint8_t jump_fail_reason;     // 跳跃退出原因 0=进行中/未触发 1=离地成功 2=超时腿长不足 3=超时但腿长够
extern float down_board_yaw_output; // 下板yaw输出

extern float speed_error; 
extern float target_roll;
extern float alpha_target_roll;
extern user_pid_t Roll_Comp_PID;
extern user_pid_t Jump_LR_Balance_PID;
extern int leg_state_count;
extern float target_Leg_L0;
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
extern float head_forward_angle;
extern uint8_t upstares_mode;

extern user_pid_t L_Leg_L0_POS_PID; //收腿
extern user_pid_t R_Leg_L0_POS_PID; //
extern user_pid_t L_Leg_L0_SPD_PID; //
extern user_pid_t R_Leg_L0_SPD_PID; //

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
void Sit_On_Ground(void);

extern uint8_t sit_mode_enable;
extern uint16_t sit_debug_counter;
extern uint8_t sit_ramp_done;


#endif

