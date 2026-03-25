#ifndef MOTOR_H
#define MOTOR_H

#include "pid.h"
#include "main.h"

#define LEG_MIN_LENTH 0.19f
#define WHEEL_RADIUS 0.061f

typedef struct Rx_Data{
    uint8_t ID;
    uint8_t State;
    float Position;
    float Velocity;
    float Torque;
    float T_Mos;
    float T_Rotor;
}Rx_Data_t;

typedef struct LK_Rx_Data{
    uint8_t ID;
    uint8_t State;
    int16_t Position;
    float Velocity;
    int16_t Torque;
    int16_t T_Mos;
    int16_t T_Rotor;
}LK_Rx_Data_t;

typedef struct DJI_Rx_Data{
    uint16_t Position;
    float Velocity;
    int16_t Speed;
    int16_t Torque;
    uint8_t temperate;
    int16_t last_ecd;
}DJI_Rx_Data_t;

typedef struct Joint_Motor{
    Rx_Data_t Rx_Data;
    float Target_Torque;
    float TMAX;
    float PMAX;
    float VMAX;
    uint16_t motor_id;
}Joint_Motor_t;

typedef struct Wheel_Motor{
    DJI_Rx_Data_t Rx_Data;
    int16_t TX_data;
    float Target_Torque;
    float TMAX;
    float PMAX;
    float VMAX;
    uint16_t motor_id;
}Wheel_Motor_t;

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

extern Joint_Motor_t L_DM8009[2], R_DM8009[2], Yaw_DM4310, Shooter_DM2325;
extern Wheel_Motor_t L_LK9025, R_LK9025;
extern Foot_Chassis_t Foot_Chassis;
extern uint8_t gimbal_follow_flag; // 1：刚站起来，云台跟随底盘 0：底盘跟随云台
extern uint8_t yaw_ctrl_mode; // 0：下板控制 1：上板控制
extern float down_board_yaw_output; // 下板yaw输出

extern float body_speed;
extern uint8_t start_mode;

void DM8009_Get_Data(uint8_t *Data ,Joint_Motor_t *Motor);
// void LK9025_Get_Data(uint8_t *Data, Wheel_Motor_t *Motor);
void DJI3508_Get_Data(uint8_t *Data, Wheel_Motor_t *Motor);
void Disable_DM_Motor(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id);
void Disable_LK_Motor(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id);
void LK_MF9025_Torque_Ctrl(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id, float torque);
void DM_Motor_MIT_Torque_ctrl(FDCAN_HandleTypeDef *hfdcan, Joint_Motor_t Motor, float torq);
void DJI_Motor_Torque_Ctrl(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id, float torque);

#endif

