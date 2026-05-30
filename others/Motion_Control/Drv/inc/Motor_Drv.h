#ifndef MOTOR_DRV_H
#define MOTOR_DRV_H

#include "main.h"
#include "USER_CAN.h"

typedef struct DJI_Rx_Data{
    uint16_t Position;
    float Velocity;
    int16_t Speed;
    int16_t Torque;
    uint8_t temperate;
    int16_t last_ecd;
}DJI_Rx_Data_t;

typedef struct Rx_Data{
    uint8_t ID;
    uint8_t State;
    float Position;
    float Velocity;
    float Torque;
    float T_Mos;
    float T_Rotor;
}Rx_Data_t;
typedef struct Joint_Motor{
    Rx_Data_t Rx_Data;
    float Target_Torque;
    float Target_Speed;
    float TMAX;
    float PMAX;
    float VMAX;
    float KD_MAX;
    float KD_MIN;
    uint16_t motor_id;
    uint8_t  last_error_code;     // 该电机最近一次进入故障态时的State码（latest-wins），0=从未出错；掉电归零
    uint8_t  clear_pending;       // 收帧解析到错误态时置1；CAN_Transmit替换本电机控制帧为清错帧后清0。优先级 > enable_pending
    uint8_t  enable_pending;      // 收帧解析到state==0(失能)且应使能时置1；CAN_Transmit替换本电机控制帧为使能帧后清0
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

void DM_Joint_Motor_Init(Joint_Motor_t *Motor, float TMAX, float PMAX,float VMAX, uint16_t motor_id);
void DM_Wheel_Motor_Init(Wheel_Motor_t *Motor, float TMAX, float PMAX,float VMAX, uint16_t motor_id);
int float_to_uint(float x_float, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
void DM8009_Get_Data(uint8_t *Data, Joint_Motor_t *Motor);
void DJ3508_Get_Data(uint8_t *Data, Wheel_Motor_t *Motor);
void Enable_DM_Motor_MIT(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id);
void Disable_DM_Motor(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id);
void Clear_DM_Motor_Error(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id);
void DM_Motor_MIT_Torque_ctrl(FDCAN_HandleTypeDef *hfdcan, Joint_Motor_t Motor, float torq);
void DM_Motor_MIT_Speed_ctrl(FDCAN_HandleTypeDef *hfdcan, Joint_Motor_t motor, float pos, float vel, float tor, float kp, float kd);
void DJI_Motor_Torque_Ctrl(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id, float torque);

extern Wheel_Motor_t L_DJ3508, R_DJ3508;
extern Joint_Motor_t L_DM8009[2], R_DM8009[2], Yaw_DM4310, Shooter_DM2325;

#endif