#ifndef MOTOR_DRV_H
#define MOTOR_DRV_H

#include "main.h"
#include "USER_CAN.h"
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

void DM_Joint_Motor_Init(Joint_Motor_t *Motor, float TMAX, float PMAX,float VMAX, uint16_t motor_id);
void DM_Wheel_Motor_Init(Wheel_Motor_t *Motor, float TMAX, float PMAX,float VMAX, uint16_t motor_id);
int float_to_uint(float x_float, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
void DM8009_Get_Data(uint8_t *Data, Joint_Motor_t *Motor);
void DJI3508_Get_Data(uint8_t *Data, Wheel_Motor_t *Motor);
// void LK9025_Get_Data(uint8_t *Data, Wheel_Motor_t *Motor);
void Enable_DM_Motor_MIT(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id);
void Disable_DM_Motor(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id);
void DM_Motor_MIT_Torque_ctrl(FDCAN_HandleTypeDef *hfdcan, Joint_Motor_t Motor, float torq);
void DM_Wheel_Motor_MIT_Torque_ctrl(FDCAN_HandleTypeDef *hfdcan, Wheel_Motor_t Motor, float torq);
void DJI_Motor_Torque_Ctrl(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id, float torque);
void Enable_LK_Motor(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id);
void Disable_LK_Motor(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id);
void LK_MF9025_Torque_Ctrl(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id, float torque);

extern Wheel_Motor_t L_LK9025, R_LK9025;
extern Joint_Motor_t L_DM8009[2], R_DM8009[2], Yaw_DM4310, Shooter_DM2325;

#endif