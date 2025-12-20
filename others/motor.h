#ifndef MOTOR_H
#define MOTOR_H

#include "pid.h"
#include "main.h"

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

typedef struct Joint_Motor{
    Rx_Data_t Rx_Data;
    float Target_Torque;
    float TMAX;
    float PMAX;
    float VMAX;
    uint16_t motor_id
}Joint_Motor_t;

typedef struct Wheel_Motor{
    LK_Rx_Data_t Rx_Data;
    float Target_Torque;
    float TMAX;
    float PMAX;
    float VMAX;
    uint16_t motor_id
}Wheel_Motor_t;


extern Joint_Motor_t L_DM8009[2], R_DM8009[2];
extern Wheel_Motor_t L_LK9025, R_LK9025;

extern float body_speed;

void DM8009_Get_Data(uint8_t *Data ,Joint_Motor_t *Motor);
void LK9025_Get_Data(uint8_t *Data, Wheel_Motor_t *Motor);
void Disable_DM_Motor(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id);
void Disable_LK_Motor(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id);
void LK_MF9025_Torque_Ctrl(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id, float torque);
void DM_Motor_MIT_Torque_ctrl(FDCAN_HandleTypeDef *hfdcan, Joint_Motor_t Motor, float torq);

#endif

