#include "Motor_Drv.h"

Wheel_Motor_t L_DJ3508, R_DJ3508;
Joint_Motor_t L_DM8009[2], R_DM8009[2], Yaw_DM4310, Shooter_DM2325;


void DM_Joint_Motor_Init(Joint_Motor_t *Motor, float TMAX, float PMAX,float VMAX, uint16_t motor_id)
{
    Motor->TMAX = TMAX;
    Motor->PMAX = PMAX;
    Motor->VMAX = VMAX;

    Motor->motor_id = motor_id;
}

void DM_Wheel_Motor_Init(Wheel_Motor_t *Motor, float TMAX, float PMAX,float VMAX, uint16_t motor_id)
{
    Motor->TMAX = TMAX;
    Motor->PMAX = PMAX;
    Motor->VMAX = VMAX;

    Motor->motor_id = motor_id;
}

int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

void DM8009_Get_Data(uint8_t *Data, Joint_Motor_t *Motor)
{
    Motor->Rx_Data.ID = Data[0] & 0x0F;
    Motor->Rx_Data.State = Data[0] >> 4;
    Motor->Rx_Data.Position = (uint_to_float(((Data[1] << 8) | Data[2]), -Motor->PMAX, Motor->PMAX, 16));
    Motor->Rx_Data.Velocity = uint_to_float(((Data[3] << 4) | (Data[4] >> 4)), -Motor->VMAX, Motor->VMAX, 12);
    Motor->Rx_Data.Torque = uint_to_float((((Data[4]&0xF) << 8) | Data[5]), -Motor->TMAX, Motor->TMAX, 12);
    Motor->Rx_Data.T_Mos = (float)Data[6];
    Motor->Rx_Data.T_Rotor = (float)Data[7];
}

void DJ3508_Get_Data(uint8_t *Data, Wheel_Motor_t *Motor)
{
    Motor->Rx_Data.Position = (uint16_t)(Data[0] << 8 | Data[1]);
    Motor->Rx_Data.Velocity = ((((int16_t)(Data[2] << 8 | Data[3])) / 60.0f) * 2 * PI) / 14.88f;
    Motor->Rx_Data.Speed = (int16_t)(Data[2] << 8 | Data[3]);
    Motor->Rx_Data.Torque = (int16_t)(Data[4] << 8 | Data[5]);
    Motor->Rx_Data.temperate = Data[6];
}

void Enable_DM_Motor_MIT(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id)
{
	uint8_t data[8];
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFC;
	
    CAN_Send_DM_Motor_Data(hfdcan, motor_id, data);//!丛庆拉的，没改名字，不是适配达妙电机的，是通用的发送函数
}

void Disable_DM_Motor(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id)
{
    uint8_t data[8];

    data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFD;

    CAN_Send_DM_Motor_Data(hfdcan, motor_id, data);//!丛庆拉的，没改名字，不是适配达妙电机的，是通用的发送函数
}

void Clear_DM_Motor_Error(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id)
{
    uint8_t data[8];

    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFB;

    CAN_Send_DM_Motor_Data(hfdcan, motor_id, data);
}

void DM_Motor_MIT_Torque_ctrl(FDCAN_HandleTypeDef *hfdcan, Joint_Motor_t Motor, float torq)
{
	uint8_t data[8];
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;

    if(torq >= Motor.TMAX)
    torq = Motor.TMAX;
    if(torq <= -Motor.TMAX)
    torq = -Motor.TMAX;

	pos_tmp = 0;
	vel_tmp = 0;
	kp_tmp  = 0;
	kd_tmp  = 0;
	tor_tmp = float_to_uint(torq, -Motor.TMAX,  Motor.TMAX,  12);

	data[0] = (pos_tmp >> 8);
	data[1] = pos_tmp;
	data[2] = (vel_tmp >> 4);
	data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	data[4] = kp_tmp;
	data[5] = (kd_tmp >> 4);
	data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	data[7] = tor_tmp;

    CAN_Send_DM_Motor_Data(hfdcan, Motor.motor_id, data);//!丛庆拉的，没改名字，不是适配达妙电机的，是通用的发送函数
}

void DM_Motor_MIT_Speed_ctrl(FDCAN_HandleTypeDef *hfdcan, Joint_Motor_t motor, float pos, float vel, float tor, float kp, float kd)
{
	uint8_t data[8];
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;

	pos_tmp = float_to_uint(pos, -motor.PMAX, motor.PMAX, 16);
	vel_tmp = float_to_uint(vel, -motor.VMAX, motor.VMAX, 12);
	tor_tmp = float_to_uint(tor, -motor.TMAX, motor.TMAX, 12);
	kp_tmp  = float_to_uint(kp,  0, 500, 12);
	kd_tmp  = float_to_uint(kd,  0, 5, 12);

	data[0] = (pos_tmp >> 8);
	data[1] = pos_tmp;
	data[2] = (vel_tmp >> 4);
	data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	data[4] = kp_tmp;
	data[5] = (kd_tmp >> 4);
	data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	data[7] = tor_tmp;

    CAN_Send_DM_Motor_Data(hfdcan, motor.motor_id, data);//!丛庆拉的，没改名字，不是适配达妙电机的，是通用的发送函数
}

void DJI_Motor_Torque_Ctrl(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id, float torque)
{
    uint8_t data[8] = {0};

    int16_t tqControl = (int16_t)(((((torque / 14.88f) / 0.02f))/20.0f) * 16384);

    if(hfdcan == &hfdcan2)
    {
        L_DJ3508.TX_data = tqControl;
    }

    if(hfdcan == &hfdcan1)
    {
        R_DJ3508.TX_data = tqControl;
    }

    data[0] = tqControl >> 8;
    data[1] = tqControl;
    data[2] = tqControl >> 8;
    data[3] = tqControl;
    data[4] = tqControl >> 8;
    data[5] = tqControl;
    data[6] = tqControl >> 8;
    data[7] = tqControl;

    CAN_Send_DM_Motor_Data(hfdcan, motor_id, data);//!丛庆拉的，没改名字，不是适配达妙电机的，是通用的发送函数
}

