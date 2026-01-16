#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "remoter.h"
#include "pid.h"
#include "motor.h"
#include "State.h"
#include "arm_math.h"
#include "imu_temp_ctrl.h"
#include "USER_CAN.h"
#include "VMC.h"
#include "observe_task.h"
#include <stdint.h>

Joint_Motor_t L_DM8009[2], R_DM8009[2];
Wheel_Motor_t L_LK9025, R_LK9025;


float PITCH_OFFSET=-0.19;

float pitch_trans[2];
float d_pitch;
float alpha_d_pitch = 0.9;

float alpha_phi0 = 0.95;
float alpha_d_phi0 = 0.95;

float Leg_L_T;
float Leg_R_T;

float Wr, Wl;
float alpha_W = 0.9;
float body_speed_L, body_speed_R, body_speed;
float target_body_speed;
float speed_limit = 1.3;
float speed_error;
float alpha_target_body_speed = 0.9;
float alpha_body_speed = 0.95;
float body_distance;

float target_body_distance;
float body_distance_error;

float target_yaw, yaw_error;
float yaw_trans[2];
float d_yaw;
float alpha_d_yaw = 0.1;

float target_roll;
float alpha_target_roll = 0.05;

float Leg_F0_Limit = 500;

float mg = 150.0f/2;
float L_Ground_F0, R_Ground_F0;

float b_phi0_offset = 0.32;

float LQR_K[4][10] = {
    -1.5319,  -4.4506,  -4.8607,  -0.84003,  -6.7098,  -0.90493,  -5.4903,  -0.83328,  -9.7772,  -0.90614,
     -1.5319,  -4.4506,  4.8607,  0.84003,  -5.4903,  -0.83328,  -6.7098,  -0.90493,  -9.7772,  -0.90614,
     12.165,  34.823,  -14.353,  -2.4417,  74.246,  7.2706,  4.5678,  3.7992,  -108.66,  -0.62272,
     12.165,  34.823,  14.353,  2.4417,  4.5678,  3.7992,  74.246,  7.2706,  -108.66,  -0.62272
};

float K_Fit_Coefficients[40][6] = {
    -0.070651,  -1.2811,  0.25301,  1.2484,  0.45274,  -0.37445,
     -0.50034,  -8.0905,  1.7271,  8.0037,  2.5346,  -2.3823,
     -9.1466,  24.365,  -15.122,  -24.984,  -1.394,  17.084,
     -1.3727,  4.7027,  -3.6793,  -3.4745,  -0.46997,  3.7493,
     -4.1952,  -47.572,  11.312,  37.82,  13.348,  -13.641,
     -0.07852,  -4.5944,  1.0191,  -0.82574,  2.9643,  -1.7127,
     -1.3961,  5.049,  -24.399,  1.1645,  5.5067,  16.885,
     -0.026159,  -0.31885,  -2.1852,  0.93648,  0.48556,  -0.40258,
     -17.809,  27.732,  24.67,  -5.3572,  -32.385,  -11.358,
     -2.0457,  1.9857,  3.5828,  0.65426,  -3.6696,  -2.2091,
     -0.070651,  0.25301,  -1.2811,  -0.37445,  0.45274,  1.2484,
     -0.50034,  1.7271,  -8.0905,  -2.3823,  2.5346,  8.0037,
     9.1466,  15.122,  -24.365,  -17.084,  1.394,  24.984,
     1.3727,  3.6793,  -4.7027,  -3.7493,  0.46997,  3.4745,
     -1.3961,  -24.399,  5.049,  16.885,  5.5067,  1.1645,
     -0.026159,  -2.1852,  -0.31885,  -0.40258,  0.48556,  0.93648,
     -4.1952,  11.312,  -47.572,  -13.641,  13.348,  37.82,
     -0.07852,  1.0191,  -4.5944,  -1.7127,  2.9643,  -0.82574,
     -17.809,  24.67,  27.732,  -11.358,  -32.385,  -5.3572,
     -2.0457,  3.5828,  1.9857,  -2.2091,  -3.6696,  0.65426,
     1.8551,  -0.94997,  -4.3724,  -2.356,  4.7372,  2.5482,
     12.076,  -6.6473,  -28.837,  -14.921,  32.483,  16.264,
     -12.676,  -62.864,  -29.391,  105.73,  -46.907,  47.581,
     -1.5261,  -15.756,  -3.2727,  23.177,  -16.377,  5.6203,
     62.409,  -88.767,  6.2586,  44.974,  106.66,  -30.336,
     2.0813,  8.1469,  -2.4228,  -7.0016,  10.939,  -0.25414,
     7.1488,  -79.8,  -21.465,  100.2,  -103.8,  37.466,
     0.77553,  -4.8539,  2.0633,  6.2873,  -14.403,  -4.3549,
     -25.772,  -367,  82.995,  387.24,  91.383,  -113.69,
     0.42515,  -27.672,  2.2785,  23.49,  13.267,  -4.7976,
     1.8551,  -4.3724,  -0.94997,  2.5482,  4.7372,  -2.356,
     12.076,  -28.837,  -6.6473,  16.264,  32.483,  -14.921,
     12.676,  29.391,  62.864,  -47.581,  46.907,  -105.73,
     1.5261,  3.2727,  15.756,  -5.6203,  16.377,  -23.177,
     7.1488,  -21.465,  -79.8,  37.466,  -103.8,  100.2,
     0.77553,  2.0633,  -4.8539,  -4.3549,  -14.403,  6.2873,
     62.409,  6.2586,  -88.767,  -30.336,  106.66,  44.974,
     2.0813,  -2.4228,  8.1469,  -0.25414,  10.939,  -7.0016,
     -25.772,  82.995,  -367,  -113.69,  91.383,  387.24,
     0.42515,  2.2785,  -27.672,  -4.7976,  13.267,  23.49,
};

PID_t L_Leg_L0_PID;
PID_t R_Leg_L0_PID;

PID_t L_Leg_L0_POS_PID;
PID_t R_Leg_L0_POS_PID;
PID_t L_Leg_L0_SPD_PID;
PID_t R_Leg_L0_SPD_PID;

PID_t Roll_Comp_PID;

PID_t Leg_Phi0_PID;
float target_Leg_L0 = 0.17f;
float alpha_target_L0 = 0.01f;
float target_L_Leg_L0 = 0.17f;
float target_R_Leg_L0 = 0.17f;
uint8_t i;
int height_wait;
uint8_t temp1;

PID_t L_Leg_Middle_PID, R_Leg_Middle_PID;
PID_t L_Leg_dphi0_PID, R_Leg_dphi0_PID;

uint8_t first_run = 1;

uint8_t start_mode = 0;
uint8_t upstares_mode = 0;
int ready_count = 0;

uint8_t leg_state = 0;
int leg_state_count;

RampGenerator Target_Speed_Ramp;

// 一个周期内对斜坡发生器状态的更新
void rampIterate(RampGenerator *ramp)
{
    if (ramp->isBusy)
    {
        if (ramp->currentValue < ramp->targetValue)
        {                                     // 如果当前值小于目标值
            ramp->currentValue += ramp->step; // 增大当前值
            if (ramp->currentValue > ramp->targetValue)
            { // 避免超调
                ramp->currentValue = ramp->targetValue;
            }
        }
        else if (ramp->currentValue > ramp->targetValue)
        {                                     // 如果当前值大于目标值
            ramp->currentValue -= ramp->step; // 减小当前值
            if (ramp->currentValue < ramp->targetValue)
            { // 避免超调
                ramp->currentValue = ramp->targetValue;
            }
        }

        // 判断是否达到目标
        // if (ramp->currentValue == ramp->targetValue)
        // {
        //     ramp->isBusy = 0; // 达到目标，标记为不忙碌
        // }
    }
}

// 初始化斜坡发生器
void rampInit(RampGenerator *ramp, float startValue, float targetValue, float finalstep, float time, float cycleTime)
{
    ramp->currentValue = startValue;
    ramp->targetValue = targetValue;
    // 计算步进值，这里需要注意的是，确保斜坡时间和周期时间都不为零来避免除以零的错误
    if (time != 0 && cycleTime != 0)
    {
        ramp->step = (finalstep - 0) * (cycleTime / time);
    }
    else
    {
        ramp->step = 0; // 出错情况下设置为0，避免非法操作
    }
    ramp->isBusy = 1; // 标记为忙碌
}

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

void DJI3508_Get_Data(uint8_t *Data, Wheel_Motor_t *Motor)
{
    Motor->Rx_Data.Position = (uint16_t)(Data[0] << 8 | Data[1]);
    Motor->Rx_Data.Velocity = ((((int16_t)(Data[2] << 8 | Data[3])) / 60.0f) * 2 * PI) / 14.88f;
    Motor->Rx_Data.Torque = (int16_t)(Data[4] << 8 | Data[5]);
    Motor->Rx_Data.temperate = Data[6];
}

// void LK9025_Get_Data(uint8_t *Data, Wheel_Motor_t *Motor)
// {
//     if(Data[0] == 0x9c || Data[0] == 0xa4 || Data[0] == 0xa2 ||Data[0] == 0xa8 ||Data[0] == 0xa1)
//     {
//         Motor->Rx_Data.T_Rotor = Data[1];
//         Motor->Rx_Data.Torque = (Data[3] << 8 | Data[2]);
//         Motor->Rx_Data.Velocity = (((int16_t)(Data[5] << 8 | Data[4])) / 180.0f) * PI;
//         Motor->Rx_Data.Position = (Data[7] << 8 | Data[6]);
//     }
// }

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
	
    CAN_Send_DM_Motor_Data(hfdcan, motor_id, data);
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

    CAN_Send_DM_Motor_Data(hfdcan, Motor.motor_id, data);
}

void DM_Wheel_Motor_MIT_Torque_ctrl(FDCAN_HandleTypeDef *hfdcan, Wheel_Motor_t Motor, float torq)
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

    CAN_Send_DM_Motor_Data(hfdcan, Motor.motor_id, data);
}

void DJI_Motor_Torque_Ctrl(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id, float torque)
{
    uint8_t data[8] = {0};

    int16_t tqControl = (int16_t)(((((torque / 14.88f) / 0.02f))/20.0f) * 16384);

    data[0] = tqControl >> 8;
    data[1] = tqControl;
    data[2] = tqControl >> 8;
    data[3] = tqControl;
    data[4] = tqControl >> 8;
    data[5] = tqControl;
    data[6] = tqControl >> 8;
    data[7] = tqControl;

    CAN_Send_DM_Motor_Data(hfdcan, motor_id, data);
}

void Enable_LK_Motor(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id)
{
    uint8_t data[8] = {0};
    data[0] = 0x88;
    CAN_Send_DM_Motor_Data(hfdcan, motor_id, data);
}

void Disable_LK_Motor(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id)
{
    uint8_t data[8] = {0};
    data[0] = 0x80;
    CAN_Send_DM_Motor_Data(hfdcan, motor_id, data);
}

void LK_MF9025_Torque_Ctrl(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id, float torque)
{
    uint8_t data[8] = {0};
    int16_t iqControl = 0;

    if(torque >= 0.338959944)
    {
        iqControl = (int16_t)(403.44f * torque - 36.75f);
    }
    else if(torque <= -0.338959944)
    {
        iqControl = (int16_t)(403.44f * torque + 36.75f);
    }
    else
    {
        iqControl = (int16_t)(295.0201097f * torque);
    }

    if(iqControl >=  2048)iqControl =  2048;
    if(iqControl <= -2048)iqControl = -2048;

    data[0] = 0xA1;
    data[1] = 0;
    data[2] = 0;
    data[3] = 0;
    data[4] = (uint8_t)(iqControl & 0xFF);
    data[5] = (uint8_t)((iqControl >> 8) & 0xFF);
    data[6] = 0;
    data[7] = 0;

    CAN_Send_DM_Motor_Data(hfdcan, motor_id, data);
}

void LQR_Get_K(float LQR[4][10], float K_Fit_Coefficients[40][6], float L0_l, float L0_r)
{
    for(uint8_t i = 0; i < 4; i++) {
        for(uint8_t j = 0; j < 10; j++) {
            uint8_t pos = i * 10 + j;
            
            float p00 = K_Fit_Coefficients[pos][0];
            float p10 = K_Fit_Coefficients[pos][1];
            float p01 = K_Fit_Coefficients[pos][2];
            float p20 = K_Fit_Coefficients[pos][3];
            float p11 = K_Fit_Coefficients[pos][4];
            float p02 = K_Fit_Coefficients[pos][5];
            
            LQR[i][j] = p00
                      + p10 * L0_l
                      + p01 * L0_r
                      + p20 * L0_l * L0_l
                      + p11 * L0_l * L0_r
                      + p02 * L0_r * L0_r;
        }
    }
}

void Roll_Comp()
{
    if(speed_error <= 0.3 && speed_error >= -0.3)
    target_roll = alpha_target_roll * (-((SBUS_CH.CH1 - 992.0f)/800.0f) * 12.0f) + (1 - alpha_target_roll) * target_roll;
    else
    target_roll = alpha_target_roll * target_roll + (1 - alpha_target_roll) * target_roll;

    PID_Set_Error(&Roll_Comp_PID, roll, target_roll + 2);
    PID_coculate(&Roll_Comp_PID);
}

void b_phi0_offset_coc(float target_Leg_L0)
{
    // b_phi0_offset = -0.6043 *target_Leg_L0 + 0.175;
}

void Leg_L0_Control()
{
    if(leg_state_count > 0)
    {
        leg_state_count --;
    }

    if(SBUS_CH.SW4 == 2 && leg_state_count == 0)
    {
        leg_state_count = 300;
        if(leg_state < 2)
        leg_state ++;
        if(leg_state >= 2)
        leg_state = 2;
    }
    if(SBUS_CH.SW4 == 0 && leg_state_count == 0)
    {
        leg_state_count = 300;
        if(leg_state > 0)
        leg_state --;
        if(leg_state <= 0)
        leg_state = 0;
    }
    target_Leg_L0 = alpha_target_L0 * (((leg_state / 2.0f) * 0.22) + 0.18f) + (1 - alpha_target_L0) * target_Leg_L0;
    if(target_Leg_L0 >= 0.40f)
    target_Leg_L0 = 0.40f;
    if(target_Leg_L0 <= 0.16f)
    target_Leg_L0 = 0.16f;

    target_L_Leg_L0 = target_Leg_L0;
    target_R_Leg_L0 = target_Leg_L0;

    // target_L_Leg_L0 = target_Leg_L0 + Roll_Comp_PID.output;
    // target_R_Leg_L0 = target_Leg_L0 - Roll_Comp_PID.output;

    if(target_L_Leg_L0 >= 0.40)
    target_L_Leg_L0 = 0.40;
    if(target_L_Leg_L0 <= 0.16)
    target_L_Leg_L0 = 0.16;

    if(target_R_Leg_L0 >= 0.40)
    target_R_Leg_L0 = 0.40;
    if(target_R_Leg_L0 <= 0.16)
    target_R_Leg_L0 = 0.16;

    PID_Set_Error(&L_Leg_L0_PID, VMC_L.L0, target_L_Leg_L0);
    PID_Set_Error(&R_Leg_L0_PID, VMC_R.L0, target_R_Leg_L0);

    PID_coculate(&L_Leg_L0_PID);
    PID_coculate(&R_Leg_L0_PID);
}

void Speed_Error_Set()
{
    speed_limit = 2.7;
    rampInit(&Target_Speed_Ramp, target_body_speed, (((SBUS_CH.CH3 - 992.0f)/800.0f) * speed_limit),speed_limit, 0.3f, 0.002f);
    rampIterate(&Target_Speed_Ramp);
    target_body_speed = Target_Speed_Ramp.currentValue;
    // target_body_speed = alpha_target_body_speed * (((SBUS_CH.CH2 - 992.0f)/800.0f) * speed_limit) + (1 - alpha_target_body_speed) * target_body_speed;
    speed_error = target_body_speed - kalman_body_speed;
    if(speed_error >= speed_limit * 0.7)
    speed_error = speed_limit * 0.7;
    if(speed_error <= -speed_limit * 0.7)
    speed_error = -speed_limit * 0.7;
}

void Distance_Error_Set()
{
    body_distance += kalman_body_speed * 0.002;
    target_body_distance += (kalman_body_speed + speed_error) * 0.002;
    body_distance_error = target_body_distance - body_distance;
}

void Body_Speed_Coculate()
{
    VMC_L.last_phi0 = VMC_L.b_phi0;
    VMC_L.b_phi0 = alpha_phi0 * (-pitch_trans[0] + VMC_L.phi0 - (PI/2)) + (1 - alpha_phi0) * VMC_L.b_phi0;
    VMC_L.last_d_phi0 = VMC_L.d_phi0;
    VMC_L.d_phi0 = alpha_d_phi0 * ((VMC_L.b_phi0 - VMC_L.last_phi0) / 0.002) + (1 - alpha_d_phi0) * VMC_L.d_phi0 ;
    VMC_L.dd_b_phi0 = (VMC_L.d_phi0 - VMC_L.last_d_phi0)/0.002f;

    Wl = alpha_W * (-L_LK9025.Rx_Data.Velocity + VMC_L.d_phi0) + (1 - alpha_W) * Wl;
    body_speed_L = alpha_body_speed * ((Wl * 0.061f) + VMC_L.d_phi0 * VMC_L.L0 * arm_cos_f32(VMC_L.b_phi0)) + (1 - alpha_body_speed) * body_speed_L;

    VMC_R.last_phi0 = VMC_R.b_phi0;
    VMC_R.b_phi0 = alpha_phi0 * (-pitch_trans[0] + (PI - VMC_R.phi0)- (PI/2)) + (1 - alpha_phi0) * VMC_R.b_phi0;
    VMC_R.last_d_phi0 = VMC_R.d_phi0;
    VMC_R.d_phi0 = alpha_d_phi0 * ((VMC_R.b_phi0 - VMC_R.last_phi0) / 0.002) + (1 - alpha_d_phi0) * VMC_R.d_phi0 ;
    VMC_R.dd_b_phi0 = (VMC_R.d_phi0 - VMC_R.last_d_phi0)/0.002f;

    Wr = alpha_W * (R_LK9025.Rx_Data.Velocity + VMC_R.d_phi0) + (1 - alpha_W) * Wr;
    body_speed_R = alpha_body_speed * ((Wr * 0.061f) + VMC_R.d_phi0 * VMC_R.L0 * arm_cos_f32(VMC_R.b_phi0)) + (1 - alpha_body_speed) * body_speed_R;

    body_speed = (body_speed_L + body_speed_R) / 2.0f;
}

void VMC_Coculate()
{
    VMC_Set_phi1_phi4(&VMC_L, L_DM8009[1].Rx_Data.Position + PI, L_DM8009[0].Rx_Data.Position);
    VMC_Set_phi1_phi4(&VMC_R, R_DM8009[0].Rx_Data.Position + PI, R_DM8009[1].Rx_Data.Position);
    VMC_Get_L0_phi0(&VMC_L);
    VMC_Get_L0_phi0(&VMC_R);
}

void INS_Coculate()
{
    pitch_trans[1] = pitch_trans[0];
    pitch_trans[0] = (pitch/180.0f) * PI;
    yaw_trans[1] = yaw_trans[0];
    yaw_trans[0] = (yaw/180.0f) * PI;
    d_pitch = alpha_d_pitch * ((pitch_trans[0] - pitch_trans[1])/0.002f) + (1 - alpha_d_pitch) * d_pitch;
    float temp = yaw_trans[0] - yaw_trans[1];
    if (temp > PI) temp -= 2 * PI;
    else if (temp < -PI) temp += 2 * PI;
    d_yaw = alpha_d_yaw * (temp/0.002f) + (1 - alpha_d_yaw) * d_yaw;

    if(first_run == 1)
    {
        target_yaw = yaw_trans[0];
        d_yaw = 0;
        first_run = 0;
    }
}

void Yaw_Error_Coculate()
{
    float yaw_speed;
    yaw_speed = 0.008;
    target_yaw -= ((SBUS_CH.CH4 - 992.0f)/800.0f) * yaw_speed;
    if(target_yaw >= PI)
    target_yaw -= 2 * PI;
    if(target_yaw <= -PI)
    target_yaw += 2 * PI;

    yaw_error = target_yaw - yaw_trans[0];
    if (yaw_error > PI) yaw_error -= 2 * PI;
    else if (yaw_error < -PI) yaw_error += 2 * PI;
}

int L_Leg_State, R_Leg_State;
int L_Ready_Count, R_Ready_Count;
int L_off_ground = 0;
int R_off_ground = 0;

void Motor_task(void const * argument)
{
    DM_Joint_Motor_Init(&L_DM8009[0], 54.0f, 3.14159265f, 45.0f, 0x01);
    DM_Joint_Motor_Init(&L_DM8009[1], 54.0f, 3.14159265f, 45.0f, 0x02);

    DM_Joint_Motor_Init(&R_DM8009[0], 54.0f, 3.14159265f, 45.0f, 0x01);
    DM_Joint_Motor_Init(&R_DM8009[1], 54.0f, 3.14159265f, 45.0f, 0x02);

    VMC_Init(&VMC_L, 0.210f, 0.250f, 0.250f, 0.210f, 0.0f);
    VMC_Init(&VMC_R, 0.210f, 0.250f, 0.250f, 0.210f, 0.0f);

	PID_INIT(&L_Leg_L0_PID, 1000, 0, 15000, 150, 0, 0, 0);
    PID_INIT(&R_Leg_L0_PID, 1000, 0, 15000, 150, 0, 0, 0);
    PID_INIT(&Leg_Phi0_PID, 300, 0.1, 5, 150, 150, 50000, 0);
    PID_INIT(&Roll_Comp_PID, 10, 0.002, 100, 150, 80, 10000, 0);

    PID_INIT(&L_Leg_Middle_PID, 20, 0.01, 0.1, 2.0, 2.0, 0, 0);
    PID_INIT(&R_Leg_Middle_PID, 20, 0.01, 0.1, 2.0, 2.0, 0, 0);
    PID_INIT(&L_Leg_dphi0_PID, 5, 0.05, 1, 50, 50, 2000, 0);
    PID_INIT(&R_Leg_dphi0_PID, 5, 0.05, 1, 50, 50, 2000, 0);

    PID_INIT(&L_Leg_L0_POS_PID, 15, 0.001, 0.1, 1.0, 1.0, 200, 0);
    PID_INIT(&R_Leg_L0_POS_PID, 15, 0.001, 0.1, 1.0, 1.0, 200, 0);
    PID_INIT(&L_Leg_L0_SPD_PID, 200, 0.000, 50, 100, 100, 2000, 0);
    PID_INIT(&R_Leg_L0_SPD_PID, 200, 0.000, 50, 100, 100, 2000, 0);

    osDelay(2000);

    pitch_trans[1] = pitch_trans[0];
    pitch_trans[0] = (pitch/180.0f) * PI;

    Enable_DM_Motor_MIT(&hfdcan2, 0x01);
    osDelay(5);
    Enable_DM_Motor_MIT(&hfdcan2, 0x02);
    osDelay(5);
    // Enable_LK_Motor(&hfdcan1, 0x141);
    
    osDelay(5);
    Enable_DM_Motor_MIT(&hfdcan1, 0x01);
    osDelay(5);	
    Enable_DM_Motor_MIT(&hfdcan1, 0x02);
    osDelay(5);
    // Enable_LK_Motor(&hfdcan2, 0x141);
    TickType_t xLastWakeTime = xTaskGetTickCount(); 
    for(;;)
    {
        



        
        /*定腿长*/
        if(start_mode == 0 && upstares_mode == 0)
        {   
            first_run = 1;
            VMC_Set_phi1_phi4(&VMC_L, L_DM8009[1].Rx_Data.Position + PI, L_DM8009[0].Rx_Data.Position);
            VMC_Set_phi1_phi4(&VMC_R, R_DM8009[0].Rx_Data.Position + PI, R_DM8009[1].Rx_Data.Position);
            VMC_Get_L0_phi0(&VMC_L);
            VMC_Get_L0_phi0(&VMC_R);

            Body_Speed_Coculate();

            PID_Set_Error(&L_Leg_L0_POS_PID, VMC_L.L0, 0.19);
            PID_Set_Error(&R_Leg_L0_POS_PID, VMC_R.L0, 0.19);
            PID_coculate(&L_Leg_L0_POS_PID);
            PID_coculate(&R_Leg_L0_POS_PID);

            PID_Set_Error(&L_Leg_L0_SPD_PID, VMC_L.d_L0, L_Leg_L0_POS_PID.output);
            PID_Set_Error(&R_Leg_L0_SPD_PID, VMC_R.d_L0, R_Leg_L0_POS_PID.output);
            PID_coculate(&L_Leg_L0_SPD_PID);
            PID_coculate(&R_Leg_L0_SPD_PID);
            if(L_Leg_State >= 1)
            {
                PID_Set_Error(&L_Leg_Middle_PID, VMC_L.phi0, PI/2-0.2);
                PID_coculate(&L_Leg_Middle_PID);
                PID_Set_Error(&L_Leg_dphi0_PID, VMC_L.d_phi0, L_Leg_Middle_PID.output);
                PID_coculate(&L_Leg_dphi0_PID);
            }
            if(R_Leg_State >= 1)
            {
                PID_Set_Error(&R_Leg_Middle_PID, VMC_R.phi0, PI/2+0.2);
                PID_coculate(&R_Leg_Middle_PID);
                PID_Set_Error(&R_Leg_dphi0_PID, VMC_R.d_phi0, -R_Leg_Middle_PID.output);
                PID_coculate(&R_Leg_dphi0_PID);
            }

            VMC_Set_F0_T(&VMC_L, L_Leg_L0_SPD_PID.output, L_Leg_dphi0_PID.output);
            VMC_Set_F0_T(&VMC_R, R_Leg_L0_SPD_PID.output, -R_Leg_dphi0_PID.output);

            L_LK9025.Target_Torque = 0;
            R_LK9025.Target_Torque = 0;

            if(L_Leg_State == 0 && fabsf(L_Leg_L0_POS_PID.error) <= 0.06)
            {
                L_Ready_Count ++;
            }
            if(L_Leg_State == 0 && L_Ready_Count >= 50)
            {
                L_Leg_State = 1;
                L_Ready_Count = 0;
            }
            if(R_Leg_State == 0 && fabsf(R_Leg_L0_POS_PID.error) <= 0.06)
            {
                R_Ready_Count ++;
            }
            if(R_Leg_State == 0 && R_Ready_Count >= 50)
            {
                R_Leg_State = 1;
                R_Ready_Count = 0;
            }

            if(L_Leg_State == 1 && fabsf(L_Leg_Middle_PID.error) <= 0.05)
            {
                L_Ready_Count ++;
            }
            if(L_Leg_State == 1 && L_Ready_Count >= 50)
            {
                L_Leg_State = 2;
                L_Ready_Count = 0;
            }
            if(R_Leg_State == 1 && fabsf(R_Leg_Middle_PID.error) <= 0.05)
            {
                R_Ready_Count ++;
            }
            if(R_Leg_State == 1 &&R_Ready_Count >= 50)
            {
                R_Leg_State = 2;
                R_Ready_Count = 0;
                
            }

            if(R_Leg_State == 2 && L_Leg_State == 2)
            {
                start_mode = 1;
                R_Leg_State = 0;
                L_Leg_State = 0;
            }			
        }

        else if(start_mode == 1)
        {
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, 1);
            
            INS_Coculate();
            VMC_Coculate();
            Body_Speed_Coculate();
            
            Yaw_Error_Coculate();
            Speed_Error_Set();

            Distance_Error_Set();
            
            Roll_Comp();
            Leg_L0_Control();

            PID_Set_Error(&Leg_Phi0_PID, (VMC_L.phi0 - PI/2) + (VMC_R.phi0 - PI/2), 0);
            PID_coculate(&Leg_Phi0_PID);

            i++;
            if(i >= 5)
            {
                i = 0;
                LQR_Get_K(LQR_K, K_Fit_Coefficients, VMC_L.L0, VMC_R.L0);
            }

            b_phi0_offset_coc(target_Leg_L0);

            L_LK9025.Target_Torque = 
            + LQR_K[0][0] * body_distance_error
            + LQR_K[0][1] * (speed_error) 
            + LQR_K[0][2] * (yaw_error)
            - LQR_K[0][3] * d_yaw
            - LQR_K[0][4] * VMC_L.b_phi0 
            - LQR_K[0][5] * VMC_L.d_phi0 
            - LQR_K[0][6] * VMC_R.b_phi0 
            - LQR_K[0][7] * VMC_R.d_phi0 
            + LQR_K[0][8] * (pitch_trans[0] - PITCH_OFFSET)
            + LQR_K[0][9] * d_pitch;

            R_LK9025.Target_Torque = 
            + LQR_K[1][0] * body_distance_error
            + LQR_K[1][1] * (speed_error) 
            + LQR_K[1][2] * (yaw_error)
            - LQR_K[1][3] * d_yaw
            - LQR_K[1][4] * VMC_L.b_phi0 
            - LQR_K[1][5] * VMC_L.d_phi0 
            - LQR_K[1][6] * VMC_R.b_phi0 
            - LQR_K[1][7] * VMC_R.d_phi0 
            + LQR_K[1][8] * (pitch_trans[0] - PITCH_OFFSET)
            + LQR_K[1][9] * d_pitch;

            Leg_L_T = 
            + LQR_K[2][0] * body_distance_error
            + LQR_K[2][1] * (speed_error) 
            + LQR_K[2][2] * (-yaw_error)
            - LQR_K[2][3] * d_yaw
            - LQR_K[2][4] * (VMC_L.b_phi0 - b_phi0_offset)
            - LQR_K[2][5] * VMC_L.d_phi0 
            - LQR_K[2][6] * VMC_R.b_phi0 
            - LQR_K[2][7] * VMC_R.d_phi0
            + LQR_K[2][8] * (pitch_trans[0] - PITCH_OFFSET)
            + LQR_K[2][9] * d_pitch;

            Leg_R_T = 
            + LQR_K[3][0] * body_distance_error
            + LQR_K[3][1] * (speed_error) 
            + LQR_K[3][2] * (-yaw_error)
            - LQR_K[3][3] * d_yaw
            - LQR_K[3][4] * VMC_L.b_phi0 
            - LQR_K[3][5] * VMC_L.d_phi0 
            - LQR_K[3][6] * (VMC_R.b_phi0 - b_phi0_offset)
            - LQR_K[3][7] * VMC_R.d_phi0
            + LQR_K[3][8] * (pitch_trans[0] - PITCH_OFFSET)
            + LQR_K[3][9] * d_pitch;

            VMC_Set_F0_T(&VMC_L, L_Leg_L0_PID.output + (mg / arm_cos_f32(VMC_L.b_phi0)) + Roll_Comp_PID.output, Leg_L_T + Leg_Phi0_PID.output);
            VMC_Set_F0_T(&VMC_R, R_Leg_L0_PID.output + (mg / arm_cos_f32(VMC_R.b_phi0)) - Roll_Comp_PID.output, -Leg_R_T + Leg_Phi0_PID.output);

            // VMC_Set_F0_T(&VMC_L, L_Leg_L0_PID.output + (mg / arm_cos_f32(VMC_L.b_phi0)), Leg_L_T + Leg_Phi0_PID.output);
            // VMC_Set_F0_T(&VMC_R, R_Leg_L0_PID.output + (mg / arm_cos_f32(VMC_R.b_phi0)), -Leg_R_T + Leg_Phi0_PID.output);

            L_Ground_F0 = 0.1 * VMC_Get_Ground_F0(&VMC_L) + 0.9 * L_Ground_F0;
            R_Ground_F0 = 0.1 * VMC_Get_Ground_F0(&VMC_R) + 0.9 * R_Ground_F0;

            

            if(L_Ground_F0 <= 20.0f)
            L_off_ground ++;
            else
            L_off_ground --;
            if(L_off_ground >= 50)
            L_off_ground = 50;
            if(L_off_ground <= 0)
            L_off_ground = 0;
            
            if(R_Ground_F0 <= 20.0f)
            R_off_ground ++;
            else
            R_off_ground --;
            if(R_off_ground >= 50)
            R_off_ground = 50;
            if(R_off_ground <= 0)
            R_off_ground = 0;


            if(L_off_ground >= 15)
            {
                Leg_L_T = 
                - LQR_K[2][4] * VMC_L.b_phi0 
                - LQR_K[2][5] * VMC_L.d_phi0 ;
                Leg_L_T *= 0.7;
                L_LK9025.Target_Torque = 0;
                VMC_Set_F0_T(&VMC_L, L_Leg_L0_PID.output + (mg / arm_cos_f32(VMC_L.b_phi0)), Leg_L_T);
                body_distance = 0;
                target_body_distance = 0;
            }

            if(R_off_ground >= 15)
            {
                Leg_R_T = 
                - LQR_K[3][6] * VMC_R.b_phi0 
                - LQR_K[3][7] * VMC_R.d_phi0;
                Leg_R_T *= 0.7;
                R_LK9025.Target_Torque = 0;
                VMC_Set_F0_T(&VMC_R, R_Leg_L0_PID.output + (mg / arm_cos_f32(VMC_R.b_phi0)), -Leg_R_T);
                body_distance = 0;
                target_body_distance = 0;
            }

            if(SBUS_CH.SW1 != 1)
            {
                start_mode = 2;
            }

            
            

            // VMC_Set_F0_T(&VMC_L, L_Leg_L0_PID.output, + L_Leg_Phi0_PID.output);
            // VMC_Set_F0_T(&VMC_R, R_Leg_L0_PID.output, + R_Leg_Phi0_PID.output);

            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, 0);

            // DM_Motor_MIT_Torque_ctrl(&hfdcan2, L_DM8009[0], VMC_L.T1);
            // DM_Motor_MIT_Torque_ctrl(&hfdcan1, R_DM8009[0], VMC_R.T2);
            // DM_Motor_MIT_Torque_ctrl(&hfdcan2, L_DM8009[1], VMC_L.T2);
            // DM_Motor_MIT_Torque_ctrl(&hfdcan1, R_DM8009[1], VMC_R.T1);

            // DM_Motor_MIT_Torque_ctrl(&hfdcan2, L_DM8009[0], 0); 
            // DM_Motor_MIT_Torque_ctrl(&hfdcan1, R_DM8009[0], 0);
            // DM_Motor_MIT_Torque_ctrl(&hfdcan2, L_DM8009[1], 0);
            // DM_Motor_MIT_Torque_ctrl(&hfdcan1, R_DM8009[1], 0);

            

            // DM_Wheel_Motor_MIT_Torque_ctrl(&hfdcan2, L_LK9025, L_LK9025.Target_Torque);
            // DM_Wheel_Motor_MIT_Torque_ctrl(&hfdcan1, R_LK9025, -R_LK9025.Target_Torque);

            // DM_Wheel_Motor_MIT_Torque_ctrl(&hfdcan2, L_LK9025, 0);
            // DM_Wheel_Motor_MIT_Torque_ctrl(&hfdcan1, R_LK9025, 0);
        }
        else if(start_mode == 2 && upstares_mode == 0)//伸腿上台阶
        {
            VMC_Set_phi1_phi4(&VMC_L, L_DM8009[1].Rx_Data.Position + PI, L_DM8009[0].Rx_Data.Position);
            VMC_Set_phi1_phi4(&VMC_R, R_DM8009[0].Rx_Data.Position + PI, R_DM8009[1].Rx_Data.Position);
            VMC_Get_L0_phi0(&VMC_L);
            VMC_Get_L0_phi0(&VMC_R);

            Body_Speed_Coculate();

            L_LK9025.Target_Torque = 0.5;
            R_LK9025.Target_Torque = 0.5;

            PID_Set_Error(&L_Leg_L0_POS_PID, VMC_L.L0, 0.44);
            PID_Set_Error(&R_Leg_L0_POS_PID, VMC_R.L0, 0.44);
            PID_coculate(&L_Leg_L0_POS_PID);
            PID_coculate(&R_Leg_L0_POS_PID);

            PID_Set_Error(&L_Leg_L0_SPD_PID, VMC_L.d_L0, L_Leg_L0_POS_PID.output);
            PID_Set_Error(&R_Leg_L0_SPD_PID, VMC_R.d_L0, R_Leg_L0_POS_PID.output);
            PID_coculate(&L_Leg_L0_SPD_PID);
            PID_coculate(&R_Leg_L0_SPD_PID);

            PID_Set_Error(&L_Leg_Middle_PID, VMC_L.phi0, PI/2 + 0.75);
            PID_coculate(&L_Leg_Middle_PID);
            PID_Set_Error(&L_Leg_dphi0_PID, VMC_L.d_phi0, L_Leg_Middle_PID.output);
            PID_coculate(&L_Leg_dphi0_PID);
            
            PID_Set_Error(&R_Leg_Middle_PID, VMC_R.phi0, PI/2 - 0.75);
            PID_coculate(&R_Leg_Middle_PID);
            PID_Set_Error(&R_Leg_dphi0_PID, VMC_R.d_phi0, -R_Leg_Middle_PID.output);
            PID_coculate(&R_Leg_dphi0_PID);

            VMC_Set_F0_T(&VMC_L, L_Leg_L0_SPD_PID.output, L_Leg_dphi0_PID.output);
            VMC_Set_F0_T(&VMC_R, R_Leg_L0_SPD_PID.output, -R_Leg_dphi0_PID.output);

            if(L_Leg_State == 0 && fabsf(L_Leg_L0_POS_PID.error) <= 0.05 && fabsf(L_Leg_Middle_PID.error) <= 0.05)
            {
                L_Ready_Count ++;
            }
            if(L_Leg_State == 0 && L_Ready_Count >= 250)
            {
                L_Leg_State = 2;
                L_Ready_Count = 0;
            }
            if(R_Leg_State == 0 && fabsf(R_Leg_L0_POS_PID.error) <= 0.05 && fabsf(R_Leg_Middle_PID.error) <= 0.05)
            {
                R_Ready_Count ++;
            }
            if(R_Leg_State == 0 && R_Ready_Count >= 250)
            {
                R_Leg_State = 2;
                R_Ready_Count = 0;
            }
            if(R_Leg_State == 2 && L_Leg_State == 2)
            {
                upstares_mode = 1;
                R_Leg_State = 0;
                L_Leg_State = 0;
            }	

        }
        else if(upstares_mode == 1)//收腿起立
        {
            
            VMC_Set_phi1_phi4(&VMC_L, L_DM8009[1].Rx_Data.Position + PI, L_DM8009[0].Rx_Data.Position);
            VMC_Set_phi1_phi4(&VMC_R, R_DM8009[0].Rx_Data.Position + PI, R_DM8009[1].Rx_Data.Position);
            VMC_Get_L0_phi0(&VMC_L);
            VMC_Get_L0_phi0(&VMC_R);

            Body_Speed_Coculate();

            PID_Set_Error(&L_Leg_L0_POS_PID, VMC_L.L0, 0.16);
            PID_Set_Error(&R_Leg_L0_POS_PID, VMC_R.L0, 0.16);
            PID_coculate(&L_Leg_L0_POS_PID);
            PID_coculate(&R_Leg_L0_POS_PID);

            PID_Set_Error(&L_Leg_L0_SPD_PID, VMC_L.d_L0, L_Leg_L0_POS_PID.output);
            PID_Set_Error(&R_Leg_L0_SPD_PID, VMC_R.d_L0, R_Leg_L0_POS_PID.output);
            PID_coculate(&L_Leg_L0_SPD_PID);
            PID_coculate(&R_Leg_L0_SPD_PID);

            
            if(L_Leg_State >= 1)
            {
                PID_Set_Error(&L_Leg_Middle_PID, VMC_L.phi0, PI/2-0.2);
                PID_coculate(&L_Leg_Middle_PID);
                PID_Set_Error(&L_Leg_dphi0_PID, VMC_L.d_phi0, L_Leg_Middle_PID.output);
                PID_coculate(&L_Leg_dphi0_PID);
            }
            else 
            {
                PID_Set_Error(&L_Leg_Middle_PID, VMC_L.phi0, PI/2-1.2);
                PID_coculate(&L_Leg_Middle_PID);
                PID_Set_Error(&L_Leg_dphi0_PID, VMC_L.d_phi0, L_Leg_Middle_PID.output);
                PID_coculate(&L_Leg_dphi0_PID);
            }
            if(R_Leg_State >= 1)
            {
                PID_Set_Error(&R_Leg_Middle_PID, VMC_R.phi0, PI/2+0.2);
                PID_coculate(&R_Leg_Middle_PID);
                PID_Set_Error(&R_Leg_dphi0_PID, VMC_R.d_phi0, -R_Leg_Middle_PID.output);
                PID_coculate(&R_Leg_dphi0_PID);
            }
            else
            {
                PID_Set_Error(&R_Leg_Middle_PID, VMC_R.phi0, PI/2+1.2);
                PID_coculate(&R_Leg_Middle_PID);
                PID_Set_Error(&R_Leg_dphi0_PID, VMC_R.d_phi0, -R_Leg_Middle_PID.output);
                PID_coculate(&R_Leg_dphi0_PID);
            }

            VMC_Set_F0_T(&VMC_L, L_Leg_L0_SPD_PID.output, L_Leg_dphi0_PID.output);
            VMC_Set_F0_T(&VMC_R, R_Leg_L0_SPD_PID.output, -R_Leg_dphi0_PID.output);

            L_LK9025.Target_Torque = 0.5;
            R_LK9025.Target_Torque = 0.5;

            if(L_Leg_State == 0 && fabsf(L_Leg_L0_POS_PID.error) <= 0.01 && fabsf(L_Leg_Middle_PID.error) <= 0.01)
            {
                L_Ready_Count ++;
            }
            if(L_Leg_State == 0 && L_Ready_Count >= 100)
            {
                L_Leg_State = 1;
                L_Ready_Count = 0;
            }
            if(R_Leg_State == 0 && fabsf(R_Leg_L0_POS_PID.error) <= 0.01 && fabsf(R_Leg_Middle_PID.error) <= 0.01)
            {
                R_Ready_Count ++;
            }
            if(R_Leg_State == 0 && R_Ready_Count >= 100)
            {
                R_Leg_State = 1;
                R_Ready_Count = 0;
            }

            if(L_Leg_State == 1 && fabsf(L_Leg_Middle_PID.error) <= 0.05)
            {
                L_Ready_Count ++;
            }
            if(L_Leg_State == 1 && L_Ready_Count >= 50)
            {
                L_Leg_State = 2;
                L_Ready_Count = 0;
                L_LK9025.Target_Torque = 0;
            }
            if(R_Leg_State == 1 && fabsf(R_Leg_Middle_PID.error) <= 0.05)
            {
                R_Ready_Count ++;
            }
            if(R_Leg_State == 1 &&R_Ready_Count >= 50)
            {
                R_Leg_State = 2;
                R_Ready_Count = 0;
                R_LK9025.Target_Torque = 0;
            }

            if(R_Leg_State == 2 && L_Leg_State == 2)
            {
                upstares_mode = 0;
                start_mode = 0;
                R_Leg_State = 1;
                L_Leg_State = 1;
                leg_state = 0;
                target_Leg_L0 = 0.16;

            }	
        }
        osDelayUntil(&xLastWakeTime, 2);
    }

    
}
