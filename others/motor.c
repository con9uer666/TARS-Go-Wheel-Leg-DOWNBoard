#include "controller.h"
#include "fdcan.h"
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
#include <math.h>
#include <stdint.h>
#include "Self_Righting.h"
#include "Board2Board.h"
#include "Slope.h"

Joint_Motor_t L_DM8009[2], R_DM8009[2], Yaw_DM4310, Shooter_DM2325;                                                                                                                                     
Wheel_Motor_t L_LK9025, R_LK9025;//!这是3508啊啊啊，没改名    
Foot_Chassis_t Foot_Chassis;//轮足底盘结构体                                                                                                                               
                                                                                                                                        
float powerPredict;                                                                                                                                     
                                                                                                                                        
                                                                                                                                        
float L_b_phi0, R_b_phi0;                                                                                                                                       
                                                                                                                                        
float PITCH_OFFSET=-0.08;                                                                                                                                       
                                                                                                                                        
//!屎作俑者：25年丛庆  数组0为当前pitch值，数组1为上一次的pitch值  单位为弧度   
float pitch_trans[2];                                                                                               
float d_pitch;//pitch速度，单位为弧度每秒                                                                                                                                      
float alpha_d_pitch = 1.0;//滤波系数                                                                                                                                      
                                                                                                                                        
                                                                                                                                     
                                                                                                                                       
                                                                                                                                        
float Leg_L_T; //模拟腿力矩                                                                                                                                     
float Leg_R_T;                                                                                                                                      
                                                                                                                                        
float Wr, Wl;//加上杆角速度的车轮速度                                                                                                                                    
float alpha_W = 0.9;//滤波系数                                                                                                                                        
float body_speed_L, body_speed_R, body_speed; //当前车体速度 ,已正交，是水平方向的速度                                                                                                                                     
float target_body_speed;//目标速度                                                                                                                                        
float speed_limit = 1.3;                                                                                                                                        
float speed_error;                                                                                                                                      
float alpha_target_body_speed = 1.0;                                                                                                                                        
float alpha_body_speed = 1.0;//单侧算车体速度 滤波系数                                                                                                                                       
float body_distance;

float target_body_distance = 2.0f;
float body_distance_error;

float target_yaw, yaw_error;
//!屎作俑者：25年丛庆  数组0为当前pitch值，数组1为上一次的pitch值     单位为弧度
float yaw_trans[2];
float d_yaw;//陀螺仪yaw速度，单位为弧度每秒
float alpha_d_yaw = 1.0;

float target_roll;
float alpha_target_roll = 0.05;

float Leg_F0_Limit = 500;

float mg = 130.0f/2;
float L_Ground_F0, R_Ground_F0; //地面支持力

float b_phi0_offset = 0.2;

float LQR_K[4][10] = {
    -1.5319,  -4.4506,  -4.8607,  -0.84003,  -6.7098,  -0.90493,  -5.4903,  -0.83328,  -9.7772,  -0.90614,
     -1.5319,  -4.4506,  4.8607,  0.84003,  -5.4903,  -0.83328,  -6.7098,  -0.90493,  -9.7772,  -0.90614,
     12.165,  34.823,  -14.353,  -2.4417,  74.246,  7.2706,  4.5678,  3.7992,  -108.66,  -0.62272,
     12.165,  34.823,  14.353,  2.4417,  4.5678,  3.7992,  74.246,  7.2706,  -108.66,  -0.62272
};

float K_Fit_Coefficients[40][6] = {
    -0.068193,  -1.6259,  0.29902,  1.552,  0.57429,  -0.43108,
     -0.4847,  -10.147,  1.9978,  9.8122,  3.2246,  -2.7018,
     -7.9957,  21.606,  -14.718,  -19.477,  -2.5806,  16.472,
     -1.2699,  4.3645,  -3.7208,  -2.598,  -1.1239,  3.8763,
     -3.9212,  -53.864,  12.878,  41.745,  15.468,  -15.151,
     -0.055559,  -5.0423,  1.1029,  -1.2235,  3.559,  -1.9008,
     -1.3831,  3.936,  -25.713,  6.1555,  0.3144,  19.088,
     -0.0080262,  -0.5688,  -2.277,  1.5415,  -0.29637,  -0.37831,
     -20.26,  26.786,  30.51,  3.2449,  -41.528,  -13.451,
     -2.1714,  1.6866,  4.0479,  1.374,  -4.1586,  -2.4665,
     -0.068193,  0.29902,  -1.6259,  -0.43108,  0.57429,  1.552,
     -0.4847,  1.9978,  -10.147,  -2.7018,  3.2246,  9.8122,
     7.9957,  14.718,  -21.606,  -16.472,  2.5806,  19.477,
     1.2699,  3.7208,  -4.3645,  -3.8763,  1.1239,  2.598,
     -1.3831,  -25.713,  3.936,  19.088,  0.3144,  6.1555,
     -0.0080262,  -2.277,  -0.5688,  -0.37831,  -0.29637,  1.5415,
     -3.9212,  12.878,  -53.864,  -15.151,  15.468,  41.745,
     -0.055559,  1.1029,  -5.0423,  -1.9008,  3.559,  -1.2235,
     -20.26,  30.51,  26.786,  -13.451,  -41.528,  3.2449,
     -2.1714,  4.0479,  1.6866,  -2.4665,  -4.1586,  1.374,
     2.658,  -1.0861,  -6.2333,  -4.3181,  6.7588,  4.06,
     17.069,  -7.6413,  -40.507,  -27.094,  45.699,  25.531,
     -9.8736,  -69.673,  -24.701,  113.33,  -59.445,  46.026,
     -1.3687,  -17.213,  -0.87145,  24.641,  -18.022,  3.5348,
     81.224,  -114.61,  6.6818,  35.596,  130.88,  -35.985,
     2.6845,  11.349,  -3.8039,  -12.594,  12.275,  1.0852,
     9.071,  -101.82,  -15.817,  120.05,  -130.29,  59.742,
     0.96975,  -5.1114,  3.4033,  5.2948,  -17.639,  -2.4251,
     -27.759,  -501.1,  113.04,  516.7,  136.53,  -159.46,
     1.2444,  -33.561,  1.5483,  26.94,  18.088,  -5.1333,
     2.658,  -6.2333,  -1.0861,  4.06,  6.7588,  -4.3181,
     17.069,  -40.507,  -7.6413,  25.531,  45.699,  -27.094,
     9.8736,  24.701,  69.673,  -46.026,  59.445,  -113.33,
     1.3687,  0.87145,  17.213,  -3.5348,  18.022,  -24.641,
     9.071,  -15.817,  -101.82,  59.742,  -130.29,  120.05,
     0.96975,  3.4033,  -5.1114,  -2.4251,  -17.639,  5.2948,
     81.224,  6.6818,  -114.61,  -35.985,  130.88,  35.596,
     2.6845,  -3.8039,  11.349,  1.0852,  12.275,  -12.594,
     -27.759,  113.04,  -501.1,  -159.46,  136.53,  516.7,
     1.2444,  1.5483,  -33.561,  -5.1333,  18.088,  26.94,
};

//
// PID控制器定义vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F+PID%E6%8E%A7%E5%88%B6%E5%99%A8%E5%AE%9A%E4%B9%89
user_pid_t L_Leg_L0_PID;     //常态
user_pid_t R_Leg_L0_PID;     //

user_pid_t L_Leg_L0_POS_PID; //收腿
user_pid_t R_Leg_L0_POS_PID; //
user_pid_t L_Leg_L0_SPD_PID; //
user_pid_t R_Leg_L0_SPD_PID; //

user_pid_t spinning_pid;//小陀螺PID

user_pid_t Roll_Comp_PID;    //ROLL补偿pid

user_pid_t Leg_Phi0_PID;     //防劈叉pid

user_pid_t gimbal_pitch_pid;//云台俯仰pid
user_pid_t gimbal_yaw_pid;//云台偏航pid

float target_Leg_L0 = LEG_MIN_LENTH;//目标腿长
float alpha_target_L0 = 0.01f;//低通滤波系数，越小越平滑，但响应越慢
float target_L_Leg_L0 = LEG_MIN_LENTH;
float target_R_Leg_L0 = LEG_MIN_LENTH;
uint8_t i;
int height_wait;
uint8_t temp1;

user_pid_t L_Leg_Middle_PID, R_Leg_Middle_PID;   //收腿角度pid
user_pid_t L_Leg_dphi0_PID, R_Leg_dphi0_PID;     //收腿角速度pid

uint8_t first_run = 1;//是否是第一次运行，第一次运行需要特殊处理一些变量的初始值

uint8_t start_mode = 0;//0为刚从急停退出来，1为正常行走，2为正在上楼模式
uint8_t upstares_mode = 0;//0为上楼收腿未完成，1为上楼收腿完成
int ready_count = 0;

uint8_t leg_state = 0;  //腿长状态 0为最短，1为中等，2为最长（未来会改）
int leg_state_count;

RampGenerator Target_Speed_Ramp;//目标速度斜坡发生器

// 一阶斜坡发生器更新函数
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

/**
 * @brief 初始化斜坡发生器
 * 
 * @param ramp 斜坡发生器结构体指针
 * @param startValue 开始值
 * @param targetValue 目标值
 * @param time 总时间
 * @param cycleTime 周期时间
 */
void rampInit(RampGenerator *ramp, float startValue, float targetValue, float time, float cycleTime)
{
    ramp->currentValue = startValue;
    ramp->targetValue = targetValue;
    // 计算步进值，这里需要注意的是，确保斜坡时间和周期时间都不为零来避免除以零的错误
    if (time != 0 && cycleTime != 0)
    {
        if(targetValue>= startValue)
        {
            ramp->step = (targetValue - startValue) * (cycleTime / time); // 计算步进值，确保在指定时间内达到目标值
        }
        else
        {
            ramp->step = (startValue - targetValue) * (cycleTime / time); // 计算步进值，确保在指定时间内达到目标值
        }
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
    Motor->Rx_Data.Speed = (int16_t)(Data[2] << 8 | Data[3]);
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

    CAN_Send_DM_Motor_Data(hfdcan, Motor.motor_id, data);//!丛庆拉的，没改名字，不是适配达妙电机的，是通用的发送函数
}

void DJI_Motor_Torque_Ctrl(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id, float torque)
{
    uint8_t data[8] = {0};

    int16_t tqControl = (int16_t)(((((torque / 14.88f) / 0.02f))/20.0f) * 16384);

    if(hfdcan == &hfdcan2)
    {
        L_LK9025.TX_data = tqControl;
    }

    if(hfdcan == &hfdcan1)
    {
        R_LK9025.TX_data = tqControl;
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

void Enable_LK_Motor(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id)
{
    uint8_t data[8] = {0};
    data[0] = 0x88;
    CAN_Send_DM_Motor_Data(hfdcan, motor_id, data);//!丛庆拉的，没改名字，不是适配达妙电机的，是通用的发送函数
}

void Disable_LK_Motor(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id)
{
    uint8_t data[8] = {0};
    data[0] = 0x80;
    CAN_Send_DM_Motor_Data(hfdcan, motor_id, data);//!丛庆拉的，没改名字，不是适配达妙电机的，是通用的发送函数
}

/**
 * @brief 9025电机的转矩控制函数
 * 
 * @param hfdcan 
 * @param motor_id 
 * @param torque 力矩值，单位牛米
 */
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

    CAN_Send_DM_Motor_Data(hfdcan, motor_id, data);//!丛庆拉的，没改名字，不是适配达妙电机的，是通用的发送函数
}

/**
 * @brief K矩阵融合
 * 
 */
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


/**
 * @brief 横滚补偿
 * 
 */
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

//pd单环腿长控制函数vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2Fpd%E5%8D%95%E7%8E%AF%E8%85%BF%E9%95%BF%E6%8E%A7%E5%88%B6%E5%87%BD%E6%95%B0
void Leg_L0_Control()
{                                                                                                                                                           
    if(leg_state_count > 0)          //?                                                                                                                                               
    {                                //? 延时                                                                                                                          
        leg_state_count --;          //?                                                                                                                                               
    }                                //?                                                                                                                           

    // if(leg_state_count == 0)                                                                                                              
    // {                                                                                                                  
    //     leg_state_count = 300;                                                                                                                 
    //     if(leg_state <= 1)                                                                                                                 
    //     leg_state ++;                                                                                                                  
    //     if(leg_state > 1)                                                                                                                  
    //     leg_state = 0;                                                                                                                                          
    // }                                                                                                                  
    // if(SBUS_CH.SW4 == 0 && leg_state_count == 0)                                                                                                                
    // {                                                                                                                   
    //     leg_state_count = 300;                                                                                                                  
    //     if(leg_state > 0)                                                                                                                   
    //     leg_state --;                                                                                                                   
    //     if(leg_state <= 0)                                                                                                                  
    //     leg_state = 0;                                                                                                                  
    // }                                                                                                                   

    //低通滤波                                                                                                                                                          
    target_Leg_L0 = alpha_target_L0 * (((Foot_Chassis.Target_Leg_State / 1.0f) * 0.22) + LEG_MIN_LENTH) + (1 - alpha_target_L0) * target_Leg_L0;                                                                                                                                                            

    if(target_Leg_L0 >= 0.40f)                                                                                                                                                          
    target_Leg_L0 = 0.40f;                                                                                                                                                          
    if(target_Leg_L0 <= LEG_MIN_LENTH)                                                                                                                                                          
    target_Leg_L0 = LEG_MIN_LENTH;                                                                                                                                                          

    target_L_Leg_L0 = target_Leg_L0;                                                                                                                                                            
    target_R_Leg_L0 = target_Leg_L0;                                                                                                                                                            

    // target_L_Leg_L0 = target_Leg_L0 + Roll_Comp_PID.output;                                                                                                                                                          
    // target_R_Leg_L0 = target_Leg_L0 - Roll_Comp_PID.output;                                                                                                                                                          

    if(target_L_Leg_L0 >= 0.40)                                                                                                                                                         
    target_L_Leg_L0 = 0.40;                                                                                                                                                         
    if(target_L_Leg_L0 <= LEG_MIN_LENTH)                                                                                                                                                            
    target_L_Leg_L0 = LEG_MIN_LENTH;                                                                                                                                                            

    if(target_R_Leg_L0 >= 0.40)                                                                                                                                                         
    target_R_Leg_L0 = 0.40;                                                                                                                                                         
    if(target_R_Leg_L0 <= LEG_MIN_LENTH)                                                                                                                                                            
    target_R_Leg_L0 = LEG_MIN_LENTH;                                                                                                                                                            

    PID_Set_Error(&L_Leg_L0_PID, VMC_L.L0, target_L_Leg_L0);                                                                                                                                                            
    PID_Set_Error(&R_Leg_L0_PID, VMC_R.L0, target_R_Leg_L0);                                                                                                                                                            

    PID_coculate(&L_Leg_L0_PID);                                                                                                                                                            
    PID_coculate(&R_Leg_L0_PID);                                                                                                                                                            
}

//speed_error | 计算前进速度误差 (yaw_error)
void Speed_Error_Set()
{
    speed_limit = 2.7f;     //车子最大速
    // rampInit(&Target_Speed_Ramp, target_body_speed, (((SBUS_CH.CH3 - 992.0f)/800.0f) * speed_limit), 0.3f, 0.002f);
    // rampIterate(&Target_Speed_Ramp);
    target_body_speed = Foot_Chassis.Target_Vy;
    
    float temp = ((0.7f - fabsf(yaw_error))/0.7f);
    if(temp < 0.0)
    temp = 0.0;

    target_body_speed = target_body_speed * temp;//yaw误差越大，目标速度越小，最大为100%，最小为0
    //// target_body_speed = alpha_target_body_speed * (((SBUS_CH.CH2 - 992.0f)/800.0f) * speed_limit) + (1 - alpha_target_body_speed) * target_body_speed;
    speed_error = target_body_speed - kalman_body_speed;

    if(speed_error >= speed_limit * 1.0f)
    speed_error = speed_limit * 1.0f;
    if(speed_error <= -speed_limit * 1.0f)
    speed_error = -speed_limit * 1.0f;
}

/**
 * @brief body_distance, target_body_distance, body_distance_error | 计算距离误差(kalman_body_speed, speed_error)，并且更新和
 * 
 */
void Distance_Error_Set()
{
    body_distance += kalman_body_speed * 0.002f;
    target_body_distance += (kalman_body_speed + speed_error) * 0.002f; //! kalman_body_speed + speed_error不等于target_body_speed，因为speed_error是经过限幅的，而target_body_speed是没有经过限幅的，始作俑者：25年丛庆
    body_distance_error = target_body_distance - body_distance;
}
/**
 * body_speed | 水平方向车身速度解算(VMC, INS.pitch_trans)
 */
void Body_Speed_Coculate()
{
    //算单侧轮子速度
    Wl = alpha_W * (-L_LK9025.Rx_Data.Velocity + VMC_L.d_b_phi0) + (1 - alpha_W) * Wl;
    //算单侧车身速度
    body_speed_L = alpha_body_speed * ((Wl * WHEEL_RADIUS) + VMC_L.d_b_phi0 * VMC_L.L0 * arm_cos_f32(VMC_L.b_phi0)) + (1 - alpha_body_speed) * body_speed_L;

    Wr = alpha_W * (R_LK9025.Rx_Data.Velocity + VMC_R.d_b_phi0) + (1 - alpha_W) * Wr;
    body_speed_R = alpha_body_speed * ((Wr * WHEEL_RADIUS) + VMC_R.d_b_phi0 * VMC_R.L0 * arm_cos_f32(VMC_R.b_phi0)) + (1 - alpha_body_speed) * body_speed_R;

    body_speed = (body_speed_L + body_speed_R) / 2.0f;
}

//! 这个函数的命名也太随意了吧，丛庆
//! PS:这个注释不是我写的，是你自己的ai备注的hhhhhhhh甚至知道你叫丛庆
 
//算左右VMC的phi1/phi4/L0/phi0
void VMC_Coculate()
{
    VMC_Set_phi1_phi4(&VMC_L, L_DM8009[1].Rx_Data.Position + PI, L_DM8009[0].Rx_Data.Position);
    VMC_Set_phi1_phi4(&VMC_R, R_DM8009[0].Rx_Data.Position + PI, R_DM8009[1].Rx_Data.Position);
    VMC_Get_L0_phi0(&VMC_L);
    VMC_Get_L0_phi0(&VMC_R);
}

//惯性导航系统数据处理，算出pitch_trans/yaw_trans/d_pitch/d_yaw
void INS_Coculate()
{
    pitch_trans[1] = pitch_trans[0];//!看不懂就看看变量的注释，有解释
    pitch_trans[0] = (pitch/180.0f) * PI;
    yaw_trans[1] = yaw_trans[0];
    yaw_trans[0] = (yaw/180.0f) * PI;
    d_pitch = alpha_d_pitch * ((pitch_trans[0] - pitch_trans[1])/0.002f) + (1 - alpha_d_pitch) * d_pitch;
    float temp = yaw_trans[0] - yaw_trans[1];//临时变量
    if (temp > PI) 
        temp -= 2 * PI;
    else if (temp < -PI)
        temp += 2 * PI;
    d_yaw = alpha_d_yaw * (temp/0.002f) + (1 - alpha_d_yaw) * d_yaw;

    if(first_run == 1)  //!听说是屎，后来好像不用了
    {
        target_yaw = yaw_trans[0];
        d_yaw = 0;
        first_run = 0;
    }
}

float yaw_error_slope_step = 0.01f;

//动态低通滤波系数
float alpha_yaw_error = 0.0f;//yaw误差低通滤波系数

//speed_error, yaw_error | 算yaw的误差，以及根据yaw误差调整target_body_speed进而调整speed_error()
void Yaw_Error_Coculate()
{
    float Yaw_motor_position;
    Yaw_motor_position = Yaw_DM4310.Rx_Data.Position - (-2.84f);//减的是零点

    //套圈处理
    if(Yaw_motor_position > PI)
    {
        Yaw_motor_position -= 2 * PI;
    }
    if(Yaw_motor_position < -PI)
    {
        Yaw_motor_position += 2 * PI;
    }

    alpha_yaw_error = Yaw_motor_position * Yaw_motor_position * 0.05f;//Yaw_motor_position越大，alpha越大，响应越慢，最大为0.5

    yaw_error = Yaw_motor_position * (1 - alpha_yaw_error) + yaw_error * alpha_yaw_error;//yaw误差低通滤波，响应速度根据yaw误差大小动态调整，yaw误差越大，响应越慢

    float yaw_error_max = 0;
    yaw_error_max = ((2.0f - fabsf(kalman_body_speed))/2.0f) * 1.5f;//速度越快，允许的yaw误差越小，最大为5度，最小为0.05度
    if(yaw_error_max <= 0.05f)
    {
        yaw_error_max = 0.05f;
    }

    //这里Speed_Error_Set要用的是原生yaw_error，所以要写在yaw_error_max之上
    Speed_Error_Set();

    if(yaw_error > yaw_error_max)
        yaw_error = yaw_error_max;
    if(yaw_error < -yaw_error_max)
        yaw_error = -yaw_error_max;
    
}

int L_Leg_State, R_Leg_State;   //收腿阶段，0为收腿中，1为起立过程中收腿完成，2为上台阶过程中收腿完成
int L_Ready_Count, R_Ready_Count;
int L_off_ground = 0;
int R_off_ground = 0;

float spinning_ramp_accel = 30.0f; // 小陀螺斜坡加速度 rad/s^2 // TODO: 调参
uint16_t motor_HZ = 500; //任务频率
float target_spinning_d_yaw = 12.0f; // 目标小陀螺yaw速度，单位为弧度每秒
float slope_target_spinning_d_yaw = 0.0f;

float wheel_track_R = 0.19242f; // 轮距半径，单位为米

float raw_yaw_error = 0.0f; // 小陀螺原始yaw误差

float spinning_pid_kp = 0.0f; // 小陀螺PID比例增益 // TODO: 调参
float spinning_pid_ki = 0.0025f; // 小陀螺PID积分增益 // TODO: 调参
float spinning_pid_kd = 0.0f; // 小陀螺PID微分增益 // TODO: 调参
float spinning_pid_integral_limit = 20.0f; // 小陀螺PID积分死区 // TODO: 调参
float spinning_pid_output_limit = 3.0f; // 小陀螺PID输出限幅 // TODO: 调参
float spinning_pid_i_limit = 6.0f; // 小陀螺PID积分限幅 // TODO: 调参
float spinning_pid_deadzone = 0.0f; // 小陀螺PID输出死区 // TODO: 调参

uint8_t gimbal_follow_flag = 0; // 1：刚站起来，云台跟随底盘 0：底盘跟随云台

uint8_t spinning_flag = 0; // 1：小陀螺运行中 0：小陀螺停止

uint8_t yaw_ctrl_mode = 0; // 0：下板控制 1：上板控制

float down_board_yaw_output = 0.0f; // 下板yaw输出

uint8_t gimbal_follow_flag_cnt = 0; // 刚站起来云台跟随底盘的计数器


//电机任务
void Motor_task(void const *argument)
{
  
//电机初始化vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F%E7%94%B5%E6%9C%BA%E5%88%9D%E5%A7%8B%E5%8C%96
        DM_Joint_Motor_Init(&L_DM8009[0], 54.0f, 3.14159265f, 45.0f, 0x01);
        DM_Joint_Motor_Init(&L_DM8009[1], 54.0f, 3.14159265f, 45.0f, 0x02);

        DM_Joint_Motor_Init(&R_DM8009[0], 54.0f, 3.14159265f, 45.0f, 0x01);
        DM_Joint_Motor_Init(&R_DM8009[1], 54.0f, 3.14159265f, 45.0f, 0x02);

        DM_Joint_Motor_Init(&Yaw_DM4310, 10.0f, 3.14159265f, 30.0f, 0x10);
        DM_Joint_Motor_Init(&Shooter_DM2325, 10.0f, 3.14159265f, 200.0f, 0x11);

//VMC初始化vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2FVMC%E5%88%9D%E5%A7%8B%E5%8C%96
        VMC_Init(&VMC_L, 0.210f, 0.250f, 0.250f, 0.210f, 0.0f, 1);
        VMC_Init(&VMC_R, 0.210f, 0.250f, 0.250f, 0.210f, 0.0f, 0);

//PID初始化vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2FPID%E5%88%9D%E5%A7%8B%E5%8C%96
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

        //小陀螺pid
        PID_INIT(&spinning_pid, spinning_pid_kp, spinning_pid_ki, spinning_pid_kd, spinning_pid_output_limit, spinning_pid_i_limit, spinning_pid_integral_limit, spinning_pid_deadzone); // TODO: 调参

        //云台pid   //TODO:调参
        PID_INIT(&gimbal_pitch_pid, 10, 0.002, 100, 150, 80, 10000, 0);
        PID_INIT(&gimbal_yaw_pid, 30, 0, 300, 6, 80, 10000, 0);

    osDelay(1000);

//机身pitch计算vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F%E6%9C%BA%E8%BA%ABpitch%E8%AE%A1%E7%AE%97
        pitch_trans[1] = pitch_trans[0];
        pitch_trans[0] = (pitch/180.0f) * PI;

//电机使能vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F%E7%94%B5%E6%9C%BA%E4%BD%BF%E8%83%BD
        Enable_DM_Motor_MIT(&hfdcan2, 0x01);
        osDelay(5);
        Enable_DM_Motor_MIT(&hfdcan2, 0x02);
        osDelay(5);
        
        Enable_DM_Motor_MIT(&hfdcan1, 0x01);
        osDelay(5);	
        Enable_DM_Motor_MIT(&hfdcan1, 0x02);
        osDelay(5);

        Enable_DM_Motor_MIT(&hfdcan3, 0x11);
        osDelay(5);
        Enable_DM_Motor_MIT(&hfdcan3, 0x10);
        osDelay(5);

//精确延时用
        TickType_t xLastWakeTime = xTaskGetTickCount();

        
    for(;;)
    {
        //刚启动收腿过程中
        if(start_mode == 0 && upstares_mode == 0)
        {   
            gimbal_follow_flag = 1;
            yaw_ctrl_mode = 0;
            first_run = 1;//第一次运行标记vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F%E7%AC%AC%E4%B8%80%E6%AC%A1%E8%BF%90%E8%A1%8C%E6%A0%87%E8%AE%B0

            //更新VMC的变量vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F%E6%9B%B4%E6%96%B0VMC%E7%9A%84%E5%8F%98%E9%87%8F
            VMC_Set_phi1_phi4(&VMC_L, L_DM8009[1].Rx_Data.Position + PI, L_DM8009[0].Rx_Data.Position);
            VMC_Set_phi1_phi4(&VMC_R, R_DM8009[0].Rx_Data.Position + PI, R_DM8009[1].Rx_Data.Position);
            VMC_Get_L0_phi0(&VMC_L);
            VMC_Get_L0_phi0(&VMC_R);

            //车身速度解算vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F%E8%BD%A6%E8%BA%AB%E9%80%9F%E5%BA%A6%E8%A7%A3%E7%AE%97
            Body_Speed_Coculate();

            //摆头防卡腿vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F%E6%91%86%E5%A4%B4%E9%98%B2%E5%8D%A1%E8%85%BF
            PID_Set_Error(&gimbal_yaw_pid, Yaw_DM4310.Rx_Data.Position, -2.3f);
            down_board_yaw_output = PID_coculate(&gimbal_yaw_pid);

            //是否姿态稳定在误差20°内的起立态vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F%E6%98%AF%E5%90%A6%E5%A7%BF%E6%80%81%E7%A8%B3%E5%AE%9A%E5%9C%A8%E8%AF%AF%E5%B7%AE20%C2%B0%E5%86%85%E7%9A%84%E8%B5%B7%E7%AB%8B%E6%80%81
            if(roll >= 20.0f || roll <= -20.0f || pitch >= 20.0f || pitch <= -20.0f)
            {
                Self_Righting_Step();
            }
            else 
            {
                g_self_righting_stage = SELF_RIGHTING_STAGE_EXTEND;
                //收腿过程腿长控制vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F%E6%94%B6%E8%85%BF%E8%BF%87%E7%A8%8B%E8%85%BF%E9%95%BF%E6%8E%A7%E5%88%B6
                PID_Set_Error(&L_Leg_L0_POS_PID, VMC_L.L0, 0.19f);//0.19这个值是通过反复试验得来的，目的是让腿在收腿过程中稍微有个前倾，防止完全竖直时不稳定
                PID_Set_Error(&R_Leg_L0_POS_PID, VMC_R.L0, 0.19f);
                PID_coculate(&L_Leg_L0_POS_PID);
                PID_coculate(&R_Leg_L0_POS_PID);

                PID_Set_Error(&L_Leg_L0_SPD_PID, VMC_L.d_L0, L_Leg_L0_POS_PID.output);
                PID_Set_Error(&R_Leg_L0_SPD_PID, VMC_R.d_L0, R_Leg_L0_POS_PID.output);
                PID_coculate(&L_Leg_L0_SPD_PID);
                PID_coculate(&R_Leg_L0_SPD_PID);

                //腿角度控制vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F%E8%85%BF%E8%A7%92%E5%BA%A6%E6%8E%A7%E5%88%B6
                if(L_Leg_State >= 1)
                {
                    PID_Set_Error(&L_Leg_Middle_PID, VMC_L.phi0, PI/2-0.2); //这个PI/2-0.2是为了让腿在收腿过程中稍微有个前倾，防止完全竖直时不稳定，丛庆加的
                    PID_coculate(&L_Leg_Middle_PID);
                    PID_Set_Error(&L_Leg_dphi0_PID, VMC_L.d_b_phi0, L_Leg_Middle_PID.output);
                    PID_coculate(&L_Leg_dphi0_PID);
                }
                if(R_Leg_State >= 1)
                {
                    PID_Set_Error(&R_Leg_Middle_PID, VMC_R.phi0, PI/2+0.2);
                    PID_coculate(&R_Leg_Middle_PID);
                    PID_Set_Error(&R_Leg_dphi0_PID, VMC_R.d_b_phi0, -R_Leg_Middle_PID.output);
                    PID_coculate(&R_Leg_dphi0_PID);
                }

                //收腿过程中VMC解算vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F%E6%94%B6%E8%85%BF%E8%BF%87%E7%A8%8B%E4%B8%ADVMC%E8%A7%A3%E7%AE%97
                VMC_Set_F0_T(&VMC_L, L_Leg_L0_SPD_PID.output, L_Leg_dphi0_PID.output);
                VMC_Set_F0_T(&VMC_R, R_Leg_L0_SPD_PID.output, -R_Leg_dphi0_PID.output);

                //轮子脱力vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F%E8%BD%AE%E5%AD%90%E8%84%B1%E5%8A%9B
                L_LK9025.Target_Torque = 0;
                R_LK9025.Target_Torque = 0;

                //腿长判断是否到达目标长度vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F%E8%85%BF%E9%95%BF%E5%88%A4%E6%96%AD%E6%98%AF%E5%90%A6%E5%88%B0%E8%BE%BE%E7%9B%AE%E6%A0%87%E9%95%BF%E5%BA%A6
                //!这史是丛庆写的
                if(L_Leg_State == 0 && fabsf(L_Leg_L0_POS_PID.error) <= 0.06)
                {
                    L_Ready_Count ++;
                }
                if(L_Leg_State == 0 && L_Ready_Count >= 50)//腿到目标长度
                {
                    L_Leg_State = 1;    //收腿完成
                    L_Ready_Count = 0;  //归零
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

                //腿长达标之后，判断腿角度是否到达目标角度vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F%E8%85%BF%E9%95%BF%E8%BE%BE%E6%A0%87%E4%B9%8B%E5%90%8E%EF%BC%8C%E5%88%A4%E6%96%AD%E8%85%BF%E8%A7%92%E5%BA%A6%E6%98%AF%E5%90%A6%E5%88%B0%E8%BE%BE%E7%9B%AE%E6%A0%87%E8%A7%92%E5%BA%A6
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
                    start_mode = 1; // 收腿完成，进入正常模式
                    //归零
                    R_Leg_State = 0;
                    L_Leg_State = 0;
                }
            }			
        }

        else if(start_mode == 1)
        {
            //占用率检测用的，留着吧，看不懂也不影响vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F%E5%8D%A0%E7%94%A8%E7%8E%87%E6%A3%80%E6%B5%8B%E7%94%A8%E7%9A%84%EF%BC%8C%E7%95%99%E7%9D%80%E5%90%A7%EF%BC%8C%E7%9C%8B%E4%B8%8D%E6%87%82%E4%B9%9F%E4%B8%8D%E5%BD%B1%E5%93%8D
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, 1);

            //惯性导航、VMC、水平方向车身速度解算vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F%E6%83%AF%E6%80%A7%E5%AF%BC%E8%88%AA%E3%80%81VMC%E3%80%81%E6%B0%B4%E5%B9%B3%E6%96%B9%E5%90%91%E8%BD%A6%E8%BA%AB%E9%80%9F%E5%BA%A6%E8%A7%A3%E7%AE%97
            INS_Coculate();
            VMC_Coculate();
            Body_Speed_Coculate();

            //算yaw的误差，以及根据yaw误差调整目标速度vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F%E7%AE%97yaw%E7%9A%84%E8%AF%AF%E5%B7%AE%EF%BC%8C%E4%BB%A5%E5%8F%8A%E6%A0%B9%E6%8D%AEyaw%E8%AF%AF%E5%B7%AE%E8%B0%83%E6%95%B4%E7%9B%AE%E6%A0%87%E9%80%9F%E5%BA%A6
            if(gimbal_follow_flag == 1)
            {
                Yaw_Error_Coculate();
                yaw_error = 0;//云台跟随底盘时，强制yaw误差为0，让底盘完全跟随云台
                Speed_Error_Set();

                //控制yaw电机归零
                PID_Set_Error(&gimbal_yaw_pid, Yaw_DM4310.Rx_Data.Position, -2.84f);
                down_board_yaw_output = PID_coculate(&gimbal_yaw_pid);

                if(fabsf(Yaw_DM4310.Rx_Data.Position - (-2.84f)) <= 0.01f)
                {
                    gimbal_follow_flag_cnt ++;
                }
                if(gimbal_follow_flag_cnt >= 100)
                {
                    gimbal_follow_flag = 0;//云台跟随底盘完成，切换到底盘跟随云台
                    gimbal_follow_flag_cnt = 0;
                }
                yaw_ctrl_mode = 0;//云台跟随底盘时，yaw控制模式为下板控制
            }
            if(gimbal_follow_flag == 0)
            {
                yaw_ctrl_mode = 1;//底盘跟随云台时，yaw控制模式为上板控制

                if(Foot_Chassis.Chassis_Mode == 0)
                {
                    if(spinning_flag == 1)
                    {
                        //小陀螺减速
                        // slope_target_spinning_d_yaw = easy_Slope(0, slope_target_spinning_d_yaw, spinning_ramp_accel * 0.002f);
                        PID_Set_Error(&spinning_pid, d_yaw, 0);
                        yaw_error = PID_coculate(&spinning_pid);
                        Speed_Error_Set();

                        if(fabsf(d_yaw) <= 4.0f && (Yaw_DM4310.Rx_Data.Position <=-0.5f || Yaw_DM4310.Rx_Data.Position >= 1.0f))
                        {
                            spinning_flag = 0;
                        }
                        
                    }
                    else
                    {
                        //常态
                        Yaw_Error_Coculate();
                        slope_target_spinning_d_yaw = 0;
                        spinning_pid.I = 0;

                        spinning_flag = 0;
                    }

                }
                //算小陀螺的yaw_error vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F%E7%AE%97%E5%B0%8F%E9%99%80%E8%9E%BA%E7%9A%84yaw_error
                if(Foot_Chassis.Chassis_Mode == 1)
                {
                    // slope_target_spinning_d_yaw = easy_Slope(target_spinning_d_yaw, slope_target_spinning_d_yaw, spinning_ramp_accel * 0.002f);
                    PID_Set_Error(&spinning_pid, d_yaw, 12.0f);
                    yaw_error = PID_coculate(&spinning_pid);
                    Speed_Error_Set();

                    spinning_flag = 1;
                }
            }
            // Speed_Error_Set();   //! 这个函数在Yaw_Error_Coculate里面被调用了，因为speed_error的计算需要用到yaw_error，所以放在Yaw_Error_Coculate里面更合理一些，虽然命名上可能有点奇怪，丛庆加的

            //计算距离误差vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F%E8%AE%A1%E7%AE%97%E8%B7%9D%E7%A6%BB%E8%AF%AF%E5%B7%AE
            Distance_Error_Set();

            //横滚补偿和PD单环腿长控制vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F%E6%A8%AA%E6%BB%9A%E8%A1%A5%E5%81%BF%E5%92%8CPD%E5%8D%95%E7%8E%AF%E8%85%BF%E9%95%BF%E6%8E%A7%E5%88%B6
            Roll_Comp();
            Leg_L0_Control();

            //放劈叉vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F%E6%94%BE%E5%8A%88%E5%8F%89
            PID_Set_Error(&Leg_Phi0_PID, (VMC_L.phi0 - PI/2) + (VMC_R.phi0 - PI/2), 0);
            PID_coculate(&Leg_Phi0_PID);

            //100hz算K值，毕竟K值的计算比较耗时vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F100hz%E7%AE%97K%E5%80%BC
            i++;
            if(i >= 5)
            {
                i = 0;
                LQR_Get_K(LQR_K, K_Fit_Coefficients, VMC_L.L0, VMC_R.L0);
            }

            //目前这是一个空函数
            b_phi0_offset_coc(target_Leg_L0);

            L_b_phi0 = VMC_L.b_phi0;
            R_b_phi0 = VMC_R.b_phi0;

            //算轮子力矩vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F%E7%AE%97%E8%BD%AE%E5%AD%90%E5%8A%9B%E7%9F%A9
            L_LK9025.Target_Torque = 
            + LQR_K[0][0] * body_distance_error
            + LQR_K[0][1] * (speed_error) 
            + LQR_K[0][2] * (yaw_error)
            - LQR_K[0][3] * d_yaw
            - LQR_K[0][4] * VMC_L.b_phi0 
            - LQR_K[0][5] * VMC_L.d_b_phi0 
            - LQR_K[0][6] * VMC_R.b_phi0 
            - LQR_K[0][7] * VMC_R.d_b_phi0 
            + LQR_K[0][8] * (pitch_trans[0] - PITCH_OFFSET)
            + LQR_K[0][9] * d_pitch;

            R_LK9025.Target_Torque = 
            + LQR_K[1][0] * body_distance_error
            + LQR_K[1][1] * (speed_error) 
            + LQR_K[1][2] * (yaw_error)
            - LQR_K[1][3] * d_yaw
            - LQR_K[1][4] * VMC_L.b_phi0 
            - LQR_K[1][5] * VMC_L.d_b_phi0 
            - LQR_K[1][6] * VMC_R.b_phi0 
            - LQR_K[1][7] * VMC_R.d_b_phi0 
            + LQR_K[1][8] * (pitch_trans[0] - PITCH_OFFSET)
            + LQR_K[1][9] * d_pitch;

            //算模拟腿力矩vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F%E7%AE%97%E6%A8%A1%E6%8B%9F%E8%85%BF%E5%8A%9B%E7%9F%A9
            Leg_L_T = 
            + LQR_K[2][0] * body_distance_error
            + LQR_K[2][1] * (speed_error)
            + LQR_K[2][2] * (-yaw_error)
            - LQR_K[2][3] * d_yaw
            - LQR_K[2][4] * (VMC_L.b_phi0 - b_phi0_offset)
            - LQR_K[2][5] * VMC_L.d_b_phi0 
            - LQR_K[2][6] * VMC_R.b_phi0 
            - LQR_K[2][7] * VMC_R.d_b_phi0
            + LQR_K[2][8] * (pitch_trans[0] - PITCH_OFFSET)
            + LQR_K[2][9] * d_pitch;

            Leg_R_T = 
            + LQR_K[3][0] * body_distance_error
            + LQR_K[3][1] * (speed_error)
            + LQR_K[3][2] * (-yaw_error)
            - LQR_K[3][3] * d_yaw
            - LQR_K[3][4] * VMC_L.b_phi0 
            - LQR_K[3][5] * VMC_L.d_b_phi0 
            - LQR_K[3][6] * (VMC_R.b_phi0 - b_phi0_offset)
            - LQR_K[3][7] * VMC_R.d_b_phi0
            + LQR_K[3][8] * (pitch_trans[0] - PITCH_OFFSET)
            + LQR_K[3][9] * d_pitch;

            //常态下VMC解算vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F%E5%B8%B8%E6%80%81%E4%B8%8BVMC%E8%A7%A3%E7%AE%97
            VMC_Set_F0_T(&VMC_L, L_Leg_L0_PID.output + (mg / arm_cos_f32(VMC_L.b_phi0)) + Roll_Comp_PID.output, Leg_L_T + Leg_Phi0_PID.output);
            VMC_Set_F0_T(&VMC_R, R_Leg_L0_PID.output + (mg / arm_cos_f32(VMC_R.b_phi0)) - Roll_Comp_PID.output, -Leg_R_T + Leg_Phi0_PID.output);

            // VMC_Set_F0_T(&VMC_L, L_Leg_L0_PID.output + (mg / arm_cos_f32(VMC_L.b_phi0)), Leg_L_T + Leg_Phi0_PID.output);
            // VMC_Set_F0_T(&VMC_R, R_Leg_L0_PID.output + (mg / arm_cos_f32(VMC_R.b_phi0)), -Leg_R_T + Leg_Phi0_PID.output);

            L_Ground_F0 = 0.1 * VMC_Get_Ground_F0(&VMC_L) + 0.9 * L_Ground_F0;
            R_Ground_F0 = 0.1 * VMC_Get_Ground_F0(&VMC_R) + 0.9 * R_Ground_F0;

            //离地检测滤波vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F%E7%A6%BB%E5%9C%B0%E6%A3%80%E6%B5%8B%E6%BB%A4%E6%B3%A2
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

            //!这段是先算一遍不离地的情况的数，再检测是否离地，如果离地，就再算一次覆盖掉
            if(L_off_ground >= 15)//正常行驶过程离地
            {
                //离地后腿归中，轮子脱力vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F%E7%A6%BB%E5%9C%B0%E5%90%8E%E8%85%BF%E5%BD%92%E4%B8%AD%EF%BC%8C%E8%BD%AE%E5%AD%90%E8%84%B1%E5%8A%9B
                Leg_L_T = 
                - LQR_K[2][4] * VMC_L.b_phi0 
                - LQR_K[2][5] * VMC_L.d_b_phi0 ;
                Leg_L_T *= 0.7; //收腿力度参数
                L_LK9025.Target_Torque = 0;//离地轮子脱力
                //正常行驶过程离地VMC解算vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F%E6%AD%A3%E5%B8%B8%E8%A1%8C%E9%A9%B6%E8%BF%87%E7%A8%8B%E7%A6%BB%E5%9C%B0VMC%E8%A7%A3%E7%AE%97
                VMC_Set_F0_T(&VMC_L, L_Leg_L0_PID.output + (mg / arm_cos_f32(VMC_L.b_phi0)), Leg_L_T);//VMC解算
                //离地距离相关量归零vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F%E7%A6%BB%E5%9C%B0%E8%B7%9D%E7%A6%BB%E7%9B%B8%E5%85%B3%E9%87%8F%E5%BD%92%E9%9B%B6
                body_distance = 0;
                target_body_distance = 2.0;
            }

            if(R_off_ground >= 15)
            {
                Leg_R_T = 
                - LQR_K[3][6] * VMC_R.b_phi0 
                - LQR_K[3][7] * VMC_R.d_b_phi0;
                Leg_R_T *= 0.7;
                R_LK9025.Target_Torque = 0;
                VMC_Set_F0_T(&VMC_R, R_Leg_L0_PID.output + (mg / arm_cos_f32(VMC_R.b_phi0)), -Leg_R_T);
                body_distance = 0;
                target_body_distance = 2.0;
            }

            if(upstairs_flag == 1)
            {
                start_mode = 2;
                upstairs_flag = 0;
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
        else if(start_mode == 2 && upstares_mode == 0)//正在磕台阶
        {
            //伸腿VMC状态参数计算vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F%E4%BC%B8%E8%85%BFVMC%E7%8A%B6%E6%80%81%E5%8F%82%E6%95%B0%E8%AE%A1%E7%AE%97
            VMC_Set_phi1_phi4(&VMC_L, L_DM8009[1].Rx_Data.Position + PI, L_DM8009[0].Rx_Data.Position);
            VMC_Set_phi1_phi4(&VMC_R, R_DM8009[0].Rx_Data.Position + PI, R_DM8009[1].Rx_Data.Position);
            VMC_Get_L0_phi0(&VMC_L);
            VMC_Get_L0_phi0(&VMC_R);

            //伸腿的水平方向车身速度解算  vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F%E4%BC%B8%E8%85%BF%E7%9A%84%E6%B0%B4%E5%B9%B3%E6%96%B9%E5%90%91%E8%BD%A6%E8%BA%AB%E9%80%9F%E5%BA%A6%E8%A7%A3%E7%AE%97
            Body_Speed_Coculate();

            //上台阶过程中轮子正转，防止滑下来vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F%E4%B8%8A%E5%8F%B0%E9%98%B6%E8%BF%87%E7%A8%8B%E4%B8%AD%E8%BD%AE%E5%AD%90%E6%AD%A3%E8%BD%AC%EF%BC%8C%E9%98%B2%E6%AD%A2%E6%BB%91%E4%B8%8B%E6%9D%A5
            L_LK9025.Target_Torque = 0.1;
            R_LK9025.Target_Torque = 0.1;

            // 磕台阶过程中双环腿长控制
            // vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F+%E7%A3%95%E5%8F%B0%E9%98%B6%E8%BF%87%E7%A8%8B%E4%B8%AD%E5%8F%8C%E7%8E%AF%E8%85%BF%E9%95%BF%E6%8E%A7%E5%88%B6
            PID_Set_Error(&L_Leg_L0_POS_PID, VMC_L.L0, 0.44);   //TODO: 写一个最大腿长的宏定义
            PID_Set_Error(&R_Leg_L0_POS_PID, VMC_R.L0, 0.44);
            PID_coculate(&L_Leg_L0_POS_PID);
            PID_coculate(&R_Leg_L0_POS_PID);

            PID_Set_Error(&L_Leg_L0_SPD_PID, VMC_L.d_L0, L_Leg_L0_POS_PID.output);
            PID_Set_Error(&R_Leg_L0_SPD_PID, VMC_R.d_L0, R_Leg_L0_POS_PID.output);
            PID_coculate(&L_Leg_L0_SPD_PID);
            PID_coculate(&R_Leg_L0_SPD_PID);

            //磕台阶过程中双环腿角度控制vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F%E7%A3%95%E5%8F%B0%E9%98%B6%E8%BF%87%E7%A8%8B%E4%B8%AD%E5%8F%8C%E7%8E%AF%E8%85%BF%E8%A7%92%E5%BA%A6%E6%8E%A7%E5%88%B6
            PID_Set_Error(&L_Leg_Middle_PID, VMC_L.phi0, PI/2 + 0.75);
            PID_coculate(&L_Leg_Middle_PID);
            PID_Set_Error(&L_Leg_dphi0_PID, VMC_L.d_b_phi0, L_Leg_Middle_PID.output);
            PID_coculate(&L_Leg_dphi0_PID);
            
            PID_Set_Error(&R_Leg_Middle_PID, VMC_R.phi0, PI/2 - 0.75);
            PID_coculate(&R_Leg_Middle_PID);
            PID_Set_Error(&R_Leg_dphi0_PID, VMC_R.d_b_phi0, -R_Leg_Middle_PID.output);
            PID_coculate(&R_Leg_dphi0_PID);

            //上台阶过程中VMC解算vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F%E4%B8%8A%E5%8F%B0%E9%98%B6%E8%BF%87%E7%A8%8B%E4%B8%ADVMC%E8%A7%A3%E7%AE%97
            VMC_Set_F0_T(&VMC_L, L_Leg_L0_SPD_PID.output, L_Leg_dphi0_PID.output);
            VMC_Set_F0_T(&VMC_R, R_Leg_L0_SPD_PID.output, -R_Leg_dphi0_PID.output);

            //上台阶收腿过程中判断腿长和腿角度是否都到位了vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F%E4%B8%8A%E5%8F%B0%E9%98%B6%E6%94%B6%E8%85%BF%E8%BF%87%E7%A8%8B%E4%B8%AD%E5%88%A4%E6%96%AD%E8%85%BF%E9%95%BF%E5%92%8C%E8%85%BF%E8%A7%92%E5%BA%A6%E6%98%AF%E5%90%A6%E9%83%BD%E5%88%B0%E4%BD%8D%E4%BA%86
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
            //收腿起立VMC状态参数计算vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F%E6%94%B6%E8%85%BF%E8%B5%B7%E7%AB%8BVMC%E7%8A%B6%E6%80%81%E5%8F%82%E6%95%B0%E8%AE%A1%E7%AE%97
            VMC_Set_phi1_phi4(&VMC_L, L_DM8009[1].Rx_Data.Position + PI, L_DM8009[0].Rx_Data.Position);
            VMC_Set_phi1_phi4(&VMC_R, R_DM8009[0].Rx_Data.Position + PI, R_DM8009[1].Rx_Data.Position);
            VMC_Get_L0_phi0(&VMC_L);
            VMC_Get_L0_phi0(&VMC_R);

            //收腿起立的水平方向车身速度解算vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F%E6%94%B6%E8%85%BF%E8%B5%B7%E7%AB%8B%E7%9A%84%E6%B0%B4%E5%B9%B3%E6%96%B9%E5%90%91%E8%BD%A6%E8%BA%AB%E9%80%9F%E5%BA%A6%E8%A7%A3%E7%AE%97
            Body_Speed_Coculate();

            //收腿起立的腿长双环控制vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F%E6%94%B6%E8%85%BF%E8%B5%B7%E7%AB%8B%E7%9A%84%E8%85%BF%E9%95%BF%E5%8F%8C%E7%8E%AF%E6%8E%A7%E5%88%B6
            //!这他妈是史啊，写这段何意味
            PID_Set_Error(&L_Leg_L0_POS_PID, VMC_L.L0, 0.16f);
            PID_Set_Error(&R_Leg_L0_POS_PID, VMC_R.L0, 0.16f);
            PID_coculate(&L_Leg_L0_POS_PID);
            PID_coculate(&R_Leg_L0_POS_PID);

            PID_Set_Error(&L_Leg_L0_SPD_PID, VMC_L.d_L0, L_Leg_L0_POS_PID.output);
            PID_Set_Error(&R_Leg_L0_SPD_PID, VMC_R.d_L0, R_Leg_L0_POS_PID.output);
            PID_coculate(&L_Leg_L0_SPD_PID);
            PID_coculate(&R_Leg_L0_SPD_PID);

            
            if(L_Leg_State >= 1)//腿长缩短完成，腿后伸完成
            {
                //收腿到准备起立态的角度双环vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F%E6%94%B6%E8%85%BF%E5%88%B0%E5%87%86%E5%A4%87%E8%B5%B7%E7%AB%8B%E6%80%81%E7%9A%84%E8%A7%92%E5%BA%A6%E5%8F%8C%E7%8E%AF
                PID_Set_Error(&L_Leg_Middle_PID, VMC_L.phi0, PI/2-0.2);//这个PI/2-0.2是为了让腿在收腿过程中稍微有个前倾，防止完全竖直时不稳定
                PID_coculate(&L_Leg_Middle_PID);
                PID_Set_Error(&L_Leg_dphi0_PID, VMC_L.d_b_phi0, L_Leg_Middle_PID.output);
                PID_coculate(&L_Leg_dphi0_PID);
            }
            else 
            {
                //伸腿到腿向后伸长态的角度双环vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F%E4%BC%B8%E8%85%BF%E5%88%B0%E8%85%BF%E5%90%91%E5%90%8E%E4%BC%B8%E9%95%BF%E6%80%81%E7%9A%84%E8%A7%92%E5%BA%A6%E5%8F%8C%E7%8E%AF
                PID_Set_Error(&L_Leg_Middle_PID, VMC_L.phi0, PI/2+1.2);//这个PI/2+1.2是为了让腿在收腿过程中先向后伸长，再收回来的时候更平滑
                PID_coculate(&L_Leg_Middle_PID);
                PID_Set_Error(&L_Leg_dphi0_PID, VMC_L.d_b_phi0, L_Leg_Middle_PID.output);
                PID_coculate(&L_Leg_dphi0_PID);
            }
            if(R_Leg_State >= 1)
            {
                PID_Set_Error(&R_Leg_Middle_PID, VMC_R.phi0, PI/2+0.2);
                PID_coculate(&R_Leg_Middle_PID);
                PID_Set_Error(&R_Leg_dphi0_PID, VMC_R.d_b_phi0, -R_Leg_Middle_PID.output);
                PID_coculate(&R_Leg_dphi0_PID);
            }
            else
            {
                PID_Set_Error(&R_Leg_Middle_PID, VMC_R.phi0, PI/2-1.2);
                PID_coculate(&R_Leg_Middle_PID);
                PID_Set_Error(&R_Leg_dphi0_PID, VMC_R.d_b_phi0, -R_Leg_Middle_PID.output);
                PID_coculate(&R_Leg_dphi0_PID);
            }

            //收腿起立VMC，轮力矩解算vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F%E6%94%B6%E8%85%BF%E8%B5%B7%E7%AB%8BVMC%EF%BC%8C%E8%BD%AE%E5%8A%9B%E7%9F%A9%E8%A7%A3%E7%AE%97
            VMC_Set_F0_T(&VMC_L, L_Leg_L0_SPD_PID.output, L_Leg_dphi0_PID.output);
            VMC_Set_F0_T(&VMC_R, R_Leg_L0_SPD_PID.output, -R_Leg_dphi0_PID.output);

            L_LK9025.Target_Torque = 0.5f;
            R_LK9025.Target_Torque = 0.5f;

            //腿收短，后伸检测vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F%E8%85%BF%E6%94%B6%E7%9F%AD%EF%BC%8C%E5%90%8E%E4%BC%B8%E6%A3%80%E6%B5%8B
            if(L_Leg_State == 0 && fabsf(L_Leg_L0_POS_PID.error) <= 0.01 && fabsf(L_Leg_Middle_PID.error) <= 0.01)//腿长和腿角度都到位了
            {
                L_Ready_Count ++;
            }
            if(L_Leg_State == 0 && L_Ready_Count >= 100)//
            {
                L_Leg_State = 1;//腿长缩短完成，腿后伸完成，准备收腿
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

            //腿长缩短完成，腿后伸完成之后，判断腿角度是否到达准备起立的目标角度，达到后轮子脱力vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F%E8%85%BF%E9%95%BF%E7%BC%A9%E7%9F%AD%E5%AE%8C%E6%88%90%EF%BC%8C%E8%85%BF%E5%90%8E%E4%BC%B8%E5%AE%8C%E6%88%90%E4%B9%8B%E5%90%8E%EF%BC%8C%E5%88%A4%E6%96%AD%E8%85%BF%E8%A7%92%E5%BA%A6%E6%98%AF%E5%90%A6%E5%88%B0%E8%BE%BE%E5%87%86%E5%A4%87%E8%B5%B7%E7%AB%8B%E7%9A%84%E7%9B%AE%E6%A0%87%E8%A7%92%E5%BA%A6%EF%BC%8C%E8%BE%BE%E5%88%B0%E5%90%8E%E8%BD%AE%E5%AD%90%E8%84%B1%E5%8A%9B
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

            //状态量归位vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F%E7%8A%B6%E6%80%81%E9%87%8F%E5%BD%92%E4%BD%8D
            if(R_Leg_State == 2 && L_Leg_State == 2)
            {
                upstares_mode = 0;
                start_mode = 0;
                R_Leg_State = 1;
                L_Leg_State = 1;
                leg_state = 0;
                target_Leg_L0 = LEG_MIN_LENTH;
            }	
        }
        osDelayUntil(&xLastWakeTime, 2);//精确延时2毫秒，同时更新xLastWakeTime的值为当前时间
    }

    
}
