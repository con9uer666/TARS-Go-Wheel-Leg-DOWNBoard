#include "controller.h"
#include "controller.h"
#include "fdcan.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "remoter.h"
#include "user_pid.h"
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
#include "Board2Board.h"
#include "Slope.h"
#include "Angle_about.h"
#include "Slope.h"
#include "Wheel_Leg_about.h"


/*====================================== 附属函数变量 =========================================== */


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

float raw_yaw_error;
float last_yaw_error;
float yaw_error_step = 0.01f;//yaw误差斜坡步长

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
user_pid_t L_Leg_L0_POS_PID; //收腿
user_pid_t R_Leg_L0_POS_PID; //
user_pid_t L_Leg_L0_SPD_PID; //
user_pid_t R_Leg_L0_SPD_PID; //

user_pid_t spinning_pid;//小陀螺PID
user_pid_t spinning_speed_pid;//小陀螺减速PID

user_pid_t Roll_Comp_PID;    //ROLL补偿pid

user_pid_t Leg_Phi0_PID;     //防劈叉pid

user_pid_t gimbal_pitch_pid;//云台俯仰pid

user_pid_t gimbal_yaw_speed_pid;//云台偏航速度环pid
user_pid_t gimbal_yaw_angle_pid;//云台偏航角度环pid

float target_Leg_L0 = LEG_MIN_LENTH;//目标腿长
float alpha_target_L0 = 0.005f;//低通滤波系数，越小越平滑，但响应越慢
float target_L_Leg_L0 = LEG_MIN_LENTH;
float target_R_Leg_L0 = LEG_MIN_LENTH;
uint8_t i;
int height_wait;
uint8_t temp1;

user_pid_t L_Leg_Middle_PID, R_Leg_Middle_PID;   //收腿角度pid
user_pid_t L_Leg_dphi0_PID, R_Leg_dphi0_PID;     //收腿角速度pid
user_pid_t L_Leg_Middle_PID, R_Leg_Middle_PID;   //收腿角度pid
user_pid_t L_Leg_dphi0_PID, R_Leg_dphi0_PID;     //收腿角速度pid

uint8_t first_run = 1;//是否是第一次运行，第一次运行需要特殊处理一些变量的初始值

uint8_t start_mode = 0;//0为刚从急停退出来，1为正常行走，2为正在上楼模式
uint8_t upstares_mode = 0;//0为未开始上楼收腿，1为开始上楼收腿
int ready_count = 0;

uint8_t leg_state = 0;  //腿长状态 0为最短，1为中等，2为最长（未来会改）
int leg_state_count;

RampGenerator Target_Speed_Ramp;//目标速度斜坡发生器

/*============================= 任务变量 ================================= */

//?标志位
uint8_t gimbal_follow_flag = 0; // 1：刚站起来，云台跟随底盘 0：底盘跟随云台
uint8_t spinning_flag = 0; // 1：小陀螺运行中 0：小陀螺停止
uint8_t spinning_usable = 1; // 小陀螺是否可用，0为不可用，1为可用
uint8_t L_Leg_State, R_Leg_State;   //收腿阶段，0为收腿中，1为起立过程中收腿完成，2为上台阶过程中收腿完成

//?计数器
uint16_t L_Ready_Count, R_Ready_Count;
int L_off_ground = 0;   //必须是int类型，因为要减去计数器，不能无符号
int R_off_ground = 0;   //必须是int类型，因为要减去计数器，不能无符号
uint16_t gimbal_follow_flag_cnt = 0; // 刚站起来云台跟随底盘的计数器

//?常量
uint16_t motor_HZ = 500; //任务频率
float head_forward_angle = -2.9f;//正视前方的yaw电机角度
float wheel_track_R = 0.19242f; // 轮距半径，单位为米

//?调参
float target_spinning_d_yaw = 8.0f; // 目标小陀螺yaw速度，单位为弧度每秒

//?中间参数
float down_board_yaw_output = 0.0f; // 下板yaw输出

float yaw_angle_PI = 0.0f;//标零处理后的yaw角度，单位rad，范围在[-PI, PI]内

/*============================= 斜坡相关 ================================= */

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






/*****************************************************************************************************
 *                                                                                                   * 
 *                                                                                                   * 
 *                                              控制函数                                              *
 *                                                                                                   * 
 *                                                                                                   * 
 *****************************************************************************************************/

/*===============================================初始化函数===============================================*/
//电机初始化参数及结构体
void task_Motor_Init()
{
    DM_Joint_Motor_Init(&L_DM8009[0], 54.0f, 3.14159265f, 45.0f, 0x01);
    DM_Joint_Motor_Init(&L_DM8009[1], 54.0f, 3.14159265f, 45.0f, 0x02);

    DM_Joint_Motor_Init(&R_DM8009[0], 54.0f, 3.14159265f, 45.0f, 0x01);
    DM_Joint_Motor_Init(&R_DM8009[1], 54.0f, 3.14159265f, 45.0f, 0x02);

    DM_Joint_Motor_Init(&Yaw_DM4310, 10.0f, 3.14159265f, 30.0f, 0x10);
    DM_Joint_Motor_Init(&Shooter_DM2325, 10.0f, 3.14159265f, 200.0f, 0x11);
}

//VMC赋值与初始化结构体
void task_VMC_Init()
{
    VMC_Init(&VMC_L, 0.210f, 0.250f, 0.250f, 0.210f, 0.0f, 1);
    VMC_Init(&VMC_R, 0.210f, 0.250f, 0.250f, 0.210f, 0.0f, 0);
}

//PID赋值与初始化结构体
void task_PID_Init()
{
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
    PID_INIT(&spinning_pid, 0.0f, 0.0025f, 0, 3.0f, 6.0f, 20.0f, 0);

    //云台pid
    PID_INIT(&gimbal_pitch_pid, 10, 0.002, 100, 150, 80, 10000, 0);
    PID_INIT(&spinning_speed_pid, -6, 0, 0, 6, 0, 0, 0);
    PID_INIT(&gimbal_yaw_angle_pid, 80, 0,1, 6, 80, 10000, 0);
    PID_INIT(&gimbal_yaw_speed_pid, 0.3, 0.00, 0.3, 5, 80, 10000, 0);
    // PID_INIT(&gimbal_follow_error_pid, 3, 0.002, 100, 150, 80, 10000, 0);
}

//机身pitch计算，记录前一帧的pitch值，单位为弧度，-PI到PI之间
void task_Pitch_Coculate()
{
    pitch_trans[1] = pitch_trans[0];
    pitch_trans[0] = (pitch/180.0f) * PI;
}

//全部电机使能
void task_Motor_Enable()
{
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
}

/*===============================================运动函数===============================================*/

//未站起 + 未上楼收腿  函数
void NotStanding_NotStairRetract()
{

    //腿长判断是否到达目标长度
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

    //腿长达标之后，判断腿角度是否到达目标角度
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



    //标志位
    gimbal_follow_flag = 1; //下板控制云台

    VMC_Coculate();
    Body_Speed_Coculate();//车身速度解算

    //位速双环PID摆头防卡腿
    PID_Set_Error(&gimbal_yaw_angle_pid, yaw_angle_PI, 0);
    PID_Set_Error(&gimbal_yaw_speed_pid, Yaw_DM4310.Rx_Data.Velocity, PID_coculate(&gimbal_yaw_angle_pid));
    down_board_yaw_output = PID_coculate(&gimbal_yaw_speed_pid);

    //是否姿态稳定在误差20°内的起立态
    if((roll >= 20.0f || roll <= -20.0f || pitch >= 20.0f || pitch <= -20.0f) && first_run == 1)//不稳定且是急停开始第一次运行
    {
        Self_Righting_Step();
    }
    else
    {
        //倒地自起成功后复位Self_Righting_Step的状态机
        g_self_righting_stage = SELF_RIGHTING_STAGE_EXTEND;
        first_run = 0;//第一次运行完成

        //收腿过程腿长控制
        PID_Set_Error(&L_Leg_L0_POS_PID, VMC_L.L0, 0.19f);//0.19这个值是通过反复试验得来的，目的是让腿在收腿过程中稍微有个前倾，防止完全竖直时不稳定
        PID_Set_Error(&R_Leg_L0_POS_PID, VMC_R.L0, 0.19f);
        PID_coculate(&L_Leg_L0_POS_PID);
        PID_coculate(&R_Leg_L0_POS_PID);

        PID_Set_Error(&L_Leg_L0_SPD_PID, VMC_L.d_L0, L_Leg_L0_POS_PID.output);
        PID_Set_Error(&R_Leg_L0_SPD_PID, VMC_R.d_L0, R_Leg_L0_POS_PID.output);
        PID_coculate(&L_Leg_L0_SPD_PID);
        PID_coculate(&R_Leg_L0_SPD_PID);

        //腿角度控制
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

        //映射到电机力矩
        VMC_Set_F0_T(&VMC_L, L_Leg_L0_SPD_PID.output, L_Leg_dphi0_PID.output);
        VMC_Set_F0_T(&VMC_R, R_Leg_L0_SPD_PID.output, -R_Leg_dphi0_PID.output);
        L_LK9025.Target_Torque = 0;
        R_LK9025.Target_Torque = 0;

        
        
    }
}

void LQR_calculate()
{
    //算轮子力矩
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

    //算模拟腿力矩
    Leg_L_T = 
    + LQR_K[2][0] * body_distance_error
    + LQR_K[2][1] * (speed_error)
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
}

void slip_fliter()
{
    L_Ground_F0 = 0.1 * VMC_Get_Ground_F0(&VMC_L) + 0.9 * L_Ground_F0;
    R_Ground_F0 = 0.1 * VMC_Get_Ground_F0(&VMC_R) + 0.9 * R_Ground_F0;

    //离地检测滤波
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
}

//站起
void Standing()
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

        PID_Set_Error(&gimbal_yaw_speed_pid, Yaw_DM4310.Rx_Data.Velocity, PID_coculate(&gimbal_yaw_angle_pid));
        down_board_yaw_output = PID_coculate(&gimbal_yaw_speed_pid);

        if(fabsf(yaw_angle_PI) <= 0.1f)
        {
            gimbal_follow_flag_cnt ++;
        }
        if(gimbal_follow_flag_cnt >= 50)
        {
            gimbal_follow_flag = 0;//云台跟随底盘完成，切换到底盘跟随云台
            gimbal_follow_flag_cnt = 0;
        }
    }
    if(gimbal_follow_flag == 0)
    {
        //算小陀螺的
        if(Foot_Chassis.Chassis_Mode == 1 && spinning_usable == 1)
        {
            PID_Set_Error(&spinning_pid, d_yaw, target_spinning_d_yaw);
            yaw_error = PID_coculate(&spinning_pid);
            Speed_Error_Set();

            spinning_flag = 1;
        }
        else
        {
            if(spinning_flag == 1)//小陀螺减速
            {
                spinning_usable = 0;
                //小陀螺减速
                // slope_target_spinning_d_yaw = easy_Slope(0, slope_target_spinning_d_yaw, spinning_ramp_accel * 0.002f);
                PID_Set_Error(&spinning_pid, d_yaw, 0);
                yaw_error = PID_coculate(&spinning_pid);
                Speed_Error_Set();//TODO:不知道加不加

                if((fabsf(d_yaw) <= 10.0f && yaw_angle_PI >= 0) || (fabsf(d_yaw) <= 3.0f))
                {
                    spinning_flag = 2;
                }
            }
            else if(spinning_flag == 2)//双环减速，目标头方向
            {
                //小陀螺急停
                PID_Set_Error(&spinning_speed_pid, yaw_angle_PI, 0);
                float spinning_speed_output = PID_coculate(&spinning_speed_pid);
                PID_Set_Error(&spinning_pid, d_yaw, spinning_speed_output);
                yaw_error = PID_coculate(&spinning_pid);
                Speed_Error_Set();//TODO:不知道加不加

                if((fabsf(yaw_angle_PI) <= 0.1f && fabsf(d_yaw) <= 4.0f) || (fabsf(d_yaw) <= 0.05f))
                {
                    spinning_flag = 0;
                    spinning_usable = 1;
                }
            }
            else if(spinning_flag == 0)//常态
            {
                spinning_pid.I = 0;
                spinning_usable = 1;
                spinning_flag = 0;
                Yaw_Error_Coculate();
            }
        }
    }

    yaw_error = 0.05 * yaw_error + 0.95 * last_yaw_error;
    last_yaw_error = yaw_error;

    //计算距离误差
    Distance_Error_Set();

    //横滚补偿和PD单环腿长控制
    Roll_Comp();
    Leg_L0_Control();

    //放劈叉
    PID_Set_Error(&Leg_Phi0_PID, (VMC_L.phi0 - PI/2) + (VMC_R.phi0 - PI/2), 0);
    PID_coculate(&Leg_Phi0_PID);

    //100hz算K值，毕竟K值的计算比较耗时
    i++;
    if(i >= 5)
    {
        i = 0;
        LQR_Get_K(LQR_K, K_Fit_Coefficients, VMC_L.L0, VMC_R.L0);
    }

    LQR_calculate();

    //常态下VMC解算
    VMC_Set_F0_T(&VMC_L, L_Leg_L0_PID.output + (mg / arm_cos_f32(VMC_L.b_phi0)) + Roll_Comp_PID.output, Leg_L_T + Leg_Phi0_PID.output);
    VMC_Set_F0_T(&VMC_R, R_Leg_L0_PID.output + (mg / arm_cos_f32(VMC_R.b_phi0)) - Roll_Comp_PID.output, -Leg_R_T + Leg_Phi0_PID.output);

    slip_fliter();

    if(upstairs_flag == 1)
    {
        start_mode = 2;
    }

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, 0);
}

void Upstair_NotStairRetract()
{
    VMC_Coculate();
    Body_Speed_Coculate();

    //上台阶过程中轮子正转，防止滑下来
    L_LK9025.Target_Torque = 0.1;
    R_LK9025.Target_Torque = 0.1;

    // 磕台阶过程中双环腿长控制
    PID_Set_Error(&L_Leg_L0_POS_PID, VMC_L.L0, 0.44);   //TODO: 写一个最大腿长的宏定义
    PID_Set_Error(&R_Leg_L0_POS_PID, VMC_R.L0, 0.44);
    PID_coculate(&L_Leg_L0_POS_PID);
    PID_coculate(&R_Leg_L0_POS_PID);

    PID_Set_Error(&L_Leg_L0_SPD_PID, VMC_L.d_L0, L_Leg_L0_POS_PID.output);
    PID_Set_Error(&R_Leg_L0_SPD_PID, VMC_R.d_L0, R_Leg_L0_POS_PID.output);
    PID_coculate(&L_Leg_L0_SPD_PID);
    PID_coculate(&R_Leg_L0_SPD_PID);

    //磕台阶过程中双环腿角度控制
    PID_Set_Error(&L_Leg_Middle_PID, VMC_L.phi0, PI/2 + 1.2);
    PID_coculate(&L_Leg_Middle_PID);
    PID_Set_Error(&L_Leg_dphi0_PID, VMC_L.d_b_phi0, L_Leg_Middle_PID.output);
    PID_coculate(&L_Leg_dphi0_PID);
    
    PID_Set_Error(&R_Leg_Middle_PID, VMC_R.phi0, PI/2 - 1.2);
    PID_coculate(&R_Leg_Middle_PID);
    PID_Set_Error(&R_Leg_dphi0_PID, VMC_R.d_b_phi0, -R_Leg_Middle_PID.output);
    PID_coculate(&R_Leg_dphi0_PID);

    //上台阶过程中VMC解算vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F%E4%B8%8A%E5%8F%B0%E9%98%B6%E8%BF%87%E7%A8%8B%E4%B8%ADVMC%E8%A7%A3%E7%AE%97
    VMC_Set_F0_T(&VMC_L, L_Leg_L0_SPD_PID.output, L_Leg_dphi0_PID.output);
    VMC_Set_F0_T(&VMC_R, R_Leg_L0_SPD_PID.output, -R_Leg_dphi0_PID.output);

    //上台阶收腿过程中判断腿长和腿角度是否都到位了
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

void StairRetract()
{
    VMC_Coculate();
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
    



/*****************************************************************************************************
 *                                                                                                   * 
 *                                                                                                   * 
 *                                            控制任务                                                *
 *                                                                                                   * 
 *                                                                                                   * 
 *****************************************************************************************************/


void Motor_task(void const *argument)
{
    task_Motor_Init();
    task_VMC_Init();
    task_PID_Init(); 
    osDelay(1000);

    task_Pitch_Coculate();
    task_Motor_Enable();

    //精确延时用
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for(;;)
    {
        //计算标零后角度值
        yaw_angle_PI = easy_angle_normalize(head_forward_angle, Yaw_DM4310.Rx_Data.Position);

        //刚启动收腿过程中
        if(start_mode == 0 && upstares_mode == 0)//未站起 + 未上楼收腿
        {  
            NotStanding_NotStairRetract();
        }

        else if(start_mode == 1)//站起
        {
            Standing();
        }
        else if(start_mode == 2 && upstares_mode == 0)//上楼梯模式 + 未上楼收腿
        {
            Upstair_NotStairRetract();
        }
        else if(upstares_mode == 1)//收腿起立
        {
            StairRetract();
        }

        osDelayUntil(&xLastWakeTime, 2);//精确延时2毫秒，同时更新xLastWakeTime的值为当前时间
    }

    
}
