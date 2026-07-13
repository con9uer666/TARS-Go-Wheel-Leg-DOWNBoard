/**
 * @file chassis_state.c
 * @brief 底盘跨动作共享状态：反馈量、物理常数、控制流标志、共享腿部 PID。
 *
 * 这些变量不专属于任何单一动作组（被站起/收腿/上台阶/坐地等多处共用），
 * 故集中定义于此，其余动作专属变量定义在各自动作文件中。
 * 全部 extern 声明见 chassis_behavior_tree.h。
 */

#include "chassis_behavior_tree.h"

Foot_Chassis_t Foot_Chassis;   //轮足底盘结构体

float powerPredict;

float LEG_MIN_LENTH = 0.23f;
float LEG_MAX_LENTH = 0.39f;

float L_b_phi0, R_b_phi0;

//!屎作俑者：25年丛庆  数组0为当前pitch值，数组1为上一次的pitch值  单位为弧度
float pitch_trans[2];
float d_pitch;            //pitch速度，单位为弧度每秒
float alpha_d_pitch = 1.0;//滤波系数

float Wr, Wl;             //加上杆角速度的车轮速度
float alpha_W = 0.9;      //滤波系数
float body_speed_L, body_speed_R, body_speed; //当前车体速度 ,已正交，是水平方向的速度
float target_body_speed;  //目标速度
float speed_limit = 1.3;
float speed_error;
float alpha_target_body_speed = 1.0;
float alpha_body_speed = 1.0;//单侧算车体速度 滤波系数
float body_distance;

float target_body_distance = 0.0f;
float body_distance_error;

float yaw_error;// 弧度制
//!屎作俑者：25年丛庆  数组0为当前pitch值，数组1为上一次的pitch值     单位为弧度
float yaw_trans[2];
float d_yaw;              //陀螺仪yaw速度，单位为弧度每秒
float alpha_d_yaw = 0.8f;

float alpha_target_roll = 0.05;

float Leg_F0_Limit = 500;

float mg = 60.0f/2;

float b_phi0_offset = 0.0f; //腿部前倾角偏置，单位为弧度，正值使腿部前倾，负值使腿部后倾。用于抵消重心前移引起的前倾，或小陀螺时抵消反作用引起的前倾

// 共享腿部 PID（被收腿/起立/上台阶/坐地等多处使用，由 task_PID_Init 统一初始化）
user_pid_t L_Leg_L0_PID;     //常态
user_pid_t R_Leg_L0_PID;     //

user_pid_t L_Leg_L0_POS_PID; //收腿
user_pid_t R_Leg_L0_POS_PID; //
user_pid_t L_Leg_L0_SPD_PID; //
user_pid_t R_Leg_L0_SPD_PID; //

user_pid_t Roll_Comp_PID;    //ROLL补偿pid

user_pid_t Leg_AntiSplit_PID;         //防劈叉pid - Kp/Kd每周期由腿长1D二次拟合得到

user_pid_t L_Leg_Middle_PID, R_Leg_Middle_PID;   //收腿角度pid
user_pid_t L_Leg_dphi0_PID, R_Leg_dphi0_PID;     //收腿角速度pid

user_pid_t gimbal_pitch_pid;//云台俯仰pid

float target_Leg_L0 = 0.23f;//目标腿长, 初值与 LEG_MIN_LENTH 保持一致
float target_L_Leg_L0 = 0.20f;
float target_R_Leg_L0 = 0.20f;

int height_wait;
uint8_t temp1;

uint8_t first_run = 1;//是否是第一次运行，第一次运行需要特殊处理一些变量的初始值

uint8_t upstares_mode = 0;//0为未开始上楼收腿，1为开始上楼收腿
int ready_count = 0;

uint8_t leg_state = 0;  //腿长状态 0为最短，1为中等，2为最长（未来会改）
int leg_state_count;

RampGenerator Target_Speed_Ramp;//目标速度斜坡发生器

uint8_t gimbal_follow_flag = 1; // 1：刚站起来，云台跟随底盘 0：底盘跟随云台

uint16_t motor_HZ = 500; //任务频率

float wheel_track_R = 0.19242f; // 轮距半径，单位为米

float down_board_yaw_output = 0.0f; // 下板yaw输出
