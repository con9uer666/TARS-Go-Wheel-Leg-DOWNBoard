/**
 * @file motor.h
 * @brief 运动控制主模块：底盘控制、腿长控制、姿态补偿、跳跃模式等的类型定义与接口声明。
 *
 * 核心结构体：
 *   - RampGenerator      一阶斜坡发生器（平滑目标值过渡）
 *   - Leg_Info_t         单腿信息（当前腿长 L0）
 *   - Foot_Chassis_Info_t 底盘状态（yaw 角度、速度、左右腿信息）
 *   - Foot_Chassis_t     底盘目标与控制模式
 *
 * 关键外部变量：
 *   - Wr, Wl             左右轮端含杆角速度的合成转速
 *   - body_speed_L/R/body_speed  卡尔曼滤波后的车体水平速度
 *   - LQR_K[4][10]       LQR 反馈增益矩阵（距离/速度/yaw/腿角/腿角速度/pitch）
 *   - pitch_trans[2]     pitch 当前值与上一拍值（用于 d_pitch 计算）
 *   - 各种 PID 结构体    腿长/收腿/防劈叉/小陀螺/ROLL补偿/云台pitch 等
 */

#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"
#include "user_pid.h"

/** @brief 最小腿长 (m) */
extern float LEG_MIN_LENTH;
/** @brief 最大腿长 (m) */
extern float LEG_MAX_LENTH;
/** @brief 轮子半径 (m) */
#define WHEEL_RADIUS 0.061f

/**
 * @brief 一阶斜坡发生器。
 *
 * 用于平滑过渡目标值（如速度/腿长），避免阶跃。
 * 每周期调用 rampIterate() 更新当前值。
 */
typedef struct RampGenerator
{
    float currentValue; /**< 当前值 */
    float targetValue;  /**< 目标值 */
    float step;         /**< 每个控制周期应当改变的数值大小 */
    uint8_t isBusy;     /**< 指示斜坡发生器是否正在调整中 */
} RampGenerator;

/**
 * @brief 单腿信息。
 */
typedef struct Leg_Info
{
    float Current_L0;   /**< 腿当前长度 (m) */
} Leg_Info_t;

/**
 * @brief 底盘状态信息（下板侧）。
 */
typedef struct Foot_Chassis_Info
{
    float Yaw_Motor_Angle;  /**< Yaw 电机角度 (rad) */
    float Current_Speed;    /**< 底盘当前的速度 (m/s)，水平方向 */
    Leg_Info_t L_Leg;       /**< 左腿信息 */
    Leg_Info_t R_Leg;       /**< 右腿信息 */
} Foot_Chassis_Info_t;

/**
 * @brief 底盘目标与控制模式（上板→下板指令或内部控制）。
 */
typedef struct Foot_Chassis
{
    float Target_Vx;           /**< 云台坐标系下的目标 X 速度 (m/s) */
    float Target_Vy;           /**< 云台坐标系下的目标 Y 速度 (m/s) */
    uint8_t Target_Leg_State;  /**< 目标腿长：0=短腿，1=长腿 */
    uint8_t Chassis_Mode;      /**< 底盘模式：0=跟随，1=小陀螺，2=静止趴下 */
    Foot_Chassis_Info_t Info;  /**< 底盘实时信息 */
} Foot_Chassis_t;



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

