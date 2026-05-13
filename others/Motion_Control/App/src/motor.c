#include "motor.h"
#include "user_pid.h"
#include "Motor_Drv.h"
#include "Gimbal.h"
#include "User_State.h"
#include "State.h"
#include "arm_math.h"
#include "USER_CAN.h"
#include "VMC.h"
#include "observe_task.h"
#include "Leg_Control.h"
#include "Self_Righting.h"
#include "Board2Board.h"
#include "Slope.h"
#include "Wheel_Leg_about.h"
#include "controller.h"
#include "remoter.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "fdcan.h"
#include <math.h>
#include <stdint.h>
#include "imu_temp_ctrl.h"
#include "Angle_about.h"
#include "User_State.h"
#include "PowerCtrl.h"
#include "Gas_Spring.h"
#include "buzzer.h"

/*====================================== 附属函数变量 =========================================== */

Foot_Chassis_t Foot_Chassis;//轮足底盘结构体 
   
float powerPredict;
   
float LEG_MIN_LENTH = 0.20f;
float LEG_MAX_LENTH = 0.37f;  

float L_b_phi0, R_b_phi0;  
   
float PITCH_OFFSET=-0.05;  
   
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

float target_body_distance = 1.0f;
float body_distance_error;

float target_yaw, yaw_error;
//!屎作俑者：25年丛庆  数组0为当前pitch值，数组1为上一次的pitch值     单位为弧度
float yaw_trans[2];
float d_yaw;//陀螺仪yaw速度，单位为弧度每秒
float alpha_d_yaw = 0.8f;

float target_roll;
float alpha_target_roll = 0.05;

float Leg_F0_Limit = 500;

float mg = 80.0f/2;
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
-0.02568,  -1.94,  0.43125,  1.9772,  0.37466,  -0.44333,
     -0.20776,  -12.191,  2.8113,  12.519,  2.0535,  -2.7858,
     -11.539,  38.967,  -25.478,  -32.627,  -3.7211,  29.545,
     -2.2273,  9.1045,  -7.5696,  -5.0242,  -1.9464,  8.37,
     -3.6004,  -63.82,  14.423,  50.206,  15.655,  -17.466,
     -0.0076835,  -5.3988,  1.1678,  -1.0727,  3.2733,  -1.9164,
     -1.0899,  0.57269,  -25.809,  12.478,  -1.6094,  22.297,
     0.045556,  -1.0017,  -2.0937,  2.3683,  -1.2221,  0.33609,
     -20.813,  21.963,  34.032,  11.833,  -40.995,  -18.66,
     -2.3058,  1.2391,  4.6907,  2.3919,  -4.492,  -3.1709,
     -0.02568,  0.43125,  -1.94,  -0.44333,  0.37466,  1.9772,
     -0.20776,  2.8113,  -12.191,  -2.7858,  2.0535,  12.519,
     11.539,  25.478,  -38.967,  -29.545,  3.7211,  32.627,
     2.2273,  7.5696,  -9.1045,  -8.37,  1.9464,  5.0242,
     -1.0899,  -25.809,  0.57269,  22.297,  -1.6094,  12.478,
     0.045556,  -2.0937,  -1.0017,  0.33609,  -1.2221,  2.3683,
     -3.6004,  14.423,  -63.82,  -17.466,  15.655,  50.206,
     -0.0076835,  1.1678,  -5.3988,  -1.9164,  3.2733,  -1.0727,
     -20.813,  34.032,  21.963,  -18.66,  -40.995,  11.833,
     -2.3058,  4.6907,  1.2391,  -3.1709,  -4.492,  2.3919,
     2.7423,  -1.3451,  -5.8207,  -4.7316,  6.712,  4.0167,
     17.638,  -9.1893,  -37.983,  -29.904,  45.371,  25.382,
     -12.857,  -136.84,  -34.992,  224.88,  -128.29,  75.146,
     -1.9616,  -39.503,  -1.5664,  59.817,  -44.461,  9.4781,
     84.682,  -101.27,  1.6203,  0.5929,  149.08,  -35.708,
     2.7013,  12.973,  -3.8135,  -17.161,  13.862,  0.80561,
     11.388,  -112.95,  -7.3867,  136.56,  -153.29,  57.445,
     0.98348,  -5.2256,  4.7506,  5.5454,  -20.005,  -1.4394,
     -16.005,  -538.83,  102.22,  550.3,  157.26,  -156.06,
     1.993,  -39.57,  2.1276,  32.449,  20.593,  -6.6039,
     2.7423,  -5.8207,  -1.3451,  4.0167,  6.712,  -4.7316,
     17.638,  -37.983,  -9.1893,  25.382,  45.371,  -29.904,
     12.857,  34.992,  136.84,  -75.146,  128.29,  -224.88,
     1.9616,  1.5664,  39.503,  -9.4781,  44.461,  -59.817,
     11.388,  -7.3867,  -112.95,  57.445,  -153.29,  136.56,
     0.98348,  4.7506,  -5.2256,  -1.4394,  -20.005,  5.5454,
     84.682,  1.6203,  -101.27,  -35.708,  149.08,  0.5929,
     2.7013,  -3.8135,  12.973,  0.80561,  13.862,  -17.161,
     -16.005,  102.22,  -538.83,  -156.06,  157.26,  550.3,
     1.993,  2.1276,  -39.57,  -6.6039,  20.593,  32.449,
};

// PID控制器定义
user_pid_t L_Leg_L0_PID;     //常态
user_pid_t R_Leg_L0_PID;     //

user_pid_t L_Leg_L0_POS_PID; //收腿
user_pid_t R_Leg_L0_POS_PID; //
user_pid_t L_Leg_L0_SPD_PID; //
user_pid_t R_Leg_L0_SPD_PID; //

user_pid_t spinning_pid;//小陀螺PID
user_pid_t spinning_speed_pid;//小陀螺减速PID

user_pid_t Roll_Comp_PID;    //ROLL补偿pid

user_pid_t Leg_Phi0_PID;     //防劈叉pid

user_pid_t gimbal_pitch_pid;//云台俯仰pid



float target_Leg_L0 = 0.20f;//目标腿长, 初值与 LEG_MIN_LENTH 保持一致

float target_L_Leg_L0 = 0.20f;
float target_R_Leg_L0 = 0.20f;
uint8_t i;
int height_wait;
uint8_t temp1;
user_pid_t L_Spin_Phi0_PID, R_Spin_Phi0_PID;
float target_spin_phi0 = PI / 2.0f;

user_pid_t L_Leg_Middle_PID, R_Leg_Middle_PID;   //收腿角度pid
user_pid_t L_Leg_dphi0_PID, R_Leg_dphi0_PID;     //收腿角速度pid

uint8_t first_run = 1;//是否是第一次运行，第一次运行需要特殊处理一些变量的初始值

uint8_t upstares_mode = 0;//0为未开始上楼收腿，1为开始上楼收腿
int ready_count = 0;

uint8_t leg_state = 0;  //腿长状态 0为最短，1为中等，2为最长（未来会改）
int leg_state_count;

RampGenerator Target_Speed_Ramp;//目标速度斜坡发生器

/*============================= 任务变量 ================================= */

//?标志位
uint8_t gimbal_follow_flag = 1; // 1：刚站起来，云台跟随底盘 0：底盘跟随云台
uint8_t spinning_flag = 0; // 1：小陀螺运行中 0：小陀螺停止
uint8_t spinning_usable = 1; // 小陀螺是否可用，0为不可用，1为可用
uint8_t L_Leg_State, R_Leg_State;   //收腿阶段，0为收腿中，1为起立过程中收腿完成，2为上台阶过程中收腿完成

//?计数器
uint16_t L_Ready_Count, R_Ready_Count;
int L_off_ground = 0;   //必须是int类型，因为要减去计数器，不能无符号
int R_off_ground = 0;   //必须是int类型，因为要减去计数器，不能无符号


//?常量
uint16_t motor_HZ = 500; //任务频率

float wheel_track_R = 0.19242f; // 轮距半径，单位为米

//?调参
float target_spinning_d_yaw = 12.0f; // 目标小陀螺yaw速度，单位为弧度每秒
float centrifugal_comp_gain = 0.8f;  // spin离心补偿系数

//?中间参数
float down_board_yaw_output = 0.0f; // 下板yaw输出





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
    DM_Joint_Motor_Init(&L_DM8009[0], 40.0f, 3.14159265f, 45.0f, 0x01);
    DM_Joint_Motor_Init(&L_DM8009[1], 40.0f, 3.14159265f, 45.0f, 0x02);

    DM_Joint_Motor_Init(&R_DM8009[0], 40.0f, 3.14159265f, 45.0f, 0x01);
    DM_Joint_Motor_Init(&R_DM8009[1], 40.0f, 3.14159265f, 45.0f, 0x02);

    DM_Joint_Motor_Init(&Yaw_DM4310, 10.0f, 3.14159265f, 30.0f, 0x10);
    DM_Joint_Motor_Init(&Shooter_DM2325, 10.0f, 3.14159265f, 200.0f, 0x11);

    // 功率控制模块初始化（仅初始化参数，不改变现有控制流）。
    // PowerCtralInit(&whell_power);

    gas_spring_enable = 1;
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
    PID_INIT(&L_Leg_L0_PID, 3000, 0, 30000, 200, 0, 0, 0, 0);
    PID_INIT(&R_Leg_L0_PID, 3000, 0, 30000, 200, 0, 0, 0, 0);
    PID_INIT(&Leg_Phi0_PID, 300, 0, 5, 150, 150, 0, 50000, 0);
    PID_INIT(&L_Spin_Phi0_PID, 80, 0, 8, 40, 0, 0, 0, 0);
    PID_INIT(&R_Spin_Phi0_PID, 80, 0, 8, 40, 0, 0, 0, 0);
    PID_INIT(&Roll_Comp_PID, 10, 0.002, 100, 150, 80, 0, 10000, 0);

    PID_INIT(&L_Leg_Middle_PID, 15, 0.1, 0.1, 5.0, 4.0, 0, 0, 0);
    PID_INIT(&R_Leg_Middle_PID, 15, 0.1, 0.1, 5.0, 4.0, 0, 0, 0);
    PID_INIT(&L_Leg_dphi0_PID, 3, 0.1, 1, 150, 150, 0, 2000, 0);
    PID_INIT(&R_Leg_dphi0_PID, 3, 0.1, 1, 150,150, 0, 2000, 0);

    PID_INIT(&L_Leg_L0_POS_PID, 60, 0.001, 0.1, 3.0, 2.0, 0, 200, 0);
    PID_INIT(&R_Leg_L0_POS_PID, 60, 0.001, 0.1, 3.0, 2.0, 0, 200, 0);
    PID_INIT(&L_Leg_L0_SPD_PID, 200, 0.03, 50, 80, 80, 0, 2000, 0);
    PID_INIT(&R_Leg_L0_SPD_PID, 200, 0.03, 50, 80, 80, 0, 2000, 0);

    //小陀螺pid
    PID_INIT(&spinning_pid, 0.0005, 0.001f, 0.001, 6.0f, 6.0f, 0.005f, 20.0f, 0);

    //云台pid
    PID_INIT(&gimbal_pitch_pid, 10, 0.002, 100, 150, 80, 0, 10000, 0);
    PID_INIT(&spinning_speed_pid, -6, 0, 0, 6, 0, 0, 0, 0);
    
    // PID_INIT(&gimbal_follow_error_pid, 3, 0.002, 100, 150, 80, 10000, 0);

    // 目标腿长初值与 LEG_MIN_LENTH 同步
    target_Leg_L0   = LEG_MIN_LENTH;
    target_L_Leg_L0 = LEG_MIN_LENTH;
    target_R_Leg_L0 = LEG_MIN_LENTH;
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

/* ---------- 收腿起立 State=1 转角阶段：卡住反向绕长路 ---------- */

// FWD = 短路径 PID 追目标角；REV = 长路径恒速反绕一圈到同一目标角
typedef enum {
    STAIR_SUB_TURN_FWD = 0,  // 沿 ShortestAngleDelta 的短路径用 PID 串级控制
    STAIR_SUB_TURN_REV       // 沿反方向 (2π − 短路径) 匀速转到原目标角
} StairSub_t;

static StairSub_t L_stair_sub = STAIR_SUB_TURN_FWD;     // 左腿子状态
static StairSub_t R_stair_sub = STAIR_SUB_TURN_FWD;     // 右腿子状态
static int   L_sub_dwell = 0,          R_sub_dwell = 0;     // 进入当前子状态后的 tick 数，< dwell 门禁不允许再切换（防抖）
static float L_rev_dir = 0.0f,         R_rev_dir = 0.0f;    // REV 期间的转向 ±1，FWD→REV 时按"短路径反方向"锁定
static float L_rev_long_remain = 0.0f, R_rev_long_remain = 0.0f;  // REV 总需要走的弧长 = 2π − |短路径剩余|，FWD→REV 时锁定
static float L_rev_traveled = 0.0f,    R_rev_traveled = 0.0f;     // REV 期间对 d_phi0 积分得到的已走弧长，避免用 phi0 做减法在 ±π 处跳变

/**
 * @brief 收腿起立 State=1 转角阶段的控制 + 卡住反向绕长路。
 *
 * 子状态机（每腿各一套）：
 *   FWD 默认：Middle_PID(角度) → dphi0_PID(角速度) 串级，走短路径。
 *   FWD 卡住：|error|>0.1 且 |d_phi0|<0.0873rad/s 持续 100ms，且进入 FWD ≥ 200ms
 *            → 锁 rev_dir / rev_long_remain / rev_traveled=0，切 REV。
 *   REV 默认：leg_turn_speed_control(rev_dir * 1.2 rad/s, 15 N·m)，不受 ShortestAngle 限制。
 *   REV 到位：积分 rev_traveled ≥ long_remain − 0.05，或过半圈后短路径误差 < 0.05（兜底防积分漂移）。
 *   REV 又卡：切回 FWD 无限重试。
 *
 * is_right 用于保留右腿原有的 −d_phi0 / −output 符号约定（仅 FWD 分支需要；
 * REV 走 leg_turn_speed_control，速度目标自带符号，不再反置）。
 *
 * @param VMC              左右腿 VMC 结构指针
 * @param is_right         0=左腿, 1=右腿
 * @param target_angle     目标 phi0，单位 rad
 * @param pid_middle       角度 PID（外环）
 * @param pid_dphi0        角速度 PID（内环）
 * @param sub              子状态，持久变量地址
 * @param dwell            dwell 计数，持久变量地址
 * @param rev_dir          REV 转向 ±1，持久变量地址
 * @param rev_long_remain  REV 目标弧长，持久变量地址
 * @param rev_traveled     REV 已走弧长积分，持久变量地址
 * @param out_T            [out] 本周期该腿的转矩命令，直接喂给 VMC_Set_F0_T
 * @return 1 = 此周期视为到位（外层 Ready_Count 累加）, 0 = 未到位
 */
static int turn_ctrl_with_stuck_flip(
    VMC_t *VMC, int is_right, float target_angle,
    user_pid_t *pid_middle, user_pid_t *pid_dphi0,
    StairSub_t *sub, int *dwell,
    float *rev_dir, float *rev_long_remain, float *rev_traveled,
    float *out_T)
{
    (*dwell)++;
    int near = 0;                               // 本周期是否算到位

    if (*sub == STAIR_SUB_TURN_FWD)
    {
        // 短路径 PID 串级：Middle_PID 出 dphi0 目标，dphi0_PID 出转矩
        PID_Set_AngleError(pid_middle, VMC->phi0, target_angle);    // 内部走 ShortestAngleDelta
        PID_coculate(pid_middle);
        float dphi0_in = is_right ? -VMC->d_phi0        : VMC->d_phi0;          // 右腿符号约定
        float dphi0_tg = is_right ? -pid_middle->output : pid_middle->output;
        PID_Set_Error(pid_dphi0, dphi0_in, dphi0_tg);
        PID_coculate(pid_dphi0);
        *out_T = is_right ? -pid_dphi0->output : pid_dphi0->output;

        // 角度误差进入容差带 → 认为到位
        if (fabsf(pid_middle->error) <= 0.05f) near = 1;

        // 卡住判据：dwell 门禁过了 + 误差仍大 + d_phi0 死区持续时间达到 → 切 REV
        if (!near && *dwell > 100 &&
            fabsf(pid_middle->error) > 0.1f &&
            leg_turn_stuck_detect(VMC, 0.0873f, 0.1f))
        {
            float se = ShortestAngleDelta(target_angle, VMC->phi0);
            *rev_dir         = (se > 0.0f) ? -1.0f : 1.0f;      // 长路径 = 短路径反方向
            *rev_long_remain = 2.0f * PI - fabsf(se);           // 绕一圈减去短路径剩余
            *rev_traveled    = 0.0f;
            *sub = STAIR_SUB_TURN_REV;
            *dwell = 0;
            leg_turn_stuck_reset(VMC);      // 清共享计数器，避免 REV 一进来就重新触发
            *out_T = 0.0f;                  // 本周期不再下发 FWD 的残留转矩
        }
    }
    else  // STAIR_SUB_TURN_REV
    {
        // 纯速度控制走长路径，不受 ShortestAngle 限制
        *out_T = leg_turn_speed_control(VMC, (*rev_dir) * 1.2f, 15.0f, 0.0f);
        // 用 d_phi0 积分记已走弧长，避免 phi0 在 ±π 翻转导致的跳变
        *rev_traveled += (*rev_dir) * VMC->d_phi0 * 0.002f;     // dt = 1/500 Hz = 2 ms

        float se_now = fabsf(ShortestAngleDelta(target_angle, VMC->phi0));
        // 到位判定：主判用积分弧长，兜底用过半圈后短路径误差，防积分长期漂移
        int arrived = (*rev_traveled >= *rev_long_remain - 0.05f) ||
                      (*rev_traveled > PI && se_now < 0.05f);
        if (arrived)
        {
            near = 1;
            *sub = STAIR_SUB_TURN_FWD;      // 复位回 FWD，等外层计满 Ready_Count 后进入 State=2
            *dwell = 0;
            leg_turn_stuck_reset(VMC);
        }
        else if (*dwell > 100 &&
                 (*rev_traveled < *rev_long_remain - 0.1f) &&
                 leg_turn_stuck_detect(VMC, 0.0873f, 0.1f))
        {
            // 反向也卡住（且确认不是快到位的低速）→ 切回 FWD，无限重试
            *sub = STAIR_SUB_TURN_FWD;
            *dwell = 0;
            leg_turn_stuck_reset(VMC);
        }
    }
    return near;
}



//未站起 + 未上楼收腿  函数
void NotStanding_NotStairRetract_for_chassis()
{

    VMC_Coculate();
    Body_Speed_Coculate();//车身速度解算

    //是否姿态稳定在误差20°内的起立态
    if((roll >= 20.0f || roll <= -20.0f || pitch >= 20.0f || pitch <= -20.0f) && first_run == 1)//不稳定且是急停开始第一次运行
    {
        Self_Righting_Step();
        return ;
    }

    //倒地自起成功后复位Self_Righting_Step的状态机
    g_self_righting_stage = SELF_RIGHTING_STAGE_EXTEND;
    first_run = 0;//第一次运行完成

    //收腿过程腿长控制
    PID_Set_Error(&L_Leg_L0_POS_PID, VMC_L.L0, 0.12f);//0.19这个值是通过反复试验得来的，目的是让腿在收腿过程中稍微有个前倾，防止完全竖直时不稳定
    PID_Set_Error(&R_Leg_L0_POS_PID, VMC_R.L0, 0.12f);
    PID_coculate(&L_Leg_L0_POS_PID);
    PID_coculate(&R_Leg_L0_POS_PID);

    PID_Set_Error(&L_Leg_L0_SPD_PID, VMC_L.d_L0, L_Leg_L0_POS_PID.output);
    PID_Set_Error(&R_Leg_L0_SPD_PID, VMC_R.d_L0, R_Leg_L0_POS_PID.output);
    PID_coculate(&L_Leg_L0_SPD_PID);
    PID_coculate(&R_Leg_L0_SPD_PID);

    //腿角度控制（State>=1 启用，含卡住反向绕长路）
    float L_T = 0.0f, R_T = 0.0f;
    int L_near = 0, R_near = 0;
    if(L_Leg_State >= 1)
    {
        L_near = turn_ctrl_with_stuck_flip(
            &VMC_L, 0, PI/2.0f - 0.1f,
            &L_Leg_Middle_PID, &L_Leg_dphi0_PID,
            &L_stair_sub, &L_sub_dwell,
            &L_rev_dir, &L_rev_long_remain, &L_rev_traveled, &L_T);
    }
    if(R_Leg_State >= 1)
    {
        R_near = turn_ctrl_with_stuck_flip(
            &VMC_R, 1, PI/2.0f + 0.1f,
            &R_Leg_Middle_PID, &R_Leg_dphi0_PID,
            &R_stair_sub, &R_sub_dwell,
            &R_rev_dir, &R_rev_long_remain, &R_rev_traveled, &R_T);
    }

    //腿长判断是否到达目标长度
    if(L_Leg_State == 0 && fabsf(L_Leg_L0_POS_PID.error) <= 0.01)
    {
        L_Ready_Count ++;
    }
    if(L_Leg_State == 0 && L_Ready_Count >= 50)//腿到目标长度
    {
        L_Leg_State = 1;    //收腿完成
        L_Ready_Count = 0;  //归零
        L_stair_sub = STAIR_SUB_TURN_FWD;
        L_sub_dwell = 0;
        leg_turn_stuck_reset(&VMC_L);
    }
    if(R_Leg_State == 0 && fabsf(R_Leg_L0_POS_PID.error) <= 0.01)
    {
        R_Ready_Count ++;
    }
    if(R_Leg_State == 0 && R_Ready_Count >= 50)
    {
        R_Leg_State = 1;
        R_Ready_Count = 0;
        R_stair_sub = STAIR_SUB_TURN_FWD;
        R_sub_dwell = 0;
        leg_turn_stuck_reset(&VMC_R);
    }

    //腿长达标之后，判断腿角度是否到达目标角度（用 helper 返回的 near）
    if(L_Leg_State == 1 && L_near) L_Ready_Count ++;
    else if(L_Leg_State == 1)      L_Ready_Count = 0;
    if(L_Leg_State == 1 && L_Ready_Count >= 50)
    {
        L_Leg_State = 2;
        L_Ready_Count = 0;
    }
    if(R_Leg_State == 1 && R_near) R_Ready_Count ++;
    else if(R_Leg_State == 1)      R_Ready_Count = 0;
    if(R_Leg_State == 1 && R_Ready_Count >= 50)
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
        body_distance = 0;
        target_body_distance = -2.5;
    }

    //映射到电机力矩
    VMC_Set_F0_T(&VMC_L, L_Leg_L0_SPD_PID.output, L_T);
    VMC_Set_F0_T(&VMC_R, R_Leg_L0_SPD_PID.output, R_T);
    L_DJ3508.Target_Torque = 0;
    R_DJ3508.Target_Torque = 0;

}

float lqr_body_distance_error ;
float lqr_speed_error;
float lqr_yaw_error;
float lqr_d_yaw;
void LQR_calculate()
{
	lqr_body_distance_error = body_distance_error;
	lqr_speed_error = speed_error;
	lqr_yaw_error = yaw_error;
	lqr_d_yaw = d_yaw;
    float leg_yaw_error = lqr_yaw_error;
    float leg_d_yaw = lqr_d_yaw;
    if(spinning_flag == 1)
    {
        leg_yaw_error = 0.0f;
        leg_d_yaw = 0.0f;
    }
    float leg_b_phi0_offset = (spinning_flag == 1) ? 0.0f : b_phi0_offset;

    //算轮子力矩
    L_DJ3508.Target_Torque = 
    + LQR_K[0][0] * lqr_body_distance_error
    + LQR_K[0][1] * (lqr_speed_error)
    + LQR_K[0][2] * (lqr_yaw_error)
    - LQR_K[0][3] * lqr_d_yaw
    - LQR_K[0][4] * VMC_L.b_phi0 
    - LQR_K[0][5] * VMC_L.d_b_phi0 
    - LQR_K[0][6] * VMC_R.b_phi0 
    - LQR_K[0][7] * VMC_R.d_b_phi0 
    + LQR_K[0][8] * (pitch_trans[0] - PITCH_OFFSET)
    + LQR_K[0][9] * d_pitch;

    R_DJ3508.Target_Torque = 
    + LQR_K[1][0] * lqr_body_distance_error
    + LQR_K[1][1] * (lqr_speed_error) 
    + LQR_K[1][2] * (lqr_yaw_error)
    - LQR_K[1][3] * lqr_d_yaw
    - LQR_K[1][4] * VMC_L.b_phi0 
    - LQR_K[1][5] * VMC_L.d_b_phi0 
    - LQR_K[1][6] * VMC_R.b_phi0 
    - LQR_K[1][7] * VMC_R.d_b_phi0 
    + LQR_K[1][8] * (pitch_trans[0] - PITCH_OFFSET)
    + LQR_K[1][9] * d_pitch;

    //算模拟腿力矩
    Leg_L_T = 
    + LQR_K[2][0] * lqr_body_distance_error
    + LQR_K[2][1] * (lqr_speed_error)
    + LQR_K[2][2] * (-leg_yaw_error)
    - LQR_K[2][3] * leg_d_yaw
    - LQR_K[2][4] * (VMC_L.b_phi0 - leg_b_phi0_offset)
    - LQR_K[2][5] * VMC_L.d_b_phi0 
    - LQR_K[2][6] * VMC_R.b_phi0 
    - LQR_K[2][7] * VMC_R.d_b_phi0
    + LQR_K[2][8] * (pitch_trans[0] - PITCH_OFFSET)
    + LQR_K[2][9] * d_pitch;

    Leg_R_T = 
    + LQR_K[3][0] * lqr_body_distance_error
    + LQR_K[3][1] * (lqr_speed_error)
    + LQR_K[3][2] * (-leg_yaw_error)
    - LQR_K[3][3] * leg_d_yaw
    - LQR_K[3][4] * VMC_L.b_phi0 
    - LQR_K[3][5] * VMC_L.d_b_phi0 
    - LQR_K[3][6] * (VMC_R.b_phi0 - leg_b_phi0_offset)
    - LQR_K[3][7] * VMC_R.d_b_phi0
    + LQR_K[3][8] * (pitch_trans[0] - PITCH_OFFSET)
    + LQR_K[3][9] * d_pitch;
}

void off_ground_detect()
{
    float alpha_G_F0 = 0.1;

    L_Ground_F0 = alpha_G_F0 * VMC_Get_Ground_F0(&VMC_L) + (1 - alpha_G_F0) * L_Ground_F0;
    R_Ground_F0 = alpha_G_F0 * VMC_Get_Ground_F0(&VMC_R) + (1 - alpha_G_F0) * R_Ground_F0;

    //离地检测滤波
    if(L_Ground_F0 <= 50.0f)
    L_off_ground ++;
    else if (L_Ground_F0 >= 20.0f)
    L_off_ground --;
    if(L_off_ground >= 50)
    L_off_ground = 50;
    if(L_off_ground <= 0)
    L_off_ground = 0;
    
    if(R_Ground_F0 <= 50.0f)
    R_off_ground ++;
    else if (R_Ground_F0 >= 20.0f)
    R_off_ground --;
    if(R_off_ground >= 50)
    R_off_ground = 50;
    if(R_off_ground <= 0)
    R_off_ground = 0;

    //!这段是先算一遍不离地的情况的数，再检测是否离地，如果离地，就再算一次覆盖掉
    if(L_off_ground >= 20)//正常行驶过程离地
    {
        //离地后腿归中，轮子脱力vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F%E7%A6%BB%E5%9C%B0%E5%90%8E%E8%85%BF%E5%BD%92%E4%B8%AD%EF%BC%8C%E8%BD%AE%E5%AD%90%E8%84%B1%E5%8A%9B
        Leg_L_T = 
        - LQR_K[2][4] * (VMC_L.b_phi0 + 0.3) 
        - LQR_K[2][5] * VMC_L.d_b_phi0 ;
        Leg_L_T *= 0.5; //收腿力度参数
        L_DJ3508.Target_Torque = 0;//离地轮子脱力
        //正常行驶过程离地VMC解算
        VMC_Set_F0_T(&VMC_L, L_Leg_L0_PID.output * 0.7, Leg_L_T);//VMC解算
        //离地距离相关量归零
        body_distance = 0;
        target_body_distance = 0.0;
    }
    if(R_off_ground >= 20)
    {
        Leg_R_T = 
        - LQR_K[3][6] * (VMC_R.b_phi0 + 0.3)
        - LQR_K[3][7] * VMC_R.d_b_phi0;
        Leg_R_T *= 0.5;
        R_DJ3508.Target_Torque = 0;
        VMC_Set_F0_T(&VMC_R, R_Leg_L0_PID.output * 0.7, -Leg_R_T);
        body_distance = 0;
        target_body_distance = 0.0;
    }
}

//speed_error, yaw_error | 算yaw的误差，以及根据yaw误差调整target_body_speed进而调整speed_error()
void Yaw_Error_Coculate()
{
    float Yaw_motor_position;
    Yaw_motor_position = Yaw_DM4310.Rx_Data.Position - (head_forward_angle);//减的是零点
    
    //套圈处理
    if(Yaw_motor_position > PI)
    {
        Yaw_motor_position -= 2 * PI;
    }
    if(Yaw_motor_position < -PI)
    {
        Yaw_motor_position += 2 * PI;
    }
    yaw_error = Yaw_motor_position;
    // alpha_yaw_error = Yaw_motor_position * Yaw_motor_position * 0.05f;//Yaw_motor_position越大，alpha越大，响应越慢，最大为0.5
    // raw_yaw_error = Yaw_motor_position;

    
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

    
        // yaw_error = easy_Slope(raw_yaw_error, yaw_error, yaw_error_step);

}

//小陀螺加速
void spinning_up()
{
    float spinning_setpoint = (g_filtered_power > power_limit)
                            ? target_spinning_d_yaw * g_power_obs_lambda
                            : target_spinning_d_yaw;
    PID_Set_Error(&spinning_pid, d_yaw, spinning_setpoint);
    yaw_error = PID_coculate(&spinning_pid);
    Speed_Error_Set();
}

//小陀螺减速
void spinning_down()
{
    PID_Set_Error(&spinning_pid, d_yaw, 0);
    yaw_error = PID_coculate(&spinning_pid);
    Speed_Error_Set();
}

//小陀螺急停
void spinning_stop()
{
    PID_Set_Error(&spinning_speed_pid, yaw_angle_PI, 0);
    float spinning_speed_output = PID_coculate(&spinning_speed_pid);
    PID_Set_Error(&spinning_pid, d_yaw, spinning_speed_output);
    yaw_error = PID_coculate(&spinning_pid);
    Speed_Error_Set();
}

//站起
void Standing()
{
//占用率检测用的，留着吧，看不懂也不影响
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, 1);

    //惯性导航、VMC、水平方向车身速度解算vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F%E6%83%AF%E6%80%A7%E5%AF%BC%E8%88%AA%E3%80%81VMC%E3%80%81%E6%B0%B4%E5%B9%B3%E6%96%B9%E5%90%91%E8%BD%A6%E8%BA%AB%E9%80%9F%E5%BA%A6%E8%A7%A3%E7%AE%97
    INS_Coculate();
    VMC_Coculate();
    Body_Speed_Coculate();

    //算yaw的误差，以及根据yaw误差调整目标速度
    if(gimbal_follow_flag == 1)
    {
        yaw_error = 0;//云台跟随底盘时，强制yaw误差为0，让底盘完全跟随云台
        Speed_Error_Set();
    }
    
    if(gimbal_follow_flag == 0)
    {
        //算小陀螺的
        if(Foot_Chassis.Chassis_Mode == 1 && spinning_usable == 1)
        {
            spinning_up();
            spinning_flag = 1;
        }
        else//普通运行
        {
            if(spinning_flag == 1)//小陀螺减速
            {
                spinning_usable = 0;

                spinning_down();

                if((fabsf(d_yaw) <= 10.0f && yaw_angle_PI >= 0) || (fabsf(d_yaw) <= 3.0f))
                {
                    spinning_flag = 2;
                }
            }
            else if(spinning_flag == 2)//双环减速，目标头方向
            {
                //小陀螺急停
                spinning_stop();

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
    if (spinning_flag == 1) { body_distance_error = 0; body_distance = 0; target_body_distance = 0; }

    // spin车身速度积分，补偿稳态漂移
    static float spin_speed_I = 0;
    if (spinning_flag == 1) {
        spin_speed_I += kalman_body_speed * 0.01f;
        if (spin_speed_I >  0.5f) spin_speed_I =  0.5f;
        if (spin_speed_I < -0.5f) spin_speed_I = -0.5f;
        speed_error -= spin_speed_I;
        //轮速共模P控制，快锁两轮转速相等
        float wheel_common = (R_DJ3508.Rx_Data.Velocity - L_DJ3508.Rx_Data.Velocity) * 0.0305f;
        speed_error -= wheel_common * 0.5f;
    } else {
        spin_speed_I = 0;
    }

    //横滚补偿和PD单环腿长控制
    Roll_Comp();
    Leg_L0_Control();

    //放劈叉
    PID_Set_Error(&Leg_Phi0_PID, (VMC_R.phi0 - PI/2) + (VMC_L.phi0 - PI/2), 0);
    PID_coculate(&Leg_Phi0_PID);

    //100hz算K值，毕竟K值的计算比较耗时
    i++;
    if(i >= 5)
    {
        i = 0;
        LQR_Get_K(LQR_K, K_Fit_Coefficients, VMC_L.L0, VMC_R.L0);
    }

	// 更新功率门控缩放系数（不直接限制扭矩，只作用于观测量）。
	PowerCtrl();

    LQR_calculate();

    //常态下VMC解算，加入PID前馈
    float centrifugal_comp = centrifugal_comp_gain * d_yaw * d_yaw;
    float L_leg_T_cmd = Leg_L_T + Leg_Phi0_PID.output - centrifugal_comp;
    float R_leg_T_cmd = -Leg_R_T + Leg_Phi0_PID.output - centrifugal_comp;
    static uint8_t spin_phi0_pid_started = 0;
    if(spinning_flag == 1)
    {
        PID_Set_AngleError(&L_Spin_Phi0_PID, VMC_L.phi0, target_spin_phi0);
        PID_Set_AngleError(&R_Spin_Phi0_PID, VMC_R.phi0, target_spin_phi0);
        if(spin_phi0_pid_started == 0)
        {
            L_Spin_Phi0_PID.pre_error = L_Spin_Phi0_PID.error;
            R_Spin_Phi0_PID.pre_error = R_Spin_Phi0_PID.error;
            spin_phi0_pid_started = 1;
        }
        L_leg_T_cmd += PID_coculate(&L_Spin_Phi0_PID);
        R_leg_T_cmd += PID_coculate(&R_Spin_Phi0_PID);
    }
    else
    {
        PID_Clear(&L_Spin_Phi0_PID);
        PID_Clear(&R_Spin_Phi0_PID);
        L_Spin_Phi0_PID.output = 0.0f;
        R_Spin_Phi0_PID.output = 0.0f;
        spin_phi0_pid_started = 0;
    }
    VMC_Set_F0_T(&VMC_L, L_Leg_L0_PID.output + (mg / arm_cos_f32(VMC_L.b_phi0)) + Roll_Comp_PID.output,
                 L_leg_T_cmd);
    VMC_Set_F0_T(&VMC_R, R_Leg_L0_PID.output + (mg / arm_cos_f32(VMC_R.b_phi0)) - Roll_Comp_PID.output,
                 R_leg_T_cmd);

    off_ground_detect();

    if(upstairs_flag == 1)
    {
        start_mode = 2;
    }

    if(sit_mode_enable == 1)
    {
        start_mode = 3;
    }

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, 0);
}

void Upstair_NotStairRetract()
{
    VMC_Coculate();
    Body_Speed_Coculate();

    //上台阶过程中轮子正转，防止滑下来
    L_DJ3508.Target_Torque = 0.1;
    R_DJ3508.Target_Torque = 0.1;

    // 磕台阶过程中双环腿长控制
    PID_Set_Error(&L_Leg_L0_POS_PID, VMC_L.L0, LEG_MAX_LENTH);   //TODO: 写一个最大腿长的宏定义
    PID_Set_Error(&R_Leg_L0_POS_PID, VMC_R.L0, LEG_MAX_LENTH);
    PID_coculate(&L_Leg_L0_POS_PID);
    PID_coculate(&R_Leg_L0_POS_PID);

    PID_Set_Error(&L_Leg_L0_SPD_PID, VMC_L.d_L0, L_Leg_L0_POS_PID.output);
    PID_Set_Error(&R_Leg_L0_SPD_PID, VMC_R.d_L0, R_Leg_L0_POS_PID.output);
    PID_coculate(&L_Leg_L0_SPD_PID);   
    PID_coculate(&R_Leg_L0_SPD_PID);

    //磕台阶过程中双环腿角度控制
    PID_Set_AngleError(&L_Leg_Middle_PID, VMC_L.phi0, PI/2 + 1.5f);
    PID_coculate(&L_Leg_Middle_PID);
    PID_Set_Error(&L_Leg_dphi0_PID, VMC_L.d_phi0, L_Leg_Middle_PID.output);
    PID_coculate(&L_Leg_dphi0_PID);

    PID_Set_AngleError(&R_Leg_Middle_PID, VMC_R.phi0, PI/2 - 1.5f);
    PID_coculate(&R_Leg_Middle_PID);
    PID_Set_Error(&R_Leg_dphi0_PID, -VMC_R.d_phi0, -R_Leg_Middle_PID.output);
    PID_coculate(&R_Leg_dphi0_PID);

    //上台阶过程中VMC解算vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F%E4%B8%8A%E5%8F%B0%E9%98%B6%E8%BF%87%E7%A8%8B%E4%B8%ADVMC%E8%A7%A3%E7%AE%97
    VMC_Set_F0_T(&VMC_L, L_Leg_L0_SPD_PID.output, L_Leg_dphi0_PID.output);
    VMC_Set_F0_T(&VMC_R, R_Leg_L0_SPD_PID.output, -R_Leg_dphi0_PID.output);

    //上台阶收腿过程中判断腿长和腿角度是否都到位了
    if(L_Leg_State == 0 && fabsf(L_Leg_L0_POS_PID.error) <= 0.1 && fabsf(L_Leg_Middle_PID.error) <= 0.1)
    {
        L_Ready_Count ++;
    }
    if(L_Leg_State == 0 && L_Ready_Count >= 120)
    {
        L_Leg_State = 2;
        L_Ready_Count = 0;
    }
    if(R_Leg_State == 0 && fabsf(R_Leg_L0_POS_PID.error) <= 0.1 && fabsf(R_Leg_Middle_PID.error) <= 0.1)
    {
        R_Ready_Count ++;
    }
    if(R_Leg_State == 0 && R_Ready_Count >= 120)
    {
        R_Leg_State = 2;
        R_Ready_Count = 0;
    }
    if(R_Leg_State == 2 && L_Leg_State == 2)
    {
        upstares_mode = 1;
        R_Leg_State = 0;
        L_Leg_State = 0;
        //初始化 StairRetract 的子状态机，避免 Self_Righting 或上一次残留计数
        L_stair_sub = STAIR_SUB_TURN_FWD;
        R_stair_sub = STAIR_SUB_TURN_FWD;
        L_sub_dwell = 0;
        R_sub_dwell = 0;
        leg_turn_stuck_reset(&VMC_L);
        leg_turn_stuck_reset(&VMC_R);
    }
}

void StairRetract()
{
    VMC_Coculate();
    Body_Speed_Coculate();

    //收腿起立的腿长双环控制（State=0/1 都跑）
    PID_Set_Error(&L_Leg_L0_POS_PID, VMC_L.L0, 0.12f);
    PID_Set_Error(&R_Leg_L0_POS_PID, VMC_R.L0, 0.12f);
    PID_coculate(&L_Leg_L0_POS_PID);
    PID_coculate(&R_Leg_L0_POS_PID);

    PID_Set_Error(&L_Leg_L0_SPD_PID, VMC_L.d_L0, L_Leg_L0_POS_PID.output);
    PID_Set_Error(&R_Leg_L0_SPD_PID, VMC_R.d_L0, R_Leg_L0_POS_PID.output);
    PID_coculate(&L_Leg_L0_SPD_PID);
    PID_coculate(&R_Leg_L0_SPD_PID);

    //腿角度控制（State>=1 启用，含卡住反向绕长路）
    float L_T = 0.0f, R_T = 0.0f;
    int L_near = 0, R_near = 0;
    if(L_Leg_State >= 1)
    {
        L_near = turn_ctrl_with_stuck_flip(
            &VMC_L, 0, PI/2.0f - 0.1f,
            &L_Leg_Middle_PID, &L_Leg_dphi0_PID,
            &L_stair_sub, &L_sub_dwell,
            &L_rev_dir, &L_rev_long_remain, &L_rev_traveled, &L_T);
    }
    if(R_Leg_State >= 1)
    {
        R_near = turn_ctrl_with_stuck_flip(
            &VMC_R, 1, PI/2.0f + 0.1f,
            &R_Leg_Middle_PID, &R_Leg_dphi0_PID,
            &R_stair_sub, &R_sub_dwell,
            &R_rev_dir, &R_rev_long_remain, &R_rev_traveled, &R_T);
    }

    //映射到电机力矩
    VMC_Set_F0_T(&VMC_L, L_Leg_L0_SPD_PID.output, L_T);
    VMC_Set_F0_T(&VMC_R, R_Leg_L0_SPD_PID.output, R_T);

    //轮力矩：运行中 0.5，State=2 完成后 0
    L_DJ3508.Target_Torque = 0.0f;
    R_DJ3508.Target_Torque = 0.0f;

    //State=0 → 1（腿长到位后进入转角阶段）
    if(L_Leg_State == 0 && fabsf(L_Leg_L0_POS_PID.error) <= 0.02f) L_Ready_Count ++;
    else if(L_Leg_State == 0) L_Ready_Count = 0;
    if(L_Leg_State == 0 && L_Ready_Count >= 50)
    {
        L_Leg_State = 1;
        L_Ready_Count = 0;
        L_stair_sub = STAIR_SUB_TURN_FWD;
        L_sub_dwell = 0;
        leg_turn_stuck_reset(&VMC_L);
    }
    if(R_Leg_State == 0 && fabsf(R_Leg_L0_POS_PID.error) <= 0.02f) R_Ready_Count ++;
    else if(R_Leg_State == 0) R_Ready_Count = 0;
    if(R_Leg_State == 0 && R_Ready_Count >= 50)
    {
        R_Leg_State = 1;
        R_Ready_Count = 0;
        R_stair_sub = STAIR_SUB_TURN_FWD;
        R_sub_dwell = 0;
        leg_turn_stuck_reset(&VMC_R);
    }

    //State=1 → 2（角度到位，由 helper 返回 near 判定）
    if(L_Leg_State == 1 && L_near) L_Ready_Count ++;
    else if(L_Leg_State == 1)      L_Ready_Count = 0;
    if(L_Leg_State == 1 && L_Ready_Count >= 50)
    {
        L_Leg_State = 2;
        L_Ready_Count = 0;
    }
    if(R_Leg_State == 1 && R_near) R_Ready_Count ++;
    else if(R_Leg_State == 1)      R_Ready_Count = 0;
    if(R_Leg_State == 1 && R_Ready_Count >= 50)
    {
        R_Leg_State = 2;
        R_Ready_Count = 0;
    }

    //状态量归位（两腿都到位）
    if(R_Leg_State == 2 && L_Leg_State == 2)
    {
        upstares_mode = 0;
        start_mode = 0;
        R_Leg_State = 0;   //下次进入从 State=0 开始
        L_Leg_State = 0;
        leg_state = 0;
        target_Leg_L0 = LEG_MIN_LENTH;
    }
}

/*====================================== 腿坐标系相关结构体 ========================================*/

typedef struct
{
    Cartesian_Def current;
    Cartesian_Def error;
    Cartesian_Def target;
} Leg_Cartesian_Def;

typedef struct
{
    Polar_Def current;
    Polar_Def error;
    Polar_Def target;
} Leg_Polar_Def;

Leg_Cartesian_Def Cartesian_left_leg;
Leg_Cartesian_Def Cartesian_right_leg;

Leg_Polar_Def Polar_left_leg;
Leg_Polar_Def Polar_right_leg;

typedef struct
{
    user_pid_t L0_speed;
    user_pid_t L0_distance;
    user_pid_t phi0_speed;
    user_pid_t phi0_angle;
} Leg_user_pid_t;

Leg_user_pid_t left_Leg_PID;
Leg_user_pid_t right_Leg_PID;

float left_square_error;
float right_square_error;

float left_mean_square_error;
float right_mean_square_error;

uint8_t count;

float final_left_F0;
float final_left_tao;
float final_right_F0;
float final_right_tao;

void Gravity_Compensation_Test_Function(void)
{
    VMC_Coculate();
    Polar_Get(&Polar_left_leg.current, VMC_L.phi0, VMC_L.L0);
    Polar_Get(&Polar_right_leg.current, VMC_R.phi0, VMC_R.L0);

    //计算方差
    left_square_error += Polar_left_leg.error.r * Polar_left_leg.error.r + Polar_left_leg.error.angle * Polar_left_leg.error.angle;
    right_square_error += Polar_right_leg.error.r * Polar_right_leg.error.r + Polar_right_leg.error.angle * Polar_right_leg.error.angle;

    left_mean_square_error = left_square_error / count;
    right_mean_square_error = right_square_error / count;

    //极坐标误差计算
    Polar_left_leg.error.angle = Polar_left_leg.target.angle - Polar_left_leg.current.angle;
    Polar_left_leg.error.r = Polar_left_leg.target.r - Polar_left_leg.current.r;
    Polar_right_leg.error.angle = Polar_right_leg.target.angle - Polar_right_leg.current.angle;
    Polar_right_leg.error.r = Polar_right_leg.target.r - Polar_right_leg.current.r;

    //pid双环
    PID_Set_Error(&left_Leg_PID.L0_distance, Polar_right_leg.current.r, Polar_right_leg.target.r);
    PID_coculate(&left_Leg_PID.L0_distance);
    PID_Set_Error(&left_Leg_PID.L0_speed, Polar_right_leg.current.d_r, left_Leg_PID.L0_distance.output);
    PID_coculate(&left_Leg_PID.L0_speed);

    PID_Set_Error(&left_Leg_PID.phi0_angle, Polar_left_leg.current.angle, Polar_left_leg.target.angle);
    PID_coculate(&left_Leg_PID.phi0_angle);
    PID_Set_Error(&left_Leg_PID.phi0_speed, Polar_left_leg.current.d_angle, left_Leg_PID.phi0_angle.output);
    PID_coculate(&left_Leg_PID.phi0_speed);

    PID_Set_Error(&right_Leg_PID.L0_distance, Polar_right_leg.current.r, Polar_right_leg.target.r);
    PID_coculate(&right_Leg_PID.L0_distance);
    PID_Set_Error(&right_Leg_PID.L0_speed, Polar_right_leg.current.d_r, right_Leg_PID.L0_distance.output);
    PID_coculate(&right_Leg_PID.L0_speed);

    PID_Set_Error(&right_Leg_PID.phi0_angle, Polar_left_leg.current.angle, Polar_left_leg.target.angle);
    PID_coculate(&right_Leg_PID.phi0_angle);
    PID_Set_Error(&right_Leg_PID.phi0_speed, Polar_left_leg.current.d_angle, right_Leg_PID.phi0_angle.output);
    PID_coculate(&right_Leg_PID.phi0_speed);

    VMC_Set_F0_T(&VMC_L, left_Leg_PID.L0_speed.output, left_Leg_PID.phi0_speed.output);
    VMC_Set_F0_T(&VMC_R, right_Leg_PID.L0_speed.output, right_Leg_PID.phi0_speed.output);
    
    if(count == 255)
    {
        if(left_mean_square_error <= 0.01f)
        {
            //重力补偿参数测试通过
            final_left_F0 = left_Leg_PID.L0_speed.output;
            final_left_tao = left_Leg_PID.phi0_speed.output;
        }
        if(right_mean_square_error <= 0.01f)
        {
            //重力补偿参数测试通过
            final_right_F0 = right_Leg_PID.L0_speed.output;
            final_right_tao = right_Leg_PID.phi0_speed.output;
        }

        count = 0;
        left_square_error = 0;
        right_square_error = 0;
    }

    count ++;
}


/*====================================== 坐地模式 =========================================== */

#define SIT_TARGET_L0_L      0.1532f
#define SIT_TARGET_L0_R      0.1555f
#define SIT_TARGET_PHI0_L    0.7353f
#define SIT_TARGET_PHI0_R    2.3164f
#define SIT_RAMP_TIME        0.8f
#define SIT_WHEEL_TORQUE     0.0f
#define SIT_GRAVITY_RATIO    0.3f

static uint8_t sit_first_entry = 1;
static RampGenerator sit_L0_ramp_L, sit_L0_ramp_R;
static RampGenerator sit_phi0_ramp_L, sit_phi0_ramp_R;

uint16_t sit_debug_counter = 0;  // 坐地模式执行周期计数，debug用
uint8_t sit_ramp_done = 0;       // 斜坡过渡是否完成

void Sit_On_Ground(void)
{
    VMC_Coculate();

    if (sit_first_entry)
    {
        rampInit(&sit_L0_ramp_L, VMC_L.L0, SIT_TARGET_L0_L, SIT_RAMP_TIME, 0.002f);
        rampInit(&sit_L0_ramp_R, VMC_R.L0, SIT_TARGET_L0_R, SIT_RAMP_TIME, 0.002f);
        rampInit(&sit_phi0_ramp_L, VMC_L.phi0, SIT_TARGET_PHI0_L, SIT_RAMP_TIME, 0.002f);
        rampInit(&sit_phi0_ramp_R, VMC_R.phi0, SIT_TARGET_PHI0_R, SIT_RAMP_TIME, 0.002f);
        sit_first_entry = 0;
    }

    sit_debug_counter++;

    rampIterate(&sit_L0_ramp_L);
    rampIterate(&sit_L0_ramp_R);
    rampIterate(&sit_phi0_ramp_L);
    rampIterate(&sit_phi0_ramp_R);

    sit_ramp_done = (fabsf(sit_L0_ramp_L.currentValue - SIT_TARGET_L0_L) < 0.001f)
                 && (fabsf(sit_L0_ramp_R.currentValue - SIT_TARGET_L0_R) < 0.001f)
                 && (fabsf(sit_phi0_ramp_L.currentValue - SIT_TARGET_PHI0_L) < 0.001f)
                 && (fabsf(sit_phi0_ramp_R.currentValue - SIT_TARGET_PHI0_R) < 0.001f);

    // 左腿腿长双环PID
    PID_Set_Error(&L_Leg_L0_POS_PID, VMC_L.L0, sit_L0_ramp_L.currentValue);
    PID_coculate(&L_Leg_L0_POS_PID);
    PID_Set_Error(&L_Leg_L0_SPD_PID, VMC_L.d_L0, L_Leg_L0_POS_PID.output);
    PID_coculate(&L_Leg_L0_SPD_PID);

    // 右腿腿长双环PID
    PID_Set_Error(&R_Leg_L0_POS_PID, VMC_R.L0, sit_L0_ramp_R.currentValue);
    PID_coculate(&R_Leg_L0_POS_PID);
    PID_Set_Error(&R_Leg_L0_SPD_PID, VMC_R.d_L0, R_Leg_L0_POS_PID.output);
    PID_coculate(&R_Leg_L0_SPD_PID);

    // 左腿角度双环PID
    PID_Set_AngleError(&L_Leg_Middle_PID, VMC_L.phi0, sit_phi0_ramp_L.currentValue);
    PID_coculate(&L_Leg_Middle_PID);
    PID_Set_Error(&L_Leg_dphi0_PID, VMC_L.d_phi0, L_Leg_Middle_PID.output);
    PID_coculate(&L_Leg_dphi0_PID);

    // 右腿角度双环PID
    PID_Set_AngleError(&R_Leg_Middle_PID, VMC_R.phi0, sit_phi0_ramp_R.currentValue);
    PID_coculate(&R_Leg_Middle_PID);
    PID_Set_Error(&R_Leg_dphi0_PID, -VMC_R.d_phi0, -R_Leg_Middle_PID.output);
    PID_coculate(&R_Leg_dphi0_PID);

    // VMC映射到电机力矩
    VMC_Set_F0_T(&VMC_L, L_Leg_L0_SPD_PID.output + (mg / arm_cos_f32(VMC_L.b_phi0)) * SIT_GRAVITY_RATIO, L_Leg_dphi0_PID.output);
    VMC_Set_F0_T(&VMC_R, R_Leg_L0_SPD_PID.output + (mg / arm_cos_f32(VMC_R.b_phi0)) * SIT_GRAVITY_RATIO, -R_Leg_dphi0_PID.output);

    // 轮子小力矩锁死
    L_DJ3508.Target_Torque = SIT_WHEEL_TORQUE;
    R_DJ3508.Target_Torque = -SIT_WHEEL_TORQUE;

    // 检测退出
    if (!sit_mode_enable)
    {
        start_mode = 0;
        first_run = 1;
        sit_first_entry = 1;
    }
}

/*****************************************************************************************************
 *                                                                                                   *
 *                                                                                                   *
 *                                            控制任务                                                *
 *                                                                                                   *
 *                                                                                                   *
 *****************************************************************************************************/

float user_gas = 0;

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
        if(user_Gravity_Compensation_Test_Function_set == 0)
        {
            user_gas = Gas_Spring_GetForce(VMC_L.L0);

            //刚启动收腿过程中
            if(start_mode == 0 && upstares_mode == 0)//未站起 + 未上楼收腿
            {
                NotStanding_NotStairRetract_for_chassis();
                gimbal_follow_flag = 1;
            }

            else if(start_mode == 1)//站起
            {
                Standing();
            }
            else if(start_mode == 2 && upstares_mode == 0)//上楼梯模式 + 未上楼收腿
            {
                Upstair_NotStairRetract();
            }
            else if(start_mode == 3)//坐地模式
            {
                Sit_On_Ground();
            }
            else if(upstares_mode == 1)//收腿起立
            {
                StairRetract();
            }
        }
        if(user_Gravity_Compensation_Test_Function_set == 1)
        {
            Gravity_Compensation_Test_Function();
        }

        osDelayUntil(&xLastWakeTime, 2);//精确延时2毫秒，同时更新xLastWakeTime的值为当前时间
    }

    
}


/**
 * 行为树（未启用）
 */
// void Action_Tree(void)
// {
//     起立
//         IF(处于倒地)
//         {
//             倒地自起动作组 
//             return
//         }
//         后伸腿动作组

//     站起
//         小陀螺动作组
//         正常开
//         IF(侧翻检测)
//         {
//             底盘脱力动作组
//             IF(持续侧翻)
//             {
//                 返回起立态
//                 return
//             }
//         }

//     上台阶
//         上台阶动作组

//     下台阶
//         下台阶动作组
// }


// 摆头
