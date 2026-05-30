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
   
float LEG_MIN_LENTH = 0.23f;
float LEG_MAX_LENTH = 0.39f;

float L_b_phi0, R_b_phi0;
   
float PITCH_OFFSET = 0.00;


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

float target_body_distance = 0.0f;
float body_distance_error;

float target_yaw, yaw_error;
//!屎作俑者：25年丛庆  数组0为当前pitch值，数组1为上一次的pitch值     单位为弧度
float yaw_trans[2];
float d_yaw;//陀螺仪yaw速度，单位为弧度每秒
float alpha_d_yaw = 0.8f;

float target_roll;
float alpha_target_roll = 0.05;

float Leg_F0_Limit = 500;

float mg = 60.0f/2;
float L_Ground_F0, R_Ground_F0; //地面支持力
float prev_L_Ground_F0 = 0.0f;
float prev_R_Ground_F0 = 0.0f;

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
0.025121,  -9.2416,  2.4065,  9.5832,  1.0396,  -2.403,
     -0.10134,  -20.704,  5.6628,  21.484,  1.5793,  -5.4275,
     -11.551,  42.46,  -20.392,  -38.027,  -0.55309,  23.617,
     -2.2884,  10.232,  -7.0008,  -5.7981,  -1.2154,  7.9753,
     -4.5954,  -67.743,  10.589,  48.606,  16.608,  -15.024,
     0.031271,  -6.338,  1.1749,  -2.2384,  3.5566,  -2.0846,
     -0.066199,  -2.7065,  -22.705,  13.51,  0.78679,  16.545,
     0.13212,  -1.7258,  -1.8516,  3.4299,  -2.7238,  0.34574,
     -12.399,  10.764,  16.666,  8.018,  -16.317,  -10.751,
     -1.8497,  0.34058,  3.7207,  2.4919,  -2.7643,  -2.9296,
     0.025121,  2.4065,  -9.2416,  -2.403,  1.0396,  9.5832,
     -0.10134,  5.6628,  -20.704,  -5.4275,  1.5793,  21.484,
     11.551,  20.392,  -42.46,  -23.617,  0.55309,  38.027,
     2.2884,  7.0008,  -10.232,  -7.9753,  1.2154,  5.7981,
     -0.066199,  -22.705,  -2.7065,  16.545,  0.78679,  13.51,
     0.13212,  -1.8516,  -1.7258,  0.34574,  -2.7238,  3.4299,
     -4.5954,  10.589,  -67.743,  -15.024,  16.608,  48.606,
     0.031271,  1.1749,  -6.338,  -2.0846,  3.5566,  -2.2384,
     -12.399,  16.666,  10.764,  -10.751,  -16.317,  8.018,
     -1.8497,  3.7207,  0.34058,  -2.9296,  -2.7643,  2.4919,
     8.4322,  -1.5359,  -16.278,  -17.863,  19.593,  10.253,
     19.78,  -4.3411,  -40.165,  -41.18,  51.633,  23.407,
     -15.546,  -101.77,  -28.044,  163.67,  -68.414,  48.602,
     -2.3132,  -35.362,  -1.9451,  50.762,  -28.348,  5.9445,
     68.402,  -59.125,  -6.0128,  2.5938,  81.812,  -13.308,
     2.2067,  14.607,  -3.6773,  -15.238,  8.231,  1.4574,
     7.181,  -60.995,  0.41124,  74.597,  -107.67,  17.509,
     0.67787,  -2.3033,  5.6756,  0.74248,  -13,  -4.4736,
     -5.9674,  -216.73,  34.749,  216.2,  52.204,  -51.509,
     1.4529,  -23.145,  -0.49862,  17.601,  12.719,  -2.2251,
     8.4322,  -16.278,  -1.5359,  10.253,  19.593,  -17.863,
     19.78,  -40.165,  -4.3411,  23.407,  51.633,  -41.18,
     15.546,  28.044,  101.77,  -48.602,  68.414,  -163.67,
     2.3132,  1.9451,  35.362,  -5.9445,  28.348,  -50.762,
     7.181,  0.41124,  -60.995,  17.509,  -107.67,  74.597,
     0.67787,  5.6756,  -2.3033,  -4.4736,  -13,  0.74248,
     68.402,  -6.0128,  -59.125,  -13.308,  81.812,  2.5938,
     2.2067,  -3.6773,  14.607,  1.4574,  8.231,  -15.238,
     -5.9674,  34.749,  -216.73,  -51.509,  52.204,  216.2,
     1.4529,  -0.49862,  -23.145,  -2.2251,  12.719,  17.601,
};

// 防劈叉PID随腿长(L0_avg)的1D二次拟合：行0=Kp, 行1=Kd；列=[p0, p1, p2]
// K(L0) = p0 + p1*L0 + p2*L0^2
// 当前初值为穿过 (LEG_MIN=0.23, base)→(LEG_MAX=0.39, top) 两点的线性拟合(p2=0)：
//   Kp: 300 @ 0.23 → 700 @ 0.39
//   Kd:  10 @ 0.23 → 200 @ 0.39
float AntiSplit_K[2];                            //[0]=Kp_eff, [1]=Kd_eff
float AntiSplit_K_Fit_Coefficients[2][3] = {
    -275.0f,    2500.0f,   0.0f,   //Kp
    -263.125f,  1187.5f,   0.0f,   //Kd
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

user_pid_t Jump_LR_Balance_PID;       //跳跃左右平衡pid：输入两腿L0差值，输出±加到jump_F0上，保持两腿同步伸长

user_pid_t Leg_AntiSplit_PID;         //防劈叉pid - Kp/Kd每周期由腿长1D二次拟合得到

user_pid_t gimbal_pitch_pid;//云台俯仰pid



float target_Leg_L0 = 0.23f;//目标腿长, 初值与 LEG_MIN_LENTH 保持一致

float target_L_Leg_L0 = 0.20f;
float target_R_Leg_L0 = 0.20f;
uint8_t i;
int height_wait;
uint8_t temp1;
user_pid_t L_Spin_Phi0_PID, R_Spin_Phi0_PID;
float target_spin_phi0 = PI / 2.0f;
float spinning_target_d_yaw_cmd = 0.0f;
float spinning_d_yaw_feedback = 0.0f;
float alpha_spinning_target_d_yaw = 0.02f;
float alpha_spinning_d_yaw = 0.2f;
float alpha_spinning_down_target_d_yaw = 0.08f;
float alpha_spinning_stop_target_d_yaw = 0.05f;

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
// 小陀螺时允许触发平移的yaw_angle_PI误差窗口(rad)：
// |yaw_angle_PI| <= 此值时，速度倍率从0线性升至+1（正向）；
// |yaw_angle_PI - ±PI| <= 此值时，倍率从0线性降至-1（反向）；
// 其余角度倍率为0。建议0.2~0.5rad
float spin_speed_tol_angle = 1.0f;
// 小陀螺平移方向偏置(rad)：补偿"拨杆向前-实际方向"的安装/解算偏差。
// 正负与 yaw_angle_PI 同号系：调一调正负看车的实际响应方向。建议先±0.1rad尝试。
float spin_speed_angle_offset = -0.9f;

// 跳跃模式（只读输出）：每周期由 Standing 根据 jump_cmd 和 jump_locked 计算得出
// =1 表示当前正在跳跃动作组中；倒地自起期间 Standing 不会被调用，天然互斥
uint8_t jump_mode = 0;
// B2B byte51 原始跳跃指令，由 Board2Board.c 写入
uint8_t jump_cmd = 0;
// 跳跃使能：=1 允许跳跃，=0 即使 jump_mode 已置位也不执行跳跃动作（一票否决）
uint8_t jump_enable = 1;
// 跳跃蜂鸣器独占标志：跳跃期间置1，Error_Buzzer_Tick 看到它必须让位
uint8_t g_jump_buzzer_active = 0;
// 跳跃锁：起跳后检测到离地时上锁，期间 jump_mode 强制为 0；jump_cmd 回 0 时解锁
// 作用：防止一次 byte51=1 期间因落地→再次满足条件而连续触发多次跳跃
static uint8_t jump_locked = 0;
// 跳跃时两腿沿腿杆向外的固定虚拟力 F0 (单位 N)，跳过腿长 PID 直接喂给 VMC
// 重力支撑约 30N/腿；跳跃需远大于此值
// !! 当前允许的最大值约 250N (受 DM8009 峰值扭矩与短腿姿态几何约束，超过会被电机饱和)
// 建议从 120N 起步，逐步上调到合适跳跃高度
float jump_F0 = 50.0f;
// 跳跃失败阈值(m)：0.5s 锁存到期时，若任意一腿 L0 变化量小于该值，判定为跳跃失败
float jump_leg_change_threshold = 0.15f;
// 跳跃退出原因（调试用，只读输出）：
//   0 = 未触发 / 锁存中
//   1 = 离地，跳跃成功
//   2 = 0.5s 到期且腿长变化不足 → 明确失败（电机饱和/腿被卡）
//   3 = 0.5s 到期但腿长已变化够 → 超时（疑似传感器漏检离地）
uint8_t jump_fail_reason = 0;

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
    PID_INIT(&L_Leg_L0_PID, 2500, 0, 30000, 200, 0, 0, 0, 0);
    PID_INIT(&R_Leg_L0_PID, 2500, 0, 30000, 200, 0, 0, 0, 0);
    PID_INIT(&Leg_AntiSplit_PID, 300, 0, 10, 150, 0, 0, 0, 0);   //Kp/Kd为占位，每周期由 AntiSplit_Get_K 覆盖
    PID_INIT(&L_Spin_Phi0_PID, 80, 0, 8, 40, 0, 0, 0, 0);
    PID_INIT(&R_Spin_Phi0_PID, 80, 0, 8, 40, 0, 0, 0, 0);
    PID_INIT(&Roll_Comp_PID, 20, 0.002, 100, 150, 80, 0, 10000, 0);
    // 跳跃LR平衡：输入VMC_L.L0-VMC_R.L0(m)，target=0；out_limit占jump_F0约1/3，避免单侧饱和
    PID_INIT(&Jump_LR_Balance_PID, 1500, 0, 2000, 40, 0, 0, 0, 0);

    PID_INIT(&L_Leg_Middle_PID, 15, 0.1, 0.1, 5.0, 4.0, 0, 0, 0);
    PID_INIT(&R_Leg_Middle_PID, 15, 0.1, 0.1, 5.0, 4.0, 0, 0, 0);
    PID_INIT(&L_Leg_dphi0_PID, 3, 0.1, 1, 150, 150, 0, 2000, 0);
    PID_INIT(&R_Leg_dphi0_PID, 3, 0.1, 1, 150,150, 0, 2000, 0);

    PID_INIT(&L_Leg_L0_POS_PID, 15, 0.001, 0.1, 3.0, 2.0, 0, 200, 0);
    PID_INIT(&R_Leg_L0_POS_PID, 15, 0.001, 0.1, 3.0, 2.0, 0, 200, 0);
    PID_INIT(&L_Leg_L0_SPD_PID, 50, 3, 400, 80, 80, 0, 2000, 0);
    PID_INIT(&R_Leg_L0_SPD_PID, 50, 3, 400, 80, 80, 0, 2000, 0);

    //小陀螺pid
    PID_INIT(&spinning_pid, 0.0005, 0.001f, 0.001, 6.0f, 6.0f, 0.005f, 20.0f, 0);

    //云台pid
    PID_INIT(&gimbal_pitch_pid, 10, 0.002, 100, 150, 80, 0, 10000, 0);
    PID_INIT(&spinning_speed_pid, -6, 0, -500, 6, 0, 0, 0, 0);
    
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

    // 标记机身所有DM关节电机期望状态为"已使能"，开启监督
    motor_should_enabled = 1;
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
        if (fabsf(pid_middle->error) <= 0.1f) near = 1;

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
                      (*rev_traveled > PI && se_now < 0.1f);
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
    //自起完成蜂鸣音 latch：刚离开自起态时响 100ms 高 si
    static uint8_t sr_was_active = 0;
    static int sr_finish_chime_remain = 0;

    VMC_Coculate();
    Body_Speed_Coculate();//车身速度解算

    //是否姿态稳定在误差20°内的起立态
    if((roll >= 40.0f || roll <= -40.0f || pitch >= 40.0f || pitch <= -40.0f) && first_run == 1)//不稳定且是急停开始第一次运行
    {
        gimbal_follow_flag = 1;//自起期间云台跟随底盘
        //只在进入自起的第一拍锁定方向：|pitch|>90° → 反面倒地 dir=+1；正面倒地 → dir=-1
        //后续 tick 沿用第一拍的 dir，防止自起过程中 pitch 穿越 90° 边界导致方向抖动
        if (sr_was_active == 0)
        {
            g_sr_turn_dir = (pitch > 90.0f || pitch < -90.0f) ? 1 : -1;
        }
        g_tip_recovery_active = 1;//占用蜂鸣器，错误码蜂鸣器必须让位
        Self_Righting_Step();
        sr_was_active = 1;
        return ;
    }

    //倒地自起成功后复位Self_Righting的状态机（stage / sync_from_stuck / VMC输出 / 蜂鸣器 / tick 一次性归零）
    if (sr_was_active)
    {
        Self_Righting_Reset();
        sr_finish_chime_remain = 50;  //50 * 2ms = 100ms
        sr_was_active = 0;
    }
    if (sr_finish_chime_remain > 0)
    {
        g_tip_recovery_active = 1;//完成提示音期间继续独占蜂鸣器
        Buzzer_Tone_Max(1976);
        sr_finish_chime_remain--;
        if (sr_finish_chime_remain == 0)
        {
            Stop_Buzzer();
            g_tip_recovery_active = 0;//完成提示音结束，把蜂鸣器交还给错误码代码
        }
    }
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
    if(L_Leg_State == 0 && fabsf(L_Leg_L0_POS_PID.error) <= 0.04)
    {
        L_Ready_Count ++;
    }
    if(L_Leg_State == 0 && L_Ready_Count >= 20)//腿到目标长度
    {
        L_Leg_State = 1;    //收腿完成
        L_Ready_Count = 0;  //归零
        L_stair_sub = STAIR_SUB_TURN_FWD;
        L_sub_dwell = 0;
        leg_turn_stuck_reset(&VMC_L);
    }
    if(R_Leg_State == 0 && fabsf(R_Leg_L0_POS_PID.error) <= 0.04)
    {
        R_Ready_Count ++;
    }
    if(R_Leg_State == 0 && R_Ready_Count >= 20)
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
    if(L_Leg_State == 1 && L_Ready_Count >= 20)
    {
        L_Leg_State = 2;
        L_Ready_Count = 0;
    }
    if(R_Leg_State == 1 && R_near) R_Ready_Count ++;
    else if(R_Leg_State == 1)      R_Ready_Count = 0;
    if(R_Leg_State == 1 && R_Ready_Count >= 20)
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
        target_body_distance = 0.5;
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
        lqr_body_distance_error = 0.0f;   // 小陀螺时禁止距离闭环，避免误差累积干扰平移
    }
    if(upstairs_flag == 1)
    {
        lqr_body_distance_error = 0.0f;   // 上台阶动作组触发后禁止距离闭环
    }

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
    - LQR_K[2][4] * VMC_L.b_phi0
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
    - LQR_K[3][6] * VMC_R.b_phi0
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
    else if (L_Ground_F0 >= 10.0f)
    L_off_ground --;
    if(L_off_ground >= 20)
    L_off_ground = 20;
    if(L_off_ground <= 0)
    L_off_ground = 0;
    
    if(R_Ground_F0 <= 50.0f)
    R_off_ground ++;
    else if (R_Ground_F0 >= 10.0f)
    R_off_ground --;
    if(R_off_ground >= 20)
    R_off_ground = 20;
    if(R_off_ground <= 0)
    R_off_ground = 0;

    //!这段是先算一遍不离地的情况的数，再检测是否离地，如果离地，就再算一次覆盖掉
    if(L_off_ground >= 10 && R_off_ground >= 10)//两腿同时离地：锁定 Target_Leg_State 为短腿，直到上位机重新发短腿(0)才解锁
    {
        leg_state_locked_short = 1;
    }
    if(L_off_ground >= 10)//正常行驶过程离地
    {
        //离地后腿归中，轮子脱力vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F%E7%A6%BB%E5%9C%B0%E5%90%8E%E8%85%BF%E5%BD%92%E4%B8%AD%EF%BC%8C%E8%BD%AE%E5%AD%90%E8%84%B1%E5%8A%9B
        Leg_L_T = 
        - LQR_K[2][4] * (VMC_L.b_phi0) 
        - LQR_K[2][5] * VMC_L.d_b_phi0 ;
        Leg_L_T *= 0.7; //收腿力度参数
        L_DJ3508.Target_Torque = 0;//离地轮子脱力
        //正常行驶过程离地VMC解算
        VMC_Set_F0_T(&VMC_L, L_Leg_L0_PID.output * 0.5, Leg_L_T);//VMC解算
        //离地距离相关量归零
        body_distance = 0;
        target_body_distance = 0.0;
    }
    if(R_off_ground >= 10)
    {
        Leg_R_T =
        - LQR_K[3][6] * (VMC_R.b_phi0)
        - LQR_K[3][7] * VMC_R.d_b_phi0;
        Leg_R_T *= 0.7;
        R_DJ3508.Target_Torque = 0;
        VMC_Set_F0_T(&VMC_R, R_Leg_L0_PID.output * 0.5, -Leg_R_T);
        body_distance = 0;
        target_body_distance = 0.0;
    }
}

static void Step_Hit_Detect(void)
{
    const float leg_torque_threshold = 3.0;   // 单腿力矩门槛
    const float leg_torque_sum_threshold = 5.0;   // 双腿力矩合力门槛(N·m)：覆盖单边顶台阶情况
    const float leg_length_threshold = LEG_MAX_LENTH - 0.03f; // 高腿长门槛
    const int step_hit_cooldown_target = motor_HZ / 2; // 0.5s 冷却
    static int step_hit_cooldown = 0;

    float left_leg_torque_cmd = fabsf(VMC_L.T_actual);
    float right_leg_torque_cmd = fabsf(VMC_R.T_actual);

    int left_leg_torque_high = (left_leg_torque_cmd > leg_torque_threshold);
    int right_leg_torque_high = (right_leg_torque_cmd > leg_torque_threshold);

    int left_leg_high = (VMC_L.L0 > leg_length_threshold);
    int right_leg_high = (VMC_R.L0 > leg_length_threshold);

    int left_step_hit = left_leg_torque_high && left_leg_high;
    int right_step_hit = right_leg_torque_high && right_leg_high;

    // 合力支路：两腿都在高腿长，且力矩之和大于合力门槛
    int torque_sum_hit = (left_leg_torque_cmd + right_leg_torque_cmd > leg_torque_sum_threshold)
                         && left_leg_high && right_leg_high;

    int step_hit = (left_step_hit && right_step_hit) || torque_sum_hit;

    int leg_off_ground = (L_off_ground >= 10 || R_off_ground >= 10);

    if (step_hit_cooldown > 0)
    {
        step_hit_cooldown--;
    }

    if (step_hit_cooldown == 0 && !leg_off_ground && step_hit && Foot_Chassis.Target_Leg_State == 1 && start_mode == 1 && upstares_mode == 0)
    {
        upstairs_flag = 1;
        step_hit_cooldown = step_hit_cooldown_target;
    }

    prev_L_Ground_F0 = L_Ground_F0;
    prev_R_Ground_F0 = R_Ground_F0;
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
    spinning_target_d_yaw_cmd = alpha_spinning_target_d_yaw * spinning_setpoint
                              + (1.0f - alpha_spinning_target_d_yaw) * spinning_target_d_yaw_cmd;
    spinning_d_yaw_feedback = alpha_spinning_d_yaw * d_yaw
                            + (1.0f - alpha_spinning_d_yaw) * spinning_d_yaw_feedback;
    PID_Set_Error(&spinning_pid, spinning_d_yaw_feedback, spinning_target_d_yaw_cmd);
    yaw_error = PID_coculate(&spinning_pid);
    Speed_Error_Set();
}

//统一小陀螺退出：转速降低过程中平滑引入角度修正，同时受功率门控
void spinning_exit()
{
    // 角度修正量：P-only，Kp=-6，即 -6 * yaw_angle_PI
    PID_Set_Error(&spinning_speed_pid, yaw_angle_PI, 0);
    float angle_correction = PID_coculate(&spinning_speed_pid);

    // 混合权重：转速越高 weight 越小，专注减速；转速越低 weight 越大，角度归位
    float speed_ratio = fabsf(d_yaw) / target_spinning_d_yaw;
    if (speed_ratio > 1.0f) speed_ratio = 1.0f;
    float angle_weight = 1.0f - speed_ratio;

    float blended_target = 1.0 * angle_correction;

    // 功率门控：超功率时缩放目标，与 spinning_up 一致
    if (g_filtered_power > power_limit)
    {
        blended_target *= g_power_obs_lambda;
    }

    PID_Set_Error(&spinning_pid, d_yaw, blended_target);
    yaw_error = PID_coculate(&spinning_pid);
    Speed_Error_Set();
}

void Sit_On_Ground_Action(void);
static uint8_t sit_first_entry = 1;

//站起
void Standing()
{
//占用率检测用的，留着吧，看不懂也不影响
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, 1);

    /* 倾覆保护：站立期间 |pitch| > 45° 连续 10 帧才触发，避免 IMU 抖动误判。
       触发后用位速双环PID把腿长压到最短(LEG_MIN_LENTH)、腿角拉到PI/2.0f(竖直)，
       不绕长路、不区分子状态，维持0.5s后回退到未站起状态
       （start_mode=0, upstares_mode=0, first_run=1）。
       Motor_task 周期 2ms，0.5s = 250 tick，10 帧滤波 = 20ms。 */
    static uint16_t tip_protect_cnt = 0;
    static uint8_t  tip_detect_cnt  = 0;
    if (tip_protect_cnt == 0)
    {
        if (pitch > 45.0f || pitch < -45.0f)
        {
            if (tip_detect_cnt < 10) tip_detect_cnt++;
            if (tip_detect_cnt >= 10)
            {
                tip_protect_cnt = 250;
                tip_detect_cnt  = 0;
            }
        }
        else
        {
            tip_detect_cnt = 0;
        }
    }
    if (tip_protect_cnt > 0)
    {
        VMC_Coculate();

        // 腿长位速双环：目标=最短腿长
        PID_Set_Error(&L_Leg_L0_POS_PID, VMC_L.L0, LEG_MIN_LENTH);
        PID_Set_Error(&R_Leg_L0_POS_PID, VMC_R.L0, LEG_MIN_LENTH);
        PID_coculate(&L_Leg_L0_POS_PID);
        PID_coculate(&R_Leg_L0_POS_PID);
        PID_Set_Error(&L_Leg_L0_SPD_PID, VMC_L.d_L0, L_Leg_L0_POS_PID.output);
        PID_Set_Error(&R_Leg_L0_SPD_PID, VMC_R.d_L0, R_Leg_L0_POS_PID.output);
        PID_coculate(&L_Leg_L0_SPD_PID);
        PID_coculate(&R_Leg_L0_SPD_PID);

        // 腿角双环：目标=PI/2.0f（不绕长路，直接走短路径）
        PID_Set_AngleError(&L_Leg_Middle_PID, VMC_L.phi0, PI / 2.0f);
        PID_coculate(&L_Leg_Middle_PID);
        PID_Set_Error(&L_Leg_dphi0_PID, VMC_L.d_phi0, L_Leg_Middle_PID.output);
        PID_coculate(&L_Leg_dphi0_PID);

        PID_Set_AngleError(&R_Leg_Middle_PID, VMC_R.phi0, PI / 2.0f);
        PID_coculate(&R_Leg_Middle_PID);
        PID_Set_Error(&R_Leg_dphi0_PID, -VMC_R.d_phi0, -R_Leg_Middle_PID.output);
        PID_coculate(&R_Leg_dphi0_PID);

        VMC_Set_F0_T(&VMC_L, L_Leg_L0_SPD_PID.output,  L_Leg_dphi0_PID.output);
        VMC_Set_F0_T(&VMC_R, R_Leg_L0_SPD_PID.output, -R_Leg_dphi0_PID.output);

        L_DJ3508.Target_Torque = 0;
        R_DJ3508.Target_Torque = 0;

        tip_protect_cnt--;
        if (tip_protect_cnt == 0)
        {
            start_mode      = 0;
            upstares_mode   = 0;
            first_run       = 1;
            sit_first_entry = 1;
        }
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, 0);
        return;
    }

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
        else//退出小陀螺 or 普通运行
        {
            if(spinning_flag == 1)//小陀螺退出（减速+归位统一）
            {
                spinning_usable = 0;

                spinning_exit();

                if(fabsf(d_yaw) <= 4.0f && fabsf(yaw_angle_PI) <= 0.5f)
                {
                    spinning_flag = 0;
                }
            }
            else if(spinning_flag == 0)//常态
            {
                spinning_pid.I = 0;
                spinning_target_d_yaw_cmd = d_yaw;
                spinning_d_yaw_feedback = d_yaw;
                spinning_usable = 1;
                Yaw_Error_Coculate();
            }
        }
    }

    yaw_error = 0.05 * yaw_error + 0.95 * last_yaw_error;
    last_yaw_error = yaw_error;

    //计算距离误差
    Distance_Error_Set();
    if (spinning_flag == 1) { body_distance_error = 0; body_distance = 0; target_body_distance = 0; }

    //小陀螺时轮速共模P反馈：抑制因左右轮共模偏置导致的车身漂移
    if (spinning_flag == 1) {
        static float wheel_common_f = 0.0f;
        float wheel_common = (R_DJ3508.Rx_Data.Velocity - L_DJ3508.Rx_Data.Velocity) * 0.0305f;
        wheel_common_f = 0.05f * wheel_common + 0.95f * wheel_common_f;
        speed_error -= wheel_common_f * 0.1f;
    }

    //横滚补偿和PD单环腿长控制
    Roll_Comp();
    Leg_L0_Control();

    //防劈叉：Kp/Kd按L0_avg做1D二次拟合(类似LQR风格)，单PID输出
    float L0_avg = (VMC_L.L0 + VMC_R.L0) * 0.5f;
    AntiSplit_Get_K(AntiSplit_K, AntiSplit_K_Fit_Coefficients, L0_avg);
    Leg_AntiSplit_PID.Kp = AntiSplit_K[0];
    Leg_AntiSplit_PID.Kd = AntiSplit_K[1];
    PID_Set_Error(&Leg_AntiSplit_PID, (VMC_R.phi0 - PI/2) + (VMC_L.phi0 - PI/2), 0);
    PID_coculate(&Leg_AntiSplit_PID);
    float anti_split_out = Leg_AntiSplit_PID.output;

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
    float L_leg_T_cmd = Leg_L_T + anti_split_out - centrifugal_comp;
    float R_leg_T_cmd = -Leg_R_T + anti_split_out - centrifugal_comp;
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
    // 跳跃指令解锁：jump_cmd 回 0 即清锁，允许下一次跳跃
    if (jump_cmd == 0)
    {
        jump_locked = 0;
    }
    // jump_mode 由原始指令和锁共同决定，外部不再直接写
    jump_mode = (jump_cmd && !jump_locked) ? 1 : 0;

    // 跳跃动作组：绕过腿长 PID 和重力补偿，两腿沿腿杆方向直接喂 jump_F0
    // 实际触发：jump_mode && jump_enable && 短腿 && !小陀螺 && 双腿未离地
    // T 维持原 LQR/anti-split 输出以保持平衡姿态
    uint8_t jump_active_raw = (jump_mode == 1
                               && jump_enable == 1
                               && spinning_flag == 0
                               && Foot_Chassis.Target_Leg_State == 0
                               && L_off_ground < 10
                               && R_off_ground < 10);

    // 跳跃锁存：raw 首次满足 → 启动 0.5s 锁存窗口
    // 锁存期间无视 raw 条件强制 jump_active=1，原表达式无权清零
    // 退出条件（统一在此处上锁，确保"一次跳跃只触发一次"）：
    //   ①离地立即解除 → jump_fail_reason = 1（成功）
    //   ②0.5s 到期 + 腿长变化不足 → jump_fail_reason = 2（失败）
    //   ③0.5s 到期 + 腿长够 → jump_fail_reason = 3（超时但伸够了，疑似漏检）
    static uint8_t jump_active_latched     = 0;
    static int     jump_active_latch_count = 0;
    static float   jump_L0_start_L         = 0.0f;
    static float   jump_L0_start_R         = 0.0f;
    const  int     jump_active_latch_target = motor_HZ / 2;   // 0.5s = 250 cycles

    if (jump_active_raw && !jump_active_latched)
    {
        jump_active_latched     = 1;
        jump_active_latch_count = jump_active_latch_target;
        jump_L0_start_L         = VMC_L.L0;
        jump_L0_start_R         = VMC_R.L0;
        jump_fail_reason        = 0;
    }
    if (jump_active_latched)
    {
        if (L_off_ground >= 10 || R_off_ground >= 10)
        {
            jump_fail_reason    = 1;
            jump_active_latched = 0;
            jump_locked         = 1;
            jump_mode           = 0;
        }
        else if (--jump_active_latch_count <= 0)
        {
            float L_change = VMC_L.L0 - jump_L0_start_L;
            float R_change = VMC_R.L0 - jump_L0_start_R;
            jump_fail_reason    = (L_change < jump_leg_change_threshold
                                   || R_change < jump_leg_change_threshold) ? 2 : 3;
            jump_active_latched = 0;
            jump_locked         = 1;
            jump_mode           = 0;
        }
    }

    uint8_t jump_active = jump_active_latched ? 1 : jump_active_raw;
    if (jump_active)
    {
        // 左右平衡：err = 0 - (L0_L - L0_R) = L0_R - L0_L
        // 左腿较长时 err<0 → output<0 → 左 F 减小、右 F 增大，使两腿伸长速度同步
        PID_Set_Error(&Jump_LR_Balance_PID, VMC_L.L0 - VMC_R.L0, 0);
        PID_coculate(&Jump_LR_Balance_PID);
        float lr_bal = Jump_LR_Balance_PID.output;
        VMC_Set_F0_T(&VMC_L, jump_F0 + lr_bal, L_leg_T_cmd);
        VMC_Set_F0_T(&VMC_R, jump_F0 - lr_bal, R_leg_T_cmd);
    }
    else
    {
        // 退出跳跃时清零所有状态：PID_Clear只清 I；额外清 error/pre_error/output 避免下次进入瞬间 D 项暴冲
        PID_Clear(&Jump_LR_Balance_PID);
        Jump_LR_Balance_PID.error     = 0.0f;
        Jump_LR_Balance_PID.pre_error = 0.0f;
        Jump_LR_Balance_PID.output    = 0.0f;
        VMC_Set_F0_T(&VMC_L, L_Leg_L0_PID.output + (mg / arm_cos_f32(VMC_L.b_phi0)) + Roll_Comp_PID.output,
                     L_leg_T_cmd);
        VMC_Set_F0_T(&VMC_R, R_Leg_L0_PID.output + (mg / arm_cos_f32(VMC_R.b_phi0)) - Roll_Comp_PID.output,
                     R_leg_T_cmd);
    }

    // 跳跃蜂鸣器：跳跃中长鸣中音 sol，结束立刻停。边沿触发避免 PWM 频繁重配
    // 注意必须用 Buzzer_Tone_Max(784) 而不是 Buzzer_sol()：后者音量受 SBUS_CH.CH10 旋钮调制，
    // 旋钮没拨上去时实际占空比为 0 → 完全无声
    if (jump_active != g_jump_buzzer_active)
    {
        if (jump_active) Buzzer_Tone_Max(784);
        else             Stop_Buzzer();
        g_jump_buzzer_active = jump_active;
    }

    off_ground_detect();
    Step_Hit_Detect();
    // 注：跳跃上锁统一在上面锁存退出时处理（离地 / 0.5s 到期），此处不再需要兜底

    if(upstairs_flag == 1)
    {
        start_mode = 2;
        upstairs_flag = 0;    // 已消费触发信号，避免动作完成后残留导致距离闭环被永久压0
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
    if(L_Leg_State == 0 && fabsf(L_Leg_L0_POS_PID.error) <= 0.15 && fabsf(L_Leg_Middle_PID.error) <= 0.15)
    {
        L_Ready_Count ++;
    }
    if(L_Leg_State == 0 && L_Ready_Count >= 60)
    {
        L_Leg_State = 2;
        L_Ready_Count = 0;
    }
    if(R_Leg_State == 0 && fabsf(R_Leg_L0_POS_PID.error) <= 0.15 && fabsf(R_Leg_Middle_PID.error) <= 0.15)
    {
        R_Ready_Count ++;
    }
    if(R_Leg_State == 0 && R_Ready_Count >= 60)
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
    if(L_Leg_State == 0 && fabsf(L_Leg_L0_POS_PID.error) <= 0.05f) L_Ready_Count ++;
    else if(L_Leg_State == 0) L_Ready_Count = 0;
    if(L_Leg_State == 0 && L_Ready_Count >= 50)
    {
        L_Leg_State = 1;
        L_Ready_Count = 0;
        L_stair_sub = STAIR_SUB_TURN_FWD;
        L_sub_dwell = 0;
        leg_turn_stuck_reset(&VMC_L);
    }
    if(R_Leg_State == 0 && fabsf(R_Leg_L0_POS_PID.error) <= 0.05f) R_Ready_Count ++;
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

    //两腿到位后固定延时再切Standing，跳过NotStanding冗余状态迁移
    static int stair_dwell_cnt = 0;
    if (R_Leg_State == 2 && L_Leg_State == 2)
    {
        stair_dwell_cnt++;
        if (stair_dwell_cnt >= 150) // 300ms
        {
            stair_dwell_cnt = 0;
            upstares_mode = 0;
            start_mode = 1;
            R_Leg_State = 0;
            L_Leg_State = 0;
            leg_state = 0;
            target_Leg_L0 = LEG_MIN_LENTH;
            body_distance = 0;
            target_body_distance = 0.0f;
        }
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

static RampGenerator sit_L0_ramp_L, sit_L0_ramp_R;
static RampGenerator sit_phi0_ramp_L, sit_phi0_ramp_R;

uint16_t sit_debug_counter = 0;  // 坐地模式执行周期计数，debug用
uint8_t sit_ramp_done = 0;       // 斜坡过渡是否完成

void Sit_On_Ground_Action(void)
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
}

void Sit_On_Ground(void)
{
    Sit_On_Ground_Action();

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

        //错误码蜂鸣器：电机错误时长响低音。倒地自起激活时本函数自动让位，不碰蜂鸣器
        Error_Buzzer_Tick();

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
