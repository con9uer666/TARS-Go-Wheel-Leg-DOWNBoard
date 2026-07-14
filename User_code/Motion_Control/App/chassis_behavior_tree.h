/**
 * @file chassis_behavior_tree.h
 * @brief 底盘行为树聚合公共头（取代原 motor.h）。
 *
 * 集中声明：
 *   - 共享类型：Foot_Chassis_t / Leg_Info_t / Foot_Chassis_Info_t（RampGenerator 见 ramp_generator.h）
 *   - 全部跨文件 extern 变量（反馈量 / 标志位 / 调参 / PID / LQR 增益等）
 *   - 全部对外函数原型（主循环 + 各动作组 + 各计算）
 *
 * 原 motor.c 被拆分为多个细粒度文件，全部包含本头；变量按动作就近定义，
 * 跨动作共享的反馈量/PID 定义在 chassis_state.c。
 */

#ifndef CHASSIS_BEHAVIOR_TREE_H
#define CHASSIS_BEHAVIOR_TREE_H

#include "main.h"
#include "user_pid.h"
#include "ramp_generator.h"
#include "VMC.h"

/** @brief 最小腿长 (m) */
extern float LEG_MIN_LENTH;
/** @brief 最大腿长 (m) */
extern float LEG_MAX_LENTH;
/** @brief 轮子半径 (m) */
#define WHEEL_RADIUS 0.061f

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
    float Remote_control_x;     /**< 遥控器前后通道，归一化 [-1, 1] */
    float Remote_control_y;     /**< 遥控器左右通道，归一化 [-1, 1] */
    float Target_Vx;           /**< 云台坐标系下的目标 X 速度 (m/s) */
    float Target_Vy;           /**< 云台坐标系下的目标 Y 速度 (m/s) */
    uint8_t Target_Leg_State;  /**< 目标腿长：0=短腿，1=长腿 */
    uint8_t Chassis_Mode;      /**< 底盘模式：0=跟随，1=小陀螺，2=静止趴下 */
    Foot_Chassis_Info_t Info;  /**< 底盘实时信息 */
} Foot_Chassis_t;

/* ---------- 收腿起立 State=1 转角阶段子状态（leg_retract_common） ---------- */
// FWD = 短路径 PID 追目标角；REV = 长路径恒速反绕一圈到同一目标角
typedef enum {
    STAIR_SUB_TURN_FWD = 0,  // 沿 ShortestAngleDelta 的短路径用 PID 串级控制
    STAIR_SUB_TURN_REV       // 沿反方向 (2π − 短路径) 匀速转到原目标角
} StairSub_t;


/*====================================== 跨文件共享变量 =========================================== */

/* ---- chassis_state.c：跨动作共享反馈量/常数/标志/PID ---- */
extern Foot_Chassis_t Foot_Chassis;
extern float mg;
extern float Leg_F0_Limit;
extern float wheel_track_R;
extern uint16_t motor_HZ;
extern float b_phi0_offset;
extern float powerPredict;
extern float L_b_phi0, R_b_phi0;

extern float pitch_trans[2];
extern float d_pitch;
extern float alpha_d_pitch;
extern float yaw_trans[2];
extern float d_yaw;
extern float alpha_d_yaw;

extern float Wr, Wl;
extern float alpha_W;
extern float body_speed_L, body_speed_R, body_speed;
extern float target_body_speed;
extern float speed_limit;
extern float speed_error;
extern float alpha_target_body_speed;
extern float alpha_body_speed;
extern float body_distance;
extern float target_body_distance;
extern float body_distance_error;
extern float yaw_error;
extern float alpha_target_roll;

extern float target_Leg_L0;
extern float target_L_Leg_L0;
extern float target_R_Leg_L0;

extern uint8_t gimbal_follow_flag;
extern uint8_t first_run;
extern uint8_t upstares_mode;
extern int ready_count;
extern uint8_t leg_state;
extern int leg_state_count;
extern RampGenerator Target_Speed_Ramp;
extern float down_board_yaw_output;

extern user_pid_t L_Leg_L0_PID, R_Leg_L0_PID;
extern user_pid_t L_Leg_L0_POS_PID, R_Leg_L0_POS_PID;
extern user_pid_t L_Leg_L0_SPD_PID, R_Leg_L0_SPD_PID;
extern user_pid_t L_Leg_Middle_PID, R_Leg_Middle_PID;
extern user_pid_t L_Leg_dphi0_PID, R_Leg_dphi0_PID;
extern user_pid_t Roll_Comp_PID;
extern user_pid_t Leg_AntiSplit_PID;
extern user_pid_t gimbal_pitch_pid;

/* ---- lqr_controller.cpp：供旧 C 离地保护和调试器读取的兼容输出 ---- */
extern float LQR_K[4][12];
extern float Leg_L_T, Leg_R_T;

/* ---- off_ground_detect.c ---- */
extern float L_Ground_F0, R_Ground_F0;
extern int L_off_ground, R_off_ground;
extern int step_hit_cooldown;

/* ---- balance_controller.cpp 内部 StepHitDetector ---- */
/**
 * @brief 磕台阶检测结果是否允许自动触发上台阶。
 *
 * 0：仅继续计算碰撞条件、命中计数和冷却时间，不产生 upstairs_flag；
 * 1：检测连续命中后产生 upstairs_flag，由平衡模式切换到 start_mode=2。
 * 该开关不影响调试器或其他模块直接写入 start_mode=2。
 */
extern uint8_t automatic_stair_climb_enable;

/* ---- spinning_motion.c ---- */
extern uint8_t spinning_flag;
extern uint8_t spinning_usable;
extern float centrifugal_comp_gain;
extern float spin_speed_tol_angle;
extern float spin_speed_angle_offset;
extern user_pid_t spinning_pid;
extern user_pid_t spinning_speed_pid;
extern user_pid_t L_Spin_Phi0_PID, R_Spin_Phi0_PID;
extern float target_spin_phi0;
extern float spinning_target_d_yaw_cmd;
extern float spinning_d_yaw_feedback;

/* ---- jump_motion.c ---- */
extern uint8_t jump_mode;            // 只读：当前是否处于跳跃中，由 jump_motion 内部根据 jump_cmd / jump_locked 计算
extern uint8_t jump_cmd;             // B2B byte51 原始跳跃指令：1=请求跳跃，0=解除跳跃锁
extern uint8_t jump_enable;          // 跳跃使能：=1 允许跳跃，=0 一票否决
extern uint8_t g_jump_buzzer_active; // 跳跃蜂鸣器独占标志，Error_Buzzer_Tick 看到=1 必须让位
extern float jump_L0_step_delta;     // 跳跃时 L0 目标阶跃量 (m)，叠加到 target_Leg_L0 让 L0 PID 闭环产生跳跃力
extern float jump_leg_change_threshold; // 跳跃失败阈值(m)：锁存到期时任一腿变化<此值判失败
extern uint8_t jump_fail_reason;     // 跳跃退出原因 0=进行中/未触发 1=离地成功 2=超时腿长不足 3=超时但腿长够

/* ---- leg_retract_common.c：自起收腿与上台阶收腿共用的转角子状态机持久量 ---- */
extern uint8_t L_Leg_State, R_Leg_State;
extern uint16_t L_Ready_Count, R_Ready_Count;
extern StairSub_t L_stair_sub, R_stair_sub;
extern int   L_sub_dwell, R_sub_dwell;
extern float L_rev_dir, R_rev_dir;
extern float L_rev_long_remain, R_rev_long_remain;
extern float L_rev_traveled, R_rev_traveled;

/* ---- sit_controller.cpp 兼容调试符号 ---- */
extern uint8_t sit_first_entry;
extern uint16_t sit_debug_counter;
extern uint8_t sit_ramp_done;

/* ---- 定义在其它模块、本子系统引用（保持原有 extern 关系） ---- */
extern uint8_t start_mode;          // 定义于 User_State.c
extern uint8_t sit_mode_enable;     // 坐地模式使能
extern float head_forward_angle;    // 云台物理零点偏置


/*====================================== 对外函数原型 =========================================== */

/* ---- chassis_init.c ---- */
void task_Motor_Init(void);
void task_VMC_Init(void);
void task_PID_Init(void);
void task_Pitch_Coculate(void);

/* ---- motor_enable.c ---- */
void task_Motor_Enable(void);

/* ---- chassis_control_task.cpp（C++ 主循环调度器） ---- */
void Motor_task(void const *argument);

/* ---- spinning_motion.c ---- */
float spinning_up(void);
float spinning_exit(void);

/* ---- jump_motion.c ---- */
uint8_t Jump_Motion_Update(void);

/* ---- off_ground_detect.c ---- */
void off_ground_detect(void);

/* ---- Lqr_Error_Calculate.h（Ctrl/src/Lqr_Error_Calculate.c） ---- */
#include "Lqr_Error_Calculate.h"

/* ---- leg_retract_common.c ---- */
int turn_ctrl_with_stuck_flip(
    VMC_t *VMC, int is_right, float target_angle,
    user_pid_t *pid_middle, user_pid_t *pid_dphi0,
    StairSub_t *sub, int *dwell,
    float *rev_dir, float *rev_long_remain, float *rev_traveled,
    float *out_T);

/* ---- gravity_comp_test.c ---- */
void Gravity_Compensation_Test_Function(void);

#endif // CHASSIS_BEHAVIOR_TREE_H
