/**
 * @file user_pid.h
 * @brief 通用 PID 控制器数据结构与接口声明。
 *
 * 该模块实现单轴 PID 控制，包括：
 *   - 比例、积分、微分三项计算
 *   - 角度误差归一化（最短路径）
 *   - 积分死区与积分变化率限制（I_step）
 *   - 输出死区
 *
 * @note 接口函数具体实现见 user_pid.c。
 */

#ifndef PID_H
#define PID_H

/** @brief 数值限幅宏：将 x 限制在 [min, max] 区间内 */
#define LIMIT(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

/**
 * @brief PID 控制器结构体。
 *
 * 存储 PID 控制器的增益、状态和限幅参数。
 * 所有成员变量均为浮点型，适用于连续或离散控制系统。
 */
typedef struct {
    float Kp;               /**< 比例增益 */
    float Kd;               /**< 微分增益 */
    float Ki;               /**< 积分增益 */
    float I;                /**< 当前积分值 */
    float I_limit;          /**< 积分项绝对值限幅 */
    float I_step;           /**< 积分变化率限制（每步最大增量，<=0 时无限制） */
    float out_limit;        /**< 输出绝对值限幅 */
    float error;            /**< 当前误差（目标 - 当前） */
    float pre_error;        /**< 上一周期误差，用于微分计算 */
    float output;           /**< 当前输出值 */
    float deadzone;         /**< 输出死区范围 */
    float Integraldead_zone; /**< 积分累加死区：误差绝对值超过该值时停止积分累加 */
} user_pid_t;

float PID_coculate(user_pid_t *PID);
void PID_INIT(user_pid_t *PID, float Kp, float Ki, float Kd, float out_limit,
              float i_limit, float I_step, float Integraldead_zone, float deadzone);
void PID_Reset_OutLimit(user_pid_t *PID, float new_limit);
void PID_Clear(user_pid_t *PID);
void PID_Set_Error(user_pid_t *PID, float now, float target);
void PID_Set_AngleError(user_pid_t *pid, float current_angle, float target_angle);
float ShortestAngleDelta(float target_angle, float current_angle);

#endif