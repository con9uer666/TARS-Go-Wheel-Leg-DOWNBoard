/**
 * @file Self_Righting.h
 * @brief 倒地自起（Self-Righting）状态机控制接口。
 *
 * 提供四阶段自起状态机变量与函数声明：
 *   1) EXTEND: 伸腿到目标腿长；
 *   2) REVERSE_TURN: 反向匀速转，将两腿并齐；
 *   3) SYNC_HIGH_TORQUE: 大力矩匀速转到目标角度；
 *   4) FINISHED: 完成，等待后续指令。
 */

#ifndef SELF_RIGHTING_H
#define SELF_RIGHTING_H

#include "main.h"

#ifndef PI
#define PI 3.14159265358979323846f
#endif

/**
 * @brief 倒地自起流程状态机枚举。
 *
 * 状态流转：
 *   - EXTEND → REVERSE_TURN → SYNC_HIGH_TORQUE → FINISHED
 */
typedef enum
{
	SELF_RIGHTING_STAGE_EXTEND = 0, // 先伸腿到目标腿长
	SELF_RIGHTING_STAGE_REVERSE_TURN,
	SELF_RIGHTING_STAGE_SYNC_HIGH_TORQUE,
	SELF_RIGHTING_STAGE_FINISHED
} SelfRightingStage_t;

/* 自起状态与模式标志 */
extern SelfRightingStage_t g_self_righting_stage;
extern uint8_t g_self_righting_enable;
extern uint8_t g_self_righting_sync_from_stuck;

/* 倒地自起激活标志：1=正在自起或正在播放完成提示音（蜂鸣器被自起独占）
 * Why: 自起期间用户需要听清楚自起阶段的音高节奏区分；错误码蜂鸣器若同时驱动会污染音频
 * How to apply: 由 motor.c 的 NotStanding_NotStairRetract_for_chassis 在自起开始时置1、
 * 完成提示音结束时清0；错误码蜂鸣器代码必须读它，为1时完全不许碰蜂鸣器 */
extern uint8_t g_tip_recovery_active;

/* 倒地自起时腿的转动方向标志位
 *   +1: 保持当前行为（左腿正方向 / 右腿负方向）
 *   -1: 整体翻向（对应车以反面倒地时的自起方向）
 * 作用范围：第一阶段解卡转、第二阶段反向匀速转、第三阶段同步转 —— 三处都乘 dir。 */
extern int8_t g_sr_turn_dir;

/* ===== 可调参数（按用户要求全部做成全局） ===== */
extern float g_sr_l0_reached_tol;
extern float g_sr_l0_ctrl_ramp_rate;
extern float g_sr_l0_ctrl_f0_max;
extern float g_sr_l0_stuck_thresh;

extern float g_sr_extend_unstuck_speed_l;
extern float g_sr_extend_unstuck_speed_r;
extern float g_sr_extend_unstuck_torque_max;
extern float g_sr_extend_unstuck_torque_ramp;
extern float g_sr_extend_passive_damping;
extern float g_sr_extend_passive_damping_torque_max;

extern float g_sr_reverse_speed_l;
extern float g_sr_reverse_speed_r;
extern float g_sr_reverse_torque_max;
extern float g_sr_reverse_torque_ramp;
extern float g_sr_turn_stuck_thresh;
extern float g_sr_align_tol;

extern float g_sr_target_angle_l;
extern float g_sr_target_angle_r;
extern float g_sr_target_angle_tol;
extern float g_sr_sync_speed;
extern float g_sr_sync_fast_speed;
extern float g_sr_sync_slow_speed;
extern float g_sr_sync_torque_max;
extern float g_sr_sync_torque_ramp;

/* 调试/观测变量 */
extern float g_sr_cmd_f_l;
extern float g_sr_cmd_t_l;
extern float g_sr_cmd_f_r;
extern float g_sr_cmd_t_r;

/* 控制接口：
 * - Self_Righting_Reset: 重置状态机和输出记忆；
 * - Self_Righting_Step: 单步执行一次（无循环），需在外部周期调用；
 */
void Self_Righting_Reset(void);
uint8_t Self_Righting_Step(void);

#endif //SELF_RIGHTING_H