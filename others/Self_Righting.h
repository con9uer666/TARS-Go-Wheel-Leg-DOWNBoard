#ifndef SELF_RIGHTING_H
#define SELF_RIGHTING_H

#include "main.h"

/*
 * 倒地自起流程状态机：
 * 1) EXTEND: 先伸腿到目标腿长；
 * 2) REVERSE_TURN: 到腿长后先反向匀速转，尝试两腿并齐；
 * 3) SYNC_HIGH_TORQUE: 大力矩匀速转到目标角度；
 * 4) FINISHED: 两腿都到目标角度，预留给用户补逻辑。
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

/* ===== 可调参数（按用户要求全部做成全局） ===== */
extern float g_sr_target_l0;
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