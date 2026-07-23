/**
 * @file Self_Righting.h
 * @brief 倒地自起控制器保留的阶段枚举、现场调参量和调试观测符号。
 *
 * 提供四阶段自起状态机和现场调试符号：
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
	SELF_RIGHTING_STAGE_EXTEND = 0,       /**< 第一阶段：伸腿到最大腿长，卡住时主动转腿解卡。 */
	SELF_RIGHTING_STAGE_REVERSE_TURN,    /**< 第二阶段：反向转腿并尝试让左右虚拟腿角并齐。 */
	SELF_RIGHTING_STAGE_SYNC_HIGH_TORQUE,/**< 第三阶段：大力矩同步转腿，直到车体姿态恢复。 */
	SELF_RIGHTING_STAGE_FINISHED         /**< 预留完成阶段；当前退出由姿态去抖直接复位。 */
} SelfRightingStage_t;

/* 自起状态与模式标志 */
extern SelfRightingStage_t g_self_righting_stage; /**< 当前倒地自起动作阶段。 */
extern uint8_t g_self_righting_enable; /**< 自起动作总开关；0 时保持模式判定但腿部命令清零。 */
extern uint8_t g_self_righting_sync_from_stuck; /**< 1 表示第三阶段由“未并齐但双腿卡住”条件进入。 */

/* 倒地自起时腿的转动方向标志位
 *   +1: 保持当前行为（左腿正方向 / 右腿负方向）
 *   -1: 整体翻向（对应车以反面倒地时的自起方向）
 * 作用范围：第一阶段解卡转、第二阶段反向匀速转、第三阶段同步转 —— 三处都乘 dir。 */
extern int8_t g_sr_turn_dir; /**< 三个动作阶段统一使用的转腿方向，取 +1 或 -1。 */

/* ===== 可调参数（按用户要求全部做成全局） ===== */
extern float g_sr_l0_reached_tol; /**< 第一阶段最大腿长到位容差，单位 m。 */
extern float g_sr_l0_ctrl_ramp_rate; /**< 预留伸腿力斜坡速率；当前 PID 路径未使用。 */
extern float g_sr_l0_ctrl_f0_max; /**< 预留伸腿支持力上限，当前 PID 路径未使用。 */
extern float g_sr_l0_stuck_thresh; /**< 腿长卡住检测的长度变化门槛，单位 m。 */

extern float g_sr_extend_unstuck_speed_l; /**< 左腿第一阶段解卡目标角速度绝对值。 */
extern float g_sr_extend_unstuck_speed_r; /**< 右腿第一阶段解卡目标角速度绝对值。 */
extern float g_sr_extend_unstuck_torque_max; /**< 第一阶段解卡腿力矩绝对值上限，单位 N·m。 */
extern float g_sr_extend_unstuck_torque_ramp; /**< 第一阶段解卡腿力矩斜坡速率。 */
extern float g_sr_extend_passive_damping; /**< 预留被动阻尼系数；当前对应控制代码保持禁用。 */
extern float g_sr_extend_passive_damping_torque_max; /**< 预留被动阻尼力矩上限。 */

extern float g_sr_reverse_speed_l; /**< 第二阶段左腿基础目标角速度。 */
extern float g_sr_reverse_speed_r; /**< 第二阶段右腿基础目标角速度。 */
extern float g_sr_reverse_torque_max; /**< 第二阶段腿力矩绝对值上限，单位 N·m。 */
extern float g_sr_reverse_torque_ramp; /**< 第二阶段腿力矩斜坡速率。 */
extern float g_sr_turn_stuck_thresh; /**< 第二阶段转腿卡住检测门槛。 */
extern float g_sr_align_tol; /**< 左右虚拟腿角并齐容差，单位 rad。 */

extern float g_sr_target_angle_l; /**< 预留左腿目标角，当前第三阶段不使用到位停止。 */
extern float g_sr_target_angle_r; /**< 预留右腿目标角，当前第三阶段不使用到位停止。 */
extern float g_sr_target_angle_tol; /**< 预留目标角容差，当前第三阶段不使用。 */
extern float g_sr_sync_speed; /**< 第三阶段左右腿同步转动的基础角速度绝对值。 */
extern float g_sr_sync_fast_speed; /**< 预留差速策略快侧速度，当前未参与命令。 */
extern float g_sr_sync_slow_speed; /**< 预留差速策略慢侧速度，当前未参与命令。 */
extern float g_sr_sync_torque_max; /**< 第三阶段腿力矩绝对值上限，单位 N·m。 */
extern float g_sr_sync_torque_ramp; /**< 第三阶段腿力矩斜坡速率。 */

/* 调试/观测变量 */
extern float g_sr_cmd_f_l; /**< 最近一次自起左腿支持力命令，单位 N。 */
extern float g_sr_cmd_t_l; /**< 最近一次自起左腿虚拟力矩命令，单位 N·m。 */
extern float g_sr_cmd_f_r; /**< 最近一次自起右腿支持力命令，单位 N。 */
extern float g_sr_cmd_t_r; /**< 最近一次自起右腿虚拟力矩命令，单位 N·m。 */

#endif //SELF_RIGHTING_H
