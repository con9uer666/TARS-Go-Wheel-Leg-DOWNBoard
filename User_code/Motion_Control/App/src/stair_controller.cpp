/**
 * @file stair_controller.cpp
 * @brief 上台阶伸腿、收腿及恢复过程的一体化 C++ 控制器实现。
 */

#include "stair_controller.hpp"

#include <cmath>

extern "C"
{
#include "chassis_behavior_tree.h"
#include "Leg_Control.h"
}

namespace chassis
{

/**
 * @brief 保存上台阶控制所需的外部对象引用和动作参数副本。
 * @param[in] dependencies 生命周期必须覆盖控制器的八个 PID、两侧 VMC 和腿长参数引用。
 * @param[in] config 伸腿目标、收腿目标、到位阈值和持续周期配置。
 */
StairController::StairController(const StairControllerDependencies& dependencies,
                                 const StairControllerConfig& config)
    : dependencies_(dependencies), config_(config)
{
}

/**
 * @brief 根据旧模式标志同步控制器内部阶段，保持灰度迁移期间的模式含义不变。
 * @param[in] signals 本周期锁存的 start_mode 和 upstares_mode 兼容信号。
 *
 * upstares_mode 仍是尚未迁移模块能够观察和修改的兼容标志，因此它在本阶段
 * 仍具有阶段选择权。控制器自行完成伸腿时也会切换内部阶段，并由任务同步该标志。
 */
void StairController::SynchronizeLegacyPhase(const LegacyModeSignals& signals)
{
    if (signals.stair_retract_mode == 1U && phase_ != StairPhase::Retract)
    {
        phase_ = StairPhase::Retract;
        PrepareRetractTurnState();
        return;
    }

    if (signals.start_mode == 2U
        && signals.stair_retract_mode == 0U
        && phase_ != StairPhase::Extend)
    {
        phase_ = StairPhase::Extend;
    }
}

/**
 * @brief 执行当前上台阶内部阶段的一次 2 ms 控制更新。
 * @param[in] state 本周期 VMC 与车身状态的只读快照。
 * @param[in] previous_command 上周期最终命令；伸腿阶段沿用其中的左右轮力矩。
 * @return 本周期完整命令，以及是否刚进入收腿阶段或完成整个动作。
 */
StairUpdateResult StairController::Update(const ChassisStateSnapshot& state,
                                          const ChassisCommand& previous_command)
{
    switch (phase_)
    {
    case StairPhase::Extend:
        return UpdateExtend(state, previous_command);

    case StairPhase::Retract:
        return UpdateRetract(state);

    case StairPhase::Inactive:
    default:
    {
        /** 防御性返回值；正常调度会在调用 Update() 前先同步出有效阶段。 */
        StairUpdateResult result{};
        result.command = previous_command;
        return result;
    }
    }
}

/**
 * @brief 获取上台阶控制器当前内部阶段。
 * @return Inactive 表示空闲，Extend 表示伸腿顶台阶，Retract 表示收腿恢复。
 */
StairPhase StairController::phase() const
{
    return phase_;
}

/**
 * @brief 执行伸腿顶台阶阶段的腿长和腿角双环控制。
 * @param[in] state 本周期左右腿长度、长度速度、腿角和腿角速度快照。
 * @param[in] previous_command 上周期命令，用于严格保留旧逻辑未覆盖的轮力矩。
 * @return 四个腿部命令，以及双腿到位后产生的收腿阶段进入事件。
 */
StairUpdateResult StairController::UpdateExtend(
    const ChassisStateSnapshot& state,
    const ChassisCommand& previous_command)
{
    /** 左腿腿长位置环 PID 的本地别名，引用构造时注入的旧 PID 实例。 */
    user_pid_t& left_length_position_pid = dependencies_.left_length_position_pid;
    /** 左腿腿长速度环 PID 的本地别名。 */
    user_pid_t& left_length_velocity_pid = dependencies_.left_length_velocity_pid;
    /** 右腿腿长位置环 PID 的本地别名。 */
    user_pid_t& right_length_position_pid = dependencies_.right_length_position_pid;
    /** 右腿腿长速度环 PID 的本地别名。 */
    user_pid_t& right_length_velocity_pid = dependencies_.right_length_velocity_pid;
    /** 左腿角度位置环 PID 的本地别名。 */
    user_pid_t& left_angle_position_pid = dependencies_.left_angle_position_pid;
    /** 左腿角度速度环 PID 的本地别名。 */
    user_pid_t& left_angle_velocity_pid = dependencies_.left_angle_velocity_pid;
    /** 右腿角度位置环 PID 的本地别名。 */
    user_pid_t& right_angle_position_pid = dependencies_.right_angle_position_pid;
    /** 右腿角度速度环 PID 的本地别名。 */
    user_pid_t& right_angle_velocity_pid = dependencies_.right_angle_velocity_pid;

    PID_Set_Error(&left_length_position_pid, state.left_leg.length_m,
                  dependencies_.maximum_leg_length_m);
    PID_Set_Error(&right_length_position_pid, state.right_leg.length_m,
                  dependencies_.maximum_leg_length_m);
    PID_coculate(&left_length_position_pid);
    PID_coculate(&right_length_position_pid);

    PID_Set_Error(&left_length_velocity_pid, state.left_leg.length_rate_mps,
                  left_length_position_pid.output);
    PID_Set_Error(&right_length_velocity_pid, state.right_leg.length_rate_mps,
                  right_length_position_pid.output);
    PID_coculate(&left_length_velocity_pid);
    PID_coculate(&right_length_velocity_pid);

    PID_Set_AngleError(&left_angle_position_pid, state.left_leg.leg_angle_rad,
                       config_.extend_left_angle_rad);
    PID_coculate(&left_angle_position_pid);
    PID_Set_Error(&left_angle_velocity_pid, state.left_leg.leg_angular_rate_radps,
                  left_angle_position_pid.output);
    PID_coculate(&left_angle_velocity_pid);

    PID_Set_AngleError(&right_angle_position_pid, state.right_leg.leg_angle_rad,
                       config_.extend_right_angle_rad);
    PID_coculate(&right_angle_position_pid);
    PID_Set_Error(&right_angle_velocity_pid, -state.right_leg.leg_angular_rate_radps,
                  -right_angle_position_pid.output);
    PID_coculate(&right_angle_velocity_pid);

    /** 伸腿阶段返回值；先复制旧命令，以保留迁移前未被该阶段覆盖的轮力矩。 */
    StairUpdateResult result{};
    result.command = previous_command;
    result.command.left_support_force_n = left_length_velocity_pid.output;
    result.command.left_leg_torque_nm = left_angle_velocity_pid.output;
    result.command.right_support_force_n = right_length_velocity_pid.output;
    result.command.right_leg_torque_nm = -right_angle_velocity_pid.output;

    if (L_Leg_State == 0U
        && std::fabs(left_length_position_pid.error) <= config_.extend_error_tolerance
        && std::fabs(left_angle_position_pid.error) <= config_.extend_error_tolerance)
    {
        ++L_Ready_Count;
    }
    if (L_Leg_State == 0U && L_Ready_Count >= config_.extend_ready_cycles)
    {
        L_Leg_State = 2U;
        L_Ready_Count = 0U;
    }

    if (R_Leg_State == 0U
        && std::fabs(right_length_position_pid.error) <= config_.extend_error_tolerance
        && std::fabs(right_angle_position_pid.error) <= config_.extend_error_tolerance)
    {
        ++R_Ready_Count;
    }
    if (R_Leg_State == 0U && R_Ready_Count >= config_.extend_ready_cycles)
    {
        R_Leg_State = 2U;
        R_Ready_Count = 0U;
    }

    if (L_Leg_State == 2U && R_Leg_State == 2U)
    {
        phase_ = StairPhase::Retract;
        PrepareRetractTurnState();
        result.entered_retract_phase = true;
    }

    return result;
}

/**
 * @brief 执行收腿、转回竖直姿态和完成等待阶段。
 * @param[in] state 本周期左右腿长度、长度速度和转角状态快照。
 * @return 本周期六维命令，以及 150 周期等待结束后的完成事件。
 */
StairUpdateResult StairController::UpdateRetract(const ChassisStateSnapshot& state)
{
    /** 左腿腿长位置环 PID 的本地别名。 */
    user_pid_t& left_length_position_pid = dependencies_.left_length_position_pid;
    /** 左腿腿长速度环 PID 的本地别名。 */
    user_pid_t& left_length_velocity_pid = dependencies_.left_length_velocity_pid;
    /** 右腿腿长位置环 PID 的本地别名。 */
    user_pid_t& right_length_position_pid = dependencies_.right_length_position_pid;
    /** 右腿腿长速度环 PID 的本地别名。 */
    user_pid_t& right_length_velocity_pid = dependencies_.right_length_velocity_pid;

    PID_Set_Error(&left_length_position_pid, state.left_leg.length_m,
                  config_.retract_target_length_m);
    PID_Set_Error(&right_length_position_pid, state.right_leg.length_m,
                  config_.retract_target_length_m);
    PID_coculate(&left_length_position_pid);
    PID_coculate(&right_length_position_pid);

    PID_Set_Error(&left_length_velocity_pid, state.left_leg.length_rate_mps,
                  left_length_position_pid.output);
    PID_Set_Error(&right_length_velocity_pid, state.right_leg.length_rate_mps,
                  right_length_position_pid.output);
    PID_coculate(&left_length_velocity_pid);
    PID_coculate(&right_length_velocity_pid);

    /** 左腿转角辅助函数输出的本周期虚拟腿力矩，单位 N·m。 */
    float left_leg_torque_nm = 0.0f;
    /** 右腿转角辅助函数输出的本周期虚拟腿力矩，单位 N·m。 */
    float right_leg_torque_nm = 0.0f;
    /** 左腿角度本周期是否进入辅助函数定义的到位范围。 */
    int left_angle_near = 0;
    /** 右腿角度本周期是否进入辅助函数定义的到位范围。 */
    int right_angle_near = 0;

    if (L_Leg_State >= 1U)
    {
        left_angle_near = turn_ctrl_with_stuck_flip(
            &dependencies_.left_vmc, 0, config_.retract_left_angle_rad,
            &dependencies_.left_angle_position_pid,
            &dependencies_.left_angle_velocity_pid,
            &L_stair_sub, &L_sub_dwell,
            &L_rev_dir, &L_rev_long_remain, &L_rev_traveled,
            &left_leg_torque_nm);
    }
    if (R_Leg_State >= 1U)
    {
        right_angle_near = turn_ctrl_with_stuck_flip(
            &dependencies_.right_vmc, 1, config_.retract_right_angle_rad,
            &dependencies_.right_angle_position_pid,
            &dependencies_.right_angle_velocity_pid,
            &R_stair_sub, &R_sub_dwell,
            &R_rev_dir, &R_rev_long_remain, &R_rev_traveled,
            &right_leg_torque_nm);
    }

    /** 收腿阶段完整命令；值初始化同时保持左右轮力矩严格为零。 */
    StairUpdateResult result{};
    result.command.left_support_force_n = left_length_velocity_pid.output;
    result.command.left_leg_torque_nm = left_leg_torque_nm;
    result.command.right_support_force_n = right_length_velocity_pid.output;
    result.command.right_leg_torque_nm = right_leg_torque_nm;

    if (L_Leg_State == 0U
        && std::fabs(left_length_position_pid.error) <= config_.retract_length_tolerance_m)
    {
        ++L_Ready_Count;
    }
    else if (L_Leg_State == 0U)
    {
        L_Ready_Count = 0U;
    }
    if (L_Leg_State == 0U && L_Ready_Count >= config_.retract_ready_cycles)
    {
        L_Leg_State = 1U;
        L_Ready_Count = 0U;
        L_stair_sub = STAIR_SUB_TURN_FWD;
        L_sub_dwell = 0;
        leg_turn_stuck_reset(&dependencies_.left_vmc);
    }

    if (R_Leg_State == 0U
        && std::fabs(right_length_position_pid.error) <= config_.retract_length_tolerance_m)
    {
        ++R_Ready_Count;
    }
    else if (R_Leg_State == 0U)
    {
        R_Ready_Count = 0U;
    }
    if (R_Leg_State == 0U && R_Ready_Count >= config_.retract_ready_cycles)
    {
        R_Leg_State = 1U;
        R_Ready_Count = 0U;
        R_stair_sub = STAIR_SUB_TURN_FWD;
        R_sub_dwell = 0;
        leg_turn_stuck_reset(&dependencies_.right_vmc);
    }

    if (L_Leg_State == 1U && left_angle_near != 0)
    {
        ++L_Ready_Count;
    }
    else if (L_Leg_State == 1U)
    {
        L_Ready_Count = 0U;
    }
    if (L_Leg_State == 1U && L_Ready_Count >= config_.retract_ready_cycles)
    {
        L_Leg_State = 2U;
        L_Ready_Count = 0U;
    }

    if (R_Leg_State == 1U && right_angle_near != 0)
    {
        ++R_Ready_Count;
    }
    else if (R_Leg_State == 1U)
    {
        R_Ready_Count = 0U;
    }
    if (R_Leg_State == 1U && R_Ready_Count >= config_.retract_ready_cycles)
    {
        R_Leg_State = 2U;
        R_Ready_Count = 0U;
    }

    if (L_Leg_State == 2U && R_Leg_State == 2U)
    {
        ++completion_dwell_count_;
        if (completion_dwell_count_ >= config_.completion_dwell_cycles)
        {
            result.completed = true;
            ResetAfterCompletion();
        }
    }

    return result;
}

/**
 * @brief 复位收腿转角辅助状态，并把双腿阶段恢复为“先收腿”。
 *
 * 该顺序与旧 Upstair_NotStairRetract() 完成时一致，可避免自起动作或上一次
 * 上台阶动作遗留的卡住检测计数直接影响新一轮收腿。
 */
void StairController::PrepareRetractTurnState()
{
    L_Leg_State = 0U;
    R_Leg_State = 0U;
    L_stair_sub = STAIR_SUB_TURN_FWD;
    R_stair_sub = STAIR_SUB_TURN_FWD;
    L_sub_dwell = 0;
    R_sub_dwell = 0;
    leg_turn_stuck_reset(&dependencies_.left_vmc);
    leg_turn_stuck_reset(&dependencies_.right_vmc);
}

/**
 * @brief 清除本次动作的内部完成等待状态，使控制器返回空闲阶段。
 */
void StairController::ResetAfterCompletion()
{
    completion_dwell_count_ = 0U;
    L_Ready_Count = 0U;
    R_Ready_Count = 0U;
    phase_ = StairPhase::Inactive;
}

} // namespace chassis
