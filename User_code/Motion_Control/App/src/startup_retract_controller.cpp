/**
 * @file startup_retract_controller.cpp
 * @brief StartupRetractController 的倒地自起兼容与收腿恢复实现。
 */

#include "startup_retract_controller.hpp"

#include <cmath>

extern "C"
{
#include "chassis_behavior_tree.h"
#include "Leg_Control.h"
}

namespace chassis
{

/**
 * @brief 保存起立恢复所需的外部依赖和控制参数。
 * @param[in] dependencies 八个 PID、两侧 VMC、旧命令目标和自起函数引用。
 * @param[in] config 腿长、腿角目标及到位判定参数。
 */
StartupRetractController::StartupRetractController(
    const StartupRetractControllerDependencies& dependencies,
    const StartupRetractControllerConfig& config)
    : dependencies_(dependencies), config_(config)
{
}

/**
 * @brief 先判定倒地自起是否接管，否则执行收腿恢复。
 * @param[in] state 本周期解算完成的只读底盘状态快照。
 * @return 自起或收腿分支产生的完整命令和状态事件。
 */
StartupRetractUpdateResult StartupRetractController::Update(
    const ChassisStateSnapshot& state)
{
    if (dependencies_.detect_self_righting() != 0U)
        return UpdateSelfRighting();

    return UpdateRetract(state);
}

/**
 * @brief 执行旧倒地自起单步动作并立即捕获其命令。
 * @return self_righting_active 为 true 的命令结果。
 */
StartupRetractUpdateResult StartupRetractController::UpdateSelfRighting()
{
    dependencies_.execute_self_righting();

    /** 旧自起动作已写入命令后的本周期返回值。 */
    StartupRetractUpdateResult result{};
    result.command = CaptureLegacyCommand();
    result.self_righting_active = true;
    return result;
}

/**
 * @brief 执行腿长收缩、腿角转正和到位状态转换。
 * @param[in] state 本周期左右腿长度、长度速度、腿角和腿角速度。
 * @return 轮力矩为零的收腿命令和恢复完成事件。
 */
StartupRetractUpdateResult StartupRetractController::UpdateRetract(
    const ChassisStateSnapshot& state)
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
                  config_.target_length_m);
    PID_Set_Error(&right_length_position_pid, state.right_leg.length_m,
                  config_.target_length_m);
    PID_coculate(&left_length_position_pid);
    PID_coculate(&right_length_position_pid);

    PID_Set_Error(&left_length_velocity_pid, state.left_leg.length_rate_mps,
                  left_length_position_pid.output);
    PID_Set_Error(&right_length_velocity_pid, state.right_leg.length_rate_mps,
                  right_length_position_pid.output);
    PID_coculate(&left_length_velocity_pid);
    PID_coculate(&right_length_velocity_pid);

    /** 左腿卡住反绕辅助器输出的本周期虚拟腿力矩，单位 N·m。 */
    float left_leg_torque_nm = 0.0f;
    /** 右腿卡住反绕辅助器输出的本周期虚拟腿力矩，单位 N·m。 */
    float right_leg_torque_nm = 0.0f;
    /** 左腿角度是否进入旧辅助函数定义的到位范围。 */
    int left_angle_near = 0;
    /** 右腿角度是否进入旧辅助函数定义的到位范围。 */
    int right_angle_near = 0;

    if (L_Leg_State >= 1U)
    {
        left_angle_near = turn_ctrl_with_stuck_flip(
            &dependencies_.left_vmc, 0, config_.left_target_angle_rad,
            &dependencies_.left_angle_position_pid,
            &dependencies_.left_angle_velocity_pid,
            &L_stair_sub, &L_sub_dwell,
            &L_rev_dir, &L_rev_long_remain, &L_rev_traveled,
            &left_leg_torque_nm);
    }
    if (R_Leg_State >= 1U)
    {
        right_angle_near = turn_ctrl_with_stuck_flip(
            &dependencies_.right_vmc, 1, config_.right_target_angle_rad,
            &dependencies_.right_angle_position_pid,
            &dependencies_.right_angle_velocity_pid,
            &R_stair_sub, &R_sub_dwell,
            &R_rev_dir, &R_rev_long_remain, &R_rev_traveled,
            &right_leg_torque_nm);
    }

    if (L_Leg_State == 0U
        && std::fabs(left_length_position_pid.error) <= config_.length_error_tolerance_m)
    {
        ++L_Ready_Count;
    }
    if (L_Leg_State == 0U && L_Ready_Count >= config_.ready_cycles)
    {
        L_Leg_State = 1U;
        L_Ready_Count = 0U;
        L_stair_sub = STAIR_SUB_TURN_FWD;
        L_sub_dwell = 0;
        leg_turn_stuck_reset(&dependencies_.left_vmc);
    }

    if (R_Leg_State == 0U
        && std::fabs(right_length_position_pid.error) <= config_.length_error_tolerance_m)
    {
        ++R_Ready_Count;
    }
    if (R_Leg_State == 0U && R_Ready_Count >= config_.ready_cycles)
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
    if (L_Leg_State == 1U && L_Ready_Count >= config_.ready_cycles)
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
    if (R_Leg_State == 1U && R_Ready_Count >= config_.ready_cycles)
    {
        R_Leg_State = 2U;
        R_Ready_Count = 0U;
    }

    /** 收腿阶段的完整返回值；零初始化使左右轮力矩保持为零。 */
    StartupRetractUpdateResult result{};
    result.command.left_support_force_n = left_length_velocity_pid.output;
    result.command.left_leg_torque_nm = left_leg_torque_nm;
    result.command.right_support_force_n = right_length_velocity_pid.output;
    result.command.right_leg_torque_nm = right_leg_torque_nm;
    result.retract_control_active = true;

    if (L_Leg_State == 2U && R_Leg_State == 2U)
    {
        result.completed = true;
        L_Leg_State = 0U;
        R_Leg_State = 0U;
    }

    return result;
}

/**
 * @brief 把旧倒地自起的 VMC 目标复制为新命令类型。
 * @return 与 VMC_Chassis_Target 六个字段一一对应的命令。
 */
ChassisCommand StartupRetractController::CaptureLegacyCommand() const
{
    /** 旧自起动作已产生的完整 VMC 映射前命令。 */
    ChassisCommand command{};
    command.left_support_force_n = dependencies_.legacy_command_target.L_F0;
    command.left_leg_torque_nm = dependencies_.legacy_command_target.L_T;
    command.right_support_force_n = dependencies_.legacy_command_target.R_F0;
    command.right_leg_torque_nm = dependencies_.legacy_command_target.R_T;
    command.left_wheel_torque_nm = dependencies_.legacy_command_target.L_Wheel_Torque;
    command.right_wheel_torque_nm = dependencies_.legacy_command_target.R_Wheel_Torque;
    return command;
}

} // namespace chassis
