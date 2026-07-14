/**
 * @file sit_controller.cpp
 * @brief SitController 的 C++ 实现及旧调试符号兼容层。
 */

#include "sit_controller.hpp"

#include <cmath>

extern "C"
{
#include "arm_math.h"
}

/**
 * @brief 旧 C 模块使用的“坐地下次需要重新初始化”兼容标志。
 *
 * Tip_Protect.c 仍会把它置 1。SitController 每周期读取该标志，并在完成
 * 斜坡初始化后清零。保留此符号可避免本阶段修改倾覆保护代码。
 */
extern "C" std::uint8_t sit_first_entry = 1;

/** @brief 保留给调试器观察的坐地动作累计周期计数。 */
extern "C" std::uint16_t sit_debug_counter = 0;

/** @brief 保留给调试器观察的坐地斜坡完成标志，1 表示四条斜坡均到位。 */
extern "C" std::uint8_t sit_ramp_done = 0;

namespace chassis
{

/**
 * @brief 保存 PID 依赖引用和坐地参数副本。
 * @param[in] dependencies 外部提供的八个 PID 与重力补偿量引用。
 * @param[in] config 坐地目标、斜坡和补偿配置。
 */
SitController::SitController(const SitControllerDependencies& dependencies,
                             const SitControllerConfig& config)
    : dependencies_(dependencies), config_(config)
{
}

/**
 * @brief 以当前腿姿态为起点初始化四条坐地目标斜坡。
 * @param[in] state 进入坐地动作时的只读状态快照。
 */
void SitController::InitializeRamps(const ChassisStateSnapshot& state)
{
    rampInit(&left_length_ramp_, state.left_leg.length_m,
             config_.left_target_length_m, config_.ramp_duration_s,
             config_.sample_period_s);
    rampInit(&right_length_ramp_, state.right_leg.length_m,
             config_.right_target_length_m, config_.ramp_duration_s,
             config_.sample_period_s);
    rampInit(&left_angle_ramp_, state.left_leg.leg_angle_rad,
             config_.left_target_angle_rad, config_.ramp_duration_s,
             config_.sample_period_s);
    rampInit(&right_angle_ramp_, state.right_leg.leg_angle_rad,
             config_.right_target_angle_rad, config_.ramp_duration_s,
             config_.sample_period_s);

    ramps_initialized_ = true;
    sit_first_entry = 0;
}

/**
 * @brief 推进四条坐地斜坡，并同步旧调试完成标志 sit_ramp_done。
 */
void SitController::UpdateRamps()
{
    rampIterate(&left_length_ramp_);
    rampIterate(&right_length_ramp_);
    rampIterate(&left_angle_ramp_);
    rampIterate(&right_angle_ramp_);

    /** 左腿长度斜坡与最终目标的绝对误差，单位 m。 */
    const float left_length_error =
        std::fabs(left_length_ramp_.currentValue - config_.left_target_length_m);
    /** 右腿长度斜坡与最终目标的绝对误差，单位 m。 */
    const float right_length_error =
        std::fabs(right_length_ramp_.currentValue - config_.right_target_length_m);
    /** 左腿角度斜坡与最终目标的绝对误差，单位 rad。 */
    const float left_angle_error =
        std::fabs(left_angle_ramp_.currentValue - config_.left_target_angle_rad);
    /** 右腿角度斜坡与最终目标的绝对误差，单位 rad。 */
    const float right_angle_error =
        std::fabs(right_angle_ramp_.currentValue - config_.right_target_angle_rad);

    ramps_finished_ = left_length_error < config_.target_tolerance
                   && right_length_error < config_.target_tolerance
                   && left_angle_error < config_.target_tolerance
                   && right_angle_error < config_.target_tolerance;
    sit_ramp_done = ramps_finished_ ? 1U : 0U;
}

/**
 * @brief 执行腿长和腿角双环 PID，生成本周期坐地命令。
 * @param[in] state 本周期只读状态快照。
 * @return 包含左右 F0/T 和轮力矩的映射前命令。
 */
ChassisCommand SitController::CalculateCommand(const ChassisStateSnapshot& state)
{
    /** 左腿腿长位置环 PID 的本地别名，引用构造时注入的实例。 */
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
                  left_length_ramp_.currentValue);
    PID_coculate(&left_length_position_pid);
    PID_Set_Error(&left_length_velocity_pid, state.left_leg.length_rate_mps,
                  left_length_position_pid.output);
    PID_coculate(&left_length_velocity_pid);

    PID_Set_Error(&right_length_position_pid, state.right_leg.length_m,
                  right_length_ramp_.currentValue);
    PID_coculate(&right_length_position_pid);
    PID_Set_Error(&right_length_velocity_pid, state.right_leg.length_rate_mps,
                  right_length_position_pid.output);
    PID_coculate(&right_length_velocity_pid);

    PID_Set_AngleError(&left_angle_position_pid, state.left_leg.leg_angle_rad,
                       left_angle_ramp_.currentValue);
    PID_coculate(&left_angle_position_pid);
    PID_Set_Error(&left_angle_velocity_pid, state.left_leg.leg_angular_rate_radps,
                  left_angle_position_pid.output);
    PID_coculate(&left_angle_velocity_pid);

    PID_Set_AngleError(&right_angle_position_pid, state.right_leg.leg_angle_rad,
                       right_angle_ramp_.currentValue);
    PID_coculate(&right_angle_position_pid);
    PID_Set_Error(&right_angle_velocity_pid, -state.right_leg.leg_angular_rate_radps,
                  -right_angle_position_pid.output);
    PID_coculate(&right_angle_velocity_pid);

    /** 左腿当前姿态对应的重力补偿分母 cos(b_phi0)。 */
    const float left_gravity_projection = arm_cos_f32(state.left_leg.body_angle_rad);
    /** 右腿当前姿态对应的重力补偿分母 cos(b_phi0)。 */
    const float right_gravity_projection = arm_cos_f32(state.right_leg.body_angle_rad);

    /** 本周期将返回给 Motor_task 的完整 VMC 映射前命令。 */
    ChassisCommand command{};
    command.left_support_force_n = left_length_velocity_pid.output
        + (dependencies_.gravity_compensation_load / left_gravity_projection)
        * config_.gravity_compensation_ratio;
    command.left_leg_torque_nm = left_angle_velocity_pid.output;
    command.right_support_force_n = right_length_velocity_pid.output
        + (dependencies_.gravity_compensation_load / right_gravity_projection)
        * config_.gravity_compensation_ratio;
    command.right_leg_torque_nm = -right_angle_velocity_pid.output;
    command.left_wheel_torque_nm = config_.wheel_hold_torque_nm;
    command.right_wheel_torque_nm = -config_.wheel_hold_torque_nm;
    return command;
}

/**
 * @brief 结束本次坐地动作，并要求下次进入时重新初始化斜坡起点。
 */
void SitController::PrepareNextEntry()
{
    ramps_initialized_ = false;
    sit_first_entry = 1;
}

/**
 * @brief 执行一次坐地动作更新。
 * @param[in] state 本周期只读底盘状态快照。
 * @param[in] sit_requested true 表示继续坐地，false 表示本周期计算后退出。
 * @return 本周期坐地命令以及是否请求切换到起立前收腿模式。
 */
SitUpdateResult SitController::Update(const ChassisStateSnapshot& state,
                                      bool sit_requested)
{
    if (!ramps_initialized_ || sit_first_entry != 0U)
        InitializeRamps(state);

    ++execution_cycle_count_;
    sit_debug_counter = execution_cycle_count_;

    UpdateRamps();

    /** 同时携带执行命令和模式切换请求的坐地更新结果。 */
    SitUpdateResult result{};
    result.command = CalculateCommand(state);
    result.request_startup_retract = !sit_requested;

    if (result.request_startup_retract)
        PrepareNextEntry();

    return result;
}

} // namespace chassis
