/**
 * @file tip_protection_controller.cpp
 * @brief TipProtectionController 的倾覆去抖、收腿双环和动作倒计时实现。
 */

#include "tip_protection_controller.hpp"

extern "C"
{
#include "VMC.h"
}

namespace chassis
{

/**
 * @brief 保存倾覆保护依赖引用和离散时序配置。
 * @param[in] dependencies 八个保护动作 PID 和最短腿长参数。
 * @param[in] config 俯仰门槛、触发去抖周期和保护持续周期。
 */
TipProtectionController::TipProtectionController(
    const TipProtectionControllerDependencies& dependencies,
    const TipProtectionConfig& config)
    : dependencies_(dependencies), config_(config)
{
}

/**
 * @brief 按迁移前 Detect→Action 顺序执行一次倾覆保护更新。
 * @param[in] state 本周期只读状态快照。
 * @return 保护接管状态、最后一帧完成事件和收腿命令。
 */
TipProtectionUpdateResult TipProtectionController::Update(
    const ChassisStateSnapshot& state)
{
    UpdateDetection(state.pitch_angle_deg);

    /** 本周期返回给 BalanceController 的保护命令与状态事件。 */
    TipProtectionUpdateResult result{};
    if (remaining_action_cycles_ == 0U)
        return result;

    result.active = true;
    result.command = CalculateProtectiveCommand(state);

    --remaining_action_cycles_;
    result.completed = remaining_action_cycles_ == 0U;
    return result;
}

/**
 * @brief 在保护空闲时累计连续超门槛周期并启动 250 周期动作窗口。
 * @param[in] pitch_angle_deg IMU 原始俯仰角，单位 deg。
 */
void TipProtectionController::UpdateDetection(float pitch_angle_deg)
{
    if (remaining_action_cycles_ != 0U)
        return;

    /** 当前俯仰角是否超出前后对称的倾覆门槛。 */
    const bool threshold_exceeded =
        pitch_angle_deg > config_.pitch_threshold_deg
        || pitch_angle_deg < -config_.pitch_threshold_deg;

    if (!threshold_exceeded)
    {
        consecutive_detection_cycles_ = 0U;
        return;
    }

    if (consecutive_detection_cycles_ < config_.detection_cycles)
        ++consecutive_detection_cycles_;

    if (consecutive_detection_cycles_ >= config_.detection_cycles)
    {
        remaining_action_cycles_ = config_.action_cycles;
        consecutive_detection_cycles_ = 0U;
    }
}

/**
 * @brief 计算一次最短腿长和竖直腿角目标下的保护命令。
 * @param[in] state 左右腿长度、伸缩速度、腿角和腿角速度快照。
 * @return 左右支持力、腿力矩以及清零后的左右轮力矩。
 */
ChassisCommand TipProtectionController::CalculateProtectiveCommand(
    const ChassisStateSnapshot& state)
{
    PID_Set_Error(&dependencies_.left_length_position_pid,
                  state.left_leg.length_m,
                  dependencies_.minimum_leg_length_m);
    PID_Set_Error(&dependencies_.right_length_position_pid,
                  state.right_leg.length_m,
                  dependencies_.minimum_leg_length_m);
    PID_coculate(&dependencies_.left_length_position_pid);
    PID_coculate(&dependencies_.right_length_position_pid);

    PID_Set_Error(&dependencies_.left_length_speed_pid,
                  state.left_leg.length_rate_mps,
                  dependencies_.left_length_position_pid.output);
    PID_Set_Error(&dependencies_.right_length_speed_pid,
                  state.right_leg.length_rate_mps,
                  dependencies_.right_length_position_pid.output);
    PID_coculate(&dependencies_.left_length_speed_pid);
    PID_coculate(&dependencies_.right_length_speed_pid);

    PID_Set_AngleError(&dependencies_.left_angle_position_pid,
                       state.left_leg.leg_angle_rad,
                       PI / 2.0f);
    PID_coculate(&dependencies_.left_angle_position_pid);
    PID_Set_Error(&dependencies_.left_angle_speed_pid,
                  state.left_leg.leg_angular_rate_radps,
                  dependencies_.left_angle_position_pid.output);
    PID_coculate(&dependencies_.left_angle_speed_pid);

    PID_Set_AngleError(&dependencies_.right_angle_position_pid,
                       state.right_leg.leg_angle_rad,
                       PI / 2.0f);
    PID_coculate(&dependencies_.right_angle_position_pid);
    PID_Set_Error(&dependencies_.right_angle_speed_pid,
                  -state.right_leg.leg_angular_rate_radps,
                  -dependencies_.right_angle_position_pid.output);
    PID_coculate(&dependencies_.right_angle_speed_pid);

    /** 与旧 VMC_Chassis_Target 六个赋值保持完全一致的保护命令。 */
    ChassisCommand command{};
    command.left_support_force_n = dependencies_.left_length_speed_pid.output;
    command.left_leg_torque_nm = dependencies_.left_angle_speed_pid.output;
    command.right_support_force_n = dependencies_.right_length_speed_pid.output;
    command.right_leg_torque_nm = -dependencies_.right_angle_speed_pid.output;
    command.left_wheel_torque_nm = 0.0f;
    command.right_wheel_torque_nm = 0.0f;
    return command;
}

} // namespace chassis
