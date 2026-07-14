/**
 * @file balance_controller.cpp
 * @brief BalanceController 和 StepHitDetector 的 C++ 实现。
 */

#include "balance_controller.hpp"

#include <cmath>

extern "C"
{
#include "chassis_behavior_tree.h"
#include "anti_split_control.h"
#include "chassis_height_control.h"
#include "Tip_Protect.h"
#include "PowerCtrl.h"
#include "Board2Board.h"
#include "arm_math.h"
}

/**
 * @brief 自动上台阶触发使能，默认关闭。
 *
 * 0 时 StepHitDetector 仍更新命中计数、长腿解禁延时和冷却状态，
 * 但不向 BalanceController 产生自动 Stair 请求。
 */
extern "C" std::uint8_t automatic_stair_climb_enable = 0U;

namespace chassis
{

/**
 * @brief 保存磕台阶检测依赖和参数副本。
 * @param[in] dependencies 腿长、控制频率、冷却计数和左右支持力引用。
 * @param[in] config 力矩阈值、腿长余量、命中计数和解禁延时参数。
 */
StepHitDetector::StepHitDetector(const StepHitDetectorDependencies& dependencies,
                                 const StepHitDetectorConfig& config)
    : dependencies_(dependencies), config_(config)
{
}

/**
 * @brief 更新长腿解禁延时、冷却、碰撞条件和命中计数。
 * @param[in] state 本周期左右腿长和实际虚拟腿力矩快照。
 * @param[in] input 腿长档位、旧模式快照和自动触发开关。
 * @return 检测达到阈值且自动触发已启用时返回 true。
 */
bool StepHitDetector::Update(const ChassisStateSnapshot& state,
                             const StepHitDetectorInput& input)
{
    /** 切换到长腿后的解禁延时，等价于旧 motor_HZ/4 整数运算。 */
    const int long_leg_arm_delay_target =
        static_cast<int>(dependencies_.motor_frequency_hz
                         / config_.long_leg_arm_delay_divisor);
    /** 一次检测成功或离地后的 1 s 冷却周期数。 */
    const int cooldown_target = static_cast<int>(dependencies_.motor_frequency_hz);

    if (input.target_leg_state == 1U && previous_target_leg_state_ != 1U)
        long_leg_arm_delay_cycles_ = long_leg_arm_delay_target;

    if (input.target_leg_state != 1U)
    {
        long_leg_arm_delay_cycles_ = 0;
    }
    else if (long_leg_arm_delay_cycles_ > 0)
    {
        --long_leg_arm_delay_cycles_;
    }
    previous_target_leg_state_ = input.target_leg_state;

    /** 左腿实际虚拟腿力矩绝对值，单位 N·m。 */
    const float left_leg_torque_magnitude = std::fabs(state.left_leg.actual_leg_torque_nm);
    /** 右腿实际虚拟腿力矩绝对值，单位 N·m。 */
    const float right_leg_torque_magnitude = std::fabs(state.right_leg.actual_leg_torque_nm);
    /** 只有腿长贴近最大腿长时才允许识别磕台阶，单位 m。 */
    const float leg_length_threshold =
        dependencies_.maximum_leg_length_m - config_.maximum_length_margin_m;

    /** 左腿同时满足力矩和腿长条件的命中标志。 */
    const bool left_step_hit =
        left_leg_torque_magnitude > config_.leg_torque_threshold_nm
        && state.left_leg.length_m > leg_length_threshold;
    /** 右腿同时满足力矩和腿长条件的命中标志。 */
    const bool right_step_hit =
        right_leg_torque_magnitude > config_.leg_torque_threshold_nm
        && state.right_leg.length_m > leg_length_threshold;

    if (dependencies_.cooldown_cycles > 0)
        --dependencies_.cooldown_cycles;

    /** 命中计数饱和上限，保留旧 required_hit_cycles*2 行为。 */
    const std::uint16_t hit_count_limit =
        static_cast<std::uint16_t>(config_.required_hit_cycles)
        * config_.count_saturation_multiple;

    if (dependencies_.cooldown_cycles == 0
        && long_leg_arm_delay_cycles_ == 0
        && left_step_hit
        && right_step_hit
        && input.target_leg_state == 1U
        && input.mode_signals.start_mode == 1U
        && input.mode_signals.stair_retract_mode == 0U)
    {
        if (hit_count_ < hit_count_limit)
            ++hit_count_;
    }
    else if (hit_count_ > 0U)
    {
        --hit_count_;
    }

    /** 本周期是否达到检测阈值；开关关闭时仍正常进入冷却。 */
    const bool detected = hit_count_ >= config_.required_hit_cycles;
    if (detected)
    {
        hit_count_ = 0U;
        dependencies_.cooldown_cycles = cooldown_target;
    }

    previous_left_ground_support_force_n_ = dependencies_.left_ground_support_force_n;
    previous_right_ground_support_force_n_ = dependencies_.right_ground_support_force_n;
    return detected && input.automatic_trigger_enabled;
}

/**
 * @brief 保存平衡控制依赖，并构造其内部磕台阶检测器。
 * @param[in] dependencies 旧命令适配区、腿长/横滚 PID、重力量和 Stair 请求引用。
 */
BalanceController::BalanceController(const BalanceControllerDependencies& dependencies)
    : dependencies_(dependencies),
      step_hit_detector_(dependencies.step_hit_dependencies)
{
}

/**
 * @brief 执行倾覆保护判定或完整的正常平衡控制链。
 * @param[in] state 本周期已锁存的腿部、车体和轮端状态。
 * @param[in] input 腿长档位、坐地请求、自动 Stair 开关和旧模式快照。
 * @return 最终命令、保护/跳跃状态和模式转换请求。
 */
BalanceUpdateResult BalanceController::Update(const ChassisStateSnapshot& state,
                                              const BalanceControlInput& input)
{
    /** 本周期将返回给 Motor_task 的命令和模式事件。 */
    BalanceUpdateResult result{};

    if (Tip_Protect_Detect() != 0U)
    {
        Tip_Protect_Action();
        result.command = CaptureLegacyCommand();
        result.tip_protection_active = true;
        return result;
    }

    Error_Calculate();
    Roll_Comp();
    Leg_L0_Control();
    LQR_Update_K();
    PowerCtrl();
    LQR_calculate();

    /** LQR 虚拟腿力矩叠加防劈叉、离心补偿和小陀螺归中后的左腿命令。 */
    float left_leg_torque_nm = 0.0f;
    /** LQR 虚拟腿力矩叠加防劈叉、离心补偿和小陀螺归中后的右腿命令。 */
    float right_leg_torque_nm = 0.0f;
    AntiSplit_Control(&left_leg_torque_nm, &right_leg_torque_nm);

    /** 跳跃状态机返回的本周期激活状态。 */
    const std::uint8_t jump_active = Jump_Motion_Update();
    result.jump_active = jump_active != 0U;

    /** 左腿重力补偿投影分母 cos(b_phi0)。 */
    const float left_gravity_projection = arm_cos_f32(state.left_leg.body_angle_rad);
    /** 右腿重力补偿投影分母 cos(b_phi0)。 */
    const float right_gravity_projection = arm_cos_f32(state.right_leg.body_angle_rad);

    if (result.jump_active)
    {
        dependencies_.legacy_command_target.L_F0 =
            200.0f + dependencies_.gravity_compensation_load / left_gravity_projection
            + dependencies_.roll_compensation_pid.output;
        dependencies_.legacy_command_target.R_F0 =
            200.0f + dependencies_.gravity_compensation_load / right_gravity_projection
            - dependencies_.roll_compensation_pid.output;
    }
    else
    {
        dependencies_.legacy_command_target.L_F0 =
            dependencies_.left_length_pid.output
            + dependencies_.gravity_compensation_load / left_gravity_projection
            + dependencies_.roll_compensation_pid.output;
        dependencies_.legacy_command_target.R_F0 =
            dependencies_.right_length_pid.output
            + dependencies_.gravity_compensation_load / right_gravity_projection
            - dependencies_.roll_compensation_pid.output;
    }
    dependencies_.legacy_command_target.L_T = left_leg_torque_nm;
    dependencies_.legacy_command_target.R_T = right_leg_torque_nm;

    off_ground_detect();

    /** 磕台阶检测器所需的长腿档位、模式和开关快照。 */
    StepHitDetectorInput step_hit_input{};
    step_hit_input.target_leg_state = input.target_leg_state;
    step_hit_input.mode_signals = input.mode_signals;
    step_hit_input.automatic_trigger_enabled = input.automatic_stair_climb_enabled;
    /** 磕台阶检测在本周期产生的自动 Stair 请求。 */
    const bool automatic_stair_request = step_hit_detector_.Update(state, step_hit_input);

    result.command = CaptureLegacyCommand();
    result.request_stair = automatic_stair_request
        || dependencies_.legacy_stair_request == 1U;
    result.request_sit = input.sit_requested;
    return result;
}

/**
 * @brief 捕获 LQR、支持力合成和离地覆盖全部结束后的命令。
 * @return 与 VMC_Chassis_Target 六个字段一一对应的 ChassisCommand。
 */
ChassisCommand BalanceController::CaptureLegacyCommand() const
{
    /** 本周期所有正常平衡覆盖逻辑结束后的最终命令。 */
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
