/**
 * @file balance_controller.cpp
 * @brief BalanceController 和 StepHitDetector 的 C++ 实现。
 */

#include "balance_controller.hpp"

#include <cmath>

extern "C"
{
#include "chassis_behavior_tree.h"
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
      tip_protection_controller_(dependencies.tip_protection_dependencies),
      height_controller_(dependencies.height_dependencies),
      jump_controller_(dependencies.jump_dependencies),
      lqr_controller_(),
      anti_split_controller_(dependencies.anti_split_dependencies),
      off_ground_detector_(dependencies.off_ground_dependencies),
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

    /** 倾覆检测和收腿保护控制器的本周期结果。 */
    const TipProtectionUpdateResult tip_result =
        tip_protection_controller_.Update(state);
    if (tip_result.active)
    {
        result.command = tip_result.command;
        result.tip_protection_active = true;
        result.tip_protection_completed = tip_result.completed;
        return result;
    }

    /** 不可修改的旧误差计算黑盒在本周期产生的三维输出快照。 */
    const LegacyBalanceErrorSnapshot error_snapshot = CalculateLegacyErrors();
    /** 横滚补偿、腿长斜坡和左右腿长 PID 的本周期输出。 */
    const ChassisHeightOutput height_output =
        height_controller_.Update(state, input.target_leg_state);
    lqr_controller_.UpdateGainMatrix(state.left_leg.length_m,
                                     state.right_leg.length_m);
    dependencies_.legacy_algorithms.update_power_control();

    /** 本周期锁存的小陀螺运行状态，供 LQR、防劈叉和跳跃使用同一取值。 */
    const bool spinning_active = dependencies_.spinning_active_flag == 1U;
    /** LQR 与跳跃必须读取离地检测器更新前的左腿计数，保持旧一周期时序。 */
    const int previous_left_off_ground_count =
        dependencies_.off_ground_dependencies.left_off_ground_count;
    /** LQR 与跳跃必须读取离地检测器更新前的右腿计数，保持旧一周期时序。 */
    const int previous_right_off_ground_count =
        dependencies_.off_ground_dependencies.right_off_ground_count;

    /** LQR 本周期使用的误差、离地计数与模式快照。 */
    LqrControlInput lqr_input{};
    lqr_input.body_distance_error_m = error_snapshot.body_distance_error_m;
    lqr_input.speed_error_mps = error_snapshot.speed_error_mps;
    lqr_input.yaw_error_rad = error_snapshot.yaw_error_rad;
    lqr_input.yaw_rate_radps = state.yaw_rate_radps;
    lqr_input.left_off_ground_count = previous_left_off_ground_count;
    lqr_input.right_off_ground_count = previous_right_off_ground_count;
    lqr_input.spinning_active = spinning_active;
    lqr_input.stair_request_active = dependencies_.legacy_stair_request == 1U;
    /** 结构化返回的左右轮力矩和防劈叉叠加前腿力矩。 */
    const LqrOutput lqr_output = lqr_controller_.Calculate(state, lqr_input);
    /** 防劈叉控制器使用的 LQR 基础力矩、yaw 角速度和小陀螺状态。 */
    AntiSplitControlInput anti_split_input{};
    anti_split_input.lqr_output = lqr_output;
    anti_split_input.yaw_rate_radps = state.yaw_rate_radps;
    anti_split_input.spinning_active = spinning_active;
    /** 防劈叉、离心补偿和小陀螺腿角归中叠加后的左右腿力矩。 */
    const AntiSplitOutput anti_split_output =
        anti_split_controller_.Update(state, anti_split_input);

    /** 跳跃控制器使用的腿长档位、常规目标、离地计数和旋转状态。 */
    JumpControlInput jump_input{};
    jump_input.target_leg_state = input.target_leg_state;
    jump_input.nominal_target_length_m = height_output.target_leg_length_m;
    jump_input.left_off_ground_count = previous_left_off_ground_count;
    jump_input.right_off_ground_count = previous_right_off_ground_count;
    jump_input.spinning_active = spinning_active;
    /** 跳跃锁存、成功/失败判定、PID 阶跃和蜂鸣器更新结果。 */
    const JumpUpdateResult jump_result = jump_controller_.Update(state, jump_input);
    result.jump_active = jump_result.active;

    /** 左腿重力补偿投影分母 cos(b_phi0)。 */
    const float left_gravity_projection = arm_cos_f32(state.left_leg.body_angle_rad);
    /** 右腿重力补偿投影分母 cos(b_phi0)。 */
    const float right_gravity_projection = arm_cos_f32(state.right_leg.body_angle_rad);

    /** 离地覆盖前由 LQR、腿长、重力补偿和防劈叉合成的正常平衡命令。 */
    ChassisCommand nominal_command{};
    nominal_command.left_wheel_torque_nm = lqr_output.left_wheel_torque_nm;
    nominal_command.right_wheel_torque_nm = lqr_output.right_wheel_torque_nm;
    nominal_command.left_leg_torque_nm = anti_split_output.left_leg_torque_nm;
    nominal_command.right_leg_torque_nm = anti_split_output.right_leg_torque_nm;

    if (result.jump_active)
    {
        nominal_command.left_support_force_n =
            jump_result.fixed_support_force_n
            + dependencies_.gravity_compensation_load / left_gravity_projection
            + height_output.roll_compensation_n;
        nominal_command.right_support_force_n =
            jump_result.fixed_support_force_n
            + dependencies_.gravity_compensation_load / right_gravity_projection
            - height_output.roll_compensation_n;
    }
    else
    {
        nominal_command.left_support_force_n =
            height_output.left_length_force_n
            + dependencies_.gravity_compensation_load / left_gravity_projection
            + height_output.roll_compensation_n;
        nominal_command.right_support_force_n =
            height_output.right_length_force_n
            + dependencies_.gravity_compensation_load / right_gravity_projection
            - height_output.roll_compensation_n;
    }

    /** 地面支持力滤波和离地单侧命令覆盖结果。 */
    const OffGroundUpdateResult off_ground_result =
        off_ground_detector_.Update(state, nominal_command);
    if (off_ground_result.request_short_leg_lock)
        dependencies_.short_leg_lock = 1U;
    if (off_ground_result.request_distance_reset)
    {
        dependencies_.body_distance_m = 0.0f;
        dependencies_.target_body_distance_m = 0.0f;
    }
    if (off_ground_result.request_step_cooldown_refresh)
    {
        dependencies_.step_hit_dependencies.cooldown_cycles =
            dependencies_.step_hit_dependencies.motor_frequency_hz;
    }

    /** 磕台阶检测器所需的长腿档位、模式和开关快照。 */
    StepHitDetectorInput step_hit_input{};
    step_hit_input.target_leg_state = input.target_leg_state;
    step_hit_input.mode_signals = input.mode_signals;
    step_hit_input.automatic_trigger_enabled = input.automatic_stair_climb_enabled;
    /** 磕台阶检测在本周期产生的自动 Stair 请求。 */
    const bool automatic_stair_request = step_hit_detector_.Update(state, step_hit_input);

    result.command = off_ground_result.command;
    result.request_stair = automatic_stair_request
        || dependencies_.legacy_stair_request == 1U;
    result.request_sit = input.sit_requested;
    return result;
}

/**
 * @brief 调用 Error_Calculate 黑盒并锁存其三个公开误差输出。
 * @return 供本周期 LQR 使用、不会再随全局量变化的误差快照。
 */
LegacyBalanceErrorSnapshot BalanceController::CalculateLegacyErrors() const
{
    dependencies_.legacy_algorithms.calculate_errors();

    /** 紧邻黑盒调用复制的位移、速度和偏航误差。 */
    LegacyBalanceErrorSnapshot snapshot{};
    snapshot.body_distance_error_m =
        dependencies_.legacy_algorithms.body_distance_error_m;
    snapshot.speed_error_mps = dependencies_.legacy_algorithms.speed_error_mps;
    snapshot.yaw_error_rad = dependencies_.legacy_algorithms.yaw_error_rad;
    return snapshot;
}

} // namespace chassis
