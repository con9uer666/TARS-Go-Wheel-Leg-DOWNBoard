/** @file jump_controller.cpp @brief JumpController 及旧调试符号兼容层实现。 */
#include "jump_controller.hpp"

/** @brief 根据 jump_cmd 与内部锁计算的当前跳跃模式兼容输出。 */
extern "C" std::uint8_t jump_mode = 0U;
/** @brief 上板下发的原始跳跃指令。 */
extern "C" std::uint8_t jump_cmd = 0U;
/** @brief 跳跃一票否决使能，1 允许，0 禁止。 */
extern "C" std::uint8_t jump_enable = 1U;
/** @brief 跳跃蜂鸣器独占标志，供现有错误蜂鸣器逻辑让位。 */
extern "C" std::uint8_t g_jump_buzzer_active = 0U;
/** @brief 跳跃腿长目标阶跃量，单位 m。 */
extern "C" float jump_L0_step_delta = 0.5f;
/** @brief 跳跃超时时的最小腿长变化判定阈值，单位 m。 */
extern "C" float jump_leg_change_threshold = 0.15f;
/** @brief 跳跃退出原因：0 未触发/锁存，1 离地成功，2 腿长不足，3 超时但腿长足够。 */
extern "C" std::uint8_t jump_fail_reason = 0U;

namespace chassis
{
/** @brief 保存跳跃依赖和配置副本。 @param[in] dependencies PID、指令、调试量和蜂鸣函数。 @param[in] config 锁存、离地、支持力与蜂鸣参数。 */
JumpController::JumpController(const JumpControllerDependencies& dependencies,
                               const JumpControllerConfig& config)
    : dependencies_(dependencies), config_(config) {}

/**
 * @brief 保留旧语句顺序更新跳跃锁、锁存计数、PID 与蜂鸣器边沿。
 * @param[in] state 左右腿长状态快照。
 * @param[in] input 腿长档位、常规目标腿长、离地计数与旋转状态。
 * @return 跳跃激活状态、固定支持力和重算 PID 输出。
 */
JumpUpdateResult JumpController::Update(const ChassisStateSnapshot& state,
                                        const JumpControlInput& input)
{
    if (dependencies_.jump_command == 0U)
        locked_ = false;

    dependencies_.jump_mode =
        (dependencies_.jump_command && !locked_) ? 1U : 0U;
    /** 未进入锁存窗口前，全部触发条件同时满足的原始跳跃状态。 */
    const bool active_raw =
        dependencies_.jump_mode == 1U
        && dependencies_.jump_enable == 1U
        && !input.spinning_active
        && input.target_leg_state == 0U
        && input.left_off_ground_count < config_.off_ground_success_threshold
        && input.right_off_ground_count < config_.off_ground_success_threshold;
    /** 0.5 s 锁存窗口对应的控制周期数。 */
    const int latch_target_cycles = static_cast<int>(
        dependencies_.motor_frequency_hz / config_.latch_duration_divisor);

    if (active_raw && !active_latched_)
    {
        active_latched_ = true;
        active_latch_cycles_remaining_ = latch_target_cycles;
        left_start_length_m_ = state.left_leg.length_m;
        right_start_length_m_ = state.right_leg.length_m;
        dependencies_.fail_reason = 0U;
    }
    if (active_latched_)
    {
        if (input.left_off_ground_count >= config_.off_ground_success_threshold
            || input.right_off_ground_count >= config_.off_ground_success_threshold)
        {
            dependencies_.fail_reason = 1U;
            active_latched_ = false;
            locked_ = true;
            dependencies_.jump_mode = 0U;
        }
        else if (--active_latch_cycles_remaining_ <= 0)
        {
            /** 锁存窗口内左腿长增量，单位 m。 */
            const float left_length_change_m =
                state.left_leg.length_m - left_start_length_m_;
            /** 锁存窗口内右腿长增量，单位 m。 */
            const float right_length_change_m =
                state.right_leg.length_m - right_start_length_m_;
            dependencies_.fail_reason =
                (left_length_change_m < dependencies_.minimum_leg_change_m
                 || right_length_change_m < dependencies_.minimum_leg_change_m)
                ? 2U : 3U;
            active_latched_ = false;
            locked_ = true;
            dependencies_.jump_mode = 0U;
        }
    }

    /** 原始触发或锁存窗口任一有效时的最终跳跃状态。 */
    const bool active = active_latched_ || active_raw;
    JumpUpdateResult result{};
    result.active = active;
    result.fixed_support_force_n = config_.fixed_support_force_n;
    if (active)
    {
        result.jump_target_length_m =
            input.nominal_target_length_m + dependencies_.target_length_step_m;
        PID_Set_Error(&dependencies_.left_length_pid, state.left_leg.length_m,
                      result.jump_target_length_m);
        PID_Set_Error(&dependencies_.right_length_pid, state.right_leg.length_m,
                      result.jump_target_length_m);
        result.left_length_pid_output = PID_coculate(&dependencies_.left_length_pid);
        result.right_length_pid_output = PID_coculate(&dependencies_.right_length_pid);
    }

    if (static_cast<std::uint8_t>(active) != dependencies_.buzzer_active)
    {
        if (active)
            dependencies_.start_buzzer(config_.buzzer_frequency_hz);
        else
            dependencies_.stop_buzzer();
        dependencies_.buzzer_active = active ? 1U : 0U;
    }
    return result;
}
} // namespace chassis
