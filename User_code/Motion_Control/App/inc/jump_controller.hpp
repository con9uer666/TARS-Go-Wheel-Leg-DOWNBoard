/**
 * @file jump_controller.hpp
 * @brief 跳跃指令锁存、成功/失败判定、腿长 PID 阶跃和蜂鸣器状态的 C++ 控制器。
 */
#ifndef JUMP_CONTROLLER_HPP
#define JUMP_CONTROLLER_HPP

#include <cstdint>
#include "chassis_control_types.hpp"
extern "C"
{
#include "user_pid.h"
}

namespace chassis
{
/** @brief 以指定频率启动蜂鸣器的函数类型。 */
using JumpBuzzerStartFunction = void (*)(int);
/** @brief 停止蜂鸣器的函数类型。 */
using JumpBuzzerStopFunction = void (*)(void);

/** @brief JumpController 使用的 PID、外部指令、调参量和兼容输出引用。 */
struct JumpControllerDependencies
{
    user_pid_t& left_length_pid;       /**< 跳跃激活时使用阶跃腿长目标重新计算的左腿 PID。 */
    user_pid_t& right_length_pid;      /**< 跳跃激活时重新计算的右腿 PID。 */
    std::uint8_t& jump_mode;           /**< 根据原始指令与内部锁计算的兼容跳跃模式输出。 */
    std::uint8_t& jump_command;        /**< 上板下发的原始跳跃指令 jump_cmd。 */
    std::uint8_t& jump_enable;         /**< 跳跃一票否决使能，只有等于 1 时允许触发。 */
    std::uint8_t& buzzer_active;       /**< 供错误蜂鸣器让位的 g_jump_buzzer_active 兼容标志。 */
    float& target_length_step_m;       /**< 跳跃时叠加到常规腿长目标上的阶跃量。 */
    float& minimum_leg_change_m;       /**< 锁存超时时判定腿长变化不足的阈值。 */
    std::uint8_t& fail_reason;         /**< 跳跃退出原因调试输出：0/1/2/3。 */
    std::uint16_t& motor_frequency_hz; /**< 用于换算 0.5 s 锁存时间的底盘控制频率。 */
    JumpBuzzerStartFunction start_buzzer; /**< 跳跃进入边沿调用的蜂鸣器启动函数。 */
    JumpBuzzerStopFunction stop_buzzer;   /**< 跳跃退出边沿调用的蜂鸣器停止函数。 */
};

/** @brief 跳跃锁存时间、离地阈值、固定支持力和蜂鸣频率配置。 */
struct JumpControllerConfig
{
    std::uint16_t latch_duration_divisor = 2U; /**< 锁存周期数为 motor_HZ/2，即 0.5 s。 */
    int off_ground_success_threshold = 10;     /**< 任一腿离地计数达到该值即判定跳跃成功。 */
    float fixed_support_force_n = 200.0f;       /**< 跳跃激活时 Balance 使用的单腿固定支持力基值。 */
    int buzzer_frequency_hz = 784;              /**< 跳跃期间蜂鸣器的最大音量 sol 频率。 */
};

/** @brief JumpController 单周期的腿长档位、旋转和离地状态。 */
struct JumpControlInput
{
    std::uint8_t target_leg_state = 0U; /**< 当前腿长档位，只有等于 0 时允许跳跃。 */
    float nominal_target_length_m = 0.0f; /**< 高度控制器斜坡后的常规腿长目标。 */
    int left_off_ground_count = 0;  /**< 左腿上周期离地计数。 */
    int right_off_ground_count = 0; /**< 右腿上周期离地计数。 */
    bool spinning_active = false;   /**< true 表示小陀螺激活，禁止新跳跃触发。 */
};

/** @brief 跳跃状态机的单周期返回值。 */
struct JumpUpdateResult
{
    bool active = false;                   /**< true 表示本周期处于原始触发或锁存跳跃阶段。 */
    float fixed_support_force_n = 200.0f;  /**< active 时 Balance 应使用的固定单腿支持力基值。 */
    float jump_target_length_m = 0.0f;     /**< active 时重新计算腿长 PID 使用的阶跃目标。 */
    float left_length_pid_output = 0.0f;   /**< 跳跃阶跃目标下重新计算的左腿 PID 输出；保留状态副作。 */
    float right_length_pid_output = 0.0f;  /**< 跳跃阶跃目标下重新计算的右腿 PID 输出。 */
};

/** @brief 保证一次指令只触发一次跳跃，并统一管理失败原因与蜂鸣器边沿的控制器。 */
class JumpController
{
public:
    /**
     * @brief 构造跳跃控制器并绑定 PID、指令、调试输出和蜂鸣器函数。
     * @param[in] dependencies 生命周期覆盖控制器的外部引用。
     * @param[in] config 锁存时间、离地阈值、固定支持力和蜂鸣频率。
     */
    JumpController(const JumpControllerDependencies& dependencies,
                   const JumpControllerConfig& config = JumpControllerConfig{});

    /**
     * @brief 更新跳跃解锁、锁存、成功/超时判定、PID 阶跃和蜂鸣器边沿。
     * @param[in] state 包含左右腿当前长度的状态快照。
     * @param[in] input 腿长档位、常规目标腿长、离地计数和小陀螺状态。
     * @return 跳跃激活状态、固定支持力和阶跃 PID 调试输出。
     */
    JumpUpdateResult Update(const ChassisStateSnapshot& state,
                            const JumpControlInput& input);

private:
    JumpControllerDependencies dependencies_; /**< PID、指令、调参量、兼容输出和蜂鸣器函数引用。 */
    JumpControllerConfig config_;              /**< 锁存、成功判定、支持力和蜂鸣参数副本。 */
    bool locked_ = false;                      /**< 当前原始 jump_cmd 是否已经成功或超时消费。 */
    bool active_latched_ = false;              /**< 是否处于 0.5 s 强制跳跃锁存窗口。 */
    int active_latch_cycles_remaining_ = 0;    /**< 强制跳跃锁存窗口剩余周期数。 */
    float left_start_length_m_ = 0.0f;         /**< 锁存首帧左腿长，用于超时失败判定。 */
    float right_start_length_m_ = 0.0f;        /**< 锁存首帧右腿长。 */
};
} // namespace chassis
#endif // JUMP_CONTROLLER_HPP
