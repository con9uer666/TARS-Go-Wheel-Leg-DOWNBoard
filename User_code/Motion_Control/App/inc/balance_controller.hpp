/**
 * @file balance_controller.hpp
 * @brief 正常平衡流程、磕台阶检测与模式请求的 C++ 控制器接口。
 */

#ifndef BALANCE_CONTROLLER_HPP
#define BALANCE_CONTROLLER_HPP

#include <cstdint>
#include "chassis_control_types.hpp"
#include "lqr_controller.hpp"

extern "C"
{
#include "user_pid.h"
#include "VMC.h"
}

namespace chassis
{

/** @brief 磕台阶检测的力矩、腿长与时间参数。 */
struct StepHitDetectorConfig
{
    float leg_torque_threshold_nm = 6.0f; /**< 单腿实际虚拟力矩判定为碰撞的下限，单位 N·m。 */
    float maximum_length_margin_m = 0.03f; /**< 腿长门槛相对最大腿长减少的余量，单位 m。 */
    std::uint8_t required_hit_cycles = 2U; /**< 两腿同时命中后产生检测事件的计数阈值。 */
    std::uint8_t count_saturation_multiple = 2U; /**< 命中计数上限相对触发阈值的倍数。 */
    std::uint16_t long_leg_arm_delay_divisor = 4U; /**< 长腿切换后解禁延时的频率除数：motor_HZ/4。 */
};

/** @brief StepHitDetector 使用的工程参数和共享冷却状态。 */
struct StepHitDetectorDependencies
{
    float& maximum_leg_length_m;         /**< 工程最大腿长 LEG_MAX_LENTH，单位 m。 */
    std::uint16_t& motor_frequency_hz;   /**< 底盘控制频率 motor_HZ，用于换算冷却和延时。 */
    int& cooldown_cycles;                /**< 与离地检测共享的 step_hit_cooldown，单位控制周期。 */
    float& left_ground_support_force_n;  /**< 左腿滤波后地面支持力，供保留原检测器的上周期记录。 */
    float& right_ground_support_force_n; /**< 右腿滤波后地面支持力，供保留原检测器的上周期记录。 */
};

/** @brief 单周期传入磕台阶检测器的模式和腿长指令。 */
struct StepHitDetectorInput
{
    std::uint8_t target_leg_state = 0U;     /**< 上位机腿长档位，1 表示长腿姿态。 */
    LegacyModeSignals mode_signals{};       /**< 本周期锁存的 start_mode/upstares_mode 兼容状态。 */
    bool automatic_trigger_enabled = false; /**< true 表示检测成功可产生自动上台阶请求。 */
};

/** @brief 保留命中计数、长腿解禁延时和离地冷却联动的磕台阶检测器。 */
class StepHitDetector
{
public:
    /**
     * @brief 构造检测器并绑定腿长、频率和共享冷却状态。
     * @param[in] dependencies 生命周期必须覆盖检测器的工程参数引用。
     * @param[in] config 碰撞阈值、计数和解禁延时参数。
     */
    StepHitDetector(const StepHitDetectorDependencies& dependencies,
                    const StepHitDetectorConfig& config = StepHitDetectorConfig{});

    /**
     * @brief 执行一次磕台阶检测、计数衰减和冷却更新。
     * @param[in] state 包含腿长和实际虚拟腿力矩的状态快照。
     * @param[in] input 长腿档位、旧模式快照和自动触发使能。
     * @return true 表示本周期检测成功且开关允许自动进入上台阶。
     */
    bool Update(const ChassisStateSnapshot& state, const StepHitDetectorInput& input);

private:
    StepHitDetectorDependencies dependencies_; /**< 腿长、频率、冷却和支持力引用。 */
    StepHitDetectorConfig config_;              /**< 碰撞阈值和计数参数副本。 */
    std::uint8_t hit_count_ = 0U;               /**< 当前双腿同时命中计数，未命中时每周期减 1。 */
    std::uint8_t previous_target_leg_state_ = 0U; /**< 上周期腿长档位，用于检测长腿上升沿。 */
    int long_leg_arm_delay_cycles_ = 0;         /**< 长腿上升沿后剩余解禁延时，单位控制周期。 */
    float previous_left_ground_support_force_n_ = 0.0f;  /**< 上周期左腿支持力记录。 */
    float previous_right_ground_support_force_n_ = 0.0f; /**< 上周期右腿支持力记录。 */
};

/** @brief BalanceController 从旧算法模块捕获命令所需的共享对象。 */
struct BalanceControllerDependencies
{
    VMC_Chassis_Target_t& legacy_command_target; /**< 离地检测和倾覆保护暂时共用的 VMC 目标适配区。 */
    user_pid_t& left_length_pid;                 /**< 正常腿长控制的左腿 PID。 */
    user_pid_t& right_length_pid;                /**< 正常腿长控制的右腿 PID。 */
    user_pid_t& roll_compensation_pid;           /**< 横滚补偿 PID，以相反符号叠加到两腿 F0。 */
    float& gravity_compensation_load;            /**< 原 mg 全局量，用于按 cos(b_phi0) 投影计算重力补偿。 */
    std::uint8_t& legacy_stair_request;          /**< 上板或调试入口写入的 upstairs_flag。 */
    StepHitDetectorDependencies step_hit_dependencies; /**< 磕台阶检测器所需的共享参数。 */
};

/** @brief 正常平衡模式的单周期外部请求快照。 */
struct BalanceControlInput
{
    std::uint8_t target_leg_state = 0U;       /**< 腿长档位，0 为短腿，1 为长腿。 */
    bool sit_requested = false;               /**< true 表示要求从平衡模式进入坐地动作。 */
    bool automatic_stair_climb_enabled = false; /**< true 表示允许磕台阶检测自动发出 Stair 请求。 */
    LegacyModeSignals mode_signals{};         /**< 本周期 start_mode/upstares_mode 锁存值。 */
};

/** @brief BalanceController 单周期计算结果。 */
struct BalanceUpdateResult
{
    ChassisCommand command;            /**< 倾覆保护或正常平衡流程产生的最终六维命令。 */
    bool tip_protection_active = false; /**< true 表示本周期由倾覆保护接管。 */
    bool jump_active = false;          /**< true 表示本周期使用跳跃固定支持力逻辑。 */
    bool request_stair = false;        /**< true 表示外部请求或启用的磕台阶检测要求进入 Stair。 */
    bool request_sit = false;          /**< true 表示要求进入 Sit；任务按旧顺序使其覆盖 Stair。 */
};

/**
 * @brief 固定执行正常平衡算法链并返回命令与模式请求的 C++ 控制器。
 *
 * 本阶段保留 Error_Calculate 和离地检测等现有 C 算法；LQR 已迁移为内部 LqrController。
 * legacy_command_target 作为受控灰度适配区，Update() 在覆盖逻辑结束后捕获一次命令。
 */
class BalanceController
{
public:
    /**
     * @brief 构造平衡控制器并绑定旧命令适配区、PID 和磕台阶检测依赖。
     * @param[in] dependencies 生命周期必须覆盖控制器的共享对象引用。
     */
    explicit BalanceController(const BalanceControllerDependencies& dependencies);

    /**
     * @brief 执行一次倾覆保护判定或完整正常平衡流程。
     * @param[in] state 本周期 VMC、INS 和轮端状态快照。
     * @param[in] input 腿长档位、坐地请求、自动 Stair 开关和旧模式快照。
     * @return 最终命令和需要由任务提交的模式转换请求。
     */
    BalanceUpdateResult Update(const ChassisStateSnapshot& state,
                               const BalanceControlInput& input);

private:
    /**
     * @brief 从旧算法适配区复制最终六维命令。
     * @return 不改变数值、符号和单位的 ChassisCommand。
     */
    ChassisCommand CaptureLegacyCommand() const;

    BalanceControllerDependencies dependencies_; /**< 旧命令适配区、PID、重力量和外部 Stair 请求引用。 */
    LqrController lqr_controller_;               /**< 拥有 100 Hz 增益刷新计数并返四路基础力矩的 LQR 控制器。 */
    StepHitDetector step_hit_detector_;           /**< 拥有命中计数和长腿解禁延时的磕台阶检测器。 */
};

} // namespace chassis

#endif // BALANCE_CONTROLLER_HPP
