/**
 * @file tip_protection_controller.hpp
 * @brief 倾覆检测、去抖计数和收腿保护命令的可注入 C++ 控制器接口。
 */

#ifndef TIP_PROTECTION_CONTROLLER_HPP
#define TIP_PROTECTION_CONTROLLER_HPP

#include <cstdint>

#include "chassis_control_types.hpp"

extern "C"
{
#include "user_pid.h"
}

namespace chassis
{

/**
 * @brief 倾覆保护的角度门槛、连续检测次数和动作持续时间配置。
 *
 * 默认值严格对应迁移前倾覆保护实现的三个宏。计数单位均为 Motor_task
 * 控制周期，不在控制器内部换算为毫秒，避免改变原有离散时序。
 */
struct TipProtectionConfig
{
    float pitch_threshold_deg = 45.0f;       /**< 前后倾覆判定使用的俯仰角绝对值门槛，单位 deg。 */
    std::uint8_t detection_cycles = 10U;     /**< 连续超出俯仰角门槛后触发保护所需的控制周期数。 */
    std::uint16_t action_cycles = 250U;      /**< 一次保护动作持续下发收腿命令的控制周期数。 */
};

/**
 * @brief 倾覆保护收腿动作所需的八个 PID 和最短腿长参数。
 *
 * 所有成员均为外部引用，控制器不拥有或复制 PID 状态。调用方必须保证这些
 * 对象的生命周期覆盖 TipProtectionController 的完整生命周期。
 */
struct TipProtectionControllerDependencies
{
    user_pid_t& left_length_position_pid;  /**< 左腿腿长位置环 PID，输出目标伸缩速度。 */
    user_pid_t& left_length_speed_pid;     /**< 左腿腿长速度环 PID，输出目标支持力。 */
    user_pid_t& right_length_position_pid; /**< 右腿腿长位置环 PID，输出目标伸缩速度。 */
    user_pid_t& right_length_speed_pid;    /**< 右腿腿长速度环 PID，输出目标支持力。 */
    user_pid_t& left_angle_position_pid;   /**< 左腿虚拟腿角位置环 PID，目标为竖直方向。 */
    user_pid_t& left_angle_speed_pid;      /**< 左腿虚拟腿角速度环 PID，输出目标腿力矩。 */
    user_pid_t& right_angle_position_pid;  /**< 右腿虚拟腿角位置环 PID，目标为竖直方向。 */
    user_pid_t& right_angle_speed_pid;     /**< 右腿虚拟腿角速度环 PID，输出经符号适配前的腿力矩。 */
    float& minimum_leg_length_m;           /**< 工程最短腿长 LEG_MIN_LENTH，单位 m。 */
};

/** @brief 倾覆保护控制器一次更新产生的命令和状态事件。 */
struct TipProtectionUpdateResult
{
    ChassisCommand command{};      /**< 保护激活时的收腿、摆正和轮力矩清零命令。 */
    bool active = false;           /**< true 表示本周期必须由保护命令接管正常平衡控制。 */
    bool completed = false;        /**< true 仅在第 250 个保护动作周期结束时出现一帧。 */
};

/**
 * @brief 拥有倾覆去抖计数与动作倒计时的收腿保护控制器。
 *
 * 控制器不读取任何全局反馈，不写 VMC_Chassis_Target，也不修改运行模式。
 * 上层根据 completed 事件执行原 start_mode/upstares_mode 等兼容状态复位。
 */
class TipProtectionController
{
public:
    /**
     * @brief 构造控制器并保存 PID、腿长参数引用及配置副本。
     * @param[in] dependencies 生命周期必须覆盖控制器的外部 PID 和腿长参数。
     * @param[in] config 俯仰门槛、检测周期数和保护动作周期数。
     */
    TipProtectionController(
        const TipProtectionControllerDependencies& dependencies,
        const TipProtectionConfig& config = TipProtectionConfig{});

    /**
     * @brief 更新倾覆检测；激活后计算一次收腿保护命令并推进倒计时。
     * @param[in] state 本周期锁存的俯仰角、左右腿长度与腿角运动状态。
     * @return 是否接管、是否在本周期结束以及对应的六维保护命令。
     */
    TipProtectionUpdateResult Update(const ChassisStateSnapshot& state);

private:
    /**
     * @brief 在保护未激活时更新连续超门槛计数并按旧时序启动倒计时。
     * @param[in] pitch_angle_deg 本周期 IMU 原始俯仰角，单位 deg。
     */
    void UpdateDetection(float pitch_angle_deg);

    /**
     * @brief 执行腿长/腿角位置速度双环并生成轮力矩为零的保护命令。
     * @param[in] state 本周期左右腿 VMC 状态快照。
     * @return 与迁移前倾覆保护动作完全同符号的六维命令。
     */
    ChassisCommand CalculateProtectiveCommand(const ChassisStateSnapshot& state);

    TipProtectionControllerDependencies dependencies_; /**< 八个 PID 与最短腿长的非拥有引用。 */
    TipProtectionConfig config_;                        /**< 倾覆门槛和两个离散计数配置副本。 */
    std::uint16_t remaining_action_cycles_ = 0U;        /**< 剩余保护动作周期，非零表示保护已激活。 */
    std::uint8_t consecutive_detection_cycles_ = 0U;   /**< 当前连续超出俯仰门槛的去抖计数。 */
};

} // namespace chassis

#endif // TIP_PROTECTION_CONTROLLER_HPP
