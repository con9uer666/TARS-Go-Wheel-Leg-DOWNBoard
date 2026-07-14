/**
 * @file startup_retract_controller.hpp
 * @brief 倒地自起与起立前收腿恢复的 C++ 灰度控制器接口。
 */

#ifndef STARTUP_RETRACT_CONTROLLER_HPP
#define STARTUP_RETRACT_CONTROLLER_HPP

#include <cstdint>
#include "chassis_control_types.hpp"

extern "C"
{
#include "user_pid.h"
#include "VMC.h"
}

namespace chassis
{

/**
 * @brief 检查当前车体是否应继续执行倒地自起的旧 C 函数类型。
 * @return 非 0 表示倒地自起正在接管输出，0 表示可进入收腿恢复。
 */
using SelfRightingModeDetectFunction = std::uint8_t (*)(void);

/**
 * @brief 执行一次旧倒地自起动作的 C 函数类型。
 */
using SelfRightingActionFunction = void (*)(void);

/**
 * @brief StartupRetractController 需要的 PID、VMC 和倒地自起兼容依赖。
 *
 * 所有对象均由 Motor_task 的组装层提供，控制器不拥有它们的生命周期。
 */
struct StartupRetractControllerDependencies
{
    user_pid_t& left_length_position_pid;  /**< 左腿腿长位置环 PID。 */
    user_pid_t& left_length_velocity_pid;  /**< 左腿腿长速度环 PID。 */
    user_pid_t& right_length_position_pid; /**< 右腿腿长位置环 PID。 */
    user_pid_t& right_length_velocity_pid; /**< 右腿腿长速度环 PID。 */
    user_pid_t& left_angle_position_pid;   /**< 左腿角度位置环 PID，供卡住反绕辅助器使用。 */
    user_pid_t& left_angle_velocity_pid;   /**< 左腿角速度环 PID。 */
    user_pid_t& right_angle_position_pid;  /**< 右腿角度位置环 PID，供卡住反绕辅助器使用。 */
    user_pid_t& right_angle_velocity_pid;  /**< 右腿角速度环 PID。 */
    VMC_t& left_vmc;                       /**< 左腿完整 VMC 状态，供旧卡住检测辅助器使用。 */
    VMC_t& right_vmc;                      /**< 右腿完整 VMC 状态，供旧卡住检测辅助器使用。 */
    VMC_Chassis_Target_t& legacy_command_target; /**< 旧 Self_Righting_Action() 写入的 VMC 映射前目标。 */
    SelfRightingModeDetectFunction detect_self_righting; /**< 倒地自起模式检测函数。 */
    SelfRightingActionFunction execute_self_righting;    /**< 倒地自起单周期动作函数。 */
};

/**
 * @brief 起立前收腿动作的目标、到位阈值和持续周期。
 */
struct StartupRetractControllerConfig
{
    float target_length_m = 0.12f;        /**< 左右腿收缩目标长度，单位 m。 */
    float left_target_angle_rad = 1.4707962f;  /**< 左腿目标角，与旧式 3.1415926f/2-0.1f 的 float 结果逐位等价，单位 rad。 */
    float right_target_angle_rad = 1.6707963f; /**< 右腿目标角，与旧式 3.1415926f/2+0.1f 的 float 结果逐位等价，单位 rad。 */
    float length_error_tolerance_m = 0.04f;   /**< 腿长位置环进入到位范围的绝对误差阈值，单位 m。 */
    std::uint16_t ready_cycles = 20U;     /**< 单腿到位周期数：腿长阶段按旧逻辑命中累计且脱离阈值不清零，腿角阶段要求连续到位。 */
};

/**
 * @brief StartupRetractController 的单周期返回值。
 */
struct StartupRetractUpdateResult
{
    ChassisCommand command;                    /**< 本周期将由任务提交的六维 VMC 映射前命令。 */
    bool self_righting_active = false;          /**< true 表示本周期命令由旧倒地自起动作产生。 */
    bool retract_control_active = false;        /**< true 表示车体姿态已允许执行收腿与转腿恢复。 */
    bool completed = false;                     /**< true 表示双腿已收缩并转回目标姿态。 */
};

/**
 * @brief 管理倒地自起接管和自起后收腿恢复的动作控制器。
 *
 * 倒地自起算法本身在本阶段保持原 C 实现；恢复姿态后的腿长和腿角
 * 控制已迁移到本类，并通过 ChassisCommand 返回命令。
 */
class StartupRetractController
{
public:
    /**
     * @brief 构造起立恢复控制器并绑定外部 PID、VMC 和旧自起接口。
     * @param[in] dependencies 生命周期必须覆盖控制器的外部依赖引用。
     * @param[in] config 收腿目标、阈值和到位周期参数。
     */
    StartupRetractController(
        const StartupRetractControllerDependencies& dependencies,
        const StartupRetractControllerConfig& config = StartupRetractControllerConfig{});

    /**
     * @brief 执行一次倒地自起判定或收腿恢复计算。
     * @param[in] state 本周期左右腿 VMC 运动状态快照。
     * @return 本周期命令、实际执行分支和恢复完成事件。
     */
    StartupRetractUpdateResult Update(const ChassisStateSnapshot& state);

private:
    /**
     * @brief 调用旧倒地自起动作并捕获其写入的六维命令。
     * @return 标记为自起接管的单周期返回值。
     */
    StartupRetractUpdateResult UpdateSelfRighting();

    /**
     * @brief 执行姿态恢复后的收腿和转腿控制。
     * @param[in] state 本周期左右腿状态快照。
     * @return 收腿命令及双腿是否全部完成。
     */
    StartupRetractUpdateResult UpdateRetract(const ChassisStateSnapshot& state);

    /**
     * @brief 把旧 VMC_Chassis_Target 的六个字段复制为统一命令。
     * @return 不改变符号、限幅和单位的 ChassisCommand。
     */
    ChassisCommand CaptureLegacyCommand() const;

    StartupRetractControllerDependencies dependencies_; /**< 构造时绑定的 PID、VMC 和自起函数引用。 */
    StartupRetractControllerConfig config_;              /**< 收腿目标、阈值和到位周期参数副本。 */
};

} // namespace chassis

#endif // STARTUP_RETRACT_CONTROLLER_HPP
