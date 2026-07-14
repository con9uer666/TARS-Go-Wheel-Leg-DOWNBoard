/**
 * @file sit_controller.hpp
 * @brief 可独立复用的 C++ 坐地动作控制器。
 */

#ifndef SIT_CONTROLLER_HPP
#define SIT_CONTROLLER_HPP

#include <cstdint>
#include "chassis_control_types.hpp"

extern "C"
{
#include "user_pid.h"
#include "ramp_generator.h"
}

namespace chassis
{

/**
 * @brief SitController 使用的 PID 和重力补偿依赖。
 *
 * 控制器通过引用接收依赖，不在内部查找全局 PID，因此可以在其他工程中用
 * 不同的 PID 实例构造同一个控制器。
 */
struct SitControllerDependencies
{
    user_pid_t& left_length_position_pid;  /**< 左腿腿长位置环 PID。 */
    user_pid_t& left_length_velocity_pid;  /**< 左腿腿长速度环 PID。 */
    user_pid_t& right_length_position_pid; /**< 右腿腿长位置环 PID。 */
    user_pid_t& right_length_velocity_pid; /**< 右腿腿长速度环 PID。 */
    user_pid_t& left_angle_position_pid;   /**< 左腿角度位置环 PID。 */
    user_pid_t& left_angle_velocity_pid;   /**< 左腿角度速度环 PID。 */
    user_pid_t& right_angle_position_pid;  /**< 右腿角度位置环 PID。 */
    user_pid_t& right_angle_velocity_pid;  /**< 右腿角度速度环 PID。 */
    float& gravity_compensation_load;      /**< 原 mg 全局量，用于计算单腿重力补偿。 */
};

/**
 * @brief 坐地动作的可调目标和时间参数。
 */
struct SitControllerConfig
{
    float left_target_length_m = 0.1532f;    /**< 左腿坐地目标长度，单位 m。 */
    float right_target_length_m = 0.1555f;   /**< 右腿坐地目标长度，单位 m。 */
    float left_target_angle_rad = 0.7353f;   /**< 左腿坐地目标虚拟腿角，单位 rad。 */
    float right_target_angle_rad = 2.3164f;  /**< 右腿坐地目标虚拟腿角，单位 rad。 */
    float ramp_duration_s = 0.8f;            /**< 从进入姿态过渡到坐地姿态的斜坡时间，单位 s。 */
    float sample_period_s = 0.002f;          /**< 控制器调用周期，单位 s。 */
    float target_tolerance = 0.001f;         /**< 判断四条斜坡完成的绝对误差阈值。 */
    float wheel_hold_torque_nm = 0.0f;       /**< 坐地时左轮保持力矩；右轮使用相反符号。 */
    float gravity_compensation_ratio = 0.3f; /**< 坐地时保留的重力补偿比例。 */
};

/**
 * @brief SitController 单周期计算结果。
 */
struct SitUpdateResult
{
    ChassisCommand command;                    /**< 本周期坐地模式的六维执行命令。 */
    bool request_startup_retract = false;      /**< 坐地请求撤销后，要求任务切回起立前收腿模式。 */
};

/**
 * @brief 两腿斜坡坐地控制器。
 *
 * 控制器拥有四个斜坡发生器及进入状态；反馈从 ChassisStateSnapshot 读取，
 * 执行结果通过 ChassisCommand 返回，不直接写 VMC_Chassis_Target。
 */
class SitController
{
public:
    /**
     * @brief 构造坐地控制器并绑定外部 PID 实例。
     * @param[in] dependencies 坐地控制所需 PID 和重力补偿引用。
     * @param[in] config 坐地目标、斜坡时间和补偿参数。
     */
    SitController(const SitControllerDependencies& dependencies,
                  const SitControllerConfig& config = SitControllerConfig{});

    /**
     * @brief 执行一次坐地控制计算。
     * @param[in] state 本周期只读底盘状态快照。
     * @param[in] sit_requested 当前是否仍保持坐地请求。
     * @return 坐地执行命令及退出到起立前收腿模式的请求。
     */
    SitUpdateResult Update(const ChassisStateSnapshot& state, bool sit_requested);

private:
    /**
     * @brief 使用进入坐地瞬间的腿状态初始化四条目标斜坡。
     * @param[in] state 进入坐地时的底盘状态快照。
     */
    void InitializeRamps(const ChassisStateSnapshot& state);

    /** @brief 推进四条斜坡一个 2 ms 周期，并更新完成标志。 */
    void UpdateRamps();

    /**
     * @brief 根据当前状态和斜坡目标计算双环 PID，生成六维坐地命令。
     * @param[in] state 本周期只读底盘状态快照。
     * @return 本周期 VMC 映射前命令。
     */
    ChassisCommand CalculateCommand(const ChassisStateSnapshot& state);

    /** @brief 标记下一次进入坐地时必须重新以当前姿态初始化斜坡。 */
    void PrepareNextEntry();

    SitControllerDependencies dependencies_; /**< 构造时绑定且生命周期覆盖控制器的外部依赖。 */
    SitControllerConfig config_;              /**< 坐地目标和斜坡参数副本。 */
    RampGenerator left_length_ramp_{};        /**< 左腿长度目标斜坡。 */
    RampGenerator right_length_ramp_{};       /**< 右腿长度目标斜坡。 */
    RampGenerator left_angle_ramp_{};         /**< 左腿角度目标斜坡。 */
    RampGenerator right_angle_ramp_{};        /**< 右腿角度目标斜坡。 */
    std::uint16_t execution_cycle_count_ = 0; /**< 坐地动作累计执行周期数，保留 uint16 回卷行为。 */
    bool ramps_initialized_ = false;          /**< 四条斜坡是否已经按本次进入姿态初始化。 */
    bool ramps_finished_ = false;             /**< 四条斜坡是否都进入目标误差阈值。 */
};

} // namespace chassis

#endif // SIT_CONTROLLER_HPP
