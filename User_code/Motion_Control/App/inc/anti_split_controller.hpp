/**
 * @file anti_split_controller.hpp
 * @brief 防劈叉、小陀螺腿角归中和离心补偿的 C++ 腿力矩控制器。
 */

#ifndef ANTI_SPLIT_CONTROLLER_HPP
#define ANTI_SPLIT_CONTROLLER_HPP

#include <cstdint>
#include "chassis_control_types.hpp"
#include "lqr_controller.hpp"

extern "C"
{
#include "user_pid.h"
}

namespace chassis
{

/** @brief AntiSplitController 所需的 PID 和现场可调参数引用。 */
struct AntiSplitControllerDependencies
{
    user_pid_t& anti_split_pid;       /**< 左右腿相对张开误差的防劈叉 PID。 */
    user_pid_t& left_spin_angle_pid;  /**< 小陀螺时左腿 phi0 独立归中 PID。 */
    user_pid_t& right_spin_angle_pid; /**< 小陀螺时右腿 phi0 独立归中 PID。 */
    float& minimum_leg_length_m;      /**< 防劈叉增益插值的最短腿长端点，单位 m。 */
    float& maximum_leg_length_m;      /**< 防劈叉增益插值的最长腿长端点，单位 m。 */
    float& centrifugal_compensation_gain; /**< 按 d_yaw² 计算的双腿共同离心补偿系数。 */
    float& spin_target_leg_angle_rad; /**< 小陀螺时左右腿的 phi0 归中目标，单位 rad。 */
};

/** @brief 防劈叉 PID 随平均腿长线性变化的端点参数。 */
struct AntiSplitControllerConfig
{
    float short_leg_kp = 300.0f; /**< 最短腿长对应的防劈叉比例增益。 */
    float long_leg_kp = 700.0f;  /**< 最长腿长对应的防劈叉比例增益。 */
    float short_leg_kd = 10.0f;  /**< 最短腿长对应的防劈叉微分增益。 */
    float long_leg_kd = 200.0f;  /**< 最长腿长对应的防劈叉微分增益。 */
    float centered_leg_angle_rad = 1.57079625f; /**< 防劈叉夹角误差的单腿中心角，与旧 PI/2.0f 逐位等价。 */
};

/** @brief AntiSplitController 单周期使用的 LQR 基础力矩和旋转状态。 */
struct AntiSplitControlInput
{
    LqrOutput lqr_output{};          /**< 防劈叉叠加前的左右腿 LQR 基础力矩。 */
    float yaw_rate_radps = 0.0f;     /**< 车体 yaw 角速度 d_yaw，单位 rad/s。 */
    bool spinning_active = false;    /**< true 表示启用左右腿独立 phi0 归中 PID。 */
};

/** @brief 防劈叉、离心补偿和小陀螺归中叠加后的左右腿力矩。 */
struct AntiSplitOutput
{
    float left_leg_torque_nm = 0.0f;  /**< 左腿最终虚拟力矩命令，单位 N·m。 */
    float right_leg_torque_nm = 0.0f; /**< 右腿最终虚拟力矩命令，单位 N·m。 */
};

/**
 * @brief 将 LQR 基础腿力矩与双腿防劈叉、旋转离心补偿和小陀螺归中力矩合成。
 */
class AntiSplitController
{
public:
    /**
     * @brief 构造控制器并绑定三个 PID、腿长端点和旋转补偿参数。
     * @param[in] dependencies 生命周期必须覆盖控制器的外部依赖引用。
     * @param[in] config 防劈叉 Kp/Kd 端点和单腿中心角。
     */
    AntiSplitController(const AntiSplitControllerDependencies& dependencies,
                        const AntiSplitControllerConfig& config = AntiSplitControllerConfig{});

    /**
     * @brief 根据腿长、LQR 基础力矩和旋转状态计算左右腿最终力矩。
     * @param[in] state 包含左右腿长与 phi0 的本周期状态快照。
     * @param[in] input LQR 基础力矩、yaw 角速度和小陀螺激活状态。
     * @return 已叠加全部防劈叉与旋转补偿的左右腿力矩。
     */
    AntiSplitOutput Update(const ChassisStateSnapshot& state,
                           const AntiSplitControlInput& input);

private:
    /**
     * @brief 按当前左右腿平均长度线性刷新防劈叉 PID 的 Kp 和 Kd。
     * @param[in] average_leg_length_m 左右腿长平均值，单位 m。
     */
    void UpdateGains(float average_leg_length_m);

    AntiSplitControllerDependencies dependencies_; /**< PID、腿长端点、离心补偿和旋转目标引用。 */
    AntiSplitControllerConfig config_;              /**< 防劈叉增益端点和中心角参数副本。 */
    bool spin_angle_pid_started_ = false;           /**< 小陀螺 phi0 PID 是否已完成首帧 pre_error 同步。 */
};

} // namespace chassis

#endif // ANTI_SPLIT_CONTROLLER_HPP
