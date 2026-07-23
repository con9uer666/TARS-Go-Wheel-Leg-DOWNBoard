/**
 * @file spinning_controller.hpp
 * @brief 小陀螺加速与退出旋转控制的可注入 C++ 控制器。
 */
#ifndef SPINNING_CONTROLLER_HPP
#define SPINNING_CONTROLLER_HPP

extern "C"
{
#include "user_pid.h"
}

namespace chassis
{
/** @brief SpinningController 使用的 PID、目标/反馈滤波状态和现场可调参数。 */
struct SpinningControllerDependencies
{
    user_pid_t& yaw_rate_pid;            /**< 小陀螺加速和退出共用的 yaw 角速度 PID。 */
    user_pid_t& exit_angle_pid;          /**< 退出阶段将 yaw_angle_PI 映射为目标角速度的 PID。 */
    float& filtered_target_yaw_rate_radps; /**< 低通后的小陀螺目标角速度。 */
    float& filtered_yaw_rate_radps;      /**< 低通后的实际 yaw 角速度。 */
    float& target_yaw_rate_radps;        /**< 小陀螺标称目标角速度。 */
    float& target_filter_alpha;          /**< 加速阶段目标角速度低通新样本权重。 */
    float& feedback_filter_alpha;        /**< 加速阶段反馈角速度低通新样本权重。 */
};

/** @brief 小陀螺控制单周期的 yaw、功率门控和缩放输入。 */
struct SpinningControlInput
{
    float yaw_rate_radps = 0.0f;       /**< 当前车体 yaw 角速度 d_yaw。 */
    float relative_yaw_angle_rad = 0.0f; /**< 车体相对云台的 yaw_angle_PI。 */
    float filtered_power_w = 0.0f;     /**< 功率观测器滤波输出 g_filtered_power。 */
    float power_limit_w = 0.0f;        /**< 当前功率限制 power_limit。 */
    float power_scale = 1.0f;          /**< 超功率时缩放旋转目标的 g_power_obs_lambda。 */
};

/** @brief 计算小陀螺加速或退出阶段 yaw 控制量的控制器。 */
class SpinningController
{
public:
    /** @brief 构造控制器并绑定 PID 与滤波状态。 @param[in] dependencies 生命周期覆盖控制器的外部引用。 */
    explicit SpinningController(const SpinningControllerDependencies& dependencies);

    /** @brief 低通目标/反馈角速度并执行加速闭环。 @param[in] input yaw 与功率状态。 @return 供 LQR yaw_error 使用的 PID 输出。 */
    float UpdateActive(const SpinningControlInput& input);

    /** @brief 用相对 yaw 角生成退出目标角速度并执行闭环。 @param[in] input yaw 与功率状态。 @return 供 LQR yaw_error 使用的 PID 输出。 */
    float UpdateExit(const SpinningControlInput& input);

private:
    SpinningControllerDependencies dependencies_; /**< PID、滤波状态、目标转速与滤波系数引用。 */
};
} // namespace chassis
#endif // SPINNING_CONTROLLER_HPP
