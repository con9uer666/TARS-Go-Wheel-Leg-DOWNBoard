/**
 * @file off_ground_detector.cpp
 * @brief OffGroundDetector 及其旧调试符号兼容层实现。
 */

#include "off_ground_detector.hpp"

extern "C"
{
#include "chassis_behavior_tree.h"
}

/** @brief 左腿滤波后地面支持力兼容调试符号，单位 N。 */
extern "C" float L_Ground_F0 = 0.0f;
/** @brief 右腿滤波后地面支持力兼容调试符号，单位 N。 */
extern "C" float R_Ground_F0 = 0.0f;
/** @brief 左腿离地滞回计数，保持 int 以允许递减后夹到零。 */
extern "C" int L_off_ground = 0;
/** @brief 右腿离地滞回计数。 */
extern "C" int R_off_ground = 0;
/** @brief 离地检测与 StepHitDetector 共享的磕台阶检测冷却计数。 */
extern "C" int step_hit_cooldown = 0;

namespace chassis
{

/**
 * @brief 保存离地检测所需的外部状态引用和参数副本。
 * @param[in] dependencies VMC、腿长 PID、K 矩阵和兼容调试量引用。
 * @param[in] config 支持力滤波、离地计数和覆盖命令缩放参数。
 */
OffGroundDetector::OffGroundDetector(
    const OffGroundDetectorDependencies& dependencies,
    const OffGroundDetectorConfig& config)
    : dependencies_(dependencies), config_(config)
{
}

/**
 * @brief 保留旧 if/else 阈值顺序更新离地计数。
 * @param[in] filtered_ground_force_n 单腿滤波后地面支持力，单位 N。
 * @param[in,out] off_ground_count 单腿离地计数。
 */
void OffGroundDetector::UpdateCounter(float filtered_ground_force_n,
                                      int& off_ground_count) const
{
    if (filtered_ground_force_n <= config_.count_increment_force_n)
        ++off_ground_count;
    else if (filtered_ground_force_n >= config_.count_decrement_force_n)
        --off_ground_count;

    if (off_ground_count >= config_.count_maximum)
        off_ground_count = config_.count_maximum;
    if (off_ground_count <= 0)
        off_ground_count = 0;
}

/**
 * @brief 更新左右支持力/离地计数，并仅覆盖离地侧的 F0、T 和轮力矩。
 * @param[in] state 左右腿 b_phi0 和角速度状态快照。
 * @param[in] nominal_command 离地覆盖前的正常平衡命令。
 * @return 覆盖后命令与短腿锁、位移清零和冷却刷新事件。
 */
OffGroundUpdateResult OffGroundDetector::Update(
    const ChassisStateSnapshot& state,
    const ChassisCommand& nominal_command)
{
    // 离地检测暂时禁用，直接透传命令
    OffGroundUpdateResult result{};
    result.command = nominal_command;
    return result;

#if 0
    /** 左腿 VMC 反解得到的本周期未滤波地面支持力，单位 N。 */
    const float left_ground_force_sample_n =
        VMC_Get_Ground_F0(&dependencies_.left_vmc);
    /** 右腿 VMC 反解得到的本周期未滤波地面支持力，单位 N。 */
    const float right_ground_force_sample_n =
        VMC_Get_Ground_F0(&dependencies_.right_vmc);

    dependencies_.left_filtered_ground_force_n =
        config_.ground_force_filter_alpha * left_ground_force_sample_n
        + (1.0f - config_.ground_force_filter_alpha)
            * dependencies_.left_filtered_ground_force_n;
    dependencies_.right_filtered_ground_force_n =
        config_.ground_force_filter_alpha * right_ground_force_sample_n
        + (1.0f - config_.ground_force_filter_alpha)
            * dependencies_.right_filtered_ground_force_n;

    UpdateCounter(dependencies_.left_filtered_ground_force_n,
                  dependencies_.left_off_ground_count);
    UpdateCounter(dependencies_.right_filtered_ground_force_n,
                  dependencies_.right_off_ground_count);

    /** 离地检测对命令和兼容状态的本周期结果。 */
    OffGroundUpdateResult result{};
    result.command = nominal_command;
    result.left_airborne =
        dependencies_.left_off_ground_count >= config_.active_count_threshold;
    result.right_airborne =
        dependencies_.right_off_ground_count >= config_.active_count_threshold;
    result.request_short_leg_lock = result.left_airborne && result.right_airborne;

    if (result.left_airborne)
    {
        dependencies_.left_base_leg_torque_nm =
            -dependencies_.lqr_gain_matrix[2][4]
                * (state.left_leg.body_angle_rad
                   - dependencies_.centered_body_leg_angle_rad)
            -dependencies_.lqr_gain_matrix[2][5]
                * state.left_leg.body_angular_rate_radps;
        dependencies_.left_base_leg_torque_nm *= config_.airborne_leg_torque_scale;

        result.command.left_wheel_torque_nm = 0.0f;
        result.command.left_support_force_n =
            dependencies_.left_length_pid.output
            * config_.airborne_support_force_scale;
        result.command.left_leg_torque_nm = dependencies_.left_base_leg_torque_nm;
    }

    if (result.right_airborne)
    {
        dependencies_.right_base_leg_torque_nm =
            -dependencies_.lqr_gain_matrix[3][6]
                * (state.right_leg.body_angle_rad
                   - dependencies_.centered_body_leg_angle_rad)
            -dependencies_.lqr_gain_matrix[3][7]
                * state.right_leg.body_angular_rate_radps;
        dependencies_.right_base_leg_torque_nm *= config_.airborne_leg_torque_scale;

        result.command.right_wheel_torque_nm = 0.0f;
        result.command.right_support_force_n =
            dependencies_.right_length_pid.output
            * config_.airborne_support_force_scale;
        result.command.right_leg_torque_nm = -dependencies_.right_base_leg_torque_nm;
    }

    result.request_distance_reset = result.left_airborne || result.right_airborne;
    result.request_step_cooldown_refresh = result.request_distance_reset;
    return result;
#endif
}

} // namespace chassis
