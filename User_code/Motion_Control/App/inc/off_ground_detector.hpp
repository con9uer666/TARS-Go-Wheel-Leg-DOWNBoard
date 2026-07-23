/**
 * @file off_ground_detector.hpp
 * @brief 地面支持力滤波、离地计数和单腿命令覆盖的 C++ 检测器。
 */

#ifndef OFF_GROUND_DETECTOR_HPP
#define OFF_GROUND_DETECTOR_HPP

#include <cstdint>
#include "chassis_control_types.hpp"

extern "C"
{
#include "user_pid.h"
#include "VMC.h"
}

namespace chassis
{

/** @brief OffGroundDetector 使用的 VMC、PID、LQR 增益和兼容调试状态。 */
struct OffGroundDetectorDependencies
{
    VMC_t& left_vmc;                     /**< 左腿完整 VMC 状态，供 VMC_Get_Ground_F0() 反解地面支持力。 */
    VMC_t& right_vmc;                    /**< 右腿完整 VMC 状态。 */
    user_pid_t& left_length_pid;         /**< 左腿离地时用于生成 0.5 倍支持力的腿长 PID。 */
    user_pid_t& right_length_pid;        /**< 右腿离地时使用的腿长 PID。 */
    float (&lqr_gain_matrix)[4][12];     /**< 离地腿归中使用的 4×12 LQR 增益矩阵。 */
    float& left_base_leg_torque_nm;      /**< 左腿离地覆盖后同步更新的 Leg_L_T 兼容调试量。 */
    float& right_base_leg_torque_nm;     /**< 右腿离地覆盖后同步更新的 Leg_R_T 兼容调试量。 */
    float& centered_body_leg_angle_rad;  /**< 离地左右腿 b_phi0 归中使用的共同偏置。 */
    float& left_filtered_ground_force_n; /**< 左腿滤波后地面支持力 L_Ground_F0，单位 N。 */
    float& right_filtered_ground_force_n;/**< 右腿滤波后地面支持力 R_Ground_F0，单位 N。 */
    int& left_off_ground_count;          /**< 左腿离地滞回计数 L_off_ground。 */
    int& right_off_ground_count;         /**< 右腿离地滞回计数 R_off_ground。 */
};

/** @brief 支持力滤波、离地计数和命令覆盖的参数。 */
struct OffGroundDetectorConfig
{
    float ground_force_filter_alpha = 0.1f; /**< 新支持力样本在一阶低通中的权重。 */
    float count_increment_force_n = 50.0f;  /**< 滤波支持力不大于该值时离地计数加 1，单位 N。 */
    float count_decrement_force_n = 10.0f;  /**< 在未命中加计数分支时，支持力不小于该值则计数减 1。 */
    int count_maximum = 20;                 /**< 单腿离地滤波计数上限。 */
    int active_count_threshold = 10;        /**< 计数达到该值时启用单腿离地命令覆盖。 */
    float airborne_leg_torque_scale = 0.7f; /**< 离地腿 b_phi0 归中力矩的缩放比例。 */
    float airborne_support_force_scale = 0.5f; /**< 离地腿长 PID 输出的支持力缩放比例。 */
};

/** @brief 离地检测对本周期命令的覆盖结果与兼容状态事件。 */
struct OffGroundUpdateResult
{
    ChassisCommand command;             /**< 按左右腿离地状态完成局部覆盖后的六维命令。 */
    bool left_airborne = false;         /**< true 表示左腿计数已达到离地阈值。 */
    bool right_airborne = false;        /**< true 表示右腿计数已达到离地阈值。 */
    bool request_short_leg_lock = false;/**< true 表示双腿同时离地，需锁定上位机腿长档位为短腿。 */
    bool request_distance_reset = false;/**< true 表示任一腿离地，需清零实际与目标位移积分。 */
    bool request_step_cooldown_refresh = false; /**< true 表示任一腿离地，需刷新磕台阶检测冷却。 */
};

/**
 * @brief 滤波左右地面支持力，更新离地计数，并仅覆盖离地侧的命令。
 */
class OffGroundDetector
{
public:
    /**
     * @brief 构造离地检测器并绑定 VMC、PID、K 矩阵和兼容调试状态。
     * @param[in] dependencies 生命周期必须覆盖检测器的外部对象引用。
     * @param[in] config 支持力滤波、离地计数和命令缩放参数。
     */
    OffGroundDetector(const OffGroundDetectorDependencies& dependencies,
                      const OffGroundDetectorConfig& config = OffGroundDetectorConfig{});

    /**
     * @brief 执行一次支持力滤波、离地判定和单腿命令覆盖。
     * @param[in] state 包含左右腿 b_phi0 及角速度的状态快照。
     * @param[in] nominal_command 离地保护覆盖前的正常 Balance 命令。
     * @return 已覆盖离地侧命令的结果和需由 BalanceController 提交的兼容事件。
     */
    OffGroundUpdateResult Update(const ChassisStateSnapshot& state,
                                 const ChassisCommand& nominal_command);

private:
    /**
     * @brief 按旧阈值分支更新并限制单腿离地计数。
     * @param[in] filtered_ground_force_n 单腿滤波后地面支持力，单位 N。
     * @param[in,out] off_ground_count 需要加减并限制到 [0,count_maximum] 的离地计数。
     */
    void UpdateCounter(float filtered_ground_force_n, int& off_ground_count) const;

    OffGroundDetectorDependencies dependencies_; /**< VMC、PID、K 矩阵和兼容支持力/计数引用。 */
    OffGroundDetectorConfig config_;              /**< 支持力滤波、计数阈值和命令缩放参数副本。 */
};

} // namespace chassis

#endif // OFF_GROUND_DETECTOR_HPP
