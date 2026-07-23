/**
 * @file chassis_state_estimator.hpp
 * @brief 底盘输入更新、VMC/车速/INS 解算和只读状态快照组装接口。
 */

#ifndef CHASSIS_STATE_ESTIMATOR_HPP
#define CHASSIS_STATE_ESTIMATOR_HPP

#include "chassis_control_types.hpp"

extern "C"
{
#include "VMC.h"
}

namespace chassis
{

/** @brief 单次状态解算步骤使用的无参数 C 函数签名。 */
using StateEstimationStepFunction = void (*)(void);

/**
 * @brief ChassisStateEstimator 使用的四个解算步骤和全部反馈引用。
 *
 * 函数入口与数据对象均由任务装配层提供，使估计器本身不通过名称查找工程全局量。
 */
struct ChassisStateEstimatorDependencies
{
    StateEstimationStepFunction update_user_input; /**< 更新键鼠/遥控输入的 Keyboard_Simulate。 */
    StateEstimationStepFunction update_vmc_geometry; /**< 更新腿部几何与速度的 VMC_Coculate。 */
    StateEstimationStepFunction update_body_speed; /**< 更新车身速度估计的 Body_Speed_Coculate。 */
    StateEstimationStepFunction update_ins; /**< 更新姿态变换量的 INS_Coculate。 */
    VMC_t& left_vmc;  /**< 左腿 VMC 反馈与反解状态。 */
    VMC_t& right_vmc; /**< 右腿 VMC 反馈与反解状态。 */
    float& estimated_body_speed_mps; /**< 卡尔曼车身速度 kalman_body_speed，单位 m/s。 */
    float (&pitch_transform)[2]; /**< 俯仰角变换数组 pitch_trans；索引 0 为当前弧度值。 */
    float& pitch_rate_radps; /**< 俯仰角速度 d_pitch，单位 rad/s。 */
    float& yaw_rate_radps; /**< 偏航角速度 d_yaw，单位 rad/s。 */
    float& raw_pitch_deg; /**< IMU 原始 pitch，单位 deg。 */
    float& raw_roll_deg; /**< IMU 原始 roll，单位 deg。 */
};

/**
 * @brief 固定执行输入、VMC、车速和 INS 顺序并更新状态快照的估计器。
 */
class ChassisStateEstimator
{
public:
    /**
     * @brief 构造估计器并保存解算函数和反馈对象的非拥有引用。
     * @param[in] dependencies 生命周期必须覆盖估计器的解算入口与反馈对象。
     */
    explicit ChassisStateEstimator(const ChassisStateEstimatorDependencies& dependencies);

    /**
     * @brief 按旧顺序执行四个状态步骤，并覆盖快照中的腿部与车体字段。
     * @param[in,out] state 轮端字段由任务提前写入；本函数保留轮端字段并更新其余状态。
     */
    void Update(ChassisStateSnapshot& state) const;

private:
    ChassisStateEstimatorDependencies dependencies_; /**< 四个解算入口和全部反馈对象引用。 */
};

} // namespace chassis

#endif // CHASSIS_STATE_ESTIMATOR_HPP
