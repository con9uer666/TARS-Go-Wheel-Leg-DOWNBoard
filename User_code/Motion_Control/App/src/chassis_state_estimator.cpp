/**
 * @file chassis_state_estimator.cpp
 * @brief ChassisStateEstimator 的固定解算顺序与状态快照复制实现。
 */

#include "chassis_state_estimator.hpp"

namespace chassis
{

/**
 * @brief 保存四个解算函数和反馈对象引用。
 * @param[in] dependencies 输入更新、状态解算入口及反馈量。
 */
ChassisStateEstimator::ChassisStateEstimator(
    const ChassisStateEstimatorDependencies& dependencies)
    : dependencies_(dependencies)
{
}

/**
 * @brief 执行 Keyboard→VMC→BodySpeed→INS，并组装本周期只读状态快照。
 * @param[in,out] state 需要更新的任务状态；已有轮端速度、加速度和原始转速保持不变。
 */
void ChassisStateEstimator::Update(ChassisStateSnapshot& state) const
{
    dependencies_.update_user_input();
    dependencies_.update_vmc_geometry();
    dependencies_.update_body_speed();
    dependencies_.update_ins();

    state.left_leg.length_m = dependencies_.left_vmc.L0;
    state.left_leg.length_rate_mps = dependencies_.left_vmc.d_L0;
    state.left_leg.leg_angle_rad = dependencies_.left_vmc.phi0;
    state.left_leg.leg_angular_rate_radps = dependencies_.left_vmc.d_phi0;
    state.left_leg.body_angle_rad = dependencies_.left_vmc.b_phi0;
    state.left_leg.body_angular_rate_radps = dependencies_.left_vmc.d_b_phi0;
    state.left_leg.actual_leg_torque_nm = dependencies_.left_vmc.T_actual;

    state.right_leg.length_m = dependencies_.right_vmc.L0;
    state.right_leg.length_rate_mps = dependencies_.right_vmc.d_L0;
    state.right_leg.leg_angle_rad = dependencies_.right_vmc.phi0;
    state.right_leg.leg_angular_rate_radps = dependencies_.right_vmc.d_phi0;
    state.right_leg.body_angle_rad = dependencies_.right_vmc.b_phi0;
    state.right_leg.body_angular_rate_radps = dependencies_.right_vmc.d_b_phi0;
    state.right_leg.actual_leg_torque_nm = dependencies_.right_vmc.T_actual;

    state.body_speed_mps = dependencies_.estimated_body_speed_mps;
    state.pitch_rad = dependencies_.pitch_transform[0];
    state.pitch_angle_deg = dependencies_.raw_pitch_deg;
    state.pitch_rate_radps = dependencies_.pitch_rate_radps;
    state.yaw_rate_radps = dependencies_.yaw_rate_radps;
    state.roll_angle_deg = dependencies_.raw_roll_deg;
}

} // namespace chassis
