/**
 * @file anti_split_controller.cpp
 * @brief AntiSplitController 的腿长增益插值与力矩合成实现。
 */

#include "anti_split_controller.hpp"

namespace chassis
{

/**
 * @brief 保存防劈叉控制所需的外部对象引用和参数副本。
 * @param[in] dependencies 防劈叉/旋转 PID、腿长端点和离心补偿参数。
 * @param[in] config Kp/Kd 端点和腿角中心值。
 */
AntiSplitController::AntiSplitController(
    const AntiSplitControllerDependencies& dependencies,
    const AntiSplitControllerConfig& config)
    : dependencies_(dependencies), config_(config)
{
}

/**
 * @brief 用最短腿和最长腿端点对 Kp/Kd 做一次线性插值。
 * @param[in] average_leg_length_m 左右腿长平均值，单位 m。
 */
void AntiSplitController::UpdateGains(float average_leg_length_m)
{
    /** 当前平均腿长在最短与最长腿端点之间的线性比例，保留旧逻辑不限幅。 */
    const float leg_length_ratio =
        (average_leg_length_m - dependencies_.minimum_leg_length_m)
        / (dependencies_.maximum_leg_length_m - dependencies_.minimum_leg_length_m);

    dependencies_.anti_split_pid.Kp = config_.short_leg_kp
        + (config_.long_leg_kp - config_.short_leg_kp) * leg_length_ratio;
    dependencies_.anti_split_pid.Kd = config_.short_leg_kd
        + (config_.long_leg_kd - config_.short_leg_kd) * leg_length_ratio;
}

/**
 * @brief 依次叠加双腿夹角 PID、d_yaw² 离心补偿和小陀螺 phi0 归中 PID。
 * @param[in] state 本周期左右腿长和虚拟腿角快照。
 * @param[in] input LQR 基础腿力矩、yaw 角速度和旋转状态。
 * @return 可直接作为 VMC 左右腿 T 目标的最终力矩。
 */
AntiSplitOutput AntiSplitController::Update(
    const ChassisStateSnapshot& state,
    const AntiSplitControlInput& input)
{
    /** 左右腿虚拟腿长平均值，单位 m。 */
    const float average_leg_length_m =
        (state.left_leg.length_m + state.right_leg.length_m) * 0.5f;
    UpdateGains(average_leg_length_m);

    PID_Set_Error(
        &dependencies_.anti_split_pid,
        (state.right_leg.leg_angle_rad - config_.centered_leg_angle_rad)
            + (state.left_leg.leg_angle_rad - config_.centered_leg_angle_rad),
        0.0f);
    /** 双腿夹角误差 PID 产生的左右腿共同补偿力矩。 */
    const float anti_split_torque_nm = PID_coculate(&dependencies_.anti_split_pid);
    /** 与旋转方向无关的 d_yaw² 离心补偿力矩。 */
    const float centrifugal_compensation_nm =
        dependencies_.centrifugal_compensation_gain
        * input.yaw_rate_radps
        * input.yaw_rate_radps;

    /** 防劈叉与离心补偿叠加后的左右腿力矩。 */
    AntiSplitOutput output{};
    output.left_leg_torque_nm = input.lqr_output.left_base_leg_torque_nm
        + anti_split_torque_nm - centrifugal_compensation_nm;
    output.right_leg_torque_nm = -input.lqr_output.right_base_leg_torque_nm
        + anti_split_torque_nm - centrifugal_compensation_nm;

    if (input.spinning_active)
    {
        PID_Set_AngleError(&dependencies_.left_spin_angle_pid,
                           state.left_leg.leg_angle_rad,
                           dependencies_.spin_target_leg_angle_rad);
        PID_Set_AngleError(&dependencies_.right_spin_angle_pid,
                           state.right_leg.leg_angle_rad,
                           dependencies_.spin_target_leg_angle_rad);

        if (!spin_angle_pid_started_)
        {
            dependencies_.left_spin_angle_pid.pre_error =
                dependencies_.left_spin_angle_pid.error;
            dependencies_.right_spin_angle_pid.pre_error =
                dependencies_.right_spin_angle_pid.error;
            spin_angle_pid_started_ = true;
        }

        output.left_leg_torque_nm +=
            PID_coculate(&dependencies_.left_spin_angle_pid);
        output.right_leg_torque_nm +=
            PID_coculate(&dependencies_.right_spin_angle_pid);
    }
    else
    {
        PID_Clear(&dependencies_.left_spin_angle_pid);
        PID_Clear(&dependencies_.right_spin_angle_pid);
        dependencies_.left_spin_angle_pid.output = 0.0f;
        dependencies_.right_spin_angle_pid.output = 0.0f;
        spin_angle_pid_started_ = false;
    }

    return output;
}

} // namespace chassis
