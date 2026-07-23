/**
 * @file leg_turn_recovery_controller.cpp
 * @brief 启动恢复和上台阶收腿共用的单腿转角卡住恢复状态机。
 *
 * 左右腿各由一个对象保存路径、驻留计数和长路径积分；两个上层动作控制器
 * 共享同一对对象，不再传递子状态、计数器和积分量指针。
 */

#include "leg_turn_recovery_controller.hpp"

extern "C"
{
#include "chassis_behavior_tree.h"
#include "user_pid.h"
#include "Motor_Drv.h"
#include "Gimbal.h"
#include "User_State.h"
#include "arm_math.h"
#include "USER_CAN.h"
#include "VMC.h"
#include "observe_task.h"
#include "Leg_Control.h"
#include "Self_Righting.h"
#include "Board2Board.h"
#include "Slope.h"
#include "Wheel_Leg_about.h"
#include "controller.h"
#include "remoter.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "fdcan.h"
#include <math.h>
#include <stdint.h>
#include "imu_temp_ctrl.h"
#include "Angle_about.h"
#include "PowerCtrl.h"
#include "Gas_Spring.h"
#include "buzzer.h"
#include "Wheel_End_Velocity.h"
}

/** @brief 左腿外层收腿阶段：0 收腿长、1 转腿角、2 单腿完成。 */
uint8_t L_Leg_State;
/** @brief 右腿外层收腿阶段：0 收腿长、1 转腿角、2 单腿完成。 */
uint8_t R_Leg_State;
/** @brief 左腿当前外层阶段的连续到位周期计数。 */
uint16_t L_Ready_Count;
/** @brief 右腿当前外层阶段的连续到位周期计数。 */
uint16_t R_Ready_Count;

/**
 * @brief 固定单腿机构符号约定并初始化为最短路径子状态。
 * @param[in] is_right true 为右腿，false 为左腿。
 */
chassis::LegTurnRecoveryController::LegTurnRecoveryController(bool is_right)
    : is_right_(is_right)
{
}

/**
 * @brief 执行一次最短路径串级 PID 或卡住后的反向长路径速度控制。
 * @param[in,out] vmc 当前腿 VMC 状态及底层卡住检测器状态。
 * @param[in] target_angle_rad 目标虚拟腿角，单位 rad。
 * @param[in,out] angle_position_pid 腿角位置外环 PID。
 * @param[in,out] angle_velocity_pid 腿角速度内环 PID。
 * @return 本周期到位标志和完成机构符号适配后的腿力矩。
 */
chassis::LegTurnRecoveryUpdateResult chassis::LegTurnRecoveryController::Update(
    VMC_t& vmc,
    float target_angle_rad,
    user_pid_t& angle_position_pid,
    user_pid_t& angle_velocity_pid)
{
    ++dwell_cycles_;
    /** 本周期输出给上层收腿流程的到位标志和腿力矩。 */
    LegTurnRecoveryUpdateResult result{};

    if (path_ == LegTurnRecoveryPath::Forward)
    {
        // 短路径 PID 串级：Middle_PID 出 dphi0 目标，dphi0_PID 出转矩
        PID_Set_AngleError(&angle_position_pid, vmc.phi0, target_angle_rad);
        PID_coculate(&angle_position_pid);
        /** 保留右腿速度反馈镜像后的角速度环输入。 */
        const float angular_rate_input = is_right_ ? -vmc.d_phi0 : vmc.d_phi0;
        /** 保留右腿位置环输出镜像后的目标角速度。 */
        const float angular_rate_target =
            is_right_ ? -angle_position_pid.output : angle_position_pid.output;
        PID_Set_Error(&angle_velocity_pid, angular_rate_input, angular_rate_target);
        PID_coculate(&angle_velocity_pid);
        result.torque_nm = is_right_ ? -angle_velocity_pid.output
                                     : angle_velocity_pid.output;

        // 角度误差进入容差带 → 认为到位
        if (fabsf(angle_position_pid.error) <= 0.1f)
            result.near_target = true;

        // 卡住判据：dwell 门禁过了 + 误差仍大 + d_phi0 死区持续时间达到 → 切 REV
        if (!result.near_target && dwell_cycles_ > 100 &&
            fabsf(angle_position_pid.error) > 0.1f &&
            leg_turn_stuck_detect(&vmc, 0.0873f, 0.1f))
        {
            /** 当前目标相对腿角的最短路径有符号误差，单位 rad。 */
            const float shortest_error = ShortestAngleDelta(target_angle_rad, vmc.phi0);
            reverse_direction_ = shortest_error > 0.0f ? -1.0f : 1.0f;
            reverse_path_length_rad_ = 2.0f * PI - fabsf(shortest_error);
            reverse_traveled_rad_ = 0.0f;
            path_ = LegTurnRecoveryPath::Reverse;
            dwell_cycles_ = 0;
            leg_turn_stuck_reset(&vmc);
            result.torque_nm = 0.0f;
        }
    }
    else
    {
        // 纯速度控制走长路径，不受 ShortestAngle 限制
        result.torque_nm =
            leg_turn_speed_control(&vmc, reverse_direction_ * 1.2f, 15.0f, 0.0f);
        // 用 d_phi0 积分记已走弧长，避免 phi0 在 ±π 翻转导致的跳变
        reverse_traveled_rad_ += reverse_direction_ * vmc.d_phi0 * 0.002f;

        /** 当前腿角相对目标的最短路径误差绝对值，单位 rad。 */
        const float current_shortest_error =
            fabsf(ShortestAngleDelta(target_angle_rad, vmc.phi0));
        // 到位判定：主判用积分弧长，兜底用过半圈后短路径误差，防积分长期漂移
        /** 长路径积分完成或过半圈后短路径误差已收敛的到位判定。 */
        const bool arrived =
            reverse_traveled_rad_ >= reverse_path_length_rad_ - 0.05f
            || (reverse_traveled_rad_ > PI && current_shortest_error < 0.1f);
        if (arrived)
        {
            result.near_target = true;
            path_ = LegTurnRecoveryPath::Forward;
            dwell_cycles_ = 0;
            leg_turn_stuck_reset(&vmc);
        }
        else if (dwell_cycles_ > 100 &&
                 reverse_traveled_rad_ < reverse_path_length_rad_ - 0.1f &&
                 leg_turn_stuck_detect(&vmc, 0.0873f, 0.1f))
        {
            // 反向也卡住（且确认不是快到位的低速）→ 切回 FWD，无限重试
            path_ = LegTurnRecoveryPath::Forward;
            dwell_cycles_ = 0;
            leg_turn_stuck_reset(&vmc);
        }
    }
    return result;
}

/**
 * @brief 清除长路径恢复状态并复位当前腿的底层卡住检测计数。
 * @param[in,out] vmc 需要同步复位卡住检测器的腿 VMC 对象。
 */
void chassis::LegTurnRecoveryController::Reset(VMC_t& vmc)
{
    path_ = LegTurnRecoveryPath::Forward;
    dwell_cycles_ = 0;
    reverse_direction_ = 0.0f;
    reverse_path_length_rad_ = 0.0f;
    reverse_traveled_rad_ = 0.0f;
    leg_turn_stuck_reset(&vmc);
}
