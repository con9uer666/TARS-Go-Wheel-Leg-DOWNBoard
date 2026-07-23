/**
 * @file leg_turn_recovery_controller.hpp
 * @brief 收腿转角短路径控制与卡住后反向绕长路径恢复的 C++ 状态机。
 */

#ifndef LEG_TURN_RECOVERY_CONTROLLER_HPP
#define LEG_TURN_RECOVERY_CONTROLLER_HPP

extern "C"
{
#include "user_pid.h"
#include "VMC.h"
}

namespace chassis
{

/** @brief 单腿收腿转角控制器的内部路径选择。 */
enum class LegTurnRecoveryPath
{
    Forward, /**< 位置环与速度环串级，沿最短角路径转向目标。 */
    Reverse  /**< 最短路径卡住后以恒定角速度沿反方向长路径绕行。 */
};

/** @brief 单腿转角恢复控制器一次更新的到位状态和力矩命令。 */
struct LegTurnRecoveryUpdateResult
{
    float torque_nm = 0.0f; /**< 本周期虚拟腿力矩命令，已包含右腿机构符号适配。 */
    bool near_target = false; /**< true 表示本周期进入目标角容差或长路径已到位。 */
};

/**
 * @brief 拥有单腿短路径/长路径子状态和绕行积分量的转角恢复控制器。
 */
class LegTurnRecoveryController
{
public:
    /**
     * @brief 构造单腿控制器并固定其机构符号约定。
     * @param[in] is_right true 为右腿，角速度环输入、目标和输出按旧逻辑镜像。
     */
    explicit LegTurnRecoveryController(bool is_right);

    /**
     * @brief 更新短路径串级 PID 或卡住后的反向长路径速度控制。
     * @param[in,out] vmc 当前腿 VMC 状态；卡住检测器内部计数会被更新或复位。
     * @param[in] target_angle_rad 目标虚拟腿角，单位 rad。
     * @param[in,out] angle_position_pid 腿角位置环 PID。
     * @param[in,out] angle_velocity_pid 腿角速度环 PID。
     * @return 本周期到位标志和虚拟腿力矩命令。
     */
    LegTurnRecoveryUpdateResult Update(VMC_t& vmc,
                                       float target_angle_rad,
                                       user_pid_t& angle_position_pid,
                                       user_pid_t& angle_velocity_pid);

    /**
     * @brief 清除路径、驻留计数、长路径积分和底层卡住检测状态。
     * @param[in,out] vmc 需要同时复位卡住检测器的腿 VMC 对象。
     */
    void Reset(VMC_t& vmc);

private:
    bool is_right_ = false; /**< true 表示使用右腿速度与力矩镜像符号。 */
    LegTurnRecoveryPath path_ = LegTurnRecoveryPath::Forward; /**< 当前执行最短路径还是反向长路径。 */
    int dwell_cycles_ = 0; /**< 进入当前路径后的控制周期数，用于 100 周期切换门禁。 */
    float reverse_direction_ = 0.0f; /**< 反向长路径目标角速度的方向，取 +1 或 -1。 */
    float reverse_path_length_rad_ = 0.0f; /**< 锁存的反向长路径总弧长，单位 rad。 */
    float reverse_traveled_rad_ = 0.0f; /**< 对有向 d_phi0 积分得到的已走长路径弧长，单位 rad。 */
};

} // namespace chassis

#endif // LEG_TURN_RECOVERY_CONTROLLER_HPP
