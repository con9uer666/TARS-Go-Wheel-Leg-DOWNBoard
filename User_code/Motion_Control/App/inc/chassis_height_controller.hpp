/**
 * @file chassis_height_controller.hpp
 * @brief 横滚补偿与左右腿长目标斜坡的 C++ 控制器。
 */
#ifndef CHASSIS_HEIGHT_CONTROLLER_HPP
#define CHASSIS_HEIGHT_CONTROLLER_HPP

#include "chassis_control_types.hpp"
extern "C"
{
#include "user_pid.h"
}

extern "C"
{
extern float ramp_target_L0_up;   /**< 伸腿时每控制周期的腿长目标最大增量，单位 m。 */
extern float ramp_target_L0_down; /**< 缩腿时每控制周期的腿长目标最大减量，单位 m。 */
}

namespace chassis
{
/** @brief 机身高度控制器使用的 PID、腿长端点和兼容目标量。 */
struct ChassisHeightControllerDependencies
{
    user_pid_t& roll_compensation_pid; /**< 车身横滚角回零 PID。 */
    user_pid_t& left_length_pid;       /**< 左腿正常高度控制 PID。 */
    user_pid_t& right_length_pid;      /**< 右腿正常高度控制 PID。 */
    float& minimum_leg_length_m;       /**< 腿长目标下限，单位 m。 */
    float& maximum_leg_length_m;       /**< 腿长目标上限，单位 m。 */
    float& target_leg_length_m;        /**< 斜坡后的统一腿长目标 target_Leg_L0。 */
    float& left_target_leg_length_m;   /**< 左腿兼容目标 target_L_Leg_L0。 */
    float& right_target_leg_length_m;  /**< 右腿兼容目标 target_R_Leg_L0。 */
    int& leg_state_delay_cycles;       /**< 每周期递减到零的腿长档位延时计数 leg_state_count。 */
    float& extension_ramp_step_m;      /**< 伸腿时每周期目标增量。 */
    float& retraction_ramp_step_m;     /**< 缩腿时每周期目标减量。 */
};

/** @brief 横滚补偿和腿长 PID 更新后的单周期输出。 */
struct ChassisHeightOutput
{
    float left_length_force_n = 0.0f;  /**< 左腿腿长 PID 输出，单位 N。 */
    float right_length_force_n = 0.0f; /**< 右腿腿长 PID 输出，单位 N。 */
    float roll_compensation_n = 0.0f;  /**< 左右腿以相反符号使用的横滚补偿输出。 */
    float target_leg_length_m = 0.0f;  /**< 本周期斜坡更新后的统一腿长目标。 */
};

/** @brief 维持横滚水平并以不对称斜坡跟踪长/短腿档位的控制器。 */
class ChassisHeightController
{
public:
    /**
     * @brief 构造高度控制器并绑定 PID、腿长目标和斜坡参数。
     * @param[in] dependencies 生命周期覆盖控制器的外部引用。
     */
    explicit ChassisHeightController(const ChassisHeightControllerDependencies& dependencies);

    /**
     * @brief 按旧顺序先更新横滚 PID，再更新腿长斜坡和左右腿 PID。
     * @param[in] state 包含左右腿当前长度和车身横滚角的状态快照。
     * @param[in] target_leg_state 腿长档位，旧逻辑按 0~1 线性映射到腿长区间。
     * @return 左右腿长 PID、横滚补偿和斜坡后腿长目标。
     */
    ChassisHeightOutput Update(const ChassisStateSnapshot& state,
                               std::uint8_t target_leg_state);

private:
    ChassisHeightControllerDependencies dependencies_; /**< PID、腿长边界、兼容目标和斜坡参数引用。 */
};
} // namespace chassis
#endif // CHASSIS_HEIGHT_CONTROLLER_HPP
