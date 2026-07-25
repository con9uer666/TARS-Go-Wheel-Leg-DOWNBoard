/** @file chassis_height_controller.cpp @brief ChassisHeightController 实现。 */
#include "chassis_height_controller.hpp"
#include "arm_math.h"
extern "C"
{
#include "observe_task.h"
}

/** @brief 伸腿时每周期腿长目标增量兼容调试符号，单位 m。 */
extern "C" float ramp_target_L0_up = 0.00085f;
/** @brief 缩腿时每周期腿长目标减量兼容调试符号，单位 m。 */
extern "C" float ramp_target_L0_down = 0.0010f;

namespace chassis
{
/** @brief 保存高度控制所需的外部引用。 @param[in] dependencies PID、腿长边界与兼容目标。 */
ChassisHeightController::ChassisHeightController(const ChassisHeightControllerDependencies& dependencies)
    : dependencies_(dependencies) {}

/**
 * @brief 更新横滚补偿、不对称腿长斜坡和左右腿 PID。
 * @param[in] state 腿长与横滚角状态快照。
 * @param[in] target_leg_state 上位机腿长档位。
 * @return 左右腿长力、横滚补偿和最新腿长目标。
 */
ChassisHeightOutput ChassisHeightController::Update(const ChassisStateSnapshot& state,
                                                    std::uint8_t target_leg_state)
{
    PID_Set_Error(&dependencies_.roll_compensation_pid, state.roll_angle_deg, 0.0f);
    PID_coculate(&dependencies_.roll_compensation_pid);

    if (dependencies_.leg_state_delay_cycles > 0)
        --dependencies_.leg_state_delay_cycles;

    /** 腿长档位在最短与最长腿之间线性映射得到的原始目标。 */
    const float requested_leg_length_m =
        (target_leg_state / 1.0f)
            * (dependencies_.maximum_leg_length_m - dependencies_.minimum_leg_length_m)
        + dependencies_.minimum_leg_length_m;
    /** 根据伸腿或缩腿方向选择的本周期斜坡步长。 */
    const float ramp_step_m = requested_leg_length_m > dependencies_.target_leg_length_m
        ? dependencies_.extension_ramp_step_m
        : dependencies_.retraction_ramp_step_m;
    dependencies_.target_leg_length_m = RAMP_float(
        requested_leg_length_m, dependencies_.target_leg_length_m, ramp_step_m);

    if (dependencies_.target_leg_length_m >= dependencies_.maximum_leg_length_m)
        dependencies_.target_leg_length_m = dependencies_.maximum_leg_length_m;
    if (dependencies_.target_leg_length_m <= dependencies_.minimum_leg_length_m)
        dependencies_.target_leg_length_m = dependencies_.minimum_leg_length_m;

    dependencies_.left_target_leg_length_m = dependencies_.target_leg_length_m;
    dependencies_.right_target_leg_length_m = dependencies_.target_leg_length_m;
    if (dependencies_.left_target_leg_length_m >= dependencies_.maximum_leg_length_m)
        dependencies_.left_target_leg_length_m = dependencies_.maximum_leg_length_m;
    if (dependencies_.left_target_leg_length_m <= dependencies_.minimum_leg_length_m)
        dependencies_.left_target_leg_length_m = dependencies_.minimum_leg_length_m;
    if (dependencies_.right_target_leg_length_m >= dependencies_.maximum_leg_length_m)
        dependencies_.right_target_leg_length_m = dependencies_.maximum_leg_length_m;
    if (dependencies_.right_target_leg_length_m <= dependencies_.minimum_leg_length_m)
        dependencies_.right_target_leg_length_m = dependencies_.minimum_leg_length_m;

    PID_Set_Error(&dependencies_.left_length_pid, state.left_leg.length_m,
                  dependencies_.left_target_leg_length_m);
    PID_Set_Error(&dependencies_.right_length_pid, state.right_leg.length_m,
                  dependencies_.right_target_leg_length_m);
    PID_coculate(&dependencies_.left_length_pid);
    PID_coculate(&dependencies_.right_length_pid);

    /** 本周期高度和横滚控制输出。 */
    ChassisHeightOutput output{};
    output.left_length_force_n = dependencies_.left_length_pid.output;
    output.right_length_force_n = dependencies_.right_length_pid.output;
    output.roll_compensation_n = dependencies_.roll_compensation_pid.output;
    output.target_leg_length_m = dependencies_.target_leg_length_m;
    return output;
}
} // namespace chassis
