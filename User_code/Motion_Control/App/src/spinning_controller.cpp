/**
 * @file spinning_controller.cpp
 * @brief SpinningController 与 Error_Calculate 所需同名 C 兼容入口。
 *        加速段闭环 d_yaw 到目标转速；退出段随转速降低平滑引入角度归位，均受功率门控。
 *        小陀螺时叠加在 BalanceController 腿力矩命令上的 phi0 归中 PID 也在此定义。
 */

#include "spinning_controller.hpp"
#include "arm_math.h"

extern "C"
{
#include "chassis_behavior_tree.h"
#include "user_pid.h"
#include "Motor_Drv.h"
#include "Gimbal.h"
#include "User_State.h"
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

/** @brief 小陀螺加速与退出共用的 yaw 角速度 PID。 */
user_pid_t spinning_pid;
/** @brief 小陀螺退出阶段的相对 yaw 角度 PID。 */
user_pid_t spinning_speed_pid;
/** @brief 小陀螺时左腿 phi0 归中 PID。 */
user_pid_t L_Spin_Phi0_PID;
/** @brief 小陀螺时右腿 phi0 归中 PID。 */
user_pid_t R_Spin_Phi0_PID;
/** @brief 小陀螺左右腿 phi0 归中目标，单位 rad。 */
float target_spin_phi0 = PI / 2.0f;
/** @brief 低通后的小陀螺目标 yaw 角速度。 */
float spinning_target_d_yaw_cmd = 0.0f;
/** @brief 低通后的小陀螺实际 yaw 角速度。 */
float spinning_d_yaw_feedback = 0.0f;
/** @brief 加速阶段目标角速度低通新样本权重。 */
float alpha_spinning_target_d_yaw = 0.02f;
/** @brief 加速阶段反馈角速度低通新样本权重。 */
float alpha_spinning_d_yaw = 0.2f;
/** @brief 预留的小陀螺降速目标滤波系数，当前控制律未使用。 */
float alpha_spinning_down_target_d_yaw = 0.08f;
/** @brief 预留的小陀螺停转目标滤波系数，当前控制律未使用。 */
float alpha_spinning_stop_target_d_yaw = 0.05f;

/** @brief 小陀螺运行/退出标志，1 表示旋转逻辑仍激活。 */
uint8_t spinning_flag = 0;
/** @brief 小陀螺是否允许新请求进入，1 可用，0 等待退出完成。 */
uint8_t spinning_usable = 1;

/** @brief 小陀螺标称目标 yaw 角速度，单位 rad/s。 */
float target_spinning_d_yaw = 8.0f;
/** @brief AntiSplitController 使用的 d_yaw² 离心补偿系数。 */
float centrifugal_comp_gain = 2.0f;
// 小陀螺时允许触发平移的yaw_angle_PI误差窗口(rad)：
// |yaw_angle_PI| <= 此值时，速度倍率从0线性升至+1（正向）；
// |yaw_angle_PI - ±PI| <= 此值时，倍率从0线性降至-1（反向）；
// 其余角度倍率为0。建议0.2~0.5rad
/** @brief 小陀螺平移方向匹配的 yaw_angle_PI 窗口半宽，单位 rad。 */
float spin_speed_tol_angle = 2.0f;
// 小陀螺平移方向偏置(rad)：补偿"拨杆向前-实际方向"的安装/解算偏差。
// 正负与 yaw_angle_PI 同号系：调一调正负看车的实际响应方向。建议先±0.1rad尝试。
/** @brief 小陀螺平移方向的安装/解算偏置，单位 rad。 */
float spin_speed_angle_offset = -0.9f;

namespace chassis
{
/** @brief 保存旋转 PID、滤波状态和目标参数引用。 @param[in] dependencies 外部 PID 与调参量引用。 */
SpinningController::SpinningController(const SpinningControllerDependencies& dependencies)
    : dependencies_(dependencies) {}

/** @brief 执行小陀螺加速阶段的目标/反馈低通和角速度 PID。 @param[in] input yaw 与功率门控输入。 @return yaw 控制量。 */
float SpinningController::UpdateActive(const SpinningControlInput& input)
{
    /** 超功率时经观测器缩放后的小陀螺目标角速度。 */
    const float setpoint = input.filtered_power_w > input.power_limit_w
        ? dependencies_.target_yaw_rate_radps * input.power_scale
        : dependencies_.target_yaw_rate_radps;
    dependencies_.filtered_target_yaw_rate_radps =
        dependencies_.target_filter_alpha * setpoint
        + (1.0f - dependencies_.target_filter_alpha)
            * dependencies_.filtered_target_yaw_rate_radps;
    dependencies_.filtered_yaw_rate_radps =
        dependencies_.feedback_filter_alpha * input.yaw_rate_radps
        + (1.0f - dependencies_.feedback_filter_alpha)
            * dependencies_.filtered_yaw_rate_radps;
    PID_Set_Error(&dependencies_.yaw_rate_pid,
                  dependencies_.filtered_yaw_rate_radps,
                  dependencies_.filtered_target_yaw_rate_radps);
    return PID_coculate(&dependencies_.yaw_rate_pid);
}

/** @brief 执行小陀螺退出阶段的相对角度与角速度串联控制。 @param[in] input yaw 与功率门控输入。 @return yaw 控制量。 */
float SpinningController::UpdateExit(const SpinningControlInput& input)
{
    PID_Set_Error(&dependencies_.exit_angle_pid, input.relative_yaw_angle_rad, 0.0f);
    /** 相对 yaw 角 PID 生成的退出目标角速度。 */
    const float angle_correction = PID_coculate(&dependencies_.exit_angle_pid);
    /** 保留旧控制律的角度修正目标；当前等于 1.0*angle_correction。 */
    float blended_target = 1.0f * angle_correction;
    if (input.filtered_power_w > input.power_limit_w)
        blended_target *= input.power_scale;
    PID_Set_Error(&dependencies_.yaw_rate_pid, input.yaw_rate_radps, blended_target);
    return PID_coculate(&dependencies_.yaw_rate_pid);
}
} // namespace chassis

/** @brief 获取 Error_Calculate C 兼容入口共用的唯一旋转控制器实例。 @return 静态生命周期 SpinningController 引用。 */
static chassis::SpinningController& GetSpinningController()
{
    /** 绑定现有调试全局量的唯一旋转控制器。 */
    static chassis::SpinningController controller(
        chassis::SpinningControllerDependencies{
            spinning_pid,
            spinning_speed_pid,
            spinning_target_d_yaw_cmd,
            spinning_d_yaw_feedback,
            target_spinning_d_yaw,
            alpha_spinning_target_d_yaw,
            alpha_spinning_d_yaw});
    return controller;
}

/** @brief 从现有全局反馈和功率观测量组装旋转控制输入。 @return 本周期 SpinningControlInput。 */
static chassis::SpinningControlInput CaptureSpinningInput()
{
    /** Error_Calculate 兼容入口使用的本周期旋转输入快照。 */
    chassis::SpinningControlInput input{};
    input.yaw_rate_radps = d_yaw;
    input.relative_yaw_angle_rad = yaw_angle_PI;
    input.filtered_power_w = g_filtered_power;
    input.power_limit_w = power_limit;
    input.power_scale = g_power_obs_lambda;
    return input;
}

/**
 * @brief 更新小陀螺运行阶段的旋转速度闭环。
 *
 * 对目标转速和反馈转速分别进行低通滤波，并根据功率门控缩放目标转速，
 * 最后通过 spinning_pid 计算供 LQR 使用的 yaw 控制量。本函数只负责旋转
 * 控制，不再更新 speed_error。
 *
 * @return 小陀螺旋转 PID 输出，作为本周期 yaw_error 的控制输入。
 */
float spinning_up(void)
{
    return GetSpinningController().UpdateActive(CaptureSpinningInput());
}

/**
 * @brief 更新小陀螺退出阶段的减速与角度归位控制。
 *
 * 根据 yaw_angle_PI 生成归位转速目标，超功率时按功率门控系数缩放目标，
 * 再通过 spinning_pid 计算旋转控制量。本函数不切换小陀螺状态，也不更新
 * speed_error；退出完成条件由上层 Error_Calculate() 统一判断。
 *
 * @return 小陀螺退出 PID 输出，作为本周期 yaw_error 的控制输入。
 */
float spinning_exit(void)
{
    return GetSpinningController().UpdateExit(CaptureSpinningInput());
}
