/**
 * @file chassis_control_task.hpp
 * @brief 底盘控制任务的 C++ 调度器与运行上下文。
 */

#ifndef CHASSIS_CONTROL_TASK_HPP
#define CHASSIS_CONTROL_TASK_HPP

#include <cstdint>
#include "chassis_control_types.hpp"
#include "balance_controller.hpp"
#include "sit_controller.hpp"
#include "stair_controller.hpp"
#include "startup_retract_controller.hpp"
#include "gravity_compensation_test_controller.hpp"
#include "leg_turn_recovery_controller.hpp"
#include "chassis_state_estimator.hpp"
#include "chassis_initializer.hpp"
#include "chassis_motor_enabler.hpp"

namespace chassis
{

/**
 * @brief Motor_task 的私有持久状态。
 *
 * context_ 的生命周期与 Motor_task 相同，不对其他模块暴露。这样可以逐步替换
 * 原来散落在文件作用域的静态变量和调试全局变量。
 */
struct TaskContext
{
    ChassisStateSnapshot state;                 /**< 当前控制周期的只读状态快照。 */
    LegacyModeSignals mode_signals;             /**< 旧模式标志的单周期快照。 */
    ChassisCommand command;                     /**< 周期末将提交到 VMC 的统一命令。 */
    std::uint16_t gravity_test_delay_count = 0; /**< 重力测试启动等待计数，单位控制周期。 */
    RunMode mode = RunMode::Hold;               /**< 本周期已经解析出的唯一顶层模式。 */
    ModeTransitionRequest transition_request = ModeTransitionRequest::None; /**< 本周期控制结果请求的下一顶层模式。 */
};

/**
 * @brief 500 Hz 底盘控制任务调度器。
 *
 * 当前灰度阶段负责固定执行顺序、模式归一化、私有状态封装和命令统一提交。
 * 起立恢复、正常平衡、坐地和上台阶已迁移为 C++ 控制器，其余分支通过旧 C 兼容桥运行。
 */
class ChassisControlTask
{
public:
    /** @brief 使用当前工程的全局 PID 依赖构造任务及其 C++ 动作控制器。 */
    ChassisControlTask();

    /** @brief 完成初始化后永久运行 500 Hz 控制循环；该函数不会返回。 */
    void Run();

private:
    /** @brief 按原顺序初始化电机、VMC、PID，并完成上电等待和电机使能。 */
    void Initialize();
    /** @brief 执行一次完整的 2 ms 控制周期。 */
    void RunCycle();
    /** @brief 更新左右轮端速度与加速度，并写入 context_.state.wheel。 */
    void UpdateWheelState();
    /** @brief 将旧模式标志锁存到 context_.mode_signals。 */
    void CaptureModeSignals();
    /**
     * @brief 将旧模式标志组合转换为唯一的 RunMode。
     * @param[in] signals 本周期锁存的旧模式标志。
     * @return 与旧 if/else 优先级等价的顶层运行模式。
     */
    RunMode ResolveMode(const LegacyModeSignals& signals) const;
    /**
     * @brief 执行指定模式，并标记命令来自旧 C 动作还是新 C++ 控制器。
     * @param[in] mode 本周期需要执行的顶层模式。
     */
    void ExecuteMode(RunMode mode);
    /**
     * @brief 在唯一位置把语义化模式请求提交到旧 start_mode 兼容变量。
     * @param[in] request 本周期控制器最终产生的顶层模式转换请求。
     */
    void ApplyModeTransition(ModeTransitionRequest request);
    /** @brief 执行带 1000 周期启动等待的重力补偿测试。 */
    void ExecuteGravityTest();
    /**
     * @brief 将统一命令复制到现有 VMC_Chassis_Target 兼容结构。
     * @param[in] command 本周期最终映射前命令。
     */
    void CommitCommand(const ChassisCommand& command);
    /** @brief 提交统一命令、执行最终 VMC 映射，并更新错误蜂鸣器。 */
    void ApplyOutputs();

    TaskContext context_{};        /**< 仅由 Motor_task 对象拥有的任务持久状态。 */
    RunMode previous_mode_ = RunMode::StartupRetract;  /**< 上一周期有效模式，用于检测模式切换边沿。 */
    ChassisStateEstimator state_estimator_; /**< 固定输入/VMC/车速/INS 顺序并组装只读快照的估计器。 */
    LegTurnRecoveryController left_leg_turn_recovery_; /**< 左腿收腿转角短路径/反向长路径共享状态机。 */
    LegTurnRecoveryController right_leg_turn_recovery_; /**< 右腿收腿转角短路径/反向长路径共享状态机。 */
    BalanceController balance_controller_; /**< 固定正常平衡算法顺序并返回命令与模式事件的 C++ 控制器。 */
    SitController sit_controller_; /**< 首个原生 C++ 动作控制器：坐地控制器。 */
    StairController stair_controller_; /**< 管理伸腿和收腿内部阶段的 C++ 上台阶控制器。 */
    StartupRetractController startup_retract_controller_; /**< 管理倒地自起接管与起立前收腿恢复的 C++ 控制器。 */
    GravityCompensationTestController gravity_test_controller_; /**< 管理机械量程探测、姿态扫描与结果遍历的标定控制器。 */
};

} // namespace chassis

#endif // CHASSIS_CONTROL_TASK_HPP
