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
    CommandSource command_source = CommandSource::LegacyGlobal; /**< 当前命令的数据来源。 */
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
    /** @brief 按原顺序执行输入、VMC、车速和 INS 状态解算。 */
    void UpdateNormalStateEstimates();
    /** @brief 将解算后的全局反馈复制到 context_.state。 */
    void CaptureStateSnapshot();
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
    /** @brief 执行带 1000 周期启动等待的重力补偿测试。 */
    void ExecuteGravityTest();
    /** @brief 从旧 VMC_Chassis_Target 复制六维命令到 context_.command。 */
    void CaptureLegacyCommand();
    /**
     * @brief 将统一命令复制到现有 VMC_Chassis_Target 兼容结构。
     * @param[in] command 本周期最终映射前命令。
     */
    void CommitCommand(const ChassisCommand& command);
    /** @brief 提交统一命令、执行最终 VMC 映射，并更新错误蜂鸣器。 */
    void ApplyOutputs();

    TaskContext context_{};        /**< 仅由 Motor_task 对象拥有的任务持久状态。 */
    BalanceController balance_controller_; /**< 固定正常平衡算法顺序并返回命令与模式事件的 C++ 控制器。 */
    SitController sit_controller_; /**< 首个原生 C++ 动作控制器：坐地控制器。 */
    StairController stair_controller_; /**< 管理伸腿和收腿内部阶段的 C++ 上台阶控制器。 */
    StartupRetractController startup_retract_controller_; /**< 管理倒地自起接管与起立前收腿恢复的 C++ 控制器。 */
};

} // namespace chassis

#endif // CHASSIS_CONTROL_TASK_HPP
