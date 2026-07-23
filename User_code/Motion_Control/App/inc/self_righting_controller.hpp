/**
 * @file self_righting_controller.hpp
 * @brief 倒地姿态进入/退出去抖与三阶段自起动作的 C++ 控制器接口。
 */

#ifndef SELF_RIGHTING_CONTROLLER_HPP
#define SELF_RIGHTING_CONTROLLER_HPP

#include "chassis_control_types.hpp"

namespace chassis
{

/** @brief 倒地自起控制器一次更新产生的接管状态和六维命令。 */
struct SelfRightingUpdateResult
{
    ChassisCommand command{}; /**< 自起激活时的腿支持力、腿力矩和轮锁止力矩命令。 */
    bool active = false;      /**< true 表示本周期自起状态机接管 StartupRetract 流程。 */
};

/**
 * @brief 管理倒地检测去抖、姿态恢复去抖和三阶段倒地自起动作。
 *
 * 本灰度阶段继续使用 Self_Righting.h 中的现场可调参数和调试观测量，
 * 但动作结果通过 ChassisCommand 返回，不再写 VMC_Chassis_Target。
 */
class SelfRightingController
{
public:
    /**
     * @brief 更新倒地模式判定，并在激活时执行一次自起动作状态机。
     * @param[in] state 本周期俯仰/横滚角、腿部状态及原始轮电机速度快照。
     * @return 自起是否接管以及需要由任务统一提交的完整命令。
     */
    SelfRightingUpdateResult Update(const ChassisStateSnapshot& state);
};

} // namespace chassis

#endif // SELF_RIGHTING_CONTROLLER_HPP
