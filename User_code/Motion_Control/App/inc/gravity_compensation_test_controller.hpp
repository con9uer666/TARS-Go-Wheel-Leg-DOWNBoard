/**
 * @file gravity_compensation_test_controller.hpp
 * @brief 重力补偿标定流程的 C++ 控制器和调试观测量声明。
 */

#ifndef GRAVITY_COMPENSATION_TEST_CONTROLLER_HPP
#define GRAVITY_COMPENSATION_TEST_CONTROLLER_HPP

#include <cstdint>

#include "chassis_control_types.hpp"

namespace chassis
{

/** @brief 重力补偿标定使用的腿角采样数量。 */
constexpr std::uint16_t kGravityTestAngleCount = 18U;

/** @brief 每个腿角下使用的腿长采样数量。 */
constexpr std::uint16_t kGravityTestLengthCount = 10U;

/**
 * @brief 执行机械量程探测、姿态扫描和完成后结果遍历的标定控制器。
 *
 * 本灰度阶段保留原有调试器全局观测符号和专用标定 PID；控制器负责保存
 * “是否已初始化”状态，并通过 Update() 返回结构化命令，不再写 VMC 目标全局量。
 */
class GravityCompensationTestController
{
public:
    /**
     * @brief 推进一次 2 ms 标定状态机并生成本周期执行命令。
     *
     * 调用前必须已完成一次 VMC_Coculate()，使腿长、腿角及其速度为本周期值。
     * @return 机械量程探测或姿态锁定阶段使用的六维 VMC 映射前命令。
     */
    ChassisCommand Update();

private:
    bool initialized_ = false; /**< true 表示方向表、结果区和标定 PID 已完成首次初始化。 */
};

} // namespace chassis

extern "C"
{

/** @brief 左腿标定支持力结果表，索引顺序为 [腿角][腿长]，单位 N。 */
extern float grav_comp_F_left[chassis::kGravityTestAngleCount]
                             [chassis::kGravityTestLengthCount];
/** @brief 左腿标定虚拟腿力矩结果表，索引顺序为 [腿角][腿长]，单位 N·m。 */
extern float grav_comp_T_left[chassis::kGravityTestAngleCount]
                             [chassis::kGravityTestLengthCount];
/** @brief 右腿标定支持力结果表，索引顺序为 [腿角][腿长]，单位 N。 */
extern float grav_comp_F_right[chassis::kGravityTestAngleCount]
                              [chassis::kGravityTestLengthCount];
/** @brief 右腿标定虚拟腿力矩结果表，索引顺序为 [腿角][腿长]，单位 N·m。 */
extern float grav_comp_T_right[chassis::kGravityTestAngleCount]
                              [chassis::kGravityTestLengthCount];
/** @brief 每个采样位姿是否稳定收敛；1 为收敛记录，0 为超时强制记录。 */
extern std::uint8_t grav_comp_settled[chassis::kGravityTestAngleCount]
                                     [chassis::kGravityTestLengthCount];

/** @brief 实测左腿最短长度，单位 m。 */
extern float grav_L0_min_left;
/** @brief 实测左腿最长长度，单位 m。 */
extern float grav_L0_max_left;
/** @brief 实测右腿最短长度，单位 m。 */
extern float grav_L0_min_right;
/** @brief 实测右腿最长长度，单位 m。 */
extern float grav_L0_max_right;

/** @brief 全部姿态采样完成标志；1 表示 720 个 F/T 数据均已记录。 */
extern std::uint8_t gravity_comp_test_done;
/** @brief 调试器置 1 可请求从机械量程探测阶段重新开始完整标定。 */
extern std::uint8_t gravity_comp_test_restart;
/** @brief 当前腿角采样索引，范围 0..17。 */
extern std::uint16_t grav_comp_angle_idx;
/** @brief 当前腿长采样索引，范围 0..9。 */
extern std::uint16_t grav_comp_len_idx;
/** @brief 当前位姿连续满足稳定条件的控制周期数。 */
extern std::uint16_t grav_comp_settle_counter;
/** @brief 当前采样位姿已经运行的控制周期数，用于超时兜底。 */
extern std::uint16_t grav_comp_pose_cycle;

/** @brief 机械量程探测时左腿是否已经确认卡在限位。 */
extern std::uint8_t grav_probe_stuck_L;
/** @brief 机械量程探测时右腿是否已经确认卡在限位。 */
extern std::uint8_t grav_probe_stuck_R;
/** @brief 左腿连续接近零伸缩速度的卡住判定计数。 */
extern std::uint16_t grav_probe_stuck_cnt_L;
/** @brief 右腿连续接近零伸缩速度的卡住判定计数。 */
extern std::uint16_t grav_probe_stuck_cnt_R;
/** @brief 当前机械量程探测阶段已经运行的控制周期数。 */
extern std::uint16_t grav_probe_cycle;

/** @brief 标定完成后遍历四张结果表的线性步号，范围 0..719。 */
extern std::uint32_t grav_traverse_step;
/** @brief 当前遍历步对应的单个结果值，供调试器逐项观察。 */
extern float grav_traverse_val;
/** @brief 完成长鸣或结果遍历时距离下一步的剩余控制周期数。 */
extern std::uint32_t grav_traverse_countdown;
/** @brief 完成后子阶段：0 长鸣、1 遍历结果、2 遍历完成。 */
extern std::uint8_t grav_traverse_phase;

} // extern "C"

#endif // GRAVITY_COMPENSATION_TEST_CONTROLLER_HPP
