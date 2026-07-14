/**
 * @file chassis_control_types.hpp
 * @brief C++ 底盘控制器之间共享的状态、模式和命令数据类型。
 *
 * 本文件只定义纯数据类型，不访问硬件、不包含控制算法，也不拥有全局状态。
 * 将公共类型独立出来，可以让各动作控制器脱离 Motor_task 单独复用和测试。
 */

#ifndef CHASSIS_CONTROL_TYPES_HPP
#define CHASSIS_CONTROL_TYPES_HPP

#include <cstdint>

namespace chassis
{

/**
 * @brief Motor_task 每周期选择的唯一顶层运行模式。
 *
 * 当前枚举是兼容旧 start_mode/upstares_mode 的灰度过渡层。后续会继续把
 * StairExtend/StairRetract 合并为一个带内部阶段的 StairController。
 */
enum class RunMode : std::uint8_t
{
    GravityTest,   /**< 重力补偿标定模式，跳过正常状态解算和动作调度。 */
    StartupRetract,/**< 未站起阶段：包含倒地自起、收腿和转腿到竖直。 */
    Balance,       /**< 正常平衡运行：包含普通移动、小陀螺和跳跃修饰动作。 */
    StairExtend,   /**< 上台阶前半段：伸腿并外摆腿角。 */
    Sit,           /**< 坐地动作：腿长和腿角沿斜坡移动到坐地目标。 */
    StairRetract,  /**< 上台阶后半段：收腿并恢复到正常平衡姿态。 */
    Hold           /**< 无有效旧模式组合时，不重新计算动作，保持已有目标。 */
};

/**
 * @brief 标识本周期命令由旧 C 动作组还是新 C++ 控制器产生。
 *
 * 灰度迁移期间两套控制器并存。任务根据该标识决定是否需要从
 * VMC_Chassis_Target 读取旧动作结果。
 */
enum class CommandSource : std::uint8_t
{
    LegacyGlobal, /**< 命令由旧 C 动作组写入 VMC_Chassis_Target。 */
    NativeCpp     /**< 命令由 C++ 控制器直接返回 ChassisCommand。 */
};

/**
 * @brief 单侧轮端运动状态。
 *
 * 数据来自 Wheel_End_Velocity_Both()，目前只作为任务状态快照保存。
 */
struct WheelSideState
{
    float velocity_mps = 0.0f;      /**< 轮缘线速度，单位 m/s。 */
    float acceleration_mps2 = 0.0f; /**< 轮缘线加速度，单位 m/s²。 */
};

/** @brief 左右轮端运动状态。 */
struct WheelMotionState
{
    WheelSideState left;  /**< 左轮轮端状态。 */
    WheelSideState right; /**< 右轮轮端状态。 */
};

/**
 * @brief 单腿 VMC 解算后的状态快照。
 *
 * 这里只保存动作控制器常用的运动状态，不复制机构常量和矩阵中间量。
 */
struct LegStateSnapshot
{
    float length_m = 0.0f;                /**< 虚拟腿长度 L0，单位 m。 */
    float length_rate_mps = 0.0f;         /**< 虚拟腿伸缩速度 d_L0，单位 m/s。 */
    float leg_angle_rad = 0.0f;           /**< 虚拟腿绝对角 phi0，单位 rad。 */
    float leg_angular_rate_radps = 0.0f;  /**< 虚拟腿角速度 d_phi0，单位 rad/s。 */
    float body_angle_rad = 0.0f;          /**< 相对车身的虚拟腿角 b_phi0，单位 rad。 */
    float body_angular_rate_radps = 0.0f; /**< b_phi0 的角速度，单位 rad/s。 */
};

/**
 * @brief 一个控制周期内保持不变的只读底盘状态。
 *
 * 新 C++ 控制器必须通过 const 引用读取此结构，不能直接访问反馈全局变量。
 */
struct ChassisStateSnapshot
{
    WheelMotionState wheel;       /**< 左右轮端速度和加速度。 */
    LegStateSnapshot left_leg;    /**< 左腿 VMC 状态。 */
    LegStateSnapshot right_leg;   /**< 右腿 VMC 状态。 */
    float body_speed_mps = 0.0f;  /**< 卡尔曼估计的车身水平速度，单位 m/s。 */
    float pitch_rad = 0.0f;       /**< 当前车身俯仰角，单位 rad。 */
    float pitch_rate_radps = 0.0f;/**< 当前车身俯仰角速度，单位 rad/s。 */
    float yaw_rate_radps = 0.0f;  /**< 当前车身偏航角速度，单位 rad/s。 */
};

/**
 * @brief 原 start_mode/upstares_mode 的单周期兼容快照。
 */
struct LegacyModeSignals
{
    std::uint8_t start_mode = 0;         /**< 原 start_mode 数值，含义由旧状态逻辑定义。 */
    std::uint8_t stair_retract_mode = 0; /**< 原 upstares_mode，1 表示执行上台阶后收腿。 */
};

/**
 * @brief VMC 映射前的完整底盘执行命令。
 *
 * 所有新控制器最终都应返回此结构。任务在周期末统一提交一次，再调用
 * VMC_Apply_Chassis_Target() 完成腿部雅可比映射和轮电机目标赋值。
 */
struct ChassisCommand
{
    float left_support_force_n = 0.0f;  /**< 左腿沿虚拟腿方向的目标支持力 F0，单位 N。 */
    float left_leg_torque_nm = 0.0f;    /**< 左腿目标虚拟力矩 T，单位 N·m。 */
    float right_support_force_n = 0.0f; /**< 右腿沿虚拟腿方向的目标支持力 F0，单位 N。 */
    float right_leg_torque_nm = 0.0f;   /**< 右腿目标虚拟力矩 T，单位 N·m。 */
    float left_wheel_torque_nm = 0.0f;  /**< 左轮目标力矩，单位 N·m。 */
    float right_wheel_torque_nm = 0.0f; /**< 右轮目标力矩，单位 N·m。 */
};

} // namespace chassis

#endif // CHASSIS_CONTROL_TYPES_HPP
