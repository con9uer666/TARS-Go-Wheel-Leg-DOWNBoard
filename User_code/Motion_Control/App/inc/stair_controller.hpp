/**
 * @file stair_controller.hpp
 * @brief 上台阶伸腿和收腿恢复的一体化 C++ 控制器。
 */

#ifndef STAIR_CONTROLLER_HPP
#define STAIR_CONTROLLER_HPP

#include <cstdint>
#include "chassis_control_types.hpp"

extern "C"
{
#include "user_pid.h"
#include "VMC.h"
}

namespace chassis
{

/**
 * @brief 上台阶控制器内部阶段。
 *
 * 顶层任务只看到一个 RunMode::Stair；伸腿和收腿属于同一行为内部的真实阶段。
 */
enum class StairPhase : std::uint8_t
{
    Inactive, /**< 当前没有执行上台阶动作。 */
    Extend,   /**< 伸长双腿并向外摆腿，将车体顶上台阶。 */
    Retract   /**< 收短双腿并转回竖直姿态，恢复正常平衡。 */
};

/**
 * @brief StairController 使用的 PID、VMC 和腿长参数依赖。
 */
struct StairControllerDependencies
{
    user_pid_t& left_length_position_pid;  /**< 左腿腿长位置环 PID。 */
    user_pid_t& left_length_velocity_pid;  /**< 左腿腿长速度环 PID。 */
    user_pid_t& right_length_position_pid; /**< 右腿腿长位置环 PID。 */
    user_pid_t& right_length_velocity_pid; /**< 右腿腿长速度环 PID。 */
    user_pid_t& left_angle_position_pid;   /**< 左腿角度位置环 PID。 */
    user_pid_t& left_angle_velocity_pid;   /**< 左腿角度速度环 PID。 */
    user_pid_t& right_angle_position_pid;  /**< 右腿角度位置环 PID。 */
    user_pid_t& right_angle_velocity_pid;  /**< 右腿角度速度环 PID。 */
    VMC_t& left_vmc;                       /**< 左腿完整 VMC 状态，供卡住检测辅助函数使用。 */
    VMC_t& right_vmc;                      /**< 右腿完整 VMC 状态，供卡住检测辅助函数使用。 */
    float& minimum_leg_length_m;           /**< 工程当前最短腿长参数，单位 m；作为恢复正常腿长的装配边界保留。 */
    float& maximum_leg_length_m;           /**< 工程当前最长腿长参数，单位 m；伸腿阶段两侧腿长目标。 */
};

/**
 * @brief 上台阶动作使用的目标、阈值和持续周期配置。
 */
struct StairControllerConfig
{
    float extend_left_angle_rad = 3.07079625f; /**< 伸腿左腿目标角，与旧式 3.1415926f/2+1.5f 的 float 结果逐位等价，单位 rad。 */
    float extend_right_angle_rad = 0.07079625f;/**< 伸腿右腿目标角，与旧式 3.1415926f/2-1.5f 的 float 结果逐位等价，单位 rad。 */
    float extend_error_tolerance = 0.15f;      /**< 伸腿阶段腿长和角度 PID 误差阈值。 */
    std::uint16_t extend_ready_cycles = 60;    /**< 单腿满足长度与角度阈值的累计周期数；旧逻辑脱离阈值时不清零。 */

    float retract_target_length_m = 0.12f;     /**< 收腿阶段腿长目标，单位 m。 */
    float retract_left_angle_rad = 1.4707962f; /**< 收腿左腿目标角，与旧式 3.1415926f/2-0.1f 的 float 结果逐位等价，单位 rad。 */
    float retract_right_angle_rad = 1.6707963f;/**< 收腿右腿目标角，与旧式 3.1415926f/2+0.1f 的 float 结果逐位等价，单位 rad。 */
    float retract_length_tolerance_m = 0.05f;  /**< 收腿阶段腿长位置环误差阈值，单位 m。 */
    std::uint16_t retract_ready_cycles = 50;   /**< 收腿和转角阶段各自连续到位周期数。 */
    std::uint16_t completion_dwell_cycles = 150; /**< 双腿完成后切回 Balance 前等待周期数。 */
};

/**
 * @brief StairController 单周期计算结果。
 */
struct StairUpdateResult
{
    ChassisCommand command;              /**< 本周期上台阶 VMC 映射前命令。 */
    bool entered_retract_phase = false;  /**< 本周期末是否刚从伸腿切换到收腿阶段。 */
    bool completed = false;              /**< 本周期末是否完成整个上台阶动作。 */
};

/**
 * @brief 把伸腿顶台阶和收腿恢复统一为一个带内部阶段的控制器。
 */
class StairController
{
public:
    /**
     * @brief 构造上台阶控制器并绑定外部 PID、VMC 和腿长参数。
     * @param[in] dependencies 控制器使用的外部依赖引用。
     * @param[in] config 上台阶目标和到位判定参数。
     */
    StairController(const StairControllerDependencies& dependencies,
                    const StairControllerConfig& config = StairControllerConfig{});

    /**
     * @brief 根据旧模式标志在首次接管时恢复正确的内部阶段。
     * @param[in] signals 本周期锁存的旧 start_mode/upstares_mode。
     */
    void SynchronizeLegacyPhase(const LegacyModeSignals& signals);

    /**
     * @brief 执行一次上台阶控制更新。
     * @param[in] state 本周期只读底盘状态快照。
     * @param[in] previous_command 上周期命令；伸腿阶段需保留其中的轮力矩。
     * @return 本周期命令以及阶段切换/完成事件。
     */
    StairUpdateResult Update(const ChassisStateSnapshot& state,
                             const ChassisCommand& previous_command);

    /** @brief 获取当前内部阶段，供调试器悬停和状态显示使用。 */
    /** @return Inactive、Extend 或 Retract。 */
    StairPhase phase() const;

private:
    /**
     * @brief 执行伸腿顶台阶阶段。
     * @param[in] state 本周期只读状态快照。
     * @param[in] previous_command 用于保留上一周期轮力矩的命令。
     * @return 伸腿阶段命令和是否进入收腿阶段。
     */
    StairUpdateResult UpdateExtend(const ChassisStateSnapshot& state,
                                   const ChassisCommand& previous_command);

    /**
     * @brief 执行收腿并恢复竖直姿态阶段。
     * @param[in] state 本周期只读状态快照。
     * @return 收腿阶段命令和整个动作是否完成。
     */
    StairUpdateResult UpdateRetract(const ChassisStateSnapshot& state);

    /** @brief 初始化旧卡住反向辅助状态，准备进入收腿转角阶段。 */
    void PrepareRetractTurnState();

    /** @brief 清除内部完成等待和到位计数，使控制器返回空闲阶段。 */
    void ResetAfterCompletion();

    StairControllerDependencies dependencies_; /**< 构造时绑定的 PID、VMC 和腿长参数。 */
    StairControllerConfig config_;              /**< 上台阶目标与判定阈值副本。 */
    StairPhase phase_ = StairPhase::Inactive;   /**< 当前上台阶内部阶段。 */
    std::uint16_t completion_dwell_count_ = 0;  /**< 双腿完成后等待切回 Balance 的周期计数。 */
};

} // namespace chassis

#endif // STAIR_CONTROLLER_HPP
