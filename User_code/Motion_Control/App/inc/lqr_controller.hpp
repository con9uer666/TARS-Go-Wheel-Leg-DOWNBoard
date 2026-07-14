/**
 * @file lqr_controller.hpp
 * @brief 12 维轮腿 LQR 增益刷新和力矩计算的 C++ 控制器接口。
 */

#ifndef LQR_CONTROLLER_HPP
#define LQR_CONTROLLER_HPP

#include <cstdint>
#include "chassis_control_types.hpp"

namespace chassis
{

/** @brief LQR 增益刷新、积分和腿摆角前馈使用的固定参数。 */
struct LqrControllerConfig
{
    std::uint8_t gain_update_period_cycles = 5U; /**< 每 5 个 500 Hz 控制周期刷新一次 K，即 100 Hz。 */
    float sample_period_s = 0.002f;              /**< 俯仰与位移积分的离散周期，单位 s。 */
    float leg_angle_feedforward_gain = 0.02f;    /**< 速度误差到虚拟腿 b_phi0 前馈偏置的比例。 */
    int off_ground_freeze_threshold = 10;        /**< 任一腿离地计数达到该值时冻结两个 LQR 积分状态。 */
};

/** @brief LqrController 单周期使用的误差、模式和离地状态。 */
struct LqrControlInput
{
    float body_distance_error_m = 0.0f; /**< 目标位移减实际位移，单位 m。 */
    float speed_error_mps = 0.0f;       /**< 目标车身速度减估计速度，单位 m/s。 */
    float yaw_error_rad = 0.0f;         /**< 供 LQR 使用的滤波后 yaw 误差或旋转 PID 输出。 */
    float yaw_rate_radps = 0.0f;        /**< 车体 yaw 角速度 d_yaw，单位 rad/s。 */
    int left_off_ground_count = 0;      /**< 左腿离地滤波计数。 */
    int right_off_ground_count = 0;     /**< 右腿离地滤波计数。 */
    bool spinning_active = false;       /**< true 表示小陀螺运行或退出过程仍在使用旋转逻辑。 */
    bool stair_request_active = false;  /**< true 表示外部 upstairs_flag 正请求切入上台阶。 */
};

/** @brief 12 维 LQR 单周期产生的左右轮力矩和基础虚拟腿力矩。 */
struct LqrOutput
{
    float left_wheel_torque_nm = 0.0f;     /**< 左轮 LQR 目标力矩，单位 N·m。 */
    float right_wheel_torque_nm = 0.0f;    /**< 右轮 LQR 目标力矩，单位 N·m。 */
    float left_base_leg_torque_nm = 0.0f;  /**< 防劈叉叠加前的左腿 LQR 虚拟力矩。 */
    float right_base_leg_torque_nm = 0.0f; /**< 防劈叉叠加前的右腿 LQR 虚拟力矩，保留旧 Leg_R_T 符号。 */
};

/**
 * @brief 根据左右腿长在线拟合 K，并将 12 维状态映射为四个力矩输出的控制器。
 *
 * K 矩阵、积分量和调试误差仍保留原全局符号，作为现场调参与旧 C 离地保护的兼容边界。
 * 控制器不再写 VMC_Chassis_Target，所有力矩都通过 LqrOutput 返回。
 */
class LqrController
{
public:
    /**
     * @brief 构造 LQR 控制器并保存固定配置。
     * @param[in] config 增益刷新周期、积分周期和腿摆角前馈参数。
     */
    explicit LqrController(const LqrControllerConfig& config = LqrControllerConfig{});

    /**
     * @brief 按 5 周期节流计数刷新 K(L0_l,L0_r)。
     * @param[in] left_leg_length_m 左腿当前虚拟腿长，单位 m。
     * @param[in] right_leg_length_m 右腿当前虚拟腿长，单位 m。
     */
    void UpdateGainMatrix(float left_leg_length_m, float right_leg_length_m);

    /**
     * @brief 更新积分状态并计算左右轮/腿力矩。
     * @param[in] state 包含左右腿 b_phi0、角速度与车身 pitch 的状态快照。
     * @param[in] input 误差、离地计数和模式快照。
     * @return 不含防劈叉与离地覆盖的四个 LQR 力矩。
     */
    LqrOutput Calculate(const ChassisStateSnapshot& state,
                        const LqrControlInput& input);

private:
    /**
     * @brief 用二次二元多项式填充 4×12 增益矩阵。
     * @param[in] left_leg_length_m 左腿虚拟腿长，单位 m。
     * @param[in] right_leg_length_m 右腿虚拟腿长，单位 m。
     */
    void FitGainMatrix(float left_leg_length_m, float right_leg_length_m);

    LqrControllerConfig config_;             /**< 增益刷新、积分和腿角前馈参数副本。 */
    std::uint8_t gain_update_cycle_count_ = 0U; /**< 距离上次 K 拟合已经过的控制周期数。 */
};

} // namespace chassis

#endif // LQR_CONTROLLER_HPP
