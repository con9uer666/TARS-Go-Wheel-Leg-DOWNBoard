/**
 * @file lqr_controller.cpp
 * @brief LqrController 的增益拟合、积分更新和四路力矩计算实现。
 *
 * LQR 状态（12维）：
 *   [ body_distance_error, speed_error, yaw_error, d_yaw,
 *     VMC_L.b_phi0, VMC_L.d_b_phi0, VMC_R.b_phi0, VMC_R.d_b_phi0,
 *     pitch_trans[0], d_pitch, int_pitch, int_s ]
 *   其中 int_pitch = ∫(pitch_trans[0] - pitch_offset_eff) dt，离地时冻结；
 *       int_s     = ∫(body_distance_error) dt，离地/小陀螺/上台阶时冻结。
 */

#include "lqr_controller.hpp"

extern "C"
{
#include "chassis_behavior_tree.h"
}

/** @brief 防劈叉叠加前的左腿 LQR 虚拟力矩兼容调试符号，单位 N·m。 */
float Leg_L_T;
/** @brief 防劈叉叠加前的右腿 LQR 虚拟力矩兼容调试符号，保留旧符号。 */
float Leg_R_T;

/** @brief 普通平衡模式的基础 pitch 目标偏置，单位 rad；当前仅作为现场调参符号保留。 */
float PITCH_OFFSET = -0.10;
/** @brief 小陀螺模式预留的附加 pitch 偏置，单位 rad；当前控制链未应用该量。 */
float PITCH_OFFSET_SPIN = 0.05f;
/** @brief pitch 偏置预留斜坡的每周期增量，单位 rad；当前未应用。 */
float pitch_offset_ramp = 0.0003f;
/** @brief LQR 当前实际使用的 pitch 目标偏置，单位 rad。 */
float pitch_offset_eff = -0.0f;

// 上电初值/备用值；第 5 个 Balance 周期由 LqrController 的二维拟合全量覆盖。
// 第 12 列(int_s 位移积分增益)暂置 0；跑完 12 维 LQR.m 后可用新模型定腿长输出整体替换。
/**
 * @brief 4×12 LQR 增益矩阵兼容调试对象。
 *
 * 上电先使用该备用值，第 5 个 Balance 周期起由腿长二次多项式完整覆盖。
 */
float LQR_K[4][12] = {
    -1.9045,  -2.9573,  -6.2325,  -1.2595,  -13.805,  -0.96974,  -3.4901,  -0.45392,  -10.489,  -1.5631,  -0.72426,  0,
     -1.9045,  -2.9573,  6.2325,  1.2595,  -3.4901,  -0.45392,  -13.805,  -0.96974,  -10.489,  -1.5631,  -0.72426,  0,
     4.5806,  6.7741,  -26.254,  -6.699,  29.403,  2.0429,  -5.5104,  0.023442,  -38.831,  -2.861,  -2.4091,  0,
     4.5806,  6.7741,  26.254,  6.699,  -5.5104,  0.023442,  29.403,  2.0429,  -38.831,  -2.861,  -2.4091,  0
};

// ⚠️ 必须整体替换：列数 11→12 后，LQR_Get_K 的行排布已改为 pos = i*12 + j，
//    下方为旧 11 维(44行)数据，在新排布下已全部错位，上台前务必用 12 维 LQR.m
//    输出的 48 行系数整体覆盖，否则在线拟合出的 K 全错。
/**
 * @brief LQR_K 的 48 组二元二次多项式拟合系数表。
 *
 * 每行依次为 p00、p10、p01、p20、p11、p02；行下标为 i*12+j。
 */
float K_Fit_Coefficients[48][6] = {
-4.4864,  -26.302,  14.454,  29.963,  -3.407,  -12.172,
     -4.7029,  -24.035,  14.691,  27.182,  -5.6658,  -11.888,
     -18.974,  57.524,  -21.007,  -74.869,  18.272,  25.913,
     -6.8255,  25.419,  -10.519,  -29.8,  8.5295,  12.531,
     -12.152,  -62.876,  16.328,  36.519,  0.19358,  -20.115,
     -0.35443,  -8.2663,  2.2085,  -3.6216,  2.2063,  -2.9825,
     -3.2062,  11.408,  -29.081,  -16.781,  27.676,  23.212,
     -0.093467,  -0.7295,  -3.7893,  1.3988,  -1.0485,  2.0283,
     -18.457,  38.417,  18.895,  -27.548,  -17.402,  -14.531,
     -2.8829,  3.6791,  4.7709,  -0.76297,  -4.1504,  -3.9878,
     -1.0255,  1.925,  1.239,  -1.2518,  -1.0694,  -0.99323,
     -1.6477,  -10.55,  5.4373,  12.059,  -0.74524,  -4.7002,
     -4.4864,  14.454,  -26.302,  -12.172,  -3.407,  29.963,
     -4.7029,  14.691,  -24.035,  -11.888,  -5.6658,  27.182,
     18.974,  21.007,  -57.524,  -25.913,  -18.272,  74.869,
     6.8255,  10.519,  -25.419,  -12.531,  -8.5295,  29.8,
     -3.2062,  -29.081,  11.408,  23.212,  27.676,  -16.781,
     -0.093467,  -3.7893,  -0.7295,  2.0283,  -1.0485,  1.3988,
     -12.152,  16.328,  -62.876,  -20.115,  0.19358,  36.519,
     -0.35443,  2.2085,  -8.2663,  -2.9825,  2.2063,  -3.6216,
     -18.457,  18.895,  38.417,  -14.531,  -17.402,  -27.548,
     -2.8829,  4.7709,  3.6791,  -3.9878,  -4.1504,  -0.76297,
     -1.0255,  1.239,  1.925,  -0.99323,  -1.0694,  -1.2518,
     -1.6477,  5.4373,  -10.55,  -4.7002,  -0.74524,  12.059,
     12.645,  -7.3525,  -31.836,  -8.7246,  36.138,  14.459,
     12.198,  -6.1261,  -32.901,  -8.9592,  38.088,  14.035,
     -26.549,  -15.222,  -48.282,  43.407,  -28.708,  55.869,
     -8.9723,  -14.241,  -19.731,  25.735,  -16.104,  20.506,
     28.058,  -11.758,  8.108,  25.246,  58.248,  -23.553,
     1.2403,  5.8563,  -1.3104,  0.99888,  4.9848,  -0.88311,
     2.3531,  -28.641,  -48.989,  44.307,  -37.543,  12.821,
     0.52278,  -2.7021,  -0.40202,  3.0998,  -1.6884,  -9.3707,
     -29.083,  -115.31,  45.797,  132.09,  3.9433,  -46.48,
     -1.8506,  -14.331,  3.7252,  13.851,  3.9173,  -4.0475,
     -1.3641,  -6.0528,  1.9831,  6.7233,  0.63031,  -2.0865,
     4.918,  -3.1143,  -11.819,  -3.209,  13.228,  5.5791,
     12.645,  -31.836,  -7.3525,  14.459,  36.138,  -8.7246,
     12.198,  -32.901,  -6.1261,  14.035,  38.088,  -8.9592,
     26.549,  48.282,  15.222,  -55.869,  28.708,  -43.407,
     8.9723,  19.731,  14.241,  -20.506,  16.104,  -25.735,
     2.3531,  -48.989,  -28.641,  12.821,  -37.543,  44.307,
     0.52278,  -0.40202,  -2.7021,  -9.3707,  -1.6884,  3.0998,
     28.058,  8.108,  -11.758,  -23.553,  58.248,  25.246,
     1.2403,  -1.3104,  5.8563,  -0.88311,  4.9848,  0.99888,
     -29.083,  45.797,  -115.31,  -46.48,  3.9433,  132.09,
     -1.8506,  3.7252,  -14.331,  -4.0475,  3.9173,  13.851,
     -1.3641,  1.9831,  -6.0528,  -2.0865,  0.63031,  6.7233,
     4.918,  -11.819,  -3.1143,  5.5791,  13.228,  -3.209,
};

/**
 * @brief 二维多项式拟合 LQR 反馈增益矩阵 K(L0_l, L0_r)。
 *
 * 对于 LQR 的 4×12 增益矩阵 K，每个元素 K[i][j] 通过二次多项式插值获得：
 *   K[i][j] = p00 + p10·L0_l + p01·L0_r + p20·L0_l² + p11·L0_l·L0_r + p02·L0_r²
 * 其中系数由预拟合表 K_Fit_Coefficients[48][6] 提供。
 *
 * @param[in] left_leg_length_m 左腿当前腿长，单位 m。
 * @param[in] right_leg_length_m 右腿当前腿长，单位 m。
 */
void chassis::LqrController::FitGainMatrix(float left_leg_length_m,
                                           float right_leg_length_m)
{
    /** K 矩阵的输出通道行下标。 */
    for (std::uint8_t i = 0U; i < 4U; ++i)
    {
        /** 12 维 LQR 状态的列下标。 */
        for (std::uint8_t j = 0U; j < 12U; ++j)
        {
            /** 当前 K[i][j] 在 48 行拟合系数表中的行优先下标。 */
            const std::uint8_t pos = static_cast<std::uint8_t>(i * 12U + j);

            /** 与腿长无关的常数项系数。 */
            const float p00 = K_Fit_Coefficients[pos][0];
            /** 左腿长一次项系数。 */
            const float p10 = K_Fit_Coefficients[pos][1];
            /** 右腿长一次项系数。 */
            const float p01 = K_Fit_Coefficients[pos][2];
            /** 左腿长平方项系数。 */
            const float p20 = K_Fit_Coefficients[pos][3];
            /** 左右腿长乘积项系数。 */
            const float p11 = K_Fit_Coefficients[pos][4];
            /** 右腿长平方项系数。 */
            const float p02 = K_Fit_Coefficients[pos][5];

            LQR_K[i][j] = p00
                      + p10 * left_leg_length_m
                      + p01 * right_leg_length_m
                      + p20 * left_leg_length_m * left_leg_length_m
                      + p11 * left_leg_length_m * right_leg_length_m
                      + p02 * right_leg_length_m * right_leg_length_m;
        }
    }
}

/** @brief 本周期 LQR 实际使用的位移误差调试副本，单位 m。 */
float lqr_body_distance_error;
/** @brief 本周期 LQR 实际使用的速度误差调试副本，单位 m/s。 */
float lqr_speed_error;
/** @brief 本周期 LQR 实际使用的 yaw 误差调试副本。 */
float lqr_yaw_error;
/** @brief 本周期 LQR 实际使用的 yaw 角速度调试副本，单位 rad/s。 */
float lqr_d_yaw;

/** @brief 第 11 维状态：pitch 误差积分，单位 rad·s，离地时冻结。 */
float int_pitch = 0.0f;
/** @brief pitch 误差积分的正负抗饱和限幅，单位 rad·s。 */
float int_pitch_limit = 0.5f;
/** @brief 第 12 维状态：位移误差积分，单位 m·s。 */
float int_s = 0.0f;
/** @brief 位移误差积分的正负抗饱和限幅，单位 m·s。 */
float int_s_limit = 1.0f;

namespace chassis
{

/**
 * @brief 保存 LQR 固定参数并将 K 刷新计数初始化为零。
 * @param[in] config 增益刷新周期、积分周期和腿角前馈参数。
 */
LqrController::LqrController(const LqrControllerConfig& config)
    : config_(config)
{
}

/**
 * @brief 每累计到指定周期数时重新拟合全部 4×12 K 矩阵。
 * @param[in] left_leg_length_m 左腿当前腿长，单位 m。
 * @param[in] right_leg_length_m 右腿当前腿长，单位 m。
 */
void LqrController::UpdateGainMatrix(float left_leg_length_m,
                                     float right_leg_length_m)
{
    ++gain_update_cycle_count_;
    if (gain_update_cycle_count_ >= config_.gain_update_period_cycles)
    {
        gain_update_cycle_count_ = 0U;
        FitGainMatrix(left_leg_length_m, right_leg_length_m);
    }
}

/**
 * @brief 更新两个积分状态并按旧符号逐项计算四路 LQR 力矩。
 * @param[in] state 左右腿 b_phi0、b_phi0 角速度和车身 pitch 状态。
 * @param[in] input 速度/位移/yaw 误差、离地计数和模式快照。
 * @return 左右轮力矩和防劈叉叠加前的左右腿力矩。
 */
LqrOutput LqrController::Calculate(const ChassisStateSnapshot& state,
                                   const LqrControlInput& input)
{
    lqr_body_distance_error = input.body_distance_error_m;
    lqr_speed_error = input.speed_error_mps;
    lqr_yaw_error = input.yaw_error_rad;
    lqr_d_yaw = input.yaw_rate_radps;

    /** 由速度误差产生的左右腿共同 b_phi0 前馈偏置，单位 rad。 */
    float leg_body_angle_offset = 0.0f;
    if (input.speed_error_mps >= 0.0f)
        leg_body_angle_offset = input.speed_error_mps * config_.leg_angle_feedforward_gain;
    else
        leg_body_angle_offset = input.speed_error_mps * config_.leg_angle_feedforward_gain;

    if (input.spinning_active || input.stair_request_active)
    {
        lqr_body_distance_error = 0.0f;
        int_s = 0.0f;
        leg_body_angle_offset = 0.0f;
    }

    if (input.left_off_ground_count < config_.off_ground_freeze_threshold
        && input.right_off_ground_count < config_.off_ground_freeze_threshold)
    {
        int_pitch += (state.pitch_rad - pitch_offset_eff) * config_.sample_period_s;
        if (int_pitch > int_pitch_limit)
            int_pitch = int_pitch_limit;
        if (int_pitch < -int_pitch_limit)
            int_pitch = -int_pitch_limit;

        int_s += lqr_body_distance_error * config_.sample_period_s;
        if (int_s > int_s_limit)
            int_s = int_s_limit;
        if (int_s < -int_s_limit)
            int_s = -int_s_limit;
    }

    /** 本周期通过 4×12 K 矩阵计算得到的四路力矩。 */
    LqrOutput output{};
    output.left_wheel_torque_nm =
      + LQR_K[0][0] * lqr_body_distance_error
      + LQR_K[0][1] * lqr_speed_error
      + LQR_K[0][2] * lqr_yaw_error
      - LQR_K[0][3] * lqr_d_yaw
      - LQR_K[0][4] * (state.left_leg.body_angle_rad - leg_body_angle_offset)
      - LQR_K[0][5] * state.left_leg.body_angular_rate_radps
      - LQR_K[0][6] * (state.right_leg.body_angle_rad - leg_body_angle_offset)
      - LQR_K[0][7] * state.right_leg.body_angular_rate_radps
      + LQR_K[0][8] * (state.pitch_rad - pitch_offset_eff)
      + LQR_K[0][9] * state.pitch_rate_radps
      + LQR_K[0][10] * int_pitch
      + LQR_K[0][11] * int_s;

    output.right_wheel_torque_nm =
      + LQR_K[1][0] * lqr_body_distance_error
      + LQR_K[1][1] * lqr_speed_error
      + LQR_K[1][2] * lqr_yaw_error
      - LQR_K[1][3] * lqr_d_yaw
      - LQR_K[1][4] * (state.left_leg.body_angle_rad - leg_body_angle_offset)
      - LQR_K[1][5] * state.left_leg.body_angular_rate_radps
      - LQR_K[1][6] * (state.right_leg.body_angle_rad - leg_body_angle_offset)
      - LQR_K[1][7] * state.right_leg.body_angular_rate_radps
      + LQR_K[1][8] * (state.pitch_rad - pitch_offset_eff)
      + LQR_K[1][9] * state.pitch_rate_radps
      + LQR_K[1][10] * int_pitch
      + LQR_K[1][11] * int_s;

    output.left_base_leg_torque_nm =
      + LQR_K[2][0] * lqr_body_distance_error
      + LQR_K[2][1] * lqr_speed_error
      + LQR_K[2][2] * (-lqr_yaw_error)
      - LQR_K[2][3] * lqr_d_yaw
      - LQR_K[2][4] * (state.left_leg.body_angle_rad - leg_body_angle_offset)
      - LQR_K[2][5] * state.left_leg.body_angular_rate_radps
      - LQR_K[2][6] * (state.right_leg.body_angle_rad - leg_body_angle_offset)
      - LQR_K[2][7] * state.right_leg.body_angular_rate_radps
      + LQR_K[2][8] * (state.pitch_rad - pitch_offset_eff)
      + LQR_K[2][9] * state.pitch_rate_radps
      + LQR_K[2][10] * int_pitch
      + LQR_K[2][11] * int_s;

    output.right_base_leg_torque_nm =
      + LQR_K[3][0] * lqr_body_distance_error
      + LQR_K[3][1] * lqr_speed_error
      + LQR_K[3][2] * (-lqr_yaw_error)
      - LQR_K[3][3] * lqr_d_yaw
      - LQR_K[3][4] * (state.left_leg.body_angle_rad - leg_body_angle_offset)
      - LQR_K[3][5] * state.left_leg.body_angular_rate_radps
      - LQR_K[3][6] * (state.right_leg.body_angle_rad - leg_body_angle_offset)
      - LQR_K[3][7] * state.right_leg.body_angular_rate_radps
      + LQR_K[3][8] * (state.pitch_rad - pitch_offset_eff)
      + LQR_K[3][9] * state.pitch_rate_radps
      + LQR_K[3][10] * int_pitch
      + LQR_K[3][11] * int_s;

    Leg_L_T = output.left_base_leg_torque_nm;
    Leg_R_T = output.right_base_leg_torque_nm;
    return output;
}

} // namespace chassis
