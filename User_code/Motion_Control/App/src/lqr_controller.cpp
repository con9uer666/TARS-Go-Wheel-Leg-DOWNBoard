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
    -4.1492,  -4.2177,  -14.833,  -4.842,  -10.563,  -0.94773,  -6.3097,  -0.70169,  -7.2558,  -1.3999,  -0.46973,  -1.5576,
     -4.1492,  -4.2177,  14.833,  4.842,  -6.3097,  -0.70169,  -10.563,  -0.94773,  -7.2558,  -1.3999,  -0.46973,  -1.5576,
     5.9096,  5.6994,  -10.949,  -4.0284,  24.109,  1.913,  -9.1119,  -0.37691,  -22.446,  -1.9834,  -1.2718,  2.3012,
     5.9096,  5.6994,  10.949,  4.0284,  -9.1119,  -0.37691,  24.109,  1.913,  -22.446,  -1.9834,  -1.2718,  2.3012,
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
-3.149,  -9.6496,  1.2386,  10.568,  0.67729,  -1.1843,
     -3.3895,  -8.5943,  2.0442,  9.3244,  -0.87746,  -1.8751,
     -14.574,  6.5967,  -7.7524,  -10.215,  0.5408,  10.973,
     -4.7901,  3.1672,  -3.3014,  -4.1791,  0.25057,  4.3067,
     -7.5469,  -37.676,  14.248,  20.061,  13.568,  -21.159,
     -0.29069,  -5.5956,  1.4574,  -1.8476,  2.3777,  -2.3643,
     -3.2971,  10.616,  -33.781,  -16.647,  16.132,  23.42,
     -0.15647,  0.5407,  -4.0938,  -0.90612,  1.255,  -0.43179,
     -11.515,  19.812,  14.396,  -13.878,  -11.511,  -8.8187,
     -2.1184,  2.8022,  3.1159,  -1.1215,  -2.9848,  -1.939,
     -0.73993,  1.207,  0.97899,  -0.80125,  -0.78761,  -0.62346,
     -1.1353,  -3.9236,  0.27572,  4.3077,  0.64093,  -0.29021,
     -3.149,  1.2386,  -9.6496,  -1.1843,  0.67729,  10.568,
     -3.3895,  2.0442,  -8.5943,  -1.8751,  -0.87746,  9.3244,
     14.574,  7.7524,  -6.5967,  -10.973,  -0.5408,  10.215,
     4.7901,  3.3014,  -3.1672,  -4.3067,  -0.25057,  4.1791,
     -3.2971,  -33.781,  10.616,  23.42,  16.132,  -16.647,
     -0.15647,  -4.0938,  0.5407,  -0.43179,  1.255,  -0.90612,
     -7.5469,  14.248,  -37.676,  -21.159,  13.568,  20.061,
     -0.29069,  1.4574,  -5.5956,  -2.3643,  2.3777,  -1.8476,
     -11.515,  14.396,  19.812,  -8.8187,  -11.511,  -13.878,
     -2.1184,  3.1159,  2.8022,  -1.939,  -2.9848,  -1.1215,
     -0.73993,  0.97899,  1.207,  -0.62346,  -0.78761,  -0.80125,
     -1.1353,  0.27572,  -3.9236,  -0.29021,  0.64093,  4.3077,
     9.1539,  9.5828,  -36.831,  -22.309,  32.473,  18.177,
     8.9335,  9.6793,  -37.348,  -21.2,  34.223,  16.983,
     -12.229,  14.436,  -8.8823,  -10.073,  -6.7568,  10.723,
     -4.0818,  2.9351,  -3.7416,  -1.3611,  -3.6852,  3.6703,
     21.215,  7.3672,  4.4747,  10.961,  57.154,  -19.57,
     1.0605,  5.9205,  -1.7292,  4.0714,  4.8527,  -0.41422,
     0.7419,  -14.008,  -52.656,  25.723,  -30.14,  6.8524,
     0.35872,  -1.2571,  -1.9693,  1.2078,  0.44145,  -12.214,
     -17.159,  -75.071,  30.32,  82.669,  0.81049,  -26.788,
     -1.0456,  -9.1067,  0.88283,  7.5102,  3.5142,  -0.54065,
     -0.91797,  -4.4057,  1.3773,  4.6696,  0.41751,  -1.2284,
     3.5375,  3.6127,  -13.874,  -8.7157,  11.9,  7.1901,
     9.1539,  -36.831,  9.5828,  18.177,  32.473,  -22.309,
     8.9335,  -37.348,  9.6793,  16.983,  34.223,  -21.2,
     12.229,  8.8823,  -14.436,  -10.723,  6.7568,  10.073,
     4.0818,  3.7416,  -2.9351,  -3.6703,  3.6852,  1.3611,
     0.7419,  -52.656,  -14.008,  6.8524,  -30.14,  25.723,
     0.35872,  -1.9693,  -1.2571,  -12.214,  0.44145,  1.2078,
     21.215,  4.4747,  7.3672,  -19.57,  57.154,  10.961,
     1.0605,  -1.7292,  5.9205,  -0.41422,  4.8527,  4.0714,
     -17.159,  30.32,  -75.071,  -26.788,  0.81049,  82.669,
     -1.0456,  0.88283,  -9.1067,  -0.54065,  3.5142,  7.5102,
     -0.91797,  1.3773,  -4.4057,  -1.2284,  0.41751,  4.6696,
     3.5375,  -13.874,  3.6127,  7.1901,  11.9,  -8.7157,
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
