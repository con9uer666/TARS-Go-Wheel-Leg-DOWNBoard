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

/**
 * @brief 轮力矩整体现场缩放系数（仅作用于左右轮，不影响腿力矩）。
 *
 * 用于快速降低平衡回路带宽、找回相位裕度：发散时从 1.0 往下调
 * （0.6 → 0.5 → 0.4 …）直到前后摆动收敛；再往上加到临界发散前一档。
 * 默认 1.0 = 不改变原始 LQR 输出。
 */
float lqr_wheel_torque_scale = 1.0f;

// 上电初值/备用值；第 5 个 Balance 周期由 LqrController 的二维拟合全量覆盖。
// 第 12 列(int_s 位移积分增益)暂置 0；跑完 12 维 LQR.m 后可用新模型定腿长输出整体替换。
/**
 * @brief 4×12 LQR 增益矩阵兼容调试对象。
 *
 * 上电先使用该备用值，第 5 个 Balance 周期起由腿长二次多项式完整覆盖。
 */
float LQR_K[4][12] = {
    -6.8787,  -7.0245,  -21.868,  -7.0216,  -13.194,  -1.2581,  -9.5692,  -1.0453,  -9.6605,  -1.7948,  -0.37838,  -2.577,
     -6.8787,  -7.0245,  21.868,  7.0216,  -9.5692,  -1.0453,  -13.194,  -1.2581,  -9.6605,  -1.7948,  -0.37838,  -2.577,
     3.373,  3.2815,  -6.5987,  -2.3753,  22.295,  1.6315,  -11.905,  -0.71354,  -29.973,  -3.2543,  -1.4878,  1.3108,
     3.373,  3.2815,  6.5987,  2.3753,  -11.905,  -0.71354,  22.295,  1.6315,  -29.973,  -3.2543,  -1.4878,  1.3108,
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
     -6.5276,  -5.3504,  2.976,  11.385,  -11.33,  1.9648,
     -7.0223,  -3.5811,  4.8265,  9.9214,  -14.646,  0.57965,
     -21.431,  -0.38287,  -2.4831,  -1.8833,  0.26319,  5.0706,
     -6.8891,  0.24148,  -1.0875,  -0.96468,  0.081978,  1.9409,
     -10.759,  -35.001,  16.357,  20.814,  10.462,  -25.665,
     -0.49734,  -6.7125,  1.9714,  -0.62946,  0.88826,  -2.5786,
     -6.1197,  11.171,  -36.386,  -18.739,  9.0448,  29.038,
     -0.35142,  1.1401,  -5.4574,  -1.1569,  -0.46671,  0.26952,
     -15.891,  27.917,  23.082,  -26.38,  -9.1873,  -20.233,
     -2.8211,  4.3406,  4.259,  -2.8935,  -3.5802,  -2.9627,
     -0.63263,  1.1841,  0.85951,  -1.2496,  -0.1131,  -0.82724,
     -2.3601,  -2.4534,  0.69049,  4.663,  -3.503,  1.0666,
     -6.5276,  2.976,  -5.3504,  1.9648,  -11.33,  11.385,
     -7.0223,  4.8265,  -3.5811,  0.57965,  -14.646,  9.9214,
     21.431,  2.4831,  0.38287,  -5.0706,  -0.26319,  1.8833,
     6.8891,  1.0875,  -0.24148,  -1.9409,  -0.081978,  0.96468,
     -6.1197,  -36.386,  11.171,  29.038,  9.0448,  -18.739,
     -0.35142,  -5.4574,  1.1401,  0.26952,  -0.46671,  -1.1569,
     -10.759,  16.357,  -35.001,  -25.665,  10.462,  20.814,
     -0.49734,  1.9714,  -6.7125,  -2.5786,  0.88826,  -0.62946,
     -15.891,  23.082,  27.917,  -20.233,  -9.1873,  -26.38,
     -2.8211,  4.259,  4.3406,  -2.9627,  -3.5802,  -2.8935,
     -0.63263,  0.85951,  1.1841,  -0.82724,  -0.1131,  -1.2496,
     -2.3601,  0.69049,  -2.4534,  1.0666,  -3.503,  4.663,
     5.439,  17.625,  -35.198,  -23.973,  24.6,  18.809,
     5.374,  18.12,  -36.272,  -23.346,  25.952,  18.338,
     -9.4067,  17.05,  1.037,  -17.671,  -1.9279,  -0.8039,
     -3.1587,  4.9859,  -0.12206,  -5.2287,  -1.0563,  -0.031524,
     18.499,  11.331,  7.4269,  23.204,  45.692,  -22.622,
     0.83827,  4.8332,  -1.1815,  7.715,  2.6865,  -0.60816,
     -2.809,  -3.3183,  -58.062,  13.802,  -23.061,  8.6805,
     0.11976,  -0.2143,  -3.7874,  -0.033539,  2.602,  -12.197,
     -26.55,  -65.433,  36.49,  75.232,  -3.4372,  -34.205,
     -2.6493,  -8.0103,  2.751,  7.4601,  1.4652,  -2.1697,
     -1.361,  -3.2749,  2.2336,  3.95,  -0.51255,  -2.0969,
     2.0942,  6.5622,  -13.142,  -9.2328,  9.0324,  7.273,
     5.439,  -35.198,  17.625,  18.809,  24.6,  -23.973,
     5.374,  -36.272,  18.12,  18.338,  25.952,  -23.346,
     9.4067,  -1.037,  -17.05,  0.8039,  1.9279,  17.671,
     3.1587,  0.12206,  -4.9859,  0.031524,  1.0563,  5.2287,
     -2.809,  -58.062,  -3.3183,  8.6805,  -23.061,  13.802,
     0.11976,  -3.7874,  -0.2143,  -12.197,  2.602,  -0.033539,
     18.499,  7.4269,  11.331,  -22.622,  45.692,  23.204,
     0.83827,  -1.1815,  4.8332,  -0.60816,  2.6865,  7.715,
     -26.55,  36.49,  -65.433,  -34.205,  -3.4372,  75.232,
     -2.6493,  2.751,  -8.0103,  -2.1697,  1.4652,  7.4601,
     -1.361,  2.2336,  -3.2749,  -2.0969,  -0.51255,  3.95,
     2.0942,  -13.142,  6.5622,  7.273,  9.0324,  -9.2328,
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
float int_pitch_limit = 0.0f;
/** @brief 第 12 维状态：位移误差积分，单位 m·s。 */
float int_s = 0.0f;
/** @brief 位移误差积分的正负抗饱和限幅，单位 m·s。 */
float int_s_limit = 0.0f;

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

    // 现场可调整体缩放：仅作用于轮力矩，用于压住前后点头/发散极限环。
    output.left_wheel_torque_nm *= lqr_wheel_torque_scale;
    output.right_wheel_torque_nm *= lqr_wheel_torque_scale;
    return output;
}

} // namespace chassis
