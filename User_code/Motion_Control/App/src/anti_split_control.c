/**
 * @file anti_split_control.c
 * @brief 防劈叉增益计算及左右腿防劈叉力矩合成。
 */

#include "anti_split_control.h"
#include "chassis_behavior_tree.h"
#include "user_pid.h"
#include "VMC.h"

/**
 * @brief 根据平均腿长线性插值防劈叉 PID 的 Kp 和 Kd。
 *
 * @param[out] AntiSplit_Kp 输出 Kp 增益。
 * @param[out] AntiSplit_Kd 输出 Kd 增益。
 * @param[in]  L0_avg       当前左右腿长均值，单位 m。
 */
static void AntiSplit_Get_K(float *AntiSplit_Kp, float *AntiSplit_Kd, float L0_avg)
{
    const float Kp_short = 300.0f;
    const float Kp_long  = 700.0f;
    const float Kd_short = 10.0f;
    const float Kd_long  = 200.0f;

    float t = (L0_avg - LEG_MIN_LENTH) / (LEG_MAX_LENTH - LEG_MIN_LENTH);
    *AntiSplit_Kp = Kp_short + (Kp_long - Kp_short) * t;
    *AntiSplit_Kd = Kd_short + (Kd_long - Kd_short) * t;
}

/**
 * @brief 统一计算左右腿防劈叉力矩命令。
 *
 * 在 LQR 模拟腿力矩基础上依次叠加双腿夹角防劈叉 PID、离心补偿，
 * 以及小陀螺模式下左右腿独立的 phi0 归中 PID。
 *
 * @param[out] L_leg_T_cmd 左腿最终虚拟力矩命令。
 * @param[out] R_leg_T_cmd 右腿最终虚拟力矩命令。
 */
void AntiSplit_Control(float *L_leg_T_cmd, float *R_leg_T_cmd)
{
    float L0_avg = (VMC_L.L0 + VMC_R.L0) * 0.5f;
    AntiSplit_Get_K(&Leg_AntiSplit_PID.Kp, &Leg_AntiSplit_PID.Kd, L0_avg);
    PID_Set_Error(&Leg_AntiSplit_PID,
                  (VMC_R.phi0 - PI / 2.0f) + (VMC_L.phi0 - PI / 2.0f),
                  0.0f);
    float anti_split_out = PID_coculate(&Leg_AntiSplit_PID);

    float centrifugal_comp = centrifugal_comp_gain * d_yaw * d_yaw;

    *L_leg_T_cmd = Leg_L_T + anti_split_out - centrifugal_comp;
    *R_leg_T_cmd = -Leg_R_T + anti_split_out - centrifugal_comp;

    static uint8_t spin_phi0_pid_started = 0;
    if (spinning_flag == 1)
    {
        PID_Set_AngleError(&L_Spin_Phi0_PID, VMC_L.phi0, target_spin_phi0);
        PID_Set_AngleError(&R_Spin_Phi0_PID, VMC_R.phi0, target_spin_phi0);

        if (spin_phi0_pid_started == 0)
        {
            L_Spin_Phi0_PID.pre_error = L_Spin_Phi0_PID.error;
            R_Spin_Phi0_PID.pre_error = R_Spin_Phi0_PID.error;
            spin_phi0_pid_started = 1;
        }

        *L_leg_T_cmd += PID_coculate(&L_Spin_Phi0_PID);
        *R_leg_T_cmd += PID_coculate(&R_Spin_Phi0_PID);
    }
    else
    {
        PID_Clear(&L_Spin_Phi0_PID);
        PID_Clear(&R_Spin_Phi0_PID);
        L_Spin_Phi0_PID.output = 0.0f;
        R_Spin_Phi0_PID.output = 0.0f;
        spin_phi0_pid_started = 0;
    }
}
