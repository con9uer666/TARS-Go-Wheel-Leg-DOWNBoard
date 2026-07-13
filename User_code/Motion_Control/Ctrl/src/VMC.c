#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "VMC.h"
#include "imu_temp_ctrl.h"
#include <math.h>
#include "Motor_Drv.h"
#include "Gas_Spring.h"
// #include <stdint.h>

VMC_t VMC_L, VMC_R;
VMC_Chassis_Target_t VMC_Chassis_Target;

volatile uint8_t chassis_hard_stop_flag = 0;
volatile uint8_t chassis_soft_stop_flag = 0;

float alpha_d_phi0 = 1.0;
float alpha_phi0 = 1.0;//滤波系数
float alpha_F = 0.5f;
float alpha_dd_L0 = 0.1f;     // dd_L0 一阶 IIR 系数
float alpha_dd_b_phi0 = 0.1f; // dd_b_phi0 一阶 IIR 系数

#define WHEEL_MASS  1.1f      // 驱动轮等效质量 (kg)
#define GRAVITY_ACC 9.81f     // 重力加速度 (m/s^2)

extern float pitch_trans[2];

void VMC_Init(VMC_t *VMC, float l1, float l2, float l3, float l4, float l5, uint8_t isLeft)
{
    VMC->l1 = l1;
    VMC->l2 = l2;
    VMC->l3 = l3;
    VMC->l4 = l4;
    VMC->l5 = l5;
    VMC->isLeft = isLeft;
}

//设置phi1和phi4
void VMC_Set_phi1_phi4(VMC_t *VMC, float phi1, float phi4)
{
    VMC->phi1 = phi1;
    VMC->phi4 = phi4;
}

//算LO和phi0
void VMC_Get_L0_phi0(VMC_t *VMC)
{
    VMC->last_phi0 = VMC->phi0;
    VMC->last_b_phi0 = VMC->b_phi0;
    VMC->last_L0 = VMC->L0;
    VMC->last_d_L0 = VMC->d_L0;

    VMC->Xb = VMC->l1 * arm_cos_f32(VMC->phi1);
    VMC->Xd = VMC->l4 * arm_cos_f32(VMC->phi4) + VMC->l5;

    VMC->Yb = VMC->l1 * arm_sin_f32(VMC->phi1);
    VMC->Yd = VMC->l4 * arm_sin_f32(VMC->phi4);

    VMC->A0 = 2 * VMC->l2 * (VMC->Xd - VMC->Xb);
    VMC->B0 = 2 * VMC->l2 * (VMC->Yd - VMC->Yb);
    arm_sqrt_f32((((VMC->Xd - VMC->Xb) * (VMC->Xd - VMC->Xb)) + ((VMC->Yd - VMC->Yb) * (VMC->Yd - VMC->Yb))), &VMC->L_BD);
    VMC->C0 = (VMC->l2 * VMC->l2) + (VMC->L_BD * VMC->L_BD) - (VMC->l3 * VMC->l3);

    float a;
    arm_sqrt_f32((VMC->A0 * VMC->A0) + (VMC->B0 * VMC->B0) - (VMC->C0 * VMC->C0), &a);
    float y = VMC->B0 + a;
    float x = VMC->A0 + VMC->C0;
    float b;
    arm_atan2_f32(y, x, &b);
    VMC->phi2 = 2 * b;

    VMC->Xc = (VMC->l1 * arm_cos_f32(VMC->phi1)) + (VMC->l2 * arm_cos_f32(VMC->phi2));
    VMC->Yc = (VMC->l1 * arm_sin_f32(VMC->phi1)) + (VMC->l2 * arm_sin_f32(VMC->phi2));

    arm_atan2_f32(VMC->Yb - VMC->Yd + (VMC->l2 * arm_sin_f32(VMC->phi2)), VMC->Xb - VMC->Xd + (VMC->l2 * arm_cos_f32(VMC->phi2)), &VMC->phi3);

    float L0;
    arm_sqrt_f32(((VMC->Xc - (VMC->l5 / 2)) * (VMC->Xc - (VMC->l5 / 2))) + (VMC->Yc * VMC->Yc), &L0);
    VMC->L0 = L0;
    arm_atan2_f32(VMC->Yc, (VMC->Xc - (VMC->l5 / 2)), &VMC->phi0);

    VMC->last_d_phi0 = VMC->d_phi0;
    VMC->d_phi0 = alpha_d_phi0 * ((VMC->phi0 - VMC->last_phi0) / 0.002f) + (1 - alpha_d_phi0) * VMC->d_phi0;
    VMC->d_b_phi0 = (VMC->b_phi0 - VMC->last_b_phi0) / 0.002f;

    VMC->d_L0 = (VMC->L0 - VMC->last_L0)/0.002f;
    VMC->dd_L0 = (VMC->d_L0 - VMC->last_d_L0) / 0.002f;
    VMC->dd_L0_f = alpha_dd_L0 * VMC->dd_L0 + (1.0f - alpha_dd_L0) * VMC->dd_L0_f;

    if(VMC->isLeft)
    {
        VMC_L.last_b_phi0 = VMC_L.b_phi0;
        VMC_L.b_phi0 = alpha_phi0 * (-pitch_trans[0] + VMC_L.phi0 - (PI/2)) + (1 - alpha_phi0) * VMC_L.b_phi0;//滤波
        VMC_L.last_d_b_phi0 = VMC_L.d_b_phi0;
        VMC_L.d_b_phi0 = alpha_d_phi0 * ((VMC_L.b_phi0 - VMC_L.last_b_phi0) / 0.002) + (1 - alpha_d_phi0) * VMC_L.d_b_phi0 ;
        VMC_L.dd_b_phi0 = (VMC_L.d_b_phi0 - VMC_L.last_d_b_phi0) / 0.002f;
        VMC_L.dd_b_phi0_f = alpha_dd_b_phi0 * VMC_L.dd_b_phi0 + (1.0f - alpha_dd_b_phi0) * VMC_L.dd_b_phi0_f;
    }
    else
    {
        VMC_R.last_b_phi0 = VMC_R.b_phi0;
        VMC_R.b_phi0 = alpha_phi0 * (-pitch_trans[0] + (PI - VMC_R.phi0)- (PI/2)) + (1 - alpha_phi0) * VMC_R.b_phi0;
        VMC_R.last_d_b_phi0 = VMC_R.d_b_phi0;
        VMC_R.d_b_phi0 = alpha_d_phi0 * ((VMC_R.b_phi0 - VMC_R.last_b_phi0) / 0.002) + (1 - alpha_d_phi0) * VMC_R.d_b_phi0 ;
        VMC_R.dd_b_phi0 = (VMC_R.d_b_phi0 - VMC_R.last_d_b_phi0)/0.002f;
        VMC_R.dd_b_phi0_f = alpha_dd_b_phi0 * VMC_R.dd_b_phi0 + (1.0f - alpha_dd_b_phi0) * VMC_R.dd_b_phi0_f;
    }
}

//VMC解算
void VMC_Set_F0_T(VMC_t *VMC, float F, float T)
{
    float matrix[4];
    VMC->F = F;
    VMC->F -= Gas_Spring_GetForce(VMC->L0);
    VMC->F = alpha_F * VMC->F + (1 - alpha_F) * VMC->last_F;
    VMC->last_F = VMC->F;
    VMC->T = T;
    matrix[0] = (VMC->l1 * arm_sin_f32(VMC->phi0 - VMC->phi3) * arm_sin_f32(VMC->phi1 - VMC->phi2))/arm_sin_f32(VMC->phi3 - VMC->phi2);
    matrix[1] = (VMC->l1 * arm_cos_f32(VMC->phi0 - VMC->phi3) * arm_sin_f32(VMC->phi1 - VMC->phi2))/(VMC->L0 * arm_sin_f32(VMC->phi3 - VMC->phi2));

    matrix[2] = (VMC->l4 * arm_sin_f32(VMC->phi0 - VMC->phi2) * arm_sin_f32(VMC->phi3 - VMC->phi4))/arm_sin_f32(VMC->phi3 - VMC->phi2);
    matrix[3] = (VMC->l4 * arm_cos_f32(VMC->phi0 - VMC->phi2) * arm_sin_f32(VMC->phi3 - VMC->phi4))/(VMC->L0 * arm_sin_f32(VMC->phi3 - VMC->phi2));

    VMC->T1 = (matrix[0] * VMC->F) + (matrix[1] * VMC->T);
    VMC->T2 = (matrix[2] * VMC->F) + (matrix[3] * VMC->T);

}

// 髋关节轮电机映射函数，含软急停处理
void VMC_Apply_Chassis_Target(void)
{
    float L_F0 = VMC_Chassis_Target.L_F0;
    float L_T = VMC_Chassis_Target.L_T;
    float R_F0 = VMC_Chassis_Target.R_F0;
    float R_T = VMC_Chassis_Target.R_T;
    float L_Wheel_Torque = VMC_Chassis_Target.L_Wheel_Torque;
    float R_Wheel_Torque = VMC_Chassis_Target.R_Wheel_Torque;

    if (chassis_soft_stop_flag)
    {
        L_F0 = 0.0f;
        L_T = 0.0f;
        R_F0 = 0.0f;
        R_T = 0.0f;
        L_Wheel_Torque = 0.0f;
        R_Wheel_Torque = 0.0f;
    }

    VMC_Set_F0_T(&VMC_L, L_F0, L_T);
    VMC_Set_F0_T(&VMC_R, R_F0, R_T);
    L_DJ3508.Target_Torque = L_Wheel_Torque;
    R_DJ3508.Target_Torque = R_Wheel_Torque;
}

// 由电机实际反馈力矩 (tau1, tau2) 反解出实际足端力 F_actual 和虚拟杆扭矩 T_actual
// 矩阵与 VMC_Set_F0_T 中一致：[T1;T2] = M * [F;T]，此处求逆。
static void VMC_Solve_F_T_From_Torque(VMC_t *VMC, float tau1, float tau2)
{
    float m00 = (VMC->l1 * arm_sin_f32(VMC->phi0 - VMC->phi3) * arm_sin_f32(VMC->phi1 - VMC->phi2))
                / arm_sin_f32(VMC->phi3 - VMC->phi2);
    float m01 = (VMC->l1 * arm_cos_f32(VMC->phi0 - VMC->phi3) * arm_sin_f32(VMC->phi1 - VMC->phi2))
                / (VMC->L0 * arm_sin_f32(VMC->phi3 - VMC->phi2));
    float m10 = (VMC->l4 * arm_sin_f32(VMC->phi0 - VMC->phi2) * arm_sin_f32(VMC->phi3 - VMC->phi4))
                / arm_sin_f32(VMC->phi3 - VMC->phi2);
    float m11 = (VMC->l4 * arm_cos_f32(VMC->phi0 - VMC->phi2) * arm_sin_f32(VMC->phi3 - VMC->phi4))
                / (VMC->L0 * arm_sin_f32(VMC->phi3 - VMC->phi2));

    float det = m00 * m11 - m01 * m10;
    // 腿伸/缩极限位姿附近矩阵奇异，回退到指令值（含弹簧）以避免发散
    if (fabsf(det) < 1e-6f)
    {
        VMC->F_actual = VMC->F + Gas_Spring_GetForce(VMC->L0);
        VMC->T_actual = VMC->T;
        return;
    }
    VMC->F_actual = ( m11 * tau1 - m01 * tau2) / det;
    VMC->T_actual = (-m10 * tau1 + m00 * tau2) / det;
}

//计算支持力
float VMC_Get_Ground_F0(VMC_t *VMC)
{
    // 1) 取电机实际反馈力矩，按左右腿映射成 tau1/tau2
    //    左：L_DM8009[1]↔φ1(T1), L_DM8009[0]↔φ4(T2)
    //    右：R_DM8009[0]↔φ1(T1), R_DM8009[1]↔φ4(T2)
    float tau1, tau2;
    if (VMC->isLeft)
    {
        tau1 = L_DM8009[1].Rx_Data.Torque;
        tau2 = L_DM8009[0].Rx_Data.Torque;
    }
    else
    {
        tau1 = R_DM8009[0].Rx_Data.Torque;
        tau2 = R_DM8009[1].Rx_Data.Torque;
    }
    VMC_Solve_F_T_From_Torque(VMC, tau1, tau2);

    // 2) 腿部机构作用于驱动轮的竖直向下分量 P
    //    F_actual 仅是电机贡献的力，必须把氮气弹簧的被动支撑力加回，才是真正的足端总力
    float F_total = VMC->F_actual + Gas_Spring_GetForce(VMC->L0);
    float P = F_total * arm_cos_f32(VMC->b_phi0)
            + (VMC->T_actual * arm_sin_f32(VMC->b_phi0)) / VMC->L0;

    // 3) 轮心竖直方向运动加速度 z̈_w（dd_L0/dd_b_phi0 已经滤波）
    float dd_zw =
          accel_n[2]
        - VMC->dd_L0_f * arm_cos_f32(VMC->b_phi0)
        + 2.0f * VMC->d_L0 * VMC->d_b_phi0 * arm_sin_f32(VMC->b_phi0)
        + VMC->L0 * VMC->dd_b_phi0_f * arm_sin_f32(VMC->b_phi0)
        + VMC->L0 * VMC->d_b_phi0 * VMC->d_b_phi0 * arm_cos_f32(VMC->b_phi0);

    // 4) F_N = P + m_w·g + m_w·z̈_w
    return P + WHEEL_MASS * (GRAVITY_ACC + dd_zw);
}

//算左右VMC的phi1/phi4/L0/phi0
void VMC_Coculate()
{
    VMC_Set_phi1_phi4(&VMC_L, L_DM8009[1].Rx_Data.Position + PI, L_DM8009[0].Rx_Data.Position);
    VMC_Set_phi1_phi4(&VMC_R, R_DM8009[0].Rx_Data.Position + PI, R_DM8009[1].Rx_Data.Position);
    VMC_Get_L0_phi0(&VMC_L);
    VMC_Get_L0_phi0(&VMC_R);
}
