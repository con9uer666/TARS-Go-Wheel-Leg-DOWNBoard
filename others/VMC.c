#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "VMC.h"
#include "imu_temp_ctrl.h"

VMC_t VMC_L, VMC_R;



void VMC_Init(VMC_t *VMC, float l1, float l2, float l3, float l4, float l5)
{
    VMC->l1 = l1;
    VMC->l2 = l2;
    VMC->l3 = l3;
    VMC->l4 = l4;
    VMC->l5 = l5;
}

void VMC_Set_phi1_phi4(VMC_t *VMC, float phi1, float phi4)
{
    VMC->phi1 = phi1;
    VMC->phi4 = phi4;
}

void VMC_Get_L0_phi0(VMC_t *VMC)
{
    VMC->last_phi0 = VMC->phi0;
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

    VMC->d_L0 = (VMC->L0 - VMC->last_L0)/0.002;
    VMC->dd_L0 = (VMC->d_L0 - VMC->last_d_L0)/0.002;
}

void VMC_Set_F0_T(VMC_t *VMC, float F, float T)
{
    float matrix[4];
    VMC->F = F;
    VMC->T = T;
    matrix[0] = (VMC->l1 * arm_sin_f32(VMC->phi0 - VMC->phi3) * arm_sin_f32(VMC->phi1 - VMC->phi2))/arm_sin_f32(VMC->phi3 - VMC->phi2);
    matrix[1] = (VMC->l1 * arm_cos_f32(VMC->phi0 - VMC->phi3) * arm_sin_f32(VMC->phi1 - VMC->phi2))/(VMC->L0 * arm_sin_f32(VMC->phi3 - VMC->phi2));

    matrix[2] = (VMC->l4 * arm_sin_f32(VMC->phi0 - VMC->phi2) * arm_sin_f32(VMC->phi3 - VMC->phi4))/arm_sin_f32(VMC->phi3 - VMC->phi2);
    matrix[3] = (VMC->l4 * arm_cos_f32(VMC->phi0 - VMC->phi2) * arm_sin_f32(VMC->phi3 - VMC->phi4))/(VMC->L0 * arm_sin_f32(VMC->phi3 - VMC->phi2));

    VMC->T1 = (matrix[0] * VMC->F) + (matrix[1] * VMC->T);
    VMC->T2 = (matrix[2] * VMC->F) + (matrix[3] * VMC->T);

}

float VMC_Get_Ground_F0(VMC_t *VMC)
{
    float F0;
    float P, m_w, dd_zw;

    P = VMC->F * arm_cos_f32(VMC->b_phi0) + ((VMC->T * arm_sin_f32(VMC->b_phi0)) / VMC->L0);
    dd_zw = accel_b[2] - VMC->dd_L0 * arm_cos_f32(VMC->b_phi0) + (2 * VMC->d_L0 * VMC->d_phi0 * arm_sin_f32(VMC->b_phi0)) + (VMC->L0 * VMC->dd_b_phi0 * arm_sin_f32(VMC->b_phi0)) + (VMC->L0 * VMC->d_phi0 * VMC->d_phi0 *arm_cos_f32(VMC->b_phi0));

    F0 = P + 1.1 * dd_zw;

    return F0;
}
