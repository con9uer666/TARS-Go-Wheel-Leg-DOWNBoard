#ifndef VMC_H
#define VMC_H

typedef struct VMC
{
    float phi1, phi2, phi3, phi4;
    float l1, l2, l3, l4, l5;
    float T1, T2;
    float A0, B0, C0;
    float L_BD;

    float Xb, Yb, Xd, Yd, Xc, Yc;

    float L0;
    float last_L0, last_d_L0;
    float d_L0, dd_L0;
    float phi0;
    float b_phi0;
    float dd_b_phi0;
    float last_phi0;
    float d_phi0, last_d_phi0;
    float F;
    float T;

}VMC_t;

extern VMC_t VMC_L, VMC_R;

void VMC_Init(VMC_t *VMC, float l1, float l2, float l3, float l4, float l5);
void VMC_Set_phi1_phi4(VMC_t *VMC, float phi1, float phi4);
void VMC_Get_L0_phi0(VMC_t *VMC);
void VMC_Set_F0_T(VMC_t *VMC, float F, float T);
float VMC_Get_Ground_F0(VMC_t *VMC);

#endif
