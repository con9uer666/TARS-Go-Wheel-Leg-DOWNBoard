#ifndef VMC_H
#define VMC_H

// #include <stdint.h>
#include "main.h"

//VMC建模vscode://lirentech.file-ref-tags?filePath=VMC.h&snippet=%2F%2FVMC%E5%BB%BA%E6%A8%A1
typedef struct VMC
{
    float phi1, phi2, phi3, phi4;
    float l1, l2, l3, l4, l5;
    float T1, T2;//T1为phi1的转矩，T2为phi4的转矩（大概吧，反正就是phi1和phi4的转矩）
    float A0, B0, C0;//中间量
    float L_BD;

    float Xb, Yb, Xd, Yd, Xc, Yc;

    float L0, d_L0, dd_L0;
    float last_L0, last_d_L0;

    float phi0, d_phi0; // 机身坐标系下的phi0
    float last_phi0, last_d_phi0;

    float b_phi0, d_b_phi0, dd_b_phi0;//大地坐标系下的phi0
    float last_b_phi0, last_d_b_phi0;

    float F;//足端力
    float last_F;
    float T; // 虚拟杆扭矩

    float F_actual;     // 由电机力矩反馈反解出的实际足端力
    float T_actual;     // 由电机力矩反馈反解出的实际虚拟杆扭矩
    float dd_L0_f;      // 滤波后的 d²L0/dt²
    float dd_b_phi0_f;  // 滤波后的 d²b_phi0/dt²

    uint8_t isLeft;//是否为左腿
}VMC_t;

// 底盘目标力矩结构体
typedef struct
{
    float L_F0;
    float L_T;
    float R_F0;
    float R_T;
    float L_Wheel_Torque;
    float R_Wheel_Torque;
} VMC_Chassis_Target_t;

extern VMC_t VMC_L, VMC_R;
extern VMC_Chassis_Target_t VMC_Chassis_Target;

extern volatile uint8_t chassis_hard_stop_flag;
extern volatile uint8_t chassis_soft_stop_flag;

/* 四电机机械零点偏移（rad），在 Keil Watch 窗口修改即可生效 */
extern float motor_zero_L_phi1;   /**< 左腿下关节零点，对应 L_DM8009[1] */
extern float motor_zero_L_phi4;   /**< 左腿上关节零点，对应 L_DM8009[0] */
extern float motor_zero_R_phi1;   /**< 右腿上关节零点，对应 R_DM8009[0] */
extern float motor_zero_R_phi4;   /**< 右腿下关节零点，对应 R_DM8009[1] */

void VMC_Init(VMC_t *VMC, float l1, float l2, float l3, float l4, float l5, uint8_t isLeft);
void VMC_Set_phi1_phi4(VMC_t *VMC, float phi1, float phi4);
void VMC_Get_L0_phi0(VMC_t *VMC);
void VMC_Set_F0_T(VMC_t *VMC, float F, float T);
void VMC_Apply_Chassis_Target(void);
float VMC_Get_Ground_F0(VMC_t *VMC);
void VMC_Coculate();
void VMC_Reset_F_History(void);

#endif
