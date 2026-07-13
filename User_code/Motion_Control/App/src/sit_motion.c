/**
 * @file sit_motion.c
 * @brief 坐地模式动作组：两腿按斜坡过渡到预设腿长/腿角并锁死轮子，退出时回未站起态。
 */

#include "chassis_behavior_tree.h"
#include "user_pid.h"
#include "Motor_Drv.h"
#include "Gimbal.h"
#include "User_State.h"
#include "State.h"
#include "arm_math.h"
#include "USER_CAN.h"
#include "VMC.h"
#include "observe_task.h"
#include "Leg_Control.h"
#include "Self_Righting.h"
#include "Board2Board.h"
#include "Slope.h"
#include "Wheel_Leg_about.h"
#include "controller.h"
#include "remoter.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "fdcan.h"
#include <math.h>
#include <stdint.h>
#include "imu_temp_ctrl.h"
#include "Angle_about.h"
#include "PowerCtrl.h"
#include "Gas_Spring.h"
#include "buzzer.h"
#include "Wheel_End_Velocity.h"

// 坐地首次进入标志：在 sit_motion 与 balance_standing(倾覆保护复位) 间共享
uint8_t sit_first_entry = 1;

#define SIT_TARGET_L0_L      0.1532f
#define SIT_TARGET_L0_R      0.1555f
#define SIT_TARGET_PHI0_L    0.7353f
#define SIT_TARGET_PHI0_R    2.3164f
#define SIT_RAMP_TIME        0.8f
#define SIT_WHEEL_TORQUE     0.0f
#define SIT_GRAVITY_RATIO    0.3f

static RampGenerator sit_L0_ramp_L, sit_L0_ramp_R;
static RampGenerator sit_phi0_ramp_L, sit_phi0_ramp_R;

uint16_t sit_debug_counter = 0;  // 坐地模式执行周期计数，debug用
uint8_t sit_ramp_done = 0;       // 斜坡过渡是否完成

void Sit_On_Ground_Action(void)
{

    if (sit_first_entry)
    {
        rampInit(&sit_L0_ramp_L, VMC_L.L0, SIT_TARGET_L0_L, SIT_RAMP_TIME, 0.002f);
        rampInit(&sit_L0_ramp_R, VMC_R.L0, SIT_TARGET_L0_R, SIT_RAMP_TIME, 0.002f);
        rampInit(&sit_phi0_ramp_L, VMC_L.phi0, SIT_TARGET_PHI0_L, SIT_RAMP_TIME, 0.002f);
        rampInit(&sit_phi0_ramp_R, VMC_R.phi0, SIT_TARGET_PHI0_R, SIT_RAMP_TIME, 0.002f);
        sit_first_entry = 0;
    }

    sit_debug_counter++;

    rampIterate(&sit_L0_ramp_L);
    rampIterate(&sit_L0_ramp_R);
    rampIterate(&sit_phi0_ramp_L);
    rampIterate(&sit_phi0_ramp_R);

    sit_ramp_done = (fabsf(sit_L0_ramp_L.currentValue - SIT_TARGET_L0_L) < 0.001f)
                 && (fabsf(sit_L0_ramp_R.currentValue - SIT_TARGET_L0_R) < 0.001f)
                 && (fabsf(sit_phi0_ramp_L.currentValue - SIT_TARGET_PHI0_L) < 0.001f)
                 && (fabsf(sit_phi0_ramp_R.currentValue - SIT_TARGET_PHI0_R) < 0.001f);

    // 左腿腿长双环PID
    PID_Set_Error(&L_Leg_L0_POS_PID, VMC_L.L0, sit_L0_ramp_L.currentValue);
    PID_coculate(&L_Leg_L0_POS_PID);
    PID_Set_Error(&L_Leg_L0_SPD_PID, VMC_L.d_L0, L_Leg_L0_POS_PID.output);
    PID_coculate(&L_Leg_L0_SPD_PID);

    // 右腿腿长双环PID
    PID_Set_Error(&R_Leg_L0_POS_PID, VMC_R.L0, sit_L0_ramp_R.currentValue);
    PID_coculate(&R_Leg_L0_POS_PID);
    PID_Set_Error(&R_Leg_L0_SPD_PID, VMC_R.d_L0, R_Leg_L0_POS_PID.output);
    PID_coculate(&R_Leg_L0_SPD_PID);

    // 左腿角度双环PID
    PID_Set_AngleError(&L_Leg_Middle_PID, VMC_L.phi0, sit_phi0_ramp_L.currentValue);
    PID_coculate(&L_Leg_Middle_PID);
    PID_Set_Error(&L_Leg_dphi0_PID, VMC_L.d_phi0, L_Leg_Middle_PID.output);
    PID_coculate(&L_Leg_dphi0_PID);

    // 右腿角度双环PID
    PID_Set_AngleError(&R_Leg_Middle_PID, VMC_R.phi0, sit_phi0_ramp_R.currentValue);
    PID_coculate(&R_Leg_Middle_PID);
    PID_Set_Error(&R_Leg_dphi0_PID, -VMC_R.d_phi0, -R_Leg_Middle_PID.output);
    PID_coculate(&R_Leg_dphi0_PID);

    // VMC映射到电机力矩
    VMC_Chassis_Target.L_F0 = L_Leg_L0_SPD_PID.output + (mg / arm_cos_f32(VMC_L.b_phi0)) * SIT_GRAVITY_RATIO;
    VMC_Chassis_Target.L_T = L_Leg_dphi0_PID.output;
    VMC_Chassis_Target.R_F0 = R_Leg_L0_SPD_PID.output + (mg / arm_cos_f32(VMC_R.b_phi0)) * SIT_GRAVITY_RATIO;
    VMC_Chassis_Target.R_T = -R_Leg_dphi0_PID.output;

    // 轮子小力矩锁死
    VMC_Chassis_Target.L_Wheel_Torque = SIT_WHEEL_TORQUE;
    VMC_Chassis_Target.R_Wheel_Torque = -SIT_WHEEL_TORQUE;
}

void Sit_On_Ground(void)
{
    Sit_On_Ground_Action();

    // 检测退出
    if (!sit_mode_enable)
    {
        start_mode = 0;
        first_run = 1;
        sit_first_entry = 1;
    }
}
