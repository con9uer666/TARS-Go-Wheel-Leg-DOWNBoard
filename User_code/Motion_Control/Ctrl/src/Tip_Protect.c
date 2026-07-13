/**
 * @file Tip_Protect.c
 * @brief 站立期间的倾覆检测与收腿保护动作。
 * @details
 * 该模块用于检测机器人站立/运动过程中是否发生倾覆（前后倾倒超过阈值），
 * 并执行收腿保护动作：将双腿收缩至最短、腿角摆正至竖直方向，
 * 同时关闭轮部输出，使机器人以最低姿态落地，降低损坏风险。
 *
 * 工作流程：
 *  1. 每周期调用 Tip_Protect_Detect() 检测 pitch 角是否超过阈值
 *  2. 连续多次超阈值后，触发保护动作（启动 tip_protect_cnt 倒计时）
 *  3. 保护动作期间，每周期调用 Tip_Protect_Action() 执行收腿控制
 *  4. 倒计时结束后，重置运动状态机变量，退出保护
 */

#include "Tip_Protect.h"
#include "chassis_behavior_tree.h"
#include "imu_temp_ctrl.h"

/** @brief 倾覆检测的 pitch 角阈值（度），前后对称 */
#define TIP_PROTECT_PITCH_THRESHOLD 45.0f

/** @brief 连续检测到倾覆所需的去抖计数，防止误触发 */
#define TIP_PROTECT_DETECT_TICKS    10U

/** @brief 保护动作持续的周期数（单次动作时长） */
#define TIP_PROTECT_ACTION_TICKS    250U

/** @brief 保护动作剩余周期计数器，>0 表示正在执行保护动作 */
static uint16_t tip_protect_cnt = 0;

/** @brief 倾覆连续检测计数器，达到 DETECT_TICKS 后触发保护 */
static uint8_t tip_detect_cnt = 0;

/**
 * @brief 倾覆检测函数，每控制周期调用一次。
 * @retval 1 当前处于倾覆保护状态（正在动作或刚触发）
 * @retval 0 未检测到倾覆，正常状态
 *
 * 当 pitch 角绝对值超过 TIP_PROTECT_PITCH_THRESHOLD 并持续
 * TIP_PROTECT_DETECT_TICKS 个周期后，认定机器人已倾覆，
 * 启动保护动作计数器并返回 1。
 * 如果 pitch 回到阈值范围内，则清零检测计数器。
 */
uint8_t Tip_Protect_Detect(void)
{
    /* 仅在非保护状态下进行检测（保护动作中不再重复触发） */
    if (tip_protect_cnt == 0)
    {
        /* pitch 角超出正负阈值范围 */
        if (pitch > TIP_PROTECT_PITCH_THRESHOLD || pitch < -TIP_PROTECT_PITCH_THRESHOLD)
        {
            /* 逐步累加去抖计数，上限为 DETECT_TICKS */
            if (tip_detect_cnt < TIP_PROTECT_DETECT_TICKS)
            {
                tip_detect_cnt++;
            }

            /* 达到去抖阈值，触发保护动作 */
            if (tip_detect_cnt >= TIP_PROTECT_DETECT_TICKS)
            {
                tip_protect_cnt = TIP_PROTECT_ACTION_TICKS;
                tip_detect_cnt = 0;
            }
        }
        else
        {
            /* pitch 在安全范围内，清零去抖计数 */
            tip_detect_cnt = 0;
        }
    }

    return tip_protect_cnt > 0;
}

/**
 * @brief 倾覆保护动作执行函数，每控制周期调用一次。
 *
 * 在保护动作期间，执行以下控制：
 *  - 先运行 VMC 解算，获取当前腿长/腿角状态
 *  - 腿长位速双环 PID：目标为最短腿长 LEG_MIN_LENTH，使双腿迅速收回
 *  - 腿角位速双环 PID：目标为竖直方向（PI/2），将腿摆正
 *    - 右腿因机构对称性，速度/输出需取反
 *  - 轮部扭矩置零，停止驱动
 *  - 保护周期结束后重置运动状态机变量
 */
void Tip_Protect_Action(void)
{

    /* ===== 腿长位速双环：目标为最短腿长 ===== */
    PID_Set_Error(&L_Leg_L0_POS_PID, VMC_L.L0, LEG_MIN_LENTH);
    PID_Set_Error(&R_Leg_L0_POS_PID, VMC_R.L0, LEG_MIN_LENTH);
    PID_coculate(&L_Leg_L0_POS_PID);
    PID_coculate(&R_Leg_L0_POS_PID);
    /* 速度环以位置环输出为目标速度 */
    PID_Set_Error(&L_Leg_L0_SPD_PID, VMC_L.d_L0, L_Leg_L0_POS_PID.output);
    PID_Set_Error(&R_Leg_L0_SPD_PID, VMC_R.d_L0, R_Leg_L0_POS_PID.output);
    PID_coculate(&L_Leg_L0_SPD_PID);
    PID_coculate(&R_Leg_L0_SPD_PID);

    /* ===== 腿角双环：目标为竖直方向，角度环走最短路径 ===== */
    /* 左腿 */
    PID_Set_AngleError(&L_Leg_Middle_PID, VMC_L.phi0, PI / 2.0f);
    PID_coculate(&L_Leg_Middle_PID);
    PID_Set_Error(&L_Leg_dphi0_PID, VMC_L.d_phi0, L_Leg_Middle_PID.output);
    PID_coculate(&L_Leg_dphi0_PID);

    /* 右腿：因机构对称，速度与输出需取反 */
    PID_Set_AngleError(&R_Leg_Middle_PID, VMC_R.phi0, PI / 2.0f);
    PID_coculate(&R_Leg_Middle_PID);
    PID_Set_Error(&R_Leg_dphi0_PID, -VMC_R.d_phi0, -R_Leg_Middle_PID.output);
    PID_coculate(&R_Leg_dphi0_PID);

    /* ===== 将控制结果写入底盘目标值 ===== */
    VMC_Chassis_Target.L_F0 = L_Leg_L0_SPD_PID.output;
    VMC_Chassis_Target.L_T = L_Leg_dphi0_PID.output;
    VMC_Chassis_Target.R_F0 = R_Leg_L0_SPD_PID.output;
    /* 右腿力矩取反，匹配机构方向 */
    VMC_Chassis_Target.R_T = -R_Leg_dphi0_PID.output;
    /* 保护期间轮部不输出扭矩 */
    VMC_Chassis_Target.L_Wheel_Torque = 0.0f;
    VMC_Chassis_Target.R_Wheel_Torque = 0.0f;

    /* 保护动作周期倒计时 */
    tip_protect_cnt--;

    /* 保护周期结束，重置运动状态机相关变量 */
    if (tip_protect_cnt == 0)
    {
        start_mode = 0;
        upstares_mode = 0;
        first_run = 1;
        sit_first_entry = 1;
    }

    /* 保护期间熄灭 LED（PE13），指示保护状态 */
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, 0);
}
