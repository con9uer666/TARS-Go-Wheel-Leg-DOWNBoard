/**
 * @file leg_retract_common.c
 * @brief 收腿转角公共逻辑：自起前收腿(self_righting_retract) 与 上台阶收腿(stair_climb) 共用。
 *
 * turn_ctrl_with_stuck_flip()：收腿起立 State=1 转角阶段的控制 + 卡住反向绕长路子状态机。
 * 各腿的子状态/计数/反绕进度为持久量，由调用方按指针传入并在切换动作时复位，故定义在此。
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

//?收腿状态机共享量
uint8_t L_Leg_State, R_Leg_State;   //收腿阶段，0为收腿中，1为起立过程中收腿完成，2为上台阶过程中收腿完成
uint16_t L_Ready_Count, R_Ready_Count;

StairSub_t L_stair_sub = STAIR_SUB_TURN_FWD;     // 左腿子状态
StairSub_t R_stair_sub = STAIR_SUB_TURN_FWD;     // 右腿子状态
int   L_sub_dwell = 0,          R_sub_dwell = 0;     // 进入当前子状态后的 tick 数，< dwell 门禁不允许再切换（防抖）
float L_rev_dir = 0.0f,         R_rev_dir = 0.0f;    // REV 期间的转向 ±1，FWD→REV 时按"短路径反方向"锁定
float L_rev_long_remain = 0.0f, R_rev_long_remain = 0.0f;  // REV 总需要走的弧长 = 2π − |短路径剩余|，FWD→REV 时锁定
float L_rev_traveled = 0.0f,    R_rev_traveled = 0.0f;     // REV 期间对 d_phi0 积分得到的已走弧长，避免用 phi0 做减法在 ±π 处跳变

/**
 * @brief 收腿起立 State=1 转角阶段的控制 + 卡住反向绕长路。
 *
 * 子状态机（每腿各一套）：
 *   FWD 默认：Middle_PID(角度) → dphi0_PID(角速度) 串级，走短路径。
 *   FWD 卡住：|error|>0.1 且 |d_phi0|<0.0873rad/s 持续 100ms，且进入 FWD ≥ 200ms
 *            → 锁 rev_dir / rev_long_remain / rev_traveled=0，切 REV。
 *   REV 默认：leg_turn_speed_control(rev_dir * 1.2 rad/s, 15 N·m)，不受 ShortestAngle 限制。
 *   REV 到位：积分 rev_traveled ≥ long_remain − 0.05，或过半圈后短路径误差 < 0.05（兜底防积分漂移）。
 *   REV 又卡：切回 FWD 无限重试。
 *
 * is_right 用于保留右腿原有的 −d_phi0 / −output 符号约定（仅 FWD 分支需要；
 * REV 走 leg_turn_speed_control，速度目标自带符号，不再反置）。
 *
 * @param VMC              左右腿 VMC 结构指针
 * @param is_right         0=左腿, 1=右腿
 * @param target_angle     目标 phi0，单位 rad
 * @param pid_middle       角度 PID（外环）
 * @param pid_dphi0        角速度 PID（内环）
 * @param sub              子状态，持久变量地址
 * @param dwell            dwell 计数，持久变量地址
 * @param rev_dir          REV 转向 ±1，持久变量地址
 * @param rev_long_remain  REV 目标弧长，持久变量地址
 * @param rev_traveled     REV 已走弧长积分，持久变量地址
 * @param out_T            [out] 本周期该腿的目标转矩命令
 * @return 1 = 此周期视为到位（外层 Ready_Count 累加）, 0 = 未到位
 */
int turn_ctrl_with_stuck_flip(
    VMC_t *VMC, int is_right, float target_angle,
    user_pid_t *pid_middle, user_pid_t *pid_dphi0,
    StairSub_t *sub, int *dwell,
    float *rev_dir, float *rev_long_remain, float *rev_traveled,
    float *out_T)
{
    (*dwell)++;
    int near = 0;                               // 本周期是否算到位

    if (*sub == STAIR_SUB_TURN_FWD)
    {
        // 短路径 PID 串级：Middle_PID 出 dphi0 目标，dphi0_PID 出转矩
        PID_Set_AngleError(pid_middle, VMC->phi0, target_angle);    // 内部走 ShortestAngleDelta
        PID_coculate(pid_middle);
        float dphi0_in = is_right ? -VMC->d_phi0        : VMC->d_phi0;          // 右腿符号约定
        float dphi0_tg = is_right ? -pid_middle->output : pid_middle->output;
        PID_Set_Error(pid_dphi0, dphi0_in, dphi0_tg);
        PID_coculate(pid_dphi0);
        *out_T = is_right ? -pid_dphi0->output : pid_dphi0->output;

        // 角度误差进入容差带 → 认为到位
        if (fabsf(pid_middle->error) <= 0.1f) near = 1;

        // 卡住判据：dwell 门禁过了 + 误差仍大 + d_phi0 死区持续时间达到 → 切 REV
        if (!near && *dwell > 100 &&
            fabsf(pid_middle->error) > 0.1f &&
            leg_turn_stuck_detect(VMC, 0.0873f, 0.1f))
        {
            float se = ShortestAngleDelta(target_angle, VMC->phi0);
            *rev_dir         = (se > 0.0f) ? -1.0f : 1.0f;      // 长路径 = 短路径反方向
            *rev_long_remain = 2.0f * PI - fabsf(se);           // 绕一圈减去短路径剩余
            *rev_traveled    = 0.0f;
            *sub = STAIR_SUB_TURN_REV;
            *dwell = 0;
            leg_turn_stuck_reset(VMC);      // 清共享计数器，避免 REV 一进来就重新触发
            *out_T = 0.0f;                  // 本周期不再下发 FWD 的残留转矩
        }
    }
    else  // STAIR_SUB_TURN_REV
    {
        // 纯速度控制走长路径，不受 ShortestAngle 限制
        *out_T = leg_turn_speed_control(VMC, (*rev_dir) * 1.2f, 15.0f, 0.0f);
        // 用 d_phi0 积分记已走弧长，避免 phi0 在 ±π 翻转导致的跳变
        *rev_traveled += (*rev_dir) * VMC->d_phi0 * 0.002f;     // dt = 1/500 Hz = 2 ms

        float se_now = fabsf(ShortestAngleDelta(target_angle, VMC->phi0));
        // 到位判定：主判用积分弧长，兜底用过半圈后短路径误差，防积分长期漂移
        int arrived = (*rev_traveled >= *rev_long_remain - 0.05f) ||
                      (*rev_traveled > PI && se_now < 0.1f);
        if (arrived)
        {
            near = 1;
            *sub = STAIR_SUB_TURN_FWD;      // 复位回 FWD，等外层计满 Ready_Count 后进入 State=2
            *dwell = 0;
            leg_turn_stuck_reset(VMC);
        }
        else if (*dwell > 100 &&
                 (*rev_traveled < *rev_long_remain - 0.1f) &&
                 leg_turn_stuck_detect(VMC, 0.0873f, 0.1f))
        {
            // 反向也卡住（且确认不是快到位的低速）→ 切回 FWD，无限重试
            *sub = STAIR_SUB_TURN_FWD;
            *dwell = 0;
            leg_turn_stuck_reset(VMC);
        }
    }
    return near;
}
