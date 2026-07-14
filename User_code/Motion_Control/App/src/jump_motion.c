/**
 * @file jump_motion.c
 * @brief 跳跃动作组：跳跃指令锁存/解锁、L0 目标阶跃、跳跃失败判定和跳跃蜂鸣器。
 *
 * Jump_Motion_Update() 只维护跳跃状态，并在跳跃期间重新计算腿长 PID；
 * VMC 目标值由 BalanceController 根据返的 jump_active 统一赋值。
 */

#include "chassis_behavior_tree.h"
#include "user_pid.h"
#include "Motor_Drv.h"
#include "Gimbal.h"
#include "User_State.h"
#include "State.h"
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


// 跳跃模式（只读输出）：每周期由 Jump_Motion_Update 根据 jump_cmd 和 jump_locked 计算得出
// =1 表示当前正在跳跃动作组中；倒地自起期间不会被调用，天然互斥
uint8_t jump_mode = 0;
// B2B byte51 原始跳跃指令，由 Board2Board.c 写入
uint8_t jump_cmd = 0;
// 跳跃使能：=1 允许跳跃，=0 即使 jump_mode 已置位也不执行跳跃动作（一票否决）
uint8_t jump_enable = 1;
// 跳跃蜂鸣器独占标志：跳跃期间置1，Error_Buzzer_Tick 看到它必须让位
uint8_t g_jump_buzzer_active = 0;
// 跳跃锁：起跳后检测到离地时上锁，期间 jump_mode 强制为 0；jump_cmd 回 0 时解锁
// 作用：防止一次 byte51=1 期间因落地→再次满足条件而连续触发多次跳跃
static uint8_t jump_locked = 0;
// 跳跃时 L0 目标阶跃量 (单位 m)，叠加到 target_Leg_L0 上让 L0 PID 闭环产生跳跃力
// 建议从 0.08m 起步，根据实际跳跃高度调参
float jump_L0_step_delta = 0.5f;
// 跳跃失败阈值(m)：0.5s 锁存到期时，若任意一腿 L0 变化量小于该值，判定为跳跃失败
float jump_leg_change_threshold = 0.15f;
// 跳跃退出原因（调试用，只读输出）：
//   0 = 未触发 / 锁存中
//   1 = 离地，跳跃成功
//   2 = 0.5s 到期且腿长变化不足 → 明确失败（电机饱和/腿被卡）
//   3 = 0.5s 到期但腿长已变化够 → 超时（疑似传感器漏检离地）
uint8_t jump_fail_reason = 0;

uint8_t Jump_Motion_Update(void)
{

//! 判断是否跳跃
    // 跳跃指令解锁：jump_cmd 回 0 即清锁，允许下一次跳跃
    if (jump_cmd == 0)
    {
        jump_locked = 0;
    }
    // jump_mode 由原始指令和锁共同决定，外部不再直接写
    jump_mode = (jump_cmd && !jump_locked) ? 1 : 0;

    // 实际触发：jump_mode && jump_enable && 短腿 && !小陀螺 && 双腿未离地
    uint8_t jump_active_raw = (jump_mode == 1
                               && jump_enable == 1
                               && spinning_flag == 0
                               && Foot_Chassis.Target_Leg_State == 0
                               && L_off_ground < 10
                               && R_off_ground < 10);

//! 判断是否跳跃成功
    // 跳跃锁存：raw 首次满足 → 启动 0.5s 锁存窗口
    // 锁存期间无视 raw 条件强制 jump_active=1，原表达式无权清零
    // 退出条件（统一在此处上锁，确保"一次跳跃只触发一次"）：
    //   ①离地立即解除 → jump_fail_reason = 1（成功）
    //   ②0.5s 到期 + 腿长变化不足 → jump_fail_reason = 2（失败）
    //   ③0.5s 到期 + 腿长够 → jump_fail_reason = 3（超时但伸够了，疑似漏检）
    static uint8_t jump_active_latched     = 0;
    static int     jump_active_latch_count = 0;
    static float   jump_L0_start_L         = 0.0f;
    static float   jump_L0_start_R         = 0.0f;
    const  int     jump_active_latch_target = motor_HZ / 2;   // 0.5s = 250 cycles

    if (jump_active_raw && !jump_active_latched)
    {
        jump_active_latched     = 1;
        jump_active_latch_count = jump_active_latch_target;
        jump_L0_start_L         = VMC_L.L0;
        jump_L0_start_R         = VMC_R.L0;
        jump_fail_reason        = 0;
    }
    if (jump_active_latched)
    {
        if (L_off_ground >= 10 || R_off_ground >= 10)
        {
            jump_fail_reason    = 1;
            jump_active_latched = 0;
            jump_locked         = 1;
            jump_mode           = 0;
        }
        else if (--jump_active_latch_count <= 0)
        {
            float L_change = VMC_L.L0 - jump_L0_start_L;
            float R_change = VMC_R.L0 - jump_L0_start_R;
            jump_fail_reason    = (L_change < jump_leg_change_threshold
                                   || R_change < jump_leg_change_threshold) ? 2 : 3;
            jump_active_latched = 0;
            jump_locked         = 1;
            jump_mode           = 0;
        }
    }

    uint8_t jump_active = jump_active_latched ? 1 : jump_active_raw;
    if (jump_active)
    {
        // 跳跃时对 L0 setpoint 施加阶跃偏置，让 L0 PID 闭环产生所需的跳跃力
        float jump_target = target_Leg_L0 + jump_L0_step_delta;
        PID_Set_Error(&L_Leg_L0_PID, VMC_L.L0, jump_target);
        PID_Set_Error(&R_Leg_L0_PID, VMC_R.L0, jump_target);
        PID_coculate(&L_Leg_L0_PID);
        PID_coculate(&R_Leg_L0_PID);

    }

    // 跳跃蜂鸣器：跳跃中长鸣中音 sol，结束立刻停。边沿触发避免 PWM 频繁重配
    // 注意必须用 Buzzer_Tone_Max(784) 而不是 Buzzer_sol()：后者音量受 SBUS_CH.CH10 旋钮调制，
    // 旋钮没拨上去时实际占空比为 0 → 完全无声
    if (jump_active != g_jump_buzzer_active)
    {
        if (jump_active) Buzzer_Tone_Max(784);
        else             Stop_Buzzer();
        g_jump_buzzer_active = jump_active;
    }

    return jump_active;
}
