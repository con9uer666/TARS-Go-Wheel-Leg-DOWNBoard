#include "cmsis_os.h"
#include <stdint.h>
#include <string.h>
#include "ui.h"
#include "Judge.h"
#include "motor.h"          // Foot_Chassis, pitch_trans
#include "Board2Board.h"    // B2B_offline_flag, shootnum
#include "Gimbal.h"         // yaw_angle_PI
#include "VMC.h"            // VMC_L, VMC_R
#include <math.h>

extern int ui_self_id;

/* ===== 来自 USER_CAN.c 的电机 CAN 心跳计数器（每帧自增） ===== */
extern volatile uint32_t rx_cnt_L_DM8009_0;
extern volatile uint32_t rx_cnt_L_DM8009_1;
extern volatile uint32_t rx_cnt_R_DM8009_0;
extern volatile uint32_t rx_cnt_R_DM8009_1;
extern volatile uint32_t rx_cnt_L_DJ3508;
extern volatile uint32_t rx_cnt_R_DJ3508;
extern volatile uint32_t rx_cnt_4310;
extern volatile uint32_t rx_cnt_wattmeter;

extern int shootnum;

/* ===== width：断联=1，正常=10 ===== */
#define UI_WIDTH_LOST     1
#define UI_WIDTH_ALIVE    10

/* 字符串"隐藏/显示"辅助：通过把 str_length 置 0 让客户端不画该文字 */
static inline void ui_text_show(ui_interface_string_t *t, const char *s, uint8_t len)
{
    strcpy(t->string, s);
    t->str_length = len;
}
static inline void ui_text_hide(ui_interface_string_t *t)
{
    t->string[0]   = '\0';
    t->str_length  = 0;
}

/* 简单的"上次计数 → 是否断联"检测：
   每帧（30Hz）读一次 rx_cnt，与上一帧对比；相等=33ms 内没收到=判断为断线。 */
static inline uint8_t check_lost(volatile uint32_t *cur, uint32_t *last)
{
    uint32_t v = *cur;
    uint8_t  lost = (v == *last);
    *last = v;
    return lost;
}

/**
 * @brief 30Hz 组动态参数刷新
 *        每次 ui_update_g_30HZ() 之前调用一次。
 */
static void UI_RefreshParams_30HZ(void)
{
    static uint32_t last_L0 = 0, last_L1 = 0, last_R0 = 0, last_R1 = 0;
    static uint32_t last_3508L = 0, last_3508R = 0;
    static uint32_t last_4310 = 0;
    static uint32_t last_wattmeter = 0;
    static uint32_t last_wattmeter_change_tick = 0;  // 功率计最近一次计数器变化时的tick

    /* ----- 关节电机断联指示（width 切换） ----- */
    ui_g_30HZ_8009LF->width = check_lost(&rx_cnt_L_DM8009_1, &last_L1) ? UI_WIDTH_LOST : UI_WIDTH_ALIVE; // 左前
    ui_g_30HZ_8009LB->width = check_lost(&rx_cnt_L_DM8009_0, &last_L0) ? UI_WIDTH_LOST : UI_WIDTH_ALIVE; // 左后
    ui_g_30HZ_8009RF->width = check_lost(&rx_cnt_R_DM8009_1, &last_R1) ? UI_WIDTH_LOST : UI_WIDTH_ALIVE; // 右前
    ui_g_30HZ_8009RB->width = check_lost(&rx_cnt_R_DM8009_0, &last_R0) ? UI_WIDTH_LOST : UI_WIDTH_ALIVE; // 右后

    ui_g_30HZ_3508L->width = check_lost(&rx_cnt_L_DJ3508, &last_3508L) ? UI_WIDTH_LOST : UI_WIDTH_ALIVE;
    ui_g_30HZ_3508R->width = check_lost(&rx_cnt_R_DJ3508, &last_3508R) ? UI_WIDTH_LOST : UI_WIDTH_ALIVE;

    /* 4310 Yaw 电机断联 → ROLL 指示灯（PITCH 留作第二批） */
    ui_g_30HZ_ROLL->width = check_lost(&rx_cnt_4310, &last_4310) ? UI_WIDTH_LOST : UI_WIDTH_ALIVE;

    /* 功率计（can3 0x213）断联指示：500ms 内未收到帧才判定为断联 */
    {
        uint32_t cur_wm = rx_cnt_wattmeter;
        uint32_t now    = osKernelSysTick();
        if (cur_wm != last_wattmeter) {
            last_wattmeter            = cur_wm;
            last_wattmeter_change_tick = now;
        }
        ui_g_30HZ_POWER_METER->width =
            ((now - last_wattmeter_change_tick) >= pdMS_TO_TICKS(500))
            ? UI_WIDTH_LOST : UI_WIDTH_ALIVE;
    }

    /* ----- 485 板间通信心跳 ----- */
    ui_g_30HZ_485->width = B2B_offline_flag ? UI_WIDTH_LOST : UI_WIDTH_ALIVE;

    /* ----- 车头方位弧线：车身在云台头坐标系中的角度 = -yaw_angle_PI -----
       弧宽固定 40°，中点指向车身方向，套圈用 mod 360 处理 */
    {
        float body_deg = -yaw_angle_PI * (180.0f / 3.14159265358979f);
        int   mid      = (int)lroundf(body_deg);
        mid = ((mid % 360) + 360) % 360;          // 归一化到 [0, 360)
        ui_g_30HZ_BODY_FRONT->start_angle = (mid + 360 - 20) % 360;
        ui_g_30HZ_BODY_FRONT->end_angle   = (mid + 20)       % 360;
    }

    /* ----- 数字：发射弹丸数 ----- */
    ui_g_30HZ_SHOOT_NUM->number = shootnum;

    /* ----- 车体 pitch 直线：中点 (1650,700)，全长 300，end→start 与水平夹角 = pitch_trans[0]
       (end 高于 start 为正)。屏幕 y 向下，所以"end 高于 start" ⇒ end.y_screen < start.y_screen */
    {
        const int   bp_mid_x = 1650;
        const int   bp_mid_y = 700;
        const float bp_half  = 150.0f;
        float c_p = cosf(pitch_trans[0]);
        float s_p = sinf(pitch_trans[0]);
        float dx  = bp_half * c_p;
        float dy  = bp_half * s_p;
        ui_g_30HZ_BODY_PITCH->start_x = (int)lroundf(bp_mid_x - dx);
        ui_g_30HZ_BODY_PITCH->start_y = (int)lroundf(bp_mid_y + dy);
        ui_g_30HZ_BODY_PITCH->end_x   = (int)lroundf(bp_mid_x + dx);
        ui_g_30HZ_BODY_PITCH->end_y   = (int)lroundf(bp_mid_y - dy);
    }

    /* ----- 左/右腿直线：start 恒定，长度 = L0*10，方向相对竖直角度 = b_phi0
       (start.x < end.x 为正)。屏幕 y 向下，腿向上延伸 ⇒ end.y_screen = start.y - L*cos(phi)。
       L_b_phi0 与 R_b_phi0 已在 VMC.c 中归一到同一符号约定（腿尖向 +x 为正），故公式一致。 */
    {
        const float leg_scale = 10.0f;
        /* 左腿 */
        {
            const int sx = 1600, sy = 700;
            float L  = VMC_L.L0 * leg_scale;
            float ex = sx + L * sinf(VMC_L.b_phi0);
            float ey = sy - L * cosf(VMC_L.b_phi0);
            ui_g_30HZ_L_LEG->start_x = sx;
            ui_g_30HZ_L_LEG->start_y = sy;
            ui_g_30HZ_L_LEG->end_x   = (int)lroundf(ex);
            ui_g_30HZ_L_LEG->end_y   = (int)lroundf(ey);
        }
        /* 右腿 */
        {
            const int sx = 1700, sy = 700;
            float L  = VMC_R.L0 * leg_scale;
            float ex = sx + L * sinf(VMC_R.b_phi0);
            float ey = sy - L * cosf(VMC_R.b_phi0);
            ui_g_30HZ_R_LEG->start_x = sx;
            ui_g_30HZ_R_LEG->start_y = sy;
            ui_g_30HZ_R_LEG->end_x   = (int)lroundf(ex);
            ui_g_30HZ_R_LEG->end_y   = (int)lroundf(ey);
        }
    }

    /* ========== 第二批待完善 ========== */
    /* ui_g_30HZ_NUC          : NUC 心跳/丢失指示  TODO */
    /* ui_g_30HZ_FRIC_SPD_L   : 左摩擦轮转速数字   TODO */
    /* ui_g_30HZ_FRIC_SPD_R   : 右摩擦轮转速数字   TODO */
    /* ui_g_30HZ_AUTO_AIM     : 自瞄状态数字       TODO */
    /* ui_g_30HZ_SUPER_CUP    : 超电电量直线       TODO */
    /* ui_g_30HZ_BUFFER_NUM   : 缓冲数字           TODO */
    /* ui_g_30HZ_PITCH        : PITCH 电机指示     TODO */
    /* ui_g_30HZ_FRIC_L       : 左摩擦轮电机指示   TODO */
    /* ui_g_30HZ_FRIC_R       : 右摩擦轮电机指示   TODO */

    /* ========== 备用预留（未指派含义，先不动） ========== */
    /* ui_g_30HZ_UNNAME1 / UNNAME2 / UNNAME3 */
}

/**
 * @brief 5Hz 组动态参数刷新
 *        每次 ui_update_g_5HZ() 之前调用一次。
 *        显示/隐藏通过 str_length 切换：=0 客户端不画文字，=原值正常显示。
 */
static void UI_RefreshParams_5HZ(void)
{
    /* Chassis_Mode == 1 时不显示 "PLEASE SPIN" */
    if (Foot_Chassis.Chassis_Mode == 1) {
        ui_text_hide(ui_g_5HZ_NewText);
    } else {
        ui_text_show(ui_g_5HZ_NewText, "PLEASE SPIN", 11);
    }

    /* Target_Leg_State == 1 时显示 "LONG LEG"，否则隐藏 */
    if (Foot_Chassis.Target_Leg_State == 1) {
        ui_text_show(ui_g_5HZ_NewText2, "LONG LEG", 8);
    } else {
        ui_text_hide(ui_g_5HZ_NewText2);
    }
}

/**
 * @brief INIT 组动态参数刷新（目前全部第二批待完善，函数留空作锚点）
 *        ui_g_INIT_NewLine ~ NewLine5 : TODO
 *        其它静态字符串/线条按 ui_init_g_INIT 的初值即可，不动。
 */
static void UI_RefreshParams_INIT(void)
{
    /* TODO: NewLine ~ NewLine5 第二批 */
}

/**
 * @brief UI 主任务（覆盖 freertos.c 里的 __weak UI_task）
 *        - 上电后最多等 5 秒读裁判系统 robot_id
 *        - INIT/5HZ/30HZ 三组先各发一次初始化
 *        - 主循环 30Hz：30HZ 组每轮发；5HZ 组每 6 轮发一次；INIT 组每 30 轮（1Hz）重发一次
 */
void UI_task(void const * argument)
{
    uint8_t id = 0;
    for (int i = 0; i < 50; i++) {
        id = JUDGE_GetSelfID();
        if (id != 0) break;
        osDelay(100);
    }
    ui_self_id = (id != 0) ? id : 3;

    /* 初始化：发各分组的 1=新增帧 */
    ui_init_g_INIT();
    osDelay(50);
    ui_init_g_5HZ();
    osDelay(50);
    ui_init_g_30HZ();
    osDelay(50);

    uint32_t cnt = 0;
    for (;;)
    {
        /* ---- 30Hz：每轮 ---- */
        UI_RefreshParams_30HZ();
        ui_update_g_30HZ();

        /* ---- 5Hz：每 6 轮 ---- */
        if ((cnt % 6) == 0) {
            UI_RefreshParams_5HZ();
            ui_update_g_5HZ();
        }

        /* ---- 1Hz：每 30 轮重发 INIT 组（静态层兜底防丢） ---- */
        if ((cnt % 30) == 0 && cnt != 0) {
            UI_RefreshParams_INIT();
            ui_update_g_INIT();
        }

        cnt++;
        osDelay(33);   /* 30Hz 周期 */
    }
}
