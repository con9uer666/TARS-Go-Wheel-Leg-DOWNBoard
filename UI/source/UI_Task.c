// ============================================================================
//  UI_Task.c —— UI 模块"业务层"
//  ----------------------------------------------------------------------------
//  职责：
//    1. 把车上各处的实时状态（电机心跳、姿态、转速、模式…）映射到 ui_g.c
//       中各图元指针所指字段。
//    2. 在 10Hz 主循环里按 cnt%10 单帧轮询发送，压在裁判系统每机器人 10pps
//       / 3.75KBps 的上限内。
//    3. INIT 静态层每 3s 兜底重发，防丢包后静态图元永远消失。
// ============================================================================

#include "cmsis_os.h"
#include <stdint.h>
#include <string.h>
#include "ui.h"
#include "Judge.h"
#include "motor.h"          // Foot_Chassis, pitch_trans, fric_speed_l_rpm/fric_speed_r_rpm, AIM_State
#include "Board2Board.h"    // B2B_offline_flag, shootnum
#include "Gimbal.h"         // yaw_angle_PI（云台 yaw 角，弧度）
#include "VMC.h"            // VMC_L, VMC_R（虚拟模型控制器，提供 L0 腿长 / b_phi0 腿角）
#include <math.h>

extern int ui_self_id;     // 当前机器人 id（ui_interface.c 定义，本任务上电写入）

/* ===== 来自 USER_CAN.c 的电机 CAN 心跳计数器（每收到一帧自增） =====
   断联判定：每个 30Hz tick 读一次，若与上一次相同，说明 33ms 内没有新帧来，
   即视为断联。这要求 USER_CAN 必须无条件自增计数器，不能被任何过滤逻辑跳过。 */
extern volatile uint32_t rx_cnt_L_DM8009_0;  // 左侧 DM8009 关节电机 #0
extern volatile uint32_t rx_cnt_L_DM8009_1;  // 左侧 DM8009 关节电机 #1
extern volatile uint32_t rx_cnt_R_DM8009_0;  // 右侧 DM8009 关节电机 #0
extern volatile uint32_t rx_cnt_R_DM8009_1;  // 右侧 DM8009 关节电机 #1
extern volatile uint32_t rx_cnt_L_DJ3508;    // 左 3508 驱动轮
extern volatile uint32_t rx_cnt_R_DJ3508;    // 右 3508 驱动轮
extern volatile uint32_t rx_cnt_4310;        // DM4310 (Yaw) 电机
extern volatile uint32_t rx_cnt_wattmeter;   // 功率计（CAN3 0x213）

extern int shootnum;        // 已发射弹丸总数（Board2Board 维护）

/* ===== ui_g.c 里的非 static 子帧 update 函数，10Hz 轮询时直接调用 =====
   ui_g.c 自动生成代码默认非 static，所以这里 extern 出来即可。
   FAST 子帧承载姿态可视化（车身朝向/双腿/pitch），分到 4 个 slot 拿 4Hz。 */
extern void _ui_update_g_30HZ_0(void);
extern void _ui_update_g_30HZ_1(void);
extern void _ui_update_g_30HZ_FAST(void);
extern void _ui_update_g_30HZ_2(void);
extern void _ui_update_g_30HZ_3(void);
extern void _ui_update_g_5HZ_0(void);
extern void _ui_update_g_5HZ_1(void);

/* ===== 心跳指示灯线宽常量：圆图元的 width 同时控制粗细，10=醒目，1=极细 =====
   断联=1（视觉上变成一个小点）、正常=10（看到一个明显的实心圆）。 */
#define UI_WIDTH_LOST     1
#define UI_WIDTH_ALIVE    10

/* ---- 字符串"显示/隐藏"辅助 -----------------------------------------------
   实现思路：不删除图元（删除后再加回来要发新增帧），只改 str_length。
   str_length=0 时客户端会忽略该字符串，但图元继续被维护，下次显示无需重建。 */
static inline void ui_text_show(ui_interface_string_t *t, const char *s, uint8_t len)
{
    strcpy(t->string, s);
    t->str_length = len;
}
static inline void ui_text_hide(ui_interface_string_t *t)
{
    /* 必须把整 30 字节 string 清零：ui_proc_string_frame 会按 strlen() 重算
       str_length，但客户端实际渲染按帧上 30 字节缓冲走 —— 只清 string[0] 会
       让"PLEASE SPIN" 只息灭首字母 P，剩下 "LEASE SPIN" 仍残留可见。 */
    memset(t->string, 0, sizeof(t->string));
    t->str_length  = 0;
}

/**
 * @brief 断联检测：比较"当前 rx_cnt"和"上次 rx_cnt"。
 * @param cur  电机的 rx_cnt 指针（volatile，因为 USER_CAN ISR 写它）
 * @param last 上次保存的值的指针（调用者自己持有 static 存储）
 * @return 1 = 这次和上次相同 → 33ms 内没收到帧 → 判定断联
 *         0 = 计数器变化过 → 正常
 *
 * 副作用：把当前值写回 *last，给下次调用使用。
 * 注意：本函数必须被 30Hz 周期性调用，否则会误判。
 */
static inline uint8_t check_lost(volatile uint32_t *cur, uint32_t *last)
{
    uint32_t v = *cur;
    uint8_t  lost = (v == *last);
    *last = v;
    return lost;
}

/**
 * @brief 30Hz 组动态参数刷新
 *        把车上各处的实时状态搬到 ui_g_30HZ_* 各指针所指的图元字段上。
 *        实际被 UI_task 在 10Hz 主循环里"每 tick 都跑一次"调用 —— 名字叫
 *        30HZ 只是因为初始 Designer 模板默认按 30Hz 设计；现已改成 10Hz 主调度。
 */
static void UI_RefreshParams_30HZ(void)
{
    // 各电机心跳计数器的"上次值"，保留在 static 里跨调用比较。
    // 0 初值保证开机后第一次调用一定判定为"未收到"(=断联)，等下次再纠正。
    static uint32_t last_L0 = 0, last_L1 = 0, last_R0 = 0, last_R1 = 0;
    static uint32_t last_3508L = 0, last_3508R = 0;
    static uint32_t last_4310 = 0;
    static uint32_t last_wattmeter = 0;
    static uint32_t last_wattmeter_change_tick = 0;  // 功率计最近一次计数器变化时的 tick

    /* ----- 关节电机断联指示（width 切换） -----
       关节电机心跳频率 >= 100Hz，10Hz 检测足够灵敏。 */
    ui_g_30HZ_8009LF->width = check_lost(&rx_cnt_L_DM8009_1, &last_L1) ? UI_WIDTH_LOST : UI_WIDTH_ALIVE; // 左前
    ui_g_30HZ_8009LB->width = check_lost(&rx_cnt_L_DM8009_0, &last_L0) ? UI_WIDTH_LOST : UI_WIDTH_ALIVE; // 左后
    ui_g_30HZ_8009RF->width = check_lost(&rx_cnt_R_DM8009_1, &last_R1) ? UI_WIDTH_LOST : UI_WIDTH_ALIVE; // 右前
    ui_g_30HZ_8009RB->width = check_lost(&rx_cnt_R_DM8009_0, &last_R0) ? UI_WIDTH_LOST : UI_WIDTH_ALIVE; // 右后

    // 驱动轮 3508
    ui_g_30HZ_3508L->width = check_lost(&rx_cnt_L_DJ3508, &last_3508L) ? UI_WIDTH_LOST : UI_WIDTH_ALIVE;
    ui_g_30HZ_3508R->width = check_lost(&rx_cnt_R_DJ3508, &last_3508R) ? UI_WIDTH_LOST : UI_WIDTH_ALIVE;

    /* 4310 Yaw 电机断联 → 复用 ROLL 指示灯位（PITCH 那个槽留作第二批接其他电机）*/
    ui_g_30HZ_ROLL->width = check_lost(&rx_cnt_4310, &last_4310) ? UI_WIDTH_LOST : UI_WIDTH_ALIVE;

    /* 功率计（CAN3 0x213）断联指示：上报频率较低 (10Hz)，简单 check_lost 会误判，
       改成"500ms 内 rx_cnt 没变化"才判定断联。 */
    {
        uint32_t cur_wm = rx_cnt_wattmeter;   // 当前心跳计数
        uint32_t now    = osKernelSysTick();  // 当前 RTOS tick
        if (cur_wm != last_wattmeter) {
            last_wattmeter             = cur_wm;
            last_wattmeter_change_tick = now;
        }
        ui_g_30HZ_POWER_METER->width =
            ((now - last_wattmeter_change_tick) >= pdMS_TO_TICKS(500))
            ? UI_WIDTH_LOST : UI_WIDTH_ALIVE;
    }

    /* ----- 485 板间通信心跳：直接看 Board2Board 模块导出的 offline 标志 ----- */
    ui_g_30HZ_485->width = B2B_offline_flag ? UI_WIDTH_LOST : UI_WIDTH_ALIVE;

    /* ----- 车头方位弧线：车身在云台头坐标系中的角度 = -yaw_angle_PI -----
       弧宽固定 40°，中点 mid 指向车身方向；mid=0 时弧覆盖 [340, 20]（绕零回绕）。
       Judge 客户端按 start_angle → end_angle 逆时针绘制，允许 start > end 自然回绕，
       所以两端直接用 (mid+340) % 360 / (mid+20) % 360 即可，不需要夹边界。 */
    {
        float body_deg = yaw_angle_PI * (180.0f / 3.14159265358979f);  // 弧度 → 角度
        int   mid      = (int)lroundf(body_deg);
        mid = ((mid % 360) + 360) % 360;          // 归一化到 [0, 360)
        ui_g_30HZ_BODY_FRONT->start_angle = (mid + 340) % 360;
        ui_g_30HZ_BODY_FRONT->end_angle   = (mid + 20)  % 360;
    }

    /* ----- 数字：发射弹丸数（取 Board2Board 模块累加值） ----- */
    ui_g_30HZ_SHOOT_NUM->number = shootnum;

    /* ----- 数字：自瞄状态（视觉模块导出枚举，0=关 / 1=锁敌 / 2=开火） ----- */
    ui_g_30HZ_AUTO_AIM->number = AIM_State;

    /* ----- 数字：左右摩擦轮实测转速 -----
       直接取 fric_speed_*_rpm 抖动剧烈，所以做一阶 IIR 低通：
         y[n] = (1-α)·y[n-1] + α·x[n]
       这里调用频率 ≈ 10Hz，α=0.5 → 时间常数 τ ≈ 0.5 × tick ≈ 50~100ms，
       够把字面跳动压成肉眼舒服的过渡。
       （注释里写"30Hz"是历史名残留，实际调用频率以 UI_task 主循环为准。） */
    {
        static float fric_disp_l = 0.0f, fric_disp_r = 0.0f;
        const float alpha = 0.5f;
        fric_disp_l = (1.0f - alpha) * fric_disp_l + alpha * (float)fric_speed_l_rpm;
        fric_disp_r = (1.0f - alpha) * fric_disp_r + alpha * (float)fric_speed_r_rpm;
        ui_g_30HZ_FRIC_SPD_L->number = (int32_t)lroundf(fric_disp_l);
        ui_g_30HZ_FRIC_SPD_R->number = (int32_t)lroundf(fric_disp_r);
    }

    /* ----- 车体 pitch 直线：可视化倾覆角度 -----
       几何模型：以 (1650,700) 为中点，半长 150，整条直线绕中点旋转。
       角度 = pitch_trans[0]（弧度，IMU 解出的车身俯仰角）。
       约定：end 比 start 高 (屏幕 y 小) 表示正俯仰。
            dx = halfLen·cosθ ; dy = halfLen·sinθ
            start = (mid_x - dx, mid_y + dy)
            end   = (mid_x + dx, mid_y - dy)
       屏幕 y 轴向下，所以正 pitch → end.y 比 start.y 小。 */
    {
        const int   bp_mid_x = 1650;    // 直线中点 X
        const int   bp_mid_y = 700;     // 直线中点 Y
        const float bp_half  = 150.0f;  // 直线半长（像素）
        float c_p = cosf(pitch_trans[0]);
        float s_p = sinf(pitch_trans[0]);
        float dx  = bp_half * c_p;      // 水平分量
        float dy  = bp_half * s_p;      // 垂直分量
        ui_g_30HZ_BODY_PITCH->start_x = (int)lroundf(bp_mid_x - dx);
        ui_g_30HZ_BODY_PITCH->start_y = (int)lroundf(bp_mid_y + dy);
        ui_g_30HZ_BODY_PITCH->end_x   = (int)lroundf(bp_mid_x + dx);
        ui_g_30HZ_BODY_PITCH->end_y   = (int)lroundf(bp_mid_y - dy);
    }

    /* ----- 左/右腿直线：可视化腿长 + 摆角 -----
       几何模型：start 在车体上固定点 (sx, sy)，长度 = L0 (米) × leg_scale（缩放到屏幕像素，
       让 0.1 ~ 0.4m 的腿长落在合适视觉尺度 → 75~300 像素），方向相对竖直方向角度 = b_phi0。
            end.x = start.x + L · sin(φ)
            end.y = start.y - L · cos(φ)    // 屏幕 y 向下，腿向上延伸 → 减号
       L 和 R 都用同一公式：因为 VMC.c 已把两侧 b_phi0 归一到"腿尖向 +x 为正"
       这同一符号约定。 */
    {
        const float leg_scale = 750.0f;    // L0(米) → 像素的放大倍数

        /* 左腿 */
        {
            const int sx = 1600, sy = 700; // 左腿起点（车体左半边某处）
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
            const int sx = 1700, sy = 700; // 右腿起点（车体右半边某处）
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
    /* ui_g_30HZ_SUPER_CUP    : 超电电量直线       TODO */
    /* ui_g_30HZ_BUFFER_NUM   : 缓冲数字           TODO */
    /* ui_g_30HZ_PITCH        : PITCH 电机指示     TODO */
    /* ui_g_30HZ_FRIC_L       : 左摩擦轮电机指示   TODO */
    /* ui_g_30HZ_FRIC_R       : 右摩擦轮电机指示   TODO */

    /* UNNAME3 当作"程序还活着"的心跳指示灯 ——
       本函数每 tick 调用一次（10Hz），blink_cnt 累到 15 → 1.5s 翻转一次 width，
       完整闪烁周期 3s。如果 UI 任务挂了，这个灯就会停在某一状态不再变化，
       是最直观的"任务死了"指标。
       （注释里写"30Hz × 15"是历史名残留，实际调用频率 ≈ 10Hz。） */
    {
        static uint8_t blink_cnt = 0;
        static uint8_t blink_on  = 1;
        if (++blink_cnt >= 15) {
            blink_cnt = 0;
            blink_on ^= 1;
        }
        ui_g_30HZ_UNNAME3->width = blink_on ? UI_WIDTH_ALIVE : UI_WIDTH_LOST;
    }

    /* ========== 备用预留（未指派含义，先不动） ========== */
    /* ui_g_30HZ_UNNAME1 / UNNAME2 */
}

/**
 * @brief 5Hz 组动态参数刷新
 *        显示/隐藏通过 str_length 切换：=0 客户端不画文字，=原长度正常显示。
 *        不删除图元，避免反复"新增/删除"造成客户端图层抖动。
 */
static void UI_RefreshParams_5HZ(void)
{
    /* PLEASE SPIN：步兵小陀螺提示。Chassis_Mode == 1 表示已经在陀螺，
       这时把提示隐藏掉；其它模式（跟随 / 静止）就显示出来提醒操作手开陀螺。 */
    if (Foot_Chassis.Chassis_Mode == 1) {
        ui_text_hide(ui_g_5HZ_NewText);
    } else {
        ui_text_show(ui_g_5HZ_NewText, "PLEASE SPIN", 11);
    }

    /* LONG LEG：长腿模式提示。Target_Leg_State == 1 表示目标在长腿状态，
       显示提示让操作手知道当前腿正在伸长；其它腿态隐藏。 */
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
 *        即使全空，UI_task 也会按 3s 调用 ui_update_g_INIT() 去重发原值，
 *        防止丢包后客户端永远丢失这些静态图元。
 */
static void UI_RefreshParams_INIT(void)
{
    /* TODO: NewLine ~ NewLine5 第二批 */
}

/**
 * @brief UI 主任务（覆盖 freertos.c 里的 __weak UI_task）
 *
 * 启动流程：
 *   1. 上电后最多等 5 秒读裁判系统 robot_id（每 100ms 重试一次，最多 50 次）。
 *      读不到就用默认值 3（红方步兵 3）。
 *   2. 先发 INIT / 5HZ / 30HZ 三组的"新增"帧，让客户端注册全部图元。
 *      三次发送之间各 osDelay(50) 留余量给裁判系统串口缓冲。
 *   3. 进入 10Hz 主循环，按 cnt%10 单帧轮询发送。
 *
 * 调度表（10 tick = 1s 一个周期）：
 *     0,3,6,8 -> 30HZ_FAST  (姿态可视化：BODY_FRONT/L_LEG/R_LEG/BODY_PITCH，4Hz)
 *     1,5     -> 30HZ_0     (4 数字 + 心跳，2Hz)
 *     2       -> 30HZ_1     (右侧 8009/超电，1Hz)
 *     4       -> 30HZ_2     (心跳/blink，1Hz)
 *     7       -> 30HZ_3     (右 3508/摩擦轮等，1Hz)
 *     9       -> 5HZ 文字   (on-change 优先；无变化时交替兜底重发)
 *   总包速率正好 10pps，卡在裁判系统每机器人 10Hz / 3.75KBps 上限内。
 *
 *   设计取舍：把 BODY_FRONT/L_LEG/R_LEG/BODY_PITCH 拆到 FAST 子帧，让姿态
 *   动画拿到 4Hz 而不是原来的 2Hz（肉眼差异显著）。代价是其它 30HZ 子帧从
 *   2Hz 降到 1Hz——这些主要是心跳/数字，1Hz 已够。
 *
 *   5HZ 文字（PLEASE SPIN / LONG LEG）从"各占 1 slot"压成"共享 1 slot"：
 *     · 状态变化时立即在下一个 slot 9 把变化对应的那条发出（最快 1s 内可见）
 *     · 两条都同时变化 → 交替优先，保证两条都不超过 2s 同步到客户端
 *     · 无变化 → 交替兜底重发，每条 0.5Hz，防止丢包后客户端永久残留旧文字
 *
 * 兜底：cnt%30==0 (每 3s) 重发一次 INIT 静态层，防止丢包后图元永久消失。
 */
void UI_task(void const * argument)
{
    /* ----- 1. 上电等待裁判系统给出本机 id ----- */
    uint8_t id = 0;
    for (int i = 0; i < 50; i++) {       // 50 × 100ms = 最多等 5s
        id = JUDGE_GetSelfID();
        if (id != 0) break;
        osDelay(100);
    }
    ui_self_id = (id != 0) ? id : 3;     // 兜底默认红 3

    /* ----- 2. 发各分组的"新增"帧，注册图元到客户端 ----- */
    ui_init_g_INIT();
    osDelay(50);
    ui_init_g_5HZ();
    osDelay(50);
    ui_init_g_30HZ();
    osDelay(50);

    /* ----- 3. 10Hz 主循环 ----- */
    uint32_t cnt = 0;                    // 主循环 tick 计数，自然 32 位回卷
    for (;;)
    {
        /* 每 tick 都跑一次参数刷新：函数体只是改指针所指的字段，CPU 开销小，
           保持所有图元的"待发送内容"是最新的；实际只发 cnt%10 选中的那一帧。
           这样无论调度落到哪一帧，发出去的数据都是当前最新的。 */
        UI_RefreshParams_30HZ();
        UI_RefreshParams_5HZ();

        /* 10 tick 一个轮询周期，按调度表分发到具体子帧 */
        switch (cnt % 10) {
        case 0: case 3: case 6: case 8:
            _ui_update_g_30HZ_FAST();  break;       // 5 图元帧，4Hz 高刷
        case 1: case 5:
            _ui_update_g_30HZ_0();     break;       // 7 图元帧，2Hz
        case 2:
            _ui_update_g_30HZ_1();     break;       // 5 图元帧，1Hz
        case 4:
            _ui_update_g_30HZ_2();     break;       // 7 图元帧，1Hz
        case 7:
            _ui_update_g_30HZ_3();     break;       // 5 图元帧，1Hz
        case 9: {
            /* 5HZ 共享 slot：状态变化优先，无变化则交替兜底 */
            static uint8_t last_chassis = 0xFF;     // 0xFF=强制首次发送
            static uint8_t last_leg     = 0xFF;
            static uint8_t alt          = 0;        // 交替索引（0=NewText, 1=NewText2）
            uint8_t cur_chassis = Foot_Chassis.Chassis_Mode;
            uint8_t cur_leg     = Foot_Chassis.Target_Leg_State;
            uint8_t chassis_pending = (cur_chassis != last_chassis);
            uint8_t leg_pending     = (cur_leg     != last_leg);

            if (chassis_pending && leg_pending) {
                /* 两条都待发：按 alt 顺序送一条，另一条等下一个 slot 9（1s 后） */
                if (alt) { _ui_update_g_5HZ_0(); last_chassis = cur_chassis; }
                else     { _ui_update_g_5HZ_1(); last_leg     = cur_leg;     }
                alt ^= 1;
            } else if (chassis_pending) {
                _ui_update_g_5HZ_0(); last_chassis = cur_chassis;
            } else if (leg_pending) {
                _ui_update_g_5HZ_1(); last_leg     = cur_leg;
            } else {
                /* 状态稳定 → 交替重发兜底丢包，每条 0.5Hz */
                if (alt) _ui_update_g_5HZ_0();
                else     _ui_update_g_5HZ_1();
                alt ^= 1;
            }
            break;
        }
        default: break;
        }

        /* INIT 静态层兜底重发：10Hz tick × 30 = 每 3 秒重发一次。
           cnt != 0 是为了避开启动时已经发过的初始化帧，不重复。
           这一次会同时发 4 个 INIT 子帧（1 个 5 图元帧 + 3 个字符串帧），
           短时间内占用裁判系统带宽，但因为周期长，平均占用很低。 */
        if ((cnt % 30) == 0 && cnt != 0) {
            UI_RefreshParams_INIT();
            ui_update_g_INIT();
        }

        cnt++;
        osDelay(100);   // 10Hz tick (100ms 一拍)
    }
}
