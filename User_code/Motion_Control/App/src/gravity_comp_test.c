/**
 * @file gravity_comp_test.c
 * @brief 重力补偿标定扫描。全文用【测腿长范围】/【测数据】/【公共】三种标签标明每段代码归属。
 *
 * ┌─ 总体流程（由 grav_phase 状态机驱动，本函数每 2ms 被调用一次）─────────────────────────┐
 * │ 阶段一【测腿长范围】 grav_phase = GRAV_PROBE_MIN → GRAV_PROBE_MAX                      │
 * │   先收腿到最短(压到机械限位卡住)，再伸腿到最长(卡住)，左右腿分别记录实测的 L0 最短/最长。 │
 * │   该阶段只沿腿轴出力(F)、不输出 T 转矩；目的是测出这台车真实的腿长可用范围。            │
 * │ 阶段二【测数据】     grav_phase = GRAV_RAMPING → GRAV_SETTLING → GRAV_DONE             │
 * │   用阶段一的实测量程把每条腿的腿长等分 10 组，配合 18 个方向(整圈等分、绕竖直对称)，    │
 * │   用位/速双环 PID 把左右两腿依次锁到各指定位姿；每到一姿态、稳定且无抖动后记录该姿态下  │
 * │   VMC 解算出的 F、T，自动推进，直至测完 18×10×2×2 = 720 个补偿数据。                   │
 * └──────────────────────────────────────────────────────────────────────────────────────┘
 *
 * 【测数据】控制律照搬 sit_motion.c 的锁位姿写法(含右腿符号镜像)，区别是标定时不加重力前馈，
 * 让 PID 自己撑出全部保持力。两阶段都用本文件独立的一套腿部 PID（数值复制自生产 PID、改名隔离，
 * 调参只动本文件、不影响其它模式）。调参/标定用，非常规控制路径。
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

/*====================================== 参数 ===============================================*/

/* ———————————————— 【测数据】阶段参数 ———————————————— */
#define GRAV_TEST_ANGLE_NUM      18           // 方向数：整圈等分 18 份
#define GRAV_TEST_LENGTH_NUM     10           // 腿长数：实测量程等分 10 组
#define GRAV_TEST_FULL_CIRCLE    (2.0f * PI)  // “一圈”大小，可改成任意 pi
#define GRAV_TEST_ANGLE_CENTER   (- PI / 2.0f)  // 方向以竖直 PI/2 为中心对称展开
#define GRAV_TEST_RAMP_TIME      0.8f         // 位姿过渡斜坡时间(s)
#define GRAV_TEST_CYCLE_TIME     0.002f       // 控制周期(s)，本函数 2ms 调用一次（两阶段共用）
#define GRAV_TEST_L0_TOL         0.003f       // 腿长稳态误差阈值(m)
#define GRAV_TEST_ANG_TOL        0.02f        // 腿角稳态误差阈值(rad)
#define GRAV_TEST_DL0_TOL        0.01f        // 腿长速度阈值(m/s)，用于抗抖判定
#define GRAV_TEST_DPHI_TOL       0.05f        // 腿角速度阈值(rad/s)，用于抗抖判定
#define GRAV_TEST_SETTLE_CYCLES  500          // 连续稳定周期数(≈1s @500Hz)
#define GRAV_TEST_TIMEOUT_CYCLES 4000         // 单位姿超时周期数(≈8s)，防角度够不到卡死

/* ———————————————— GRAV_DONE 完成提示与遍历 ———————————————— */
#define GRAV_DONE_BUZZ_FREQ          880       // 完成提示音高(Hz)，la/A5
#define GRAV_DONE_BUZZ_TIME          5.0f      // 长鸣持续时间(s)
#define GRAV_TRAVERSE_STEP_INTERVAL  100       // 遍历每步间隔 tick 数(100×2ms=200ms/步)

/* ———————————————— 【测腿长范围】阶段参数 ———————————————— */
#define GRAV_PROBE_PUSH_FORCE_RETRACT   30.0f // 收腿时沿腿轴的恒定推力(N)：需要用大力才能把腿压到最短机械限位
#define GRAV_PROBE_PUSH_FORCE_EXTEND    30.0f  // 伸腿时沿腿轴的恒定推力(N)：腿自重+氮气弹簧已提供大部分伸力，只需轻推
                                              // 不经斜坡/位置环直给力，否则力被喂得过小压不到底。压不到底就调大对应值
#define GRAV_PROBE_SETTLE_CYCLES 100          // 加力后先等几拍让腿动起来再判卡住(≈0.2s)，避开起步瞬间速度为0的误判
#define GRAV_PROBE_VEL_TOL       0.01f        // 卡住判定：腿长速度阈值(m/s)
#define GRAV_PROBE_STUCK_CYCLES  500          // 卡住持续周期数(≈1.0s)
#define GRAV_PROBE_TIMEOUT_CYCLES 3000        // 单次探测超时(≈6s)，超时按当前 L0 锁存
#define GRAV_PROBE_MARGIN        0.01f        // 量程两端内缩(m)：略离开机械硬限位，避免端点采样被限位反力污染。
                                              // 取很小值以尽量覆盖全量程(拟合时近乎无外推)；置 0 则采到实测极值本身

/*====================================== 结果（调试器读取）==================================*/

/* ———————————————— 【测数据】结果 ———————————————— */
// 索引：[方向][腿长]。F/T 取 VMC 解算值（VMC.F 已减氮气弹簧力并低通）。
float grav_comp_F_left[GRAV_TEST_ANGLE_NUM][GRAV_TEST_LENGTH_NUM];
float grav_comp_T_left[GRAV_TEST_ANGLE_NUM][GRAV_TEST_LENGTH_NUM];
float grav_comp_F_right[GRAV_TEST_ANGLE_NUM][GRAV_TEST_LENGTH_NUM];
float grav_comp_T_right[GRAV_TEST_ANGLE_NUM][GRAV_TEST_LENGTH_NUM];
// 数据质量标记：1=稳定收敛后记录，0=超时强制记录（该姿态够不到/没稳住）
uint8_t grav_comp_settled[GRAV_TEST_ANGLE_NUM][GRAV_TEST_LENGTH_NUM];

/* ———————————————— 【测腿长范围】结果 ———————————————— */
// 左右腿各自实测的 L0 最短/最长（阶段二据此生成腿长目标）
float grav_L0_min_left  = 0.0f, grav_L0_max_left  = 0.0f;
float grav_L0_min_right = 0.0f, grav_L0_max_right = 0.0f;

/*====================================== 运行状态（调试器观察/控制）=========================*/

/* ———————————————— 【公共】总控 ———————————————— */
uint8_t  gravity_comp_test_done    = 0;  // 全流程测完置 1
uint8_t  gravity_comp_test_restart = 0;  // 置 1 重新开始整个流程（含【测腿长范围】）

/* ———————————————— 【测数据】进度 ———————————————— */
uint16_t grav_comp_angle_idx      = 0;   // 当前方向索引 0..17
uint16_t grav_comp_len_idx        = 0;   // 当前腿长索引 0..9
uint16_t grav_comp_settle_counter = 0;   // 连续稳定计数
uint16_t grav_comp_pose_cycle     = 0;   // 当前位姿已用周期数（超时用）

/* ———————————————— 【测腿长范围】中间状态 ———————————————— */
uint8_t  grav_probe_stuck_L = 0, grav_probe_stuck_R = 0;        // 本次探测该腿是否已卡住锁存
uint16_t grav_probe_stuck_cnt_L = 0, grav_probe_stuck_cnt_R = 0; // 连续卡住计数
uint16_t grav_probe_cycle = 0;                                  // 本次探测已用周期数

/* ———————————————— GRAV_DONE 后遍历 ———————————————— */
// 标定结束后遍历结果数组的专用步进变量（volatile 全局，调试器可直接 watch 每一步的值）
uint32_t grav_traverse_step;       // 0..4×18×10=720，依次遍历 4 个结果数组的全部元素
float    grav_traverse_val;        // 当前步对应的数组元素值，方便调试器逐元素观察
uint32_t grav_traverse_countdown;  // 遍历步间倒计数 tick，到 0 推进一步
uint8_t  grav_traverse_phase;      // 0=蜂鸣器长鸣 5 秒, 1=遍历结果数组

/*====================================== 目标表与斜坡 ======================================*/

static float grav_angle_target[GRAV_TEST_ANGLE_NUM];      // 【测数据】18 个 phi0 目标(rad)
static float grav_L0_target_left[GRAV_TEST_LENGTH_NUM];   // 【测数据】左腿 10 个 L0 目标(m)，由实测量程生成
static float grav_L0_target_right[GRAV_TEST_LENGTH_NUM];  // 【测数据】右腿 10 个 L0 目标(m)，由实测量程生成

static RampGenerator grav_L0_ramp_L,   grav_L0_ramp_R;    // 【测数据】腿长斜坡（测腿长范围阶段改用恒力直推，不用斜坡）
static RampGenerator grav_phi0_ramp_L, grav_phi0_ramp_R;  // 【测数据】腿角斜坡（测腿长范围阶段不用）

static float grav_probe_force = 0.0f;  // 【测腿长范围】本次探测沿腿轴的恒定推力(N)：负值由 PUSH_FORCE_RETRACT(200N) 取反、收向最短；正值为 PUSH_FORCE_EXTEND(10N)、伸向最长

// 状态机阶段：前两个属【测腿长范围】，中间两个属【测数据】
typedef enum
{
    GRAV_PROBE_MIN = 0, // 【测腿长范围】收腿到最短测 L0 下限
    GRAV_PROBE_MAX,     // 【测腿长范围】伸腿到最长测 L0 上限
    GRAV_RAMPING,       // 【测数据】斜坡到位姿
    GRAV_SETTLING,      // 【测数据】等稳定后记录
    GRAV_DONE           // 全部测完
} Grav_Test_Phase;
static Grav_Test_Phase grav_phase = GRAV_PROBE_MIN; // 标定状态机当前阶段（两阶段共 5 状态）

/*=========================== 本文件独立的腿部 PID =========================================*/
/* 数值从生产 PID（chassis_init.c 的 L/R_Leg_*）原样复制而来，仅加 grav_ 前缀改名隔离。
 * 标定全程只用这一套；★调 PID 参数只改下面 Gravity_Init_Leg_PIDs() 里的数值★，不影响其它模式。 */
user_pid_t grav_L_Leg_L0_POS_PID, grav_R_Leg_L0_POS_PID;  // 腿长位置环(外环)
user_pid_t grav_L_Leg_L0_SPD_PID, grav_R_Leg_L0_SPD_PID;  // 腿长速度环(内环)，输出即 F
user_pid_t grav_L_Leg_Middle_PID, grav_R_Leg_Middle_PID;  // 腿角角度环(外环)
user_pid_t grav_L_Leg_dphi0_PID,  grav_R_Leg_dphi0_PID;   // 腿角角速度环(内环)，输出即 T

/*====================================== 内部函数 ==========================================*/

// 【公共】★调参在这★：初始化本文件 8 个腿部 PID，数值与 chassis_init.c 的生产腿 PID 完全一致。
// 参数顺序 = (PID, Kp, Ki, Kd, out_limit, i_limit, I_step, Integraldead_zone, deadzone)
static void Gravity_Init_Leg_PIDs(void)
{
    PID_INIT(&grav_L_Leg_L0_POS_PID, 10, 0.0, 1, 3.0, 2.0, 0, 200, 0);
    PID_INIT(&grav_R_Leg_L0_POS_PID, 10, 0.0, 1, 3.0, 2.0, 0, 200, 0);
    PID_INIT(&grav_L_Leg_L0_SPD_PID, 5, 1, 40, 100, 80, 0, 2000, 0);
    PID_INIT(&grav_R_Leg_L0_SPD_PID, 5, 1, 40, 100, 80, 0, 2000, 0);
    PID_INIT(&grav_L_Leg_Middle_PID, 5, 0, 0.1, 5.0, 4.0, 0, 0, 0);
    PID_INIT(&grav_R_Leg_Middle_PID, 5, 0, 0.1, 5.0, 4.0, 0, 0, 0);
    PID_INIT(&grav_L_Leg_dphi0_PID, 3, 0.1, 1, 150, 150, 0, 2000, 0);
    PID_INIT(&grav_R_Leg_dphi0_PID, 3, 0.1, 1, 150, 150, 0, 2000, 0);
}

// 【公共】清 8 个本文件腿 PID 的积分(只清 I、保留增益)，避免跨阶段残留
static void Gravity_Clear_Leg_PIDs(void)
{
    PID_Clear(&grav_L_Leg_L0_POS_PID);
    PID_Clear(&grav_L_Leg_L0_SPD_PID);
    PID_Clear(&grav_R_Leg_L0_POS_PID);
    PID_Clear(&grav_R_Leg_L0_SPD_PID);
    PID_Clear(&grav_L_Leg_Middle_PID);
    PID_Clear(&grav_L_Leg_dphi0_PID);
    PID_Clear(&grav_R_Leg_Middle_PID);
    PID_Clear(&grav_R_Leg_dphi0_PID);
}

// 【测腿长范围】启动一次探测：设定沿腿轴的恒定推力方向(force)，复位卡住状态/计数。
// 直接给恒力把腿顶到机械限位（不经斜坡/位置环，否则命令力被喂得太小、压不到底——就是“L0方向没力”的原因）。
static void Gravity_Probe_Setup(float force)
{
    grav_probe_force = force;
    grav_probe_stuck_L = 0;
    grav_probe_stuck_R = 0;
    grav_probe_stuck_cnt_L = 0;
    grav_probe_stuck_cnt_R = 0;
    grav_probe_cycle = 0;
}

// 【测腿长范围 → 测数据 衔接】用实测量程为左右腿各建 10 个腿长目标。
// 两端各内缩 MARGIN，避开机械硬限位(端点采样会被限位反力污染、且极限位姿附近力解算变差)。
static void Gravity_Build_L0_Targets(void)
{
    float lo_L = grav_L0_min_left  + GRAV_PROBE_MARGIN; // 左腿可用量程下限（内缩后）
    float hi_L = grav_L0_max_left  - GRAV_PROBE_MARGIN; // 左腿可用量程上限（内缩后）
    float lo_R = grav_L0_min_right + GRAV_PROBE_MARGIN; // 右腿可用量程下限（内缩后）
    float hi_R = grav_L0_max_right - GRAV_PROBE_MARGIN; // 右腿可用量程上限（内缩后）
    for (uint16_t j = 0; j < GRAV_TEST_LENGTH_NUM; j++) // j: 腿长索引 0..9
    {
        float f = (float)j / (GRAV_TEST_LENGTH_NUM - 1); // 归一化比例 f: 0..1（含两端）
        grav_L0_target_left[j]  = lo_L + (hi_L - lo_L) * f;
        grav_L0_target_right[j] = lo_R + (hi_R - lo_R) * f;
    }
}

// 【测数据】为当前 (grav_comp_angle_idx, grav_comp_len_idx) 位姿初始化 4 条斜坡，从当前实际位姿平滑过渡；
// 置阶段为 GRAV_RAMPING。
static void Gravity_Test_Setup_Pose(void)
{
    float L0_t_L = grav_L0_target_left[grav_comp_len_idx];  // 左腿目标腿长
    float L0_t_R = grav_L0_target_right[grav_comp_len_idx]; // 右腿目标腿长（量程可能不同）
    float phi0_t = grav_angle_target[grav_comp_angle_idx];  // 两腿共用的目标腿角

    rampInit(&grav_L0_ramp_L,   VMC_L.L0,   L0_t_L, GRAV_TEST_RAMP_TIME, GRAV_TEST_CYCLE_TIME);
    rampInit(&grav_L0_ramp_R,   VMC_R.L0,   L0_t_R, GRAV_TEST_RAMP_TIME, GRAV_TEST_CYCLE_TIME);
    rampInit(&grav_phi0_ramp_L, VMC_L.phi0, phi0_t, GRAV_TEST_RAMP_TIME, GRAV_TEST_CYCLE_TIME);
    rampInit(&grav_phi0_ramp_R, VMC_R.phi0, phi0_t, GRAV_TEST_RAMP_TIME, GRAV_TEST_CYCLE_TIME);

    grav_comp_settle_counter = 0;
    grav_comp_pose_cycle     = 0;
    grav_phase               = GRAV_RAMPING;
}

// 【公共/总入口】初始化或重启整个流程：建方向表、清结果、清 PID，并从【测腿长范围】(先收最短)开始。
static void Gravity_Test_Init(void)
{
    // 方向表：整圈等分、绕 PI/2 对称展开（不依赖实测量程，先建好；【测数据】用）
    for (uint16_t k = 0; k < GRAV_TEST_ANGLE_NUM; k++)
    {
        // 等间隔19个方向: 以PI/2(竖直)为中心, k=0..17对称分布在360°整圈上
        // k=0 对应 CENTER - 8.5°步, k=17 对应 CENTER + 8.5°步, 共17个步长覆盖整圈
        grav_angle_target[k] = GRAV_TEST_ANGLE_CENTER
            + ((float)k - (GRAV_TEST_ANGLE_NUM - 1) * 0.5f)
              * (GRAV_TEST_FULL_CIRCLE / GRAV_TEST_ANGLE_NUM);
    }

    // 清【测数据】结果与质量标记
    for (uint16_t k = 0; k < GRAV_TEST_ANGLE_NUM; k++)
    {
        for (uint16_t j = 0; j < GRAV_TEST_LENGTH_NUM; j++)
        {
            grav_comp_F_left[k][j]  = 0.0f;
            grav_comp_T_left[k][j]  = 0.0f;
            grav_comp_F_right[k][j] = 0.0f;
            grav_comp_T_right[k][j] = 0.0f;
            grav_comp_settled[k][j] = 0;
        }
    }

    // 清【测腿长范围】结果
    grav_L0_min_left  = 0.0f; grav_L0_max_left  = 0.0f;
    grav_L0_min_right = 0.0f; grav_L0_max_right = 0.0f;

    Gravity_Init_Leg_PIDs(); // 初始化本文件 PID（设增益 + 清状态）

    grav_comp_angle_idx    = 0;
    grav_comp_len_idx      = 0;
    gravity_comp_test_done = 0;

    // 从【测腿长范围】第一步——收腿到最短——开始（负推力把腿往最短方向顶）
    grav_phase = GRAV_PROBE_MIN;
    Gravity_Probe_Setup(-GRAV_PROBE_PUSH_FORCE_RETRACT); // 收腿：负推力 200N 把腿往最短方向压
}

// 【测腿长范围】卡住检测与阶段推进（仅在 GRAV_PROBE_MIN / GRAV_PROBE_MAX 阶段、每周期调用）。
// 判定逻辑：恒力已施加一小段时间(让腿先动起来) 且 该腿腿长速度持续接近 0 → 顶到极限卡住，锁存当前 L0。
static void Gravity_Probe_Update(void)
{
    uint8_t timeout = (grav_probe_cycle >= GRAV_PROBE_TIMEOUT_CYCLES); // 超时兜底，防永远检不到卡住
    // 加力起步阶段腿尚未动(速度≈0)会误判卡住，先等过 SETTLE_CYCLES 再开始判
    uint8_t armed = (grav_probe_cycle >= GRAV_PROBE_SETTLE_CYCLES);

    // 已加力一段时间 + 腿不再动 → 累计卡住计数；一旦动起来就清零（过滤途中减速的误判）
    if (armed && fabsf(VMC_L.d_L0) < GRAV_PROBE_VEL_TOL) grav_probe_stuck_cnt_L++;
    else                                                 grav_probe_stuck_cnt_L = 0;
    if (armed && fabsf(VMC_R.d_L0) < GRAV_PROBE_VEL_TOL) grav_probe_stuck_cnt_R++;
    else                                                 grav_probe_stuck_cnt_R = 0;

    // 该腿首次确认卡住(或超时)时，锁存其当前 L0 为本阶段的极限值
    if (!grav_probe_stuck_L && (grav_probe_stuck_cnt_L >= GRAV_PROBE_STUCK_CYCLES || timeout))
    {
        grav_probe_stuck_L = 1;
        if (grav_phase == GRAV_PROBE_MIN) grav_L0_min_left = VMC_L.L0;
        else                              grav_L0_max_left = VMC_L.L0;
    }
    if (!grav_probe_stuck_R && (grav_probe_stuck_cnt_R >= GRAV_PROBE_STUCK_CYCLES || timeout))
    {
        grav_probe_stuck_R = 1;
        if (grav_phase == GRAV_PROBE_MIN) grav_L0_min_right = VMC_R.L0;
        else                              grav_L0_max_right = VMC_R.L0;
    }

    if (!(grav_probe_stuck_L && grav_probe_stuck_R))
    {
        return; // 等两腿都卡住再推进
    }

    if (grav_phase == GRAV_PROBE_MIN)
    {
        // 收到最短完成 → 转去伸最长（正推力）
        Gravity_Clear_Leg_PIDs(); // 清掉压最短时的积分窝囊
        grav_phase = GRAV_PROBE_MAX;
        Gravity_Probe_Setup(+GRAV_PROBE_PUSH_FORCE_EXTEND); // 伸腿：正推力 10N 把腿往最长方向轻推
    }
    else
    {
        // 伸到最长完成 → 量程测全了：建【测数据】腿长目标，进入数据采集首位姿
        Gravity_Build_L0_Targets();
        Gravity_Clear_Leg_PIDs();
        grav_comp_angle_idx = 0;
        grav_comp_len_idx   = 0;
        Gravity_Test_Setup_Pose(); // 内部置 grav_phase = GRAV_RAMPING
    }
}

/*====================================== 标定主函数 ========================================*/

void Gravity_Compensation_Test_Function(void)
{
    static uint8_t inited = 0; // 首次调用标志：0=待初始化，1=已初始化，避免重复建方向表/清结果

    /* —————————————— 【公共】每周期前置 —————————————— */
    VMC_Coculate(); // 更新 L0/phi0/d_* 等，供斜坡起点与稳定/卡住判据使用

    if (!inited || gravity_comp_test_restart) // 首次进入、或被置 restart → 从头(含测腿长范围)开始
    {
        Gravity_Test_Init();
        inited = 1;
        gravity_comp_test_restart = 0;
    }

    // 推进 4 条斜坡（phi0 斜坡在【测腿长范围】阶段尚未 rampInit，isBusy=0，rampIterate 为空操作）
    rampIterate(&grav_L0_ramp_L);
    rampIterate(&grav_L0_ramp_R);
    rampIterate(&grav_phi0_ramp_L);
    rampIterate(&grav_phi0_ramp_R);

    /* ════════════════════════════════════════════════════════════════════════════════════
     * 【测腿长范围】阶段：沿腿轴直接给恒定推力(grav_probe_force)把腿顶到机械限位；T=0(不输出转矩)。
     *   不走位/速双环——位置环跟斜坡只会喂出很小的力、压不到底（这正是之前“L0方向没力”的原因）。
     * ════════════════════════════════════════════════════════════════════════════════════ */
    if (grav_phase == GRAV_PROBE_MIN || grav_phase == GRAV_PROBE_MAX)
    {
        grav_probe_cycle++;

        // 直接命令恒定足端力(沿腿轴)，方向由 grav_probe_force 符号决定；不输出 T 转矩
        VMC_Chassis_Target.L_F0 = grav_probe_force;
        VMC_Chassis_Target.L_T = 0.0f;
        VMC_Chassis_Target.R_F0 = grav_probe_force;
        VMC_Chassis_Target.R_T = 0.0f;

        // 轮子不主动驱动
        VMC_Chassis_Target.L_Wheel_Torque = 0.0f;
        VMC_Chassis_Target.R_Wheel_Torque = 0.0f;

        Gravity_Probe_Update(); // 卡住检测 + 阶段推进（两腿都卡住后转下一步 / 进入测数据）
        return;
    }

    /* ════════════════════════════════════════════════════════════════════════════════════
     * 【测数据】阶段：位/速双环把两腿锁到目标位姿，稳定无抖后记录 VMC 的 F/T
     *   控制律同 sit_motion（右腿符号镜像），但标定时不加重力前馈——让 PID 撑出全部保持力(即要测的量)
     * ════════════════════════════════════════════════════════════════════════════════════ */
    // 左腿腿长双环
    PID_Set_Error(&grav_L_Leg_L0_POS_PID, VMC_L.L0, grav_L0_ramp_L.currentValue);
    PID_coculate(&grav_L_Leg_L0_POS_PID);
    PID_Set_Error(&grav_L_Leg_L0_SPD_PID, VMC_L.d_L0, grav_L_Leg_L0_POS_PID.output);
    PID_coculate(&grav_L_Leg_L0_SPD_PID);

    // 右腿腿长双环
    PID_Set_Error(&grav_R_Leg_L0_POS_PID, VMC_R.L0, grav_L0_ramp_R.currentValue);
    PID_coculate(&grav_R_Leg_L0_POS_PID);
    PID_Set_Error(&grav_R_Leg_L0_SPD_PID, VMC_R.d_L0, grav_R_Leg_L0_POS_PID.output);
    PID_coculate(&grav_R_Leg_L0_SPD_PID);

    // 左腿腿角双环（角度环走最短角）
    PID_Set_AngleError(&grav_L_Leg_Middle_PID, VMC_L.phi0, grav_phi0_ramp_L.currentValue);
    PID_coculate(&grav_L_Leg_Middle_PID);
    PID_Set_Error(&grav_L_Leg_dphi0_PID, VMC_L.d_phi0, grav_L_Leg_Middle_PID.output);
    PID_coculate(&grav_L_Leg_dphi0_PID);

    // 右腿腿角双环（符号镜像，与 sit_motion 一致）
    PID_Set_AngleError(&grav_R_Leg_Middle_PID, VMC_R.phi0, grav_phi0_ramp_R.currentValue);
    PID_coculate(&grav_R_Leg_Middle_PID);
    PID_Set_Error(&grav_R_Leg_dphi0_PID, -VMC_R.d_phi0, -grav_R_Leg_Middle_PID.output);
    PID_coculate(&grav_R_Leg_dphi0_PID);

    // VMC 映射到电机力矩：不加前馈，让 PID 撑出全部保持力（这正是要标定记录的量）
    VMC_Chassis_Target.L_F0 = grav_L_Leg_L0_SPD_PID.output;
    VMC_Chassis_Target.L_T = grav_L_Leg_dphi0_PID.output;
    VMC_Chassis_Target.R_F0 = grav_R_Leg_L0_SPD_PID.output;
    VMC_Chassis_Target.R_T = -grav_R_Leg_dphi0_PID.output;

    // 轮子不主动驱动（静态测试，置 0 防沿用上一个模式的旧力矩）
    VMC_Chassis_Target.L_Wheel_Torque = 0.0f;
    VMC_Chassis_Target.R_Wheel_Torque = 0.0f;

    if (grav_phase == GRAV_DONE)
    {
        // — GRAV_DONE 后处理：子阶段 0=蜂鸣器长鸣 5s, 子阶段 1=遍历结果数组 —
        // PID 继续跑锁住最后位姿，不 return；轮子输出已在上方置 0
        if (grav_traverse_phase == 0)
        {
            // 子阶段 0：蜂鸣器长鸣 5 秒
            Buzzer_Tone_Max(GRAV_DONE_BUZZ_FREQ); // 每 tick 刷新，保持鸣响
            grav_traverse_countdown--;
            if (grav_traverse_countdown == 0)
            {
                Stop_Buzzer();
                grav_traverse_phase     = 1;   // 进入遍历阶段
                grav_traverse_step      = 0;
                grav_traverse_countdown = GRAV_TRAVERSE_STEP_INTERVAL; // 立即推第一步
            }
        }
        else // grav_traverse_phase == 1
        {
            // 子阶段 1：遍历 4 个结果数组，每一步给 grav_traverse_val 赋对应元素值
            if (grav_traverse_countdown == 0)
            {
                // 取当前步对应的数组元素值赋给 grav_traverse_val（调试器可直接 watch）
                if (grav_traverse_step < GRAV_TEST_ANGLE_NUM * GRAV_TEST_LENGTH_NUM)
                    grav_traverse_val = grav_comp_F_left
                        [grav_traverse_step / GRAV_TEST_LENGTH_NUM]
                        [grav_traverse_step % GRAV_TEST_LENGTH_NUM];
                else if (grav_traverse_step < 2 * GRAV_TEST_ANGLE_NUM * GRAV_TEST_LENGTH_NUM)
                    grav_traverse_val = grav_comp_T_left
                        [(grav_traverse_step - GRAV_TEST_ANGLE_NUM * GRAV_TEST_LENGTH_NUM) / GRAV_TEST_LENGTH_NUM]
                        [(grav_traverse_step - GRAV_TEST_ANGLE_NUM * GRAV_TEST_LENGTH_NUM) % GRAV_TEST_LENGTH_NUM];
                else if (grav_traverse_step < 3 * GRAV_TEST_ANGLE_NUM * GRAV_TEST_LENGTH_NUM)
                    grav_traverse_val = grav_comp_F_right
                        [(grav_traverse_step - 2 * GRAV_TEST_ANGLE_NUM * GRAV_TEST_LENGTH_NUM) / GRAV_TEST_LENGTH_NUM]
                        [(grav_traverse_step - 2 * GRAV_TEST_ANGLE_NUM * GRAV_TEST_LENGTH_NUM) % GRAV_TEST_LENGTH_NUM];
                else if (grav_traverse_step < 4 * GRAV_TEST_ANGLE_NUM * GRAV_TEST_LENGTH_NUM)
                    grav_traverse_val = grav_comp_T_right
                        [(grav_traverse_step - 3 * GRAV_TEST_ANGLE_NUM * GRAV_TEST_LENGTH_NUM) / GRAV_TEST_LENGTH_NUM]
                        [(grav_traverse_step - 3 * GRAV_TEST_ANGLE_NUM * GRAV_TEST_LENGTH_NUM) % GRAV_TEST_LENGTH_NUM];
                else
                    grav_traverse_val = 0.0f; // 遍历完毕：归零

                grav_traverse_step++;
                grav_traverse_countdown = GRAV_TRAVERSE_STEP_INTERVAL;
            }
            else
            {
                grav_traverse_countdown--;
            }

            if (grav_traverse_step >= 4 * GRAV_TEST_ANGLE_NUM * GRAV_TEST_LENGTH_NUM)
            {
                grav_traverse_phase = 2; // 遍历完成，停在此处（PID 继续锁位姿）
            }
        }
        // 遍历期间 PID 继续跑（上方目标量照常更新），轮子输出已在上方置 0
    }

    grav_comp_pose_cycle++;

    // 当前位姿的目标（左右腿腿长可能不同，腿角相同）
    float L0_t_L = grav_L0_target_left[grav_comp_len_idx];
    float L0_t_R = grav_L0_target_right[grav_comp_len_idx];
    float phi0_t = grav_angle_target[grav_comp_angle_idx];

    // 【测数据】阶段一：等 4 条斜坡都到位，再转入稳定判定
    if (grav_phase == GRAV_RAMPING)
    {
        uint8_t ramp_done = // 4 条斜坡是否全部到位（当前值 ≈ 目标值，误差 < 1e-4）
               (fabsf(grav_L0_ramp_L.currentValue   - L0_t_L) < 1e-4f)
            && (fabsf(grav_L0_ramp_R.currentValue   - L0_t_R) < 1e-4f)
            && (fabsf(grav_phi0_ramp_L.currentValue - phi0_t) < 1e-4f)
            && (fabsf(grav_phi0_ramp_R.currentValue - phi0_t) < 1e-4f);
        if (ramp_done)
        {
            grav_phase = GRAV_SETTLING;
        }
    }

    // 【测数据】阶段二：等稳定且无抖动(误差+速度都达标)，连续够久就记录并推进；超时则强填
    if (grav_phase == GRAV_SETTLING)
    {
        uint8_t stable = // 该位姿是否已稳定（位置误差 + 速度均低于阈值，无抖动）
               (fabsf(VMC_L.L0 - L0_t_L) < GRAV_TEST_L0_TOL)                       // 左腿长到位
            && (fabsf(VMC_R.L0 - L0_t_R) < GRAV_TEST_L0_TOL)                       // 右腿长到位
            && (fabsf(ShortestAngleDelta(phi0_t, VMC_L.phi0)) < GRAV_TEST_ANG_TOL) // 左腿角到位
            && (fabsf(ShortestAngleDelta(phi0_t, VMC_R.phi0)) < GRAV_TEST_ANG_TOL) // 右腿角到位
            && (fabsf(VMC_L.d_L0)   < GRAV_TEST_DL0_TOL)                           // 左腿长不抖
            && (fabsf(VMC_R.d_L0)   < GRAV_TEST_DL0_TOL)                           // 右腿长不抖
            && (fabsf(VMC_L.d_phi0) < GRAV_TEST_DPHI_TOL)                          // 左腿角不抖
            && (fabsf(VMC_R.d_phi0) < GRAV_TEST_DPHI_TOL);                         // 右腿角不抖

        if (stable)
        {
            grav_comp_settle_counter++;
        }
        else
        {
            grav_comp_settle_counter = 0;
        }

        uint8_t settled_ok = (grav_comp_settle_counter >= GRAV_TEST_SETTLE_CYCLES); // 稳定够久
        uint8_t timeout    = (grav_comp_pose_cycle >= GRAV_TEST_TIMEOUT_CYCLES);    // 角度够不到时兜底

        if (settled_ok || timeout)
        {
            // 记录该位姿下 VMC 解出的 F、T（左右分开存）
            grav_comp_F_left[grav_comp_angle_idx][grav_comp_len_idx]  = VMC_L.F;
            grav_comp_T_left[grav_comp_angle_idx][grav_comp_len_idx]  = VMC_L.T;
            grav_comp_F_right[grav_comp_angle_idx][grav_comp_len_idx] = VMC_R.F;
            grav_comp_T_right[grav_comp_angle_idx][grav_comp_len_idx] = VMC_R.T;
            grav_comp_settled[grav_comp_angle_idx][grav_comp_len_idx] = settled_ok ? 1 : 0;

            // 推进到下一个位姿：腿长内层、方向外层
            grav_comp_len_idx++;
            if (grav_comp_len_idx >= GRAV_TEST_LENGTH_NUM)
            {
                grav_comp_len_idx = 0;
                grav_comp_angle_idx++;
            }

            if (grav_comp_angle_idx >= GRAV_TEST_ANGLE_NUM)
            {
                gravity_comp_test_done = 1; // 720 个全测完
                grav_phase = GRAV_DONE;

                // 初始化 GRAV_DONE 后遍历：先蜂鸣器长鸣 5 秒，再遍历结果数组
                grav_traverse_phase     = 0;   // 子阶段 0：蜂鸣器长鸣
                grav_traverse_step      = 0;
                grav_traverse_countdown = (uint32_t)(GRAV_DONE_BUZZ_TIME / GRAV_TEST_CYCLE_TIME); // 5s / 2ms = 2500 tick
                Buzzer_Tone_Max(GRAV_DONE_BUZZ_FREQ);
            }
            else
            {
                Gravity_Test_Setup_Pose(); // 起下一个位姿的斜坡，回到 GRAV_RAMPING
            }
        }
    }
}
