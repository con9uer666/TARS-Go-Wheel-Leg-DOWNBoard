//
// Created by RM UI Designer
// Static Edition
//
// ============================================================================
//  ui_g.c —— UI 模块"图层层"实现（RM UI Designer 自动生成）
//  ----------------------------------------------------------------------------
//  约定：
//    · 每个分组拆成若干"子帧"(ui_g_<group>_<index>)，每子帧载 1/5/7 个图元
//      或 1 个字符串。子帧选什么 ui_X_frame_t 取决于图元数。
//    · 每个图元同时有两个"身份"：在 data[] 里的 ui_interface_figure_t 通用
//      占位，以及上层用具体子类型指针 (round/line/...) 直接访问。指针都
//      指向同一块内存，改字段=改下次发送内容。
//    · 三类操作：
//        _ui_init_*   设 operate_type=1（新增图元 + 初始化字段 + 立即发一次）
//        _ui_update_* 设 operate_type=2（修改图元，data 已被上层改过 → 发出去）
//        _ui_remove_* 设 operate_type=3（删除图元）
//    · figure_name[3] 是图元唯一 ID，全局每个图元必须不同（同图层下重名会被
//      客户端覆盖）。这里用 figure_name = (0, 分组号, 序号) 三字节布局。
//
//  分组划分（详见 ui_g.h）：
//    30HZ_0 / 30HZ_1 / 30HZ_2 (各 7 图元)、30HZ_3 (5 图元)：动态主屏
//    5HZ_0  / 5HZ_1   (各 1 字符串)：低频文字提示
//    INIT_0 (5 图元) / INIT_1~3 (各 1 字符串)：永久静态层
// ============================================================================

#include <string.h>

#include "ui_interface.h"
#include "cmsis_os.h"   // 引入 osDelay：用于在聚合入口里按帧节流。
                        // UI_FRAME_GAP_MS：相邻子帧之间的最小间隔（ms）。
                        // 协议帧最长 116B @115200 ≈ 10.1ms 物理传输 + HAL gState 复位 ≈ 1ms。
                        // 30ms 给一倍以上余量，覆盖 FreeRTOS osDelay 的 tick 对齐抖动。
                        // 实测：osDelay(15) 时 30HZ_0 / 30HZ_1 / INIT_1 这种"前面有较长帧"
                        // 的下一帧仍会被客户端拒收（init 阶段一次性，update 永久救不回来）。
#define UI_FRAME_GAP_MS 30

// =====================================================================
//  分组 30HZ 子帧 0：7 个图元
//  figure_name 第二字节=0（30HZ 分组前缀），第三字节=0..6
// =====================================================================
ui_7_frame_t ui_g_30HZ_0;   // 实际发送缓冲区（含帧头/data/CRC16）

// 把 data[0..6] 重新解释为具体子类型指针，供 UI_Task.c 直接改字段。
// 注意每个指针指向的内存与 ui_g_30HZ_0.data[i] 同址。
ui_interface_round_t  *ui_g_30HZ_8009LF     = (ui_interface_round_t*) &(ui_g_30HZ_0.data[0]); // 左前 DM8009 心跳圆
ui_interface_round_t  *ui_g_30HZ_8009LB     = (ui_interface_round_t*) &(ui_g_30HZ_0.data[1]); // 左后 DM8009
ui_interface_round_t  *ui_g_30HZ_NUC        = (ui_interface_round_t*) &(ui_g_30HZ_0.data[2]); // NUC 心跳
ui_interface_number_t *ui_g_30HZ_FRIC_SPD_L = (ui_interface_number_t*)&(ui_g_30HZ_0.data[3]); // 左摩擦轮转速
ui_interface_number_t *ui_g_30HZ_AUTO_AIM   = (ui_interface_number_t*)&(ui_g_30HZ_0.data[4]); // 自瞄状态数字
ui_interface_number_t *ui_g_30HZ_SHOOT_NUM  = (ui_interface_number_t*)&(ui_g_30HZ_0.data[5]); // 已发射弹丸数
ui_interface_number_t *ui_g_30HZ_FRIC_SPD_R = (ui_interface_number_t*)&(ui_g_30HZ_0.data[6]); // 右摩擦轮转速

/**
 * @brief 30HZ 子帧 0 初始化（图元注册）
 *        - 给 7 个图元分配 figure_name = (0, 0, 0..6)
 *        - 设 operate_type=1（"新增"）
 *        - 填初始坐标/颜色/线宽等
 *        - 立即组帧 + DMA 发出去
 */
void _ui_init_g_30HZ_0() {
    // 给本子帧的 7 个图元统一打 ID + 新增标志
    for (int i = 0; i < 7; i++) {
        ui_g_30HZ_0.data[i].figure_name[0] = 0;
        ui_g_30HZ_0.data[i].figure_name[1] = 0;
        ui_g_30HZ_0.data[i].figure_name[2] = i + 0;   // 第三字节是子帧内序号
        ui_g_30HZ_0.data[i].operate_type = 1;          // 1=新增
    }
    // 兼容 Designer 模板：当本子帧没填满 7 槽时把剩余槽设为 0=空操作
    // (i 起点 7、终点 7 = 空循环，等价于"本子帧 7 槽满载"。)
    for (int i = 7; i < 7; i++) {
        ui_g_30HZ_0.data[i].operate_type = 0;
    }

    // ----- 8009LF：左前 DM8009 心跳圆（屏幕右侧竖排心跳灯第 1 个） -----
    ui_g_30HZ_8009LF->figure_type = 2;     // 2=圆
    ui_g_30HZ_8009LF->operate_type = 1;    // 新增
    ui_g_30HZ_8009LF->layer = 0;           // 图层 0
    ui_g_30HZ_8009LF->color = 4;           // 颜色 4 = 粉
    ui_g_30HZ_8009LF->start_x = 1900;      // 圆心 x
    ui_g_30HZ_8009LF->start_y = 900;       // 圆心 y（1080 屏）
    ui_g_30HZ_8009LF->width = 10;          // 在线=10 / 断联=1（运行时切换）
    ui_g_30HZ_8009LF->r = 5;               // 半径 5px

    // ----- 8009LB：左后 DM8009 心跳圆 -----
    ui_g_30HZ_8009LB->figure_type = 2;
    ui_g_30HZ_8009LB->operate_type = 1;
    ui_g_30HZ_8009LB->layer = 0;
    ui_g_30HZ_8009LB->color = 4;
    ui_g_30HZ_8009LB->start_x = 1900;
    ui_g_30HZ_8009LB->start_y = 876;
    ui_g_30HZ_8009LB->width = 10;
    ui_g_30HZ_8009LB->r = 5;

    // ----- NUC：上位机/视觉主机心跳圆（TODO：还没接驱动） -----
    ui_g_30HZ_NUC->figure_type = 2;
    ui_g_30HZ_NUC->operate_type = 1;
    ui_g_30HZ_NUC->layer = 0;
    ui_g_30HZ_NUC->color = 4;
    ui_g_30HZ_NUC->start_x = 1900;
    ui_g_30HZ_NUC->start_y = 660;
    ui_g_30HZ_NUC->width = 10;
    ui_g_30HZ_NUC->r = 5;

    // ----- FRIC_SPD_L：左摩擦轮转速数字（与右数字纵向并排） -----
    ui_g_30HZ_FRIC_SPD_L->figure_type = 6;     // 6=整数
    ui_g_30HZ_FRIC_SPD_L->operate_type = 1;
    ui_g_30HZ_FRIC_SPD_L->layer = 0;
    ui_g_30HZ_FRIC_SPD_L->color = 4;           // 粉
    ui_g_30HZ_FRIC_SPD_L->start_x = 300;
    ui_g_30HZ_FRIC_SPD_L->start_y = 870;
    ui_g_30HZ_FRIC_SPD_L->width = 2;           // 数字线宽（粗细）
    ui_g_30HZ_FRIC_SPD_L->font_size = 20;      // 字号 20
    ui_g_30HZ_FRIC_SPD_L->number = 12345;      // 占位值，运行时被 UI_Task 覆盖

    // ----- AUTO_AIM：自瞄状态数字（0/1/2…，含义在视觉端定义） -----
    ui_g_30HZ_AUTO_AIM->figure_type = 6;
    ui_g_30HZ_AUTO_AIM->operate_type = 1;
    ui_g_30HZ_AUTO_AIM->layer = 0;
    ui_g_30HZ_AUTO_AIM->color = 4;
    ui_g_30HZ_AUTO_AIM->start_x = 300;
    ui_g_30HZ_AUTO_AIM->start_y = 835;
    ui_g_30HZ_AUTO_AIM->width = 2;
    ui_g_30HZ_AUTO_AIM->font_size = 20;
    ui_g_30HZ_AUTO_AIM->number = 12345;

    // ----- SHOOT_NUM：已发射弹丸数（UI_Task 中由 shootnum 全局变量填入） -----
    ui_g_30HZ_SHOOT_NUM->figure_type = 6;
    ui_g_30HZ_SHOOT_NUM->operate_type = 1;
    ui_g_30HZ_SHOOT_NUM->layer = 0;
    ui_g_30HZ_SHOOT_NUM->color = 4;
    ui_g_30HZ_SHOOT_NUM->start_x = 300;
    ui_g_30HZ_SHOOT_NUM->start_y = 800;
    ui_g_30HZ_SHOOT_NUM->width = 2;
    ui_g_30HZ_SHOOT_NUM->font_size = 20;
    ui_g_30HZ_SHOOT_NUM->number = 12345;

    // ----- FRIC_SPD_R：右摩擦轮转速数字 -----
    ui_g_30HZ_FRIC_SPD_R->figure_type = 6;
    ui_g_30HZ_FRIC_SPD_R->operate_type = 1;
    ui_g_30HZ_FRIC_SPD_R->layer = 0;
    ui_g_30HZ_FRIC_SPD_R->color = 4;
    ui_g_30HZ_FRIC_SPD_R->start_x = 417;
    ui_g_30HZ_FRIC_SPD_R->start_y = 864;
    ui_g_30HZ_FRIC_SPD_R->width = 2;
    ui_g_30HZ_FRIC_SPD_R->font_size = 20;
    ui_g_30HZ_FRIC_SPD_R->number = 12345;


    // 组帧 + 发送
    ui_proc_7_frame(&ui_g_30HZ_0);
    SEND_MESSAGE((uint8_t *) &ui_g_30HZ_0, sizeof(ui_g_30HZ_0));
}

/**
 * @brief 30HZ 子帧 0 刷新：把 7 个图元的 operate_type 全打成 2(修改) 再发。
 *        各图元字段已被 UI_Task.c::UI_RefreshParams_30HZ() 提前改好。
 */
void _ui_update_g_30HZ_0() {
    for (int i = 0; i < 7; i++) {
        ui_g_30HZ_0.data[i].operate_type = 2;   // 2=修改
    }

    ui_proc_7_frame(&ui_g_30HZ_0);
    SEND_MESSAGE((uint8_t *) &ui_g_30HZ_0, sizeof(ui_g_30HZ_0));
}

/**
 * @brief 30HZ 子帧 0 删除：把 7 个图元从客户端图层移除。当前未被调用，
 *        留作下电/切场景时清屏使用。
 */
void _ui_remove_g_30HZ_0() {
    for (int i = 0; i < 7; i++) {
        ui_g_30HZ_0.data[i].operate_type = 3;   // 3=删除
    }

    ui_proc_7_frame(&ui_g_30HZ_0);
    SEND_MESSAGE((uint8_t *) &ui_g_30HZ_0, sizeof(ui_g_30HZ_0));
}

// =====================================================================
//  分组 30HZ 子帧 1：7 个图元
//  figure_name = (0, 0, 7..13)，含车身朝向弧/右侧 8009/双腿/超电
// =====================================================================
ui_7_frame_t ui_g_30HZ_1;

ui_interface_arc_t   *ui_g_30HZ_BODY_FRONT = (ui_interface_arc_t*)  &(ui_g_30HZ_1.data[0]); // 车身朝向圆弧
ui_interface_round_t *ui_g_30HZ_8009RF     = (ui_interface_round_t*)&(ui_g_30HZ_1.data[1]); // 右前 8009 心跳
ui_interface_line_t  *ui_g_30HZ_SUPER_CUP  = (ui_interface_line_t*) &(ui_g_30HZ_1.data[2]); // 超电电量条
ui_interface_line_t  *ui_g_30HZ_L_LEG      = (ui_interface_line_t*) &(ui_g_30HZ_1.data[3]); // 左腿可视化
ui_interface_line_t  *ui_g_30HZ_R_LEG      = (ui_interface_line_t*) &(ui_g_30HZ_1.data[4]); // 右腿可视化
ui_interface_line_t  *ui_g_30HZ_BODY_PITCH = (ui_interface_line_t*) &(ui_g_30HZ_1.data[5]); // 车身 pitch 直线
ui_interface_round_t *ui_g_30HZ_8009RB     = (ui_interface_round_t*)&(ui_g_30HZ_1.data[6]); // 右后 8009 心跳

/**
 * @brief 30HZ 子帧 1 初始化
 *        figure_name 第三字节 7..13，避免与子帧 0 (0..6) 冲突。
 */
void _ui_init_g_30HZ_1() {
    for (int i = 0; i < 7; i++) {
        ui_g_30HZ_1.data[i].figure_name[0] = 0;
        ui_g_30HZ_1.data[i].figure_name[1] = 0;
        ui_g_30HZ_1.data[i].figure_name[2] = i + 7;  // 偏移 7，让 ID 全局唯一
        ui_g_30HZ_1.data[i].operate_type = 1;
    }
    for (int i = 7; i < 7; i++) {
        ui_g_30HZ_1.data[i].operate_type = 0;
    }

    // ----- BODY_FRONT：车身朝向圆弧（屏幕正中央的"罗盘指针") -----
    ui_g_30HZ_BODY_FRONT->figure_type = 4;     // 4=圆弧
    ui_g_30HZ_BODY_FRONT->operate_type = 1;
    ui_g_30HZ_BODY_FRONT->layer = 0;
    ui_g_30HZ_BODY_FRONT->color = 4;
    ui_g_30HZ_BODY_FRONT->start_x = 960;       // 椭圆中心 = 屏幕中央 (1920/2, 1080/2)
    ui_g_30HZ_BODY_FRONT->start_y = 540;
    ui_g_30HZ_BODY_FRONT->width = 2;
    ui_g_30HZ_BODY_FRONT->start_angle = 340;   // 起角(度)，运行时随车身角度更新
    ui_g_30HZ_BODY_FRONT->end_angle = 20;      // 止角(度)
    ui_g_30HZ_BODY_FRONT->rx = 200;            // 椭圆长轴半径
    ui_g_30HZ_BODY_FRONT->ry = 200;            // 椭圆短轴半径（rx=ry => 正圆弧）

    // ----- 8009RF：右前 DM8009 心跳圆 -----
    ui_g_30HZ_8009RF->figure_type = 2;
    ui_g_30HZ_8009RF->operate_type = 1;
    ui_g_30HZ_8009RF->layer = 0;
    ui_g_30HZ_8009RF->color = 4;
    ui_g_30HZ_8009RF->start_x = 1900;
    ui_g_30HZ_8009RF->start_y = 852;
    ui_g_30HZ_8009RF->width = 10;
    ui_g_30HZ_8009RF->r = 5;

    // ----- SUPER_CUP：超级电容剩余电量条（水平直线，宽 15 模拟"进度条"） -----
    ui_g_30HZ_SUPER_CUP->figure_type = 0;      // 0=直线
    ui_g_30HZ_SUPER_CUP->operate_type = 1;
    ui_g_30HZ_SUPER_CUP->layer = 0;
    ui_g_30HZ_SUPER_CUP->color = 2;            // 颜色 2 = 黄
    ui_g_30HZ_SUPER_CUP->start_x = 623;        // 左端点
    ui_g_30HZ_SUPER_CUP->start_y = 123;
    ui_g_30HZ_SUPER_CUP->width = 15;
    ui_g_30HZ_SUPER_CUP->end_x = 1307;         // 右端点（满电量长度）
    ui_g_30HZ_SUPER_CUP->end_y = 121;

    // ----- L_LEG：左腿可视化直线（起点固定，终点随 L0/b_phi0 实时摆动） -----
    ui_g_30HZ_L_LEG->figure_type = 0;
    ui_g_30HZ_L_LEG->operate_type = 1;
    ui_g_30HZ_L_LEG->layer = 0;
    ui_g_30HZ_L_LEG->color = 2;
    ui_g_30HZ_L_LEG->start_x = 1596;
    ui_g_30HZ_L_LEG->start_y = 710;
    ui_g_30HZ_L_LEG->width = 1;
    ui_g_30HZ_L_LEG->end_x = 1594;
    ui_g_30HZ_L_LEG->end_y = 530;

    // ----- R_LEG：右腿可视化直线 -----
    ui_g_30HZ_R_LEG->figure_type = 0;
    ui_g_30HZ_R_LEG->operate_type = 1;
    ui_g_30HZ_R_LEG->layer = 0;
    ui_g_30HZ_R_LEG->color = 2;
    ui_g_30HZ_R_LEG->start_x = 1713;
    ui_g_30HZ_R_LEG->start_y = 718;
    ui_g_30HZ_R_LEG->width = 1;
    ui_g_30HZ_R_LEG->end_x = 1715;
    ui_g_30HZ_R_LEG->end_y = 538;

    // ----- BODY_PITCH：车身 pitch 角直线（以 (1650,700) 为中心、半长 150 旋转） -----
    ui_g_30HZ_BODY_PITCH->figure_type = 0;
    ui_g_30HZ_BODY_PITCH->operate_type = 1;
    ui_g_30HZ_BODY_PITCH->layer = 0;
    ui_g_30HZ_BODY_PITCH->color = 2;
    ui_g_30HZ_BODY_PITCH->start_x = 1791;   // 1800 - cosf 微调
    ui_g_30HZ_BODY_PITCH->start_y = 711;
    ui_g_30HZ_BODY_PITCH->width = 1;
    ui_g_30HZ_BODY_PITCH->end_x = 1476;     // 1500 - cosf 微调
    ui_g_30HZ_BODY_PITCH->end_y = 718;

    // ----- 8009RB：右后 DM8009 心跳圆 -----
    ui_g_30HZ_8009RB->figure_type = 2;
    ui_g_30HZ_8009RB->operate_type = 1;
    ui_g_30HZ_8009RB->layer = 0;
    ui_g_30HZ_8009RB->color = 4;
    ui_g_30HZ_8009RB->start_x = 1900;
    ui_g_30HZ_8009RB->start_y = 828;
    ui_g_30HZ_8009RB->width = 10;
    ui_g_30HZ_8009RB->r = 5;


    ui_proc_7_frame(&ui_g_30HZ_1);
    SEND_MESSAGE((uint8_t *) &ui_g_30HZ_1, sizeof(ui_g_30HZ_1));
}

/** @brief 30HZ 子帧 1 刷新（修改）。字段已由 UI_RefreshParams_30HZ 改好。 */
void _ui_update_g_30HZ_1() {
    for (int i = 0; i < 7; i++) {
        ui_g_30HZ_1.data[i].operate_type = 2;
    }

    ui_proc_7_frame(&ui_g_30HZ_1);
    SEND_MESSAGE((uint8_t *) &ui_g_30HZ_1, sizeof(ui_g_30HZ_1));
}

/** @brief 30HZ 子帧 1 删除。当前不调用。 */
void _ui_remove_g_30HZ_1() {
    for (int i = 0; i < 7; i++) {
        ui_g_30HZ_1.data[i].operate_type = 3;
    }

    ui_proc_7_frame(&ui_g_30HZ_1);
    SEND_MESSAGE((uint8_t *) &ui_g_30HZ_1, sizeof(ui_g_30HZ_1));
}

// =====================================================================
//  分组 30HZ 子帧 2：7 个图元
//  figure_name = (0, 0, 14..20)
// =====================================================================
ui_7_frame_t ui_g_30HZ_2;

ui_interface_number_t *ui_g_30HZ_BUFFER_NUM = (ui_interface_number_t*)&(ui_g_30HZ_2.data[0]); // 缓冲能量数字(TODO)
ui_interface_round_t  *ui_g_30HZ_POWER_METER = (ui_interface_round_t*)&(ui_g_30HZ_2.data[1]); // 功率计心跳
ui_interface_round_t  *ui_g_30HZ_485         = (ui_interface_round_t*)&(ui_g_30HZ_2.data[2]); // 485 板间通信心跳
ui_interface_round_t  *ui_g_30HZ_UNNAME1     = (ui_interface_round_t*)&(ui_g_30HZ_2.data[3]); // 备用 1
ui_interface_round_t  *ui_g_30HZ_UNNAME2     = (ui_interface_round_t*)&(ui_g_30HZ_2.data[4]); // 备用 2
ui_interface_round_t  *ui_g_30HZ_UNNAME3     = (ui_interface_round_t*)&(ui_g_30HZ_2.data[5]); // 1Hz 程序运行指示灯
ui_interface_round_t  *ui_g_30HZ_3508L       = (ui_interface_round_t*)&(ui_g_30HZ_2.data[6]); // 左 3508 心跳

/** @brief 30HZ 子帧 2 初始化，figure_name 第三字节 14..20 */
void _ui_init_g_30HZ_2() {
    for (int i = 0; i < 7; i++) {
        ui_g_30HZ_2.data[i].figure_name[0] = 0;
        ui_g_30HZ_2.data[i].figure_name[1] = 0;
        ui_g_30HZ_2.data[i].figure_name[2] = i + 14;   // ID 偏移 14
        ui_g_30HZ_2.data[i].operate_type = 1;
    }
    for (int i = 7; i < 7; i++) {
        ui_g_30HZ_2.data[i].operate_type = 0;
    }

    // ----- BUFFER_NUM：缓冲能量数字（TODO，未接驱动） -----
    ui_g_30HZ_BUFFER_NUM->figure_type = 6;
    ui_g_30HZ_BUFFER_NUM->operate_type = 1;
    ui_g_30HZ_BUFFER_NUM->layer = 0;
    ui_g_30HZ_BUFFER_NUM->color = 4;
    ui_g_30HZ_BUFFER_NUM->start_x = 628;
    ui_g_30HZ_BUFFER_NUM->start_y = 102;
    ui_g_30HZ_BUFFER_NUM->width = 2;
    ui_g_30HZ_BUFFER_NUM->font_size = 20;
    ui_g_30HZ_BUFFER_NUM->number = 12345;

    // ----- POWER_METER：功率计心跳（CAN3 0x213，500ms 内未变即判断断联） -----
    ui_g_30HZ_POWER_METER->figure_type = 2;
    ui_g_30HZ_POWER_METER->operate_type = 1;
    ui_g_30HZ_POWER_METER->layer = 0;
    ui_g_30HZ_POWER_METER->color = 4;
    ui_g_30HZ_POWER_METER->start_x = 1900;
    ui_g_30HZ_POWER_METER->start_y = 636;
    ui_g_30HZ_POWER_METER->width = 10;
    ui_g_30HZ_POWER_METER->r = 5;

    // ----- 485：板间通信心跳（看 Board2Board.B2B_offline_flag） -----
    ui_g_30HZ_485->figure_type = 2;
    ui_g_30HZ_485->operate_type = 1;
    ui_g_30HZ_485->layer = 0;
    ui_g_30HZ_485->color = 4;
    ui_g_30HZ_485->start_x = 1900;
    ui_g_30HZ_485->start_y = 612;
    ui_g_30HZ_485->width = 10;
    ui_g_30HZ_485->r = 5;

    // ----- UNNAME1：备用心跳灯 1（未指派含义） -----
    ui_g_30HZ_UNNAME1->figure_type = 2;
    ui_g_30HZ_UNNAME1->operate_type = 1;
    ui_g_30HZ_UNNAME1->layer = 0;
    ui_g_30HZ_UNNAME1->color = 4;
    ui_g_30HZ_UNNAME1->start_x = 1900;
    ui_g_30HZ_UNNAME1->start_y = 588;
    ui_g_30HZ_UNNAME1->width = 10;
    ui_g_30HZ_UNNAME1->r = 5;

    // ----- UNNAME2：备用心跳灯 2 -----
    ui_g_30HZ_UNNAME2->figure_type = 2;
    ui_g_30HZ_UNNAME2->operate_type = 1;
    ui_g_30HZ_UNNAME2->layer = 0;
    ui_g_30HZ_UNNAME2->color = 4;
    ui_g_30HZ_UNNAME2->start_x = 1900;
    ui_g_30HZ_UNNAME2->start_y = 564;
    ui_g_30HZ_UNNAME2->width = 10;
    ui_g_30HZ_UNNAME2->r = 5;

    // ----- UNNAME3：被 UI_Task 用作 1Hz "程序还活着"闪烁指示灯 -----
    ui_g_30HZ_UNNAME3->figure_type = 2;
    ui_g_30HZ_UNNAME3->operate_type = 1;
    ui_g_30HZ_UNNAME3->layer = 0;
    ui_g_30HZ_UNNAME3->color = 4;
    ui_g_30HZ_UNNAME3->start_x = 1900;
    ui_g_30HZ_UNNAME3->start_y = 540;
    ui_g_30HZ_UNNAME3->width = 10;
    ui_g_30HZ_UNNAME3->r = 5;

    // ----- 3508L：左 3508 驱动轮电机心跳 -----
    ui_g_30HZ_3508L->figure_type = 2;
    ui_g_30HZ_3508L->operate_type = 1;
    ui_g_30HZ_3508L->layer = 0;
    ui_g_30HZ_3508L->color = 4;
    ui_g_30HZ_3508L->start_x = 1900;
    ui_g_30HZ_3508L->start_y = 804;
    ui_g_30HZ_3508L->width = 10;
    ui_g_30HZ_3508L->r = 5;


    ui_proc_7_frame(&ui_g_30HZ_2);
    SEND_MESSAGE((uint8_t *) &ui_g_30HZ_2, sizeof(ui_g_30HZ_2));
}

/** @brief 30HZ 子帧 2 刷新（修改） */
void _ui_update_g_30HZ_2() {
    for (int i = 0; i < 7; i++) {
        ui_g_30HZ_2.data[i].operate_type = 2;
    }

    ui_proc_7_frame(&ui_g_30HZ_2);
    SEND_MESSAGE((uint8_t *) &ui_g_30HZ_2, sizeof(ui_g_30HZ_2));
}

/** @brief 30HZ 子帧 2 删除（当前未调用） */
void _ui_remove_g_30HZ_2() {
    for (int i = 0; i < 7; i++) {
        ui_g_30HZ_2.data[i].operate_type = 3;
    }

    ui_proc_7_frame(&ui_g_30HZ_2);
    SEND_MESSAGE((uint8_t *) &ui_g_30HZ_2, sizeof(ui_g_30HZ_2));
}

// =====================================================================
//  分组 30HZ 子帧 3：5 个图元（5 图元帧 ui_5_frame_t）
//  figure_name = (0, 0, 21..25)
// =====================================================================
ui_5_frame_t ui_g_30HZ_3;

ui_interface_round_t *ui_g_30HZ_3508R = (ui_interface_round_t*)&(ui_g_30HZ_3.data[0]); // 右 3508 心跳
ui_interface_round_t *ui_g_30HZ_PITCH = (ui_interface_round_t*)&(ui_g_30HZ_3.data[1]); // PITCH 电机心跳(TODO)
ui_interface_round_t *ui_g_30HZ_ROLL  = (ui_interface_round_t*)&(ui_g_30HZ_3.data[2]); // 4310 Yaw 电机心跳(复用 ROLL 位)
ui_interface_round_t *ui_g_30HZ_FRIC_L= (ui_interface_round_t*)&(ui_g_30HZ_3.data[3]); // 左摩擦轮电机心跳(TODO)
ui_interface_round_t *ui_g_30HZ_FRIC_R= (ui_interface_round_t*)&(ui_g_30HZ_3.data[4]); // 右摩擦轮电机心跳(TODO)

/** @brief 30HZ 子帧 3 初始化（5 槽满载），figure_name 第三字节 21..25 */
void _ui_init_g_30HZ_3() {
    for (int i = 0; i < 5; i++) {
        ui_g_30HZ_3.data[i].figure_name[0] = 0;
        ui_g_30HZ_3.data[i].figure_name[1] = 0;
        ui_g_30HZ_3.data[i].figure_name[2] = i + 21;
        ui_g_30HZ_3.data[i].operate_type = 1;
    }
    for (int i = 5; i < 5; i++) {
        ui_g_30HZ_3.data[i].operate_type = 0;
    }

    // ----- 3508R：右 3508 驱动轮电机心跳 -----
    ui_g_30HZ_3508R->figure_type = 2;
    ui_g_30HZ_3508R->operate_type = 1;
    ui_g_30HZ_3508R->layer = 0;
    ui_g_30HZ_3508R->color = 4;
    ui_g_30HZ_3508R->start_x = 1900;
    ui_g_30HZ_3508R->start_y = 780;
    ui_g_30HZ_3508R->width = 10;
    ui_g_30HZ_3508R->r = 5;

    // ----- PITCH：俯仰电机心跳（TODO，尚未接驱动） -----
    ui_g_30HZ_PITCH->figure_type = 2;
    ui_g_30HZ_PITCH->operate_type = 1;
    ui_g_30HZ_PITCH->layer = 0;
    ui_g_30HZ_PITCH->color = 4;
    ui_g_30HZ_PITCH->start_x = 1900;
    ui_g_30HZ_PITCH->start_y = 756;
    ui_g_30HZ_PITCH->width = 10;
    ui_g_30HZ_PITCH->r = 5;

    // ----- ROLL：当前被复用为 4310 Yaw 电机心跳（见 UI_Task.c::check_lost） -----
    ui_g_30HZ_ROLL->figure_type = 2;
    ui_g_30HZ_ROLL->operate_type = 1;
    ui_g_30HZ_ROLL->layer = 0;
    ui_g_30HZ_ROLL->color = 4;
    ui_g_30HZ_ROLL->start_x = 1900;
    ui_g_30HZ_ROLL->start_y = 732;
    ui_g_30HZ_ROLL->width = 10;
    ui_g_30HZ_ROLL->r = 5;

    // ----- FRIC_L：左摩擦轮电机心跳（TODO） -----
    ui_g_30HZ_FRIC_L->figure_type = 2;
    ui_g_30HZ_FRIC_L->operate_type = 1;
    ui_g_30HZ_FRIC_L->layer = 0;
    ui_g_30HZ_FRIC_L->color = 4;
    ui_g_30HZ_FRIC_L->start_x = 1900;
    ui_g_30HZ_FRIC_L->start_y = 708;
    ui_g_30HZ_FRIC_L->width = 10;
    ui_g_30HZ_FRIC_L->r = 5;

    // ----- FRIC_R：右摩擦轮电机心跳（TODO） -----
    ui_g_30HZ_FRIC_R->figure_type = 2;
    ui_g_30HZ_FRIC_R->operate_type = 1;
    ui_g_30HZ_FRIC_R->layer = 0;
    ui_g_30HZ_FRIC_R->color = 4;
    ui_g_30HZ_FRIC_R->start_x = 1900;
    ui_g_30HZ_FRIC_R->start_y = 684;
    ui_g_30HZ_FRIC_R->width = 10;
    ui_g_30HZ_FRIC_R->r = 5;


    ui_proc_5_frame(&ui_g_30HZ_3);
    SEND_MESSAGE((uint8_t *) &ui_g_30HZ_3, sizeof(ui_g_30HZ_3));
}

/** @brief 30HZ 子帧 3 刷新（修改） */
void _ui_update_g_30HZ_3() {
    for (int i = 0; i < 5; i++) {
        ui_g_30HZ_3.data[i].operate_type = 2;
    }

    ui_proc_5_frame(&ui_g_30HZ_3);
    SEND_MESSAGE((uint8_t *) &ui_g_30HZ_3, sizeof(ui_g_30HZ_3));
}

/** @brief 30HZ 子帧 3 删除（当前未调用） */
void _ui_remove_g_30HZ_3() {
    for (int i = 0; i < 5; i++) {
        ui_g_30HZ_3.data[i].operate_type = 3;
    }

    ui_proc_5_frame(&ui_g_30HZ_3);
    SEND_MESSAGE((uint8_t *) &ui_g_30HZ_3, sizeof(ui_g_30HZ_3));
}


// =====================================================================
//  30HZ 分组聚合入口（由 ui.h 暴露给业务层调用）
//  init/update/remove 一次性把 4 个子帧全部发掉。
//  注：UI_Task 启动期才用 ui_init_g_30HZ；运行期是按 cnt%10 单帧轮询
//      调用 _ui_update_g_30HZ_X，不调用这里的聚合 update。
//
//  子帧之间 osDelay(UI_FRAME_GAP_MS)：单帧最长 116B @115200 ≈ 10ms，等 30ms 留充足余量
//  覆盖 osDelay 的 tick 对齐误差和 HAL gState 复位的潜在拖尾。
// =====================================================================
void ui_init_g_30HZ() {
    _ui_init_g_30HZ_0();
    osDelay(UI_FRAME_GAP_MS);
    _ui_init_g_30HZ_1();
    osDelay(UI_FRAME_GAP_MS);
    _ui_init_g_30HZ_2();
    osDelay(UI_FRAME_GAP_MS);
    _ui_init_g_30HZ_3();
}

void ui_update_g_30HZ() {
    _ui_update_g_30HZ_0();
    osDelay(UI_FRAME_GAP_MS);
    _ui_update_g_30HZ_1();
    osDelay(UI_FRAME_GAP_MS);
    _ui_update_g_30HZ_2();
    osDelay(UI_FRAME_GAP_MS);
    _ui_update_g_30HZ_3();
}

void ui_remove_g_30HZ() {
    _ui_remove_g_30HZ_0();
    osDelay(UI_FRAME_GAP_MS);
    _ui_remove_g_30HZ_1();
    osDelay(UI_FRAME_GAP_MS);
    _ui_remove_g_30HZ_2();
    osDelay(UI_FRAME_GAP_MS);
    _ui_remove_g_30HZ_3();
}


// =====================================================================
//  分组 5HZ：低频文字提示
//  每个文字独立成帧（字符串包 sub_id=0x0110，无法多图元同包）
//  figure_name 第二字节 = 1 (5HZ 分组前缀)，第三字节 = 子帧内序号
// =====================================================================
ui_string_frame_t ui_g_5HZ_0;                                 // "PLEASE SPIN" 文字所在帧
ui_interface_string_t* ui_g_5HZ_NewText = &(ui_g_5HZ_0.option); // 指针指向帧内 option

/** @brief 5HZ 子帧 0 初始化 "PLEASE SPIN"（左下提示） */
void _ui_init_g_5HZ_0() {
    ui_g_5HZ_0.option.figure_name[0] = 0;
    ui_g_5HZ_0.option.figure_name[1] = 1;   // 1 = 5HZ 分组前缀
    ui_g_5HZ_0.option.figure_name[2] = 0;
    ui_g_5HZ_0.option.operate_type = 1;

    ui_g_5HZ_NewText->figure_type = 7;      // 7 = 字符串
    ui_g_5HZ_NewText->operate_type = 1;
    ui_g_5HZ_NewText->layer = 0;
    ui_g_5HZ_NewText->color = 4;
    ui_g_5HZ_NewText->start_x = 743;        // 屏幕坐标
    ui_g_5HZ_NewText->start_y = 852;
    ui_g_5HZ_NewText->width = 4;            // 笔划粗细
    ui_g_5HZ_NewText->font_size = 40;       // 字号
    ui_g_5HZ_NewText->str_length = 11;      // strlen("PLEASE SPIN")=11
    strcpy(ui_g_5HZ_NewText->string, "PLEASE SPIN");


    ui_proc_string_frame(&ui_g_5HZ_0);
    SEND_MESSAGE((uint8_t *) &ui_g_5HZ_0, sizeof(ui_g_5HZ_0));
}

/** @brief 5HZ 子帧 0 刷新；UI_Task 通过 str_length=0 切"隐藏" */
void _ui_update_g_5HZ_0() {
    ui_g_5HZ_0.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_5HZ_0);
    SEND_MESSAGE((uint8_t *) &ui_g_5HZ_0, sizeof(ui_g_5HZ_0));
}

/** @brief 5HZ 子帧 0 删除（当前未调用） */
void _ui_remove_g_5HZ_0() {
    ui_g_5HZ_0.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_5HZ_0);
    SEND_MESSAGE((uint8_t *) &ui_g_5HZ_0, sizeof(ui_g_5HZ_0));
}

ui_string_frame_t ui_g_5HZ_1;                                  // "LONG LEG" 文字所在帧
ui_interface_string_t* ui_g_5HZ_NewText2 = &(ui_g_5HZ_1.option);

/** @brief 5HZ 子帧 1 初始化 "LONG LEG"（右下提示） */
void _ui_init_g_5HZ_1() {
    ui_g_5HZ_1.option.figure_name[0] = 0;
    ui_g_5HZ_1.option.figure_name[1] = 1;
    ui_g_5HZ_1.option.figure_name[2] = 1;
    ui_g_5HZ_1.option.operate_type = 1;

    ui_g_5HZ_NewText2->figure_type = 7;
    ui_g_5HZ_NewText2->operate_type = 1;
    ui_g_5HZ_NewText2->layer = 0;
    ui_g_5HZ_NewText2->color = 4;
    ui_g_5HZ_NewText2->start_x = 1346;
    ui_g_5HZ_NewText2->start_y = 852;
    ui_g_5HZ_NewText2->width = 4;
    ui_g_5HZ_NewText2->font_size = 40;
    ui_g_5HZ_NewText2->str_length = 8;          // strlen("LONG LEG")=8
    strcpy(ui_g_5HZ_NewText2->string, "LONG LEG");


    ui_proc_string_frame(&ui_g_5HZ_1);
    SEND_MESSAGE((uint8_t *) &ui_g_5HZ_1, sizeof(ui_g_5HZ_1));
}

/** @brief 5HZ 子帧 1 刷新 */
void _ui_update_g_5HZ_1() {
    ui_g_5HZ_1.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_5HZ_1);
    SEND_MESSAGE((uint8_t *) &ui_g_5HZ_1, sizeof(ui_g_5HZ_1));
}

/** @brief 5HZ 子帧 1 删除（当前未调用） */
void _ui_remove_g_5HZ_1() {
    ui_g_5HZ_1.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_5HZ_1);
    SEND_MESSAGE((uint8_t *) &ui_g_5HZ_1, sizeof(ui_g_5HZ_1));
}

/** @brief 5HZ 分组聚合入口：启动期发一次新增，让客户端注册两个文字图元 */
void ui_init_g_5HZ() {
    _ui_init_g_5HZ_0();
    osDelay(UI_FRAME_GAP_MS);
    _ui_init_g_5HZ_1();
}

void ui_update_g_5HZ() {
    _ui_update_g_5HZ_0();
    osDelay(UI_FRAME_GAP_MS);
    _ui_update_g_5HZ_1();
}

void ui_remove_g_5HZ() {
    _ui_remove_g_5HZ_0();
    osDelay(UI_FRAME_GAP_MS);
    _ui_remove_g_5HZ_1();
}

// =====================================================================
//  分组 INIT：永久静态层（5 条装饰线 + 3 个标签字）
//  上电发一次，UI_Task 每 3s 兜底重发，防丢包后图元永远消失。
//  figure_name 第二字节 = 2 (INIT 分组前缀)
// =====================================================================
ui_5_frame_t ui_g_INIT_0;     // 5 条静态线打成一个 5 图元帧

ui_interface_line_t *ui_g_INIT_NewLine  = (ui_interface_line_t*)&(ui_g_INIT_0.data[0]); // 左斜装饰线
ui_interface_line_t *ui_g_INIT_NewLine2 = (ui_interface_line_t*)&(ui_g_INIT_0.data[1]); // 右斜装饰线
ui_interface_line_t *ui_g_INIT_NewLine3 = (ui_interface_line_t*)&(ui_g_INIT_0.data[2]); // 装饰横线 1
ui_interface_line_t *ui_g_INIT_NewLine4 = (ui_interface_line_t*)&(ui_g_INIT_0.data[3]); // 装饰横线 2
ui_interface_line_t *ui_g_INIT_NewLine5 = (ui_interface_line_t*)&(ui_g_INIT_0.data[4]); // 顶部横线

/**
 * @brief INIT 子帧 0 初始化：5 条静态装饰线
 *
 * 这里只做"打标识 + 标新增"两件事，真正的几何坐标在下面 5 个图元块里填。
 * 拆成两步是因为 figure_name / operate_type 对 5 个图元写法一样，
 * 用循环统一处理可避免漏改。
 */
void _ui_init_g_INIT_0() {
    // --- 第一步：把帧里 5 个图元槽统一打 ID，并标"新增"------------------
    // ui_g_INIT_0.data[i] —— 第 i 个原始图元槽，类型 ui_interface_figure_t，15 字节。
    // 同帧里所有图元的 figure_name 必须各不相同，否则客户端会把后填的覆盖到前面。
    //
    // figure_name[3] 是协议规定的 3 字节唯一 ID，本项目按"分组 + 序号"切分使用：
    //   figure_name[0] —— 预留位，全工程恒为 0（裁判系统未规定用途，保持兼容）
    //   figure_name[1] —— 分组前缀：0=30HZ 动态层，1=5HZ 文字层，2=INIT 静态层
    //   figure_name[2] —— 组内序号：本子帧填 0..4，对应屏幕上的 5 条装饰线
    //
    // operate_type 是 3bit 位域，协议定义：
    //   0=空操作（客户端忽略该槽）  1=新增  2=修改  3=删除
    // 这里调用场景是"上电首次注册图元"，所以四槽全部标 1=新增。
    for (int i = 0; i < 5; i++) {
        ui_g_INIT_0.data[i].figure_name[0] = 0;       // 预留位恒 0
        ui_g_INIT_0.data[i].figure_name[1] = 2;       // 分组前缀=2，表示 INIT 静态层
        ui_g_INIT_0.data[i].figure_name[2] = i + 0;   // 组内序号 0..4；"+0"是 Designer 模板留下的基址偏移
        ui_g_INIT_0.data[i].operate_type = 1;         // 1=新增，让客户端首次画出该图元
    }

    // --- 第二步：把"未使用的槽位"标成空操作 ------------------------------
    // 这是 RM UI Designer 代码生成器留下的模板片段：
    // 模板原型是 `for (int i = N_USED; i < N_MAX; i++)`，意思是
    // "本帧能装 N_MAX 槽但只用了 N_USED 槽，剩下的标 0 让客户端忽略"。
    // 本帧是 ui_5_frame_t（N_MAX=5）且 5 槽全填满了，所以 N_USED=N_MAX=5，
    // 循环条件 5<5 立即为假 —— 这是空循环、本身不会执行任何代码。
    // 不删它的原因：将来若 Designer 重新生成代码，会按"槽满 / 没满"两种模式
    // 输出统一的两段循环结构，留着可以无缝复盖、避免手改后被覆盖丢失。
    for (int i = 5; i < 5; i++) {
        ui_g_INIT_0.data[i].operate_type = 0;         // 0=空操作（不会真正被执行）
    }

    // ----- NewLine：左侧大斜装饰线 -----
    ui_g_INIT_NewLine->figure_type = 0;     // 0=直线
    ui_g_INIT_NewLine->operate_type = 1;
    ui_g_INIT_NewLine->layer = 0;
    ui_g_INIT_NewLine->color = 8;           // 颜色 8 = 紫
    ui_g_INIT_NewLine->start_x = 571;
    ui_g_INIT_NewLine->start_y = 101;
    ui_g_INIT_NewLine->width = 1;
    ui_g_INIT_NewLine->end_x = 824;
    ui_g_INIT_NewLine->end_y = 412;

    // ----- NewLine2：右侧大斜装饰线 -----
    ui_g_INIT_NewLine2->figure_type = 0;
    ui_g_INIT_NewLine2->operate_type = 1;
    ui_g_INIT_NewLine2->layer = 0;
    ui_g_INIT_NewLine2->color = 8;
    ui_g_INIT_NewLine2->start_x = 1345;
    ui_g_INIT_NewLine2->start_y = 92;
    ui_g_INIT_NewLine2->width = 1;
    ui_g_INIT_NewLine2->end_x = 1066;
    ui_g_INIT_NewLine2->end_y = 395;

    // ----- NewLine3：装饰横线 1 -----
    ui_g_INIT_NewLine3->figure_type = 0;
    ui_g_INIT_NewLine3->operate_type = 1;
    ui_g_INIT_NewLine3->layer = 0;
    ui_g_INIT_NewLine3->color = 8;
    ui_g_INIT_NewLine3->start_x = 1023;
    ui_g_INIT_NewLine3->start_y = 688;
    ui_g_INIT_NewLine3->width = 1;
    ui_g_INIT_NewLine3->end_x = 883;
    ui_g_INIT_NewLine3->end_y = 690;

    // ----- NewLine4：装饰横线 2 -----
    ui_g_INIT_NewLine4->figure_type = 0;
    ui_g_INIT_NewLine4->operate_type = 1;
    ui_g_INIT_NewLine4->layer = 0;
    ui_g_INIT_NewLine4->color = 8;
    ui_g_INIT_NewLine4->start_x = 1025;
    ui_g_INIT_NewLine4->start_y = 641;
    ui_g_INIT_NewLine4->width = 1;
    ui_g_INIT_NewLine4->end_x = 885;
    ui_g_INIT_NewLine4->end_y = 643;

    // ----- NewLine5：顶部装饰横线 -----
    ui_g_INIT_NewLine5->figure_type = 0;
    ui_g_INIT_NewLine5->operate_type = 1;
    ui_g_INIT_NewLine5->layer = 0;
    ui_g_INIT_NewLine5->color = 8;
    ui_g_INIT_NewLine5->start_x = 626;
    ui_g_INIT_NewLine5->start_y = 278;
    ui_g_INIT_NewLine5->width = 1;
    ui_g_INIT_NewLine5->end_x = 1216;
    ui_g_INIT_NewLine5->end_y = 285;


    // ===== 封口 & 发送 ============================================
    // 到这里 ui_g_INIT_0 的图元数据(5 个 15 字节槽)已经填完，
    // 但裁判系统要求的"协议头 + CRC8 + CRC16"还没补，发出去客户端会丢包。
    //
    // ui_proc_5_frame() —— 5 图元帧封口函数（在 ui_interface.c 由
    //   宏 DEFINE_FRAME_PROC(5, 0x0103) 展开生成）。它做 4 件事：
    //     ① 填协议头 5 字节：SOF=0xA5 / length=6+15*5=81 / seq 自增 / CRC8
    //     ② 填命令字段 cmd_id=0x0301（"客户端绘图"指令）
    //     ③ 填 sub_id=0x0103（"5 图元帧"子命令）+ send_id/recv_id（己方机器人 ID
    //        转客户端 ID = ui_self_id + 256）
    //     ④ 对整包前 13+15*5=88 字节算 CRC16 写到帧尾 msg->crc16
    //   计算细节见 ui_interface.c 的 DEFINE_FRAME_PROC 宏定义。
    //
    // SEND_MESSAGE 是 ui_interface.h 里把 ui_send_message 套了一层的宏，
    // 走 UART1 DMA 把整包(sizeof(ui_g_INIT_0)=90 字节)异步搬到裁判系统。
    // 异步：函数立刻返回，发送过程由 DMA 中断完成，无阻塞。
    ui_proc_5_frame(&ui_g_INIT_0);
    SEND_MESSAGE((uint8_t *) &ui_g_INIT_0, sizeof(ui_g_INIT_0));
}

/** @brief INIT 子帧 0 刷新：UI_Task 每 3s 兜底重发用，operate_type=2(修改) */
void _ui_update_g_INIT_0() {
    for (int i = 0; i < 5; i++) {
        ui_g_INIT_0.data[i].operate_type = 2;
    }

    ui_proc_5_frame(&ui_g_INIT_0);
    SEND_MESSAGE((uint8_t *) &ui_g_INIT_0, sizeof(ui_g_INIT_0));
}

/** @brief INIT 子帧 0 删除（当前未调用） */
void _ui_remove_g_INIT_0() {
    for (int i = 0; i < 5; i++) {
        ui_g_INIT_0.data[i].operate_type = 3;
    }

    ui_proc_5_frame(&ui_g_INIT_0);
    SEND_MESSAGE((uint8_t *) &ui_g_INIT_0, sizeof(ui_g_INIT_0));
}

// ===== INIT 子帧 1："FRIC SPD" 标签 ====================================
ui_string_frame_t ui_g_INIT_1;
ui_interface_string_t* ui_g_INIT_FRIC_SPD = &(ui_g_INIT_1.option);

/** @brief INIT 子帧 1 初始化 "FRIC SPD" 标签字 */
void _ui_init_g_INIT_1() {
    ui_g_INIT_1.option.figure_name[0] = 0;
    ui_g_INIT_1.option.figure_name[1] = 2;
    ui_g_INIT_1.option.figure_name[2] = 5;
    ui_g_INIT_1.option.operate_type = 1;

    ui_g_INIT_FRIC_SPD->figure_type = 7;
    ui_g_INIT_FRIC_SPD->operate_type = 1;
    ui_g_INIT_FRIC_SPD->layer = 0;
    ui_g_INIT_FRIC_SPD->color = 1;          // 颜色 1 = 红
    ui_g_INIT_FRIC_SPD->start_x = 60;
    ui_g_INIT_FRIC_SPD->start_y = 870;
    ui_g_INIT_FRIC_SPD->width = 2;
    ui_g_INIT_FRIC_SPD->font_size = 20;
    ui_g_INIT_FRIC_SPD->str_length = 8;
    strcpy(ui_g_INIT_FRIC_SPD->string, "FRIC SPD");


    ui_proc_string_frame(&ui_g_INIT_1);
    SEND_MESSAGE((uint8_t *) &ui_g_INIT_1, sizeof(ui_g_INIT_1));
}

/** @brief INIT 子帧 1 刷新 */
void _ui_update_g_INIT_1() {
    ui_g_INIT_1.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_INIT_1);
    SEND_MESSAGE((uint8_t *) &ui_g_INIT_1, sizeof(ui_g_INIT_1));
}

/** @brief INIT 子帧 1 删除（当前未调用） */
void _ui_remove_g_INIT_1() {
    ui_g_INIT_1.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_INIT_1);
    SEND_MESSAGE((uint8_t *) &ui_g_INIT_1, sizeof(ui_g_INIT_1));
}

// ===== INIT 子帧 2："AUTO AIM" 标签 ====================================
ui_string_frame_t ui_g_INIT_2;
ui_interface_string_t* ui_g_INIT_AUTO_AIM = &(ui_g_INIT_2.option);

/** @brief INIT 子帧 2 初始化 "AUTO AIM" 标签字 */
void _ui_init_g_INIT_2() {
    ui_g_INIT_2.option.figure_name[0] = 0;
    ui_g_INIT_2.option.figure_name[1] = 2;
    ui_g_INIT_2.option.figure_name[2] = 6;
    ui_g_INIT_2.option.operate_type = 1;

    ui_g_INIT_AUTO_AIM->figure_type = 7;
    ui_g_INIT_AUTO_AIM->operate_type = 1;
    ui_g_INIT_AUTO_AIM->layer = 0;
    ui_g_INIT_AUTO_AIM->color = 1;
    ui_g_INIT_AUTO_AIM->start_x = 60;
    ui_g_INIT_AUTO_AIM->start_y = 835;
    ui_g_INIT_AUTO_AIM->width = 2;
    ui_g_INIT_AUTO_AIM->font_size = 20;
    ui_g_INIT_AUTO_AIM->str_length = 8;
    strcpy(ui_g_INIT_AUTO_AIM->string, "AUTO AIM");


    ui_proc_string_frame(&ui_g_INIT_2);
    SEND_MESSAGE((uint8_t *) &ui_g_INIT_2, sizeof(ui_g_INIT_2));
}

/** @brief INIT 子帧 2 刷新 */
void _ui_update_g_INIT_2() {
    ui_g_INIT_2.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_INIT_2);
    SEND_MESSAGE((uint8_t *) &ui_g_INIT_2, sizeof(ui_g_INIT_2));
}

/** @brief INIT 子帧 2 删除（当前未调用） */
void _ui_remove_g_INIT_2() {
    ui_g_INIT_2.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_INIT_2);
    SEND_MESSAGE((uint8_t *) &ui_g_INIT_2, sizeof(ui_g_INIT_2));
}

// ===== INIT 子帧 3："SHOOT NUM" 标签 ===================================
ui_string_frame_t ui_g_INIT_3;
ui_interface_string_t* ui_g_INIT_SHOOT_NUM = &(ui_g_INIT_3.option);

/** @brief INIT 子帧 3 初始化 "SHOOT NUM" 标签字 */
void _ui_init_g_INIT_3() {
    ui_g_INIT_3.option.figure_name[0] = 0;
    ui_g_INIT_3.option.figure_name[1] = 2;
    ui_g_INIT_3.option.figure_name[2] = 7;
    ui_g_INIT_3.option.operate_type = 1;

    ui_g_INIT_SHOOT_NUM->figure_type = 7;
    ui_g_INIT_SHOOT_NUM->operate_type = 1;
    ui_g_INIT_SHOOT_NUM->layer = 0;
    ui_g_INIT_SHOOT_NUM->color = 1;
    ui_g_INIT_SHOOT_NUM->start_x = 60;
    ui_g_INIT_SHOOT_NUM->start_y = 800;
    ui_g_INIT_SHOOT_NUM->width = 2;
    ui_g_INIT_SHOOT_NUM->font_size = 20;
    ui_g_INIT_SHOOT_NUM->str_length = 9;
    strcpy(ui_g_INIT_SHOOT_NUM->string, "SHOOT NUM");


    ui_proc_string_frame(&ui_g_INIT_3);
    SEND_MESSAGE((uint8_t *) &ui_g_INIT_3, sizeof(ui_g_INIT_3));
}

/** @brief INIT 子帧 3 刷新 */
void _ui_update_g_INIT_3() {
    ui_g_INIT_3.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_INIT_3);
    SEND_MESSAGE((uint8_t *) &ui_g_INIT_3, sizeof(ui_g_INIT_3));
}

/** @brief INIT 子帧 3 删除（当前未调用） */
void _ui_remove_g_INIT_3() {
    ui_g_INIT_3.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_INIT_3);
    SEND_MESSAGE((uint8_t *) &ui_g_INIT_3, sizeof(ui_g_INIT_3));
}

// =====================================================================
//  INIT 分组聚合入口：上电时调用一次。
//  UI_Task 的 cnt%30 兜底分支也会调用 ui_update_g_INIT() 重发。
//  注：兜底分支会和当拍 cnt%10 选中的某个 30HZ 子帧叠在同一 tick 内，
//      原本 5 帧裸连发会撞 gState 复位窗口；4 个子帧间 osDelay(UI_FRAME_GAP_MS) 后
//      整组 ≈ 4×30=120ms 串行，由 UI_Task 在启动阶段独占调用，对主循环无影响。
// =====================================================================
void ui_init_g_INIT() {
    _ui_init_g_INIT_0();
    osDelay(UI_FRAME_GAP_MS);
    _ui_init_g_INIT_1();
    osDelay(UI_FRAME_GAP_MS);
    _ui_init_g_INIT_2();
    osDelay(UI_FRAME_GAP_MS);
    _ui_init_g_INIT_3();
}

void ui_update_g_INIT() {
    _ui_update_g_INIT_0();
    osDelay(UI_FRAME_GAP_MS);
    _ui_update_g_INIT_1();
    osDelay(UI_FRAME_GAP_MS);
    _ui_update_g_INIT_2();
    osDelay(UI_FRAME_GAP_MS);
    _ui_update_g_INIT_3();
}

void ui_remove_g_INIT() {
    _ui_remove_g_INIT_0();
    osDelay(UI_FRAME_GAP_MS);
    _ui_remove_g_INIT_1();
    osDelay(UI_FRAME_GAP_MS);
    _ui_remove_g_INIT_2();
    osDelay(UI_FRAME_GAP_MS);
    _ui_remove_g_INIT_3();
}

