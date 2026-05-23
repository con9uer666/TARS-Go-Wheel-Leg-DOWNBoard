//
// Created by RM UI Designer
// Static Edition
//
// ============================================================================
//  ui_g.h —— UI 模块"图层层"对外接口
//  ----------------------------------------------------------------------------
//  本文件由 RM UI Designer 自动生成（Static Edition），列出每个图元的
//  全局指针。指针指向 ui_g.c 内部的帧缓冲，修改指针所指字段就等于改
//  下一次发送时该图元的属性。
//
//  命名规则：ui_g_<分组>_<图元名>
//    分组 = 30HZ / 5HZ / INIT （仅命名分组，实际刷新频率见 UI_Task.c）
//    图元名 = Designer 中给图元起的标识
// ============================================================================

#ifndef UI_g_H
#define UI_g_H

#include "ui_interface.h"

// ============== 30HZ 分组 ==============
// 这一批图元按 7+7+7+5=26 个分装成 4 个子帧（3 个 7 帧 + 1 个 5 帧），
// 由 UI_Task 的 10Hz 主循环按 cnt%10 轮流发送。各图元含义见 UI_Task.c
// 中的 UI_RefreshParams_30HZ() 注释。

// 子帧 0：四个 8009 关节心跳 + NUC + 摩擦轮/自瞄/弹数 3 个数字
extern ui_interface_round_t  *ui_g_30HZ_8009LF;     // 左前 DM8009 关节断联指示圆
extern ui_interface_round_t  *ui_g_30HZ_8009LB;     // 左后 DM8009
extern ui_interface_round_t  *ui_g_30HZ_NUC;        // NUC 心跳（TODO）
extern ui_interface_number_t *ui_g_30HZ_FRIC_SPD_L; // 左摩擦轮转速显示
extern ui_interface_number_t *ui_g_30HZ_AUTO_AIM;   // 自瞄状态数字
extern ui_interface_number_t *ui_g_30HZ_SHOOT_NUM;  // 已发射弹丸数
extern ui_interface_number_t *ui_g_30HZ_FRIC_SPD_R; // 右摩擦轮转速显示

// 子帧 1：车身朝向弧 + 右前/后 8009 + 超电直线 + 双腿直线 + body_pitch 直线
extern ui_interface_arc_t    *ui_g_30HZ_BODY_FRONT; // 车身朝向（云台头坐标系下的弧）
extern ui_interface_round_t  *ui_g_30HZ_8009RF;     // 右前 8009
extern ui_interface_line_t   *ui_g_30HZ_SUPER_CUP;  // 超级电容剩余电量条
extern ui_interface_line_t   *ui_g_30HZ_L_LEG;      // 左腿可视化
extern ui_interface_line_t   *ui_g_30HZ_R_LEG;      // 右腿可视化
extern ui_interface_line_t   *ui_g_30HZ_BODY_PITCH; // 车身 pitch 角直线
extern ui_interface_round_t  *ui_g_30HZ_8009RB;     // 右后 8009

// 子帧 2：缓冲能量数 + 功率/485/UNNAMEx 心跳灯 + 左 3508
extern ui_interface_number_t *ui_g_30HZ_BUFFER_NUM; // 缓冲能量数字（TODO）
extern ui_interface_round_t  *ui_g_30HZ_POWER_METER;// 功率计心跳（CAN3 0x213）
extern ui_interface_round_t  *ui_g_30HZ_485;        // 485 板间通信心跳
extern ui_interface_round_t  *ui_g_30HZ_UNNAME1;    // 备用 1
extern ui_interface_round_t  *ui_g_30HZ_UNNAME2;    // 备用 2
extern ui_interface_round_t  *ui_g_30HZ_UNNAME3;    // 1Hz 闪烁灯（程序运行指示）
extern ui_interface_round_t  *ui_g_30HZ_3508L;      // 左 3508 心跳

// 子帧 3：右 3508 + PITCH/ROLL + 双摩擦轮电机心跳
extern ui_interface_round_t  *ui_g_30HZ_3508R;      // 右 3508
extern ui_interface_round_t  *ui_g_30HZ_PITCH;      // PITCH 电机心跳（TODO）
extern ui_interface_round_t  *ui_g_30HZ_ROLL;       // 4310 Yaw 电机心跳（暂复用）
extern ui_interface_round_t  *ui_g_30HZ_FRIC_L;     // 左摩擦轮电机心跳（TODO）
extern ui_interface_round_t  *ui_g_30HZ_FRIC_R;     // 右摩擦轮电机心跳（TODO）

void ui_init_g_30HZ();    // 发"新增"帧，把 4 个子帧注册到客户端
void ui_update_g_30HZ();  // 发"修改"帧，把所有 4 子帧一次刷一遍（启动期使用）
void ui_remove_g_30HZ();  // 发"删除"帧

// ============== 5HZ 分组 ==============
// 文本提示：根据车况显示/隐藏。隐藏用 str_length=0 实现，不删除图元。
extern ui_interface_string_t *ui_g_5HZ_NewText;     // "PLEASE SPIN"
extern ui_interface_string_t *ui_g_5HZ_NewText2;    // "LONG LEG"

void ui_init_g_5HZ();
void ui_update_g_5HZ();
void ui_remove_g_5HZ();

// ============== INIT 分组 ==============
// 永久静态层：装饰线条 + 标签文字。一次画完不再变，UI_Task 每 3s 重发兜底。
extern ui_interface_line_t   *ui_g_INIT_NewLine;     // 左侧斜线
extern ui_interface_line_t   *ui_g_INIT_NewLine2;    // 右侧斜线
extern ui_interface_line_t   *ui_g_INIT_NewLine3;    // 装饰横线 1
extern ui_interface_line_t   *ui_g_INIT_NewLine4;    // 装饰横线 2
extern ui_interface_line_t   *ui_g_INIT_NewLine5;    // 顶部横线
extern ui_interface_string_t *ui_g_INIT_FRIC_SPD;    // "FRIC SPD" 标签
extern ui_interface_string_t *ui_g_INIT_AUTO_AIM;    // "AUTO AIM" 标签
extern ui_interface_string_t *ui_g_INIT_SHOOT_NUM;   // "SHOOT NUM" 标签

void ui_init_g_INIT();
void ui_update_g_INIT();
void ui_remove_g_INIT();


#endif // UI_g_H
