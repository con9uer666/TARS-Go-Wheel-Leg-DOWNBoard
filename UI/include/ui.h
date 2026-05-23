//
// Created by RM UI Designer
// Static Edition
//
// ============================================================================
//  ui.h —— UI 模块对业务层的统一入口
//  ----------------------------------------------------------------------------
//  把 ui_g 的"按刷新频率分组的 init/update/remove"接口聚合一层，让
//  UI_Task.c 只需要 #include "ui.h"。当前工程实现里 ui_init_g_*
//  函数其实就是直接定义在 ui_g.c 中，本头只做声明聚合 + 转发 ui_g.h。
// ============================================================================

#ifndef UI_H
#define UI_H
#ifdef __cplusplus
extern "C" {
#endif

#include "ui_interface.h"

#include "ui_g.h"

// 30Hz 组：步兵主屏所有动态图元（电机心跳、姿态、转速、车头朝向、腿长…）。
//          init 发"新增"帧给客户端注册图元；update 持续刷"修改"；remove 发"删除"。
void ui_init_g_30HZ();
void ui_update_g_30HZ();
void ui_remove_g_30HZ();

// 5Hz 组：变化频率低的文本提示（"PLEASE SPIN" / "LONG LEG"）。
void ui_init_g_5HZ();
void ui_update_g_5HZ();
void ui_remove_g_5HZ();

// INIT 组：永不变化的静态层（背景线条、固定文字标签）。
//          上电发一次，UI_Task 每 3s 兜底重发，防客户端丢包后图元消失。
void ui_init_g_INIT();
void ui_update_g_INIT();
void ui_remove_g_INIT();

#ifdef __cplusplus
}
#endif

#endif // UI_H
