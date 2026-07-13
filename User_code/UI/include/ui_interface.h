//
// Created by bismarckkk on 2024/2/17.
//
// ============================================================================
//  ui_interface.h —— UI 模块"协议层"对外头
//  ----------------------------------------------------------------------------
//  对外暴露：
//    1. ui_self_id   : 当前机器人 ID（由 UI_task 上电时从裁判系统读取覆盖）
//    2. ui_send_message / SEND_MESSAGE 宏 : 串口 DMA 发送入口
//    3. ui_proc_{1,2,5,7,string}_frame    : 帧封口函数，填头/算 CRC
//
//  这一层不关心"画什么"，只关心"如何按裁判系统协议把数据搬出去"。
// ============================================================================

#ifndef UI_INTERFACE_H
#define UI_INTERFACE_H

#include <stdio.h>
#include "ui_types.h"

// 当前机器人 id（红 1~7 / 蓝 101~107），UI_task 上电时通过 JUDGE_GetSelfID() 覆盖。
// 默认值 3 = 红方步兵 3，仅当读取裁判系统失败时作为兜底。
extern int ui_self_id;

// 调试用：以 hex 形式 dump 一段消息到 printf 流。
void print_message(const uint8_t* message, int length);

// User Code Begin
// 串口 DMA 发送入口（实现里做了 DMA 通道空闲查询 + 100ms 看门狗 + osDelay 让位）。
// SEND_MESSAGE 宏是 ui_g.c 自动生成代码用的统一调用名，便于以后切换通道。
void ui_send_message(const uint8_t *message, uint16_t length);
#define SEND_MESSAGE(message, length) ui_send_message((message), (length))
// User Code End

// 帧封口函数：调用前 ui_g.c 已经填好 data[] 与 header.length 之外的字段，
// 这些函数负责打 SOF/length/seq/cmd_id/sub_id/send_id/recv_id/CRC8/CRC16。
void ui_proc_1_frame(ui_1_frame_t *msg);      // sub_id=0x0101
void ui_proc_2_frame(ui_2_frame_t *msg);      // sub_id=0x0102
void ui_proc_5_frame(ui_5_frame_t *msg);      // sub_id=0x0103
void ui_proc_7_frame(ui_7_frame_t *msg);      // sub_id=0x0104
void ui_proc_string_frame(ui_string_frame_t *msg); // sub_id=0x0110

#endif //UI_INTERFACE_H
