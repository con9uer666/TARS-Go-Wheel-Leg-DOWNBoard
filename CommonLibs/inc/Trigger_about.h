#ifndef TRIGGER_ABOUT_H
#define TRIGGER_ABOUT_H

#include "main.h"

typedef struct 
{
    uint8_t trigger_state; // 按键状态，0表示未锁定，1表示锁定
    uint16_t trigger_cnt; // 触发次数
}KeyProtectSingleTrigger_Def;

uint8_t easy_KeyProtectSingleTrigger_Update(KeyProtectSingleTrigger_Def *trigger, uint16_t key_state, uint16_t trigger_count);

#endif