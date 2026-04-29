#include "Trigger_about.h"

// 触发器相关函数实现
//@param trigger 触发器结构体指针
//@param key_state 当前按键状态，0表示未按下，1表示按下
//@param trigger_count 触发次数阈值，达到该值后触发器锁定
//@return 1表示触发事件，0表示未触发
uint8_t easy_KeyProtectSingleTrigger_Update(KeyProtectSingleTrigger_Def *trigger, uint16_t key_state, uint16_t trigger_count)
{
    if (key_state != 0 && trigger->trigger_state == 0) 
    {
        // 按键从未按下变为按下，触发事件
        trigger->trigger_cnt++; // 触发次数加1
        if(trigger->trigger_cnt >= trigger_count)
        {
            trigger->trigger_state = 1; // 锁定状态
        }
        else 
        {
            trigger->trigger_state = 0; // 解锁
        }
        return 1;
    } 
    else if (key_state == 0) 
    {
        // 按键未按下，重置状态
        trigger->trigger_state = 0; // 解锁状态
        trigger->trigger_cnt = 0; // 重置触发次数
        return 0;
    }
    else 
    {
        return 0;
    }
}