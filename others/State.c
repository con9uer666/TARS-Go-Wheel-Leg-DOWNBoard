#include "main.h"
#include "cmsis_os.h"
#include "remoter.h"
#include "state.h"
#include "buzzer.h"
#include "chassis_behavior_tree.h"
#include "adc.h"

enum State System_State;
enum State System_Last_State;

uint8_t State_changed = 0;

void State_Init()
{
    System_State = Normal;
}

void Change_State()
{
    if(SBUS_CH.SW1 == 0 && System_State > Normal)
    {
        System_State--;
        State_changed = 1;
    }
    else if(SBUS_CH.SW1 == 2 && System_State < Fast)
    {
        System_State++;
        State_changed = 1;
    }
    else
    {
        State_changed = 0;
    }
}

int adc;
float bat_voltage;

void State_machine_task(void const * argument)
{
	State_Init();
	int i = 0;
	bat_voltage = 24.0f;   // 已关闭电压检测：给一个安全默认值，避免下游误判低压
    for(;;)
    {
			// 已删除 ADC 电压采样：单板调试时电池未接，浮空读值会触发低压报警
			osDelay(100);
    }
}
