#include "main.h"
#include "cmsis_os.h"
#include "remoter.h"
#include "state.h"
#include "buzzer.h"
#include "motor.h"
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
    for(;;)
    {
			HAL_ADC_Start(&hadc1);
            adc = HAL_ADC_GetValue(&hadc1);
			HAL_ADC_Stop(&hadc1);
			bat_voltage = ((adc/65536.0f) * 3.3f * 11.0f) - 0.4f;
            osDelay(5);
    }
}
