#include "buzzer.h"
#include "tim.h"
#include "main.h"
#include "remoter.h"


void Buzzer_init()
{
    HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,0);
}

void Set_buzzer_frq(int frq)
{
    uint16_t Prescaler = (960000 / frq) - 1;
    __HAL_TIM_SET_PRESCALER(&htim12, Prescaler);
    HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_2);
}

void Run_Buzzer()
{
    __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,(uint16_t)(((SBUS_CH.CH10 - 192)/1600.0f) * 125));
}

void Stop_Buzzer()
{
    __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,0);
}

void Buzzer_do()
{
    Stop_Buzzer();
    Set_buzzer_frq(523);
    Run_Buzzer();
}

void Buzzer_re()
{
    Stop_Buzzer();
    Set_buzzer_frq(587);
    Run_Buzzer();
}

void Buzzer_mi()
{
    Stop_Buzzer();
    Set_buzzer_frq(659);
    Run_Buzzer();
}

void Buzzer_fa()
{
    Stop_Buzzer();
    Set_buzzer_frq(698);
    Run_Buzzer();
}

void Buzzer_sol()
{
    Stop_Buzzer();
    Set_buzzer_frq(784);
    Run_Buzzer();
}

void Buzzer_la()
{
    Stop_Buzzer();
    Set_buzzer_frq(880);
    Run_Buzzer();
}

void Buzzer_si()
{
    Stop_Buzzer();
    Set_buzzer_frq(988);
    Run_Buzzer();
}

void Buzzer_High_do()
{
    Stop_Buzzer();
    Set_buzzer_frq(1046);
    Run_Buzzer();
}

void Buzzer_High_re()
{
    Stop_Buzzer();
    Set_buzzer_frq(1175);
    Run_Buzzer();
}

void Buzzer_High_mi()
{
    Stop_Buzzer();
    Set_buzzer_frq(1318);
    Run_Buzzer();
}

void Buzzer_High_fa()
{
    Stop_Buzzer();
    Set_buzzer_frq(1397);
    Run_Buzzer();
}

void Buzzer_High_sol()
{
    Stop_Buzzer();
    Set_buzzer_frq(1568);
    Run_Buzzer();
}

void Buzzer_High_la()
{
    Stop_Buzzer();
    Set_buzzer_frq(1760);
    Run_Buzzer();
}

void Buzzer_High_si()
{
    Stop_Buzzer();
    Set_buzzer_frq(1976);
    Run_Buzzer();
}
