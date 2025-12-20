#include "LED.h"
#include "main.h"
#include "cmsis_os.h"
#include "remoter.h"
#include "buzzer.h"
#include "State.h"
 
//显存数组，长度为 灯的数量*24+复位周期
uint16_t WS2812_1_RGB_Buff[LED_1_NUM*DATA_LEN+WS2812_RST_NUM] = {0}; 

extern SBUS_CH_Struct SBUS_CH;
 
/**
 * 函数：WS2812单灯设置函数
 * 参数：num:灯的位置，R、G、B分别为三个颜色通道的亮度，最大值为255
 * 作用：单独设置每一个WS2812的颜色
***/
void WS2812_1_Set(uint16_t num,uint8_t R,uint8_t G,uint8_t B)
{
  uint32_t indexx=(num*(3*8));
  for (uint8_t i = 0;i < 8;i++)
  {
	//填充数组
	WS2812_1_RGB_Buff[indexx+i]      = (G << i) & (0x80)?WS_H:WS_L;
	WS2812_1_RGB_Buff[indexx+i + 8]  = (R << i) & (0x80)?WS_H:WS_L;
	WS2812_1_RGB_Buff[indexx+i + 16] = (B << i) & (0x80)?WS_H:WS_L;
  }
	HAL_TIM_PWM_Start_DMA(&htim3,TIM_CHANNEL_2,(uint32_t *)WS2812_1_RGB_Buff,sizeof(WS2812_1_RGB_Buff)/sizeof(uint16_t)); 
}

 
//WS2812初始化函数
void WS2812_1_Init(void)
{
	//设置关闭所有灯
	WS2812_1_Set(0,0,0,100);
  //作用：调用DMA将显存中的内容实时搬运至定时器的比较寄存器
  HAL_TIM_PWM_Start_DMA(&htim3,TIM_CHANNEL_2,(uint32_t *)WS2812_1_RGB_Buff,sizeof(WS2812_1_RGB_Buff)/sizeof(uint16_t)); 
}

void Run_Remind(void)
{
		Buzzer_init();
		Buzzer_do();
		WS2812_1_Set(0,50,0,0);
		osDelay(160);
		Stop_Buzzer();
		osDelay(10);
		Buzzer_fa();
		WS2812_1_Set(0,50,50,0);
		osDelay(150);
		Stop_Buzzer();
		osDelay(10);
		Buzzer_la();
		WS2812_1_Set(0,0,50,50);
		osDelay(150);
		Stop_Buzzer();
		osDelay(10);
		Buzzer_High_do();
		WS2812_1_Set(0,50,0,50);
		osDelay(150);
		Stop_Buzzer();
		WS2812_1_Set(0,0,0,0);
}


void LED_task(void const * argument)
{
  	WS2812_1_Init();
	WS2812_1_Set(0,0,0,0);
	Run_Remind();
	
	for(;;)
	{
		// Stop_Buzzer();
		if(bat_voltage >= 15.0)
		{
			if(System_State == Normal)
			{
				WS2812_1_Set(0,0,50,0);
				osDelay(500);
				WS2812_1_Set(0,0,0,0);
				osDelay(500);
			}
			else if(System_State == Fast)
			{
				WS2812_1_Set(0,50,50,0);
				osDelay(500);
				WS2812_1_Set(0,0,0,0);
				osDelay(500);
			}
		}
		else
		{
			Buzzer_High_si();
			WS2812_1_Set(0,50,0,0);
		}
	}
}
