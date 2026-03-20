#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "remoter.h"
#include "LED.h"
// #include "core_cm7.h"
// #include "cmsis_armcc.h"
#include "buzzer.h"
#include "USER_CAN.h"
#include "fdcan.h"
#include "motor.h"
#include "Board2Board.h"

extern SBUS_CH_Struct SBUS_CH;
uint8_t Rx_Data[BUFF_SIZE];
extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;

extern uint8_t usart2RxBuf[256];

/**
主要功能：

1.初始化和挂起任务：

调用 Remoter_Init() 初始化遥控器。
使用 vTaskSuspend(NULL) 挂起当前任务。

2.禁用电机：

调用 Disable_DM_Motor 禁用多个电机（通过 hfdcan1 和 hfdcan2 总线）。
调用 DJI_Motor_Torque_Ctrl 设置电机扭矩为 0。

3.蜂鸣器提示：

调用 Buzzer_High_si 和 Stop_Buzzer 控制蜂鸣器发出提示音。

4. LED 控制：

使用 WS2812_1_Set 设置 LED 的颜色。

5.错误状态循环：

在无限循环中重复禁用电机、设置 LED 颜色。
检查遥控器的 SW3 开关状态，如果为 1，则调用 HAL_NVIC_SystemReset 重启系统。

6.DMA 接收：

使用 HAL_UARTEx_ReceiveToIdle_DMA 和 __HAL_DMA_DISABLE_IT 配置 UART 的 DMA 接收。
这个任务的主要目的是在错误状态下执行一系列安全操作（如禁用电机、发出警告提示、重启系统等），以确保设备的安全性和稳定性。
 */
void Error_task(void const * argument)
{																				
		Remoter_Init();																
    	vTaskSuspend(NULL);									
		Disable_DM_Motor(&hfdcan2, 0x01);					
		Disable_DM_Motor(&hfdcan1, 0x01);					
		Disable_DM_Motor(&hfdcan3, 0x11);					
		HAL_Delay(1);												
		Disable_DM_Motor(&hfdcan2, 0x02);					
		Disable_DM_Motor(&hfdcan1, 0x02);					
		Disable_DM_Motor(&hfdcan3, 0x10);					
		HAL_Delay(1);												
		DJI_Motor_Torque_Ctrl(&hfdcan2, 0x200, 0);				
		DJI_Motor_Torque_Ctrl(&hfdcan1, 0x1FF, 0);				
		Buzzer_High_si();																			
		HAL_Delay(100);																			
		Stop_Buzzer();																			
		HAL_Delay(50);																				
		Buzzer_High_si();																			
		HAL_Delay(100);																			
		Stop_Buzzer();																			
		HAL_Delay(50);																																																								


    for(;;)
    {																																										
			WS2812_1_Set(0,100,0,0);//红绿灯		
			HAL_Delay(100);								
			Disable_DM_Motor(&hfdcan2, 0x01);	
			Disable_DM_Motor(&hfdcan1, 0x01);	
			Disable_DM_Motor(&hfdcan3, 0x11);	
			HAL_Delay(1);								
			Disable_DM_Motor(&hfdcan2, 0x02);	
			Disable_DM_Motor(&hfdcan1, 0x02);	
			Disable_DM_Motor(&hfdcan3, 0x10);	
			HAL_Delay(1);								
			DJI_Motor_Torque_Ctrl(&hfdcan2, 0x200, 0);																																		
			DJI_Motor_Torque_Ctrl(&hfdcan1, 0x1FF, 0);																																		
			if(STOPFLAG != 1)																						
			{																															
				__set_FAULTMASK(1);//禁止所有的可屏蔽中断																																					
				HAL_NVIC_SystemReset();//重启系统																													
			}																															
			WS2812_1_Set(0,0,0,0);//灭灯																											
			HAL_Delay(100);																												
			Disable_DM_Motor(&hfdcan2, 0x01);																												
			Disable_DM_Motor(&hfdcan1, 0x01);																												
			Disable_DM_Motor(&hfdcan3, 0x11);																												
			HAL_Delay(1);																												
			Disable_DM_Motor(&hfdcan2, 0x02);																												
			Disable_DM_Motor(&hfdcan1, 0x02);																														
			Disable_DM_Motor(&hfdcan3, 0x10);																												
			HAL_Delay(1);																														
			// Disable_LK_Motor(&hfdcan2, 0x141);											
			// Disable_LK_Motor(&hfdcan1, 0x141);																										
			DJI_Motor_Torque_Ctrl(&hfdcan2, 0x200, 0);																										
			DJI_Motor_Torque_Ctrl(&hfdcan1, 0x1FF, 0);																										
			if(STOPFLAG != 1)																							
			{																							
				__set_FAULTMASK(1);//禁止所有的可屏蔽中断																													
				HAL_NVIC_SystemReset();																							
			}																																		
			HAL_UARTEx_ReceiveToIdle_DMA(&huart2,usart2RxBuf,sizeof(usart2RxBuf));																																								
			__HAL_DMA_DISABLE_IT(&hdma_usart2_rx,DMA_IT_HT);																															
    }																														
}																																										
