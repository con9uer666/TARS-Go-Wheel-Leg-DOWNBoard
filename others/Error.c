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
			WS2812_1_Set(0,100,0,0);
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
				HAL_NVIC_SystemReset();
			}
			WS2812_1_Set(0,0,0,0);
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
