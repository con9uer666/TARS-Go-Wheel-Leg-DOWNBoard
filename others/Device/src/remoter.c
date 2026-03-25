#include "remoter.h"
#include <main.h>
#include "usart.h"
#include "string.h"
#include "cmsis_os.h"
#include "state.h"


extern uint8_t Rx_Data[BUFF_SIZE];
extern uint8_t usart2RxBuf[256];

extern osThreadId defaultTaskHandle;
extern osThreadId IMUHandle;
extern osThreadId CANHandle;
extern osThreadId LEDHandle;
extern osThreadId ErrorHandle;

extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;

int Remoter_disconnected_times;
uint8_t Error_state = 0;

SBUS_CH_Struct SBUS_CH;

void Sbus_Data_Count(uint8_t *buf)
{
    if (buf[23] == 0 && buf[0] == 0x0F)
    {
		Remoter_disconnected_times = 0;
        SBUS_CH.ConnectState = 1;
        SBUS_CH.CH1 = ((int16_t)buf[ 1] >> 0 | ((int16_t)buf[ 2] << 8 )) & 0x07FF;
        SBUS_CH.CH2 = ((int16_t)buf[ 2] >> 3 | ((int16_t)buf[ 3] << 5 )) & 0x07FF;
        SBUS_CH.CH3 = ((int16_t)buf[ 3] >> 6 | ((int16_t)buf[ 4] << 2 ) | (int16_t)buf[ 5] << 10 ) & 0x07FF;
        SBUS_CH.CH4 = ((int16_t)buf[ 5] >> 1 | ((int16_t)buf[ 6] << 7 )) & 0x07FF;
        SBUS_CH.CH5 = ((int16_t)buf[ 6] >> 4 | ((int16_t)buf[ 7] << 4 )) & 0x07FF;
        SBUS_CH.CH6 = ((int16_t)buf[ 7] >> 7 | ((int16_t)buf[ 8] << 1 ) | (int16_t)buf[9] << 9 ) & 0x07FF;
        SBUS_CH.CH7 = ((int16_t)buf[ 9] >> 2 | ((int16_t)buf[10] << 6 )) & 0x07FF;
        SBUS_CH.CH8 = ((int16_t)buf[10] >> 5 | ((int16_t)buf[11] << 3 )) & 0x07FF;
        SBUS_CH.CH9 = ((int16_t)buf[12] << 0 | ((int16_t)buf[13] << 8 )) & 0x07FF;
        SBUS_CH.CH10 = ((int16_t)buf[13] >> 3 | ((int16_t)buf[14] << 5 )) & 0x07FF;
        SBUS_CH.CH11 = ((int16_t)buf[14] >> 6 | ((int16_t)buf[15] << 2 ) | (int16_t)buf[16] << 10 ) & 0x07FF;
        SBUS_CH.CH12 = ((int16_t)buf[16] >> 1 | ((int16_t)buf[17] << 7 )) & 0x07FF;
        SBUS_CH.CH13 = ((int16_t)buf[17] >> 4 | ((int16_t)buf[18] << 4 )) & 0x07FF;
        SBUS_CH.CH14 = ((int16_t)buf[18] >> 7 | ((int16_t)buf[19] << 1 ) | (int16_t)buf[20] << 9 ) & 0x07FF;
        SBUS_CH.CH15 = ((int16_t)buf[20] >> 2 | ((int16_t)buf[21] << 6 )) & 0x07FF;
        SBUS_CH.CH16 = ((int16_t)buf[21] >> 5 | ((int16_t)buf[22] << 3 )) & 0x07FF;
    }
    else
    {
		Remoter_disconnected_times ++;
		if(Remoter_disconnected_times >= 50)
      	SBUS_CH.ConnectState = 0;
    }
		
	if(SBUS_CH.CH5 > 1700)SBUS_CH.SW1 = 2;
	else if(SBUS_CH.CH5 < 200)SBUS_CH.SW1 = 0;
	else SBUS_CH.SW1 = 1;
	
	if(SBUS_CH.CH6 > 1700)SBUS_CH.SW2 = 1;
	else if(SBUS_CH.CH6 < 200)SBUS_CH.SW2 = 0;
	else SBUS_CH.SW2 = 0;
	
	if(Error_state == 0 || SBUS_CH.ConnectState == 1)
	{
		if(SBUS_CH.CH7 > 1700)SBUS_CH.SW3 = 1;
		else if(SBUS_CH.CH7 < 200)SBUS_CH.SW3 = 0;
		else SBUS_CH.SW3 = 0;
	}
	
	if(SBUS_CH.CH8 > 1700)SBUS_CH.SW4 = 2;
	else if(SBUS_CH.CH8 < 200)SBUS_CH.SW4 = 0;
	else SBUS_CH.SW4 = 1;


}

void Error_Judge()
{
	if(SBUS_CH.SW3 == 0 || Remoter_disconnected_times >= 50)
	{
		SBUS_CH.SW3 = 0;
		Remoter_disconnected_times = 0;
		if(Error_state != 1)xTaskResumeFromISR(ErrorHandle);
		Error_state = 1;
		System_State = Error;
	}
}

void Remoter_Init()
{
		SBUS_CH.CH1 = 992;
		SBUS_CH.CH2 = 992;
		SBUS_CH.CH3 = 992;
		SBUS_CH.CH4 = 992;
		SBUS_CH.CH5 = 992;
		SBUS_CH.CH6 = 192;
		SBUS_CH.CH7 = 1792;
		SBUS_CH.CH8 = 992;
		SBUS_CH.CH9 = 192;
		SBUS_CH.CH10 = 992;
		SBUS_CH.SW3 = 1;
	
		Error_state = 0;
		
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, Rx_Data, BUFF_SIZE);
		__HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);
}

//下板单独调试遥控器接收
// void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
// {

// 	if(huart->Instance == UART5)
// 	{
// 			// 接收完毕后重启
// 			Sbus_Data_Count(Rx_Data);
// 			Error_Judge();
// 			HAL_UARTEx_ReceiveToIdle_DMA(&huart5, Rx_Data, BUFF_SIZE);
// 	}
// }

int a;

void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
	if(huart->Instance == UART5)
	{
		__HAL_UNLOCK(huart);
		memset(Rx_Data, 0, BUFF_SIZE);							   // 清除接收缓存		
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, Rx_Data, BUFF_SIZE);// 接收发生错误后重启
	}
	if (huart == &huart2)
    {
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2,usart2RxBuf,sizeof(usart2RxBuf));
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx,DMA_IT_HT);
		a ++;
	}
}
