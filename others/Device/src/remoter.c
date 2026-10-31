#include "remoter.h"
#include <main.h>
#include "usart.h"
#include "string.h"
#include "cmsis_os.h"
#include "state.h"
#include "USER_CAN.h"


extern uint8_t Rx_Data[BUFF_SIZE];
extern uint8_t usart2RxBuf[256];

extern osThreadId defaultTaskHandle;
extern osThreadId IMUHandle;
extern osThreadId CANHandle;
extern osThreadId LEDHandle;
extern osThreadId ErrorHandle;

extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;

extern uint8_t STOPFLAG;

int Remoter_disconnected_times;
uint8_t Error_state = 0;
static uint32_t remoter_last_valid_tick = 0;
static uint8_t remoter_heartbeat_lost = 1;

#define REMOTER_HEARTBEAT_TIMEOUT_MS 100U

SBUS_CH_Struct SBUS_CH;

void Sbus_Data_Count(uint8_t *buf)
{
    if (buf[23] == 0 && buf[0] == 0x0F)
    {
		Remoter_disconnected_times = 0;
        remoter_last_valid_tick = HAL_GetTick();
        remoter_heartbeat_lost = 0;
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
	if(SBUS_CH.SW3 == 0 || remoter_heartbeat_lost)
	{
		SBUS_CH.SW3 = 0;
		STOPFLAG = 1;
		motor_output_enable = 0;
		Error_state = 1;
		System_State = Error;
	}
	else if (Error_state == 1 && SBUS_CH.SW3 == 1)
	{
		STOPFLAG = 0;
		motor_output_enable = 1;
		Error_state = 0;
		System_State = Normal;
	}
}

// 遥控器掉线检测
void Remoter_Heartbeat_Tick(void)
{
	if ((HAL_GetTick() - remoter_last_valid_tick) >= REMOTER_HEARTBEAT_TIMEOUT_MS)
	{
		remoter_heartbeat_lost = 1;
		SBUS_CH.ConnectState = 0;
	}
	Error_Judge();
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
		remoter_last_valid_tick = HAL_GetTick();
		remoter_heartbeat_lost = 1;
		SBUS_CH.ConnectState = 0;
		
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, Rx_Data, BUFF_SIZE);
		__HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);
}

// °嵥ң
// void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
// {

// 	if(huart->Instance == UART5)
// 	{
// // Ϻ
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
		memset(Rx_Data, 0, BUFF_SIZE);							   // 注释已根据代码语义重写。
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, Rx_Data, BUFF_SIZE);// 注释已根据代码语义重写。
	}
	if (huart == &huart2)
    {
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2,usart2RxBuf,sizeof(usart2RxBuf));
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx,DMA_IT_HT);
		a ++;
	}
}
