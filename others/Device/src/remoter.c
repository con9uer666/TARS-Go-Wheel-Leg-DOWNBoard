#include "remoter.h"
#include <main.h>
#include "usart.h"
#include "string.h"
#include "cmsis_os.h"
#include "state.h"
#include "USER_CAN.h"
#include "motor.h"
#include "Board2Board.h"


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

uint8_t Error_state = 0;
static uint32_t remoter_last_valid_tick = 0;
static uint8_t remoter_heartbeat_lost = 1;

#define REMOTER_HEARTBEAT_TIMEOUT_MS 100U

SBUS_CH_Struct SBUS_CH;

DR16_Data_t DR16_Data;
static uint8_t requested_long_leg = 0;

void Remoter_Receive(const uint8_t *buf, uint16_t size)
{
    uint8_t was_connected = !remoter_heartbeat_lost &&
        (HAL_GetTick() - remoter_last_valid_tick < REMOTER_HEARTBEAT_TIMEOUT_MS);
    if (!DR16_Parse(buf, size, &DR16_Data))
        return;

    remoter_last_valid_tick = HAL_GetTick();
    remoter_heartbeat_lost = 0;
    SBUS_CH.ConnectState = 1;
    /* Keep the existing control scale: center 992, full travel +/-800. */
    SBUS_CH.CH1 = 992 + (int32_t)DR16_Data.ch[0] * 800 / 660;
    SBUS_CH.CH2 = 992 + (int32_t)DR16_Data.ch[1] * 800 / 660;
    SBUS_CH.CH3 = 992 + (int32_t)DR16_Data.ch[3] * 800 / 660;
    SBUS_CH.CH4 = 992 + (int32_t)DR16_Data.ch[2] * 800 / 660;
    SBUS_CH.SW1 = DR16_Data.left == DR16_SWITCH_UP ? 2 :
                  DR16_Data.left == DR16_SWITCH_DOWN ? 0 : 1;
    SBUS_CH.SW3 = DR16_Data.right != DR16_SWITCH_DOWN;
    Error_Judge();
    sit_mode_enable = !Error_state && DR16_Data.right == DR16_SWITCH_UP;
    if (!Error_state && was_connected && DR16_Data.left != DR16_Data.left_last)
    {
        if (DR16_Data.left == DR16_SWITCH_UP)
        {
            requested_long_leg ^= 1U;
            if (!requested_long_leg)
                leg_state_locked_short = 0;
        }
        else if (DR16_Data.left == DR16_SWITCH_DOWN && !sit_mode_enable)
        {
            upstairs_flag = 1;
        }
    }
    Foot_Chassis.Target_Leg_State = leg_state_locked_short ? 0 : requested_long_leg;
    Foot_Chassis.Chassis_Mode = 0;
    spinning_enable = 0;
    jump_enable = 0;
    jump_cmd = 0;
}

void Error_Judge()
{
	if(SBUS_CH.SW3 == 0 || remoter_heartbeat_lost)
	{
		SBUS_CH.SW3 = 0;
		STOPFLAG = 1;
		upstairs_flag = 0;
		sit_mode_enable = 0;
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
		SBUS_CH.SW3 = 0;
	
		Error_state = 0;
		remoter_last_valid_tick = HAL_GetTick();
		remoter_heartbeat_lost = 1;
		SBUS_CH.ConnectState = 0;
		memset(&DR16_Data, 0, sizeof(DR16_Data));
		requested_long_leg = 0;
		Foot_Chassis.Target_Leg_State = 0;
		Error_Judge();
		
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
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, Rx_Data, BUFF_SIZE);
		__HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);// 注释已根据代码语义重写。
	}
	if (huart == &huart2)
    {
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2,usart2RxBuf,sizeof(usart2RxBuf));
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx,DMA_IT_HT);
		a ++;
	}
}
