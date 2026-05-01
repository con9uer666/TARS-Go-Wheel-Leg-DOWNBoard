#include "USER_CAN.h"
#include "fdcan.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "remoter.h"
#include "motor.h"
#include "VMC.h"
#include "arm_math.h"
#include "Motor_Drv.h"

extern uint8_t first_run;
extern uint8_t start_mode;
extern uint8_t gimbal_follow_flag;
extern float down_board_yaw_output;

uint16_t can_send_error,can_receive_error;

/*====================================== 电机输出总开关 ====================================== */
uint8_t motor_output_enable = 1;  // 1=正常输出力矩, 0=强制全部电机输出零力矩

/*====================================== 心跳检测 =========================================== */
#define HEARTBEAT_CHECK_INTERVAL_MS  10    // 每10ms检测一次

// 每个电机独立的CAN数据接收计数器（每次收到消息时在中断回调中自增）
static volatile uint32_t rx_cnt_L_DM8009_0 = 0;  // L_DM8009[0]
static volatile uint32_t rx_cnt_L_DM8009_1 = 0;  // L_DM8009[1]
static volatile uint32_t rx_cnt_R_DM8009_0 = 0;  // R_DM8009[0]
static volatile uint32_t rx_cnt_R_DM8009_1 = 0;  // R_DM8009[1]
static volatile uint32_t rx_cnt_L_DJ3508    = 0;  // L_DJ3508
static volatile uint32_t rx_cnt_R_DJ3508    = 0;  // R_DJ3508
static volatile uint32_t rx_cnt_4310        = 0;  // Yaw_DM4310
static volatile uint32_t rx_cnt_2325        = 0;  // Shooter_DM2325

// 上一次检测时保存的计数值（用于对比是否增长）
static uint32_t last_rx_cnt_L_DM8009_0 = 0;
static uint32_t last_rx_cnt_L_DM8009_1 = 0;
static uint32_t last_rx_cnt_R_DM8009_0 = 0;
static uint32_t last_rx_cnt_R_DM8009_1 = 0;
static uint32_t last_rx_cnt_L_DJ3508    = 0;
static uint32_t last_rx_cnt_R_DJ3508    = 0;
static uint32_t last_rx_cnt_4310        = 0;
static uint32_t last_rx_cnt_2325        = 0;

// 上一次执行心跳检测的时间戳
static uint32_t last_heartbeat_check_tick = 0;

// 掉线状态（被CAN_Transmit消费后保持，用于掉线→重连边沿检测）
static uint8_t lost_8009_3508 = 0;
static uint8_t lost_4310      = 0;
static uint8_t lost_2325      = 0;

void CAN_Init(void)
{
	FDCAN_FilterTypeDef filter;
	filter.IdType = FDCAN_STANDARD_ID;
	filter.FilterIndex = 0;	
	filter.FilterType = FDCAN_FILTER_MASK;
	filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	filter.FilterID1 = 0x00000000;				
	filter.FilterID2 = 0x00000000;

	HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0); 
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 1);
	HAL_FDCAN_Start(&hfdcan1);

	HAL_FDCAN_ConfigFilter(&hfdcan2, &filter);
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan2, FDCAN_CFG_RX_FIFO0, 1);
	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	HAL_FDCAN_Start(&hfdcan2);		

	HAL_FDCAN_ConfigFilter(&hfdcan3, &filter);
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan3, FDCAN_CFG_RX_FIFO0, 1);
	HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	HAL_FDCAN_Start(&hfdcan3);
}

FDCAN_RxHeaderTypeDef rx_header;
uint8_t rx_data[8];

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	HAL_StatusTypeDef if_can_get_message_ok;
		if(hfdcan == &hfdcan1)
    {
		HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);
		switch(rx_header.Identifier)
		{
			case 0x11:
			{
				DM8009_Get_Data(rx_data, &R_DM8009[0]);
				rx_cnt_R_DM8009_0++;
				break;
			}
			case 0x12:
			{
				DM8009_Get_Data(rx_data, &R_DM8009[1]);
				rx_cnt_R_DM8009_1++;
				break;
			}
			case 0x205:
			{
				DJ3508_Get_Data(rx_data, &R_DJ3508);
				rx_cnt_R_DJ3508++;
				break;
			}
		}
    }
	if(hfdcan == &hfdcan2)
    {
		HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);
		switch(rx_header.Identifier)
		{
			case 0x11:
			{
				DM8009_Get_Data(rx_data, &L_DM8009[0]);
				rx_cnt_L_DM8009_0++;
				break;
			}
			case 0x12:
			{
				DM8009_Get_Data(rx_data, &L_DM8009[1]);
				rx_cnt_L_DM8009_1++;
				break;
			}
			case 0x203:
			{
				DJ3508_Get_Data(rx_data, &L_DJ3508);
				rx_cnt_L_DJ3508++;
				break;
			}
		}
    }
	if(hfdcan == &hfdcan3)
    {
		HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);
		switch(rx_header.Identifier)
		{
			case 0x00:
			{
				DM8009_Get_Data(rx_data, &Yaw_DM4310);
				rx_cnt_4310++;
				break;
			}
			case 0x01:
			{
				DM8009_Get_Data(rx_data, &Shooter_DM2325);
				rx_cnt_2325++;
				break;
			}
		}
    }
}

/**
 * @brief 普通的电机数据发送函数，适用于所有电机
 * 
 * @param hfdcan 
 * @param StdId 
 * @param Data 
 */
void CAN_Send_DM_Motor_Data(FDCAN_HandleTypeDef *hfdcan, int16_t StdId, uint8_t *Data)
{
  	FDCAN_TxHeaderTypeDef tx_header;
	tx_header.Identifier = StdId;
	tx_header.IdType = FDCAN_STANDARD_ID;
	tx_header.TxFrameType = FDCAN_DATA_FRAME;
	tx_header.DataLength = 8;
	tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	tx_header.BitRateSwitch = FDCAN_BRS_OFF;
	tx_header.FDFormat = FDCAN_CLASSIC_CAN;
	tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	tx_header.MessageMarker = 0;

    if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, Data) != HAL_OK)
    {
        can_send_error++;
    }
}
float touqer;

uint8_t user_j = 0;

void CAN_Transmit(void const * argument)
{
	osDelay(2500);
    for(;;)
    {
		//输出开关vscode://lirentech.file-ref-tags?filePath=USER_CAN.c&snippet=%2F%2F%E8%BE%93%E5%87%BA%E5%BC%80%E5%85%B3
		
		/*========== 总开关：关闭时强制全部电机输出零力矩 ==========*/
		if (!motor_output_enable)
		{
			DJI_Motor_Torque_Ctrl(&hfdcan2, 0x200, 0);
			DJI_Motor_Torque_Ctrl(&hfdcan1, 0x1FF, 0);
			osDelay(1);
			DM_Motor_MIT_Torque_ctrl(&hfdcan2, L_DM8009[1], 0);
			DM_Motor_MIT_Torque_ctrl(&hfdcan1, R_DM8009[1], 0);
			DM_Motor_MIT_Torque_ctrl(&hfdcan2, L_DM8009[0], 0);
			DM_Motor_MIT_Torque_ctrl(&hfdcan1, R_DM8009[0], 0);
			osDelay(1);
			DM_Motor_MIT_Speed_ctrl(&hfdcan3, Yaw_DM4310, 0, 0, 0, 0, 0);
			DM_Motor_MIT_Torque_ctrl(&hfdcan3, Shooter_DM2325, 0);
			continue;
		}

		/*========== 心跳检测：每10ms执行一次，对比计数器是否增长 ==========*/
		uint32_t now = xTaskGetTickCount();
		if ((now - last_heartbeat_check_tick) >= HEARTBEAT_CHECK_INTERVAL_MS)
		{
			last_heartbeat_check_tick = now;

			// 读取当前计数值（volatile，在中断中自增）
			uint32_t cur_L_DM8009_0 = rx_cnt_L_DM8009_0;
			uint32_t cur_L_DM8009_1 = rx_cnt_L_DM8009_1;
			uint32_t cur_R_DM8009_0 = rx_cnt_R_DM8009_0;
			uint32_t cur_R_DM8009_1 = rx_cnt_R_DM8009_1;
			uint32_t cur_L_DJ3508   = rx_cnt_L_DJ3508;
			uint32_t cur_R_DJ3508   = rx_cnt_R_DJ3508;
			uint32_t cur_4310       = rx_cnt_4310;
			uint32_t cur_2325       = rx_cnt_2325;

			// 任意一个8009/3508计数器未增长 -> 该组掉线
			uint8_t any_8009_3508_lost = 
				(cur_L_DM8009_0 == last_rx_cnt_L_DM8009_0) ||
				(cur_L_DM8009_1 == last_rx_cnt_L_DM8009_1) ||
				(cur_R_DM8009_0 == last_rx_cnt_R_DM8009_0) ||
				(cur_R_DM8009_1 == last_rx_cnt_R_DM8009_1) ||
				(cur_L_DJ3508   == last_rx_cnt_L_DJ3508)   ||
				(cur_R_DJ3508   == last_rx_cnt_R_DJ3508);

			// 记录上一次的掉线状态（用于恢复边沿检测）
			uint8_t prev_lost_8009_3508 = lost_8009_3508;
			uint8_t prev_lost_4310      = lost_4310;
			uint8_t prev_lost_2325      = lost_2325;

			// 更新掉线状态
			lost_8009_3508 = any_8009_3508_lost;
			lost_4310      = (cur_4310 == last_rx_cnt_4310);
			lost_2325      = (cur_2325 == last_rx_cnt_2325);

			// 保存本次计数值用于下次对比
			last_rx_cnt_L_DM8009_0 = cur_L_DM8009_0;
			last_rx_cnt_L_DM8009_1 = cur_L_DM8009_1;
			last_rx_cnt_R_DM8009_0 = cur_R_DM8009_0;
			last_rx_cnt_R_DM8009_1 = cur_R_DM8009_1;
			last_rx_cnt_L_DJ3508   = cur_L_DJ3508;
			last_rx_cnt_R_DJ3508   = cur_R_DJ3508;
			last_rx_cnt_4310       = cur_4310;
			last_rx_cnt_2325       = cur_2325;

			/*========== 恢复检测：掉线→重连边沿处理 ==========*/
			// 8009/3508恢复
			if (prev_lost_8009_3508 && !lost_8009_3508)
			{
				start_mode = 0;
				first_run = 1;
			}
			// 4310恢复
			if (prev_lost_4310 && !lost_4310)
			{
				gimbal_follow_flag = 1;
			}
			// 2325恢复：无需特殊标志，直接恢复正常发送即可
		}

		/*========== 按优先级发送电机指令 ==========*/
		// 优先级：4310掉线 > 8009/3508掉线 > 2325掉线 > 正常
		if (lost_4310)
		{
			// 4310掉线：全部电机输出零力矩
			DJI_Motor_Torque_Ctrl(&hfdcan2, 0x200, 0);
			DJI_Motor_Torque_Ctrl(&hfdcan1, 0x1FF, 0);
			osDelay(1);
			DM_Motor_MIT_Torque_ctrl(&hfdcan2, L_DM8009[1], 0);
			DM_Motor_MIT_Torque_ctrl(&hfdcan1, R_DM8009[1], 0);
			DM_Motor_MIT_Torque_ctrl(&hfdcan2, L_DM8009[0], 0);
			DM_Motor_MIT_Torque_ctrl(&hfdcan1, R_DM8009[0], 0);
			osDelay(1);
			DM_Motor_MIT_Speed_ctrl(&hfdcan3, Yaw_DM4310, 0, 0, 0, 0, 0);
			DM_Motor_MIT_Torque_ctrl(&hfdcan3, Shooter_DM2325, 0);
		}
		else
		{
			// 4310正常，检查8009/3508
			if (lost_8009_3508)
			{
				// 8009/3508掉线：仅8009和3508输出零力矩
				DJI_Motor_Torque_Ctrl(&hfdcan2, 0x200, 0);
				DJI_Motor_Torque_Ctrl(&hfdcan1, 0x1FF, 0);
				osDelay(1);
				DM_Motor_MIT_Torque_ctrl(&hfdcan2, L_DM8009[1], 0);
				DM_Motor_MIT_Torque_ctrl(&hfdcan1, R_DM8009[1], 0);
				DM_Motor_MIT_Torque_ctrl(&hfdcan2, L_DM8009[0], 0);
				DM_Motor_MIT_Torque_ctrl(&hfdcan1, R_DM8009[0], 0);
			}
			else
			{
				// 8009/3508正常
				DJI_Motor_Torque_Ctrl(&hfdcan2, 0x200, -L_DJ3508.Target_Torque);
				DJI_Motor_Torque_Ctrl(&hfdcan1, 0x1FF, R_DJ3508.Target_Torque);
				osDelay(1);
				DM_Motor_MIT_Torque_ctrl(&hfdcan2, L_DM8009[1], VMC_L.T1);
				DM_Motor_MIT_Torque_ctrl(&hfdcan1, R_DM8009[1], VMC_R.T2);
				DM_Motor_MIT_Torque_ctrl(&hfdcan2, L_DM8009[0], VMC_L.T2);
				DM_Motor_MIT_Torque_ctrl(&hfdcan1, R_DM8009[0], VMC_R.T1);
			}
			osDelay(1);
			// 4310正常时，始终发送4310指令
			DM_Motor_MIT_Speed_ctrl(&hfdcan3, Yaw_DM4310, 0, down_board_yaw_output, 0, 0, 2);
			// 2325
			if (lost_2325)
			{
				DM_Motor_MIT_Torque_ctrl(&hfdcan3, Shooter_DM2325, 0);
			}
			else
			{
				DM_Motor_MIT_Torque_ctrl(&hfdcan3, Shooter_DM2325, Shooter_DM2325.Target_Torque);
			}
		}

        // osDelay(1);
		// DM_Motor_MIT_Torque_ctrl(&hfdcan2, L_DM8009[1], 0);
        // DM_Motor_MIT_Torque_ctrl(&hfdcan1, R_DM8009[1], 0);
        // DM_Motor_MIT_Torque_ctrl(&hfdcan2, L_DM8009[0], 0);
        // DM_Motor_MIT_Torque_ctrl(&hfdcan1, R_DM8009[0], 0);
		// DM_Motor_MIT_Torque_ctrl(&hfdcan3, Yaw_DM4310, 0);
		// DM_Motor_MIT_Torque_ctrl(&hfdcan3, Shooter_DM2325, 0);
		// osDelay(1);
		// DJI_Motor_Torque_Ctrl(&hfdcan2, 0x200, 0);
        // DJI_Motor_Torque_Ctrl(&hfdcan1, 0x1FF, 0);

    }
}
