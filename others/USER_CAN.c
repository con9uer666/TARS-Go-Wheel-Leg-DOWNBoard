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

uint16_t can_send_error,can_receive_error;

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
				break;
			}
			case 0x12:
			{
				DM8009_Get_Data(rx_data, &R_DM8009[1]);
				break;
			}
			// case 0x141:
			// {
			// 	LK9025_Get_Data(rx_data, &R_LK9025);
			// 	break;
			// }
			case 0x205:
			{
				DJI3508_Get_Data(rx_data, &R_LK9025);
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
				break;
			}
			case 0x12:
			{
				DM8009_Get_Data(rx_data, &L_DM8009[1]);
				break;
			}
			// case 0x141:
			// {
			// 	LK9025_Get_Data(rx_data, &L_LK9025);
			// 	break;
			// }
			case 0x203:
			{
				DJI3508_Get_Data(rx_data, &L_LK9025);
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
				
				break;
			}
			case 0x01:
			{
				DM8009_Get_Data(rx_data, &Shooter_DM2325);
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
void CAN_Transmit(void const * argument)
{
	osDelay(2500);
    for(;;)
    {
		DJI_Motor_Torque_Ctrl(&hfdcan2, 0x200, -L_LK9025.Target_Torque);
        DJI_Motor_Torque_Ctrl(&hfdcan1, 0x1FF, R_LK9025.Target_Torque);
        osDelay(1);
		DM_Motor_MIT_Torque_ctrl(&hfdcan2, L_DM8009[1], VMC_L.T1);
        DM_Motor_MIT_Torque_ctrl(&hfdcan1, R_DM8009[1], VMC_R.T2);
        DM_Motor_MIT_Torque_ctrl(&hfdcan2, L_DM8009[0], VMC_L.T2);
        DM_Motor_MIT_Torque_ctrl(&hfdcan1, R_DM8009[0], VMC_R.T1);
		osDelay(1);
		DM_Motor_MIT_Torque_ctrl(&hfdcan3, Yaw_DM4310, Yaw_DM4310.Target_Torque);
        DM_Motor_MIT_Torque_ctrl(&hfdcan3, Shooter_DM2325, Shooter_DM2325.Target_Torque);
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
