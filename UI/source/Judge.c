#include "Judge.h"
#include "string.h"
#include "cRc.h"
#include "usart.h"
#include "Detect.h"
#include "Graphics.h"
#include "userfreertos.h"
#include "usart.h"
#include "Board2Board.h"
#include "myQueue.h"
#define EN_JUDGE_TASK

void JUDGE_GraphTest_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);

/*****************ϵͳ���ݶ���**********************/
ext_game_status_t GameState;						   // 0x0001
ext_game_result_t GameResult;						   // 0x0002
ext_game_robot_HP_t GameRobotHP;					   // 0x0003
ext_event_data_t EventData;							   // 0x0101
ext_referee_warning_t RefereeWarning;				   // 0x0104
ext_dart_info_t DartRemainingTime;					   // 0x0105
ext_game_robot_status_t GameRobotStat;				   // 0x0201
ext_power_heat_data_t PowerHeatData;				   // 0x0202
ext_game_robot_pos_t GameRobotPos;					   // 0x0203
ext_buff_musk_t BuffMusk;							   // 0x0204
ext_robot_hurt_t RobotHurt;							   // 0x0206
ext_shoot_data_t ShootData;							   // 0x0207
ext_bullet_remaining_t BulletRemaining;				   // 0x0208
ext_rfid_status_t RfidStatus;						   // 0x0209
ext_dart_client_cmd_t DartClientCmd;				   // 0x020A
sentry_info_t    SentryDecision;			//0x20D

xFrameHeader FrameHeader; // ����֡ͷ��Ϣ
/****************************************************/
bool Judge_Data_TF = FALSE; // ���������Ƿ����,������������

// ���Ͷ���
Queue judgeQueue = EMPTY_QUEUE;
// ���Ͷ������ݱ�����
JudgeTxFrame judgeQueueBuf[JUDGE_QUEUE_SIZE];
extern Line line;
extern Rect rect;
extern Circle circle;
extern Oval oval;
extern Arc arc;
extern Text text;
extern FloatShape floatShape;
extern IntShape intShape;

// ���ڽ��ջ�����
uint8_t usart1RxBuf[JUDGE_MAX_RX_LENGTH];

uint16_t shootNum = 0; // ͳ�Ʒ�����

/**************����ϵͳ���ݸ���****************/

/**
 * @brief  ��ȡ��������,�ж��ж�ȡ��֤�ٶ�
 * @param  ��������
 * @retval �Ƿ�������ж�������
 * @attention  �ڴ��ж�֡ͷ��CRCУ��,������д�����ݣ����ظ��ж�֡ͷ
 */

 extern int aaaa;
bool JUDGE_Read_Data(uint8_t *ReadFromUsart)
{
	bool retval_tf = FALSE; // ������ȷ����־,ÿ�ε��ö�ȡ����ϵͳ���ݺ�������Ĭ��Ϊ����

	uint16_t judge_length; // ͳ��һ֡���ݳ���
	int CmdID = 0;		   // �������������

	/***------------------*****/
	// �����ݰ��������κδ���
	if (ReadFromUsart == NULL)
	{
		return -1;
	}
	// д��֡ͷ����,�����ж��Ƿ�ʼ�洢��������
	memcpy(&FrameHeader, ReadFromUsart, LEN_HEADER);

	// �ж�֡ͷ�����Ƿ�Ϊ0xA5
	if (ReadFromUsart[SOF] == JUDGE_FRAME_HEADER)
	{
		// ֡ͷCRC8У��
		if (Verify_CRC8_Check_Sum(ReadFromUsart, LEN_HEADER) == TRUE)
		{
			// ͳ��һ֡���ݳ���,����CR16У��
			judge_length = ReadFromUsart[DATA_LENGTH] + LEN_HEADER + LEN_CMDID + LEN_TAIL;
			;

			// ֡βCRC16У��
			if (Verify_CRC16_Check_Sum(ReadFromUsart, judge_length) == TRUE)
			{
				retval_tf = TRUE; // ��У�������˵�����ݿ���
				aaaa ++;
				CmdID = (ReadFromUsart[6] << 8 | ReadFromUsart[5]);
				// ��������������,�����ݿ�������Ӧ�ṹ����(ע�⿽�����ݵĳ���)
				switch (CmdID)
				{
				case ID_game_state: // 0x0001
					memcpy(&GameState, (ReadFromUsart + DATA), LEN_game_state);
					break;

				case ID_game_result: // 0x0002
					memcpy(&GameResult, (ReadFromUsart + DATA), LEN_game_result);
					break;

				case ID_game_robot_HP: // 0x0003
					memcpy(&GameRobotHP, (ReadFromUsart + DATA), LEN_game_robot_HP);
					//						if(JUDGE_GetSelfColor()==RobotColor_Blue)
					//						{
					//							Vision_SendBloodInfo1(GameRobotHP.red_1_robot_HP,GameRobotHP.red_2_robot_HP,GameRobotHP.red_3_robot_HP);
					//							Vision_SendBloodInfo2(GameRobotHP.red_4_robot_HP,GameRobotHP.red_5_robot_HP,GameRobotHP.red_7_robot_HP);
					//						}
					//						else
					//						{
					//							Vision_SendBloodInfo1(GameRobotHP.blue_1_robot_HP,GameRobotHP.blue_2_robot_HP,GameRobotHP.blue_3_robot_HP);
					//							Vision_SendBloodInfo2(GameRobotHP.blue_4_robot_HP,GameRobotHP.blue_5_robot_HP,GameRobotHP.blue_7_robot_HP);
					//						}
					break;

				case ID_event_data: // 0x0101
					memcpy(&EventData, (ReadFromUsart + DATA), LEN_event_data);
					break;


				case ID_referee_warning: // 0x0104
					memcpy(&RefereeWarning, (ReadFromUsart + DATA), LEN_referee_warning);
					break;

				case ID_dart_remaining_time: // 0x0105
					memcpy(&DartRemainingTime, (ReadFromUsart + DATA), LEN_dart_remaining_time);
					break;

				case ID_game_robot_state: // 0x0201
					memcpy(&GameRobotStat, (ReadFromUsart + DATA), LEN_game_robot_state);
					break;

				case ID_power_heat_data: // 0x0202
					memcpy(&PowerHeatData, (ReadFromUsart + DATA), LEN_power_heat_data);
					break;

				case ID_game_robot_pos: // 0x0203
					memcpy(&GameRobotPos, (ReadFromUsart + DATA), LEN_game_robot_pos);
					break;

				case ID_buff_musk: // 0x0204
					memcpy(&BuffMusk, (ReadFromUsart + DATA), LEN_buff_musk);
					break;

				case ID_robot_hurt: // 0x0206
					memcpy(&RobotHurt, (ReadFromUsart + DATA), LEN_robot_hurt);
					break;

				case ID_shoot_data: // 0x0207
					memcpy(&ShootData, (ReadFromUsart + DATA), LEN_shoot_data);
					shootNum++; // ����һ�����Ƿ�����һ��
					// Vision_SendShootSpeed(ShootData.bullet_speed);
					break;

				case ID_bullet_remaining: // 0x0208
					memcpy(&BulletRemaining, (ReadFromUsart + DATA), LEN_bullet_remaining);
					break;

				case ID_rfid_status: // 0x0209
					memcpy(&RfidStatus, (ReadFromUsart + DATA), LEN_rfid_status);
					break;
				}
				// �׵�ַ��֡����,ָ��CRC16��һ�ֽ�,�����ж��Ƿ�Ϊ0xA5,�����ж�һ�����ݰ��Ƿ��ж�֡����
				if (*(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL) == 0xA5)
				{
					// ���һ�����ݰ������˶�֡����,���ٴζ�ȡ
					JUDGE_Read_Data(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL);
				}
			}
		}
		// �׵�ַ��֡����,ָ��CRC16��һ�ֽ�,�����ж��Ƿ�Ϊ0xA5,�����ж�һ�����ݰ��Ƿ��ж�֡����
		if (*(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL) == 0xA5)
		{
			// ���һ�����ݰ������˶�֡����,���ٴζ�ȡ
			JUDGE_Read_Data(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL);
		}
	}

	if (retval_tf == TRUE)
	{
		Judge_Data_TF = TRUE; // ����������
	}
	else // ֻҪCRC16У�鲻ͨ����ΪFALSE
	{
		Judge_Data_TF = FALSE; // ����������
	}

	return retval_tf; // ����������������
}

 void USART1_dma_init()
 {
// 	LL_USART_SetTransferDirection(USART1, LL_USART_DIRECTION_TX_RX);

// 	// IT trans
//	LL_USART_EnableIT_IDLE(USART1);
// 	LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_0, (uint32_t)usart1RxBuf);
// 	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_0, sizeof(usart1RxBuf));
// 	LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_0, (uint32_t)&USART1->RDR);
// 	LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_0);
// 	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_0);
//  LL_USART_EnableDMAReq_RX(USART1);
//  
// 	// send
// 	LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_1, (uint32_t)&USART1->TDR);
// 	LL_USART_EnableDMAReq_TX(USART1);
// 	LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_1);
 }


void USER_USART1_IRQHandler(void)
{
//    if (LL_USART_IsActiveFlag_IDLE(USART1) && LL_USART_IsEnabledIT_IDLE(USART1))
//    {
//        LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_0);               
////        int usart1RxLen=JUDGE_MAX_RX_LENGTH - LL_DMA_GetDataLength(DMA2, LL_DMA_STREAM_0);
//        // ������������       
//                JUDGE_Read_Data(usart1RxBuf);
////                judgedata_update();
//                memset(usart1RxBuf,0,sizeof(usart1RxBuf));                         
//                LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_0,JUDGE_MAX_RX_LENGTH);
//                LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_0);
//                LL_USART_ClearFlag_IDLE(USART1);     
//    }
}

////����1�жϻص�
//void USER_USART1_IRQHandler()
//{
//	if (LL_USART_IsActiveFlag_IDLE(USART1) && LL_USART_IsEnabledIT_IDLE(USART1))
//	{
//		LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_0);

//		// ��ȡ���յ������ݳ���
//		uint16_t rxLen = JUDGE_MAX_RX_LENGTH - LL_DMA_GetDataLength(DMA2, LL_DMA_STREAM_0);
//		JUDGE_Read_Data(usart1RxBuf);
//		
//		memset(usart1RxBuf,0,sizeof(usart1RxBuf));
//		LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_0, JUDGE_MAX_RX_LENGTH);
//		LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_0);
//		LL_USART_ClearFlag_IDLE(USART1);
//		
//		Detect_Update(DeviceID_Judge);
//	}
//}


void USART1_DMA_Send(uint8_t *tx_buffer, uint16_t size)
{
//	while (LL_DMA_IsEnabledStream(DMA2, LL_DMA_STREAM_1) &&
//		   !LL_DMA_IsActiveFlag_TC1(DMA2))
//	{
//		// ���ó�ʱ����������
//		static uint32_t timeout = 0;
//		if (++timeout > 10000)
//		{
//			LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_1);
//			LL_DMA_ClearFlag_TC1(DMA2);
//			timeout = 0;
//			break;
//		}
//	}
//	LL_DMA_ClearFlag_TC1(DMA2);
//	LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_1);
//	LL_DMA_ConfigTransfer(DMA2, LL_DMA_STREAM_1,
//						  LL_DMA_DIRECTION_MEMORY_TO_PERIPH |
//							  LL_DMA_MEMORY_INCREMENT |
//							  LL_DMA_PERIPH_NOINCREMENT |
//							  LL_DMA_PDATAALIGN_BYTE |
//							  LL_DMA_MDATAALIGN_BYTE);
//	LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_1, (uint32_t)&USART1->TDR);
//	LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_1, (uint32_t)tx_buffer);
//	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_1, size);
//	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_1);
}

extern DMA_HandleTypeDef hdma_usart1_rx;

// ����ϵͳ���߻ص�����
void Judge_UartLostCallback()
{
	
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1,usart1RxBuf,sizeof(usart1RxBuf));
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx , DMA_IT_HT);
	
//	LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_0);
//	// ������б�־λ
//	LL_USART_ClearFlag_IDLE(USART1);
//	LL_USART_ClearFlag_ORE(USART1);
//	LL_USART_ClearFlag_FE(USART1);
//	LL_USART_ClearFlag_NE(USART1);
//	LL_USART_ClearFlag_PE(USART1);

//	memset(usart1RxBuf, 0, sizeof(usart1RxBuf));
//	Judge_Data_TF = FALSE;
//	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_0, sizeof(usart1RxBuf));
//	LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_0, (uint32_t)usart1RxBuf);
//	LL_USART_EnableIT_IDLE(USART1);
//	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_0);
}
// ����ϵͳ��ʼ��


void JUDGE_Init()
{

//	USART1_dma_init();

		HAL_UARTEx_ReceiveToIdle_DMA(&huart1,usart1RxBuf,sizeof(usart1RxBuf));
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx , DMA_IT_HT);
}

void JUDGE_SendTextStruct(graphic_data_struct_t *textConf, uint8_t text[30], uint8_t len)
{
	JudgeTxFrame txFrame;
	ext_TextData_t textData;
	textData.txFrameHeader.SOF = 0xA5;
	textData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
	textData.txFrameHeader.Seq = 0;
	memcpy(txFrame.data, &textData.txFrameHeader, sizeof(xFrameHeader)); // д��֡ͷ����
	Append_CRC8_Check_Sum(txFrame.data, sizeof(xFrameHeader));			 // д��֡ͷCRC8У����

	textData.CmdID = 0x301;										// ����֡ID
	textData.dataFrameHeader.data_cmd_id = 0x0110;				// ���ݶ�ID
	textData.dataFrameHeader.send_ID = JUDGE_GetSelfID();		// �����ߵ�ID
	textData.dataFrameHeader.receiver_ID = JUDGE_GetClientID(); // �ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���

	textData.textData.grapic_data_struct = *textConf;
	memcpy(textData.textData.data, text, len);

	memcpy(
		txFrame.data + sizeof(xFrameHeader),
		(uint8_t *)&textData.CmdID,
		sizeof(textData.CmdID) + sizeof(textData.dataFrameHeader) + sizeof(textData.textData));
	Append_CRC16_Check_Sum(txFrame.data, sizeof(textData));

	txFrame.frameLength = sizeof(textData);
	Queue_Enqueue(&judgeQueue, &txFrame);
}

void JUDGE_SendGraphStruct(graphic_data_struct_t *data)
{
	JudgeTxFrame txFrame;
	ext_GraphData_t graphData;
	graphData.txFrameHeader.SOF = 0xA5;
	graphData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_graphic_single_t);
	graphData.txFrameHeader.Seq = 0;
	memcpy(txFrame.data, &graphData.txFrameHeader, sizeof(xFrameHeader)); // д��֡ͷ����
	Append_CRC8_Check_Sum(txFrame.data, sizeof(xFrameHeader));			  // д��֡ͷCRC8У����

	graphData.CmdID = 0x301;									 // ����֡ID
	graphData.dataFrameHeader.data_cmd_id = 0x0101;				 // ���ݶ�ID
	graphData.dataFrameHeader.send_ID = JUDGE_GetSelfID();		 // �����ߵ�ID
	graphData.dataFrameHeader.receiver_ID = JUDGE_GetClientID(); // �ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���

	graphData.graphData.grapic_data_struct = *data;

	memcpy(
		txFrame.data + sizeof(xFrameHeader),
		(uint8_t *)&graphData.CmdID,
		sizeof(graphData.CmdID) + sizeof(graphData.dataFrameHeader) + sizeof(graphData.graphData));
	Append_CRC16_Check_Sum(txFrame.data, sizeof(graphData));

	txFrame.frameLength = sizeof(graphData);
	Queue_Enqueue(&judgeQueue, &txFrame);
}

// ��ȡ������ɫ
RobotColor JUDGE_GetSelfColor()
{
	if (JUDGE_GetSelfID() > 10) // ����
	{
		return RobotColor_Blue;
	}
	else // �췽
	{
		return RobotColor_Red;
	}
}

// ��ȡ����ID
uint8_t JUDGE_GetSelfID()
{
	return GameRobotStat.robot_id;
}

// ��ȡ�ͻ���ID
uint16_t JUDGE_GetClientID()
{
	return 0x100 + GameRobotStat.robot_id;
}

// ��ȡ����������
void JUDGE_GetPosition(float *x, float *y)
{
	*x = GameRobotPos.x;
	*y = GameRobotPos.y;
}

// ��ȡ���̹�������
uint8_t JUDGE_GetChassisPowerLimit()
{
	return GameRobotStat.chassis_power_limit;
}

// �жϷ����Դ�Ƿ����
bool JUDGE_GetShooterOutputState()
{
	return GameRobotStat.power_management_shooter_output;
}

bool JUDGE_GetGimbalOutputState()
{
	return GameRobotStat.power_management_gimbal_output;
}

// ��ȡǹ����������
uint16_t JUDGE_GetHeatLimit()
{
	return GameRobotStat.shooter_barrel_heat_limit;
}

// ��ȡ��������
uint16_t JUDGE_GetShootSpeedLimit()
{
	return 25;
}

// ��ȡ���̻�������
uint16_t JUDGE_GetPowerBuffer()
{
	return PowerHeatData.chassis_power_buffer;
}

// ��ȡʣ��ǹ������
int16_t JUDGE_GetRemainHeat()
{
	return (int16_t)GameRobotStat.shooter_barrel_heat_limit - (int16_t)PowerHeatData.shooter_id1_17mm_cooling_heat;
}

// ʣ��17������
uint16_t JUDGE_GetRemain_42_Num()
{
	return BulletRemaining.projectile_allowance_42mm;
}

// ����ϵͳ�����Ƿ���Ч
bool JUDGE_IsValid(void)
{
	return Judge_Data_TF;
}

// ��Ѫԭ��
uint8_t HP_deduction_reason()
{
	return RobotHurt.hurt_type;
}

// ��ȡ��ǰѪ��
uint16_t JUDGE_GetHP()
{
	return GameRobotStat.current_HP;
}

// ��ȡ��ȴ�ٶ�
uint16_t JUDGE_GetCoolingValue()
{
	return GameRobotStat.shooter_barrel_cooling_value;
}



/**********************freertos����*********************************/
// ����ϵͳ��������ص�
void Task_Judge_Callback()
{
//	if (Queue_IsEmpty(&judgeQueue))
//		return;
//	// ȡ��ͷ����Ϣ����
//	JudgeTxFrame *frame = (JudgeTxFrame *)Queue_Dequeue(&judgeQueue);
////	USART1_DMA_Send((uint8_t *)frame->data, frame->frameLength);
//	HAL_UART_Transmit_DMA(&huart1,(uint8_t*)frame->data,frame->frameLength);
}
extern int aaaa;
int aaaaa;

#ifdef EN_JUDGE_TASK
void OS_JudgeCallback(void const *argument)
{

	osDelay(500);
	for (;;)
	{
		// Task_Judge_Callback();
		aaaaa = aaaa;
		aaaa = 0;
		osDelay(1000);
	}
}
#endif
