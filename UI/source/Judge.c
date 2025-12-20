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

/*****************系统数据定义**********************/
ext_game_status_t GameState;						   // 0x0001
ext_game_result_t GameResult;						   // 0x0002
ext_game_robot_HP_t GameRobotHP;					 // 0x0003
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

remote_control_t RemoteControl; // 0x0304

xFrameHeader FrameHeader; // 发送帧头信息
/****************************************************/
bool Judge_Data_TF = FALSE; // 裁判数据是否可用,辅助函数调用

// 发送队列
Queue judgeQueue = EMPTY_QUEUE;
// 发送队列数据保存区
JudgeTxFrame judgeQueueBuf[JUDGE_QUEUE_SIZE];
extern Line line;
extern Rect rect;
extern Circle circle;
extern Oval oval;
extern Arc arc;
extern Text text;
extern FloatShape floatShape;
extern IntShape intShape;

// 串口接收缓冲区
uint8_t usart1RxBuf[JUDGE_MAX_RX_LENGTH];

uint16_t shootNum = 0; // 统计发弹量

/**************裁判系统数据辅助****************/

/**
 * @brief  读取裁判数据,中断中读取保证速度
 * @param  缓存数据
 * @retval 是否对正误判断做处理
 * @attention  在此判断帧头和CRC校验,无误再写入数据，不重复判断帧头
 */
bool JUDGE_Read_Data(uint8_t *ReadFromUsart)
{
	bool retval_tf = FALSE; // 数据正确与否标志,每次调用读取裁判系统数据函数都先默认为错误

	uint16_t judge_length; // 统计一帧数据长度
	int CmdID = 0;		   // 数据命令码解析

	/***------------------*****/
	// 无数据包，则不作任何处理
	if (ReadFromUsart == NULL)
	{
		return -1;
	}
	// 写入帧头数据,用于判断是否开始存储裁判数据
	memcpy(&FrameHeader, ReadFromUsart, LEN_HEADER);

	// 判断帧头数据是否为0xA5
	if (ReadFromUsart[SOF] == JUDGE_FRAME_HEADER)
	{
		// 帧头CRC8校验
		if (Verify_CRC8_Check_Sum(ReadFromUsart, LEN_HEADER) == TRUE)
		{
			// 统计一帧数据长度,用于CR16校验
			judge_length = ReadFromUsart[DATA_LENGTH] + LEN_HEADER + LEN_CMDID + LEN_TAIL;
			;

			// 帧尾CRC16校验
			if (Verify_CRC16_Check_Sum(ReadFromUsart, judge_length) == TRUE)
			{
				retval_tf = TRUE; // 都校验过了则说明数据可用

				CmdID = (ReadFromUsart[6] << 8 | ReadFromUsart[5]);
				// 解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)
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
					shootNum++; // 触发一次则是发射了一颗
					// Vision_SendShootSpeed(ShootData.bullet_speed);
					break;

				case ID_bullet_remaining: // 0x0208
					memcpy(&BulletRemaining, (ReadFromUsart + DATA), LEN_bullet_remaining);
					break;

				case ID_rfid_status: // 0x0209
					memcpy(&RfidStatus, (ReadFromUsart + DATA), LEN_rfid_status);
					break;

				case ID_game_remote_control://0x0304
					memcpy(&RemoteControl, (ReadFromUsart + DATA), LEN_remote_control);
					break;
				}
				// 首地址加帧长度,指向CRC16下一字节,用来判断是否为0xA5,用来判断一个数据包是否有多帧数据
				if (*(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL) == 0xA5)
				{
					// 如果一个数据包出现了多帧数据,则再次读取
					JUDGE_Read_Data(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL);
				}
			}
		}
		// 首地址加帧长度,指向CRC16下一字节,用来判断是否为0xA5,用来判断一个数据包是否有多帧数据
		if (*(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL) == 0xA5)
		{
			// 如果一个数据包出现了多帧数据,则再次读取
			JUDGE_Read_Data(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL);
		}
	}

	if (retval_tf == TRUE)
	{
		Judge_Data_TF = TRUE; // 辅助函数用
	}
	else // 只要CRC16校验不通过就为FALSE
	{
		Judge_Data_TF = FALSE; // 辅助函数用
	}

	return retval_tf; // 对数据正误做处理
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
//        // 解析串口数据       
//                JUDGE_Read_Data(usart1RxBuf);
////                judgedata_update();
//                memset(usart1RxBuf,0,sizeof(usart1RxBuf));                         
//                LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_0,JUDGE_MAX_RX_LENGTH);
//                LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_0);
//                LL_USART_ClearFlag_IDLE(USART1);     
//    }
}

////串口1中断回调
//void USER_USART1_IRQHandler()
//{
//	if (LL_USART_IsActiveFlag_IDLE(USART1) && LL_USART_IsEnabledIT_IDLE(USART1))
//	{
//		LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_0);

//		// 获取接收到的数据长度
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
//		// 设置超时，避免死等
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

// 裁判系统掉线回调函数
void Judge_UartLostCallback()
{
	
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1,usart1RxBuf,sizeof(usart1RxBuf));
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx , DMA_IT_HT);
	
//	LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_0);
//	// 清除所有标志位
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
// 裁判系统初始化


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
	memcpy(txFrame.data, &textData.txFrameHeader, sizeof(xFrameHeader)); // 写入帧头数据
	Append_CRC8_Check_Sum(txFrame.data, sizeof(xFrameHeader));			 // 写入帧头CRC8校验码

	textData.CmdID = 0x301;										// 数据帧ID
	textData.dataFrameHeader.data_cmd_id = 0x0110;				// 数据段ID
	textData.dataFrameHeader.send_ID = JUDGE_GetSelfID();		// 发送者的ID
	textData.dataFrameHeader.receiver_ID = JUDGE_GetClientID(); // 客户端的ID，只能为发送者机器人对应的客户端

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
	memcpy(txFrame.data, &graphData.txFrameHeader, sizeof(xFrameHeader)); // 写入帧头数据
	Append_CRC8_Check_Sum(txFrame.data, sizeof(xFrameHeader));			  // 写入帧头CRC8校验码

	graphData.CmdID = 0x301;									 // 数据帧ID
	graphData.dataFrameHeader.data_cmd_id = 0x0101;				 // 数据段ID
	graphData.dataFrameHeader.send_ID = JUDGE_GetSelfID();		 // 发送者的ID
	graphData.dataFrameHeader.receiver_ID = JUDGE_GetClientID(); // 客户端的ID，只能为发送者机器人对应的客户端

	graphData.graphData.grapic_data_struct = *data;

	memcpy(
		txFrame.data + sizeof(xFrameHeader),
		(uint8_t *)&graphData.CmdID,
		sizeof(graphData.CmdID) + sizeof(graphData.dataFrameHeader) + sizeof(graphData.graphData));
	Append_CRC16_Check_Sum(txFrame.data, sizeof(graphData));

	txFrame.frameLength = sizeof(graphData);
	Queue_Enqueue(&judgeQueue, &txFrame);
}

// 获取己方颜色
RobotColor JUDGE_GetSelfColor()
{
	if (JUDGE_GetSelfID() > 10) // 蓝方
	{
		return RobotColor_Blue;
	}
	else // 红方
	{
		return RobotColor_Red;
	}
}

// 获取自身ID
uint8_t JUDGE_GetSelfID()
{
	return GameRobotStat.robot_id;
}

// 获取客户端ID
uint16_t JUDGE_GetClientID()
{
	return 0x100 + GameRobotStat.robot_id;
}

// 获取机器人坐标
void JUDGE_GetPosition(float *x, float *y)
{
	*x = GameRobotPos.x;
	*y = GameRobotPos.y;
}

// 获取底盘功率限制
uint8_t JUDGE_GetChassisPowerLimit()
{
	return GameRobotStat.chassis_power_limit;
}

// 判断发射电源是否输出
bool JUDGE_GetShooterOutputState()
{
	return GameRobotStat.power_management_shooter_output;
}

bool JUDGE_GetGimbalOutputState()
{
	return GameRobotStat.power_management_gimbal_output;
}

// 获取枪口热量限制
uint16_t JUDGE_GetHeatLimit()
{
	return GameRobotStat.shooter_barrel_heat_limit;
}

// 获取射速限制
uint16_t JUDGE_GetShootSpeedLimit()
{
	return 25;
}

// 获取底盘缓冲能量
uint16_t JUDGE_GetPowerBuffer()
{
	return PowerHeatData.chassis_power_buffer;
}

// 获取剩余枪口热量
int16_t JUDGE_GetRemainHeat()
{
	return (int16_t)GameRobotStat.shooter_barrel_heat_limit - (int16_t)PowerHeatData.shooter_id1_17mm_cooling_heat;
}

// 剩余17发弹数
uint16_t JUDGE_GetRemain_42_Num()
{
	return BulletRemaining.projectile_allowance_42mm;
}

// 裁判系统数据是否有效
bool JUDGE_IsValid(void)
{
	return Judge_Data_TF;
}

// 扣血原因
uint8_t HP_deduction_reason()
{
	return RobotHurt.hurt_type;
}

// 读取当前血量
uint16_t JUDGE_GetHP()
{
	return GameRobotStat.current_HP;
}

// 获取冷却速度
uint16_t JUDGE_GetCoolingValue()
{
	return GameRobotStat.shooter_barrel_cooling_value;
}



/**********************freertos任务*********************************/
// 裁判系统发送任务回调
void Task_Judge_Callback()
{
//	if (Queue_IsEmpty(&judgeQueue))
//		return;
//	// 取队头的消息发送
//	JudgeTxFrame *frame = (JudgeTxFrame *)Queue_Dequeue(&judgeQueue);
////	USART1_DMA_Send((uint8_t *)frame->data, frame->frameLength);
//	HAL_UART_Transmit_DMA(&huart1,(uint8_t*)frame->data,frame->frameLength);
}


#ifdef EN_JUDGE_TASK
void OS_JudgeCallback(void const *argument)
{

	osDelay(500);
	for (;;)
	{
		Task_Judge_Callback();
		osDelay(100);
	}
}
#endif
