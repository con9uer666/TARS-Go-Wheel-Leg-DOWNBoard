#include "Board2Board.h"
#include "USER_CAN.h"
#include "Detect.h"
#include "UserFreertos.h"
#include "Judge.h"
#include "Com.h"
#include "usart.h"
#include "cmsis_os.h"
#include "motor.h"

extern uint8_t usart1RxBuf[JUDGE_MAX_RX_LENGTH];
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern osThreadId ErrorHandle;

uint8_t rs485_isvalid = 0;
float total_turnPower = 0;
uint8_t cap_fastMode=0;
uint8_t diagonal_enable = 1;
uint8_t trigger_block;
int	shootnum;
uint8_t trigger_reverse;

#define EN_B2B_TASK		  // 使能任务
uint8_t usart2RxBuf[256]; // 串口2缓冲区
uint8_t STOPFLAG = 0;
uint8_t FEEDBACK = 0;
int16_t fricMotor_left_speed;
uint8_t chassis_rotate_mode;
int16_t chassis_rotate_angle;
uint8_t vision_mode; // 0为自瞄，1为小符，2为大符
uint8_t visionFindcheck;
int16_t visionX = 0;
int16_t visionY = 0;
uint8_t vision_exposure;
uint8_t vision_rune_dirt; ////0-anti-clockwise 1-clockwise
uint16_t speed_limit1 = 0;
float v_dis = 0;

uint32_t rs485_cnt = 0;

uint8_t UP_Leg;

// 板间通信初始化
void B2B_Init()
{
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, usart2RxBuf, sizeof(usart2RxBuf));
	__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
}


void B2B_LostCallback()
{
		rs485_cnt++;
		B2B_Init();
		if (rs485_cnt <= 2)
			return; // 双板485初次进入等上板发送对齐时间

		if (rs485_cnt > 10)
		{
//			osThreadResume(ErrorTaskHandle);
		}
		
}
// 解析串口数据并转发数据回传利用代码执行时间进行短时间延时
uint8_t txbuffer[64] = {0};
int receive_times;

extern float target_body_speed;
float Foot_Target_Relative_Angle;

void B2B_ParseUsart() // 先发低字节
{
	if (usart2RxBuf[0] == 0xAA && usart2RxBuf[63] == 0xFE)
	{
		Foot_Target_Relative_Angle = (float)((int16_t)(usart2RxBuf[1] | usart2RxBuf[2] << 8))/1000.0f;
		target_body_speed = (float)((int16_t)(usart2RxBuf[3] | usart2RxBuf[4] << 8))/1000.0f;
		uint8_t stopFlag = (usart2RxBuf[29] >> 7) & 0x01;	 // 最高位
		uint8_t chassisMode = (usart2RxBuf[29] >> 5) & 0x03; // 第6-7位
		uint8_t visionFind = (usart2RxBuf[29] >> 4) & 0x01;	 // 第5位+
		uint8_t visionMode = (usart2RxBuf[29] >> 2) & 0x03;	 // 第3-4位
		cap_fastMode=(usart2RxBuf[29] >> 1) & 0x01;
		RemoteControl.keyboard_value.bit.C = usart2RxBuf[29] & 0x01;
		
		// 更新相应的变量
		STOPFLAG = stopFlag;
		chassis_rotate_mode = chassisMode;
		visionFindcheck = visionFind;
		vision_mode = visionMode;

		Yaw_DM4310.Target_Torque = (float)((int16_t)(usart2RxBuf[25] | usart2RxBuf[26] << 8)) / 1000.0f;

		Shooter_DM2325.Target_Torque = (float)((((int16_t)(usart2RxBuf[27] | usart2RxBuf[28] << 8))/1000.0f)*0.18f);

		fricMotor_left_speed = usart2RxBuf[30] | usart2RxBuf[31] << 8;
		chassis_rotate_angle = usart2RxBuf[32] | usart2RxBuf[33] << 8;

		v_dis = usart2RxBuf[34];
		speed_limit1 = usart2RxBuf[35] & 0x0F;

		shootnum = usart2RxBuf[38] << 16 | usart2RxBuf[37] << 8 | usart2RxBuf[36]; 
		trigger_reverse = usart2RxBuf[39];
		diagonal_enable = (usart2RxBuf[41] >> 7) & 0x01;
		vision_exposure = (usart2RxBuf[41] >> 2) & 0x1F;
		vision_rune_dirt = (usart2RxBuf[41] >> 1) & 0x01;
		trigger_block = usart2RxBuf[41] & 0x01;

		UP_Leg = usart2RxBuf[42];

		// for(int i = 0; i <= 127; i++)
		// {
		// 	usart2RxBuf[i] = 0;
		// }
		
		/* 发送    */
		txbuffer[0] = 0xAB;
		txbuffer[63] = 0xFD;

		txbuffer[25] = (int16_t)(Yaw_DM4310.Rx_Data.Position * 1000);
		txbuffer[26] = ((int16_t)(Yaw_DM4310.Rx_Data.Position * 1000)) >> 8;

		txbuffer[27] = (int16_t)(Yaw_DM4310.Rx_Data.Velocity * 1000);
		txbuffer[28] = (int16_t)(Yaw_DM4310.Rx_Data.Velocity * 1000) >> 8;

		txbuffer[29] = (int16_t)(Shooter_DM2325.Rx_Data.Position * 1000);
		txbuffer[30] = ((int16_t)(Shooter_DM2325.Rx_Data.Position * 1000)) >> 8;

		txbuffer[31] = (int16_t)(Shooter_DM2325.Rx_Data.Velocity * 100);
		txbuffer[32] = ((int16_t)(Shooter_DM2325.Rx_Data.Velocity * 100)) >> 8;

		GameRobotStat.power_management_gimbal_output = 1;
		GameRobotStat.power_management_chassis_output = 1;
		GameRobotStat.power_management_shooter_output = 1;
		detectList[DeviceID_YawMotor].isLost = 0;

		txbuffer[33] = FEEDBACK << 7 | detectList[DeviceID_YawMotor].isLost << 6 | GameRobotStat.power_management_gimbal_output << 5 | GameRobotStat.power_management_chassis_output << 4 |
					   GameRobotStat.power_management_shooter_output << 3 | GameRobotStat.robot_level;


		txbuffer[35] = JUDGE_GetRemainHeat();
		txbuffer[36] = JUDGE_GetRemainHeat() >> 8;

		txbuffer[39] = JUDGE_GetCoolingValue();
		txbuffer[40] = JUDGE_GetCoolingValue() >> 8;

		txbuffer[41] = JUDGE_GetPowerBuffer();
		txbuffer[42] = JUDGE_GetPowerBuffer() >> 8;
		txbuffer[43] = JUDGE_GetChassisPowerLimit();
		txbuffer[44] = JUDGE_GetChassisPowerLimit() >> 8;

		txbuffer[45] = JUDGE_IsValid();
		
		rs485_isvalid = 1;

		HAL_UART_Transmit_DMA(&huart2, txbuffer, 64);
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart == &huart1)
	{
		// JUDGE_Read_Data(usart1RxBuf);
		Detect_Update(DeviceID_Judge);
	}

	if (huart == &huart2 && huart->ReceptionType == HAL_UART_RECEPTION_STANDARD)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1,usart1RxBuf,sizeof(usart1RxBuf));
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx , DMA_IT_HT);
		receive_times ++;
		B2B_ParseUsart();
		Detect_Update(DeviceID_B2B);
		detectList[DeviceID_B2B].isLost = 0;
	}
}

void Task_B2B_Callback()
{
	/**********特殊情况处理*********************/
	if (STOPFLAG == 1)
	{
		FEEDBACK = 1;
		B2B_ParseUsart();
		osThreadResume(ErrorHandle); // 恢复错误任务 饿死其他任务
	}
}
int times;
/************************freertos任务****************************/
#ifdef EN_B2B_TASK // 使能任务
void OS_Board2BoardCallback(void const *argument)
{
	for (;;)
	{
		times ++;
		if(times >= 100)
		{
			times = 0;
			if(receive_times <= 5)
			{
				HAL_UARTEx_ReceiveToIdle_DMA(&huart2,usart2RxBuf,sizeof(usart2RxBuf));
				__HAL_DMA_DISABLE_IT(&hdma_usart2_rx,DMA_IT_HT);
			}
			receive_times = 0;
		}
		Task_B2B_Callback();
		osDelay(2);
	}
}
#endif
