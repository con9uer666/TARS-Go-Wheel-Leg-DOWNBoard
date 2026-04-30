#include "Board2Board.h"
#include "USER_CAN.h"
#include "Detect.h"
#include "UserFreertos.h"
#include "Judge.h"
#include "Com.h"
#include "usart.h"
#include "cmsis_os.h"
#include "motor.h"
#include <stdint.h>
#include "User_State.h"
#include "Gimbal.h"
#include "Motor_Drv.h"

extern uint8_t usart1RxBuf[JUDGE_MAX_RX_LENGTH];
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_uart7_rx;
extern DMA_HandleTypeDef hdma_usart10_rx;
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
uint8_t STOPFLAG = 0;	// 停止标志，1为停止，0为正常
uint8_t FEEDBACK = 0;
int16_t fricMotor_left_speed;
uint8_t chassis_rotate_mode;	//画UI用的
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
float Foot_Target_Relative_Angle;//!目前没用。以后可能会用
uint8_t upstairs_flag = 0;//0：常态；1：上台阶的瞬间


void B2B_ParseUsart() // 先发低字节
{
	if (usart2RxBuf[0] == 0xAA && usart2RxBuf[63] == 0xFE)
	{
		Foot_Chassis.Target_Vx = (float)((int16_t)(usart2RxBuf[1] | usart2RxBuf[2] << 8))/1000.0f;
		Foot_Chassis.Target_Vy = (float)((int16_t)(usart2RxBuf[3] | usart2RxBuf[4] << 8))/1000.0f;
		uint8_t stopFlag = (usart2RxBuf[29] >> 7) & 0x01;	 // 最高位
		uint8_t chassisMode = (usart2RxBuf[29] >> 5) & 0x03; // 第6-7位
		uint8_t visionFind = (usart2RxBuf[29] >> 4) & 0x01;	 // 第5位+
		uint8_t visionMode = (usart2RxBuf[29] >> 2) & 0x03;	 // 第3-4位
		cap_fastMode=(usart2RxBuf[29] >> 1) & 0x01;
		// RemoteControl.keyboard_value.bit.C = usart2RxBuf[29] & 0x01;
		
		// 更新相应的变量
		STOPFLAG = stopFlag;
		chassis_rotate_mode = chassisMode;
		visionFindcheck = visionFind;
		vision_mode = visionMode;

		Yaw_DM4310.Target_Speed = (float)((int16_t)(usart2RxBuf[25] | usart2RxBuf[26] << 8)) / 1000.0f;

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

		Foot_Chassis.Target_Leg_State = usart2RxBuf[42];
		Foot_Chassis.Chassis_Mode = usart2RxBuf[43];
		// if(usart2RxBuf[44] == 1)
		// {
		// 	upstairs_flag = 1;
		// }
		upstairs_flag = usart2RxBuf[44];
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

		txbuffer[33] = FEEDBACK << 7 | detectList[DeviceID_YawMotor].isLost << 6 | GameRobotStat.power_management_gimbal_output << 5 | GameRobotStat.power_management_chassis_output << 4 | GameRobotStat.power_management_shooter_output << 3 | GameRobotStat.robot_level;
		//!FEEDBACK：因为上板开启急停之后就不发消息了，所以下板这个feedback意思就是告诉上板“我听见你的急停了，你可以死了”上板听到这句话，然后就死了，485就静默了……这个骂不了丛庆，因为是他的老登写的，史作俑者：王传祺

		txbuffer[35] = JUDGE_GetRemainHeat();
		txbuffer[36] = JUDGE_GetRemainHeat() >> 8;

		txbuffer[39] = JUDGE_GetCoolingValue();
		txbuffer[40] = JUDGE_GetCoolingValue() >> 8;

		txbuffer[41] = JUDGE_GetPowerBuffer();
		txbuffer[42] = JUDGE_GetPowerBuffer() >> 8;
		txbuffer[43] = JUDGE_GetChassisPowerLimit();
		txbuffer[44] = JUDGE_GetChassisPowerLimit() >> 8;

		txbuffer[45] = JUDGE_IsValid();

		txbuffer[46] = gimbal_follow_flag;

		rs485_isvalid = 1;

		HAL_UART_Transmit_DMA(&huart2, txbuffer, 64);
	}
}

uint16_t B2B_offline_cnt = 0;
uint16_t pre_B2B_offline_cnt = 0;
uint8_t B2B_time_cnt = 0;
uint8_t B2B_offline_flag = 0;
uint8_t usart7RxBuf[128];
uint8_t usart10RxBuf[128];

// TFmini Plus 激光雷达 — see Board2Board.h for struct definition
TFmini_Data_t tfmini_data;         // 解析后的最新有效数据（供外部读取）
static uint8_t  tfmini_buf[9];     // 帧拼装缓冲（9字节 = 0x59 0x59 + 6数据 + 1校验）
static uint8_t  tfmini_idx;        // 拼装位置 (0=未同步, 1~8=正在拼, 9=帧完整)

// 从 usart10RxBuf 字节流中解析一帧 TFmini Plus 数据，填入 tfmini_data
void TFmini_Parse(void)
{
    // 遍历本次 DMA 接收的所有字节
    for (uint16_t i = 0; i < sizeof(usart10RxBuf); i++)
    {
        uint8_t b = usart10RxBuf[i];            // 当前字节

        if (tfmini_idx == 0)                    // 未同步，找帧头 0x59
        {
            if (b == 0x59) { tfmini_buf[0] = 0x59; tfmini_idx = 1; } // 收到第一个 0x59
        }
        else if (tfmini_idx == 1)               // 验证第二个帧头
        {
            if (b == 0x59) { tfmini_buf[1] = 0x59; tfmini_idx = 2; } // 帧头完整
            else           { tfmini_idx = 0; }                       // 不是 0x59，重新同步
        }
        else                                    // 正在拼数据字节（idx=2~8）
        {
            tfmini_buf[tfmini_idx] = b;         // 存入拼装缓冲
            tfmini_idx++;                       // 下标后移

            if (tfmini_idx == 9)                // 9 字节收齐，开始解析
            {
                tfmini_idx = 0;                 // 复位，准备下一帧

                // ——— 校验和：前 8 字节累加取低 8 位 ———
                uint8_t sum = 0;
                for (int k = 0; k < 8; k++) sum += tfmini_buf[k];
                if (sum != tfmini_buf[8]) continue; // 校验失败，丢弃

                // ——— 小端解析 ———
                uint16_t dist     = tfmini_buf[2] | (uint16_t)tfmini_buf[3] << 8; // 距离，单位 cm
                uint16_t strength = tfmini_buf[4] | (uint16_t)tfmini_buf[5] << 8; // 信号强度
                int16_t  temp_raw = tfmini_buf[6] | (uint16_t)tfmini_buf[7] << 8; // 温度原始值

                // ——— 有效性判断：强度 < 100 或 == 65535 视为无效 ———
                if (strength < 100 || strength == 65535)
                {
                    tfmini_data.distance = 0;
                    tfmini_data.valid    = 0;
                }
                else
                {
                    tfmini_data.distance = dist;
                    tfmini_data.valid    = 1;
                }
                tfmini_data.strength    = strength;
                tfmini_data.temperature = (float)temp_raw / 8.0f - 256.0f; // ℃ = raw/8 - 256
            }
        }
    }
}

int aaaa;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart == &huart1)
	{
		JUDGE_Read_Data(usart1RxBuf);
		Detect_Update(DeviceID_Judge);
	}

	if (huart == &huart2 && huart->ReceptionType == HAL_UART_RECEPTION_STANDARD)
	{
		//启动下一次UART1接收
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1,usart1RxBuf,sizeof(usart1RxBuf));
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx , DMA_IT_HT);


		receive_times ++;
		B2B_ParseUsart();
		Detect_Update(DeviceID_B2B);
		detectList[DeviceID_B2B].isLost = 0;

		B2B_offline_cnt ++;
	}
	//功率计模块
	if (huart == &huart7)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart7, usart7RxBuf, sizeof(usart7RxBuf));
		__HAL_DMA_DISABLE_IT(&hdma_uart7_rx, DMA_IT_HT);
	}
	if (huart == &huart10)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart10, usart10RxBuf, sizeof(usart10RxBuf));
		__HAL_DMA_DISABLE_IT(&hdma_usart10_rx, DMA_IT_HT);
		TFmini_Parse();
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

		if(B2B_time_cnt >= 250)
		{
			if(B2B_offline_cnt == pre_B2B_offline_cnt)
			{
				B2B_offline_flag = 1;
				STOPFLAG = 1;
			}
			pre_B2B_offline_cnt = B2B_offline_cnt;
			B2B_time_cnt = 0;
		}

		B2B_time_cnt ++;
		osDelay(2);
	}
}
#endif
