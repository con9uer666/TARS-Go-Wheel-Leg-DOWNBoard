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
#include "wattmeter.h"
#include "buzzer.h"
#include "Self_Righting.h"

extern uint8_t first_run;
extern uint8_t start_mode;
extern uint8_t gimbal_follow_flag;
extern float down_board_yaw_output;

uint16_t can_send_error,can_receive_error;

/*====================================== 电机输出总开关 ====================================== */
uint8_t motor_output_enable = 1;      // 1=正常输出力矩, 0=强制全部电机输出零力矩
uint8_t wheel_leg_output_enable = 1;  // 1=正常输出, 0=仅关断3508和8009（4310/2325不受影响）

/*====================================== 电机使能监督 ======================================== */
// 记录机身所有DM关节电机当前期望状态：1=应使能 0=应失能
// 由task_Motor_Enable完成后置1
uint8_t motor_should_enabled = 0;

// 最近一次在任意DM电机反馈帧里读到的错误码（latest-wins，后来者覆盖前者）
// 0 表示从未读到过错误；非0 = 最近一次的State错误码（见 is_dm_error_state 注释）
// 一旦置为非0，会驱动错误码蜂鸣器一直响（除非倒地自起占用蜂鸣器）。
// 设计上不自动清零（用户要求"一直不停"），仅在掉电时归零。
uint8_t motor_last_error_code = 0;

#define DM_SUPERVISOR_INTERVAL_MS  100  // 每个电机最多每100ms下达一次恢复指令

// 每个被监督电机上次下达恢复指令的tick
static uint32_t last_action_L_DM8009_0   = 0;
static uint32_t last_action_L_DM8009_1   = 0;
static uint32_t last_action_R_DM8009_0   = 0;
static uint32_t last_action_R_DM8009_1   = 0;
static uint32_t last_action_Yaw_4310     = 0;
static uint32_t last_action_Shooter_2325 = 0;

// 轮询计数：每轮检查一个电机，避免一次性塞满FDCAN TX FIFO
static uint8_t supervisor_round = 0;

// DM电机State字段错误码判定（兼容 DM4310/DM8009/DM2325）：
//   0=失能, 1=使能（正常）
//   3/4/5 = DM4310 传感器相关错误（编码器失校准/磁场干扰等）
//   5 = DM2325 传感器错误
//   6 = DM2325 电机参数错误
//   8=过压, 9=欠压, A=过流, B=MOS过温, C=线圈过温, D=通信丢失, E=过载
static uint8_t is_dm_error_state(uint8_t state)
{
    return (state == 3) || (state == 4) || (state == 5) || (state == 6) || (state >= 8);
}

// 监督单个DM关节电机：错误→记录错误码+清错+按期望状态再发使能/失能；状态与期望不一致→纠正
static void DM_Motor_State_Supervisor(FDCAN_HandleTypeDef *hfdcan, Joint_Motor_t *motor, uint32_t *last_action)
{
    uint32_t now = xTaskGetTickCount();
    if ((now - *last_action) < DM_SUPERVISOR_INTERVAL_MS) return;

    uint8_t state = motor->Rx_Data.State;

    if (is_dm_error_state(state))
    {
        // 先把错误码寄存进全局变量（latest-wins），再清错——用户要求的顺序
        motor_last_error_code = state;
        Clear_DM_Motor_Error(hfdcan, motor->motor_id);
        if (motor_should_enabled)
            Enable_DM_Motor_MIT(hfdcan, motor->motor_id);
        else
            Disable_DM_Motor(hfdcan, motor->motor_id);
        *last_action = now;
        return;
    }

    if (motor_should_enabled && state == 0)
    {
        Enable_DM_Motor_MIT(hfdcan, motor->motor_id);
        *last_action = now;
        return;
    }

    if (!motor_should_enabled && state == 1)
    {
        Disable_DM_Motor(hfdcan, motor->motor_id);
        *last_action = now;
        return;
    }
}

// 每个CAN_Transmit循环调用一次，轮询监督6个DM关节电机中的1个
static void DM_Motor_State_Supervisor_Tick(void)
{
    switch (supervisor_round)
    {
        case 0: DM_Motor_State_Supervisor(&hfdcan2, &L_DM8009[0],    &last_action_L_DM8009_0);   break;
        case 1: DM_Motor_State_Supervisor(&hfdcan2, &L_DM8009[1],    &last_action_L_DM8009_1);   break;
        case 2: DM_Motor_State_Supervisor(&hfdcan1, &R_DM8009[0],    &last_action_R_DM8009_0);   break;
        case 3: DM_Motor_State_Supervisor(&hfdcan1, &R_DM8009[1],    &last_action_R_DM8009_1);   break;
        case 4: DM_Motor_State_Supervisor(&hfdcan3, &Yaw_DM4310,     &last_action_Yaw_4310);     break;
        case 5: DM_Motor_State_Supervisor(&hfdcan3, &Shooter_DM2325, &last_action_Shooter_2325); break;
        default: break;
    }
    supervisor_round = (supervisor_round + 1) % 6;
}

/*====================================== 错误码蜂鸣器仲裁 ====================================== */
// 错误码蜂鸣器频率：用最低的可清晰发声音高（远低于自起的 do/sol/高mi/高si 与完成提示的高si）
#define ERROR_BUZZER_FREQ_HZ  200

// 由 Motor_task 周期调用：当 motor_last_error_code != 0 时持续驱动蜂鸣器以最低音高长响。
// 倒地自起占用蜂鸣器期间（g_tip_recovery_active==1）或跳跃中（g_jump_buzzer_active==1），
// 本函数完全不许碰蜂鸣器（不调用 Buzzer_Tone_Max 也不调用 Stop_Buzzer），把音频独占权交出去。
void Error_Buzzer_Tick(void)
{
    if (g_tip_recovery_active) return;   // 倒地自起绝对优先：连 Stop 都不发
    if (g_jump_buzzer_active)  return;   // 跳跃期间让位

    if (motor_last_error_code != 0)
    {
        Buzzer_Tone_Max(ERROR_BUZZER_FREQ_HZ);
    }
}

/*====================================== 心跳检测 =========================================== */
#define HEARTBEAT_CHECK_INTERVAL_MS  10    // 每10ms检测一次

// 每个电机独立的CAN数据接收计数器（每次收到消息时在中断回调中自增）
// 去掉 static 以便 UI_Task.c 通过 extern 读取做断联指示
volatile uint32_t rx_cnt_L_DM8009_0 = 0;  // L_DM8009[0]
volatile uint32_t rx_cnt_L_DM8009_1 = 0;  // L_DM8009[1]
volatile uint32_t rx_cnt_R_DM8009_0 = 0;  // R_DM8009[0]
volatile uint32_t rx_cnt_R_DM8009_1 = 0;  // R_DM8009[1]
volatile uint32_t rx_cnt_L_DJ3508    = 0;  // L_DJ3508
volatile uint32_t rx_cnt_R_DJ3508    = 0;  // R_DJ3508
volatile uint32_t rx_cnt_4310        = 0;  // Yaw_DM4310
volatile uint32_t rx_cnt_2325        = 0;  // Shooter_DM2325
volatile uint32_t rx_cnt_wattmeter   = 0;  // WattMeter (can3, 0x213)

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
			case 0x213:
			{
				WattMeter_CAN_Parse(rx_data);
				rx_cnt_wattmeter++;
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

		/*========== DM电机使能/错误监督（每轮检查一个） ==========*/
		DM_Motor_State_Supervisor_Tick();

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
			// DM_Motor_MIT_Speed_ctrl(&hfdcan3, Yaw_DM4310, 0, 0, 0, 0, 0);
			DM_Motor_MIT_Torque_ctrl(&hfdcan3, Yaw_DM4310, 0);
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
				// 8009/3508正常，但受wheel_leg_output_enable控制
				if (!wheel_leg_output_enable)
				{
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
					DJI_Motor_Torque_Ctrl(&hfdcan2, 0x200, -L_DJ3508.Target_Torque);
					DJI_Motor_Torque_Ctrl(&hfdcan1, 0x1FF, R_DJ3508.Target_Torque);
					osDelay(1);
					DM_Motor_MIT_Torque_ctrl(&hfdcan2, L_DM8009[1], VMC_L.T1);
					DM_Motor_MIT_Torque_ctrl(&hfdcan1, R_DM8009[1], VMC_R.T2);
					DM_Motor_MIT_Torque_ctrl(&hfdcan2, L_DM8009[0], VMC_L.T2);
					DM_Motor_MIT_Torque_ctrl(&hfdcan1, R_DM8009[0], VMC_R.T1);
				}
			}
			osDelay(1);
			// 4310正常时，始终发送4310指令
			if(gimbal_follow_flag == 1)
			{
				DM_Motor_MIT_Speed_ctrl(&hfdcan3, Yaw_DM4310, 0, down_board_yaw_output, 0, 0, 2);
			}
			else
			{
				DM_Motor_MIT_Torque_ctrl(&hfdcan3, Yaw_DM4310, Yaw_DM4310.Target_Torque);
			}
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
