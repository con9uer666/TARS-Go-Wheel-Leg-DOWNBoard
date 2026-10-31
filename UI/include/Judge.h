#ifndef _JUDGEMENT_H_
#define _JUDGEMENT_H_

#include "main.h"
#include "stdbool.h"

#define    JUDGE_DATA_ERROR      0
#define    JUDGE_DATA_CORRECT    1

#define    LEN_HEADER    5        // 注释已根据代码语义重写。
#define    LEN_CMDID     2        // 注释已根据代码语义重写。
#define    LEN_TAIL      2	      //帧尾CRC16

// 注释已根据代码语义重写。
#define    JUDGE_FRAME_HEADER         (0xA5)

// 注释已根据代码语义重写。
#define JUDGE_MAX_TX_LENGTH     64
// 注释已根据代码语义重写。
#define JUDGE_MAX_RX_LENGTH     1536
// 注释已根据代码语义重写。
#define JUDGE_QUEUE_SIZE 25

// 注释已根据代码语义重写。
typedef enum
{
	RobotColor_Red = 0,
	RobotColor_Blue = 1
}RobotColor;

typedef enum 
{
	FRAME_HEADER         = 0,
	CMD_ID               = 5,
	DATA                 = 7,
}JudgeFrameOffset;

// 注释已根据代码语义重写。
typedef enum
{
	SOF          = 0,// 注释已根据代码语义重写。
	DATA_LENGTH  = 1,// 帧捷筹拷,饺★拷莩
	SEQ          = 3,// 注释已根据代码语义重写。
	CRC8         = 4 //CRC8
	
}FrameHeaderOffset;

/* **************ID******************* */

/* 注释已根据代码语义重写。 */


// 注释已根据代码语义重写。
typedef enum
{
	ID_game_state       				= 0x0001,// 注释已根据代码语义重写。
	ID_game_result 	   					= 0x0002,// 注释已根据代码语义重写。
	ID_game_robot_HP      			= 0x0003,// 注释已根据代码语义重写。
	ID_event_data  							= 0x0101,// 铰硷拷 *
	ID_referee_warning					= 0x0104,// 注释已根据代码语义重写。
	ID_dart_remaining_time			= 0x0105,// 注释已根据代码语义重写。
	ID_game_robot_state    			= 0x0201,// 状态 *
	ID_power_heat_data    			= 0x0202,// 注释已根据代码语义重写。
	ID_game_robot_pos        		= 0x0203,// 位 *
	ID_buff_musk								= 0x0204,// 注释已根据代码语义重写。
	ID_robot_hurt								= 0x0206,// 注释已根据代码语义重写。
	ID_shoot_data								= 0x0207,// 注释已根据代码语义重写。
	ID_bullet_remaining					= 0x0208,// 注释已根据代码语义重写。
	ID_rfid_status							= 0x0209,// RFID状态 *
	ID_sentry_status          	= 0x020D,
} CmdID;



// 注释已根据代码语义重写。
typedef enum
{
	LEN_game_state       					= 11,	//0x0001
	LEN_game_result       				= 1,	//0x0002
	LEN_game_robot_HP							= 16,	//0x0003
	LEN_event_data  							= 4,	//0x0101
	LEN_referee_warning						= 3,	//0x0104
	LEN_dart_remaining_time				= 3,	//0x0105
	LEN_game_robot_state    			= 13,	//0x0201
	LEN_power_heat_data   				= 14,	//0x0202
	LEN_game_robot_pos        		= 12,	//0x0203  
	LEN_buff_musk        					= 8,	//0x0204
	LEN_robot_hurt        				= 1,	//0x0206
	LEN_shoot_data       					= 7,	//0x0207
	LEN_bullet_remaining					= 6,	//0x0208
	LEN_rfid_status								= 5,	//0x0209
	LEN_sentry_status   					= 6,  //0x020D
} JudgeDataLength;


// 注释已根据代码语义重写。
//typedef struct __attribute__((packed))
//{
//	xFrameHeader   							txFrameHeader;//帧头
// uint16_t CmdID;//
// ext_student_interactive_header_data_t dataFrameHeader;//捷讹拷头结构
// robot_interactive_data_t interactData;//捷讹拷
//	uint16_t		 						FrameTail;//帧尾
//}ext_CommunatianData_t;


/* 皆讹拷帧头 */
typedef struct __attribute__((packed)) // 系统帧头
{
	uint8_t  SOF;
	uint16_t DataLength;
	uint8_t  Seq;
	uint8_t  CRC8;
} xFrameHeader;

typedef struct __attribute__((packed)){
	uint16_t data_cmd_id;    
	uint16_t send_ID;    
	uint16_t receiver_ID;	
	uint32_t 		sentry_cmd;
}sentry_cmd_t;



/* ID: 0x0001 Byte: 11 状态 */
typedef struct __attribute__((packed))
{
	uint8_t game_type : 4;
	uint8_t game_progress : 4;
	uint16_t stage_remain_time;
	uint64_t SyncTimeStamp;
} ext_game_status_t;

/* ID: 0x0002 Byte: 1 */
typedef struct __attribute__((packed)) 
{ 
	uint8_t winner;
} ext_game_result_t; 

/* ID: 0x0003 Byte: 32 血 */   // 2026.3.14 侥讹拷
typedef struct __attribute__((packed))
{
	uint16_t ally_1_robot_HP;
	uint16_t ally_2_robot_HP;
	uint16_t ally_3_robot_HP;
	uint16_t ally_4_robot_HP;
	uint16_t reserved;
	uint16_t ally_7_robot_HP;
	uint16_t ally_outpost_HP;
	uint16_t ally_base_HP;
} ext_game_robot_HP_t;
 
/* ID: 0x0101 Byte: 4 铰硷拷 */
typedef struct __attribute__((packed)) 					// 注释已根据代码语义重写。
{ 	
	uint32_t event_data;
	
	// 角讹拷皆讹拷峁癸拷
	// 0未占/未 1占/窖硷拷
	/* 注释已根据代码语义重写。 */
	
//	uint8_t supply_area_state : 3;
//	uint8_t own_buff_state : 4;
//	uint8_t own_central_land_state : 2;
//	uint8_t own_trapezoidal_land_state : 2;
//	uint16_t darts_hitted_time : 9;
//	uint8_t darts_hitted_target : 3;
//	uint8_t center_gain_state : 2;
//	uint8_t own_fortress_state :2;
// uint8_t own_outpost_state : 2; // bit27-28 前站阶刺
// uint8_t own_base_state : 1; // bit29 阶刺
//	
//	uint32_t reserved : 2;
	
} ext_event_data_t; 



/* 注释已根据代码语义重写。 */
typedef struct __attribute__((packed))
{
  uint8_t level; 
  uint8_t offending_robot_id; 
  uint8_t count; 
} ext_referee_warning_t;

/* 注释已根据代码语义重写。 */
typedef struct __attribute__((packed)) 
{ 
  uint8_t dart_remaining_time; 
  uint16_t dart_info; 
}ext_dart_info_t; 

/* ID: 0X0201 Byte: 13 状态 */
typedef struct __attribute__((packed))
{
	uint8_t robot_id; 
  uint8_t robot_level; 
  uint16_t current_HP;  
  uint16_t maximum_HP; 
  uint16_t shooter_barrel_cooling_value; 
  uint16_t shooter_barrel_heat_limit; 
  uint16_t chassis_power_limit;  
  uint8_t power_management_gimbal_output : 1; 
  uint8_t power_management_chassis_output : 1;  
  uint8_t power_management_shooter_output : 1;
} ext_game_robot_status_t;


/* ID: 0X0202 Byte: 16 实时 */
typedef struct __attribute__((packed))
{
	uint16_t reserved_1;
	uint16_t reserved_2;
	float reserved_3;
	uint16_t chassis_power_buffer;
	uint16_t shooter_id1_17mm_cooling_heat;
	uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;


/* ID: 0x0203 Byte: 12 位 */
typedef struct __attribute__((packed)) 
{ 
  float x; 
  float y; 
  float angle; 
} ext_game_robot_pos_t; 


/* ID: 0x0204 Byte: 8 */
typedef struct __attribute__((packed)) 
{ 
  uint8_t recovery_buff;  
  uint16_t cooling_buff;  
  uint8_t defence_buff;  
  uint8_t vulnerability_buff; 
  uint16_t attack_buff; 
	uint8_t remaining_energy; 
} ext_buff_musk_t; 


/* 注释已根据代码语义重写。 */
typedef struct __attribute__((packed)) 
{ 
	uint8_t armor_id : 4; 
	uint8_t hurt_type : 4; 
} ext_robot_hurt_t; 


/* ID: 0x0207 Byte: 7 实时 */
typedef struct __attribute__((packed))
{
	uint8_t bullet_type;  
  uint8_t shooter_number; 
  uint8_t launching_frequency;  
  float initial_speed; 
} ext_shoot_data_t;

/* 注释已根据代码语义重写。 */
typedef struct __attribute__((packed))
{
	uint16_t projectile_allowance_17mm; 
  uint16_t projectile_allowance_42mm;  
  uint16_t remaining_gold_coin; 
  uint16_t projectile_allowance_fortress;
} ext_bullet_remaining_t;

/* ID: 0x0209 Byte: 5 RFID状态 */ // 注释已根据代码语义重写。
typedef struct __attribute__((packed))
{
	uint32_t rfid_status;
	uint8_t rfid_status_2;
} ext_rfid_status_t;

/* 注释已根据代码语义重写。 */
typedef struct __attribute__((packed))
{
	uint8_t dart_launch_opening_status;  
  uint8_t reserved;  
  uint16_t target_change_time;  
  uint16_t latest_launch_cmd_time; 
} ext_dart_client_cmd_t;	

/* 注释已根据代码语义重写。 */
typedef struct __attribute__((packed))
{
	uint32_t sentry_info;
	uint16_t sentry_info_2;
}sentry_info_t;

/* 注释已根据代码语义重写。 */
/* 捷斤拷息0x0301 */
typedef struct __attribute__((packed)) 	// 注释已根据代码语义重写。
{ 
	uint16_t data_cmd_id;    
	uint16_t send_ID;    
	uint16_t receiver_ID; 
} ext_student_interactive_header_data_t; 
/* 注释已根据代码语义重写。 */
typedef struct __attribute__((packed)) 
{ 
	uint8_t data[106]; // 捷讹拷,n要小113
} robot_interactive_data_t;



// 注释已根据代码语义重写。
typedef struct __attribute__((packed))
{
	xFrameHeader   							txFrameHeader;//帧头
	uint16_t								CmdID;// 注释已根据代码语义重写。
	ext_student_interactive_header_data_t   dataFrameHeader;// 捷讹拷头结构
	robot_interactive_data_t  	 			interactData;// 捷讹拷
	uint16_t		 						FrameTail;//帧尾
}ext_CommunatianData_t;



// 系统帧
typedef struct
{
	uint8_t data[JUDGE_MAX_TX_LENGTH];
	uint16_t frameLength;
}JudgeTxFrame;

typedef struct __attribute__((packed)){
	uint8_t game_progress; // 前状态 0:未始 1:准阶讹拷 2:皆硷拷锥 3:诫倒时 4: 5
	uint16_t remain_time;  // 剩时 位:s
	uint16_t current_hp;   // 注释已根据代码语义重写。
	uint16_t projectile;   // 注释已根据代码语义重写。
	uint8_t sentry_info;   // bit 0: 装阶帮拷角否被癸拷 0: 1
												 // 注释已根据代码语义重写。
												 // 注释已根据代码语义重写。
													 // 注释已根据代码语义重写。
													 // 注释已根据代码语义重写。
													 // 注释已根据代码语义重写。
													 // bit 6-7: 0
													 // building state
	uint16_t red_outpost_hp;  // 届方前站血
	uint16_t red_base_hp;     // 届方血
	uint16_t blue_outpost_hp; // 前站血
	uint16_t blue_base_hp;    // 注释已根据代码语义重写。
/* **********为ai************* */
	uint16_t shooter_barrel_cooling_value;  // 却劫讹拷
	uint16_t shooter_barrel_heat_limit; 		// 注释已根据代码语义重写。
	uint8_t power_management_gimbal_output : 1;
	uint8_t power_management_chassis_output : 1;
	uint8_t power_management_shooter_output : 1;
	uint16_t shooter_17mm_barrel_heat;			// 注释已根据代码语义重写。
	float initial_speed;										// 注释已根据代码语义重写。
	uint8_t self_color;
}JudgeData_t;

/* ***************************** */
void JUDGE_Init(void);
bool JUDGE_Read_Data(uint8_t *ReadFromUsart);
RobotColor JUDGE_GetSelfColor(void);
uint8_t JUDGE_GetSelfID(void);
uint16_t JUDGE_GetClientID(void);
bool JUDGE_IsValid(void);
void JUDGE_GetPosition(float *x,float *y);
uint8_t JUDGE_GetChassisPowerLimit(void);
bool JUDGE_GetShooterOutputState(void);
bool JUDGE_GetGimbalOutputState(void);
uint16_t JUDGE_GetHeatLimit(void);
uint16_t JUDGE_GetShootSpeedLimit(void);
uint16_t JUDGE_GetPowerBuffer(void);
int16_t JUDGE_GetRemainHeat(void);
uint16_t JUDGE_GetRemain_42_Num(void);
uint8_t HP_deduction_reason(void);
uint16_t JUDGE_GetHP(void);
uint16_t JUDGE_GetCoolingValue(void);


// 注释已根据代码语义重写。
void USER_USART1_IRQHandler(void);
// 注释已根据代码语义重写。
void Task_Judge_Callback(void);
// 注释已根据代码语义重写。
void Judge_UartLostCallback(void);

void Judge_Receive(void);
/* ***************解部************** */
extern ext_game_robot_pos_t			GameRobotPos;
extern ext_shoot_data_t				ShootData;
extern ext_game_robot_status_t		GameRobotStat;
extern JudgeData_t 					USER_JudgeData;
extern ext_power_heat_data_t 		PowerHeatData;

#endif // 头侥硷拷
