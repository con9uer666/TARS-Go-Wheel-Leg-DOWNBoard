#ifndef _JUDGEMENT_H_
#define _JUDGEMENT_H_

#include "main.h"
#include "stdbool.h"

#define    JUDGE_DATA_ERROR      0
#define    JUDGE_DATA_CORRECT    1

#define    LEN_HEADER    5        //帧头锟斤拷
#define    LEN_CMDID     2        //锟斤拷锟斤拷锟诫长锟斤拷
#define    LEN_TAIL      2	      //帧尾CRC16

//锟斤拷始锟街斤拷,协锟斤拷潭锟轿�0xA5
#define    JUDGE_FRAME_HEADER         (0xA5)

//锟斤拷锟酵伙拷锟斤拷锟斤拷锟斤拷锟斤拷
#define JUDGE_MAX_TX_LENGTH     64
//锟斤拷锟秸伙拷锟斤拷锟斤拷锟斤拷锟斤拷
#define JUDGE_MAX_RX_LENGTH     1536
//锟斤拷锟酵讹拷锟叫筹拷锟斤拷
#define JUDGE_QUEUE_SIZE 25

//锟斤拷锟斤拷锟斤拷锟斤拷色
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

//5锟街斤拷帧头,偏锟斤拷位锟斤拷
typedef enum
{
	SOF          = 0,//锟斤拷始位
	DATA_LENGTH  = 1,//帧锟斤拷锟斤拷锟捷筹拷锟斤拷,锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟饺★拷锟斤拷莩锟斤拷锟�
	SEQ          = 3,//锟斤拷锟斤拷锟�
	CRC8         = 4 //CRC8
	
}FrameHeaderOffset;

/***************锟斤拷锟斤拷锟斤拷ID********************/

/* 

	ID: 0x0001  Byte:  11    锟斤拷锟斤拷状态锟斤拷锟斤拷       			锟斤拷锟斤拷频锟斤拷 1Hz      
	ID: 0x0002  Byte:  1    锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟�         		锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷      
	ID: 0x0003  Byte:  16    锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷血锟斤拷锟斤拷锟斤拷   		1Hz锟斤拷锟斤拷    //2026.3.14锟睫革拷 16byte
	ID: 0x0101  Byte:  4    锟斤拷锟斤拷锟铰硷拷锟斤拷锟斤拷   				锟铰硷拷锟侥憋拷锟斤拷锟�
	ID: 0x0104	Byte: 	3		锟斤拷锟叫撅拷锟斤拷锟斤拷息				//2026.3.14锟睫革拷 3byte
	ID: 0x0105	Byte: 	3		锟斤拷锟节凤拷锟斤拷诘锟斤拷锟绞�		//2026.3.14锟睫革拷 3byte
	ID: 0X0201  Byte: 13    锟斤拷锟斤拷锟斤拷状态锟斤拷锟斤拷        		10Hz		//2026.3.14锟睫革拷 13byte
	ID: 0X0202  Byte: 14    实时锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷   			50Hz       
	ID: 0x0203  Byte: 12    锟斤拷锟斤拷锟斤拷位锟斤拷锟斤拷锟斤拷           	10Hz
	ID: 0x0204  Byte:  8    锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷           	锟斤拷锟斤拷状态锟侥憋拷锟斤拷锟�
	ID: 0x0206  Byte:  1    锟剿猴拷状态锟斤拷锟斤拷           		锟剿猴拷锟斤拷锟斤拷锟斤拷锟斤拷
	ID: 0x0207  Byte:  7    实时锟斤拷锟斤拷锟斤拷锟�           		锟接碉拷锟斤拷锟斤拷锟斤拷锟�
	ID: 0x0208  Byte:  8    锟接碉拷剩锟洁发锟斤拷锟斤拷					//2026.3.14锟睫革拷 6byte
	ID: 0x0209  Byte:  5    锟斤拷锟斤拷锟斤拷RFID状态					//2026.3.14锟睫革拷 5byte
	ID: 0x020A  Byte:  6    锟斤拷锟节伙拷锟斤拷锟剿客伙拷锟斤拷指锟斤拷锟斤拷锟斤拷
	ID: 0x020D Byte:  6		锟节憋拷锟斤拷锟斤拷锟剿碉拷前状态
	ID: 0x0301  Byte:  118    锟斤拷锟斤拷锟剿间交锟斤拷锟斤拷锟斤拷           	锟斤拷锟酵凤拷锟斤拷锟斤拷锟斤拷锟斤拷,10Hz  //2026.3.14锟睫革拷 16byte
*/


//锟斤拷锟斤拷锟斤拷ID,锟斤拷锟斤拷锟叫断斤拷锟秸碉拷锟斤拷什么锟斤拷锟斤拷
typedef enum
{
	ID_game_state       				= 0x0001,//锟斤拷锟斤拷状态锟斤拷锟斤拷
	ID_game_result 	   					= 0x0002,//锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟�
	ID_game_robot_HP      			= 0x0003,//锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷血锟斤拷锟斤拷锟斤拷
	ID_event_data  							= 0x0101,//锟斤拷锟斤拷锟铰硷拷锟斤拷锟斤拷 *
	ID_referee_warning					= 0x0104,//锟斤拷锟叫撅拷锟斤拷锟斤拷息
	ID_dart_remaining_time			= 0x0105,//锟斤拷锟节凤拷锟斤拷诘锟斤拷锟绞�
	ID_game_robot_state    			= 0x0201,//锟斤拷锟斤拷锟斤拷状态锟斤拷锟斤拷  *
	ID_power_heat_data    			= 0x0202,//实时锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
	ID_game_robot_pos        		= 0x0203,//锟斤拷锟斤拷锟斤拷位锟斤拷锟斤拷锟斤拷  *
	ID_buff_musk								= 0x0204,//锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷//
	ID_robot_hurt								= 0x0206,//锟剿猴拷状态锟斤拷锟斤拷
	ID_shoot_data								= 0x0207,//实时锟斤拷锟斤拷锟斤拷锟�
	ID_bullet_remaining					= 0x0208,//锟接碉拷剩锟洁发锟斤拷锟斤拷
	ID_rfid_status							= 0x0209,//锟斤拷锟斤拷锟斤拷RFID状态  *	
	ID_sentry_status          	= 0x020D,
} CmdID;



//锟斤拷锟斤拷锟斤拷锟斤拷锟捷段筹拷,锟斤拷锟捷官凤拷协锟斤拷锟斤拷锟斤拷锟藉长锟斤拷
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


////锟斤拷锟斤拷锟剿斤拷锟斤拷锟斤拷息
//typedef __packed struct
//{
//	xFrameHeader   							txFrameHeader;//帧头
//	uint16_t								CmdID;//锟斤拷锟斤拷锟斤拷
//	ext_student_interactive_header_data_t   dataFrameHeader;//锟斤拷锟捷讹拷头锟结构
//	robot_interactive_data_t  	 			interactData;//锟斤拷锟捷讹拷
//	uint16_t		 						FrameTail;//帧尾
//}ext_CommunatianData_t;


/* 锟皆讹拷锟斤拷帧头 */
typedef __packed struct //锟斤拷锟斤拷系统帧头
{
	uint8_t  SOF;
	uint16_t DataLength;
	uint8_t  Seq;
	uint8_t  CRC8;
} xFrameHeader;

typedef __packed struct{
	uint16_t data_cmd_id;    
	uint16_t send_ID;    
	uint16_t receiver_ID;	
	uint32_t 		sentry_cmd;
}sentry_cmd_t;



/* ID: 0x0001  Byte:  11    锟斤拷锟斤拷状态锟斤拷锟斤拷 */
typedef __packed struct
{
	uint8_t game_type : 4;
	uint8_t game_progress : 4;
	uint16_t stage_remain_time;
	uint64_t SyncTimeStamp;
} ext_game_status_t;

/* ID: 0x0002  Byte:  1    锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟� */
typedef __packed struct 
{ 
	uint8_t winner;
} ext_game_result_t; 

/* ID: 0x0003  Byte:  32    锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷血锟斤拷锟斤拷锟斤拷 */   //2026.3.14 锟侥讹拷
typedef __packed struct
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
 
/* ID: 0x0101  Byte:  4    锟斤拷锟斤拷锟铰硷拷锟斤拷锟斤拷 */
typedef __packed struct 					//2026.3.14锟睫革拷
{ 	
	uint32_t event_data;
	
	//锟斤拷锟斤拷锟角讹拷锟斤拷锟皆讹拷锟斤拷峁癸拷锟� 
	//0锟斤拷未占锟斤拷/未锟斤拷锟斤拷  1锟斤拷锟斤拷占锟斤拷/锟窖硷拷锟斤拷 
	/*bit 0-2锟斤拷 
		bit 0锟斤拷锟斤拷锟斤拷锟斤拷一锟斤拷锟斤拷锟斤拷氐锟斤拷牟锟斤拷锟斤拷锟秸硷拷锟阶刺拷锟�1为锟斤拷占锟斤拷 
		bit 1锟斤拷锟斤拷锟斤拷锟斤拷一锟斤拷锟斤拷氐锟斤拷牟锟斤拷锟斤拷锟秸硷拷锟阶刺拷锟�1为锟斤拷占锟斤拷 
		bit 2锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷占锟斤拷状态锟斤拷1为锟斤拷占锟届（锟斤拷 RMUL 锟斤拷锟矫ｏ拷 
		bit 3-6锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷状态 
		bit 3-4锟斤拷锟斤拷锟斤拷小锟斤拷锟斤拷锟斤拷锟截的硷拷锟斤拷状态锟斤拷0为未锟斤拷锟筋，1为锟窖硷拷锟筋，2为锟斤拷锟节硷拷锟斤拷
		bit 5-6锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟截的硷拷锟斤拷状态锟斤拷0为未锟斤拷锟筋，1为锟窖硷拷锟筋，2为锟斤拷锟节硷拷锟斤拷
		bit 7-8锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷叩氐锟秸硷拷锟阶刺拷锟�1为锟斤拷锟斤拷锟斤拷占锟届，2为锟斤拷锟皆凤拷占锟斤拷 
		bit 9-10锟斤拷锟斤拷锟斤拷锟斤拷锟轿高地碉拷占锟斤拷状态锟斤拷1为锟斤拷占锟斤拷 
		bit 11-19锟斤拷锟皆凤拷锟斤拷锟斤拷锟斤拷锟揭伙拷位锟斤拷屑锟斤拷锟角帮拷锟秸撅拷锟斤拷锟截碉拷时锟戒（0-420锟斤拷锟斤拷锟斤拷默锟斤拷为0锟斤拷 
		bit 20-22锟斤拷锟皆凤拷锟斤拷锟斤拷锟斤拷锟揭伙拷位锟斤拷屑锟斤拷锟角帮拷锟秸撅拷锟斤拷锟截的撅拷锟斤拷目锟疥，锟斤拷锟斤拷默锟斤拷为0锟斤拷
							 1为锟斤拷锟斤拷前锟斤拷站锟斤拷2为锟斤拷锟叫伙拷锟截固讹拷目锟疥，3为锟斤拷锟叫伙拷锟斤拷锟斤拷锟�
							 锟教讹拷目锟疥，4为锟斤拷锟叫伙拷锟斤拷锟斤拷锟斤拷贫锟侥匡拷锟� 5为锟斤拷锟叫伙拷锟斤拷末锟斤拷锟狡讹拷目锟斤拷
		bit 23-24锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷占锟斤拷状态锟斤拷0为未锟斤拷占锟届，1为锟斤拷锟斤拷锟斤拷占锟届，2
							 为锟斤拷锟皆凤拷占锟届，3为锟斤拷双锟斤拷占锟届。锟斤拷锟斤拷RMUL锟斤拷锟矫ｏ拷 
		bit 25-26锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷占锟斤拷状态锟斤拷0为未锟斤拷占锟届，1为锟斤拷锟斤拷锟斤拷占
							 锟届，2为锟斤拷锟皆凤拷占锟届，3为锟斤拷双锟斤拷占锟届。
		bit 27-28锟斤拷锟斤拷锟斤拷前锟斤拷站锟斤拷锟斤拷锟斤拷占锟斤拷状态锟斤拷 0 为未锟斤拷占锟届， 1 为锟斤拷锟斤拷锟斤拷
							 占锟届， 2 为锟斤拷锟皆凤拷占锟斤拷
		bit 29锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷占锟斤拷状态锟斤拷 1 为锟斤拷占锟斤拷							 */
	
//	uint8_t supply_area_state : 3;
//	uint8_t own_buff_state : 4;
//	uint8_t own_central_land_state : 2;
//	uint8_t own_trapezoidal_land_state : 2;
//	uint16_t darts_hitted_time : 9;
//	uint8_t darts_hitted_target : 3;
//	uint8_t center_gain_state : 2;
//	uint8_t own_fortress_state :2;
//	uint8_t own_outpost_state : 2;   // bit27-28 锟斤拷锟斤拷前锟斤拷站锟斤拷锟斤拷锟阶刺�
//	uint8_t own_base_state : 1;      // bit29 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟阶刺�
//	
//	uint32_t reserved : 2;
	
} ext_event_data_t; 



/* ID: 0x104    Byte: 3    锟斤拷锟叫撅拷锟斤拷锟斤拷息 */
typedef __packed struct
{
  uint8_t level; 
  uint8_t offending_robot_id; 
  uint8_t count; 
} ext_referee_warning_t;

/* ID: 0x105    Byte: 3    锟斤拷锟节凤拷锟斤拷诘锟斤拷锟绞� */  
typedef __packed struct 
{ 
  uint8_t dart_remaining_time; 
  uint16_t dart_info; 
}ext_dart_info_t; 

/* ID: 0X0201  Byte: 13    锟斤拷锟斤拷锟斤拷状态锟斤拷锟斤拷 */
typedef __packed struct
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


/* ID: 0X0202  Byte: 16    实时锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷 */
typedef __packed struct
{
	uint16_t reserved_1;
	uint16_t reserved_2;
	float reserved_3;
	uint16_t chassis_power_buffer;
	uint16_t shooter_id1_17mm_cooling_heat;
	uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;


/* ID: 0x0203  Byte: 12    锟斤拷锟斤拷锟斤拷位锟斤拷锟斤拷锟斤拷 */
typedef __packed struct 
{ 
  float x; 
  float y; 
  float angle; 
} ext_game_robot_pos_t; 


/* ID: 0x0204  Byte:  8    锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷 */
typedef __packed struct 
{ 
  uint8_t recovery_buff;  
  uint16_t cooling_buff;  
  uint8_t defence_buff;  
  uint8_t vulnerability_buff; 
  uint16_t attack_buff; 
	uint8_t remaining_energy; 
} ext_buff_musk_t; 


/* ID: 0x0206  Byte:  2    锟剿猴拷状态锟斤拷锟斤拷 */
typedef __packed struct 
{ 
	uint8_t armor_id : 4; 
	uint8_t hurt_type : 4; 
} ext_robot_hurt_t; 


/* ID: 0x0207  Byte:  7    实时锟斤拷锟斤拷锟斤拷锟� */
typedef __packed struct
{
	uint8_t bullet_type;  
  uint8_t shooter_number; 
  uint8_t launching_frequency;  
  float initial_speed; 
} ext_shoot_data_t;

/* ID: 0x0208  Byte:  8   锟接碉拷剩锟洁发锟斤拷锟斤拷 */
typedef __packed struct
{
	uint16_t projectile_allowance_17mm; 
  uint16_t projectile_allowance_42mm;  
  uint16_t remaining_gold_coin; 
  uint16_t projectile_allowance_fortress;
} ext_bullet_remaining_t;

/* ID: 0x0209  Byte:  5    锟斤拷锟斤拷锟斤拷RFID状态 */ //2026.3.14锟睫革拷
typedef __packed struct
{
	uint32_t rfid_status;
	uint8_t rfid_status_2;
} ext_rfid_status_t;

/* ID: 0x020A  Byte:  6    锟斤拷锟节伙拷锟斤拷锟剿客伙拷锟斤拷指锟斤拷锟斤拷锟斤拷 */
typedef __packed struct
{
	uint8_t dart_launch_opening_status;  
  uint8_t reserved;  
  uint16_t target_change_time;  
  uint16_t latest_launch_cmd_time; 
} ext_dart_client_cmd_t;	

/* ID: 0x020D  Byte:  6    锟节憋拷锟斤拷锟斤拷锟剿碉拷前状态锟斤拷锟斤拷 */
typedef __packed struct
{
	uint32_t sentry_info;
	uint16_t sentry_info_2;
}sentry_info_t;

/* 
	
	锟斤拷锟斤拷锟斤拷锟捷ｏ拷锟斤拷锟斤拷一锟斤拷统一锟斤拷锟斤拷锟捷讹拷头锟结构锟斤拷
	锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷 ID锟斤拷锟斤拷锟斤拷锟斤拷锟皆硷拷锟斤拷锟斤拷锟竭碉拷 ID 锟斤拷锟斤拷锟斤拷锟斤拷锟捷段ｏ拷
	锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟捷的帮拷锟杰癸拷锟斤拷锟斤拷锟轿� 128 锟斤拷锟街节ｏ拷
	锟斤拷去 frame_header,cmd_id,frame_tail 锟皆硷拷锟斤拷锟捷讹拷头锟结构锟斤拷 6 锟斤拷锟街节ｏ拷
	锟绞讹拷锟斤拷锟酵碉拷锟斤拷锟斤拷锟斤拷锟捷讹拷锟斤拷锟轿� 113锟斤拷
	锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷 0x0301 锟侥帮拷锟斤拷锟斤拷频锟斤拷为 10Hz锟斤拷

	锟斤拷锟斤拷锟斤拷 ID锟斤拷
	1锟斤拷英锟斤拷(锟斤拷)锟斤拷
	2锟斤拷锟斤拷锟斤拷(锟斤拷)锟斤拷
	3/4/5锟斤拷锟斤拷锟斤拷(锟斤拷)锟斤拷
	6锟斤拷锟斤拷锟斤拷(锟斤拷)锟斤拷
	7锟斤拷锟节憋拷(锟斤拷)锟斤拷
	11锟斤拷英锟斤拷(锟斤拷)锟斤拷
	12锟斤拷锟斤拷锟斤拷(锟斤拷)锟斤拷
	13/14/15锟斤拷锟斤拷锟斤拷(锟斤拷)锟斤拷
	16锟斤拷锟斤拷锟斤拷(锟斤拷)锟斤拷
	17锟斤拷锟节憋拷(锟斤拷)锟斤拷 
	锟酵伙拷锟斤拷 ID锟斤拷 
	0x0101 为英锟桔诧拷锟斤拷锟街客伙拷锟斤拷( 锟斤拷) 锟斤拷
	0x0102 锟斤拷锟斤拷锟教诧拷锟斤拷锟街客伙拷锟斤拷 ((锟斤拷 )锟斤拷
	0x0103/0x0104/0x0105锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟街客伙拷锟斤拷(锟斤拷)锟斤拷
	0x0106锟斤拷锟斤拷锟叫诧拷锟斤拷锟街客伙拷锟斤拷((锟斤拷)锟斤拷 
	0x0111锟斤拷英锟桔诧拷锟斤拷锟街客伙拷锟斤拷(锟斤拷)锟斤拷
	0x0112锟斤拷锟斤拷锟教诧拷锟斤拷锟街客伙拷锟斤拷(锟斤拷)锟斤拷
	0x0113/0x0114/0x0115锟斤拷锟斤拷锟斤拷锟街客伙拷锟剿诧拷锟斤拷(锟斤拷)锟斤拷
	0x0116锟斤拷锟斤拷锟叫诧拷锟斤拷锟街客伙拷锟斤拷(锟斤拷)锟斤拷 
*/
/* 锟斤拷锟斤拷锟斤拷锟捷斤拷锟斤拷锟斤拷息锟斤拷0x0301  */
typedef __packed struct 	//2026.3.14锟睫革拷
{ 
	uint16_t data_cmd_id;    
	uint16_t send_ID;    
	uint16_t receiver_ID; 
} ext_student_interactive_header_data_t; 
/* 
	学锟斤拷锟斤拷锟斤拷锟剿硷拷通锟斤拷 cmd_id 0x0301锟斤拷锟斤拷锟斤拷 ID:0x0200~0x02FF
	锟斤拷锟斤拷锟斤拷锟斤拷 锟斤拷锟斤拷锟剿硷拷通锟脚ｏ拷0x0301锟斤拷
	锟斤拷锟斤拷频锟绞ｏ拷锟斤拷锟斤拷 10Hz  

	锟街斤拷偏锟斤拷锟斤拷 	锟斤拷小 	说锟斤拷 			锟斤拷注 
	0 			2 		锟斤拷锟捷碉拷锟斤拷锟斤拷 ID 	0x0200~0x02FF 
										锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷 ID 锟斤拷选取锟斤拷锟斤拷锟斤拷 ID 锟斤拷锟斤拷锟缴诧拷锟斤拷锟斤拷锟皆讹拷锟斤拷 
	
	2 			2 		锟斤拷锟斤拷锟竭碉拷 ID 	锟斤拷要校锟介发锟斤拷锟竭碉拷 ID 锟斤拷确锟皆ｏ拷 
	
	4 			2 		锟斤拷锟斤拷锟竭碉拷 ID 	锟斤拷要校锟斤拷锟斤拷锟斤拷叩锟� ID 锟斤拷确锟皆ｏ拷
										锟斤拷锟界不锟杰凤拷锟酵碉拷锟叫对伙拷锟斤拷锟剿碉拷ID 
	
	6 			n 		锟斤拷锟捷讹拷 			n 锟斤拷要小锟斤拷 113 

*/
typedef __packed struct 
{ 
	uint8_t data[106]; //锟斤拷锟捷讹拷,n锟斤拷要小锟斤拷113
} robot_interactive_data_t;



//锟斤拷锟斤拷锟剿斤拷锟斤拷锟斤拷息
typedef __packed struct
{
	xFrameHeader   							txFrameHeader;//帧头
	uint16_t								CmdID;//锟斤拷锟斤拷锟斤拷
	ext_student_interactive_header_data_t   dataFrameHeader;//锟斤拷锟捷讹拷头锟结构
	robot_interactive_data_t  	 			interactData;//锟斤拷锟捷讹拷
	uint16_t		 						FrameTail;//帧尾
}ext_CommunatianData_t;



//锟斤拷锟斤拷系统锟斤拷锟斤拷锟斤拷锟斤拷帧
typedef struct
{
	uint8_t data[JUDGE_MAX_TX_LENGTH];
	uint16_t frameLength;
}JudgeTxFrame;

typedef __packed struct{
	uint8_t game_progress; // 锟斤拷前锟斤拷锟斤拷状态 0:未锟斤拷始锟斤拷锟斤拷 1:准锟斤拷锟阶讹拷 2:锟皆硷拷锥锟� 3:锟斤拷锟诫倒锟斤拷时 4:锟斤拷锟斤拷锟斤拷 5:锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
	uint16_t remain_time;  // 锟斤拷锟斤拷剩锟斤拷时锟斤拷 锟斤拷位:s
	uint16_t current_hp;   // 锟斤拷前血锟斤拷
	uint16_t projectile;   // 锟斤拷锟街伙拷剩锟斤拷锟劫碉拷
	uint8_t sentry_info;   // bit 0: 装锟阶帮拷锟角否被癸拷锟斤拷 0:锟斤拷 1:锟斤拷
											 	 // bit 1: 锟角凤拷锟斤拷战 0:锟斤拷 1:锟斤拷
												 // bit 2: RFID 锟角凤拷锟解到锟斤拷锟斤拷 0:锟斤拷 1:锟斤拷
													 // bit 3: RFID 锟角凤拷锟解到锟斤拷锟斤拷锟斤拷(锟斤拷一锟秸撅拷锟斤拷氐锟�) 0:锟斤拷 1:锟斤拷
													 // bit 4: RFID 锟角凤拷锟解到锟斤拷锟斤拷锟斤拷(锟斤拷一锟秸撅拷氐锟�) 0:锟斤拷 1:锟斤拷
													 // bit 5: 锟斤拷前剩锟斤拷锟斤拷锟斤拷值锟角凤拷小锟斤拷30% 0:锟斤拷 1:锟斤拷
													 // bit 6-7: 0
													 // building state
	uint16_t red_outpost_hp;  // 锟届方前锟斤拷站血锟斤拷
	uint16_t red_base_hp;     // 锟届方锟斤拷锟斤拷血锟斤拷
	uint16_t blue_outpost_hp; // 锟斤拷锟斤拷前锟斤拷站血锟斤拷
	uint16_t blue_base_hp;    // 锟斤拷锟斤拷锟斤拷锟斤拷血锟斤拷
/***********锟斤拷锟斤拷为锟斤拷锟斤拷ai锟斤拷**************/
	uint16_t shooter_barrel_cooling_value;  // 锟斤拷却锟劫讹拷
	uint16_t shooter_barrel_heat_limit; 		//锟斤拷锟斤拷锟斤拷锟斤拷
	uint8_t power_management_gimbal_output : 1;
	uint8_t power_management_chassis_output : 1;
	uint8_t power_management_shooter_output : 1;
	uint16_t shooter_17mm_barrel_heat;			//剩锟斤拷锟斤拷锟斤拷
	float initial_speed;										//锟斤拷锟斤拷
	uint8_t self_color;
}JudgeData_t;

/****************锟斤拷锟斤拷锟斤拷锟斤拷***************/
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


//锟斤拷锟斤拷6锟叫断回碉拷
void USER_USART1_IRQHandler(void);
//锟斤拷锟斤拷氐锟�
void Task_Judge_Callback(void);
//锟斤拷锟竭回碉拷
void Judge_UartLostCallback(void);

void Judge_Receive(void);
/****************锟解部锟斤拷锟斤拷***************/
extern ext_game_robot_pos_t			GameRobotPos;
extern ext_shoot_data_t				ShootData;
extern ext_game_robot_status_t		GameRobotStat;
extern JudgeData_t 					USER_JudgeData;
extern ext_power_heat_data_t 		PowerHeatData;

#endif //头锟侥硷拷
