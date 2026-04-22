#ifndef _JUDGEMENT_H_
#define _JUDGEMENT_H_

#include "main.h"
#include "stdbool.h"

#define    JUDGE_DATA_ERROR      0
#define    JUDGE_DATA_CORRECT    1

#define    LEN_HEADER    5        //֡ͷ��
#define    LEN_CMDID     2        //�����볤��
#define    LEN_TAIL      2	      //֡βCRC16

//��ʼ�ֽ�,Э��̶�Ϊ0xA5
#define    JUDGE_FRAME_HEADER         (0xA5)

//���ͻ���������
#define JUDGE_MAX_TX_LENGTH     64
//���ջ���������
#define JUDGE_MAX_RX_LENGTH     1536
//���Ͷ��г���
#define JUDGE_QUEUE_SIZE 25

//��������ɫ
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

//5�ֽ�֡ͷ,ƫ��λ��
typedef enum
{
	SOF          = 0,//��ʼλ
	DATA_LENGTH  = 1,//֡�����ݳ���,�����������ȡ���ݳ���
	SEQ          = 3,//�����
	CRC8         = 4 //CRC8
	
}FrameHeaderOffset;

/***************������ID********************/

/* 

	ID: 0x0001  Byte:  11    ����״̬����       			����Ƶ�� 1Hz      
	ID: 0x0002  Byte:  1    �����������         		������������      
	ID: 0x0003  Byte:  16    ����������Ѫ������   		1Hz����    //2026.3.14�޸� 16byte
	ID: 0x0101  Byte:  4    �����¼�����   				�¼��ı����
	ID: 0x0104	Byte: 	3		���о�����Ϣ				//2026.3.14�޸� 3byte
	ID: 0x0105	Byte: 	3		���ڷ���ڵ���ʱ		//2026.3.14�޸� 3byte
	ID: 0X0201  Byte: 13    ������״̬����        		10Hz		//2026.3.14�޸� 13byte
	ID: 0X0202  Byte: 14    ʵʱ������������   			50Hz       
	ID: 0x0203  Byte: 12    ������λ������           	10Hz
	ID: 0x0204  Byte:  8    ��������������           	����״̬�ı����
	ID: 0x0206  Byte:  1    �˺�״̬����           		�˺���������
	ID: 0x0207  Byte:  7    ʵʱ�������           		�ӵ��������
	ID: 0x0208  Byte:  8    �ӵ�ʣ�෢����					//2026.3.14�޸� 6byte
	ID: 0x0209  Byte:  5    ������RFID״̬					//2026.3.14�޸� 5byte
	ID: 0x020A  Byte:  6    ���ڻ����˿ͻ���ָ������
	ID: 0x020D Byte:  6		�ڱ������˵�ǰ״̬
	ID: 0x0301  Byte:  118    �����˼佻������           	���ͷ���������,10Hz  //2026.3.14�޸� 16byte
*/


//������ID,�����жϽ��յ���ʲô����
typedef enum
{
	ID_game_state       				= 0x0001,//����״̬����
	ID_game_result 	   					= 0x0002,//�����������
	ID_game_robot_HP      			= 0x0003,//����������Ѫ������
	ID_event_data  							= 0x0101,//�����¼����� *
	ID_referee_warning					= 0x0104,//���о�����Ϣ
	ID_dart_remaining_time			= 0x0105,//���ڷ���ڵ���ʱ
	ID_game_robot_state    			= 0x0201,//������״̬����  *
	ID_power_heat_data    			= 0x0202,//ʵʱ������������
	ID_game_robot_pos        		= 0x0203,//������λ������  *
	ID_buff_musk								= 0x0204,//��������������//
	ID_robot_hurt								= 0x0206,//�˺�״̬����
	ID_shoot_data								= 0x0207,//ʵʱ�������
	ID_bullet_remaining					= 0x0208,//�ӵ�ʣ�෢����
	ID_rfid_status							= 0x0209,//������RFID״̬  *	
	ID_sentry_status          	= 0x020D,
} CmdID;



//���������ݶγ�,���ݹٷ�Э�������峤��
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


////�����˽�����Ϣ
//typedef __packed struct
//{
//	xFrameHeader   							txFrameHeader;//֡ͷ
//	uint16_t								CmdID;//������
//	ext_student_interactive_header_data_t   dataFrameHeader;//���ݶ�ͷ�ṹ
//	robot_interactive_data_t  	 			interactData;//���ݶ�
//	uint16_t		 						FrameTail;//֡β
//}ext_CommunatianData_t;


/* �Զ���֡ͷ */
typedef __packed struct //����ϵͳ֡ͷ
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



/* ID: 0x0001  Byte:  11    ����״̬���� */
typedef __packed struct
{
	uint8_t game_type : 4;
	uint8_t game_progress : 4;
	uint16_t stage_remain_time;
	uint64_t SyncTimeStamp;
} ext_game_status_t;

/* ID: 0x0002  Byte:  1    ����������� */
typedef __packed struct 
{ 
	uint8_t winner;
} ext_game_result_t; 

/* ID: 0x0003  Byte:  32    ����������Ѫ������ */   //2026.3.14 �Ķ�
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
 
/* ID: 0x0101  Byte:  4    �����¼����� */
typedef __packed struct 					//2026.3.14�޸�
{ 	
	uint32_t event_data;
	
	//�����Ƕ����Զ���ṹ�� 
	//0��δռ��/δ����  1����ռ��/�Ѽ��� 
	/*bit 0-2�� 
		bit 0��������һ������ص��Ĳ�����ռ��״̬��1Ϊ��ռ�� 
		bit 1��������һ����ص��Ĳ�����ռ��״̬��1Ϊ��ռ�� 
		bit 2��������������ռ��״̬��1Ϊ��ռ�죨�� RMUL ���ã� 
		bit 3-6��������������״̬ 
		bit 3-4������С�������صļ���״̬��0Ϊδ���1Ϊ�Ѽ��2Ϊ���ڼ���
		bit 5-6���������������صļ���״̬��0Ϊδ���1Ϊ�Ѽ��2Ϊ���ڼ���
		bit 7-8����������ߵص�ռ��״̬��1Ϊ������ռ�죬2Ϊ���Է�ռ�� 
		bit 9-10���������θߵص�ռ��״̬��1Ϊ��ռ�� 
		bit 11-19���Է��������һ�λ��м���ǰ��վ����ص�ʱ�䣨0-420������Ĭ��Ϊ0�� 
		bit 20-22���Է��������һ�λ��м���ǰ��վ����صľ���Ŀ�꣬����Ĭ��Ϊ0��
							 1Ϊ����ǰ��վ��2Ϊ���л��ع̶�Ŀ�꣬3Ϊ���л������
							 �̶�Ŀ�꣬4Ϊ���л�������ƶ�Ŀ�� 5Ϊ���л���ĩ���ƶ�Ŀ��
		bit 23-24������������ռ��״̬��0Ϊδ��ռ�죬1Ϊ������ռ�죬2
							 Ϊ���Է�ռ�죬3Ϊ��˫��ռ�졣����RMUL���ã� 
		bit 25-26����������������ռ��״̬��0Ϊδ��ռ�죬1Ϊ������ռ
							 �죬2Ϊ���Է�ռ�죬3Ϊ��˫��ռ�졣
		bit 27-28������ǰ��վ������ռ��״̬�� 0 Ϊδ��ռ�죬 1 Ϊ������
							 ռ�죬 2 Ϊ���Է�ռ��
		bit 29����������������ռ��״̬�� 1 Ϊ��ռ��							 */
	
//	uint8_t supply_area_state : 3;
//	uint8_t own_buff_state : 4;
//	uint8_t own_central_land_state : 2;
//	uint8_t own_trapezoidal_land_state : 2;
//	uint16_t darts_hitted_time : 9;
//	uint8_t darts_hitted_target : 3;
//	uint8_t center_gain_state : 2;
//	uint8_t own_fortress_state :2;
//	uint8_t own_outpost_state : 2;   // bit27-28 ����ǰ��վ�����״̬
//	uint8_t own_base_state : 1;      // bit29 �������������״̬
//	
//	uint32_t reserved : 2;
	
} ext_event_data_t; 



/* ID: 0x104    Byte: 3    ���о�����Ϣ */
typedef __packed struct
{
  uint8_t level; 
  uint8_t offending_robot_id; 
  uint8_t count; 
} ext_referee_warning_t;

/* ID: 0x105    Byte: 3    ���ڷ���ڵ���ʱ */  
typedef __packed struct 
{ 
  uint8_t dart_remaining_time; 
  uint16_t dart_info; 
}ext_dart_info_t; 

/* ID: 0X0201  Byte: 13    ������״̬���� */
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


/* ID: 0X0202  Byte: 16    ʵʱ������������ */
typedef __packed struct
{
	uint16_t reserved_1;
	uint16_t reserved_2;
	float reserved_3;
	uint16_t chassis_power_buffer;
	uint16_t shooter_id1_17mm_cooling_heat;
	uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;


/* ID: 0x0203  Byte: 12    ������λ������ */
typedef __packed struct 
{ 
  float x; 
  float y; 
  float angle; 
} ext_game_robot_pos_t; 


/* ID: 0x0204  Byte:  8    �������������� */
typedef __packed struct 
{ 
  uint8_t recovery_buff;  
  uint16_t cooling_buff;  
  uint8_t defence_buff;  
  uint8_t vulnerability_buff; 
  uint16_t attack_buff; 
	uint8_t remaining_energy; 
} ext_buff_musk_t; 


/* ID: 0x0206  Byte:  2    �˺�״̬���� */
typedef __packed struct 
{ 
	uint8_t armor_id : 4; 
	uint8_t hurt_type : 4; 
} ext_robot_hurt_t; 


/* ID: 0x0207  Byte:  7    ʵʱ������� */
typedef __packed struct
{
	uint8_t bullet_type;  
  uint8_t shooter_number; 
  uint8_t launching_frequency;  
  float initial_speed; 
} ext_shoot_data_t;

/* ID: 0x0208  Byte:  8   �ӵ�ʣ�෢���� */
typedef __packed struct
{
	uint16_t projectile_allowance_17mm; 
  uint16_t projectile_allowance_42mm;  
  uint16_t remaining_gold_coin; 
  uint16_t projectile_allowance_fortress;
} ext_bullet_remaining_t;

/* ID: 0x0209  Byte:  5    ������RFID״̬ */ //2026.3.14�޸�
typedef __packed struct
{
	uint32_t rfid_status;
	uint8_t rfid_status_2;
} ext_rfid_status_t;

/* ID: 0x020A  Byte:  6    ���ڻ����˿ͻ���ָ������ */
typedef __packed struct
{
	uint8_t dart_launch_opening_status;  
  uint8_t reserved;  
  uint16_t target_change_time;  
  uint16_t latest_launch_cmd_time; 
} ext_dart_client_cmd_t;	

/* ID: 0x020D  Byte:  6    �ڱ������˵�ǰ״̬���� */
typedef __packed struct
{
	uint32_t sentry_info;
	uint16_t sentry_info_2;
}sentry_info_t;

/* 
	
	�������ݣ�����һ��ͳһ�����ݶ�ͷ�ṹ��
	���������� ID���������Լ������ߵ� ID ���������ݶΣ�
	�����������ݵİ��ܹ������Ϊ 128 ���ֽڣ�
	��ȥ frame_header,cmd_id,frame_tail �Լ����ݶ�ͷ�ṹ�� 6 ���ֽڣ�
	�ʶ����͵��������ݶ����Ϊ 113��
	������������ 0x0301 �İ�����Ƶ��Ϊ 10Hz��

	������ ID��
	1��Ӣ��(��)��
	2������(��)��
	3/4/5������(��)��
	6������(��)��
	7���ڱ�(��)��
	11��Ӣ��(��)��
	12������(��)��
	13/14/15������(��)��
	16������(��)��
	17���ڱ�(��)�� 
	�ͻ��� ID�� 
	0x0101 ΪӢ�۲����ֿͻ���( ��) ��
	0x0102 �����̲����ֿͻ��� ((�� )��
	0x0103/0x0104/0x0105�����������ֿͻ���(��)��
	0x0106�����в����ֿͻ���((��)�� 
	0x0111��Ӣ�۲����ֿͻ���(��)��
	0x0112�����̲����ֿͻ���(��)��
	0x0113/0x0114/0x0115�������ֿͻ��˲���(��)��
	0x0116�����в����ֿͻ���(��)�� 
*/
/* �������ݽ�����Ϣ��0x0301  */
typedef __packed struct 	//2026.3.14�޸�
{ 
	uint16_t data_cmd_id;    
	uint16_t send_ID;    
	uint16_t receiver_ID; 
} ext_student_interactive_header_data_t; 
/* 
	ѧ�������˼�ͨ�� cmd_id 0x0301������ ID:0x0200~0x02FF
	�������� �����˼�ͨ�ţ�0x0301��
	����Ƶ�ʣ����� 10Hz  

	�ֽ�ƫ���� 	��С 	˵�� 			��ע 
	0 			2 		���ݵ����� ID 	0x0200~0x02FF 
										���������� ID ��ѡȡ������ ID �����ɲ������Զ��� 
	
	2 			2 		�����ߵ� ID 	��ҪУ�鷢���ߵ� ID ��ȷ�ԣ� 
	
	4 			2 		�����ߵ� ID 	��ҪУ������ߵ� ID ��ȷ�ԣ�
										���粻�ܷ��͵��жԻ����˵�ID 
	
	6 			n 		���ݶ� 			n ��ҪС�� 113 

*/
typedef __packed struct 
{ 
	uint8_t data[106]; //���ݶ�,n��ҪС��113
} robot_interactive_data_t;



//�����˽�����Ϣ
typedef __packed struct
{
	xFrameHeader   							txFrameHeader;//֡ͷ
	uint16_t								CmdID;//������
	ext_student_interactive_header_data_t   dataFrameHeader;//���ݶ�ͷ�ṹ
	robot_interactive_data_t  	 			interactData;//���ݶ�
	uint16_t		 						FrameTail;//֡β
}ext_CommunatianData_t;



//����ϵͳ��������֡
typedef struct
{
	uint8_t data[JUDGE_MAX_TX_LENGTH];
	uint16_t frameLength;
}JudgeTxFrame;

typedef __packed struct{
	uint8_t game_progress; // ��ǰ����״̬ 0:δ��ʼ���� 1:׼���׶� 2:�Լ�׶� 3:���뵹��ʱ 4:������ 5:����������
	uint16_t remain_time;  // ����ʣ��ʱ�� ��λ:s
	uint16_t current_hp;   // ��ǰѪ��
	uint16_t projectile;   // ���ֻ�ʣ���ٵ�
	uint8_t sentry_info;   // bit 0: װ�װ��Ƿ񱻹��� 0:�� 1:��
											 	 // bit 1: �Ƿ���ս 0:�� 1:��
												 // bit 2: RFID �Ƿ��⵽���� 0:�� 1:��
													 // bit 3: RFID �Ƿ��⵽������(��һ�վ���ص�) 0:�� 1:��
													 // bit 4: RFID �Ƿ��⵽������(��һ�վ�ص�) 0:�� 1:��
													 // bit 5: ��ǰʣ������ֵ�Ƿ�С��30% 0:�� 1:��
													 // bit 6-7: 0
													 // building state
	uint16_t red_outpost_hp;  // �췽ǰ��վѪ��
	uint16_t red_base_hp;     // �췽����Ѫ��
	uint16_t blue_outpost_hp; // ����ǰ��վѪ��
	uint16_t blue_base_hp;    // ��������Ѫ��
/***********����Ϊ����ai��**************/
	uint16_t shooter_barrel_cooling_value;  // ��ȴ�ٶ�
	uint16_t shooter_barrel_heat_limit; 		//��������
	uint8_t power_management_gimbal_output : 1;
	uint8_t power_management_chassis_output : 1;
	uint8_t power_management_shooter_output : 1;
	uint16_t shooter_17mm_barrel_heat;			//ʣ������
	float initial_speed;										//����
	uint8_t self_color;
}JudgeData_t;

/****************��������***************/
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


//����6�жϻص�
void USER_USART1_IRQHandler(void);
//����ص�
void Task_Judge_Callback(void);
//���߻ص�
void Judge_UartLostCallback(void);

void Judge_Receive(void);
/****************�ⲿ����***************/
extern ext_game_robot_pos_t			GameRobotPos;
extern ext_shoot_data_t				ShootData;
extern ext_game_robot_status_t		GameRobotStat;
extern JudgeData_t 					USER_JudgeData;
extern ext_power_heat_data_t 		PowerHeatData;

#endif //ͷ�ļ�
