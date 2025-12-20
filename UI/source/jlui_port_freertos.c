
#include "jlui.h"
#include "usart.h"
#include <freertos.h>
#include <semphr.h>
#include "userfreertos.h"
#include "Board2Board.h"
#include "judge.h"
#include "Graphics.h"

extern int16_t fricMotor_left_speed;
extern uint8_t chassis_rotate_mode;
extern uint8_t visionFindcheck;
extern uint8_t vision_mode; // 0为自瞄，1为小符，2为大符
extern int16_t chassis_rotate_angle;
extern int16_t visionX;
extern int16_t visionY;
extern float v_dis;
extern uint16_t shootNum ;
extern uint8_t cap_fastMode;
extern uint8_t diagonal_enable;
extern uint8_t trigger_block;
extern int	shootnum;
extern uint8_t trigger_reverse;
extern uint8_t vision_exposure;
extern uint8_t vision_rune_dirt; ////0-anti-clockwise 1-clockwise

uint8_t UI_RES = 0;
int16_t last_chassis_mode;

#define ChassisMode_Follow 0
#define ChassisMode_Spin 1

#define EN_UI_TASK

// 注1：对FreeRTOS移植时，需要在使用JLUI库前传递一个已经创建好的互斥锁给JLUI库。
// 互斥锁的创建方式如下：
// void *mutex = xSemaphoreCreateMutex();
// JLUI_SetMutexObject(mutex);
//
// 注2：在FreeRTOS中使用JLUI库时，调用10Hz Tick请使用FreeRTOS定时器；不要用中断服务函数，
// 这里给出的互斥锁移植是不能在中断中使用的！！！
// 创建定时器的方式如下：
// TimerHandle_t timer = xTimerCreate("JLUI_Tick", pdMS_TO_TICKS(100), pdTRUE, NULL, JLUI_Tick);
// xTimerStart(timer, 0);
// JLUI_Tick函数中调用JLUI_Tick10Hz即可：
// void JLUI_Tick(TimerHandle_t xTimer) { JLUI_10HzTick(); }

// 【用户需要实现此函数】
// 对互斥锁上锁。如果成功获取了锁，返回1；否则返回0。
int JLUI_MutexLock(void *mutex)
{
	// return xSemaphoreTake((SemaphoreHandle_t)mutex, 10) == pdTRUE;
	// 容易在各种电控的车上出现问题，先禁用互斥锁。后续可能也不需要使用锁
	return 1;
}

// 【用户需要实现此函数】
// 对互斥锁解锁。
void JLUI_MutexUnlock(void *mutex)
{
	//	xSemaphoreGive((SemaphoreHandle_t)mutex);
}

// 【用户需要实现此函数】
// 从串口向裁判系统发送数据函数。注意！从函数返回后，data缓冲区即应视为失效。如要做异步发送，请自行拷贝data。
uint8_t x2;
void JLUI_SendData(const uint8_t *data, size_t len)
{
	// TODO: !!!
	//	huart6.gState = HAL_UART_STATE_READY;//由于hal库上锁机制有问题所以暴力解锁
	HAL_UART_Transmit_DMA(&huart1, data, len);
}

#ifdef EN_UI_TASK
uint32_t UI_time = 300;
float anglestart = 0;
float angleend = 0;
void OS_UICallBack(void const *argument)
{
	uint32_t cnt = 0;
//	uint32_t Current_HP = 0;
//	uint32_t Last_HP = JUDGE_GetHP();
//	while (!JUDGE_GetSelfID())
		osDelay(500);
	JLUI_SetSenderReceiverId(JUDGE_GetSelfID(), JUDGE_GetClientID());

	// 测试用例
	//		JLUI_CreateString(4,UiMagenta,0,600,130,20,"slow");
	//		JLUI_CreateString(4,UiGreen,0,600,180,20,"fast");
	//		Uiid test666 = JLUI_CreateInt(5, color666, 0, 600, 400, 30, 666);
	//		Uiid Fslash = JLUI_CreateLine(5, colorFslash, 1, 0, 0, 1920, 1080);
	//		Uiid Bslash = JLUI_CreateLine(20, colorBslash, 1, 0, 1080, 1920, 0);
	//		Uiid uiTick = JLUI_CreateInt(5, UiTeam, 2, 600, 230, 30, 0);
	//		Uiid changingString = JLUI_CreateString(5, UiWhite, 2, 700, 150, 20, stateString[0]);

	// 超电能量条
	Uiid Super_Rec = JLUI_CreateRect(5, UiMagenta, 0, 800,140,1200,80);
	Uiid Super_Line = JLUI_CreateLine(40, UiYellow, 0, 800, 110, 1200, 110);
	// 受击未开小陀螺提示
	Uiid HP_Dec;
	// 视觉识别提示
	Uiid VisionFound;
	// 车头方位提示
	Uiid headstock;
	// 四种视觉模式
	Uiid VisionMode;
	// 摩擦轮转速
	Uiid FricSpeed;
	// 超电开启状态
	Uiid SuperCap;
	SuperCap = JLUI_CreateCircle(5, UiGreen, 0, 960, 538, 220);
	// 超电爆发状态
	Uiid CAPBurst;
//	CAPBurst = JLUI_CreateCircle(5, UiMagenta, 0, 960, 538, 240);

// 车宽线
//	Uiid CarBody;
	//近战
//	JLUI_CreateLine(4,UiGreen,1,935,397,965,397);
//	JLUI_CreateLine(4,UiGreen,1,949,382,949,413);
//	//7.8m
//	JLUI_CreateLine(4,UiOrange,1,942,454,973,454);
//	JLUI_CreateLine(4,UiOrange,1,954,439,954,470);
	// 车宽线
	Uiid CarBody1 = JLUI_CreateLine(4,UiCyan,1,1263,0,990,439);
	Uiid CarBody2 =JLUI_CreateLine(4,UiCyan,1,551,0,832,439);
//	过热提醒
	Uiid YawHeat;
	JLUI_CreateString(4, UiPink, 0, 100, 750, 20, "YawTemp:");
	YawHeat = JLUI_CreateInt(4, UiPink, 0, 300, 750, 20, 0);
//	JLUI_CreateString;
	
	Uiid VisionExpo;
	JLUI_CreateString(4, UiWhite, 0, 1500, 400, 20, "VisExpo:");
	VisionExpo = JLUI_CreateInt(4, UiWhite, 0, 1700, 400, 20, vision_exposure*100);

	Uiid Vis_Runedirt;
	JLUI_CreateArc(4, UiPink, 0, 1750, 510, 45, 45, 0, 270);
	Vis_Runedirt = JLUI_CreateArc(4, UiPink, 0, 1807, 519, 65, 65, 300, 330); //anti
//	Vis_Runedirt = JLUI_CreateArc(4, UiPink, 0, 1807, 493, 65, 65, 300, 330); //anti

// 视觉识别框
//	Uiid VisionRect;
//	Uiid VisionPoint;
	Uiid shoot_cnt;
	Uiid Diagonal;
	Diagonal = JLUI_CreateInt(4, UiGreen, 0, 300, 700, 20, diagonal_enable);
	JLUI_CreateString(4, UiGreen, 0, 100, 700, 20, "Diagonal:");

	// 小陀螺提示
	HP_Dec = JLUI_CreateString(4, UiMagenta, 0, 750, 700, 40, "Please Spin");
	// 视觉识别提示
	VisionFound = JLUI_CreateCircle(5, UiTeam, 0, 960, 538, 200);
	// 车头方位提示
	headstock = JLUI_CreateArc(5, UiMagenta, 0, 960, 538, 280, 280, 0, 0);
	// 三种视觉模式
	JLUI_CreateString(4, UiBlack, 0, 100, 600, 20, "VisionMode:");
	VisionMode = JLUI_CreateInt(5, UiBlack, 0, 310, 600, 20, vision_mode);
	// 摩擦轮转速
	JLUI_CreateString(4, UiOrange, 0, 100, 550, 20, "FricSpeed:");
	FricSpeed = JLUI_CreateInt(5, UiOrange, 0, 300, 550, 20, fricMotor_left_speed);
	//射击计数
	JLUI_CreateString(4, UiYellow, 0, 100, 650, 20, "ShootNum:");
	shoot_cnt = JLUI_CreateInt(5, UiYellow, 0, 300, 650, 20, shootNum);
	
	
	for (;;)
	{
		osDelay(UI_time);
		if (UI_time > 35)
			UI_time -= 2;
		JLUI_10HzTick();

		if (RemoteControl.keyboard_value.bit.C == 1)
		{
			UI_RES = 1;
		}
		if (UI_RES == 1)
		{
//			JLUI_DeleteAll();
//			UI_RES = 0;
//			UI_time = 100;
//			break;
		}

		cnt++;
		//车头指示
		anglestart = chassis_rotate_angle - 15;
		angleend = chassis_rotate_angle + 15;
		if (anglestart < 0)
			anglestart += 360;
		if (angleend > 360)
			angleend -= 360;
		if (angleend < 0)
			angleend += 360;
		JLUI_SetStartAngle(headstock, anglestart);
		JLUI_SetEndAngle(headstock, angleend);

		//底盘状态(HPDEC变成底盘了。。)
		if(chassis_rotate_mode == ChassisMode_Spin)
				JLUI_SetVisible(HP_Dec, 0);
		if(chassis_rotate_mode == ChassisMode_Follow)
				JLUI_SetVisible(HP_Dec, 1); 
		
//		if (cnt % 10 == 0)
//		{
//			Current_HP = JUDGE_GetHP();
//			Last_HP = Current_HP;
//			if (Last_HP - Current_HP >= 10)
//			{
//				if (chassis_rotate_mode == ChassisMode_Follow)
//					JLUI_SetVisible(HP_Dec, 1);
//			}
//			else if (Last_HP <= Current_HP)
//			{
//				JLUI_SetVisible(HP_Dec, 0);
//			}
//		}	未测试
	
		
		// 刷新视觉识别状态
		if (visionFindcheck == 1)
		{
			JLUI_SetVisible(VisionFound, 1);
			//	JLUI_MoveTo(VisionRect,visionX,visionY);
			//	JLUI_MoveTo(VisionPoint,visionX,visionY);
			//	JLUI_SetVisible(VisionRect, 1);
			//	JLUI_SetVisible(VisionPoint, 1);
		}
		else
		{
			JLUI_SetVisible(VisionFound, 0);
			//	JLUI_SetVisible(VisionRect, 0);
			//	JLUI_SetVisible(VisionPoint, 0);
		}
		//更新视觉模式
		JLUI_SetInt(VisionMode, vision_mode);
		
		//视觉顺逆时针
		if(vision_rune_dirt == 1)
		{
			JLUI_MoveTo(Vis_Runedirt,1736, 460);
		}
		else
		{
			JLUI_MoveTo(Vis_Runedirt,1807, 519);
		}
		//视觉曝光
		JLUI_SetInt(VisionExpo,vision_exposure*100);
		
		// 更新摩擦轮转速
		JLUI_SetInt(FricSpeed, fricMotor_left_speed);
		//shootnumber
		
		JLUI_SetInt(shoot_cnt, shootnum);
		if(trigger_block == 1)
		{
			JLUI_SetColor(shoot_cnt,UiPink);
		}
		else if(trigger_reverse == 1)
		{
			JLUI_SetColor(shoot_cnt,UiOrange);
		}
		else
			JLUI_SetColor(shoot_cnt,UiYellow);

		
		//	YAW过热
		JLUI_SetInt(YawHeat,0);

		//  对角舵显示
		JLUI_SetInt( Diagonal,diagonal_enable);
		if(diagonal_enable == 1)
		{
			JLUI_SetVisible(CarBody1,0);
			JLUI_SetVisible(CarBody2,0);
		}
		else
		{
			JLUI_SetVisible(CarBody1,1);
			JLUI_SetVisible(CarBody2,1);
		}

		// 刷新超电能量条
		JLUI_MoveP2To(Super_Line, 800+0*4, 110);
		if(0<20)
		{
			JLUI_SetColor(Super_Line,UiWhite);
		}
		else
		{
			JLUI_SetColor(Super_Line,UiYellow);
		}
		// 更新超电开启状态
		if(cap_fastMode == 1){
			JLUI_SetVisible(SuperCap, 1);
		}
		if(cap_fastMode != 1){
			JLUI_SetVisible(SuperCap, 0);
		}
	}
}
#endif
