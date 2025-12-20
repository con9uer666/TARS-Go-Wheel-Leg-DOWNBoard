#ifndef _DETECT_H_
#define _DETECT_H_

#include "main.h"

// 连接在主板上的设备的标识(即设备离线信息列表中的下标)
enum
{
	// 底盘电机
	DeviceID_ChassisMotor1 = 0,
	DeviceID_ChassisMotor2,
	DeviceID_ChassisMotor3,
	DeviceID_ChassisMotor4,

  DeviceID_Turn_Motor1,
  DeviceID_Turn_Motor4,  
	// 云台电机
	DeviceID_YawMotor,
	// 拨弹电机
	DeviceID_TrigMotor,
	// 板间通信
	DeviceID_B2B,
	// 裁判系统接收
	DeviceID_Judge,
	
	DETECT_DEVICE_NUM // 放在最后，代表设备数量(设备离线信息列表长度)
};

// 掉线检测结构体，每个设备对应一个结构体变量
typedef struct _DetectDevice
{
	uint32_t maxInterval;	   // 收到数据的最大间隔时间，超过则认为断开
	uint32_t lastRecieveTime;  // 记录到的上次收到数据的时间
	uint8_t isLost;			   // 标记是否已丢失
	void (*lostFunc)(void);	   // 掉线处理函数
	void (*recoverFunc)(void); // 恢复连接处理函数
} DetectDevice;

/****接口函数声明****/
// 更新数据接收时间
void Detect_Update(uint8_t deviceID);
// 获取设备丢失情况
uint8_t Detect_IsDeviceLost(uint8_t deviceID);
// 初始化
void Detect_InitAll(void);
// 任务回调
void Task_Detect_Callback(void);

extern DetectDevice detectList[DETECT_DEVICE_NUM];

#endif
