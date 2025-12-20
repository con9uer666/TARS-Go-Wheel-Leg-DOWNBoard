#include "Detect.h"
#include "USER_CAN.h"
#include "UserFreertos.h"
#include "judge.h"
#include "Board2Board.h"

/****内部函数声明****/
// 默认掉线处理函数
void Detect_DefaultLostHandler(uint8_t deviceID);
// 默认上线处理函数
void Detect_DefaultRecoverHandler(uint8_t deviceID);
// 初始化单个设备
void Detect_InitDevice(uint8_t deviceID, uint32_t maxInterval, void (*lostFunc)(void), void (*recoverFunc)(void));

// 设备离线信息列表
DetectDevice detectList[DETECT_DEVICE_NUM];

// 更新指定id设备的状态
void Detect_Update(uint8_t deviceID)
{
	detectList[deviceID].lastRecieveTime = HAL_GetTick();
}
// 默认掉线处理函数，若lostFunc==NULL则调用
void Detect_DefaultLostHandler(uint8_t deviceID)
{

}

// 默认掉线处理函数，若recoverFunc==NULL则调用
void Detect_DefaultRecoverHandler(uint8_t deviceID)
{
}

// 初始化一个设备的掉线检测信息(设备id，数据接收最大间隔，掉线处理回调函数)
void Detect_InitDevice(uint8_t deviceID, uint32_t maxInterval, void (*lostFunc)(void), void (*recoverFunc)(void))
{
	detectList[deviceID].maxInterval = maxInterval;
	detectList[deviceID].isLost = 0;
	detectList[deviceID].lostFunc = lostFunc;
	detectList[deviceID].recoverFunc = recoverFunc;
}

// 初始化所有设备的掉线检测信息
void Detect_InitAll()
{
	Detect_InitDevice(DeviceID_ChassisMotor1, 50, NULL, NULL);
	Detect_InitDevice(DeviceID_ChassisMotor2, 50, NULL, NULL);
	Detect_InitDevice(DeviceID_ChassisMotor3, 50, NULL, NULL);
	Detect_InitDevice(DeviceID_ChassisMotor4, 50, NULL, NULL);

	Detect_InitDevice(DeviceID_Turn_Motor1, 50, NULL, NULL);
//	Detect_InitDevice(DeviceID_Turn_Motor2, 50, NULL, NULL);
//	Detect_InitDevice(DeviceID_Turn_Motor3, 50, NULL, NULL);
	Detect_InitDevice(DeviceID_Turn_Motor4, 50, NULL, NULL);

	Detect_InitDevice(DeviceID_YawMotor, 50, NULL, NULL);

	detectList[DeviceID_YawMotor].isLost = 1;//默认掉线
	
	Detect_InitDevice(DeviceID_TrigMotor, 50, NULL, NULL);

	Detect_InitDevice(DeviceID_B2B, 50, B2B_LostCallback, NULL);

	Detect_InitDevice(DeviceID_Judge, 500, Judge_UartLostCallback, NULL);
}

extern uint8_t FLAG;

// 掉线检测任务回调
void Task_Detect_Callback()
{
	uint32_t presentTime = HAL_GetTick();

	{
		for (uint8_t id = 0; id < DETECT_DEVICE_NUM; id++)
		{
			// 判定是否掉线
			if (presentTime - detectList[id].lastRecieveTime > detectList[id].maxInterval)
			{
				// 判定执行默认还是自定义的处理函数
				if (detectList[id].lostFunc == NULL)
					Detect_DefaultLostHandler(id);
				else
					detectList[id].lostFunc();
				// 更新标识
				detectList[id].isLost = 1;
			}
			else if (presentTime - detectList[id].lastRecieveTime <= detectList[id].maxInterval && detectList[id].isLost == 1)
			{
				// 判定执行默认还是自定义的处理函数
				if (detectList[id].recoverFunc == NULL)
					Detect_DefaultRecoverHandler(id);
				else
					detectList[id].recoverFunc();
				// 更新标识
				detectList[id].isLost = 0;
			}
		}
	}
}

// 获取设备离线情况
// 返回值 1-离线 0-在线
uint8_t Detect_IsDeviceLost(uint8_t deviceID)
{
	return detectList[deviceID].isLost;
}

void OS_DetectCallback(void const *argument)
{
	osDelay(500);
	for (;;)
	{
		Task_Detect_Callback();
		osDelay(10);
	}
}
