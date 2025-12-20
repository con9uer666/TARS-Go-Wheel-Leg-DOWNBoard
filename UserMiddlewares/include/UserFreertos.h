#ifndef _USER_FREERTOS_H_
#define _USER_FREERTOS_H_
#include "cmsis_os.h"
#include "spi.h"
#define WS2812_SPI_UNIT     hspi6
extern SPI_HandleTypeDef WS2812_SPI_UNIT;

//cube生成的供外部文件调用的变量
//task
extern  osThreadId IMUTaskHandle;
extern  osThreadId LEDTaskHandle;
extern  osThreadId MotorTaskHandle;
extern  osThreadId ChassisTaskHandle;
extern  osThreadId RcTaskHandle;
extern  osThreadId ErrorTaskHandle;
extern  osThreadId GimbalTaskHandle;
extern  osThreadId ShooterTaskHandle;
extern  osThreadId VisionTxTaskHandle;
extern  osThreadId BeepTaskHandle;
extern  osThreadId JudgeTaskHandle;
extern  osThreadId DetectTaskHandle;
#endif

