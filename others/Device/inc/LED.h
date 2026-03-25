#ifndef LED_H
#define LED_H

#include "main.h"
#include "tim.h"
 
#define WS_H           66   // 1 码相对计数值
#define WS_L           33   // 0 码相对计数值
#define WS_REST        45   // 复位信号脉冲数量
#define LED_1_NUM      1  // 直灯板WS2812灯个数
#define DATA_LEN       24   // WS2812数据长度，单个需要24个字节
#define WS2812_RST_NUM 55   // 官方复位时间为50us（40个周期），保险起见使用50个周期


 
void WS2812_1_Init(void);
void WS2812_1_Set(uint16_t num,uint8_t R,uint8_t G,uint8_t B);

#endif


