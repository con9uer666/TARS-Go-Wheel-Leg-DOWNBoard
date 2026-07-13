#ifndef _TFMINI_H_
#define _TFMINI_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* TFmini Plus 激光雷达数据结构体 */
typedef struct {
    uint16_t distance;    // 距离 cm (0~1200)
    uint16_t strength;    // 信号强度 (0~65535)
    float temperature;    // 温度 ℃ (公式: raw/8 - 256)
    uint8_t valid;        // 数据有效标志: 1=有效, 0=无效 (强度<100或==65535时无效)
} TFmini_Data_t;

/* 解析后的最新有效数据，由 TFmini_Parse() 维护，外部只读 */
extern TFmini_Data_t tfmini_data;

/*
 * @brief 从 usart10RxBuf 字节流中解析一帧 TFmini Plus 数据，结果写入 tfmini_data。
 *        在 huart10 的 DMA 接收回调里调用。
 */
void TFmini_Parse(void);

#ifdef __cplusplus
}
#endif

#endif /* _TFMINI_H_ */
