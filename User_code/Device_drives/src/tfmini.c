#include "tfmini.h"

/* usart10 DMA 接收缓冲，定义在 Board2Board.c */
extern uint8_t usart10RxBuf[128];

/* 解析后的最新有效数据（供外部读取） */
TFmini_Data_t tfmini_data;

/* 帧拼装缓冲（9字节 = 0x59 0x59 + 6数据 + 1校验） */
static uint8_t  tfmini_buf[9];

/* 拼装位置 (0=未同步, 1~8=正在拼, 9=帧完整) */
static uint8_t  tfmini_idx;

// 从 usart10RxBuf 字节流中解析一帧 TFmini Plus 数据，填入 tfmini_data
void TFmini_Parse(void)
{
    // 遍历本次 DMA 接收的所有字节
    for (uint16_t i = 0; i < sizeof(usart10RxBuf); i++)
    {
        uint8_t b = usart10RxBuf[i];            // 当前字节

        if (tfmini_idx == 0)                    // 未同步，找帧头 0x59
        {
            if (b == 0x59) { tfmini_buf[0] = 0x59; tfmini_idx = 1; } // 收到第一个 0x59
        }
        else if (tfmini_idx == 1)               // 验证第二个帧头
        {
            if (b == 0x59) { tfmini_buf[1] = 0x59; tfmini_idx = 2; } // 帧头完整
            else           { tfmini_idx = 0; }                       // 不是 0x59，重新同步
        }
        else                                    // 正在拼数据字节（idx=2~8）
        {
            tfmini_buf[tfmini_idx] = b;         // 存入拼装缓冲
            tfmini_idx++;                       // 下标后移

            if (tfmini_idx == 9)                // 9 字节收齐，开始解析
            {
                tfmini_idx = 0;                 // 复位，准备下一帧

                // ——— 校验和：前 8 字节累加取低 8 位 ———
                uint8_t sum = 0;
                for (int k = 0; k < 8; k++) sum += tfmini_buf[k];
                if (sum != tfmini_buf[8]) continue; // 校验失败，丢弃

                // ——— 小端解析 ———
                uint16_t dist     = tfmini_buf[2] | (uint16_t)tfmini_buf[3] << 8; // 距离，单位 cm
                uint16_t strength = tfmini_buf[4] | (uint16_t)tfmini_buf[5] << 8; // 信号强度
                int16_t  temp_raw = tfmini_buf[6] | (uint16_t)tfmini_buf[7] << 8; // 温度原始值

                // ——— 有效性判断：强度 < 100 或 == 65535 视为无效 ———
                if (strength < 100 || strength == 65535)
                {
                    tfmini_data.distance = 0;
                    tfmini_data.valid    = 0;
                }
                else
                {
                    tfmini_data.distance = dist;
                    tfmini_data.valid    = 1;
                }
                tfmini_data.strength    = strength;
                tfmini_data.temperature = (float)temp_raw / 8.0f - 256.0f; // ℃ = raw/8 - 256
            }
        }
    }
}
