#ifndef USB_H
#define USB_H

#include "main.h"

typedef struct __attribute__((packed))
{
    uint8_t     header[2];      // 帧头，固定为 0xA5 0x5A
    float       F0;             // F0力
    float       T;              // T力矩
    uint16_t    checksum;       // CRC 校验码
} F0_T_TxFrame;

void Transmit_F0_T(void);

#endif // USB_H
