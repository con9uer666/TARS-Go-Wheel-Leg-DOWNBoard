#include "CRC16.h"

uint16_t CalcCrc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;

    while (length > 0U)
    {
        crc ^= *data++;

        for (uint8_t bit = 0; bit < 8U; bit++)
        {
            if ((crc & 0x0001U) != 0U)
            {
                crc = (uint16_t)((crc >> 1U) ^ 0x8408U);
            }
            else
            {
                crc >>= 1U;
            }
        }

        length--;
    }

    return crc;
}
