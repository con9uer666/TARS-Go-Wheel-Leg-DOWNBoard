#ifndef CRC16_H
#define CRC16_H

#include <stdint.h>

/**
 * @brief Calculate reflected CRC16 with init 0xFFFF and polynomial 0x8408.
 */
uint16_t CalcCrc16(const uint8_t *data, uint16_t length);

#endif // CRC16_H
