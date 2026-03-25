#ifndef USER_CAN_H
#define USER_CAN_H

#include "fdcan.h"
#include "main.h"

void CAN_Init(void);
void CAN_Send_DM_Motor_Data(FDCAN_HandleTypeDef *hfdcan, int16_t StdId, uint8_t *Data);

#endif
