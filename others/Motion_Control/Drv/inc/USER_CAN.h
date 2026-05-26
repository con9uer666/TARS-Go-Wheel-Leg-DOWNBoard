#ifndef USER_CAN_H
#define USER_CAN_H

#include "fdcan.h"
#include "main.h"

extern uint8_t motor_output_enable;
extern uint8_t wheel_leg_output_enable;
extern uint8_t motor_should_enabled;
extern uint8_t motor_last_error_code;

void CAN_Init(void);
void CAN_Send_DM_Motor_Data(FDCAN_HandleTypeDef *hfdcan, int16_t StdId, uint8_t *Data);
void Error_Buzzer_Tick(void);

#endif
