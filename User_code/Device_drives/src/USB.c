#include "USB.h"
#include "CRC16.h"
#include "VMC.h"
#include "usbd_cdc_if.h"
#include <stddef.h>

_Static_assert(sizeof(F0_T_TxFrame) == 12U, "F0_T_TxFrame layout changed");

void Transmit_F0_T(void)
{
    F0_T_TxFrame tx_frame_ = {
        .header = {0xA5, 0x5A},
        .F0 = VMC_Chassis_Target.L_F0,
        .T = VMC_Chassis_Target.L_T,
        .checksum = 0
    };

    /* CRC 覆盖范围：帧头到 T，不含 checksum 字段 */
    uint16_t data_len = (uint16_t)offsetof(F0_T_TxFrame, checksum);
    tx_frame_.checksum = CalcCrc16((uint8_t*)&tx_frame_, data_len);

    CDC_Transmit_HS((uint8_t*)&tx_frame_, (uint16_t)sizeof(F0_T_TxFrame));
}

