#include "dr16.h"

/* 移植自 F4_version/User_Code/drv/src/RC_drv.c 的 DR16 分支。 */
uint8_t DR16_Parse(const uint8_t *buf, uint16_t size, DR16_Data_t *data)
{
    if (size != DR16_FRAME_SIZE)
        return 0;

    DR16_Data_t next = {0};
    next.ch[0] = (int16_t)((buf[0] | (buf[1] << 8)) & 0x07FF) - 1024;
    next.ch[1] = (int16_t)(((buf[1] >> 3) | (buf[2] << 5)) & 0x07FF) - 1024;
    next.ch[2] = (int16_t)(((buf[2] >> 6) | (buf[3] << 2) | (buf[4] << 10)) & 0x07FF) - 1024;
    next.ch[3] = (int16_t)(((buf[4] >> 1) | (buf[5] << 7)) & 0x07FF) - 1024;
    next.left = (buf[5] >> 6) & 3U;
    next.right = (buf[5] >> 4) & 3U;
    next.wheel = (int16_t)(buf[16] | (buf[17] << 8)) - 1024;
    if (next.left == 0 || next.right == 0 || buf[12] > 1 || buf[13] > 1)
        return 0;
    for (uint8_t i = 0; i < 4; ++i)
    {
        if (next.ch[i] < -660 || next.ch[i] > 660)
            return 0;
        if (next.ch[i] > -50 && next.ch[i] < 50)
            next.ch[i] = 0;
    }
    next.left_last = data->left;
    next.right_last = data->right;
    next.mouse.x = (int16_t)(buf[6] | (buf[7] << 8));
    next.mouse.y = (int16_t)(buf[8] | (buf[9] << 8));
    next.mouse.z = (int16_t)(buf[10] | (buf[11] << 8));
    next.mouse.l = buf[12];
    next.mouse.r = buf[13];
    next.keyboard = (uint16_t)(buf[14] | (buf[15] << 8));
    *data = next;
    return 1;
}
