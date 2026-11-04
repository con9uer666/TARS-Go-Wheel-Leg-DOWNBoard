#ifndef DR16_H
#define DR16_H

#include <stdint.h>

#define DR16_FRAME_SIZE 18U
#define DR16_SWITCH_UP 1U
#define DR16_SWITCH_DOWN 2U
#define DR16_SWITCH_MIDDLE 3U

typedef struct
{
    int16_t ch[4]; /* 右横、右竖、左横、左竖；中心为 0，范围约 +/-660 */
    /* ch[1]: right vertical (down < 0); ch[2]: left horizontal (left < 0). */
    /* Switch encoding: up=1, middle=3, down=2; right=2 is emergency stop. */
    uint8_t left;
    uint8_t right;
    uint8_t left_last;
    uint8_t right_last;
    struct
    {
        int16_t x, y, z;
        uint8_t l, r;
    } mouse;
    uint16_t keyboard;
    int16_t wheel;
} DR16_Data_t;

/* 成功返回 1；长度或数据非法时返回 0，不修改输出。 */
uint8_t DR16_Parse(const uint8_t *buf, uint16_t size, DR16_Data_t *data);

#endif
