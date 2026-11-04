#include "dr16.h"
#include <assert.h>
#include <string.h>

static void frame(uint8_t *buf, const int16_t ch[4], uint8_t left, uint8_t right)
{
    memset(buf, 0, DR16_FRAME_SIZE);
    for (unsigned i = 0; i < 4; ++i)
    {
        unsigned raw = (unsigned)(ch[i] + 1024);
        for (unsigned b = 0; b < 11; ++b)
            buf[(i * 11 + b) / 8] |= ((raw >> b) & 1U) << ((i * 11 + b) % 8);
    }
    buf[5] |= (left << 6) | (right << 4);
    buf[17] = 4;
}

int main(void)
{
    uint8_t buf[DR16_FRAME_SIZE];
    DR16_Data_t data = {0};
    const int16_t endpoints[4] = {-660, 660, -50, 50};
    frame(buf, endpoints, DR16_SWITCH_UP, DR16_SWITCH_DOWN);
    buf[6] = 0xFE; buf[7] = 0xFF;
    buf[8] = 0x34; buf[9] = 0x12;
    buf[12] = 1; buf[14] = 0x81; buf[15] = 0x80;
    assert(DR16_Parse(buf, sizeof(buf), &data));
    assert(memcmp(data.ch, endpoints, sizeof(endpoints)) == 0);
    assert(data.left == 1 && data.right == 2);
    assert(data.mouse.x == -2 && data.mouse.y == 0x1234);
    assert(data.mouse.l == 1 && data.keyboard == 0x8081 && data.wheel == 0);

    const int16_t center[4] = {-49, 49, 0, 1};
    frame(buf, center, 3, 1);
    assert(DR16_Parse(buf, sizeof(buf), &data));
    for (unsigned i = 0; i < 4; ++i) assert(data.ch[i] == 0);
    assert(data.left_last == 1 && data.right_last == 2);
    DR16_Data_t saved = data;
    assert(!DR16_Parse(buf, 17, &data));
    assert(!DR16_Parse(buf, 19, &data));
    buf[5] &= 0x3F;
    assert(!DR16_Parse(buf, sizeof(buf), &data));
    assert(memcmp(&data, &saved, sizeof(data)) == 0);
    const int16_t invalid[4] = {661, 0, 0, 0};
    frame(buf, invalid, 1, 1);
    assert(!DR16_Parse(buf, sizeof(buf), &data));
    frame(buf, center, 1, 1);
    buf[12] = 2;
    assert(!DR16_Parse(buf, sizeof(buf), &data));
    assert(memcmp(&data, &saved, sizeof(data)) == 0);
    return 0;
}
