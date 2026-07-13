#ifndef _WATTMETER_H_
#define _WATTMETER_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float voltage;    // V
    float current;    // A
    float power;      // W (voltage * current)
} WattMeter_Data_t;

extern WattMeter_Data_t wattmeter_data;

void WattMeter_CAN_Parse(uint8_t *rx_data);

#ifdef __cplusplus
}
#endif

#endif /* _WATTMETER_H_ */
