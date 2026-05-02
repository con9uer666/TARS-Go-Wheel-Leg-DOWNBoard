#include "wattmeter.h"

WattMeter_Data_t wattmeter_data;

void WattMeter_CAN_Parse(uint8_t *rx_data)
{
    int16_t raw_voltage = (int16_t)(rx_data[0] | ((uint16_t)rx_data[1] << 8));
    int16_t raw_current = (int16_t)(rx_data[2] | ((uint16_t)rx_data[3] << 8));

    wattmeter_data.voltage = raw_voltage / 100.0f;
    wattmeter_data.current = raw_current / 100.0f;
    wattmeter_data.power   = wattmeter_data.voltage * wattmeter_data.current;
}
