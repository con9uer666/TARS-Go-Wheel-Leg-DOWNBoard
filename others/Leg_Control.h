#ifndef LEG_CONTROL_H
#define LEG_CONTROL_H

#include "pid.h"
#include "VMC.h"
#include <stdint.h>

extern int HZ;
extern float torque_limit;
extern float torque_release_rate;
extern float stuck_counter_time_s;
extern int turn_stuck_counter;
extern float turn_stuck_counter_time_s;

int get_sign_float(float value);
float leg_length_control(VMC_t *VMC, float target_L0, float ramp_rate, float F0_max, float I_limit);
float leg_turn_speed_control(VMC_t *VMC, float target_speed, float max_torque, float ramp_rate, float I_limit);
int leg_length_stuck_detect(VMC_t *VMC, float L0_stuck, float stuck_counter_time_s);
int leg_turn_stuck_detect(VMC_t *VMC, float phi0_stuck, float turn_stuck_counter_time_s);

// void leg_control(VMC_t *VMC, float target_speed, float max_torque, float target_L0);

#endif // LEG_CONTROL_H