#ifndef PID_H
#define PID_H

#define LIMIT(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

typedef struct {
    float Kp;
    float Kd;
    float Ki;
    float I;
    float I_limit;
    float out_limit;
    float error;
    float pre_error;
    float output;
		float deadzone;
		float Integraldead_zone;
}PID_t;

float PID_coculate(PID_t *PID);
void PID_INIT(PID_t *PID, float Kp, float Ki, float Kd, float out_limit, float i_limit, float Integraldead_zone, float deadzone);
void PID_Reset_OutLimit(PID_t *PID, float new_limit);
void PID_Clear(PID_t *PID);
void PID_Set_Error(PID_t *PID, float now, float target);
void PID_Error_Correct(PID_t *PID);
float PID_coculate_speed(PID_t *PID);

#endif
