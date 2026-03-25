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
		float deadzone;//输出死区范围
		float Integraldead_zone;//积分累加死区范围，当error在这个范围外时，积分不累加
}user_pid_t;


float PID_coculate(user_pid_t *PID);
void PID_INIT(user_pid_t *PID, float Kp, float Ki, float Kd, float out_limit, float i_limit, float Integraldead_zone, float deadzone);
void PID_Reset_OutLimit(user_pid_t *PID, float new_limit);
void PID_Clear(user_pid_t *PID);
void PID_Set_Error(user_pid_t *PID, float now, float target);
void PID_Error_Correct(user_pid_t *PID);
float PID_coculate_speed(user_pid_t *PID);

#endif
