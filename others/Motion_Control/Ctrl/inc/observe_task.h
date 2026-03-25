#ifndef __OBSERVE_TASK_H
#define __OBSERVE_TASK_H

#include "kalman_filter1.h"

extern float kalman_body_speed;

extern void Observe_task(void);
extern void xvEstimateKF_Init(KalmanFilter_t *EstimateKF);
extern void xvEstimateKF_Update(KalmanFilter_t *EstimateKF ,float acc,float vel);
extern float  RAMP_float(float final, float now, float ramp);	

#endif





