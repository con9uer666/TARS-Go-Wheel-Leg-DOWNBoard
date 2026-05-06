#ifndef POWER_OBSERVER_LIMIT_H
#define POWER_OBSERVER_LIMIT_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float lambda;
    float integral;
} PowerObsCtrl;

typedef struct
{
    float body_distance_error;
    float speed_error;
    float yaw_error;
    float d_yaw;
} PowerObsInput;

typedef struct
{
    float body_distance_error;
    float speed_error;
    float yaw_error;
    float d_yaw;
    float lambda;
} PowerObsOutput;

void  PowerObsCtrl_Reset(PowerObsCtrl *ctrl);

float PowerObsCtrl_ComputeLambda(PowerObsCtrl *ctrl,
                                 float power_limit,
                                 float measured_power,
                                 float dt_s,
                                 float kp,
                                 float ki,
                                 float alpha_fall,
                                 float alpha_rise,
                                 float lambda_min);

void  PowerObsCtrl_Apply(const PowerObsCtrl *ctrl,
                         const PowerObsInput *in,
                         PowerObsOutput *out);

extern float obs_lambda_min;

#ifdef __cplusplus
}
#endif

#endif
