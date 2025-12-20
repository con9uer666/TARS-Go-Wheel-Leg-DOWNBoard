#include "pid.h"
#include "arm_math.h"



void PID_INIT(PID_t *PID, float Kp, float Ki, float Kd, float out_limit, float i_limit, float Integraldead_zone, float deadzone)
{
    PID->Kp = Kp;
    PID->Ki = Ki;
    PID->Kd = Kd;
    PID->out_limit = out_limit;
    PID->I_limit = i_limit;
		PID->Integraldead_zone = Integraldead_zone;
		PID->deadzone = deadzone;

    PID->I = 0;
}

void PID_Reset_OutLimit(PID_t *PID, float new_limit)
{
	PID->out_limit = new_limit;
}

void PID_Clear(PID_t *PID)
{
    PID->I = 0;
}

void PID_Set_Error(PID_t *PID, float now, float target)
{
    PID->error = target - now;
}

void PID_Error_Correct(PID_t *PID)
{
    if(PID->error >= 8192)PID->error -= 16384;
    else if(PID->error <= -8192)PID->error += 16384;
}

float PID_coculate(PID_t *PID)
{
	if(PID->error <= PID->deadzone && PID->error >= -PID->deadzone)
	{
			PID->error = 0;
	}
	else
	{
			if(PID->error >= 0)PID->error -= PID->deadzone;
			if(PID->error <= 0)PID->error += PID->deadzone;
	}
	
    float P=0,D=0,out=0;
    P= PID->Kp * PID->error;
	D= PID->Kd * (PID->error-PID->pre_error);
	if(PID->error <= PID->Integraldead_zone && PID->error >= -PID->Integraldead_zone)
	PID->I += PID->Ki * PID->error;

    if(PID->I>=PID->I_limit || PID->I<=-PID->I_limit)
    {
        if(PID->I>0)
        {
			PID->I=PID->I_limit;
        }
        else
        {
			PID->I=-PID->I_limit;
        }
    }
    out=P+PID->I+D;
    if(out>=PID->out_limit||out<=-PID->out_limit)
	{
		if(out>0)
		{
			out=PID->out_limit;
		}
		else
		{
			out=-PID->out_limit;
		}
		
	}
    PID->pre_error=PID->error;
    PID->output = out;
    return out;
}


float PID_coculate_speed(PID_t *PID)
{
    float P=0,D=0,out=0;
    P= PID->Kp * PID->error;
	D= PID->Kd * (PID->error-PID->pre_error);
	PID->I += PID->Ki * PID->error;
	if(PID->error <= 10 && PID->error >= -10)
	{
		PID->I = 0;
	}

    if(PID->I>=PID->I_limit || PID->I<=-PID->I_limit)
    {
        if(PID->I>0)
        {
			PID->I=PID->I_limit;
        }
        else
        {
			PID->I=-PID->I_limit;
        }
    }
    out=P+PID->I+D;
    if(out>=PID->out_limit||out<=-PID->out_limit)
	{
		if(out>0)
		{
			out=PID->out_limit;
		}
		else
		{
			out=-PID->out_limit;
		}
		
	}
    PID->pre_error=PID->error;
    PID->output = out;
    return out;
}



