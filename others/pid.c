#include "pid.h"
#include "arm_math.h"


/**
 * @brief 
 * 
 * @param PID 
 * @param Kp 
 * @param Ki 
 * @param Kd 
 * @param out_limit 
 * @param i_limit 
 * @param Integraldead_zone 积分累加死区范围，当error在这个范围外时，积分不累加
 * @param deadzone 输出死区范围
 */
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

//重写PID输出限幅
void PID_Reset_OutLimit(PID_t *PID, float new_limit)
{
	PID->out_limit = new_limit;
}

//归零PID积分
void PID_Clear(PID_t *PID)
{
    PID->I = 0;
}

//PID误差计算
void PID_Set_Error(PID_t *PID, float now, float target)
{
    PID->error = target - now;
}

//PID计算	//!只负责计算，不负责更新误差
float PID_coculate(PID_t *PID)
{
	//死区处理
	if((PID->error <= PID->deadzone) && (PID->error >= -PID->deadzone))
	{
			PID->error = 0;
	}
	else
	{
			if(PID->error >= 0)PID->error -= PID->deadzone;
			if(PID->error <= 0)PID->error += PID->deadzone;
	}

	//基本计算
    float P=0,D=0,out=0;
    P = PID->Kp * PID->error;
    D = PID->Kd * (PID->error - PID->pre_error);

    //积分累加
	if(PID->error <= PID->Integraldead_zone && PID->error >= -PID->Integraldead_zone)
		PID->I += PID->Ki * PID->error;

	//积分限幅
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
    out = P + PID->I + D;

    //输出限幅
    if (out >= PID->out_limit || out <= -PID->out_limit)
	{
      if (out > 0)
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
