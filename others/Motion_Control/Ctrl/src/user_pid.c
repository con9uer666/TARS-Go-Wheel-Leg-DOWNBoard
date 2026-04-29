#include "user_pid.h"
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
 * @param I_step 积分变化率限制
 * @param Integraldead_zone 积分累加死区范围，当error在这个范围外时，积分不累加
 * @param deadzone 输出死区范围
 */
void PID_INIT(user_pid_t *PID, float Kp, float Ki, float Kd, float out_limit, float i_limit, float I_step, float Integraldead_zone, float deadzone)
{
    PID->Kp = Kp;
    PID->Ki = Ki;
    PID->Kd = Kd;
    PID->out_limit = out_limit;
    PID->I_limit = i_limit;
    PID->I_step = I_step;
    PID->Integraldead_zone = Integraldead_zone;
    PID->deadzone = deadzone;

    PID->I = 0;
}

//重写PID输出限幅
void PID_Reset_OutLimit(user_pid_t *PID, float new_limit)
{
	PID->out_limit = new_limit;
}

//归零PID积分
void PID_Clear(user_pid_t *PID)
{
    PID->I = 0;
}

//PID误差计算
void PID_Set_Error(user_pid_t *PID, float now, float target)
{
    PID->error = target - now;
}


// --------------------------
// 辅助函数1：将角度归一化到 [-π, π] 区间
// --------------------------
static float NormalizeAngle(float angle)
{
    angle = fmodf(angle, 2.0f * PI); // 先取模到 [-2π, 2π]
    if (angle > PI) {
        angle -= 2.0f * PI;
    } else if (angle < -PI) {
        angle += 2.0f * PI;
    }
    return angle;
}

// --------------------------
// 辅助函数2：计算从当前角度到目标角度的最小角度差（结果在 [-π, π] 内）
// --------------------------
static float ShortestAngleDelta(float target_angle, float current_angle)
{
    target_angle = NormalizeAngle(target_angle);
    current_angle = NormalizeAngle(current_angle);
    
    float delta = target_angle - current_angle;
    return NormalizeAngle(delta); // 对差值再次归一化，确保是最小路径
}


//角度控制专用的PID误差设置函数
void PID_Set_AngleError(user_pid_t *PID, float current_angle, float target_angle)
{
    // 计算最小角度差并赋值给PID的error
    PID->error = ShortestAngleDelta(target_angle, current_angle);

}

//PID计算	//!只负责计算，不负责更新误差
float PID_coculate(user_pid_t *PID)
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

    if (PID->error <= PID->Integraldead_zone && PID->error >= -PID->Integraldead_zone)
{
    float I_target = PID->I + PID->Ki * PID->error;

    // I_step = 0 时直接走原始积分
    if (PID->I_step <= 0.0f)
    {
        PID->I = I_target;
    }
    else
    {
        float delta = I_target - PID->I;

        // I变化率限制
        if (delta > PID->I_step)
            delta = PID->I_step;
        else if (delta < -PID->I_step)
            delta = -PID->I_step;

        PID->I += delta;
    }
}

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
