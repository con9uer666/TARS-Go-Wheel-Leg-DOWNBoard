/**
  *********************************************************************
  * @file      observe_task.c/h
  * @brief     该任务是对机体运动速度估计，用于抑制打滑
  * @note       
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */
	
#include "observe_task.h"
#include "kalman_filter1.h"
#include "imu_temp_ctrl.h"
#include "cmsis_os.h"
#include "motor.h"
#include "float.h"
#include "string.h"
#include "freertos.h"




KalmanFilter_t vaEstimateKF;	   // 卡尔曼滤波器结构体

float vaEstimateKF_F[4] = {1.0f, 0.003f, 
                           0.0f, 1.0f};	   // 状态转移矩阵，控制周期为0.001s

float vaEstimateKF_P[4] = {1.0f, 0.0f,
                           0.0f, 1.0f};    // 后验估计协方差初始值

float vaEstimateKF_Q[4] = {100.0f, 0.0f, 
                           0.0f, 1.0f};    // Q矩阵初始值

float vaEstimateKF_R[4] = {1000000.0f, 0.0f, 
                            0.0f,  0.01f};
														
float vaEstimateKF_K[4];
													 
const float vaEstimateKF_H[4] = {1.0f, 0.0f,
                                 0.0f, 1.0f};	// 设置矩阵H为常量
														 															 											 																					

float vel_acc[2];
float kalman_body_speed;

																 
void Observe_Tasks(void const * argument)
{
	xvEstimateKF_Init(&vaEstimateKF);
	TickType_t xLastWakeTime = xTaskGetTickCount(); 
  	while(1)
	{  
    	xvEstimateKF_Update(&vaEstimateKF, accel_b[1], body_speed);
		//原地自转的过程中v_filter和x_filter应该都是为0
		kalman_body_speed = vel_acc[0];//得到卡尔曼滤波后的速度
		osDelayUntil(&xLastWakeTime, 2);
	}
}

void xvEstimateKF_Init(KalmanFilter_t *EstimateKF)
{
    Kalman_Filter1_Init(EstimateKF, 2, 0, 2);	// 状态向量2维 没有控制量 测量向量2维
	
		memcpy(EstimateKF->F_data, vaEstimateKF_F, sizeof(vaEstimateKF_F));
    memcpy(EstimateKF->P_data, vaEstimateKF_P, sizeof(vaEstimateKF_P));
    memcpy(EstimateKF->Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
    memcpy(EstimateKF->R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));
    memcpy(EstimateKF->H_data, vaEstimateKF_H, sizeof(vaEstimateKF_H));

}

int iii;

void xvEstimateKF_Update(KalmanFilter_t *EstimateKF ,float acc,float vel)
{   	
	 memcpy(EstimateKF->Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
   memcpy(EstimateKF->R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));
	iii = sizeof(vaEstimateKF_R);
    //卡尔曼滤波器测量值更新
    EstimateKF->MeasuredVector[0] =	vel;//测量速度
    EstimateKF->MeasuredVector[1] = acc;//测量加速度
    		
    //卡尔曼滤波器更新函数
    Kalman_Filter1_Update(EstimateKF);

    // 提取估计值
    for (uint8_t i = 0; i < 2; i++)
    {
      vel_acc[i] = EstimateKF->FilteredValue[i];
    }
}

float  RAMP_float( float  final, float  now, float  ramp )
{
	  float  buffer = 0;
		
	  buffer = final - now;
	
		if (buffer > 0)
		{
				if (buffer > ramp)
				{  
						now += ramp;
				}   
				else
				{
						now += buffer;
				}
		}
		else
		{
				if (buffer < -ramp)
				{
						now += -ramp;
				}
				else
				{
						now += buffer;
				}
		}
		
		return now;
}
