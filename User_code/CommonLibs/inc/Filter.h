#ifndef _FILTER_H_
#define _FILTER_H_

#include "stdint.h"

//均值滤波器最大允许的缓冲区长度
#define AVER_FILTER_MAX_SIZE 100

//均值滤波器
typedef struct
{
	float buffer[AVER_FILTER_MAX_SIZE];
	uint16_t bufferSize;
}AverFilter;


typedef struct 
{
    float P; //估算协方差
    float G; //卡尔曼增益
    float Q; //过程噪声协方差,Q增大，动态响应变快，收敛稳定性变坏
    float R; //测量噪声协方差,R增大，动态响应变慢，收敛稳定性变好
    float Output; //卡尔曼滤波器输出 
}KFPTypeS;

void Filter_InitAverFilter(AverFilter *filter,uint16_t size);
float Filter_AverCalc(AverFilter *filter,float newVal);
void Filter_AverClear(AverFilter *filter);
float KalmanFilter(KFPTypeS *kfp, float input);

#endif
