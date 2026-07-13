#include "Filter.h"

//初始化均值滤波器
void Filter_InitAverFilter(AverFilter *filter,uint16_t size)
{
	filter->bufferSize=size;
	for(uint16_t i=0;i<size;i++)
		filter->buffer[i]=0;
}

//均值滤波计算
float Filter_AverCalc(AverFilter *filter,float newVal)
{
	float sum=0;
	for(uint16_t i=0;i<filter->bufferSize-1;i++)//遍历整个缓冲区
	{
		filter->buffer[i]=filter->buffer[i+1];//所有数据前移一位
		sum+=filter->buffer[i];//求和
	}
	filter->buffer[filter->bufferSize-1]=newVal;//写入新数据
	sum+=newVal;
	return sum/filter->bufferSize;//计算均值
}

//清空均值滤波缓冲区
void Filter_AverClear(AverFilter *filter)
{
	for(uint16_t i=0;i<filter->bufferSize;i++)
		filter->buffer[i]=0;
}
/**
  ******************************************************************************
  * @brief  卡尔曼滤波器 函数
  * @param  *kfp    - 卡尔曼结构体参数
  * @param  input   - 需要滤波的参数的测量值（即传感器的采集值）
  * @return 卡尔曼滤波器输出值（最优值）
  * @note   
  ******************************************************************************
  */
float KalmanFilter(KFPTypeS *kfp, float input)
{
    //估算协方差方程：当前 估算协方差 = 上次更新 协方差 + 过程噪声协方差
    kfp->P = kfp->P + kfp->Q;
 
    //卡尔曼增益方程：当前 卡尔曼增益 = 当前 估算协方差 / （当前 估算协方差 + 测量噪声协方差）
    kfp->G = kfp->P / (kfp->P + kfp->R);
 
    //更新最优值方程：当前 最优值 = 当前 估算值 + 卡尔曼增益 * （当前 测量值 - 当前 估算值）
    kfp->Output = kfp->Output + kfp->G * (input - kfp->Output); //当前 估算值 = 上次 最优值
 
    //更新 协方差 = （1 - 卡尔曼增益） * 当前 估算协方差。
    kfp->P = (1 - kfp->G) * kfp->P;
 
     return kfp->Output;
}
