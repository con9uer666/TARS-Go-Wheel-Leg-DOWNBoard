#ifndef _POWERCTRAL_H_
#define _POWERCTRAL_H_

#define TOQUE_CONST  600 //�ؽ�Ħ��ϵ��

#include "main.h"

typedef struct
{
	float output;
	float LastOutput[4];//��һ�ε����Ҳ���ǵ�ǰ�ĵ���������ֵ
	float SumPowerSpeed;//ת��ƽ��
	float SumPowerTorque;//����ƽ��
	float EffetivePower;//��е����
	float InitialGivePower[4];//����ǰ����
	float InitialTotalPower;//����ǰ�ܹ���
	float PredictPower;//Ԥ�⹦��
	float MeasurePower;//��������
	float TotalPower;//�ܹ���
	float scaleFactor;//����ϵ��
	float paramVector[3][1];//��̬��ϳ�ʼֵ����
	float transVector[3][3];//��̬������������仯�ķ�Χ
	float toque_coefficient;//��λת������������ת��ת����ʵA��rad/s
	float a; //����ƽ����ϵ��
	float k2;//ת��ƽ����ϵ��
	float constant;//��ϳ�����
	float kp;//平步反解系数
	float sdmax;
	uint16_t moto_type;//�������
	uint16_t UserPowerLimit;
	uint16_t MaxPowerLimit;
	float InputPower;
}ChassisPower;

extern ChassisPower whell_power;
void PowerCtrl();

#endif