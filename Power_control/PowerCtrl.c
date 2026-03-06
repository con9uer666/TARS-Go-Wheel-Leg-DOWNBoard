#include "PowerCtrl.h"
#include "main.h"
#include "arm_math.h"
#include "math.h"
#include "RLS.h"
#include "motor.h"

uint16_t SET_WHEELSPEED_MAX = 8000;

float power_measure;

extern float LQR_K[4][10];

ChassisPower turn_power;
ChassisPower whell_power;
void PowerInit4310(ChassisPower* turn_power)
{
	turn_power->toque_coefficient = 1.0;
	turn_power->paramVector[0][0] = 5.42751688e-04; // RPM
  turn_power->paramVector[1][0] = 4.630179e-01;  // Torque
  turn_power->paramVector[2][0] = 4.24808025;      // constant
	//experience points
  turn_power->transVector[0][0] = 1e-11;   // RPM
  turn_power->transVector[1][1] = 1e-11;   // Torque
  turn_power->transVector[2][2] = 2.5e-5;  // constant
	turn_power->moto_type = 4310;
	turn_power->UserPowerLimit = 60;
}

void PowerInit3508(ChassisPower* whell_power)
{
	whell_power->toque_coefficient =  2.4324e-6f;//(20/16384)*(0.3)*(187/3591)/9.55
	whell_power->paramVector[0][0] = 1.2158888e-7; // RPM
  whell_power->paramVector[1][0] = 1.5822148e-7;  // Torque
  whell_power->paramVector[2][0] = 3.04855824;      // constant
	//experience points
  whell_power->transVector[0][0] = 2.5e-15;   // RPM
  whell_power->transVector[1][1] = 2.5e-15;   // Torque
  whell_power->transVector[2][2] = 0.000025;  // constant
	whell_power->moto_type = 3508;
	whell_power->UserPowerLimit = 120;
}

//自研减速箱3508
void PowerInit3508v1(ChassisPower* whell_power)
{
	whell_power->toque_coefficient =  2.4324e-6f;//(20/16384)*(0.3)*(187/3591)/9.55
	whell_power->paramVector[0][0] = 1.2158888e-7; // RPM
  whell_power->paramVector[1][0] = 1.5822148e-7;  // Torque
  whell_power->paramVector[2][0] = 3.04855824;      // constant
	//experience points
  whell_power->transVector[0][0] = 2.5e-15;   // RPM
  whell_power->transVector[1][1] = 2.5e-15;   // Torque
  whell_power->transVector[2][2] = 0.000025;  // constant
	whell_power->moto_type = 3508;
	whell_power->UserPowerLimit = 120;
}

void PowerCtralInit(ChassisPower* turn_power,ChassisPower* whell_power)
{
//	PowerInit4310(turn_power);
//	PowerControl_AutoUpdateParamInit(turn_power);//��̬��ϳ�ʼ��
//	PowerInit3508(whell_power);
//	PowerControl_AutoUpdateParamInit(&chassis_power->Dj3508Power);
	PowerInit3508v1(whell_power);
	PowerControl_AutoUpdateParamInit(whell_power);
}

//�ֵ�����ʿ���
void Whellv1PowerCtral()
{
	whell_power.a = fmaxf(1e-7, whell_power.paramVector[1][0]);
	whell_power.k2 = fmaxf(whell_power.paramVector[0][0], 1e-7);
	whell_power.constant = fmaxf(0.7f, whell_power.paramVector[2][0]);
	whell_power.kp = 1/sqrt(2*whell_power.a*0.3*0.3*(-0.5*(LQR_K[0][2]+LQR_K[1][2]))*(-0.5*(LQR_K[0][2]+LQR_K[1][2])));
	
	//��������
	whell_power.SumPowerSpeed = 0;
	whell_power.SumPowerTorque = 0;
	whell_power.EffetivePower = 0;
	whell_power.InitialTotalPower = 0;
	whell_power.scaleFactor = 0;
	whell_power.PredictPower = 0;
	
	//��ȡ�����
	//whell_power.MaxPowerLimit = JUDGE_GetChassisPowerLimit();
	//chassis_power_buffer = JUDGE_GetPowerBuffer();
	// if (whell_power.MaxPowerLimit < 15 || whell_power.MaxPowerLimit > 200)
	// {
		whell_power.MaxPowerLimit = whell_power.UserPowerLimit;
	//}
	
	//���������
			whell_power.SumPowerSpeed += L_LK9025.Rx_Data.Speed * L_LK9025.Rx_Data.Speed + R_LK9025.Rx_Data.Speed*R_LK9025.Rx_Data.Speed;
			whell_power.SumPowerTorque+= whell_power.LastOutput[0] * whell_power.LastOutput[0] + whell_power.LastOutput[1]*whell_power.LastOutput[1];
			whell_power.EffetivePower += (whell_power.toque_coefficient * L_LK9025.Rx_Data.Speed * whell_power.LastOutput[0]) + (whell_power.toque_coefficient * R_LK9025.Rx_Data.Speed * whell_power.LastOutput[1]);

	whell_power.PredictPower = whell_power.paramVector[1][0] * whell_power.SumPowerTorque + whell_power.paramVector[0][0] * whell_power.SumPowerSpeed + 2 * whell_power.paramVector[2][0] + whell_power.EffetivePower;
	// power_measure = cap.receive_data.bus_power / 100.0f;
		if (whell_power.EffetivePower > 10 && whell_power.MeasurePower>10 )//&& chassis.move.fastMode == 0)
	{
		PowerControl_AutoUpdateParam(whell_power.SumPowerSpeed / 2.0f, whell_power.SumPowerTorque / 2.0f, 1, (whell_power.MeasurePower-whell_power.EffetivePower) / 2.0f,whell_power);
	}
	//�ֵ�������
	whell_power.InputPower = whell_power.MaxPowerLimit;
	LIMIT(whell_power.InputPower,5, whell_power.MaxPowerLimit+20);
	
	whell_power.sdmax = whell_power.kp*sqrt(whell_power.InputPower);

		// LIMIT(chassis.M3508[i].speedPID.output,-16000,16000);
		whell_power.LastOutput[0] = L_LK9025.TX_data;
		whell_power.LastOutput[1] = R_LK9025.TX_data;
	
}

void PowerCtrl()
{
	Whellv1PowerCtral();
}
