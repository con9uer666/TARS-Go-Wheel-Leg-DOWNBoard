#include "SMC.h"
#include <math.h>

static double saturation(double x, double a, double b);
static double signum(double val);
static double anti_symble=819.2;
//初始化
void SMCInit(SMC *smc,double A,double B,double k,double f,double disturb)
{	
	smc->A_SMC=A;
	smc->B_SMC=B;
	smc->k_SMC=k;
	smc->f_SMC=f;
	smc->Disturb_SMC=disturb;
}

void SMC_Calc(SMC*smc,double reference,double feedback)
{
	//更新数据
	double u1,u2;	//u1是error
	u1=reference-feedback;
	u2=feedback;
	smc->sys=anti_symble*(smc->k_SMC/smc->B_SMC* u1+ 1/smc->B_SMC* (smc->A_SMC * u2 + smc->Disturb_SMC * smc->B_SMC + smc->f_SMC* signum(u1)) * saturation(u1, -300, 300));
	if(smc->sys>16384)
		smc->sys =16384;
	if(smc->sys<-16384)
		smc->sys=-16384;
}
	



static double saturation(double x, double a, double b) {
    if (x < a) return -1.0;
    else if (x > b) return 1.0;
    else return 0.00333 * x;
}
static double signum(double val) {
    return (0 < val) - (val < 0);
}
