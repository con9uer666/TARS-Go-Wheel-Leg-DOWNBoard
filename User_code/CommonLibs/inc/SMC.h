#ifndef SMC_H
#define SMC_H

typedef struct _SMC
{
	double	u1_SMC,u2_SMC,A_SMC,B_SMC,k_SMC,Disturb_SMC,f_SMC ;
	double sys;
}SMC;

// º¯ÊýÉùÃ÷
void SMCInit(SMC *smc,double A,double B,double k,double f,double disturb);
void SMC_Calc(SMC*smc,double reference,double feedback);
#endif 
