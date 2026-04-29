#ifndef BUZZER_H
#define BUZZER_H

#include "main.h"
#include "tim.h"

void Buzzer_init(void);
void Set_buzzer_frq(int frq);
void Run_Buzzer(void);
void Stop_Buzzer(void);
void Buzzer_do(void);
void Buzzer_re(void);
void Buzzer_mi(void);
void Buzzer_fa(void);
void Buzzer_sol(void);
void Buzzer_la(void);
void Buzzer_si(void);

void Buzzer_High_do(void);
void Buzzer_High_re(void);
void Buzzer_High_mi(void);
void Buzzer_High_fa(void);
void Buzzer_High_sol(void);
void Buzzer_High_la(void);
void Buzzer_High_si(void);

#endif
