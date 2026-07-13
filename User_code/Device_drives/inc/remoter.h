#ifndef REMOTER_H
#define REMOTER_H

#include <main.h>

#define BUFF_SIZE	100



typedef struct
{
	uint16_t CH1;//ͨ��1��ֵ
	uint16_t CH2;//ͨ��2��ֵ
	uint16_t CH3;//ͨ��3��ֵ
	uint16_t CH4;//ͨ��4��ֵ
	uint16_t CH5;//ͨ��5��ֵ
	uint16_t CH6;//ͨ��6��ֵ
	uint16_t CH7;//ͨ��7��ֵ
	uint16_t CH8;//ͨ��8��ֵ
	uint16_t CH9;//ͨ��9��ֵ
	uint16_t CH10;//ͨ��10��ֵ
	uint16_t CH11;//ͨ��11��ֵ
	uint16_t CH12;//ͨ��12��ֵ
	uint16_t CH13;//ͨ��13��ֵ
	uint16_t CH14;//ͨ��14��ֵ
	uint16_t CH15;//ͨ��15��ֵ
	uint16_t CH16;//ͨ��16��ֵ
	uint8_t ConnectState;//ң���������������״̬ 0=δ���ӣ�1=��������
	uint8_t SW1;
	uint8_t SW2;
	uint8_t SW3;
	uint8_t SW4;
	
}SBUS_CH_Struct;

extern SBUS_CH_Struct SBUS_CH;



void Remoter_Init(void);

void Keyboard_Simulate(void);

#endif
