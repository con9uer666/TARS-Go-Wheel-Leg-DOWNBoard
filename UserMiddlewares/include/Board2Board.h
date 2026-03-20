#ifndef _BOARD2BOARD_H_
#define _BOARD2BOARD_H_
#include "stdint.h"
#include "usart.h"
#include "main.h"
#include "stdbool.h"
#define KEY_NUM 18
#define MAX_KEY_CALLBACK_NUM 10

//各个键位的ID，即对应结构体在keyList中的下标
typedef enum _KeyType
{
	//可用键盘按键，顺序需与遥控器数据相同
	Key_W=1<<0,
	Key_S=1<<1,
	Key_A=1<<2,
	Key_D=1<<3,
	Key_Shift=1<<4,
	Key_Ctrl=1<<5,
	Key_Q=1<<6,
	Key_E=1<<7,
	Key_R=1<<8,
	Key_F=1<<9,
	Key_G=1<<10,
	Key_Z=1<<11,
	Key_X=1<<12,
	Key_C=1<<13,
	Key_V=1<<14,
	Key_B=1<<15,
	//鼠标左右键
	Key_Left=1<<16,
	Key_Right=1<<17,
	Key_All=0x3ffff
}KeyType;

//按键事件类型
typedef enum _KeyEventType
{
	KeyEvent_OnClick,
	KeyEvent_OnLongPress,
	KeyEvent_OnDown,
	KeyEvent_OnUp,
	KeyEvent_OnPressing//只要按下就会在每个检测周期调用一次
}KeyEventType;

//组合键类型
typedef enum _KeyCombineType
{
	CombineKey_None,
	CombineKey_Ctrl,
	CombineKey_Shift
}KeyCombineType;

//回调函数
typedef void (*KeyCallbackFunc)(KeyType,KeyCombineType,KeyEventType);

/**
  * @brief  遥控数据结构体
  */
typedef struct 
{
  /* rocker channel information */
  int16_t ch1;
  int16_t ch2;
  int16_t ch3;
  int16_t ch4;
  /* left and right lever information */
  uint8_t left;
  uint8_t right;
  /* mouse movement and button information */
  struct
  {
    int16_t x;
    int16_t y;
    int16_t z;

    uint8_t l;
    uint8_t r;
  } mouse;
  /* keyboard key information */
  union {
    uint16_t key_code;
    struct
    {
      uint16_t W : 1;
      uint16_t S : 1;
      uint16_t A : 1;
      uint16_t D : 1;
      uint16_t SHIFT : 1;
      uint16_t CTRL : 1;
      uint16_t Q : 1;
      uint16_t E : 1;
      uint16_t R : 1;
      uint16_t F : 1;
      uint16_t G : 1;
      uint16_t Z : 1;
      uint16_t X : 1;
      uint16_t C : 1;
      uint16_t V : 1;
      uint16_t B : 1;
    } bit;
  } kb;
  int16_t wheel;
}RC_TypeDef;

//按键结构体，用于计算键盘/鼠标的按键事件
typedef struct _Key
{
	//需要配置的参数
	uint16_t clickDelayTime;//按下多久才算单击一次
	uint16_t longPressTime;//按下多久才算长按
	
	//用来使用的参数，仅在对应条件有效的一瞬间为1
	uint8_t isClicked;
	uint8_t isLongPressed;
	uint8_t isUp;
	uint8_t isPressing;
	
	//回调
	struct
	{
		KeyCombineType combineKey[MAX_KEY_CALLBACK_NUM];//组合键类型列表
		KeyCallbackFunc func[MAX_KEY_CALLBACK_NUM];//回调函数列表
		uint8_t number;//已注册的回调个数
	}onClickCb,onLongCb,onDownCb,onUpCb,onPressCb;//四种按键事件的回调
	
	//中间变量
	uint8_t lastState;//1/0为按下/松开
	uint32_t startPressTime;
}Key;

extern RC_TypeDef rcInfo;
extern uint8_t STOPFLAG;
extern uint8_t FEEDBACK;
extern uint8_t upstairs_flag;

void B2B_ParseUsart(void);
//RC初始化
void B2B_Init(void);
void B2B_LostCallback(void);

//注册一个按键回调
void RC_Register(uint32_t key,KeyCombineType combine,KeyEventType event,KeyCallbackFunc func);

#endif


