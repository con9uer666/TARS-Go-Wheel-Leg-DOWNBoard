#ifndef STATE_H
#define STATE_H

#include "main.h"
#include "remoter.h"

extern enum State System_State;
extern uint8_t State_changed;
extern float bat_voltage;

enum State{
    Error,
    Normal,
    Fast,
    Slippage
};

#endif
