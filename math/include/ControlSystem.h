// 控制系统头文件
#ifndef CONTROLSYSTEM_H
#define CONTROLSYSTEM_H

// 这里包括SMC和FricModelSIM的头文件
#include "SMC.h"
#include "FricModelSIM.h"

// 控制系统函数声明
void ControlSystem_Initialize();
void ControlSystem_Update(double reference, double *rpm, double *control_signal);

#endif // CONTROLSYSTEM_H
