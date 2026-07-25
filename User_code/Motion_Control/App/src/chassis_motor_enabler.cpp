/**
 * @file chassis_motor_enabler.cpp
 * @brief 全部机身 DM 关节电机使能动作。
 */

#include "chassis_motor_enabler.hpp"
#include "arm_math.h"

extern "C"
{
#include "chassis_behavior_tree.h"
#include "user_pid.h"
#include "Motor_Drv.h"
#include "Gimbal.h"
#include "User_State.h"
#include "USER_CAN.h"
#include "VMC.h"
#include "observe_task.h"
#include "Leg_Control.h"
#include "Self_Righting.h"
#include "Board2Board.h"
#include "Slope.h"
#include "Wheel_Leg_about.h"
#include "controller.h"
#include "remoter.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "fdcan.h"
#include <math.h>
#include <stdint.h>
#include "imu_temp_ctrl.h"
#include "Angle_about.h"
#include "PowerCtrl.h"
#include "Gas_Spring.h"
#include "buzzer.h"
#include "Wheel_End_Velocity.h"
}

/**
 * @brief 按左腿、右腿、拨弹、Yaw 的旧顺序使能六个 DM 电机。
 *
 * 每次使能后保留 5 ms 延时；全部命令发送完成后才开启电机掉使能监督。
 */
void chassis::ChassisMotorEnabler::EnableAll()
{
    Enable_DM_Motor_MIT(&hfdcan2, 0x01);
    osDelay(5);
    Enable_DM_Motor_MIT(&hfdcan2, 0x02);
    osDelay(5);

    Enable_DM_Motor_MIT(&hfdcan1, 0x01);
    osDelay(5);
    Enable_DM_Motor_MIT(&hfdcan1, 0x02);
    osDelay(5);

    Enable_DM_Motor_MIT(&hfdcan3, 0x11);
    osDelay(5);
    Enable_DM_Motor_MIT(&hfdcan3, 0x10);
    osDelay(5);

    // 标记机身所有DM关节电机期望状态为"已使能"，开启监督
    motor_should_enabled = 1;
}
