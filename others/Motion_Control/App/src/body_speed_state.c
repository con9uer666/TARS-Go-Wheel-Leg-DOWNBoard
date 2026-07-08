/**
 * @file body_speed_state.c
 * @brief 车身速度状态判断实现。
 *
 * 根据遥控器指令 Remote_control_y 与卡尔曼滤波速度 kalman_body_speed
 * 判断当前车身属于加速、减速还是静止状态。
 */
#include "body_speed_state.h"
#include "chassis_behavior_tree.h" /* Foot_Chassis.Remote_control_y */
#include "observe_task.h"          /* kalman_body_speed                 */
#include <math.h>                  /* fabsf                            */

/** @brief 当前车身速度状态全局变量，由 BodySpeedState_Get() 同步更新。 */
BodySpeedState_t g_body_speed_state = BODY_SPEED_STATIONARY;

/**
 * @brief 获取并更新车身速度状态。
 *
 * 判断逻辑（含状态转移约束）：
 * 1. 若遥控器 y 轴指令绝对值 > 0.1，认为有加速意图 → ACCEL
 * 2. 若遥控器无指令但卡尔曼滤波速度绝对值 ≥ 0.2 m/s → DECEL（惯性减速滑行）
 *    - DECEL 只能从 ACCEL 进入：若前一周期为 STATIONARY，维持 STATIONARY
 * 3. 遥控器无指令且速度接近零 → STATIONARY（静止）
 *
 * @return 当前 BodySpeedState_t 枚举值，同时写入 g_body_speed_state。
 */
BodySpeedState_t BodySpeedState_Get(void)
{
    float rc_y = Foot_Chassis.Remote_control_y;

    /* 遥控器有 y 轴指令 → 加速 */
    if (fabsf(rc_y) > 0.1f) {
        g_body_speed_state = BODY_SPEED_ACCEL;
    }
    /* 遥控器无 y 轴指令，但车身仍有明显速度 → 减速滑行 */
    else if (fabsf(kalman_body_speed) >= 0.1f) {
        /* DECEL 只能从 ACCEL 进入：静止状态下即使有残余速度也不算减速 */
        if (g_body_speed_state == BODY_SPEED_STATIONARY) {
            /* 保持 STATIONARY，不进入 DECEL */
        } else {
            g_body_speed_state = BODY_SPEED_DECEL;
        }
    }
    /* 遥控器无 y 轴指令且速度接近零 → 静止 */
    else {
        g_body_speed_state = BODY_SPEED_STATIONARY;
    }

    return g_body_speed_state;
}
