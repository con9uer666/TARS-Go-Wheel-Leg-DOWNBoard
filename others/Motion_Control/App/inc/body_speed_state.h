/**
 * @file body_speed_state.h
 * @brief 车身速度状态判断（加速/减速/静止）。
 *
 * 根据遥控器指令 Remote_control_y 与卡尔曼滤波速度 kalman_body_speed
 * 综合判断当前车身速度状态，输出枚举值并同步写入全局变量。
 *
 * 判断逻辑：
 * - |Remote_control_y| > 0.1 → 加速（ACCEL）
 * - |Remote_control_y| ≤ 0.1 且 |kalman_body_speed| ≥ 0.5 → 减速（DECEL，惯性滑行）
 * - |Remote_control_y| ≤ 0.1 且 |kalman_body_speed| < 0.5 → 静止（STATIONARY）
 */

#ifndef BODY_SPEED_STATE_H
#define BODY_SPEED_STATE_H

/**
 * @brief 车身速度状态枚举。
 */
typedef enum {
    BODY_SPEED_ACCEL = 0,      /**< 加速：遥控器有指令（Remote_control_y 非零） */
    BODY_SPEED_DECEL,          /**< 减速：遥控器无指令，但车身速度绝对值 ≥ 0.5 m/s（惯性滑行） */
    BODY_SPEED_STATIONARY      /**< 静止：遥控器无指令，且车身速度绝对值 < 0.5 m/s */
} BodySpeedState_t;

/** @brief 当前车身速度状态，每次调用 BodySpeedState_Get() 同步更新。 */
extern BodySpeedState_t g_body_speed_state;

/**
 * @brief 获取当前车身速度状态。
 *
 * 每次调用会根据 Remote_control_y 和 kalman_body_speed 重新判断，
 * 结果同时写入全局变量 g_body_speed_state 并通过返回值返回。
 *
 * @return 当前 BodySpeedState_t 枚举值。
 */
BodySpeedState_t BodySpeedState_Get(void);

#endif /* BODY_SPEED_STATE_H */