/**
 * @file Lqr_Error_Calculate.c
 * @brief 统一计算 LQR 使用的速度、位移和 yaw 误差。
 *
 * Error_Calculate() 是本模块的统一更新入口。它先确定底盘控制模式，再分别
 * 计算 yaw 控制量和平移倍率，随后更新 speed_error、body_distance_error，
 * 最后对供 LQR 使用的 yaw_error 进行低通滤波。
 */

#include "Lqr_Error_Calculate.h"
#include "chassis_behavior_tree.h"
#include "PowerCtrl.h"
#include "observe_task.h"
#include "ramp_generator.h"
#include "body_speed_state.h"
#include "Motor_Drv.h"
#include "user_pid.h"
#include "Maths_about.h"
#include "Gimbal.h"
#include <math.h>

/* ---- 模块内部状态与控制模式 ---- */

/** @brief 普通平移模式使用的目标速度斜坡发生器。 */
static RampStep_Handle target_speed_ramp;
/** @brief 标记普通平移目标速度斜坡是否已经完成首次初始化。 */
static uint8_t target_speed_ramp_inited = 0;

/** @brief 决定目标平移速度采用普通前向输入还是小陀螺二维输入。 */
typedef enum
{
    SPEED_CONTROL_NORMAL = 0,
    SPEED_CONTROL_SPIN
} SpeedControlMode;

/** @brief 本控制周期的 yaw/旋转控制路径。 */
typedef enum
{
    CHASSIS_CONTROL_GIMBAL_FOLLOW = 0,
    CHASSIS_CONTROL_NORMAL,
    CHASSIS_CONTROL_SPIN_ACTIVE,
    CHASSIS_CONTROL_SPIN_EXITING
} ChassisControlMode;

/* ---- 内部辅助函数 ---- */

/**
 * @brief 小陀螺模式下的平移方向倍率计算。
 *
 * 根据车身相对云台的角度、目标平移方向和安装偏置，生成小陀螺模式下
 * 沿车身前后方向的平移倍率：
 *   - |a - offset| ≤ tol            → ratio 从 0 线性降至 -1（方向重合时 -1）
 *   - |a - offset ± π| ≤ tol        → ratio 从 0 线性升至 +1（反向时 +1）
 *   - 其余                          → ratio = 0
 *
 * 调用者会将遥控目标方向与安装偏置合并后传入 offset。
 *
 * @param[in] a      当前车身相对云台的 yaw 角 yaw_angle_PI，单位 rad。
 * @param[in] tol    窗口半宽度，单位 rad。
 * @param[in] offset 目标平移方向与安装补偿角之和，单位 rad。
 * @return 平移倍率 ratio，范围 [-1, 1]。
 */
static float Spin_Translation_Ratio(float a, float tol, float offset)
{
    if (tol <= 1e-6f) return 0.0f;
    a -= offset;
    while (a >  PI) a -= 2.0f * PI;
    while (a < -PI) a += 2.0f * PI;
    if (a >= -tol && a <= tol)         return -(tol - fabsf(a)) / tol;
    if (a >=  PI - tol)                return  (tol - fabsf(a - PI)) / tol;
    if (a <= -PI + tol)                return  (tol - fabsf(a + PI)) / tol;
    return 0.0f;
}

/**
 * @brief 计算普通模式下由原始 yaw 误差决定的平移速度倍率。
 *
 * yaw 误差为 0 时倍率为 1；误差随绝对值增大而线性下降，达到
 * 0.7 rad 后倍率保持为 0。这里必须使用限幅前的原始 yaw 误差，
 * 避免 LQR 的 yaw 限幅削弱大角度偏航时的减速效果。
 *
 * @param[in] raw_yaw_error 套圈到 [-PI, PI] 后、尚未限幅的 yaw 误差，单位 rad。
 * @return 平移速度倍率，范围 [0, 1]。
 */
static float Normal_Translation_Ratio(float raw_yaw_error)
{
    float ratio = 1.0f - fabsf(raw_yaw_error) / 0.7f;
    if (ratio < 0.0f) ratio = 0.0f;
    return ratio;
}

/**
 * @brief 根据当前控制请求和小陀螺标志确定本周期的控制模式。
 *
 * 状态判定优先级为：云台跟随、进入或保持小陀螺、小陀螺退出、普通运行。
 * 进入小陀螺时会先置位 spinning_flag，保证首帧立即采用小陀螺速度逻辑；
 * 回到普通模式时清除旋转 PID 积分，并用当前 d_yaw 同步旋转目标和反馈滤波值，
 * 避免下次进入小陀螺时继承退出阶段的残余状态。
 *
 * @return 本周期使用的底盘控制模式 ChassisControlMode。
 */
static ChassisControlMode Chassis_Control_Mode_Update(void)
{
    if (gimbal_follow_flag == 1)
        return CHASSIS_CONTROL_GIMBAL_FOLLOW;

    if (Foot_Chassis.Chassis_Mode == 1 && spinning_usable == 1)
    {
        // 在本周期速度计算前置位，消除进入小陀螺时的一帧模式延迟。
        spinning_flag = 1;
        return CHASSIS_CONTROL_SPIN_ACTIVE;
    }

    if (spinning_flag == 1)
    {
        spinning_usable = 0;
        return CHASSIS_CONTROL_SPIN_EXITING;
    }

    spinning_pid.I = 0;
    spinning_target_d_yaw_cmd = d_yaw;
    spinning_d_yaw_feedback = d_yaw;
    spinning_usable = 1;
    return CHASSIS_CONTROL_NORMAL;
}

/**
 * @brief 更新普通模式的 yaw 误差，并分别输出原始值和 LQR 限幅值。
 *
 * 首先对云台 yaw 电机位置进行零点修正和 [-PI, PI] 套圈；原始误差通过
 * raw_yaw_error 输出，供普通模式的平移减速使用。返回值按照当前车速动态
 * 限幅，随后由 Error_Calculate() 低通滤波并写入 LQR 状态 yaw_error。
 *
 * @param[out] raw_yaw_error 限幅前的原始 yaw 误差，单位 rad。
 * @return 经过车速相关限幅的 yaw 误差，单位 rad。
 */
static float Normal_Yaw_Error_Update(float *raw_yaw_error)
{
    float yaw_motor_position = Yaw_DM4310.Rx_Data.Position - head_forward_angle;

    if (yaw_motor_position > PI)
        yaw_motor_position -= 2.0f * PI;
    if (yaw_motor_position < -PI)
        yaw_motor_position += 2.0f * PI;

    *raw_yaw_error = yaw_motor_position;

    float yaw_error_max = 2.5f - fabsf(kalman_body_speed);
    if (yaw_error_max < 0.05f)
        yaw_error_max = 0.05f;

    if (yaw_motor_position > yaw_error_max)
        yaw_motor_position = yaw_error_max;
    if (yaw_motor_position < -yaw_error_max)
        yaw_motor_position = -yaw_error_max;

    return yaw_motor_position;
}

/* ---- 速度与位移误差计算 ---- */

/**
 * @brief 根据速度模式和平移倍率统一更新目标速度及 speed_error。
 *
 * 普通模式读取前向目标速度并使用腿长相关斜坡；小陀螺模式读取二维目标速度
 * 的模长。两种模式均执行功率限速，再乘以调用者计算好的 translation_ratio，
 * 因而本函数不依赖 yaw_error，也不负责判断底盘控制状态。
 *
 * @param[in] mode 速度目标的计算模式：普通平移或小陀螺平移。
 * @param[in] translation_ratio 方向匹配/偏航衰减倍率，普通模式范围 [0, 1]，
 *                              小陀螺模式范围 [-1, 1]。
 * @note speed_error 最终限幅为 speed_limit 的 70%。
 */
static void Speed_Error_Update(SpeedControlMode mode, float translation_ratio)
{

//region[rgb(0, 50, 0)] 平移速度上限
    // 长腿使用固定上限；短腿根据基础功率限制调整上限。
    if (Foot_Chassis.Target_Leg_State == 1) // 长腿
    {
        speed_limit = 2.0f;
    } 
    else // 短腿
    {
        // 短腿：base_power_limit 在 [60 W, 100 W] 时，speed_limit 从 2.1 线性增加到 2.5 m/s。
        if (base_power_limit <= 60.0f)         speed_limit = 2.1f;
        else if (base_power_limit >= 100.0f)   speed_limit = 2.5f;
        else                                   speed_limit = 2.1f + (base_power_limit - 60.0f) * 0.01f;
    }
    if (mode == SPEED_CONTROL_SPIN) {
        speed_limit = 4.0f;     // 小陀螺平移速度使用独立上限。
    }
//endregion

    if (mode == SPEED_CONTROL_SPIN)
    {
        // 小陀螺模式保留云台坐标系二维平移指令的模长（Vy=前，Vx=右）。
        float v_mag = sqrtf(Foot_Chassis.Target_Vx * Foot_Chassis.Target_Vx
                          + Foot_Chassis.Target_Vy * Foot_Chassis.Target_Vy);
        // 先执行固定速度上限，再应用功率控制给出的目标速度限制。
        if (v_mag > speed_limit) v_mag = speed_limit;
        target_body_speed = PowerCtrl_LimitTargetSpeed(v_mag, speed_limit);

    } 
    else 
    {
        // 普通模式只使用云台坐标系前向分量，并对目标速度施加斜坡。
        {
            float raw_target = Foot_Chassis.Target_Vy;
            raw_target = PowerCtrl_LimitTargetSpeed(raw_target, speed_limit);

            // 根据目标腿长在短腿与长腿参数之间线性插值斜坡参数。
            float leg_ratio = (target_Leg_L0 - LEG_MIN_LENTH) / (LEG_MAX_LENTH - LEG_MIN_LENTH);
            if (leg_ratio < 0.0f) leg_ratio = 0.0f;
            if (leg_ratio > 1.0f) leg_ratio = 1.0f;

            float step_accel = 1.2f + leg_ratio * (0.3f - 1.2f);
            float step_decel = 0.8f + leg_ratio * (0.3f - 0.8f);
            float ramp_rate  = 1.0f + leg_ratio * (0.2f - 1.0f);

            // 首次使用当前目标速度初始化，避免斜坡状态从无效初值起步。
            if (!target_speed_ramp_inited) 
            {
                RampStep_Init(&target_speed_ramp, target_body_speed);
                target_speed_ramp_inited = 1;
            }
            target_body_speed = RampStep_Update(&target_speed_ramp, raw_target,
                                                 step_accel, step_decel, ramp_rate, 0.002f);
        }

    }

    // 平移倍率由 yaw/方向计算逻辑提供，本函数不直接读取 yaw_error。
    target_body_speed = target_body_speed * translation_ratio;

    // 速度误差定义为目标车身速度减去卡尔曼估计的实际车身速度。
    speed_error = target_body_speed - kalman_body_speed;

    // 限制进入 LQR 的速度误差，避免大速度阶跃产生过大控制输出。
    if (speed_error >= speed_limit * 0.7f)
        speed_error = speed_limit * 0.7f;
    if (speed_error <= -speed_limit * 0.7f)
        speed_error = -speed_limit * 0.7f;
}

/**
 * @brief 计算车身位移误差 body_distance_error。
 *
 * 以 0.002s 为周期，对 kalman_body_speed 积分得到实际位移 body_distance，
 * 对 target_body_speed 积分得到目标位移 target_body_distance（不是用带限幅后的 speed_error），
 * 两者之差即为 body_distance_error。
 *
 * 调用 BodySpeedState_Get() 判断当前车身速度状态：
 * - ACCEL / STATIONARY：正常积分累加位移；
 * - DECEL：清零实际位移、目标位移和位移误差；
 * - 小陀螺运行或退出：同样清零，不在旋转过程中累计直线位移误差。
 */
void Distance_Error_Set()
{
    /* 同步更新车身速度状态，后续逻辑据此决定是否累计位移。 */
    int speed_state = BodySpeedState_Get();

    /* 减速滑行或小陀螺期间不累计位移，防止无效误差进入 LQR。 */
    if (speed_state == BODY_SPEED_DECEL || spinning_flag == 1) 
    {
        body_distance = 0.0f;
        target_body_distance = 0.0f;
        body_distance_error = 0.0f;
        return;
    }

    /* ACCEL 或 STATIONARY：分别积分实际速度和目标速度。 */
    body_distance += kalman_body_speed * 0.002f;
    target_body_distance += target_body_speed * 0.002f;
    body_distance_error = target_body_distance - body_distance;
}

/**
 * @brief 统一更新本周期的 yaw、速度和位移误差。
 *
 * 执行顺序固定为：
 * 1. 判定云台跟随/普通/小陀螺运行/小陀螺退出状态；
 * 2. 计算当前模式的 yaw 控制量；
 * 3. 计算普通 yaw 减速倍率或小陀螺方向倍率；
 * 4. 统一更新 target_body_speed 与 speed_error；
 * 5. 更新 body_distance_error；
 * 6. 对最终 yaw_error 进行跨周期低通滤波。
 *
 * 该函数是误差计算模块的统一对外入口，避免 yaw 与速度计算依赖隐式调用顺序。
 */
void Error_Calculate(void)
{
    float translation_ratio = 1.0f;
    SpeedControlMode speed_mode = SPEED_CONTROL_NORMAL;
    ChassisControlMode control_mode = Chassis_Control_Mode_Update();

// region [rgb(42, 42, 75)] 控制模式对应的 yaw 控制量
    switch (control_mode)
    {
    case CHASSIS_CONTROL_GIMBAL_FOLLOW:
        // 云台跟随底盘时关闭底盘侧 yaw 修正。
        yaw_error = 0;
        break;

    case CHASSIS_CONTROL_NORMAL:
    {
        // 原始 yaw 用于平移减速；限幅 yaw 用于 LQR。
        float raw_yaw_error;
        yaw_error = Normal_Yaw_Error_Update(&raw_yaw_error);
        translation_ratio = Normal_Translation_Ratio(raw_yaw_error);
        break;
    }

    case CHASSIS_CONTROL_SPIN_ACTIVE:
        // 小陀螺运行阶段使用转速 PID 输出作为 yaw 控制量。
        speed_mode = SPEED_CONTROL_SPIN;
        yaw_error = spinning_up();
        break;

    case CHASSIS_CONTROL_SPIN_EXITING:
        // 退出阶段继续使用小陀螺平移方式，直到转速和角度同时进入完成窗口。
        speed_mode = SPEED_CONTROL_SPIN;
        yaw_error = spinning_exit();
        if (fabsf(d_yaw) <= 4.0f && fabsf(yaw_angle_PI) <= 0.5f)
            spinning_flag = 0;
        break;
    }
// endregion

// region [rgb(51, 0, 0)] 小陀螺方向倍率
    if (speed_mode == SPEED_CONTROL_SPIN)
    {
        float target_direction = atan2f(Foot_Chassis.Target_Vx, Foot_Chassis.Target_Vy);
        translation_ratio = Spin_Translation_Ratio(yaw_angle_PI, spin_speed_tol_angle,
                                                   target_direction + spin_speed_angle_offset);
    }
// endregion

    // 使用已经确定的速度模式和平移倍率，统一更新速度目标和速度误差。
    Speed_Error_Update(speed_mode, translation_ratio);

    // 速度目标更新后再积分目标/实际位移。
    Distance_Error_Set();

    // 保存上周期滤波结果；函数内 static 限制其作用域，同时保留跨周期状态。
    static float last_yaw_error;
    // 对普通 yaw 误差或小陀螺 PID 输出统一低通滤波后再交给 LQR。
    yaw_error = 0.05 * yaw_error + 0.95 * last_yaw_error;
    last_yaw_error = yaw_error;
}
