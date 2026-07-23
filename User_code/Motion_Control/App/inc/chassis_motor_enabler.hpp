/**
 * @file chassis_motor_enabler.hpp
 * @brief 底盘六个 DM 电机顺序使能接口。
 */

#ifndef CHASSIS_MOTOR_ENABLER_HPP
#define CHASSIS_MOTOR_ENABLER_HPP

namespace chassis
{

/** @brief 执行带固定 5 ms 间隔的全部 DM 电机启动使能序列。 */
class ChassisMotorEnabler
{
public:
    /** @brief 依次使能左右腿、拨弹和 Yaw 电机，并开启掉使能监督。 */
    static void EnableAll();
};

} // namespace chassis

#endif // CHASSIS_MOTOR_ENABLER_HPP
