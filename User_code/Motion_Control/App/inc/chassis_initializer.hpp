/**
 * @file chassis_initializer.hpp
 * @brief 底盘电机、VMC、PID 和俯仰历史初始化方法接口。
 */

#ifndef CHASSIS_INITIALIZER_HPP
#define CHASSIS_INITIALIZER_HPP

namespace chassis
{

/**
 * @brief 集中执行底盘启动阶段的固定参数初始化。
 *
 * 类不保存运行时状态；静态方法只负责把既有工程对象初始化为迁移前的数值。
 */
class ChassisInitializer
{
public:
    /** @brief 初始化四个腿关节、Yaw 和拨弹电机参数，并启用气弹簧补偿。 */
    static void InitializeMotors();

    /** @brief 初始化左右五连杆 VMC 几何参数和机构镜像方向。 */
    static void InitializeVmc();

    /** @brief 按原数值初始化正常、收腿、防劈叉、小陀螺、横滚和云台 PID。 */
    static void InitializePids();

    /** @brief 把当前 IMU pitch 转为弧度并推进 pitch_trans 前后帧历史。 */
    static void UpdatePitchHistory();
};

} // namespace chassis

#endif // CHASSIS_INITIALIZER_HPP
