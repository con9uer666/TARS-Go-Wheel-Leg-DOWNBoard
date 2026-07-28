/**
 * @file chassis_runtime_state.cpp
 * @brief 底盘跨动作共享状态：反馈量、物理常数、控制流标志、共享腿部 PID。
 *
 * 这些变量不专属于任何单一动作组（被站起/收腿/上台阶/坐地等多处共用），
 * 故集中定义于此，其余动作专属变量定义在各自动作文件中。
 * 全部 extern 声明见 chassis_behavior_tree.h。
 */

extern "C"
{
#include "chassis_behavior_tree.h"
}

/** @brief 上下板通信使用的轮足底盘目标、模式和反馈聚合结构。 */
Foot_Chassis_t Foot_Chassis;

/** @brief 预留功率预测观测量，供调试器和旧功率模块读取。 */
float powerPredict;

/** @brief 正常控制允许的最短虚拟腿长，单位 m。 */
float LEG_MIN_LENTH = 0.20f;
/** @brief 正常控制允许的最长虚拟腿长，单位 m。 */
float LEG_MAX_LENTH = 0.39f;

/** @brief 左腿相对车体角 b_phi0 的兼容观测量，单位 rad。 */
float L_b_phi0;
/** @brief 右腿相对车体角 b_phi0 的兼容观测量，单位 rad。 */
float R_b_phi0;

/** @brief 俯仰角历史：[0] 当前帧、[1] 上一帧，单位 rad。 */
float pitch_trans[2];
/** @brief 俯仰角速度，单位 rad/s。 */
float d_pitch;
/** @brief 俯仰角速度一阶滤波中新样本权重。 */
float alpha_d_pitch = 1.0f;

/** @brief 右轮叠加腿摆动后的等效轮缘速度。 */
float Wr;
/** @brief 左轮叠加腿摆动后的等效轮缘速度。 */
float Wl;
/** @brief 等效轮速一阶滤波中历史样本权重。 */
float alpha_W = 0.9f;
/** @brief 左侧运动学解算得到的车体水平速度，单位 m/s。 */
float body_speed_L;
/** @brief 右侧运动学解算得到的车体水平速度，单位 m/s。 */
float body_speed_R;
/** @brief 左右侧融合后的车体水平速度，单位 m/s。 */
float body_speed;
/** @brief 经过输入和斜坡处理后的目标车体速度，单位 m/s。 */
float target_body_speed;
/** @brief 正常平移速度绝对值上限，单位 m/s。 */
float speed_limit = 1.3f;
/** @brief Error_Calculate 输出的本周期速度误差，单位 m/s。 */
float speed_error;
/** @brief 目标车速滤波中新目标样本权重。 */
float alpha_target_body_speed = 1.0f;
/** @brief 单侧车体速度滤波中新样本权重。 */
float alpha_body_speed = 1.0f;
/** @brief 实际车体水平位移积分，单位 m。 */
float body_distance;

/** @brief 目标车体水平位移积分，单位 m。 */
float target_body_distance = 0.0f;
/** @brief Error_Calculate 输出的目标与实际位移误差，单位 m。 */
float body_distance_error;

/** @brief Error_Calculate 输出的偏航控制误差或小陀螺控制量。 */
float yaw_error;
/** @brief 偏航角历史：[0] 当前帧、[1] 上一帧，单位 rad。 */
float yaw_trans[2];
/** @brief IMU 解算后的偏航角速度，单位 rad/s。 */
float d_yaw;
/** @brief 偏航角速度滤波中新样本权重。 */
float alpha_d_yaw = 0.8f;

/** @brief 目标横滚角一阶滤波中新目标样本权重。 */
float alpha_target_roll = 0.05f;

/** @brief 单腿支持力命令兼容上限，单位 N。 */
float Leg_F0_Limit = 500.0f;

/** @brief 单腿承担的车体重力补偿基准，单位 N。 */
float mg = 0.0f;

/** @brief 腿部前倾角偏置，正值前倾、负值后倾，单位 rad。 */
float b_phi0_offset = 0.0f;

/** @brief 正常平衡左腿腿长 PID。 */
user_pid_t L_Leg_L0_PID;
/** @brief 正常平衡右腿腿长 PID。 */
user_pid_t R_Leg_L0_PID;

/** @brief 收腿/坐地/上台阶共用的左腿腿长位置环 PID。 */
user_pid_t L_Leg_L0_POS_PID;
/** @brief 收腿/坐地/上台阶共用的右腿腿长位置环 PID。 */
user_pid_t R_Leg_L0_POS_PID;
/** @brief 收腿/坐地/上台阶共用的左腿腿长速度环 PID。 */
user_pid_t L_Leg_L0_SPD_PID;
/** @brief 收腿/坐地/上台阶共用的右腿腿长速度环 PID。 */
user_pid_t R_Leg_L0_SPD_PID;

/** @brief 左右腿支持力差动横滚补偿 PID。 */
user_pid_t Roll_Comp_PID;

/** @brief 防劈叉 PID；Kp/Kd 每周期按平均腿长覆盖。 */
user_pid_t Leg_AntiSplit_PID;

/** @brief 左腿收腿转角位置环 PID。 */
user_pid_t L_Leg_Middle_PID;
/** @brief 右腿收腿转角位置环 PID。 */
user_pid_t R_Leg_Middle_PID;
/** @brief 左腿收腿转角速度环 PID。 */
user_pid_t L_Leg_dphi0_PID;
/** @brief 右腿收腿转角速度环 PID。 */
user_pid_t R_Leg_dphi0_PID;

/** @brief 云台俯仰控制 PID。 */
user_pid_t gimbal_pitch_pid;

/** @brief 正常模式公共目标腿长，启动时与最短腿长一致，单位 m。 */
float target_Leg_L0 = 0.23f;
/** @brief 横滚补偿后左腿独立目标腿长，单位 m。 */
float target_L_Leg_L0 = 0.20f;
/** @brief 横滚补偿后右腿独立目标腿长，单位 m。 */
float target_R_Leg_L0 = 0.20f;

/** @brief 预留腿高等待计数，保留供旧调试逻辑使用。 */
int height_wait;
/** @brief 预留单字节调试变量。 */
uint8_t temp1;

/** @brief 起立前收腿流程首次运行标志；1 表示需要初始化动作状态。 */
uint8_t first_run = 1U;

/** @brief 上台阶内部相位兼容标志：0 伸腿/未运行，1 收腿恢复。 */
uint8_t upstares_mode = 0U;
/** @brief 预留跨动作到位计数兼容量。 */
int ready_count = 0;

/** @brief 当前腿长档位：0 短腿、1 中间态、2 长腿。 */
uint8_t leg_state = 0U;
/** @brief 腿长斜坡和档位切换使用的周期计数。 */
int leg_state_count;

/** @brief 目标车速通用斜坡发生器。 */
RampGenerator Target_Speed_Ramp;

/** @brief 云台/底盘跟随关系：1 云台跟随底盘，0 底盘跟随云台。 */
uint8_t gimbal_follow_flag = 1U;

/** @brief Motor_task 标称控制频率，单位 Hz。 */
uint16_t motor_HZ = 500U;

/** @brief 车体中心到单侧车轮的横向距离，单位 m。 */
float wheel_track_R = 0.19242f;

/** @brief 下板侧最终偏航控制输出兼容观测量。 */
float down_board_yaw_output = 0.0f;
