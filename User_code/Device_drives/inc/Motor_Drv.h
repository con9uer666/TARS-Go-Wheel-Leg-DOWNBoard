/**
 * @file Motor_Drv.h
 * @brief 电机驱动层接口：DJI/达妙电机数据解析与CAN控制。
 *
 * 提供以下功能：
 *   - 电机数据结构体定义（关节电机 Joint_Motor_t、轮毂电机 Wheel_Motor_t）
 *   - 达妙电机 MIT 模式控制（扭矩/速度）
 *   - DJI 3508 电机扭矩控制
 *   - 电机初始化、使能、失能、清错
 *   - 浮点/定点转换工具函数
 */

#ifndef MOTOR_DRV_H
#define MOTOR_DRV_H

#include "main.h"
#include "USER_CAN.h"

/** @brief DJI电机回传数据结构（3508 轮毂电机）。 */
typedef struct DJI_Rx_Data{
    uint16_t Position;      /**< 编码器位置（原始值，0-65535） */
    float Velocity;         /**< 角速度 (rad/s) */
    int16_t Speed;          /**< 转速（原始值，rpm 相关） */
    int16_t Torque;         /**< 扭矩电流（原始值） */
    uint8_t temperate;      /**< 电机温度 (°C) */
    int16_t last_ecd;       /**< 上次编码器值（用于多圈计数） */
}DJI_Rx_Data_t;

/** @brief 达妙电机回传数据结构（DM8009/DM4310/DM2325）。 */
typedef struct Rx_Data{
    uint8_t ID;             /**< 电机ID (低4位), 状态 (高4位) */
    uint8_t State;          /**< 电机状态码 */
    float Position;         /**< 位置 (rad) */
    float Velocity;         /**< 速度 (rad/s) */
    float Torque;           /**< 扭矩 (N·m) */
    float T_Mos;            /**< MOS管温度 (°C) */
    float T_Rotor;          /**< 转子温度 (°C) */
}Rx_Data_t;

/** @brief 达妙关节电机完整数据结构。 */
typedef struct Joint_Motor{
    Rx_Data_t Rx_Data;      /**< 回传数据 */
    float Target_Torque;    /**< 目标扭矩 (N·m) */
    float Target_Speed;     /**< 目标速度 (rad/s) */
    float TMAX;             /**< 扭矩上限 (N·m) */
    float PMAX;             /**< 位置上限 (rad) */
    float VMAX;             /**< 速度上限 (rad/s) */
    float KD_MAX;           /**< KD上限 */
    float KD_MIN;           /**< KD下限 */
    uint16_t motor_id;      /**< CAN标准帧ID */
    uint8_t  last_error_code;     /**< 最近错误码，0=正常 */
    uint8_t  clear_pending;       /**< 待清错标志 */
    uint8_t  enable_pending;      /**< 待使能标志 */
}Joint_Motor_t;

/** @brief DJI 3508 轮毂电机完整数据结构。 */
typedef struct Wheel_Motor{
    DJI_Rx_Data_t Rx_Data;  /**< 回传数据 */
    int16_t TX_data;        /**< 上次发送的扭矩电流值 */
    float Target_Torque;    /**< 目标扭矩 (N·m) */
    float TMAX;             /**< 扭矩上限 (N·m) */
    float PMAX;             /**< 位置上限 (rad) - 保留 */
    float VMAX;             /**< 速度上限 (rad/s) - 保留 */
    uint16_t motor_id;      /**< CAN标准帧ID */
}Wheel_Motor_t;

/* ======================== 函数声明 ======================== */

void DM_Joint_Motor_Init(Joint_Motor_t *Motor, float TMAX, float PMAX,float VMAX, uint16_t motor_id);
void DM_Wheel_Motor_Init(Wheel_Motor_t *Motor, float TMAX, float PMAX,float VMAX, uint16_t motor_id);

/** @brief 浮点数转定点整数（用于CAN协议编码）。 */
int float_to_uint(float x_float, float x_min, float x_max, int bits);

/** @brief 定点整数转浮点数（用于CAN协议解码）。 */
float uint_to_float(int x_int, float x_min, float x_max, int bits);

/** @brief 解析达妙电机 (DM8009) 回传数据帧。 */
void DM8009_Get_Data(uint8_t *Data, Joint_Motor_t *Motor);

/** @brief 解析DJI 3508电机回传数据帧。 */
void DJ3508_Get_Data(uint8_t *Data, Wheel_Motor_t *Motor);

/* CAN控制指令函数 */
void Enable_DM_Motor_MIT(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id);
void Disable_DM_Motor(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id);
void Clear_DM_Motor_Error(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id);

/** @brief 达妙电机MIT模式扭矩控制。 */
void DM_Motor_MIT_Torque_ctrl(FDCAN_HandleTypeDef *hfdcan, Joint_Motor_t Motor, float torq);

/** @brief 达妙电机MIT模式速度控制（带位置/速度/扭矩前馈）。 */
void DM_Motor_MIT_Speed_ctrl(FDCAN_HandleTypeDef *hfdcan, Joint_Motor_t motor, float pos, float vel, float tor, float kp, float kd);

/** @brief DJI 3508电机扭矩控制。 */
void DJI_Motor_Torque_Ctrl(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id, float torque);

/* ======================== 全局电机实例 ======================== */

/** @brief 左轮 DJI 3508 电机 */
extern Wheel_Motor_t L_DJ3508;
/** @brief 右轮 DJI 3508 电机 */
extern Wheel_Motor_t R_DJ3508;

/** @brief 左腿 DM8009 关节电机数组：[0]=上关节, [1]=下关节 */
extern Joint_Motor_t L_DM8009[2];
/** @brief 右腿 DM8009 关节电机数组：[0]=上关节, [1]=下关节 */
extern Joint_Motor_t R_DM8009[2];
/** @brief 云台偏航 DM4310 电机 */
extern Joint_Motor_t Yaw_DM4310;
/** @brief 发射机构 DM2325 电机 */
extern Joint_Motor_t Shooter_DM2325;

#endif