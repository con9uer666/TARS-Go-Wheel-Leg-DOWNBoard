#ifndef POWER_OBSERVER_LIMIT_H
#define POWER_OBSERVER_LIMIT_H

/*
 * 本头文件定义“观测量门控”模块：
 * 输入：功率限制、缓冲能量、预测功率；
 * 输出：观测缩放系数 lambda 及门控后的 LQR 观测量。
 */

#include <stdint.h> // 基础整型定义。

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 观测量功率控制参数。
 * 通过缓冲能量与功率预测共同计算观测量缩放系数 lambda。
 */
typedef struct
{
    float buffer_min;        /* 缓冲能量低阈值，低于该值进入保守控制 */
    float buffer_ref;        /* 缓冲能量参考值，用于修正可用功率 */
    float buffer_max;        /* 缓冲能量高阈值，高于该值可放开控制 */

    float lambda_min;        /* 最小缩放系数，避免完全丢失控制输入 */
    float lambda_rise_rate;  /* lambda 上升速率（1/s），防止突增 */
    float lambda_fall_rate;  /* lambda 下降速率（1/s），防止突降 */

    float power_margin;      /* 功率安全裕量，预留给测量误差与扰动 */
    float buffer_gain;       /* 缓冲能量对允许功率的修正增益 */
} PowerObsCtrlParam;

/*
 * 观测量功率控制器状态。
 * 保存参数与运行时量，建议定义为静态或全局单例。
 */
typedef struct
{
    PowerObsCtrlParam param; /* 控制参数 */

    float lambda;               /* 当前观测量缩放系数 */
    float predicted_power;      /* 当前周期预测功率 */
    float allowed_power;        /* 当前周期允许功率 */
    float last_predicted_power; /* 上一周期预测功率 */
} PowerObsCtrl;

/*
 * 需要门控的输入观测量。
 * 建议只放运动需求相关状态，平衡核心状态尽量不缩放。
 */
typedef struct
{
    float body_distance_error;  /* 车体位移误差 */
    float speed_error;          /* 车体速度误差 */
    float yaw_error;            /* 偏航角误差 */
    float d_yaw;                /* 偏航角速度 */
} PowerObsInput;

/*
 * 门控后的输出观测量。
 * 输出可直接替换原观测量参与 LQR 计算。
 */
typedef struct
{
    float body_distance_error;  /* 门控后位移误差 */
    float speed_error;          /* 门控后速度误差 */
    float yaw_error;            /* 门控后偏航误差 */
    float d_yaw;                /* 门控后偏航角速度 */

    float lambda;               /* 本周期使用的缩放系数 */
} PowerObsOutput;

/*
 * @brief 填充一组稳定可用的默认参数。
 * @param param 输出参数结构体指针。
 */
void PowerObsCtrl_DefaultParam(PowerObsCtrlParam *param);

/*
 * @brief 初始化控制器状态。
 * @param ctrl 控制器对象指针。
 * @param param 参数指针，传 NULL 时使用默认参数。
 */
void PowerObsCtrl_Init(PowerObsCtrl *ctrl, const PowerObsCtrlParam *param);

/*
 * @brief 结合功率限制、缓冲能量与预测功率，更新并返回 lambda。
 * @param ctrl 控制器对象指针。
 * @param power_limit 本周期功率限制（W）。
 * @param power_buffer 本周期缓冲能量（J）。
 * @param predicted_power 本周期预测功率（W）。
 * @param dt_s 控制周期（s）。
 * @return 本周期更新后的 lambda。
 */
float PowerObsCtrl_ComputeLambda(PowerObsCtrl *ctrl,
                                 float power_limit,
                                 float power_buffer,
                                 float predicted_power,
                                 float dt_s);

/*
 * @brief 对输入观测量执行缩放。
 * @param ctrl 控制器对象指针，传 NULL 时按 lambda=1 处理。
 * @param in 门控前输入观测量。
 * @param out 门控后输出观测量。
 */
void PowerObsCtrl_Apply(const PowerObsCtrl *ctrl,
                        const PowerObsInput *in,
                        PowerObsOutput *out);

#ifdef __cplusplus
}
#endif

#endif
