#ifndef __RLS__
#define __RLS__

#include "arm_math.h" // arm_matrix_instance_f32 等矩阵类型。
#include "PowerCtrl.h" // ChassisPower 结构体定义。
 
/*
 * @brief 初始化 RLS 更新器。
 * @param power 功率控制对象指针，内部会绑定其 paramVector/transVector 作为 RLS 工作内存。
 */
void PowerControl_AutoUpdateParamInit(ChassisPower* power);

/*
 * @brief RLS 在线更新函数。
 * @param x1 输入样本1：轮速平方和。
 * @param x2 输入样本2：轮扭矩平方和。
 * @param x3 输入样本3：常数项（双轮场景通常取 2.0f）。
 * @param y 输出样本：实测损耗功率（总电功率减机械功）。
 * @param power 功率对象（按值传入，主要用于异常分支下的参数访问）。
 * @return 当前模型估计的损耗功率。
 */
float PowerControl_AutoUpdateParam(float x1, float x2, float x3, float y,ChassisPower power);

/*
 * @brief 使用当前参数向量计算模型损耗项。
 * @param x1 轮速平方和。
 * @param x2 轮扭矩平方和。
 * @param x3 常数项。
 * @param power 功率对象指针（内部读取 paramVector）。
 * @return 损耗功率估计值。
 * @note 数学表达式：p_loss = a*x1 + b*x2 + c*x3。
 */
float PowerControl_ModelLossPredict(float x1, float x2, float x3, const ChassisPower *power);

/*
 * @brief 轮组总电功率预测（损耗项 + 机械功项）。
 * @param wl 左轮角速度（rad/s）。
 * @param wr 右轮角速度（rad/s）。
 * @param tl 左轮目标扭矩（N*m）。
 * @param tr 右轮目标扭矩（N*m）。
 * @param power 功率对象指针（读取 paramVector 与 toque_coefficient）。
 * @return 总电功率预测值（W）。
 */
float PowerControl_WheelPowerPredict(float wl, float wr, float tl, float tr, const ChassisPower *power);


#endif
