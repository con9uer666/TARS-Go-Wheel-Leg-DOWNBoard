#include "RLS.h" // RLS 参数更新模块头文件。

 float kainVector[3][1] = {0}; // 增益向量缓存。
 float Xsample[3][1] = {0}; // 输入样本向量。
 float Ysample[1][1] = {0}; // 输出样本向量。
 float lamdaPowerAutoUpdate = 0.9999; // 遗忘因子。

 float TRANS_M_X[3][1] = {0}; // 中间矩阵缓存。
 float XT[1][3] = {0}; // 转置后的输入样本。
 float XT_M_TRANS[1][3] = {0}; // 中间矩阵缓存。
 float XT_M_TRANS_M_X[1][1] = {0}; // 标量中间结果缓存。
 float XT_M_PARAM[1][1] = {0}; // 参数乘积缓存。
 float deta_PARAM[3][1] = {0}; // 参数增量缓存。

 float lastParamVector[3][1] = {0}; // 参数更新前缓存。
 float lastTrans[3][3] = {0}; // 协方差更新前缓存。

 float KAIN_M_XT[3][3] = {0}; // 增益与转置输入的乘积缓存。
 float KAIN_M_XT_M_TRANS[3][3] = {0}; // 增益更新协方差缓存。

 arm_matrix_instance_f32 paramMatrix; // 参数矩阵实例。
 arm_matrix_instance_f32 transMatrix; // 协方差矩阵实例。
 arm_matrix_instance_f32 kainMatrix; // 增益矩阵实例。
 arm_matrix_instance_f32 XsampleMatrix; // 输入样本矩阵实例。
 arm_matrix_instance_f32 YsampleMatrix; // 输出样本矩阵实例。
 arm_matrix_instance_f32 TRANS_M_XMatrix; // 中间矩阵实例。
 arm_matrix_instance_f32 XTMatrix; // 转置输入矩阵实例。
 arm_matrix_instance_f32 XT_M_TRANSMatrix; // 中间矩阵实例。
 arm_matrix_instance_f32 XT_M_TRANS_M_XMatrix; // 标量矩阵实例。
 arm_matrix_instance_f32 XT_M_PARAMMatrix; // 参数乘积矩阵实例。
 arm_matrix_instance_f32 deta_PARAMMatrix; // 参数增量矩阵实例。
 arm_matrix_instance_f32 lastParamMatrix; // 旧参数矩阵实例。
 arm_matrix_instance_f32 lastTransMatrix; // 旧协方差矩阵实例。
 arm_matrix_instance_f32 KAIN_M_XTMatrix; // 增益乘积矩阵实例。
 arm_matrix_instance_f32 KAIN_M_XT_M_TRANSMatrix; // 协方差更新矩阵实例。

/*
 * @brief 初始化 RLS 更新器。
 * @param power 功率对象指针；其 paramVector/transVector 将作为 RLS 的在线参数与协方差容器。
 */
void PowerControl_AutoUpdateParamInit(ChassisPower* power)
{
    lamdaPowerAutoUpdate = 0.9999f; // 步骤1：设置遗忘因子（越接近1越平滑，越小越快追踪变化）。

    // 步骤2：将参数向量/协方差矩阵与外部功率对象绑定。
    arm_mat_init_f32(&paramMatrix, 3, 1, (float *)power->paramVector);
    arm_mat_init_f32(&transMatrix, 3, 3, (float *)power->transVector);

    // 步骤3：初始化 RLS 计算过程中需要的所有临时矩阵对象。
    arm_mat_init_f32(&kainMatrix, 3, 1, (float *)kainVector);
    arm_mat_init_f32(&XsampleMatrix, 3, 1, (float *)Xsample);
    arm_mat_init_f32(&YsampleMatrix, 1, 1, (float *)Ysample);
    arm_mat_init_f32(&TRANS_M_XMatrix, 3, 1, (float *)TRANS_M_X);
    arm_mat_init_f32(&XTMatrix, 1, 3, (float *)XT);
    arm_mat_init_f32(&XT_M_TRANSMatrix, 1, 3, (float *)XT_M_TRANS);
    arm_mat_init_f32(&XT_M_TRANS_M_XMatrix, 1, 1, (float *)XT_M_TRANS_M_X);
    arm_mat_init_f32(&XT_M_PARAMMatrix, 1, 1, (float *)XT_M_PARAM);
    arm_mat_init_f32(&deta_PARAMMatrix, 3, 1, (float *)deta_PARAM);
    arm_mat_init_f32(&lastParamMatrix, 3, 1, (float *)lastParamVector);
    arm_mat_init_f32(&lastTransMatrix, 3, 3, (float *)lastTrans);
    arm_mat_init_f32(&KAIN_M_XTMatrix, 3, 3, (float *)KAIN_M_XT);
    arm_mat_init_f32(&KAIN_M_XT_M_TRANSMatrix, 3, 3, (float *)KAIN_M_XT_M_TRANS);
} // 初始化结束。
extern float powerPredict; // 外部预测功率变量。

/*
 * @brief 使用模型参数预测损耗项。
 * @param x1 轮速平方和。
 * @param x2 轮扭矩平方和。
 * @param x3 常数项（双轮一般为 2）。
 * @param power 功率对象指针。
 * @return 损耗功率估计值。
 */
float PowerControl_ModelLossPredict(float x1, float x2, float x3, const ChassisPower *power)
{
    if (power == NULL)
    {
        return 0.0f;
    }

    return power->paramVector[0][0] * x1 + power->paramVector[1][0] * x2 + power->paramVector[2][0] * x3;
}

/*
 * @brief 预测轮组总电功率。
 * @param wl 左轮速度（rad/s）。
 * @param wr 右轮速度（rad/s）。
 * @param tl 左轮扭矩（N*m）。
 * @param tr 右轮扭矩（N*m）。
 * @param power 功率对象指针。
 * @return 预测总电功率（W）。
 */
float PowerControl_WheelPowerPredict(float wl, float wr, float tl, float tr, const ChassisPower *power)
{
    float x1;
    float x2;
    float x3;
    float loss_power;
    float mech_power;
    float total_power;

    if (power == NULL)
    {
        return 0.0f;
    }

    // 步骤1：构造模型输入向量。
    x1 = wl * wl + wr * wr;
    x2 = tl * tl + tr * tr;
    x3 = 2.0f; // 两个轮电机的常数项总和。

    // 步骤2：计算损耗功率项与机械功项。
    loss_power = PowerControl_ModelLossPredict(x1, x2, x3, power);
    mech_power = power->toque_coefficient * (wl * tl + wr * tr);

    // 步骤3：合成总电功率预测值并做下限保护。
    total_power = loss_power + mech_power;

    if (total_power < 0.0f)
    {
        total_power = 0.0f;
    }

    return total_power;
}

/*
 * @brief 根据输入/输出样本执行 RLS 在线更新。
 * @param x1 输入样本1（轮速平方和）。
 * @param x2 输入样本2（轮扭矩平方和）。
 * @param x3 输入样本3（常数项）。
 * @param y 输出样本（实测损耗功率）。
 * @param power 功率对象（按值传入，供异常分支访问参数）。
 * @return 当前模型损耗功率估计值。
 */
float PowerControl_AutoUpdateParam(float x1, float x2, float x3, float y,ChassisPower power)
{
     if (powerPredict < 0) // 预测值异常时直接返回。
     {
         return PowerControl_ModelLossPredict(Xsample[0][0], Xsample[1][0], Xsample[2][0], &power); // 返回当前模型输出。
     } // 异常分支结束。
// x1 为 电机转速平方和，x2 为电机扭矩平方和，x3 为常数项（偏置项），y 为总功率，effectivePower 为机械功率 // 输入含义说明。
// 若使用 setCurrent，需要保证 x2 与当前电机实际输出一致，需要根据实际情况对当前功率进行修正 // 使用说明。

    // 步骤1：写入本周期输入/输出样本。
    Xsample[0][0] = x1;
    Xsample[1][0] = x2;
    Xsample[2][0] = x3;
    Ysample[0][0] = y;

    // 步骤2：当样本有效时执行标准 RLS 更新流程。
    if (y > 1) // 观测值足够大时更新参数。
    {
        arm_mat_trans_f32(&XsampleMatrix, &XTMatrix); // 计算输入转置。
        arm_mat_mult_f32(&transMatrix, &XsampleMatrix, &TRANS_M_XMatrix); // 计算协方差与输入乘积。
        arm_mat_mult_f32(&XTMatrix, &transMatrix, &XT_M_TRANSMatrix); // 计算转置输入与协方差乘积。
        arm_mat_mult_f32(&XT_M_TRANSMatrix, &XsampleMatrix, &XT_M_TRANS_M_XMatrix); // 计算标量分母项。
        arm_mat_scale_f32(&TRANS_M_XMatrix, // 计算卡尔曼增益。
                          1.0f / (1 + (XT_M_TRANS_M_X[0][0] / lamdaPowerAutoUpdate) / lamdaPowerAutoUpdate),
                          &kainMatrix); // 保存增益矩阵。

        arm_mat_mult_f32(&XTMatrix, &paramMatrix, &XT_M_PARAMMatrix); // 计算当前预测值。
        arm_mat_scale_f32(&kainMatrix, Ysample[0][0] - XT_M_PARAM[0][0], &deta_PARAMMatrix); // 计算参数增量。

        arm_mat_scale_f32(&paramMatrix, 1.0f, &lastParamMatrix); // 备份旧参数。
        arm_mat_add_f32(&lastParamMatrix, &deta_PARAMMatrix, &paramMatrix); // 更新参数。

        arm_mat_mult_f32(&kainMatrix, &XTMatrix, &KAIN_M_XTMatrix); // 计算增益与转置输入乘积。
        arm_mat_mult_f32(&KAIN_M_XTMatrix, &transMatrix, &KAIN_M_XT_M_TRANSMatrix); // 计算协方差修正量。
        arm_mat_scale_f32(&transMatrix, 1.0f, &lastTransMatrix); // 备份旧协方差。
        arm_mat_sub_f32(&lastTransMatrix, &KAIN_M_XT_M_TRANSMatrix, &transMatrix); // 更新协方差。
    } // 更新分支结束。
    // 步骤3：返回更新后参数对应的损耗预测值，便于上层调试。
    return PowerControl_ModelLossPredict(Xsample[0][0], Xsample[1][0], Xsample[2][0], &power);
} // 更新函数结束。
