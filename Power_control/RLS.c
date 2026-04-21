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

void PowerControl_AutoUpdateParamInit(ChassisPower* power) // 初始化 RLS 更新器。
{
     lamdaPowerAutoUpdate = 0.9999f; // 设定遗忘因子。
     arm_mat_init_f32(&paramMatrix, 3, 1, (float *)power->paramVector); // 初始化参数矩阵。
     arm_mat_init_f32(&transMatrix, 3, 3, (float *)power->transVector); // 初始化协方差矩阵。
     arm_mat_init_f32(&kainMatrix, 3, 1, (float *)kainVector); // 初始化增益矩阵。
     arm_mat_init_f32(&XsampleMatrix, 3, 1, (float *)Xsample); // 初始化输入样本矩阵。
     arm_mat_init_f32(&YsampleMatrix, 1, 1, (float *)Ysample); // 初始化输出样本矩阵。
     arm_mat_init_f32(&TRANS_M_XMatrix, 3, 1, (float *)TRANS_M_X); // 初始化中间矩阵。
     arm_mat_init_f32(&XTMatrix, 1, 3, (float *)XT); // 初始化转置矩阵。
     arm_mat_init_f32(&XT_M_TRANSMatrix, 1, 3, (float *)XT_M_TRANS); // 初始化中间矩阵。
     arm_mat_init_f32(&XT_M_TRANS_M_XMatrix, 1, 1, (float *)XT_M_TRANS_M_X); // 初始化标量矩阵。
     arm_mat_init_f32(&XT_M_PARAMMatrix, 1, 1, (float *)XT_M_PARAM); // 初始化参数乘积矩阵。
     arm_mat_init_f32(&deta_PARAMMatrix, 3, 1, (float *)deta_PARAM); // 初始化参数增量矩阵。
     arm_mat_init_f32(&lastParamMatrix, 3, 1, (float *)lastParamVector); // 初始化旧参数矩阵。
     arm_mat_init_f32(&lastTransMatrix, 3, 3, (float *)lastTrans); // 初始化旧协方差矩阵。
     arm_mat_init_f32(&KAIN_M_XTMatrix, 3, 3, (float *)KAIN_M_XT); // 初始化增益乘积矩阵。
     arm_mat_init_f32(&KAIN_M_XT_M_TRANSMatrix, 3, 3, (float *)KAIN_M_XT_M_TRANS); // 初始化协方差更新矩阵。
} // 初始化结束。
extern float powerPredict; // 外部预测功率变量。

float PowerControl_ModelLossPredict(float x1, float x2, float x3, const ChassisPower *power) // 使用模型参数预测损耗项。
{
    if (power == NULL)
    {
        return 0.0f;
    }

    return power->paramVector[0][0] * x1 + power->paramVector[1][0] * x2 + power->paramVector[2][0] * x3;
}

float PowerControl_WheelPowerPredict(float wl, float wr, float tl, float tr, const ChassisPower *power) // 预测轮组总电功率。
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

    x1 = wl * wl + wr * wr;
    x2 = tl * tl + tr * tr;
    x3 = 2.0f; // 两个轮电机的常数项总和。

    loss_power = PowerControl_ModelLossPredict(x1, x2, x3, power);
    mech_power = power->toque_coefficient * (wl * tl + wr * tr);
    total_power = loss_power + mech_power;

    if (total_power < 0.0f)
    {
        total_power = 0.0f;
    }

    return total_power;
}

float PowerControl_AutoUpdateParam(float x1, float x2, float x3, float y,ChassisPower power) // 根据样本更新参数。
{
     if (powerPredict < 0) // 预测值异常时直接返回。
     {
         return PowerControl_ModelLossPredict(Xsample[0][0], Xsample[1][0], Xsample[2][0], &power); // 返回当前模型输出。
     } // 异常分支结束。
// x1 为 电机转速平方和，x2 为电机扭矩平方和，x3 为常数项（偏置项），y 为总功率，effectivePower 为机械功率 // 输入含义说明。
// 若使用 setCurrent，需要保证 x2 与当前电机实际输出一致，需要根据实际情况对当前功率进行修正 // 使用说明。

    Xsample[0][0] = x1; // 电机转速平方和。
    Xsample[1][0] = x2; // 电机扭矩平方和。
    Xsample[2][0] = x3; // 常数项。
    Ysample[0][0] = y;  // 功率观测值。
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
    float deltaPower = PowerControl_ModelLossPredict(Xsample[0][0], Xsample[1][0], Xsample[2][0], &power); // 计算功率估计值。
    return deltaPower; // 返回估计功率。
} // 更新函数结束。
