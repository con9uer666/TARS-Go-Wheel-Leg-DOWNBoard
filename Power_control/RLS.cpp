#include "RLS.h"
#include <cmath>

static float kainVector[3][1] = {0};
static float Xsample[3][1] = {0};
static float Ysample[1][1] = {0};
static float lamdaPowerAutoUpdate = 0.9999f;

static float TRANS_M_X[3][1] = {0};
static float XT[1][3] = {0};
static float XT_M_TRANS[1][3] = {0};
static float XT_M_TRANS_M_X[1][1] = {0};
static float XT_M_PARAM[1][1] = {0};
static float deta_PARAM[3][1] = {0};

static float lastParamVector[3][1] = {0};
static float lastTrans[3][3] = {0};

static float KAIN_M_XT[3][3] = {0};
static float KAIN_M_XT_M_TRANS[3][3] = {0};

static arm_matrix_instance_f32 paramMatrix;
static arm_matrix_instance_f32 transMatrix;
static arm_matrix_instance_f32 kainMatrix;
static arm_matrix_instance_f32 XsampleMatrix;
static arm_matrix_instance_f32 YsampleMatrix;
static arm_matrix_instance_f32 TRANS_M_XMatrix;
static arm_matrix_instance_f32 XTMatrix;
static arm_matrix_instance_f32 XT_M_TRANSMatrix;
static arm_matrix_instance_f32 XT_M_TRANS_M_XMatrix;
static arm_matrix_instance_f32 XT_M_PARAMMatrix;
static arm_matrix_instance_f32 deta_PARAMMatrix;
static arm_matrix_instance_f32 lastParamMatrix;
static arm_matrix_instance_f32 lastTransMatrix;
static arm_matrix_instance_f32 KAIN_M_XTMatrix;
static arm_matrix_instance_f32 KAIN_M_XT_M_TRANSMatrix;

extern "C" float powerPredict;

void PowerControl_AutoUpdateParamInit(ChassisPower* power)
{
    lamdaPowerAutoUpdate = 0.9999f;

    arm_mat_init_f32(&paramMatrix, 3, 1, reinterpret_cast<float*>(power->paramVector));
    arm_mat_init_f32(&transMatrix, 3, 3, reinterpret_cast<float*>(power->transVector));

    arm_mat_init_f32(&kainMatrix, 3, 1, reinterpret_cast<float*>(kainVector));
    arm_mat_init_f32(&XsampleMatrix, 3, 1, reinterpret_cast<float*>(Xsample));
    arm_mat_init_f32(&YsampleMatrix, 1, 1, reinterpret_cast<float*>(Ysample));
    arm_mat_init_f32(&TRANS_M_XMatrix, 3, 1, reinterpret_cast<float*>(TRANS_M_X));
    arm_mat_init_f32(&XTMatrix, 1, 3, reinterpret_cast<float*>(XT));
    arm_mat_init_f32(&XT_M_TRANSMatrix, 1, 3, reinterpret_cast<float*>(XT_M_TRANS));
    arm_mat_init_f32(&XT_M_TRANS_M_XMatrix, 1, 1, reinterpret_cast<float*>(XT_M_TRANS_M_X));
    arm_mat_init_f32(&XT_M_PARAMMatrix, 1, 1, reinterpret_cast<float*>(XT_M_PARAM));
    arm_mat_init_f32(&deta_PARAMMatrix, 3, 1, reinterpret_cast<float*>(deta_PARAM));
    arm_mat_init_f32(&lastParamMatrix, 3, 1, reinterpret_cast<float*>(lastParamVector));
    arm_mat_init_f32(&lastTransMatrix, 3, 3, reinterpret_cast<float*>(lastTrans));
    arm_mat_init_f32(&KAIN_M_XTMatrix, 3, 3, reinterpret_cast<float*>(KAIN_M_XT));
    arm_mat_init_f32(&KAIN_M_XT_M_TRANSMatrix, 3, 3, reinterpret_cast<float*>(KAIN_M_XT_M_TRANS));
}

float PowerControl_ModelLossPredict(float x1, float x2, float x3, const ChassisPower *power)
{
    if (power == nullptr)
        return 0.0f;

    return power->paramVector[0][0] * x1
         + power->paramVector[1][0] * x2
         + power->paramVector[2][0] * x3;
}

float PowerControl_WheelPowerPredict(float wl, float wr, float il, float ir, const ChassisPower *power)
{
    if (power == nullptr)
        return 0.0f;

    float il_limited = il;
    float ir_limited = ir;
    if (il_limited > 16384.0f) il_limited = 16384.0f;
    else if (il_limited < -16384.0f) il_limited = -16384.0f;
    if (ir_limited > 16384.0f) ir_limited = 16384.0f;
    else if (ir_limited < -16384.0f) ir_limited = -16384.0f;

    float x1 = wl * wl + wr * wr;
    float x2 = il_limited * il_limited + ir_limited * ir_limited;
    float x3 = 2.0f;

    float loss_power = PowerControl_ModelLossPredict(x1, x2, x3, power);
    float mech_power = power->toque_coefficient * (wl * il_limited + wr * ir_limited);
    float total_power = loss_power + mech_power;

    if (total_power < 0.0f)
        total_power = 0.0f;

    return total_power;
}

float PowerControl_SingleWheelCurrentFromPower(float w, float power_limit, const ChassisPower *power)
{
    if (power == nullptr)
        return 0.0f;

    float a = power->paramVector[0][0];
    float b = power->paramVector[1][0];
    float c = power->paramVector[2][0];
    float Kt = power->toque_coefficient;

    float A = b;
    float B = Kt * w;
    float C = a * w * w + c - power_limit;

    if (std::fabs(A) < 1e-9f)
    {
        if (std::fabs(B) < 1e-9f)
            return 0.0f;
        float i_root = -C / B;
        if (i_root > 16384.0f) i_root = 16384.0f;
        if (i_root < -16384.0f) i_root = -16384.0f;
        return i_root;
    }

    float delta_val = B * B - 4.0f * A * C;
    if (delta_val < 0.0f)
        return 0.0f;

    float sqrt_delta = std::sqrt(delta_val);
    float i1 = (-B + sqrt_delta) / (2.0f * A);
    float i2 = (-B - sqrt_delta) / (2.0f * A);

    float i_sel;
    if (w > 0.0f)
        i_sel = (i1 > 0.0f) ? i1 : i2;
    else if (w < 0.0f)
        i_sel = (i1 < 0.0f) ? i1 : i2;
    else
        i_sel = (std::fabs(i1) < std::fabs(i2)) ? i1 : i2;

    if (i_sel > 16384.0f) i_sel = 16384.0f;
    if (i_sel < -16384.0f) i_sel = -16384.0f;

    return i_sel;
}

float PowerControl_AutoUpdateParam(float x1, float x2, float x3, float y, ChassisPower power)
{
    if (powerPredict < 0)
        return PowerControl_ModelLossPredict(Xsample[0][0], Xsample[1][0], Xsample[2][0], &power);

    Xsample[0][0] = x1;
    Xsample[1][0] = x2;
    Xsample[2][0] = x3;
    Ysample[0][0] = y;

    if (y > 1)
    {
        arm_mat_trans_f32(&XsampleMatrix, &XTMatrix);
        arm_mat_mult_f32(&transMatrix, &XsampleMatrix, &TRANS_M_XMatrix);
        arm_mat_mult_f32(&XTMatrix, &transMatrix, &XT_M_TRANSMatrix);
        arm_mat_mult_f32(&XT_M_TRANSMatrix, &XsampleMatrix, &XT_M_TRANS_M_XMatrix);
        arm_mat_scale_f32(&TRANS_M_XMatrix,
                          1.0f / (1 + (XT_M_TRANS_M_X[0][0] / lamdaPowerAutoUpdate) / lamdaPowerAutoUpdate),
                          &kainMatrix);

        arm_mat_mult_f32(&XTMatrix, &paramMatrix, &XT_M_PARAMMatrix);
        arm_mat_scale_f32(&kainMatrix, Ysample[0][0] - XT_M_PARAM[0][0], &deta_PARAMMatrix);

        arm_mat_scale_f32(&paramMatrix, 1.0f, &lastParamMatrix);
        arm_mat_add_f32(&lastParamMatrix, &deta_PARAMMatrix, &paramMatrix);

        arm_mat_mult_f32(&kainMatrix, &XTMatrix, &KAIN_M_XTMatrix);
        arm_mat_mult_f32(&KAIN_M_XTMatrix, &transMatrix, &KAIN_M_XT_M_TRANSMatrix);
        arm_mat_scale_f32(&transMatrix, 1.0f, &lastTransMatrix);
        arm_mat_sub_f32(&lastTransMatrix, &KAIN_M_XT_M_TRANSMatrix, &transMatrix);
    }

    return PowerControl_ModelLossPredict(Xsample[0][0], Xsample[1][0], Xsample[2][0], &power);
}
