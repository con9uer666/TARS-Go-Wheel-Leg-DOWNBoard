 #include "RLS.h"

 float kainVector[3][1] = {0};
 float Xsample[3][1] = {0};
 float Ysample[1][1] = {0};
 float lamdaPowerAutoUpdate = 0.9999;

 float TRANS_M_X[3][1] = {0};
 float XT[1][3] = {0};
 float XT_M_TRANS[1][3] = {0};
 float XT_M_TRANS_M_X[1][1] = {0};
 float XT_M_PARAM[1][1] = {0};
 float deta_PARAM[3][1] = {0};

 float lastParamVector[3][1] = {0};
 float lastTrans[3][3] = {0};

 float KAIN_M_XT[3][3] = {0};
 float KAIN_M_XT_M_TRANS[3][3] = {0};

 arm_matrix_instance_f32 paramMatrix;
 arm_matrix_instance_f32 transMatrix;
 arm_matrix_instance_f32 kainMatrix;
 arm_matrix_instance_f32 XsampleMatrix;
 arm_matrix_instance_f32 YsampleMatrix;
 arm_matrix_instance_f32 TRANS_M_XMatrix;
 arm_matrix_instance_f32 XTMatrix;
 arm_matrix_instance_f32 XT_M_TRANSMatrix;
 arm_matrix_instance_f32 XT_M_TRANS_M_XMatrix;
 arm_matrix_instance_f32 XT_M_PARAMMatrix;
 arm_matrix_instance_f32 deta_PARAMMatrix;
 arm_matrix_instance_f32 lastParamMatrix;
 arm_matrix_instance_f32 lastTransMatrix;
 arm_matrix_instance_f32 KAIN_M_XTMatrix;
 arm_matrix_instance_f32 KAIN_M_XT_M_TRANSMatrix;

 void PowerControl_AutoUpdateParamInit(ChassisPower* power)
{
     lamdaPowerAutoUpdate = 0.9999f;
     arm_mat_init_f32(&paramMatrix, 3, 1, (float *)power->paramVector);
     arm_mat_init_f32(&transMatrix, 3, 3, (float *)power->transVector);
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
 }
float powerPredict = 0;

 float PowerControl_AutoUpdateParam(float x1, float x2, float x3, float y,ChassisPower power)
{
     if (powerPredict < 0)
     {
         return power.paramVector[0][0] * Xsample[0][0] + power.paramVector[1][0] * Xsample[1][0] + power.paramVector[2][0] * Xsample[2][0];
     }
    // x1? ??????????? x2????????????? x3 ???????????????????? y??????????  effctivePower???§Ö????
    // ??????setCurrent ???x2??????????????? ???????????????????¨´???
    Xsample[0][0] = x1; // ???????????
    Xsample[1][0] = x2; // ????????????
    Xsample[2][0] = x3; // ??????
    Ysample[0][0] = y;  // detaP
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
    float deltaPower = power.paramVector[0][0] * Xsample[0][0] + power.paramVector[1][0] * Xsample[1][0] + power.paramVector[2][0] * Xsample[2][0]; // ??????????
    return deltaPower;
}
