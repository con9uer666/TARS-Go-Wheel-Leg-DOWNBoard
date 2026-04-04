#include "Wheel_Leg_about.h"
#include "imu_temp_ctrl.h"
#include "user_pid.h"
#include "motor.h"
#include "remoter.h"
#include "VMC.h"
#include "observe_task.h"
#include "Variable.h"

/*============================ 轮腿相关算法 ================================*/

/**
 * @brief K矩阵融合
 * 
 */
void LQR_Get_K(float LQR[4][10], float K_Fit_Coefficients[40][6], float L0_l, float L0_r)
{
    for(uint8_t i = 0; i < 4; i++) 
    {
        for(uint8_t j = 0; j < 10; j++) 
        {
            uint8_t pos = i * 10 + j;
            
            float p00 = K_Fit_Coefficients[pos][0];
            float p10 = K_Fit_Coefficients[pos][1];
            float p01 = K_Fit_Coefficients[pos][2];
            float p20 = K_Fit_Coefficients[pos][3];
            float p11 = K_Fit_Coefficients[pos][4];
            float p02 = K_Fit_Coefficients[pos][5];
            
            LQR[i][j] = p00
                      + p10 * L0_l
                      + p01 * L0_r
                      + p20 * L0_l * L0_l
                      + p11 * L0_l * L0_r
                      + p02 * L0_r * L0_r;
        }
    }
}


/**
 * @brief 横滚补偿
 * 
 */
void Roll_Comp()
{
    if(speed_error <= 0.3 && speed_error >= -0.3)
    target_roll = alpha_target_roll * (-((SBUS_CH.CH1 - 992.0f)/800.0f) * 12.0f) + (1 - alpha_target_roll) * target_roll;
    else
    target_roll = alpha_target_roll * target_roll + (1 - alpha_target_roll) * target_roll;

    PID_Set_Error(&Roll_Comp_PID, roll, target_roll + 2);
    PID_coculate(&Roll_Comp_PID);
}

//pd单环腿长控制函数
void Leg_L0_Control()
{
    if(leg_state_count > 0)
    {
        leg_state_count --;
    }

    //低通滤波
    target_Leg_L0 = alpha_target_L0 * (((Foot_Chassis.Target_Leg_State / 1.0f) * 0.22) + LEG_MIN_LENTH) + (1 - alpha_target_L0) * target_Leg_L0;                       
    target_Leg_L0 = alpha_target_L0 * (((Foot_Chassis.Target_Leg_State / 1.0f) * 0.22) + LEG_MIN_LENTH) + (1 - alpha_target_L0) * target_Leg_L0;                       

    if(target_Leg_L0 >= 0.40f)          
    target_Leg_L0 = 0.40f;              
    if(target_Leg_L0 <= LEG_MIN_LENTH)  
    target_Leg_L0 = LEG_MIN_LENTH;      

    target_L_Leg_L0 = target_Leg_L0;    
    target_R_Leg_L0 = target_Leg_L0;    

    if(target_L_Leg_L0 >= 0.40)         
    target_L_Leg_L0 = 0.40;
    if(target_L_Leg_L0 <= LEG_MIN_LENTH)
    target_L_Leg_L0 = LEG_MIN_LENTH;

    if(target_R_Leg_L0 >= 0.40)         
    target_R_Leg_L0 = 0.40;             
    if(target_R_Leg_L0 <= LEG_MIN_LENTH)
    target_R_Leg_L0 = LEG_MIN_LENTH;    

    PID_Set_Error(&L_Leg_L0_PID, VMC_L.L0, target_L_Leg_L0);                       
    PID_Set_Error(&R_Leg_L0_PID, VMC_R.L0, target_R_Leg_L0);                       

    PID_coculate(&L_Leg_L0_PID);                       
    PID_coculate(&R_Leg_L0_PID);                       
}

//speed_error | 计算前进速度误差 (yaw_error)
void Speed_Error_Set()
{
    speed_limit = 2.2f;     //车子最大速
    // rampInit(&Target_Speed_Ramp, target_body_speed, (((SBUS_CH.CH3 - 992.0f)/800.0f) * speed_limit), 0.3f, 0.002f);
    // rampIterate(&Target_Speed_Ramp);
    target_body_speed = Foot_Chassis.Target_Vy;

    if(target_body_speed >= speed_limit)
    target_body_speed = speed_limit;
    else if(target_body_speed <= -speed_limit)
    target_body_speed = -speed_limit;
    
    float temp = ((0.7f - fabsf(yaw_error))/0.7f);//速度和转速功率分配倍率为0.7
    if(temp < 0.0)
    temp = 0.0;

    target_body_speed = target_body_speed * temp;//yaw误差越大，目标速度越小，最大为100%，最小为0
    //// target_body_speed = alpha_target_body_speed * (((SBUS_CH.CH2 - 992.0f)/800.0f) * speed_limit) + (1 - alpha_target_body_speed) * target_body_speed;
    speed_error = target_body_speed - kalman_body_speed;

    if(speed_error >= speed_limit * 0.7f)
    speed_error = speed_limit * 0.7f;
    if(speed_error <= -speed_limit * 0.7f)
    speed_error = -speed_limit * 0.7f;
}

/**
 * @brief body_distance, target_body_distance, body_distance_error | 计算距离误差(kalman_body_speed, speed_error)，并且更新和
 * 
 */
void Distance_Error_Set()
{
    body_distance += kalman_body_speed * 0.002f;
    target_body_distance += (kalman_body_speed + speed_error) * 0.002f; //! kalman_body_speed + speed_error不等于target_body_speed，因为speed_error是经过限幅的，而target_body_speed是没有经过限幅的，始作俑者：25年丛庆
    body_distance_error = target_body_distance - body_distance;
}

float alpha_W = 0.9;//滤波系数
float body_speed_L, body_speed_R, body_speed; //当前车体速度 ,已正交，是水平方向的速度
/**
 * body_speed | 水平方向车身速度解算(VMC, INS.pitch_trans)
 */
void Body_Speed_Coculate()
{
    //算单侧轮子速度
    Wl = alpha_W * (-L_LK9025.Rx_Data.Velocity + VMC_L.d_b_phi0) + (1 - alpha_W) * Wl;
    //算单侧车身速度
    body_speed_L = alpha_body_speed * ((Wl * WHEEL_RADIUS) + VMC_L.d_b_phi0 * VMC_L.L0 * arm_cos_f32(VMC_L.b_phi0)) + (1 - alpha_body_speed) * body_speed_L;

    Wr = alpha_W * (R_LK9025.Rx_Data.Velocity + VMC_R.d_b_phi0) + (1 - alpha_W) * Wr;
    body_speed_R = alpha_body_speed * ((Wr * WHEEL_RADIUS) + VMC_R.d_b_phi0 * VMC_R.L0 * arm_cos_f32(VMC_R.b_phi0)) + (1 - alpha_body_speed) * body_speed_R;

    body_speed = (body_speed_L + body_speed_R) / 2.0f;
}

//! 这个函数的命名也太随意了吧，丛庆
//! PS:这个注释不是我写的，是你自己的ai备注的hhhhhhhh甚至知道你叫丛庆
 

float alpha_d_pitch = 1.0;//滤波系数

//惯性导航系统数据处理，算出pitch_trans/yaw_trans/d_pitch/d_yaw
void INS_Coculate()
{
    task_Pitch_Coculate();

    yaw_trans[1] = yaw_trans[0];
    yaw_trans[0] = (yaw/180.0f) * PI;
    d_pitch = alpha_d_pitch * ((pitch_trans[0] - pitch_trans[1])/0.002f) + (1 - alpha_d_pitch) * d_pitch;
    float temp = yaw_trans[0] - yaw_trans[1];//临时变量
    if (temp > PI) 
        temp -= 2 * PI;
    else if (temp < -PI)
        temp += 2 * PI;
    d_yaw = alpha_d_yaw * (temp/0.002f) + (1 - alpha_d_yaw) * d_yaw;
}

