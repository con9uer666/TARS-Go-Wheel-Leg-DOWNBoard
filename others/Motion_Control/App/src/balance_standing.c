/**
 * @file balance_standing.c
 * @brief 站起 / 正常运行动作组 Standing()：
 *        倾覆保护 → INS/VMC/车速解算 → yaw误差(常态/小陀螺) → 距离误差 →
 *        横滚补偿 + 腿长PID + 防劈叉 → LQR_Update_K + 功率门控 + LQR_calculate →
 *        小陀螺 phi0 归中叠加 → 跳跃/常态 VMC 下发(Jump_Motion_Update) →
 *        离地检测 + 磕台阶检测 → 上台阶/坐地模式切换。
 */

#include "chassis_behavior_tree.h"
#include "user_pid.h"
#include "Motor_Drv.h"
#include "Gimbal.h"
#include "User_State.h"
#include "State.h"
#include "arm_math.h"
#include "USER_CAN.h"
#include "VMC.h"
#include "observe_task.h"
#include "Leg_Control.h"
#include "Self_Righting.h"
#include "Board2Board.h"
#include "Slope.h"
#include "Wheel_Leg_about.h"
#include "controller.h"
#include "remoter.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "fdcan.h"
#include <math.h>
#include <stdint.h>
#include "imu_temp_ctrl.h"
#include "Angle_about.h"
#include "PowerCtrl.h"
#include "Gas_Spring.h"
#include "buzzer.h"
#include "Wheel_End_Velocity.h"

float last_yaw_error;

//站起
void Standing()
{
//占用率检测用的，留着吧，看不懂也不影响
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, 1);

    /* 倾覆保护：站立期间 |pitch| > 45° 连续 10 帧才触发，避免 IMU 抖动误判。
       触发后用位速双环PID把腿长压到最短(LEG_MIN_LENTH)、腿角拉到PI/2.0f(竖直)，
       不绕长路、不区分子状态，维持0.5s后回退到未站起状态
       （start_mode=0, upstares_mode=0, first_run=1）。
       Motor_task 周期 2ms，0.5s = 250 tick，10 帧滤波 = 20ms。 */
    static uint16_t tip_protect_cnt = 0;
    static uint8_t  tip_detect_cnt  = 0;
    if (tip_protect_cnt == 0)
    {
        if (pitch > 45.0f || pitch < -45.0f)
        {
            if (tip_detect_cnt < 10) tip_detect_cnt++;
            if (tip_detect_cnt >= 10)
            {
                tip_protect_cnt = 250;
                tip_detect_cnt  = 0;
            }
        }
        else
        {
            tip_detect_cnt = 0;
        }
    }
    if (tip_protect_cnt > 0)
    {
        VMC_Coculate();

        // 腿长位速双环：目标=最短腿长
        PID_Set_Error(&L_Leg_L0_POS_PID, VMC_L.L0, LEG_MIN_LENTH);
        PID_Set_Error(&R_Leg_L0_POS_PID, VMC_R.L0, LEG_MIN_LENTH);
        PID_coculate(&L_Leg_L0_POS_PID);
        PID_coculate(&R_Leg_L0_POS_PID);
        PID_Set_Error(&L_Leg_L0_SPD_PID, VMC_L.d_L0, L_Leg_L0_POS_PID.output);
        PID_Set_Error(&R_Leg_L0_SPD_PID, VMC_R.d_L0, R_Leg_L0_POS_PID.output);
        PID_coculate(&L_Leg_L0_SPD_PID);
        PID_coculate(&R_Leg_L0_SPD_PID);

        // 腿角双环：目标=PI/2.0f（不绕长路，直接走短路径）
        PID_Set_AngleError(&L_Leg_Middle_PID, VMC_L.phi0, PI / 2.0f);
        PID_coculate(&L_Leg_Middle_PID);
        PID_Set_Error(&L_Leg_dphi0_PID, VMC_L.d_phi0, L_Leg_Middle_PID.output);
        PID_coculate(&L_Leg_dphi0_PID);

        PID_Set_AngleError(&R_Leg_Middle_PID, VMC_R.phi0, PI / 2.0f);
        PID_coculate(&R_Leg_Middle_PID);
        PID_Set_Error(&R_Leg_dphi0_PID, -VMC_R.d_phi0, -R_Leg_Middle_PID.output);
        PID_coculate(&R_Leg_dphi0_PID);

        VMC_Set_F0_T(&VMC_L, L_Leg_L0_SPD_PID.output,  L_Leg_dphi0_PID.output);
        VMC_Set_F0_T(&VMC_R, R_Leg_L0_SPD_PID.output, -R_Leg_dphi0_PID.output);

        L_DJ3508.Target_Torque = 0;
        R_DJ3508.Target_Torque = 0;

        tip_protect_cnt--;
        if (tip_protect_cnt == 0)
        {
            start_mode      = 0;
            upstares_mode   = 0;
            first_run       = 1;
            sit_first_entry = 1;
        }
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, 0);
        return;
    }

    //惯性导航、VMC、水平方向车身速度解算vscode://lirentech.file-ref-tags?filePath=motor.c&snippet=%2F%2F%E6%83%AF%E6%80%A7%E5%AF%BC%E8%88%AA%E3%80%81VMC%E3%80%81%E6%B0%B4%E5%B9%B3%E6%96%B9%E5%90%91%E8%BD%A6%E8%BA%AB%E9%80%9F%E5%BA%A6%E8%A7%A3%E7%AE%97
    INS_Coculate();
    VMC_Coculate();
    Body_Speed_Coculate();

    //算yaw的误差，以及根据yaw误差调整目标速度
    if(gimbal_follow_flag == 1)
    {
        yaw_error = 0;//云台跟随底盘时，强制yaw误差为0，让底盘完全跟随云台
        Speed_Error_Set();
    }

    if(gimbal_follow_flag == 0)
    {
        //算小陀螺的
        if(Foot_Chassis.Chassis_Mode == 1 && spinning_usable == 1)
        {
            spinning_up();
            spinning_flag = 1;
        }
        else//退出小陀螺 or 普通运行
        {
            if(spinning_flag == 1)//小陀螺退出（减速+归位统一）
            {
                spinning_usable = 0;

                spinning_exit();

                if(fabsf(d_yaw) <= 4.0f && fabsf(yaw_angle_PI) <= 0.5f)
                {
                    spinning_flag = 0;
                }
            }
            else if(spinning_flag == 0)//常态
            {
                spinning_pid.I = 0;
                spinning_target_d_yaw_cmd = d_yaw;
                spinning_d_yaw_feedback = d_yaw;
                spinning_usable = 1;
                Yaw_Error_Coculate();
            }
        }
    }

    yaw_error = 0.05 * yaw_error + 0.95 * last_yaw_error;
    last_yaw_error = yaw_error;

    //计算距离误差
    Distance_Error_Set();
    if (spinning_flag == 1) { body_distance_error = 0; body_distance = 0; target_body_distance = 0; }

    //小陀螺时轮速共模P反馈：抑制因左右轮共模偏置导致的车身漂移
    if (spinning_flag == 1) {
        static float wheel_common_f = 0.0f;
        float wheel_common = (R_DJ3508.Rx_Data.Velocity - L_DJ3508.Rx_Data.Velocity) * 0.0305f;
        wheel_common_f = 0.05f * wheel_common + 0.95f * wheel_common_f;
        speed_error -= wheel_common_f * 0.1f;
    }

    //横滚补偿和PD单环腿长控制
    Roll_Comp();
    Leg_L0_Control();

    //防劈叉：Kp/Kd按L0_avg做1D二次拟合(类似LQR风格)，单PID输出
    float L0_avg = (VMC_L.L0 + VMC_R.L0) * 0.5f;
    AntiSplit_Get_K(AntiSplit_K, AntiSplit_K_Fit_Coefficients, L0_avg);
    Leg_AntiSplit_PID.Kp = AntiSplit_K[0];
    Leg_AntiSplit_PID.Kd = AntiSplit_K[1];
    PID_Set_Error(&Leg_AntiSplit_PID, (VMC_R.phi0 - PI/2) + (VMC_L.phi0 - PI/2), 0);
    PID_coculate(&Leg_AntiSplit_PID);
    float anti_split_out = Leg_AntiSplit_PID.output;

    //100hz算K值，毕竟K值的计算比较耗时
    LQR_Update_K();

	// 更新功率门控缩放系数（不直接限制扭矩，只作用于观测量）。
	PowerCtrl();

    LQR_calculate();

    //常态下VMC解算，加入PID前馈
    float centrifugal_comp = centrifugal_comp_gain * d_yaw * d_yaw;
    float L_leg_T_cmd = Leg_L_T + anti_split_out - centrifugal_comp;
    float R_leg_T_cmd = -Leg_R_T + anti_split_out - centrifugal_comp;
    static uint8_t spin_phi0_pid_started = 0;
    if(spinning_flag == 1)
    {
        PID_Set_AngleError(&L_Spin_Phi0_PID, VMC_L.phi0, target_spin_phi0);
        PID_Set_AngleError(&R_Spin_Phi0_PID, VMC_R.phi0, target_spin_phi0);
        if(spin_phi0_pid_started == 0)
        {
            L_Spin_Phi0_PID.pre_error = L_Spin_Phi0_PID.error;
            R_Spin_Phi0_PID.pre_error = R_Spin_Phi0_PID.error;
            spin_phi0_pid_started = 1;
        }
        L_leg_T_cmd += PID_coculate(&L_Spin_Phi0_PID);
        R_leg_T_cmd += PID_coculate(&R_Spin_Phi0_PID);
    }
    else
    {
        PID_Clear(&L_Spin_Phi0_PID);
        PID_Clear(&R_Spin_Phi0_PID);
        L_Spin_Phi0_PID.output = 0.0f;
        R_Spin_Phi0_PID.output = 0.0f;
        spin_phi0_pid_started = 0;
    }

    // 跳跃/常态 VMC 力矩下发（跳跃锁存、LR平衡、跳跃蜂鸣器统一在 jump_motion 内处理）
    Jump_Motion_Update(L_leg_T_cmd, R_leg_T_cmd);

    off_ground_detect();
    Step_Hit_Detect();
    // 注：跳跃上锁统一在 jump_motion 锁存退出时处理（离地 / 0.5s 到期），此处不再需要兜底

    if(upstairs_flag == 1)
    {
        start_mode = 2;
        upstairs_flag = 0;    // 已消费触发信号，避免动作完成后残留导致距离闭环被永久压0
    }

    if(sit_mode_enable == 1)
    {
        start_mode = 3;
    }

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, 0);
}
