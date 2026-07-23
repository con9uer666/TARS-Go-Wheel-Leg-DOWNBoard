/**
 * @file self_righting_controller.cpp
 * @brief 倒地自起模式判断与动作状态机实现。
 *
 * 状态机流程：
 *   1) EXTEND: 伸腿到最大腿长，卡住时换角度再伸；
 *   2) REVERSE_TURN: 反向匀速转，尝试两腿并齐；
 *   3) SYNC_HIGH_TORQUE: 大力矩匀速转到目标角度（含差速策略）；
 *   模式退出由姿态恢复消抖判断负责。
 */

#include "self_righting_controller.hpp"

extern "C"
{
#include "Self_Righting.h"
#include <math.h>
#include "Leg_Control.h"
#include "VMC.h"
#include "chassis_behavior_tree.h"
#include "user_pid.h"
#include "Angle_about.h"
#include "Motor_Drv.h"
#include "imu_temp_ctrl.h"
}

#define SELF_RIGHTING_ENTER_ANGLE_DEG 90.0f
#define SELF_RIGHTING_EXIT_ANGLE_DEG  40.0f
#define SELF_RIGHTING_ENTER_TICKS     10U
#define SELF_RIGHTING_EXIT_TICKS      50U

/* ========================= 状态与模式标志 ========================= */

//调试强制入口
//0：程序正常跑		1：强制第一阶段		2：强制第二阶段		3：强制第三阶段		4：强制第四阶段
int SR_test_state = 0;	

//当前自起状态机阶段
SelfRightingStage_t g_self_righting_stage = SELF_RIGHTING_STAGE_EXTEND;

//总开关：置1执行自起，置0时倒地自起不输出力矩
uint8_t g_self_righting_enable = 1;

/* 倒地自起时腿的转动方向标志位
 *   +1: 保持当前行为（左正/右负）
 *   -1: 整体翻向，三个阶段的左右腿目标转速符号一并反过来
 * 默认 +1，对应"车正面（默认方向）倒地"的自起方向。
 * 在 motor.c 检测到反面倒地时把它置 -1。 */
int8_t g_sr_turn_dir = 1;


//标记是否由“未并齐但卡住”触发进入第三阶段
//
//@ 0: 正常并齐后进入第三阶段；
//@ 1: 未并齐但卡住，直接进入第三阶段并启用差速策略。
uint8_t g_self_righting_sync_from_stuck = 0;

static uint8_t sr_mode_active = 0;
static uint8_t sr_enter_count = 0;
static uint8_t sr_exit_count = 0;
static int8_t sr_pending_turn_dir = 1;

//卡腿标志
int turn_stuck_l;//左腿转动卡住标签，1：卡住；0：不卡住
int turn_stuck_r;//右腿转动卡住标签，1：卡住；0：不卡住

//伸腿到位标志
int l0_reached_l;//左腿达到目标腿长标签，1：达到；0：未达到
int l0_reached_r;//右腿达到目标腿长标签，1：达到；0：未达到

//两腿并齐标签，1：并齐；0：未并齐
int aligned;

/* ========================= 参数区 ========================= */

//轮子PID结构体
user_pid_t wheel_PID_l;
user_pid_t wheel_PID_r;

// 防劈叉PID结构体
user_pid_t anti_split_PID;

/* 第一阶段（伸腿）目标与判定参数 */
float g_sr_l0_reached_tol = 0.05f;    /* 到达目标腿长的误差阈值。 */
float g_sr_l0_ctrl_ramp_rate = 0.1f;   /* 伸腿力斜坡速率；0表示不用斜坡。 */
float g_sr_l0_ctrl_f0_max = 30.0f;    /* 伸腿力F0上限。 */
float g_sr_l0_stuck_thresh = 0.01f;   /* 伸腿阶段卡住判定阈值（腿长变化死区）。 */

/* 第一阶段卡住时“换角度再伸腿”参数 */
float g_sr_extend_unstuck_speed_l = 4.0f;      /* 左腿解卡转速指令（算绝对值） */
float g_sr_extend_unstuck_speed_r = 4.0f;     /* 右腿解卡转速指令（算绝对值） */
float g_sr_extend_unstuck_torque_max = 40.0f;   /* 解卡时最大转矩。 */
float g_sr_extend_unstuck_torque_ramp = 0.0f;  /* 解卡转矩斜坡速率。 */

/* 第一阶段未卡住时“被动阻尼”参数（不主动给目标角速度，仅按速度反向阻尼） */
// float g_sr_extend_passive_damping = 0.05f;             /* 阻尼系数，T=-c*w。 */     //TODO: 调参
// float g_sr_extend_passive_damping_torque_max = 10.0f;   /* 被动阻尼转矩限幅。 */

/* 第二阶段（反向匀速转到并齐）参数 */
float g_sr_reverse_speed_l = 0.0f;        /* 左腿反向匀速转速度目标 */
float g_sr_reverse_speed_r = 0.0f;        /* 右腿反向匀速转速度目标 */
float g_sr_reverse_torque_max = 15.0f;      /* 第二阶段转矩上限 */
float g_sr_reverse_torque_ramp = 0.0f;     /* 第二阶段转矩斜坡速率 */
float g_sr_turn_stuck_thresh = 0.0873f;     /* 第二阶段转动卡住判定阈值。 */
float g_sr_align_tol = 0.1745f;              /* 两腿并齐角度误差阈值（|phiL-phiR| <= tol） */

/* 第三阶段（大力矩匀速转到目标角）参数 */
float g_sr_target_angle_l = 2.0 * PI;          /* 左腿目标角度（车身坐标系 phi0）。 */ 
float g_sr_target_angle_r = 2.0 * PI;          /* 右腿目标角度（车身坐标系 phi0）。 */ 
float g_sr_target_angle_tol = 0.0873f;       /* 达到目标角度的误差阈值。 */
float g_sr_sync_speed = 4.0f;              /* 并齐后同步转的基准匀速,绝对值 */	
float g_sr_sync_fast_speed = 0.5f;         /* 未并齐且卡住进入第三阶段时，远端腿速度。 */
float g_sr_sync_slow_speed = 0.1f;         /* 未并齐且卡住进入第三阶段时，近端腿最慢速度。 */
float g_sr_sync_torque_max = 40.0f;        /* 第三阶段大力矩上限。 */
float g_sr_sync_torque_ramp = 0.0f;        /* 第三阶段转矩斜坡速率。 */

/* 调试/观测输出：记录本次单步给到VMC的命令。 */
float g_sr_cmd_f_l = 0.0f;
float g_sr_cmd_t_l = 0.0f;
float g_sr_cmd_f_r = 0.0f;
float g_sr_cmd_t_r = 0.0f;

/** @brief 跨阶段保留的自起六维命令，等价于迁移前持续存在的 VMC_Chassis_Target 字段。 */
static chassis::ChassisCommand sr_command{};

//封装的0-2PI的角度变量
float left_phi0_0_to_2PI;
float right_phi0_0_to_2PI;

float cmd_spd_l;
float cmd_spd_r;

float rem_l;//非负数
float rem_r;//非负数

float f_l = 0.0f;//左伸腿力
float f_r = 0.0f;//右伸腿力
float t_l = 0.0f;//左转矩
float t_r = 0.0f;//右转矩

float l0_err_l;//左腿腿长误差
float l0_err_r;//右腿腿长误差

int l0_stuck_l;
int l0_stuck_r;

float phi_diff;//两腿角度差

/* ========================= PID参数 ========================= */

//轮子PID参数
float wheel_kp = 0.01;
float wheel_ki = 0;
float wheel_kd = 0.01;
float wheel_out_limit = 3;
float wheel_i_limit = 0;
float wheel_I_step = 0;
float wheel_Integraldead_zone = 0;
float wheel_deadzone = 0.01;

//防劈叉PID参数 
float anti_split_kp = 10;
float anti_split_ki = 0;
float anti_split_kd = 0;
float anti_split_out_limit = 3;
float anti_split_i_limit = 0;
float anti_split_I_step = 0;
float anti_split_Integraldead_zone = 0;
float anti_split_deadzone = 0;

//腿角度到位标志
float reached_ang_l = 0;//左腿角度到位标志	1：到位 0：未到位
float reached_ang_r = 0;//右腿角度到位标志	1：到位 0：未到位

/* ========================= 用户调试观测量 ========================= */

float user_a;
float user_b;
float user_c;
float user_d;

/* ========================= 内部辅助函数 ========================= */

/**
 * @brief 限幅
 * 
 * @param value 
 * @param max_mag 最大值，非负数
 * @return float 
 */
static float limit_function(float value, float max_mag)
{
	if (max_mag < 0.0f)
	{
		max_mag = -max_mag;
	}

	if (value > max_mag)
	{
		return max_mag;
	}
	if (value < -max_mag)
	{
		return -max_mag;
	}
	return value;
}


//统一写入两条腿的输出，并更新调试变量，左右腿旋转方向没封装
static void sr_apply_cmd(float f_l, float t_l, float f_r, float t_r)
{
	g_sr_cmd_f_l = f_l;
	g_sr_cmd_t_l = t_l;
	g_sr_cmd_f_r = f_r;
	g_sr_cmd_t_r = t_r;

	sr_command.left_support_force_n = f_l;
	sr_command.left_leg_torque_nm = t_l;
	sr_command.right_support_force_n = f_r;
	sr_command.right_leg_torque_nm = t_r;
}

//封装角度到0-2PI，方便后续判断转动卡住和是否到达目标角度
void update_phi0_0_to_2PI()
{
	left_phi0_0_to_2PI = VMC_L.phi0;
	if (left_phi0_0_to_2PI < 0.0f)
	{
		left_phi0_0_to_2PI = VMC_L.phi0 + 2.0f * PI;
	}

	right_phi0_0_to_2PI = VMC_R.phi0;
	right_phi0_0_to_2PI = PI - VMC_R.phi0;
}

//计算两腿角度差值，得到的是左腿-右腿弧度值，-PI到PI
float update_differ_phi0_0_to_2PI()
{
	return calculate_angle_diff_double_direction(left_phi0_0_to_2PI, right_phi0_0_to_2PI);
}

/* ========================= 对外接口实现 ========================= */

/*
 * 重置状态机：
 * - 回到第一阶段（伸腿）
 * - 清零模式标志
 * - 清零本模块输出命令
 */
static void Self_Righting_Reset(void)
{
    //归零
    g_self_righting_stage = SELF_RIGHTING_STAGE_EXTEND;
	g_self_righting_sync_from_stuck = 0;
    f_l = 0.0f;
    f_r = 0.0f;
    t_l = 0.0f;
    t_r = 0.0f;
    //清零本模块输出命令
	sr_apply_cmd(0.0f, 0.0f, 0.0f, 0.0f);
    sr_command.left_wheel_torque_nm = 0.0f;
    sr_command.right_wheel_torque_nm = 0.0f;
}

/*
 * 单步执行倒地自起控制（无循环，供外部周期调用）。
 *
 * 逻辑对应用户要求：
 * 1) 先伸腿到0.44；
 *    - 伸腿卡住：自动转腿换角度尝试；
 *    - 伸腿不卡住：腿不主动转，仅施加被动阻尼转矩；
 * 2) 到腿长后反方向匀速转，力矩不超过上限；
 *    - 转动卡住：继续转，但不额外增力；
 *    - 两腿并齐后进入下一阶段；
 * 3) 大力矩匀速转到指定角度；
 *    - 若未并齐但卡住，也直接进入本阶段；
 *    - 未并齐路径下使用差速策略，远的快、近的慢，尽量同时到达；
 *    - 哪条腿先到角度就先停，等待另一条腿。
 * 4) 两腿都到位后保留占位分支，用户后续填入逻辑。
 * 		
 *
 * 返回值：0：正在执行自起；2：自起未启用（总开关关了）。
 */
static uint8_t Self_Righting_Step(const chassis::ChassisStateSnapshot& state)
{
	// 每周期重新初始化原轮锁止 PID，并把左右轮原始转速目标设为 0。
	PID_INIT(&wheel_PID_l, wheel_kp, wheel_ki, wheel_kd, wheel_out_limit, wheel_i_limit, wheel_I_step, wheel_Integraldead_zone, wheel_deadzone);
	PID_INIT(&wheel_PID_r, wheel_kp, wheel_ki, wheel_kd, wheel_out_limit, wheel_i_limit, wheel_I_step, wheel_Integraldead_zone, wheel_deadzone);

	PID_Set_Error(&wheel_PID_l, state.wheel.left.raw_motor_speed, 0);
	PID_Set_Error(&wheel_PID_r, state.wheel.right.raw_motor_speed, 0);

	PID_INIT(&anti_split_PID, anti_split_kp, anti_split_ki, anti_split_kd, anti_split_out_limit, anti_split_i_limit, anti_split_I_step, anti_split_Integraldead_zone, anti_split_deadzone);

	// 更新用于并齐误差计算的 0..2π 角度观测量。
	update_phi0_0_to_2PI();

	PID_Set_Error(&anti_split_PID, update_differ_phi0_0_to_2PI(), 0);

	// 总开关关闭时仅清零腿部命令，轮命令保持迁移前的持久字段语义。
	if (g_self_righting_enable == 0U)
	{
		sr_apply_cmd(0.0f, 0.0f, 0.0f, 0.0f);
		return 2;
	}

	// 计算两腿相对最大腿长的误差。
	l0_err_l = LEG_MAX_LENTH - VMC_L.L0;
    l0_err_r = LEG_MAX_LENTH - VMC_R.L0;
    // 分别判断左右腿是否进入目标腿长容差。
	l0_reached_l = (fabsf(l0_err_l) <= g_sr_l0_reached_tol) ? 1 : 0;
	l0_reached_r = (fabsf(l0_err_r) <= g_sr_l0_reached_tol) ? 1 : 0;

    phi_diff = VMC_L.phi0 - VMC_R.phi0;
    // 判断左右虚拟腿角是否已经并齐。
	aligned = (fabsf(phi_diff) <= g_sr_align_tol) ? 1 : 0;

	/* ===================== 第一阶段：伸腿 ===================== */
	if ((g_self_righting_stage == SELF_RIGHTING_STAGE_EXTEND && SR_test_state == 0) || (SR_test_state == 1))
	{


                /*
                 * 伸腿力：两条腿都调用你在Leg_Control里写的腿长控制函数。
                 * 这里是“单步调用”，是否使用斜坡由 g_sr_l0_ctrl_ramp_rate
                 * 决定。   
                 */
        //算伸腿力，是否使用斜坡由
		PID_Set_Error(&L_Leg_L0_POS_PID, VMC_L.L0, LEG_MAX_LENTH);
		PID_Set_Error(&R_Leg_L0_POS_PID, VMC_R.L0, LEG_MAX_LENTH);
		PID_Set_Error(&L_Leg_L0_SPD_PID, VMC_L.d_L0, PID_coculate(&L_Leg_L0_POS_PID));
		PID_Set_Error(&R_Leg_L0_SPD_PID, VMC_R.d_L0, PID_coculate(&R_Leg_L0_POS_PID));

		f_l = PID_coculate(&L_Leg_L0_SPD_PID);
		f_r = PID_coculate(&R_Leg_L0_SPD_PID);
		// f_l = leg_length_control(&VMC_L, LEG_MAX_LENTH, g_sr_l0_ctrl_ramp_rate, g_sr_l0_ctrl_f0_max);
		// f_r = leg_length_control(&VMC_R, LEG_MAX_LENTH, g_sr_l0_ctrl_ramp_rate, g_sr_l0_ctrl_f0_max);

		// 检测恒定伸腿目标下的左右腿长度是否卡住。
		l0_stuck_l = leg_length_stuck_detect(&VMC_L, g_sr_l0_stuck_thresh, 0.2f);
		l0_stuck_r = leg_length_stuck_detect(&VMC_R, g_sr_l0_stuck_thresh, 0.2f);

                /*
                 * 左腿转矩策略：
                 * - 卡住：主动转腿换角度（速度闭环+转矩上限）
                 * - 不卡：不做主动目标转速，仅施加被动阻尼模拟阻尼力
				 * - 算绝对值
                 */

		if (l0_stuck_l && l0_reached_l == 0)//卡住未到位
		{
            // 左腿卡住且未到位时，按配置方向主动转腿解卡。
            t_l = leg_turn_speed_control(&VMC_L, g_sr_turn_dir * g_sr_extend_unstuck_speed_l, g_sr_extend_unstuck_torque_max, g_sr_extend_unstuck_torque_ramp);
		}
		else//不卡
		{
            // 未卡住分支保留旧空操作；被动阻尼代码仍保持禁用。
            // t_l = -g_sr_extend_passive_damping * VMC_L.d_phi0;
			// t_l = limit_function(t_l, g_sr_extend_passive_damping_torque_max);
		}

		/* 右腿同样策略。 */
		if (l0_stuck_r && l0_reached_r == 0)
		{
			t_r = leg_turn_speed_control(&VMC_R, -g_sr_turn_dir * g_sr_extend_unstuck_speed_r, g_sr_extend_unstuck_torque_max, g_sr_extend_unstuck_torque_ramp);
		}
		else
		{
			// t_r = g_sr_extend_passive_damping * VMC_R.d_phi0;
			// t_r = limit_function(t_r, g_sr_extend_passive_damping_torque_max);
		}

		// 两腿均到达最大腿长后进入反向转腿并齐阶段。
		if ((l0_reached_l == 1) && (l0_reached_r == 1))//都到位
		{
			g_self_righting_stage = SELF_RIGHTING_STAGE_REVERSE_TURN;
			g_self_righting_sync_from_stuck = 0;
		}

		l0_reached_l = 0;
		l0_reached_r = 0;
	}
	/* ===================== 第二阶段：反向匀速转并尝试并齐 ===================== */
	else if ((g_self_righting_stage == SELF_RIGHTING_STAGE_REVERSE_TURN && SR_test_state == 0) || (SR_test_state == 2))
	{
		user_a = PID_coculate(&anti_split_PID);
		user_b = g_sr_reverse_speed_l + PID_coculate(&anti_split_PID);


        // 第二阶段继续通过腿长位置/速度双环保持最大腿长。
		PID_Set_Error(&L_Leg_L0_POS_PID, VMC_L.L0, LEG_MAX_LENTH);
		PID_Set_Error(&R_Leg_L0_POS_PID, VMC_R.L0, LEG_MAX_LENTH);
		PID_Set_Error(&L_Leg_L0_SPD_PID, VMC_L.d_L0, PID_coculate(&L_Leg_L0_POS_PID));
		PID_Set_Error(&R_Leg_L0_SPD_PID, VMC_R.d_L0, PID_coculate(&R_Leg_L0_POS_PID));

		f_l = PID_coculate(&L_Leg_L0_SPD_PID);
		f_r = PID_coculate(&R_Leg_L0_SPD_PID);

		// 第二阶段按配置方向匀速转腿，并叠加防劈叉 PID 输出。
		t_l = leg_turn_speed_control(&VMC_L,   g_sr_turn_dir * g_sr_reverse_speed_l + PID_coculate(&anti_split_PID), g_sr_reverse_torque_max, g_sr_reverse_torque_ramp);
		t_r = leg_turn_speed_control(&VMC_R, -(g_sr_turn_dir * g_sr_reverse_speed_r - PID_coculate(&anti_split_PID)), g_sr_reverse_torque_max, g_sr_reverse_torque_ramp);

		// 检查并齐和双腿卡住状态，决定第三阶段是否启用差速来源标志。
		aligned = (fabsf(VMC_L.phi0 - VMC_R.phi0) <= g_sr_align_tol) ? 1 : 0;
		turn_stuck_l = leg_turn_stuck_detect(&VMC_L, g_sr_turn_stuck_thresh, 0.1f);
		turn_stuck_r = leg_turn_stuck_detect(&VMC_R, g_sr_turn_stuck_thresh, 0.1f);

		if (aligned)//并齐
		{
			// 已并齐：正常进入第三阶段。
			g_self_righting_stage = SELF_RIGHTING_STAGE_SYNC_HIGH_TORQUE;
			g_self_righting_sync_from_stuck = 0;
		}
		else if ((turn_stuck_l != 0) && (turn_stuck_r != 0))//未并齐但卡住
		{
			// 未并齐但双腿卡住：记录来源后进入第三阶段。
			g_self_righting_stage = SELF_RIGHTING_STAGE_SYNC_HIGH_TORQUE;
			g_self_righting_sync_from_stuck = 1;
		}
	}
	/* ===================== 第三阶段：大力矩匀速反向转，直到外层判定姿态恢复 ===================== */
	else if ((g_self_righting_stage == SELF_RIGHTING_STAGE_SYNC_HIGH_TORQUE && SR_test_state == 0) || (SR_test_state == 3))
	{
		sr_command.left_wheel_torque_nm = -PID_coculate(&wheel_PID_l);
		sr_command.right_wheel_torque_nm = PID_coculate(&wheel_PID_r);

		//第三阶段继续保持腿长
		PID_Set_Error(&L_Leg_L0_POS_PID, VMC_L.L0, LEG_MAX_LENTH);
		PID_Set_Error(&R_Leg_L0_POS_PID, VMC_R.L0, LEG_MAX_LENTH);
		PID_Set_Error(&L_Leg_L0_SPD_PID, VMC_L.d_L0, PID_coculate(&L_Leg_L0_POS_PID));
		PID_Set_Error(&R_Leg_L0_SPD_PID, VMC_R.d_L0, PID_coculate(&R_Leg_L0_POS_PID));

		f_l = PID_coculate(&L_Leg_L0_SPD_PID);
		f_r = PID_coculate(&R_Leg_L0_SPD_PID);

		//大力矩匀速反向转 + 防劈叉，不再判断"到目标角"，持续转到外层 motor.c 检测到姿态恢复
		float cmd_spd = fabsf(g_sr_sync_speed);
		t_l = leg_turn_speed_control(&VMC_L,   g_sr_turn_dir * cmd_spd + PID_coculate(&anti_split_PID), g_sr_sync_torque_max, g_sr_sync_torque_ramp);
		t_r = leg_turn_speed_control(&VMC_R, -(g_sr_turn_dir * cmd_spd - PID_coculate(&anti_split_PID)), g_sr_sync_torque_max, g_sr_sync_torque_ramp);
	}

	// 更新持久的腿部命令字段；轮命令仅由重置或第三阶段更新。
    sr_apply_cmd(f_l, t_l, f_r, t_r);

	turn_stuck_l = 0;
	turn_stuck_r = 0;

	aligned = 0;
        
	reached_ang_r = 0;
	reached_ang_l = 0;

    return 0;
}

static uint8_t Self_Righting_Mode_Detect(const chassis::ChassisStateSnapshot& state)
{
    if (sr_mode_active == 0U)
    {
        sr_exit_count = 0;

        if (fabsf(state.roll_angle_deg) >= SELF_RIGHTING_ENTER_ANGLE_DEG ||
            fabsf(state.pitch_angle_deg) >= SELF_RIGHTING_ENTER_ANGLE_DEG)
        {
            if (sr_enter_count == 0U)
            {
                sr_pending_turn_dir =
                    (state.pitch_angle_deg > SELF_RIGHTING_ENTER_ANGLE_DEG ||
                     state.pitch_angle_deg < -SELF_RIGHTING_ENTER_ANGLE_DEG) ? 1 : -1;
            }

            if (sr_enter_count < SELF_RIGHTING_ENTER_TICKS)
            {
                sr_enter_count++;
            }

            if (sr_enter_count >= SELF_RIGHTING_ENTER_TICKS)
            {
                Self_Righting_Reset();
                g_sr_turn_dir = sr_pending_turn_dir;
                sr_mode_active = 1;
                sr_enter_count = 0;
            }
        }
        else
        {
            sr_enter_count = 0;
        }

        return sr_mode_active;
    }

    sr_enter_count = 0;

    if (fabsf(state.roll_angle_deg) < SELF_RIGHTING_EXIT_ANGLE_DEG &&
        fabsf(state.pitch_angle_deg) < SELF_RIGHTING_EXIT_ANGLE_DEG)
    {
        if (sr_exit_count < SELF_RIGHTING_EXIT_TICKS)
        {
            sr_exit_count++;
        }

        if (sr_exit_count >= SELF_RIGHTING_EXIT_TICKS)
        {
            sr_mode_active = 0;
            sr_exit_count = 0;
            Self_Righting_Reset();
        }
    }
    else
    {
        sr_exit_count = 0;
    }

    return sr_mode_active;
}

/**
 * @brief 更新倒地进入/退出判定，并在激活时执行一次三阶段自起动作。
 * @param[in] state 本周期姿态、腿部反馈和原始轮电机速度快照。
 * @return 自起接管标志以及保持原阶段间字段语义的六维命令。
 */
chassis::SelfRightingUpdateResult chassis::SelfRightingController::Update(
    const ChassisStateSnapshot& state)
{
    /** 返回给 StartupRetractController 的本周期自起结果。 */
    SelfRightingUpdateResult result{};
    if (Self_Righting_Mode_Detect(state) == 0U)
        return result;

    (void)Self_Righting_Step(state);
    result.command = sr_command;
    result.active = true;
    return result;
}
