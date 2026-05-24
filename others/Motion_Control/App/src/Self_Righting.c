#include "Self_Righting.h"
#include <math.h>
#include "Leg_Control.h"
#include "VMC.h"
#include "motor.h"
#include "user_pid.h"
#include "Angle_about.h"
#include "Motor_Drv.h"
#include "buzzer.h"

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


/*
 * 按当前自起阶段驱动蜂鸣器：每阶段不同音高 + 不同蜂鸣节奏，满音量。
 * 调用频率与 Self_Righting_Step 一致（500 Hz / 2 ms 一拍）。
 * Stage 1 EXTEND            : do  523Hz, 500ms on / 500ms off  → 1Hz
 * Stage 2 REVERSE_TURN      : sol 784Hz, 125ms on / 125ms off  → 4Hz
 * Stage 3 SYNC_HIGH_TORQUE  : 高mi 1318Hz, 50ms on / 50ms off  → 10Hz
 * Stage 4 FINISHED          : 这里不响，离开自起态后由外层放100ms完成音
 */
static uint32_t sr_buzz_tick = 0;
static void sr_buzzer_update(SelfRightingStage_t stage)
{
    int pitch = 0;
    uint32_t on_ticks = 0;
    uint32_t cycle_ticks = 0;
    switch (stage)
    {
        case SELF_RIGHTING_STAGE_EXTEND:
            pitch = 523;  on_ticks = 250; cycle_ticks = 500; break;
        case SELF_RIGHTING_STAGE_REVERSE_TURN:
            pitch = 784;  on_ticks = 63;  cycle_ticks = 125; break;
        case SELF_RIGHTING_STAGE_SYNC_HIGH_TORQUE:
            pitch = 1318; on_ticks = 25;  cycle_ticks = 50;  break;
        default:
            Stop_Buzzer();
            sr_buzz_tick = 0;
            return;
    }
    if ((sr_buzz_tick % cycle_ticks) < on_ticks) Buzzer_Tone_Max(pitch);
    else                                         Stop_Buzzer();
    sr_buzz_tick++;
}

//统一写入两条腿的输出，并更新调试变量，左右腿旋转方向没封装
static void sr_apply_cmd(float f_l, float t_l, float f_r, float t_r)
{
	g_sr_cmd_f_l = f_l;
	g_sr_cmd_t_l = t_l;
	g_sr_cmd_f_r = f_r;
	g_sr_cmd_t_r = t_r;

	VMC_Set_F0_T(&VMC_L, f_l, t_l);
	VMC_Set_F0_T(&VMC_R, f_r, t_r);
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
void Self_Righting_Reset(void)
{
    //归零
    g_self_righting_stage = SELF_RIGHTING_STAGE_EXTEND;
	g_self_righting_sync_from_stuck = 0;
    //清零本模块输出命令
	sr_apply_cmd(0.0f, 0.0f, 0.0f, 0.0f);
    //蜂鸣器复位
    sr_buzz_tick = 0;
    Stop_Buzzer();
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
 * 返回值：0：正在执行自起；1：自起完成（两腿都到目标角度了）；2：自起未启用（总开关关了）。
 */
uint8_t Self_Righting_Step(void)
{
	//锁死轮子vscode://lirentech.file-ref-tags?filePath=Self_Righting.c&snippet=%2F%2F%E9%94%81%E6%AD%BB%E8%BD%AE%E5%AD%90
	PID_INIT(&wheel_PID_l, wheel_kp, wheel_ki, wheel_kd, wheel_out_limit, wheel_i_limit, wheel_I_step, wheel_Integraldead_zone, wheel_deadzone);
	PID_INIT(&wheel_PID_r, wheel_kp, wheel_ki, wheel_kd, wheel_out_limit, wheel_i_limit, wheel_I_step, wheel_Integraldead_zone, wheel_deadzone);

	PID_Set_Error(&wheel_PID_l, L_DJ3508.Rx_Data.Speed, 0);
	PID_Set_Error(&wheel_PID_r, R_DJ3508.Rx_Data.Speed, 0);

	PID_INIT(&anti_split_PID, anti_split_kp, anti_split_ki, anti_split_kd, anti_split_out_limit, anti_split_i_limit, anti_split_I_step, anti_split_Integraldead_zone, anti_split_deadzone);

	//封装的0-2PI的角度变量，方便后续判断转动卡住和是否到达目标角度vscode://lirentech.file-ref-tags?filePath=Self_Righting.c&snippet=%2F%2F%E5%B0%81%E8%A3%85%E7%9A%840-2PI%E7%9A%84%E8%A7%92%E5%BA%A6%E5%8F%98%E9%87%8F%EF%BC%8C%E6%96%B9%E4%BE%BF%E5%90%8E%E7%BB%AD%E5%88%A4%E6%96%AD%E8%BD%AC%E5%8A%A8%E5%8D%A1%E4%BD%8F%E5%92%8C%E6%98%AF%E5%90%A6%E5%88%B0%E8%BE%BE%E7%9B%AE%E6%A0%87%E8%A7%92%E5%BA%A6
	update_phi0_0_to_2PI();

	PID_Set_Error(&anti_split_PID, update_differ_phi0_0_to_2PI(), 0);

	//函数锁vscode://lirentech.file-ref-tags?filePath=Self_Righting.c&snippet=%2F%2F%E5%87%BD%E6%95%B0%E9%94%81
	if (g_self_righting_enable == 0U)
	{
		sr_apply_cmd(0.0f, 0.0f, 0.0f, 0.0f);
		Stop_Buzzer();
		sr_buzz_tick = 0;
		return 2;
	}

	//按当前阶段驱动蜂鸣器（含调试强制阶段 SR_test_state）
	{
		SelfRightingStage_t effective_stage;
		if (SR_test_state >= 1 && SR_test_state <= 4)
			effective_stage = (SelfRightingStage_t)(SR_test_state - 1);
		else
			effective_stage = g_self_righting_stage;
		sr_buzzer_update(effective_stage);
	}

	//算常用误差量vscode://lirentech.file-ref-tags?filePath=Self_Righting.c&snippet=%2F%2F%E7%AE%97%E5%B8%B8%E7%94%A8%E8%AF%AF%E5%B7%AE%E9%87%8F
	l0_err_l = LEG_MAX_LENTH - VMC_L.L0;
    l0_err_r = LEG_MAX_LENTH - VMC_R.L0;
    //达到目标腿长检测vscode://lirentech.file-ref-tags?filePath=Self_Righting.c&snippet=%2F%2F%E8%BE%BE%E5%88%B0%E7%9B%AE%E6%A0%87%E8%85%BF%E9%95%BF%E6%A3%80%E6%B5%8B
	l0_reached_l = (fabsf(l0_err_l) <= g_sr_l0_reached_tol) ? 1 : 0;
	l0_reached_r = (fabsf(l0_err_r) <= g_sr_l0_reached_tol) ? 1 : 0;

    phi_diff = VMC_L.phi0 - VMC_R.phi0;
    //并齐检测vscode://lirentech.file-ref-tags?filePath=Self_Righting.c&snippet=%2F%2F%E5%B9%B6%E9%BD%90%E6%A3%80%E6%B5%8B
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

		// 卡住检测vscode://lirentech.file-ref-tags?filePath=Self_Righting.c&snippet=%2F%2F+%E5%8D%A1%E4%BD%8F%E6%A3%80%E6%B5%8B
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
            //转动vscode://lirentech.file-ref-tags?filePath=Self_Righting.c&snippet=%2F%2F%E8%BD%AC%E5%8A%A8
            t_l = leg_turn_speed_control(&VMC_L, g_sr_turn_dir * g_sr_extend_unstuck_speed_l, g_sr_extend_unstuck_torque_max, g_sr_extend_unstuck_torque_ramp);
		}
		else//不卡
		{
            //被动阻尼和限幅vscode://lirentech.file-ref-tags?filePath=Self_Righting.c&snippet=%2F%2F%E8%A2%AB%E5%8A%A8%E9%98%BB%E5%B0%BC%E5%92%8C%E9%99%90%E5%B9%85
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

		//两腿腿长都到位检测vscode://lirentech.file-ref-tags?filePath=Self_Righting.c&snippet=%2F%2F%E4%B8%A4%E8%85%BF%E8%85%BF%E9%95%BF%E9%83%BD%E5%88%B0%E4%BD%8D%E6%A3%80%E6%B5%8B
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


        //第二阶段保持腿长vscode://lirentech.file-ref-tags?filePath=Self_Righting.c&snippet=%2F%2F%E7%AC%AC%E4%BA%8C%E9%98%B6%E6%AE%B5%E4%BF%9D%E6%8C%81%E8%85%BF%E9%95%BF
		PID_Set_Error(&L_Leg_L0_POS_PID, VMC_L.L0, LEG_MAX_LENTH);
		PID_Set_Error(&R_Leg_L0_POS_PID, VMC_R.L0, LEG_MAX_LENTH);
		PID_Set_Error(&L_Leg_L0_SPD_PID, VMC_L.d_L0, PID_coculate(&L_Leg_L0_POS_PID));
		PID_Set_Error(&R_Leg_L0_SPD_PID, VMC_R.d_L0, PID_coculate(&R_Leg_L0_POS_PID));

		f_l = PID_coculate(&L_Leg_L0_SPD_PID);
		f_r = PID_coculate(&R_Leg_L0_SPD_PID);

		//第二阶段反向匀速转vscode://lirentech.file-ref-tags?filePath=Self_Righting.c&snippet=%2F%2F%E7%AC%AC%E4%BA%8C%E9%98%B6%E6%AE%B5%E5%8F%8D%E5%90%91%E5%8C%80%E9%80%9F%E8%BD%AC
		t_l = leg_turn_speed_control(&VMC_L,   g_sr_turn_dir * g_sr_reverse_speed_l + PID_coculate(&anti_split_PID), g_sr_reverse_torque_max, g_sr_reverse_torque_ramp);
		t_r = leg_turn_speed_control(&VMC_R, -(g_sr_turn_dir * g_sr_reverse_speed_r - PID_coculate(&anti_split_PID)), g_sr_reverse_torque_max, g_sr_reverse_torque_ramp);

		//第二阶段检查并齐和卡住状态，用于决定何时进入第三阶段vscode://lirentech.file-ref-tags?filePath=Self_Righting.c&snippet=%2F%2F%E7%AC%AC%E4%BA%8C%E9%98%B6%E6%AE%B5%E6%A3%80%E6%9F%A5%E5%B9%B6%E9%BD%90%E5%92%8C%E5%8D%A1%E4%BD%8F%E7%8A%B6%E6%80%81%EF%BC%8C%E7%94%A8%E4%BA%8E%E5%86%B3%E5%AE%9A%E4%BD%95%E6%97%B6%E8%BF%9B%E5%85%A5%E7%AC%AC%E4%B8%89%E9%98%B6%E6%AE%B5
		aligned = (fabsf(VMC_L.phi0 - VMC_R.phi0) <= g_sr_align_tol) ? 1 : 0;
		turn_stuck_l = leg_turn_stuck_detect(&VMC_L, g_sr_turn_stuck_thresh, 0.1f);
		turn_stuck_r = leg_turn_stuck_detect(&VMC_R, g_sr_turn_stuck_thresh, 0.1f);

		if (aligned)//并齐
		{
			//并齐方式进入第三阶段vscode://lirentech.file-ref-tags?filePath=Self_Righting.c&snippet=%2F%2F%E5%B9%B6%E9%BD%90%E6%96%B9%E5%BC%8F%E8%BF%9B%E5%85%A5%E7%AC%AC%E4%B8%89%E9%98%B6%E6%AE%B5
			g_self_righting_stage = SELF_RIGHTING_STAGE_SYNC_HIGH_TORQUE;
			g_self_righting_sync_from_stuck = 0;
		}
		else if ((turn_stuck_l != 0) && (turn_stuck_r != 0))//未并齐但卡住
		{
			//差速方式进入第三阶段vscode://lirentech.file-ref-tags?filePath=Self_Righting.c&snippet=%2F%2F%E5%B7%AE%E9%80%9F%E6%96%B9%E5%BC%8F%E8%BF%9B%E5%85%A5%E7%AC%AC%E4%B8%89%E9%98%B6%E6%AE%B5
			g_self_righting_stage = SELF_RIGHTING_STAGE_SYNC_HIGH_TORQUE;
			g_self_righting_sync_from_stuck = 1;
		}
	}
	/* ===================== 第三阶段：大力矩匀速反向转，直到外层判定姿态恢复 ===================== */
	else if ((g_self_righting_stage == SELF_RIGHTING_STAGE_SYNC_HIGH_TORQUE && SR_test_state == 0) || (SR_test_state == 3))
	{
		L_DJ3508.Target_Torque = -PID_coculate(&wheel_PID_l);
		R_DJ3508.Target_Torque = PID_coculate(&wheel_PID_r);

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

	//VMC输出vscode://lirentech.file-ref-tags?filePath=Self_Righting.c&snippet=%2F%2FVMC%E8%BE%93%E5%87%BA
    sr_apply_cmd(f_l, t_l, f_r, t_r);

	turn_stuck_l = 0;
	turn_stuck_r = 0;

	aligned = 0;
        
	reached_ang_r = 0;
	reached_ang_l = 0;

    return 0;
}
