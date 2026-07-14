/**
 * @file chassis_control_task.cpp
 * @brief Motor_task 的第一阶段 C++ 灰度重构。
 *
 * 每周期固定执行：轮端状态更新 → 状态解算 → 顶层模式选择 → 模式控制计算
 * → VMC 最终映射 → 错误蜂鸣器。起立恢复、正常平衡、坐地和上台阶已由 C++ 控制器执行。
 */

#include "chassis_control_task.hpp"

extern "C"
{
#include "chassis_behavior_tree.h"
#include "User_State.h"
#include "USER_CAN.h"
#include "remoter.h"
#include "Wheel_End_Velocity.h"
#include "Wheel_Leg_about.h"
#include "VMC.h"
#include "observe_task.h"
#include "controller.h"
#include "PowerCtrl.h"
#include "Self_Righting.h"
#include "Board2Board.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
}

namespace chassis
{

/**
 * @brief 构造底盘任务，并把现有全局对象注入各 C++ 动作控制器。
 *
 * 此处是新控制器与旧工程全局对象之间唯一的装配位置。控制器本身
 * 只保存依赖引用，不在算法内部查找全局 PID 或 VMC 对象。
 */
ChassisControlTask::ChassisControlTask()
    : balance_controller_(BalanceControllerDependencies{
          VMC_Chassis_Target,
          L_Leg_L0_PID,
          R_Leg_L0_PID,
          Roll_Comp_PID,
          mg,
          upstairs_flag,
          StepHitDetectorDependencies{
              LEG_MAX_LENTH,
              motor_HZ,
              step_hit_cooldown,
              L_Ground_F0,
              R_Ground_F0}}),
      sit_controller_(SitControllerDependencies{
          L_Leg_L0_POS_PID,
          L_Leg_L0_SPD_PID,
          R_Leg_L0_POS_PID,
          R_Leg_L0_SPD_PID,
          L_Leg_Middle_PID,
          L_Leg_dphi0_PID,
          R_Leg_Middle_PID,
          R_Leg_dphi0_PID,
          mg}),
      stair_controller_(StairControllerDependencies{
          L_Leg_L0_POS_PID,
          L_Leg_L0_SPD_PID,
          R_Leg_L0_POS_PID,
          R_Leg_L0_SPD_PID,
          L_Leg_Middle_PID,
          L_Leg_dphi0_PID,
          R_Leg_Middle_PID,
          R_Leg_dphi0_PID,
          VMC_L,
          VMC_R,
          LEG_MIN_LENTH,
          LEG_MAX_LENTH}),
      startup_retract_controller_(StartupRetractControllerDependencies{
          L_Leg_L0_POS_PID,
          L_Leg_L0_SPD_PID,
          R_Leg_L0_POS_PID,
          R_Leg_L0_SPD_PID,
          L_Leg_Middle_PID,
          L_Leg_dphi0_PID,
          R_Leg_Middle_PID,
          R_Leg_dphi0_PID,
          VMC_L,
          VMC_R,
          VMC_Chassis_Target,
          Self_Righting_Mode_Detect,
          Self_Righting_Action})
{
}

/**
 * @brief 按迁移前顺序完成 Motor_task 的一次性初始化。
 */
void ChassisControlTask::Initialize()
{
    task_Motor_Init();
    task_VMC_Init();
    task_PID_Init();
    osDelay(1000);

    task_Pitch_Coculate();
    task_Motor_Enable();
}

/**
 * @brief 更新左右轮端速度和加速度到任务状态快照。
 */
void ChassisControlTask::UpdateWheelState()
{
    Wheel_End_Velocity_Both(&context_.state.wheel.left.velocity_mps,
                            &context_.state.wheel.left.acceleration_mps2,
                            &context_.state.wheel.right.velocity_mps,
                            &context_.state.wheel.right.acceleration_mps2);
}

/**
 * @brief 按旧任务顺序执行正常控制所需的全部输入和状态解算。
 */
void ChassisControlTask::UpdateNormalStateEstimates()
{
    // 保持迁移前顺序：输入更新 → VMC 几何解算 → 车速解算 → INS 解算。
    Keyboard_Simulate();
    VMC_Coculate();
    Body_Speed_Coculate();
    INS_Coculate();
}

/**
 * @brief 将本周期解算完成后的全局反馈复制为只读状态快照。
 *
 * 该函数不做滤波、不改符号、不参与状态估计，只建立明确的数据所有权边界。
 * 因此旧 C 控制器继续读取全局量时，控制效果不会发生变化。
 */
void ChassisControlTask::CaptureStateSnapshot()
{
    context_.state.left_leg.length_m = VMC_L.L0;
    context_.state.left_leg.length_rate_mps = VMC_L.d_L0;
    context_.state.left_leg.leg_angle_rad = VMC_L.phi0;
    context_.state.left_leg.leg_angular_rate_radps = VMC_L.d_phi0;
    context_.state.left_leg.body_angle_rad = VMC_L.b_phi0;
    context_.state.left_leg.body_angular_rate_radps = VMC_L.d_b_phi0;
    context_.state.left_leg.actual_leg_torque_nm = VMC_L.T_actual;

    context_.state.right_leg.length_m = VMC_R.L0;
    context_.state.right_leg.length_rate_mps = VMC_R.d_L0;
    context_.state.right_leg.leg_angle_rad = VMC_R.phi0;
    context_.state.right_leg.leg_angular_rate_radps = VMC_R.d_phi0;
    context_.state.right_leg.body_angle_rad = VMC_R.b_phi0;
    context_.state.right_leg.body_angular_rate_radps = VMC_R.d_b_phi0;
    context_.state.right_leg.actual_leg_torque_nm = VMC_R.T_actual;

    context_.state.body_speed_mps = kalman_body_speed;
    context_.state.pitch_rad = pitch_trans[0];
    context_.state.pitch_rate_radps = d_pitch;
    context_.state.yaw_rate_radps = d_yaw;
}

/**
 * @brief 锁存旧模式标志，确保一次模式判定只使用同一时刻的数据。
 */
void ChassisControlTask::CaptureModeSignals()
{
    context_.mode_signals.start_mode = start_mode;
    context_.mode_signals.stair_retract_mode = upstares_mode;
}

/**
 * @brief 把旧模式标志快照转换为唯一的顶层 RunMode。
 * @param[in] signals 本周期已经锁存的 start_mode/upstares_mode 兼容信号。
 * @return 与迁移前 if/else 优先级严格一致的顶层模式。
 */
RunMode ChassisControlTask::ResolveMode(const LegacyModeSignals& signals) const
{
    // 以下顺序严格保持原 if/else 分支优先级。
    if (signals.start_mode == 0 && signals.stair_retract_mode == 0)
        return RunMode::StartupRetract;
    if (signals.start_mode == 1)
        return RunMode::Balance;
    if (signals.start_mode == 2 && signals.stair_retract_mode == 0)
        return RunMode::Stair;
    if (signals.start_mode == 3)
        return RunMode::Sit;
    if (signals.stair_retract_mode == 1)
        return RunMode::Stair;

    return RunMode::Hold;
}

/**
 * @brief 执行一个顶层模式的控制计算。
 * @param[in] mode 本周期解析出的唯一运行模式。
 *
 * 起立恢复、正常平衡、坐地和上台阶模式由 C++ 控制器返回 ChassisCommand；尚未迁移的模式
 * 仍调用旧 C 动作组，并在周期末从 VMC_Chassis_Target 捕获命令。
 */
void ChassisControlTask::ExecuteMode(RunMode mode)
{
    context_.command_source = CommandSource::LegacyGlobal;

    switch (mode)
    {
    case RunMode::StartupRetract:
    {
        /** 起立恢复控制器返回的命令、当前分支与完成事件。 */
        const StartupRetractUpdateResult startup_result =
            startup_retract_controller_.Update(context_.state);
        context_.command = startup_result.command;
        context_.command_source = CommandSource::NativeCpp;

        if (startup_result.retract_control_active)
            first_run = 0U;

        if (startup_result.completed)
        {
            start_mode = 1U;
            L_Leg_State = 0U;
            R_Leg_State = 0U;
            body_distance = 0.0f;
            target_body_distance = 0.0f;
        }
        gimbal_follow_flag = 1;
        break;
    }

    case RunMode::Balance:
    {
        /** 平衡控制器使用的腿长档位、坐地请求、自动 Stair 开关和模式快照。 */
        BalanceControlInput balance_input{};
        balance_input.target_leg_state = Foot_Chassis.Target_Leg_State;
        balance_input.sit_requested = sit_mode_enable == 1U;
        balance_input.automatic_stair_climb_enabled =
            automatic_stair_climb_enable != 0U;
        balance_input.mode_signals = context_.mode_signals;

        /** 正常平衡算法链返回的最终命令和模式转换请求。 */
        const BalanceUpdateResult balance_result =
            balance_controller_.Update(context_.state, balance_input);
        context_.command = balance_result.command;
        context_.command_source = CommandSource::NativeCpp;

        if (balance_result.request_stair)
        {
            start_mode = 2U;
            upstairs_flag = 0U;
        }
        if (balance_result.request_sit)
            start_mode = 3U;
        break;
    }

    case RunMode::Stair:
    {
        stair_controller_.SynchronizeLegacyPhase(context_.mode_signals);
        /** 上台阶控制器返回的本周期命令和阶段转换事件。 */
        const StairUpdateResult stair_result =
            stair_controller_.Update(context_.state, context_.command);
        context_.command = stair_result.command;
        context_.command_source = CommandSource::NativeCpp;

        if (stair_result.entered_retract_phase)
            upstares_mode = 1U;

        if (stair_result.completed)
        {
            upstares_mode = 0U;
            start_mode = 1U;
            L_Leg_State = 0U;
            R_Leg_State = 0U;
            leg_state = 0U;
            target_Leg_L0 = LEG_MIN_LENTH;
            body_distance = 0.0f;
            target_body_distance = 0.0f;
        }
        break;
    }

    case RunMode::Sit:
    {
        /** 坐地控制器根据状态快照计算出的命令和模式切换请求。 */
        const SitUpdateResult sit_result =
            sit_controller_.Update(context_.state, sit_mode_enable != 0U);
        context_.command = sit_result.command;
        context_.command_source = CommandSource::NativeCpp;

        if (sit_result.request_startup_retract)
        {
            start_mode = 0;
            first_run = 1;
        }
        break;
    }

    case RunMode::GravityTest:
    case RunMode::Hold:
        break;
    }
}

/**
 * @brief 执行重力补偿测试的启动等待和测试动作。
 */
void ChassisControlTask::ExecuteGravityTest()
{
    if (context_.gravity_test_delay_count < 1000)
    {
        ++context_.gravity_test_delay_count;
        return;
    }

    Gravity_Compensation_Test_Function();
}

/**
 * @brief 从旧全局目标结构读取本周期动作组计算结果。
 *
 * 这是灰度迁移桥：尚未迁移的 C 动作组仍写 VMC_Chassis_Target，本函数立即
 * 将结果复制进 C++ ChassisCommand。随着动作组逐个迁移，这个读取桥会缩小，
 * 最终所有控制器将直接返回 ChassisCommand。
 */
void ChassisControlTask::CaptureLegacyCommand()
{
    context_.command.left_support_force_n = VMC_Chassis_Target.L_F0;
    context_.command.left_leg_torque_nm = VMC_Chassis_Target.L_T;
    context_.command.right_support_force_n = VMC_Chassis_Target.R_F0;
    context_.command.right_leg_torque_nm = VMC_Chassis_Target.R_T;
    context_.command.left_wheel_torque_nm = VMC_Chassis_Target.L_Wheel_Torque;
    context_.command.right_wheel_torque_nm = VMC_Chassis_Target.R_Wheel_Torque;
}

/**
 * @brief 将统一命令提交到现有 VMC 映射入口使用的兼容目标结构。
 *
 * 本函数只复制六个映射前目标，不进行限幅、符号变换或控制计算。
 */
void ChassisControlTask::CommitCommand(const ChassisCommand& command)
{
    VMC_Chassis_Target.L_F0 = command.left_support_force_n;
    VMC_Chassis_Target.L_T = command.left_leg_torque_nm;
    VMC_Chassis_Target.R_F0 = command.right_support_force_n;
    VMC_Chassis_Target.R_T = command.right_leg_torque_nm;
    VMC_Chassis_Target.L_Wheel_Torque = command.left_wheel_torque_nm;
    VMC_Chassis_Target.R_Wheel_Torque = command.right_wheel_torque_nm;
}

/**
 * @brief 从唯一出口提交命令、执行 VMC 映射并更新错误蜂鸣器。
 */
void ChassisControlTask::ApplyOutputs()
{
    // 所有控制模式共用唯一的命令提交点和最终 VMC 映射点。
    CommitCommand(context_.command);

    if (!chassis_hard_stop_flag)
        VMC_Apply_Chassis_Target();

    Error_Buzzer_Tick();
}

/**
 * @brief 执行一次 2 ms 控制周期。
 *
 * 周期内先更新状态，再选择并执行模式；只有旧 C 动作组需要经过
 * CaptureLegacyCommand()，原生 C++ 控制器返回的命令不会被旧全局量覆盖。
 */
void ChassisControlTask::RunCycle()
{
    UpdateWheelState();
    context_.command_source = CommandSource::LegacyGlobal;

    if (user_Gravity_Compensation_Test_Function_set == 1)
    {
        context_.mode = RunMode::GravityTest;
        ExecuteGravityTest();
    }
    else
    {
        UpdateNormalStateEstimates();
        CaptureStateSnapshot();
        CaptureModeSignals();
        context_.mode = ResolveMode(context_.mode_signals);
        ExecuteMode(context_.mode);
    }

    if (context_.command_source == CommandSource::LegacyGlobal)
        CaptureLegacyCommand();

    ApplyOutputs();
}

/**
 * @brief 初始化并永久运行 500 Hz 底盘控制循环。
 */
void ChassisControlTask::Run()
{
    Initialize();
    /** FreeRTOS 绝对延时基准，确保循环周期稳定为 2 ms。 */
    TickType_t last_wake_time = xTaskGetTickCount();

    for (;;)
    {
        RunCycle();
        osDelayUntil(&last_wake_time, 2);
    }
}

} // namespace chassis

/**
 * @brief FreeRTOS C 入口，转交给唯一的 C++ ChassisControlTask 实例。
 * @param[in] argument FreeRTOS 任务参数；当前未使用。
 */
extern "C" void Motor_task(void const *argument)
{
    (void)argument;
    /** Motor_task 唯一控制器实例；静态生命周期避免使用动态内存。 */
    static chassis::ChassisControlTask task;
    task.Run();
}
