/**
 * @file chassis_control_task.cpp
 * @brief Motor_task 的第一阶段 C++ 灰度重构。
 *
 * 每周期固定执行：轮端状态更新 → 状态解算 → 顶层模式选择 → 模式控制计算
 * → VMC 最终映射 → 错误蜂鸣器。起立恢复、正常平衡、坐地和上台阶已由 C++ 控制器执行。
 */

#include "chassis_control_task.hpp"
#include "arm_math.h"

extern "C"
{
#include "chassis_behavior_tree.h"
#include "User_State.h"
#include "USER_CAN.h"
#include "Motor_Drv.h"
#include "remoter.h"
#include "Wheel_End_Velocity.h"
#include "Wheel_Leg_about.h"
#include "VMC.h"
#include "observe_task.h"
#include "controller.h"
#include "PowerCtrl.h"
#include "Board2Board.h"
#include "imu_temp_ctrl.h"
#include "buzzer.h"
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
    : state_estimator_(ChassisStateEstimatorDependencies{
          Keyboard_Simulate,
          VMC_Coculate,
          Body_Speed_Coculate,
          INS_Coculate,
          VMC_L,
          VMC_R,
          kalman_body_speed,
          pitch_trans,
          d_pitch,
          d_yaw,
          pitch,
          roll}),
      left_leg_turn_recovery_(false),
      right_leg_turn_recovery_(true),
      balance_controller_(BalanceControllerDependencies{
          L_Leg_L0_PID,
          R_Leg_L0_PID,
          Roll_Comp_PID,
          mg,
          upstairs_flag,
          ChassisHeightControllerDependencies{
              Roll_Comp_PID,
              L_Leg_L0_PID,
              R_Leg_L0_PID,
              LEG_MIN_LENTH,
              LEG_MAX_LENTH,
              target_Leg_L0,
              target_L_Leg_L0,
              target_R_Leg_L0,
              leg_state_count,
              ramp_target_L0_up,
              ramp_target_L0_down},
          JumpControllerDependencies{
              L_Leg_L0_PID,
              R_Leg_L0_PID,
              jump_mode,
              jump_cmd,
              jump_enable,
              g_jump_buzzer_active,
              jump_L0_step_delta,
              jump_leg_change_threshold,
              jump_fail_reason,
              motor_HZ,
              Buzzer_Tone_Max,
              Stop_Buzzer},
          AntiSplitControllerDependencies{
              Leg_AntiSplit_PID,
              L_Spin_Phi0_PID,
              R_Spin_Phi0_PID,
              LEG_MIN_LENTH,
              LEG_MAX_LENTH,
              centrifugal_comp_gain,
              target_spin_phi0},
          OffGroundDetectorDependencies{
              VMC_L,
              VMC_R,
              L_Leg_L0_PID,
              R_Leg_L0_PID,
              LQR_K,
              Leg_L_T,
              Leg_R_T,
              b_phi0_offset,
              L_Ground_F0,
              R_Ground_F0,
              L_off_ground,
              R_off_ground},
          leg_state_locked_short,
          body_distance,
          target_body_distance,
          StepHitDetectorDependencies{
              LEG_MAX_LENTH,
              motor_HZ,
              step_hit_cooldown,
              L_Ground_F0,
              R_Ground_F0},
          TipProtectionControllerDependencies{
              L_Leg_L0_POS_PID,
              L_Leg_L0_SPD_PID,
              R_Leg_L0_POS_PID,
              R_Leg_L0_SPD_PID,
              L_Leg_Middle_PID,
              L_Leg_dphi0_PID,
              R_Leg_Middle_PID,
              R_Leg_dphi0_PID,
              LEG_MIN_LENTH},
          LegacyBalanceAlgorithmDependencies{
              Error_Calculate,
              PowerCtrl,
              body_distance_error,
              speed_error,
              yaw_error},
          spinning_flag}),
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
          LEG_MAX_LENTH,
          left_leg_turn_recovery_,
          right_leg_turn_recovery_}),
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
          left_leg_turn_recovery_,
          right_leg_turn_recovery_})
{
}

/**
 * @brief 按迁移前顺序完成 Motor_task 的一次性初始化。
 */
void ChassisControlTask::Initialize()
{
    ChassisInitializer::InitializeMotors();
    ChassisInitializer::InitializeVmc();
    ChassisInitializer::InitializePids();
    osDelay(1000);

    ChassisInitializer::UpdatePitchHistory();
    ChassisMotorEnabler::EnableAll();
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
    context_.state.wheel.left.raw_motor_speed = L_DJ3508.Rx_Data.Speed;
    context_.state.wheel.right.raw_motor_speed = R_DJ3508.Rx_Data.Speed;
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
 * 起立恢复、正常平衡、坐地和上台阶模式均由 C++ 控制器返回 ChassisCommand；
 * Hold 分支不覆盖命令，从而保持上周期已经提交的目标。
 */
void ChassisControlTask::ExecuteMode(RunMode mode)
{
    context_.transition_request = ModeTransitionRequest::None;

    switch (mode)
    {
    case RunMode::StartupRetract:
    {
        /** 起立恢复控制器返回的命令、当前分支与完成事件。 */
        const StartupRetractUpdateResult startup_result =
            startup_retract_controller_.Update(context_.state);
        context_.command = startup_result.command;

        if (startup_result.retract_control_active)
            first_run = 0U;

        if (startup_result.completed)
        {
            context_.transition_request = ModeTransitionRequest::Balance;
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
        if (previous_mode_ != RunMode::Balance)
            balance_controller_.SuppressTipProtection(500U);

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

        if (balance_result.tip_protection_active)
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);

        if (balance_result.tip_protection_completed)
        {
            context_.transition_request = ModeTransitionRequest::StartupRetract;
            upstares_mode = 0U;
            first_run = 1U;
            sit_first_entry = 1U;
        }

        if (balance_result.request_stair)
        {
            context_.transition_request = ModeTransitionRequest::Stair;
            upstairs_flag = 0U;
        }
        if (balance_result.request_sit)
            context_.transition_request = ModeTransitionRequest::Sit;
        break;
    }

    case RunMode::Stair:
    {
        stair_controller_.SynchronizeLegacyPhase(context_.mode_signals);
        /** 上台阶控制器返回的本周期命令和阶段转换事件。 */
        const StairUpdateResult stair_result =
            stair_controller_.Update(context_.state, context_.command);
        context_.command = stair_result.command;

        if (stair_result.entered_retract_phase)
            upstares_mode = 1U;

        if (stair_result.completed)
        {
            upstares_mode = 0U;
            context_.transition_request = ModeTransitionRequest::Balance;
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

        if (sit_result.request_startup_retract)
        {
            context_.transition_request = ModeTransitionRequest::StartupRetract;
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

    // 与迁移前一致：标定分支只更新 VMC 几何状态，不运行正常模式的输入/车速/INS 链。
    VMC_Coculate();
    context_.command = gravity_test_controller_.Update();
}

/**
 * @brief 把控制器产生的顶层模式请求集中映射为旧 start_mode 数值。
 * @param[in] request 本周期最终模式转换请求；None 时保持现有模式。
 */
void ChassisControlTask::ApplyModeTransition(ModeTransitionRequest request)
{
    switch (request)
    {
    case ModeTransitionRequest::StartupRetract:
        start_mode = 0U;
        break;
    case ModeTransitionRequest::Balance:
        start_mode = 1U;
        break;
    case ModeTransitionRequest::Stair:
        start_mode = 2U;
        break;
    case ModeTransitionRequest::Sit:
        start_mode = 3U;
        break;
    case ModeTransitionRequest::None:
        break;
    }
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
 * 周期内先更新状态，再选择并执行模式。所有动作控制器均直接更新
 * context_.command；Hold 和重力测试启动等待期不覆盖它，因此自然保持上周期命令。
 */
void ChassisControlTask::RunCycle()
{
    UpdateWheelState();

    if (user_Gravity_Compensation_Test_Function_set == 1)
    {
        context_.mode = RunMode::GravityTest;
        ExecuteGravityTest();
    }
    else
    {
        state_estimator_.Update(context_.state);
        CaptureModeSignals();
        context_.mode = ResolveMode(context_.mode_signals);
        ExecuteMode(context_.mode);
        ApplyModeTransition(context_.transition_request);
    }

    ApplyOutputs();
    previous_mode_ = context_.mode;
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
