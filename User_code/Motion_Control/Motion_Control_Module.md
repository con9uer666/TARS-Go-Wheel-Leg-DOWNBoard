# Motion_Control 运动控制模块文档

> 文件树：`others/Motion_Control/`，包含三层：App（应用层）、Ctrl（算法层）、Drv（驱动层）

---

## 1. 总体架构

```
┌─────────────────────────────────────────────────────────────┐
│                       App 层 (应用层)                        │
│  ChassisControlTask(主循环) + 可注入 C++ 动作控制器            │
│  Gimbal.c/h   self_righting_controller.cpp   User_State.c/h  │
│  行为树调度 / LQR / 跳跃 / 小陀螺 / 上台阶 / 坐地 / 离地检测  │
├─────────────────────────────────────────────────────────────┤
│                       Ctrl 层 (算法层)                        │
│  VMC.c/h  Wheel_Leg_about.c/h  Wheel_End_Velocity.c/h        │
│  observe_task.c/h  user_pid.c/h  Leg_Control.c/h             │
│  Gas_Spring.c/h  kalman_filter1.c/h                          │
│  正向/逆向运动学 / 卡尔曼滤波 / PID库 / LQR矩阵 / 防劈叉      │
├─────────────────────────────────────────────────────────────┤
│                       Drv 层 (驱动层)                         │
│  Motor_Drv.c/h    USER_CAN.c/h                               │
│  DM电机MIT模式 / CAN收发中断 / IMU数据解析                    │
└─────────────────────────────────────────────────────────────┘
```

### 数据流总图

```mermaid
flowchart TD
    subgraph 传感器输入
        IMU["IMU (BMI088)"]
        ENC["电机编码器\n(DM8009/DM4310/DM3508)"]
        B2B["板间通信\n(Board2Board)"]
    end

    subgraph Ctrl层算法
        VMC["VMC_Coculate()\n逆向运动学:\nphi0,d_phi0,L0,d_L0"]
        WL["Wheel_Leg_about.c\n正向运动学:\nWr, Wl, body_speed"]
        WEV["Wheel_End_Velocity.c\n轮端世界速度/加速度"]
        KF["observe_task.c\n卡尔曼滤波:\nkalman_body_speed"]
        LQR_F["LqrController::UpdateGainMatrix()\n二维多项式拟合 K 矩阵"]
    end

    subgraph App层控制
        STATE["LegacyModeSignals → 扁平 RunMode\nModeTransitionRequest 单点提交"]
        YAW["Yaw误差计算\nYaw_Error_Coculate()"]
        LEG["腿长斜坡 + PID\nChassisHeightController::Update()"]
        LQR_C["LqrController::Calculate()\n返回轮力矩 + 基础腿力矩"]
        JUMP["JumpController\n指令锁存 + 固定支持力"]
    end

    subgraph Drv层
        CAN["CAN发送\nMIT模式力矩指令"]
    end

    IMU --> VMC
    ENC --> VMC
    VMC --> WL
    VMC --> WEV
    WL --> KF
    KF --> STATE
    B2B --> STATE
    STATE --> YAW
    STATE --> LEG
    STATE --> LQR_C
    STATE --> JUMP
    LQR_C --> CAN
    LEG --> CAN
    JUMP --> CAN
```

---

## 2. 坐标系与符号约定

### 2.1 机身坐标系（body frame）

| 轴 | 正方向 | 说明 |
|---|---|---|
| X (前) | 机器人前进方向 | `body_speed` 正值 = 前进 |
| Y (左) | 左手定则确定 | 横向速度 |
| Z (上) | 垂直地面向上 | 俯仰轴 |

### 2.2 轮腿极坐标 (VMC 模型)

```
极坐标定义 (每条腿独立):
  r  = L0     — 腿长度 (m)，腿收缩方向为正
  θ  = phi0   — 腿摆角 (rad)，从竖直下垂起算
                 左腿: 前摆为正 (phi0增大=腿向前)
                 右腿: 前摆为正 (同向)
  b_phi0      — VMC等效摆角 (rad)，有偏置
```

**参考零点**:
- `phi0 = 0` : 腿竖直下垂
- `phi0 = PI/2` : 腿水平（站起后的常态）
- `L0 = 0.23m` (LEG_MIN) : 最短腿长
- `L0 = 0.39m` (LEG_MAX) : 最长腿长

### 2.3 云台坐标系 (gimbal frame)

| 参数 | 含义 | 零点 |
|---|---|---|
| `yaw_angle_PI` | 云台Yaw角 (rad) | 与底盘对齐时为0 |
| `head_forward_angle` | 云台物理零点偏置 (rad) | 机头正前方 |

### 2.4 世界坐标系

惯性参考系，用于 `vel_acc[0]` (卡尔曼滤波速度) 和 `body_distance` (累计位移)。

### 2.5 全局关键物理量

| 变量 | 含义 | 单位 | 正方向 |
|---|---|---|---|
| `body_speed` | 水平方向车身速度 | m/s | 前进为正 |
| `kalman_body_speed` | 卡尔曼滤波速度 | m/s | 前进为正 |
| `body_distance` | 车身位移积分 | m | 前进为正 |
| `Wr`, `Wl` | 左右轮世界系线速度 | m/s | 前进为正 |
| `d_yaw` | 车身Yaw角速度 | rad/s | CCW (逆时针) 为正 |
| `d_pitch` | 车身Pitch角速度 | rad/s | 抬头为正 |
| `pitch` | 机身俯仰角 | ° (IMU) | 抬头为正 |
| `roll` | 机身横滚角 | ° (IMU) | 右倾为正 |
| `yaw_angle_PI` | 云台相对Yaw角 | rad | 逆时针为正 |
| `Leg_L_T`, `Leg_R_T` | 腿杆力矩 (T = τ) | N·m | 前摆为正 |

---

## 3. 核心算法流程图

### 3.1 状态机 (Motor_task 主循环)

```mermaid
flowchart TD
    A["Motor_task 500Hz"] --> B["轮端状态更新"]
    B --> C["输入更新 + VMC/车速/INS 解算"]
    C --> D{"扁平 RunMode"}
    D -->|StartupRetract| E["StartupRetractController::Update\n倒地自起 → 收腿恢复"]
    D -->|Balance| F["BalanceController::Update\nLQR + 腿长PID + 跳跃 + 模式事件"]
    D -->|Stair| G["StairController::Update\n伸腿 → 收腿内部阶段"]
    D -->|Sit| I["SitController::Update\nC++ 坐地控制器"]
    D -->|GravityTest| J["GravityCompensationTestController::Update\n量程探测 → 姿态扫描"]
    D -->|Hold| K["保持上一周期目标"]
    
    E -->|两腿到位| D
    G -->|收腿完成| D
    F -->|磕台阶/坐地请求| D
```

### 3.2 VMC 逆向运动学 (VMC_Coculate)

```mermaid
flowchart LR
    A["电机编码器\n角度 + 转速"] --> B["正运动学转换\nWheel_Leg_about.c"]
    B --> C["得到 phi0, d_phi0\nL0, d_L0, b_phi0, d_b_phi0"]
    C --> D["算模拟地面支持力\nVMC_Get_Ground_F0()"]
```

### 3.3 VMC 正映射 (VMC_Set_F0_T)

```mermaid
flowchart LR
    A["F0 (腿杆力 N)\nT (腿摆力矩 N·m)"] --> B["雅可比矩阵\nJ^T = [∂L0/∂α, ∂L0/∂β;\n       ∂phi0/∂α, ∂phi0/∂β]"]
    B --> C["τ1 = J^T[0]*F0 + J^T[1]*T\nτ2 = J^T[2]*F0 + J^T[3]*T"]
    C --> D["CAN发送 MIT 指令\n(位置=0, 速度=0, KP=0, KD=0, 转矩=τ)"]
```

### 3.4 卡尔曼滤波速度估计 (observe_task)

```mermaid
flowchart LR
    A["accel_b[1]\n(IMU加速度 m/s²)"] --> B["积分: accel_speed"]
    C["body_speed\n(轮速解算)"] --> D["卡尔曼滤波\n状态: [速度, 加速度]"]
    B --> D
    D --> E["vel_acc[0]: 滤波速度\nvel_acc[1]: 滤波加速度"]
```

- 状态向量 2 维：`[v, a]`
- 测量向量 2 维：`[轮速解算速度(v), IMU加速度(a)]`
- 周期：`0.002s` (500Hz 积分，但 task 是 2ms delay = 隐式 500/2=250Hz)

### 3.5 LQR 控制器 (Balance 模式下)

```mermaid
flowchart LR
    A["K矩阵\nLQR_K[4][12]\n(左右腿长2D二次拟合)"] --> B["LqrController::Calculate()"]
    B --> C1["L_DJ3508.Target_Torque\n(左轮力矩 N·m)"]
    B --> C2["R_DJ3508.Target_Torque\n(右轮力矩 N·m)"]
    B --> C3["Leg_L_T\n(左腿摆力矩 N·m)"]
    B --> C4["Leg_R_T\n(右腿摆力矩 N·m)"]
```

LQR 状态（10维）：
```
[ body_distance_error, speed_error, yaw_error, d_yaw,
  VMC_L.b_phi0, VMC_L.d_b_phi0, VMC_R.b_phi0, VMC_R.d_b_phi0,
  pitch_trans[0], d_pitch ]
```

### 3.6 跳跃动作组

```mermaid
flowchart TD
    A["jump_cmd=1\n(B2B byte51)"] --> B{"条件满足?\njump_enable && 短腿\n&& !spin && 双腿着地"}
    B -->|YES| C["锁存0.5s\njump_F0直喂VMC"]
    C --> D{"离地检测?"}
    D -->|YES| E["jump_fail_reason=1\n跳跃成功"]
    D -->|NO, 0.5s到| F{"腿长变化?"}
    F -->|<阈值| G["jump_fail_reason=2\n失败(饱和/卡住)"]
    F -->|≥阈值| H["jump_fail_reason=3\n超时但伸够了"]
    E --> I["jump_locked=1\n等待jump_cmd回0"]
    G --> I
    H --> I
```

---

## 4. 接口清单

### 4.1 App 层对外函数

| 函数 | 文件 | 功能 |
|---|---|---|
| `Motor_task()` | chassis_control_task.cpp | C++ 主控制任务调度器，500Hz |
| `ChassisInitializer::InitializeMotors()` | chassis_initializer.cpp | 电机参数初始化 |
| `ChassisInitializer::InitializeVmc()` | chassis_initializer.cpp | VMC 结构体初始化 |
| `ChassisInitializer::InitializePids()` | chassis_initializer.cpp | PID 控制器初始化 |
| `task_Pitch_Coculate()` | chassis_initializer.cpp | 未迁移 INS C 代码使用的 pitch 历史兼容入口 |
| `ChassisStateEstimator::Update()` | chassis_state_estimator.cpp | 固定输入、VMC、车速、INS 顺序并组装只读状态快照 |
| `ChassisMotorEnabler::EnableAll()` | chassis_motor_enabler.cpp | 按固定顺序和 5 ms 间隔使能全部 DM 电机 |
| `BalanceController::Update()` | balance_controller.cpp | 固定平衡算法顺序，合成最终命令并返回 Stair/Sit 请求 |
| `TipProtectionController::Update()` | tip_protection_controller.cpp | 连续俯仰超限去抖、250 周期收腿保护命令及完成事件 |
| `LqrController::Calculate()` | lqr_controller.cpp | 更新积分状态并返回左右轮/腿 LQR 力矩 |
| `LqrController::UpdateGainMatrix()` | lqr_controller.cpp | 拥有节流计数，以 100 Hz 二维拟合 K(L0_l,L0_r) |
| `AntiSplitController::Update()` | anti_split_controller.cpp | 叠加防劈叉 PID、d_yaw² 离心补偿和小陀螺 phi0 归中力矩 |
| `spinning_up()` / `spinning_exit()` | spinning_controller.cpp | 保留给误差计算模块的 C 兼容入口，内部委托 `SpinningController` 完成小陀螺加速 / 退出控制 |
| `JumpController::Update()` | jump_controller.cpp | 管理单次触发锁、0.5 s 锁存、成功/失败原因、腿长 PID 阶跃与蜂鸣器边沿 |
| `OffGroundDetector::Update()` | off_ground_detector.cpp | 滤波左右支持力、更新离地计数并覆盖离地侧命令 |
| `StepHitDetector::Update()` | balance_controller.cpp | 持续更新磕台阶命中、长腿延时与离地冷却；自动触发默认关闭 |
| `Yaw_Error_Coculate()` | yaw_error.c | Yaw误差+速度误差 |
| `LegTurnRecoveryController::Update()` | leg_turn_recovery_controller.cpp | 单腿短路径转角控制与卡住后的反向长路径恢复 |
| `StartupRetractController::Update()` | startup_retract_controller.cpp | 组合原生 C++ 倒地自起与姿态恢复后的收腿 ChassisCommand |
| `StairController::Update()` | stair_controller.cpp | 单一 Stair 顶层模式内管理伸腿、收腿和完成等待阶段 |
| `SitController::Update()` | sit_controller.cpp | 读取状态快照并返回坐地 ChassisCommand |
| `GravityCompensationTestController::Update()` | gravity_compensation_test_controller.cpp | 探测机械腿长量程、扫描 18×10 位姿并返回结构化标定命令 |
| `Body_Speed_Coculate()` | Wheel_Leg_about.c | 车身速度解算 |
| `Distance_Error_Set()` | Wheel_Leg_about.c | 距离误差计算 |
| `Speed_Error_Set()` | Wheel_Leg_about.c | 速度误差计算 |
| `ChassisHeightController::Update()` | chassis_height_controller.cpp | 先更新横滚补偿，再以不对称斜坡更新腿长目标与左右 PID |
| `PowerCtrl()` | PowerCtrl.c | 功率门控 |
| `SelfRightingController::Update()` | self_righting_controller.cpp | 倒地进入/退出去抖与伸腿、并齐、大力矩转腿三阶段自起 |
| `Wheel_End_Velocity()` | Wheel_End_Velocity.c | 轮端世界速度+加速度 |
| `rampInit()` / `rampIterate()` | ramp_generator.c | 通用斜坡发生器 |

### 4.2 Ctrl 层对外函数

| 函数 | 文件 | 功能 |
|---|---|---|
| `VMC_Init()` | VMC.c | VMC初始化 |
| `VMC_Coculate()` | VMC.c | 逆向运动学（编码器→极坐标） |
| `VMC_Set_F0_T()` | VMC.c | 正映射（F0+T→电机力矩） |
| `VMC_Get_Ground_F0()` | VMC.c | 获取地面支持力 |
| `PID_INIT()` | user_pid.c | PID初始化 |
| `PID_coculate()` | user_pid.c | PID计算 |
| `PID_Set_Error()` | user_pid.c | 设置线性误差 |
| `PID_Set_AngleError()` | user_pid.c | 设置角度误差（自动最短路径） |
| `PID_Clear()` | user_pid.c | 清零积分 |
| `PID_Reset_OutLimit()` | user_pid.c | 重置输出限幅 |
| `xvEstimateKF_Init()` | observe_task.c | 卡尔曼滤波器初始化 |
| `xvEstimateKF_Update()` | observe_task.c | 卡尔曼滤波器更新 |
| `Kalman_Filter1_Init()` | kalman_filter1.c | 通用卡尔曼初始化 |
| `Kalman_Filter1_Update()` | kalman_filter1.c | 通用卡尔曼更新 |
| `Gas_Spring_GetForce()` | Gas_Spring.c | 气弹簧等效力的拟合 |
| `RAMP_float()` | observe_task.c | 斜坡函数 |
| `LQR_Get_K()` | Wheel_Leg_about.c | LQR矩阵1D二次拟合查表 |
| `anti_split_is_stuck()` | Leg_Control.c | 防劈叉卡住检测 |
| `leg_turn_stuck_detect()` | Leg_Control.c | 腿角卡住检测 |
| `leg_turn_stuck_reset()` | Leg_Control.c | 清卡住计数器 |
| `leg_turn_speed_control()` | Leg_Control.c | 收腿角速度控制 |
| `AntiSplit_Get_K()` | Leg_Control.c | 防劈叉PID参数1D拟合 |
| `INS_Coculate()` | Wheel_Leg_about.c | 惯性导航解算 |
| `Polar_Get()` | Wheel_Leg_about.c | 极坐标信息获取 |
| `body_distance_count()` | Wheel_Leg_about.c | 位移积分 |
| `Wheel_End_Velocity()` | Wheel_End_Velocity.c | 轮端速度+加速度计算 |

### 4.3 Drv 层对外函数

| 函数 | 文件 | 功能 |
|---|---|---|
| `DM_Joint_Motor_Init()` | Motor_Drv.c | DM电机MIT初始化 |
| `Enable_DM_Motor_MIT()` | Motor_Drv.c | DM电机使能 |
| `DM_Motor_Position_Ctrl()` | Motor_Drv.c | 位置模式MIT指令 |
| `DM_Motor_Speed_Ctrl()` | Motor_Drv.c | 速度模式MIT指令 |
| `CAN_Send_MIT_Msg()` | Motor_Drv.c | CAN MIT消息发送 |
| `CAN_Send_AMT16_Msg()` | Motor_Drv.c | AMT16编码器读取 |
| `CAN1_IMU_Filter()` | USER_CAN.c | CAN1 IMU数据滤波 |
| `CAN2_IMU_Filter()` | USER_CAN.c | CAN2 IMU数据滤波 |

---

## 5. 关键参数与单位

| 参数 | 值 | 单位 | 说明 |
|---|---|---|---|
| WHEEL_RADIUS | 0.061 | m | 轮子半径 |
| wheel_track_R | 0.19242 | m | 半轮距（差速半径） |
| LEG_MIN_LENTH | 0.23 | m | 最短腿长 |
| LEG_MAX_LENTH | 0.39 | m | 最长腿长 |
| motor_HZ | 500 | Hz | 控制频率 |
| 控制周期 dt | 0.002 | s | 2ms |
| speed_limit | 1.3 | m/s | 速度限幅 |
| mg | 30 (60/2) | N | 单腿承担重力 ≈ 3kg |
| jump_F0 | 50 | N | 跳跃虚拟力 |
| Leg_F0_Limit | 500 | N | F0幅值上限 |
| PITCH_OFFSET | -0.10 | rad | 常态pitch偏置 |
| WHEEL_RADIUS | 0.061 | m | 轮子半径 |

---

## 6. 文件树总览

```
others/Motion_Control/
├── App/                                  应用层
│   ├── inc/
│   │   ├── chassis_behavior_tree.h      ★ 底盘聚合公共头（取代 motor.h，所有extern/类型/原型）
│   │   ├── chassis_control_task.hpp      C++ 任务调度器、扁平模式与任务上下文
│   │   ├── chassis_control_types.hpp     C++ 控制器共享的状态快照、模式与命令类型
│   │   ├── chassis_state_estimator.hpp   输入/VMC/车速/INS 解算与快照接口
│   │   ├── chassis_initializer.hpp       电机、VMC、PID 与 pitch 历史初始化接口
│   │   ├── chassis_motor_enabler.hpp     六个 DM 电机顺序使能接口
│   │   ├── balance_controller.hpp       正常平衡流程与磕台阶检测器接口
│   │   ├── tip_protection_controller.hpp 倾覆检测、收腿保护命令与完成事件接口
│   │   ├── lqr_controller.hpp           12 维 LQR 输入/输出、配置与控制器接口
│   │   ├── anti_split_controller.hpp    防劈叉、离心补偿与小陀螺腿角归中控制器
│   │   ├── off_ground_detector.hpp      支持力滤波、离地计数与单腿命令覆盖接口
│   │   ├── chassis_height_controller.hpp 横滚补偿与腿长斜坡/PID 控制器
│   │   ├── spinning_controller.hpp      小陀螺加速/退出控制器、输入快照与依赖接口
│   │   ├── jump_controller.hpp           跳跃指令锁存、失败判定、PID 阶跃与蜂鸣控制器
│   │   ├── sit_controller.hpp           坐地控制器接口、依赖与参数
│   │   ├── stair_controller.hpp         上台阶控制器接口、内部阶段与参数
│   │   ├── startup_retract_controller.hpp 倒地自起接管与起立前收腿控制器
│   │   ├── self_righting_controller.hpp   倒地检测去抖与三阶段自起命令接口
│   │   ├── leg_turn_recovery_controller.hpp 单腿收腿转角与卡住反绕状态机
│   │   ├── gravity_compensation_test_controller.hpp 重力标定控制器与调试观测量
│   │   ├── Gimbal.h                     云台控制
│   │   ├── Self_Righting.h              倒地自复位
│   │   └── User_State.h                 用户状态
│   └── src/
│       ├── chassis_control_task.cpp     ★ C++ 主循环：状态更新→模式调度→最终映射
│       ├── chassis_runtime_state.cpp    C linkage 共享反馈量/常数/标志/共享 PID
│       ├── chassis_initializer.cpp      C++ 电机/VMC/PID 初始化 + pitch 兼容入口
│       ├── chassis_state_estimator.cpp  C++ 输入→VMC→车速→INS→状态快照
│       ├── chassis_motor_enabler.cpp    C++ 六个 DM 电机顺序使能动作
│       ├── balance_controller.cpp       C++ 正常平衡流程 + 磕台阶检测器
│       ├── tip_protection_controller.cpp C++ 倾覆去抖 + 收腿双环保护命令
│       ├── lqr_controller.cpp           C++ LQR 力矩输出 + K 矩阵节流拟合
│       ├── anti_split_controller.cpp    C++ 防劈叉 + 旋转离心补偿 + phi0 归中
│       ├── off_ground_detector.cpp      C++ 支持力滤波 + 离地单侧命令覆盖
│       ├── chassis_height_controller.cpp C++ 横滚补偿 + 腿长斜坡/PID
│       ├── spinning_controller.cpp      小陀螺控制器实现及 C 兼容入口（加速/退出）
│       ├── jump_controller.cpp          C++ 跳跃锁存 + 失败判定 + PID/蜂鸣更新
│       ├── yaw_error.c                  常态Yaw误差计算
│       ├── leg_turn_recovery_controller.cpp C++ 单腿短路径转角 + 卡住反向长路径恢复
│       ├── startup_retract_controller.cpp C++ 起立恢复控制器（原生自起→收腿）
│       ├── stair_controller.cpp         C++ 上台阶控制器（伸腿→收腿内部阶段）
│       ├── sit_controller.cpp           C++ 坐地控制器（状态输入→命令输出）
│       ├── gravity_compensation_test_controller.cpp C++ 机械量程探测 + 重力补偿姿态扫描
│       ├── Gimbal.c
│       ├── self_righting_controller.cpp  C++ 倒地进入/退出去抖 + 三阶段自起
│       └── User_State.c
├── Ctrl/                                 算法层
│   ├── inc/
│   │   ├── Gas_Spring.h                 气弹簧拟合
│   │   ├── kalman_filter1.h             通用卡尔曼滤波
│   │   ├── Leg_Control.h                腿长/防劈叉/收腿控制
│   │   ├── observe_task.h               速度估计任务
│   │   ├── ramp_generator.h             通用斜坡发生器
│   │   ├── user_pid.h                   用户PID库
│   │   ├── VMC.h                        VMC虚拟模型控制
│   │   ├── Wheel_End_Velocity.h         轮端世界速度
│   │   └── Wheel_Leg_about.h            轮腿运动学 + LQR
│   └── src/
│       ├── Gas_Spring.c
│       ├── kalman_filter1.c
│       ├── Leg_Control.c
│       ├── observe_task.c
│       ├── ramp_generator.c             通用斜坡发生器
│       ├── user_pid.c
│       ├── VMC.c
│       ├── Wheel_End_Velocity.c         ★ 轮端速度加速度
│       └── Wheel_Leg_about.c            ★ 正逆运动学 + LQR拟合
└── Drv/                                  驱动层
    ├── inc/
    │   ├── Motor_Drv.h                  DM电机驱动
    │   └── USER_CAN.h                   CAN通信
    └── src/
        ├── Motor_Drv.c
        └── USER_CAN.c
```

---

## 7. 版本历史

| 日期 | 变更 |
|---|---|
| 2025 | 丛庆：初始代码，pitch_trans/yaw_trans 数组 |
| 2026.05 | 补充跳跃动作组、防劈叉、坐地模式、收腿反向绕长路 |
| 2026.05 | 补充 Wheel_End_Velocity 轮端世界速度计算 |
| 2026.06 | 将 motor.c 按"底盘行为树 + 细粒度动作/计算文件"拆分；motor.h→chassis_behavior_tree.h 聚合头；新增 ramp_generator |
