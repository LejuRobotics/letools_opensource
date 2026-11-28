# 07 手臂与头部并行-instructional_case_02

[返回文档首页](../README.md) · [04-简单案例运行](../04-简单案例运行/) · [03-接口详细文档](../03-接口详细文档/)

本文中的相对路径一律相对 **LeTools 仓库根目录**；各命令需在仓库根目录下执行。

**难度**：⭐⭐

**适合读者**：跑通了 [06-头部顺序动作](06-头部顺序动作-instructional_case_01.md)，想让多个部位同时动作的人。

**前置知识**：`Sequence`、`memory`、`READ_BOARD`（见 [06](06-头部顺序动作-instructional_case_01.md)）。

**核心概念**：用 `Parallel` 组合头部和手臂动作；`Parallel` 按顺序 tick 子节点，子节点的排列顺序决定物理动作能否重叠；对比两种手臂关节接口——TimedCmd 单目标和 SDK 关节轨迹。

**学习目标**：

1. 看懂「`Sequence` 套 5 个 `Parallel`」的阶段式结构；
2. 理解阶段 1 非阻塞等待与阶段 2–5 SDK 阻塞发送的区别，以及头部前置顺序的作用；
3. 分清 `TimedArmJoint` 和 `ArmJointTrajSdkMove` 的差别，知道 SDK 轨迹的起点为什么必须等于当前姿态；
4. 会在 `board.json` 中修改双臂姿态，并保持相邻阶段的轨迹首尾相接。

**由简到难进阶路线**：[06](06-头部顺序动作-instructional_case_01.md)（头部顺序动作）→ **本篇**（头部 + 双臂）→ [08](08-头手躯干串行-instructional_case_03.md)（头部 → 躯干 → 双臂串行）→ [09](09-头手躯干与底盘并行-instructional_case_04.md)（加入底盘）。

---

## 一、场景定位

| 维度 | 值 |
|------|----|
| 场景目录 | `orchestration/scenarios/instructional_case_02/` |
| 主树 | `py_tree.json`，根 `Sequence(five_arm_head_phases)`，共 **18 个节点** |
| 子树 | `py_tree_child.json` 为空集合，本案例不使用子树 |
| 黑板 | `board.json`：手臂回零目标、四条手臂关节轨迹、头部角度、动作时间 |
| 硬件配置 | 无 `hardware_config.json`，使用 `config/app_config.yaml` 的全局配置（默认会初始化本案例需要的 TimedCmd、Arm SDK 与 LowLevel SDK 管理器） |
| 运动部位 | 头部、双臂 |

---

## 二、树结构

```text
Sequence(five_arm_head_phases)  memory=true
├── Parallel(phase_1_zero_and_down)        双臂全零   + 低头
├── Parallel(phase_2_forward_and_left)     双臂前伸   + 左转
├── Parallel(phase_3_up_and_up)            双臂上举   + 抬头
├── Parallel(phase_4_spread_and_right)     双臂展开   + 右转
└── Parallel(phase_5_zero_and_center)      双臂回全零 + 头部正中
```

每个 `Parallel` 的子节点固定为两个，头部在前、手臂在后：

```text
Parallel(policy=success_on_all)
├── HeadControlSdkMove     下发头部目标后立即返回
└── 手臂节点               下发后定时等待 / 执行 SDK 轨迹发送循环
    ├── 阶段 1：Sequence(memory=true)：TimedArmJoint → WaitSeconds
    └── 阶段 2–5：ArmJointTrajSdkMove
```

### 为什么头部要排在前面

`Parallel` 不是多线程，而是在每一拍里按顺序 tick 子节点：

1. `HeadControlSdkMove` 先下发头部目标并返回，头部开始运动；
2. 阶段 1 的手臂分支提交 TimedCmd 后，由独立 `WaitSeconds` 非阻塞地返回 `RUNNING`；阶段 2–5 执行阻塞的 SDK 轨迹发送循环。两条路径都不查询实测到位。

这样头部和手臂的物理动作可以重叠。阶段 1 等待不阻塞 tick，交换顺序不会因等待而延后头部；阶段 2–5 若把手臂节点放在前面，SDK 发送循环会阻塞本拍，头部命令要等其返回后才下发。

`success_on_all` 表示两个子节点都成功后本阶段才成功；任一子节点失败，本阶段和外层 `Sequence` 都会失败。本案例不需要用 `Async` 包裹。

---

## 三、手臂目标

- 关节顺序：`[左臂 J0–J6, 右臂 J0–J6]`；
- 单位：TimedArmJoint 固定度；SDK 轨迹沿用适配器 `angle_unit=deg` 配置；
- 每阶段规划期望/轨迹发送时长：3 秒（`arm_motion_duration_sec`），不是整个阶段的精确耗时或实测到位时限。

| 阶段 | 手臂节点 | 手臂目标 | 头部目标 |
|---|---|---|---|
| 1 | `TimedArmJoint → WaitSeconds` | 双臂全零 | 低头：`yaw=0°`，`pitch=20°` |
| 2 | `ArmJointTrajSdkMove` | 双臂 J0=`-30°`，其余 `0°`（前伸） | 左转：`yaw=20°`，`pitch=0°` |
| 3 | `ArmJointTrajSdkMove` | 双臂 J3=`-90°`，其余 `0°`（上举） | 抬头：`yaw=0°`，`pitch=-20°` |
| 4 | `ArmJointTrajSdkMove` | 左臂 J1=`90°`、右臂 J1=`-90°`，其余 `0°`（展开） | 右转：`yaw=-20°`，`pitch=0°` |
| 5 | `ArmJointTrajSdkMove` | 双臂全零 | 正中：`yaw=0°`，`pitch=0°` |

### 两种手臂关节接口

| 节点 | 底层调用 | 插值位置 | 输入 |
|---|---|---|---|
| `TimedArmJoint` | `hardware.timed_arm_joint()` | TimedCmd 服务端，从**当前位置**规划到目标 | 单个 14 维目标 |
| `ArmJointTrajSdkMove` | `hardware.send_arm_joint_traj_sdk()` | 客户端，50 Hz 线性插值后逐帧下发 | 至少两个 14 维关键点 |

`ArmJointTrajSdkMove` 不读取手臂当前位置，只在给定的关键点之间插值，由此有两条约束：

- 轨迹的**第一个点必须等于手臂当前姿态**，否则手臂会从当前位置跳到第一个点；
- 轨迹**至少要有两个点**。只给一个点时接口只打印告警、手臂不动，节点仍返回成功。

所以阶段 2 至 5 的轨迹都写成 `[上一阶段姿态, 本阶段姿态]`。阶段 1 使用 `TimedArmJoint` 请求从当前位置规划回零，发送成功即返回，由独立 `WaitSeconds` 按返回的 `actual_time`（字段缺失（含无返回数据） 时用本次请求的 3 秒）等待；该等待节点的 `settle_time` 默认 0。它不测量回零误差，不能保证任意起始姿态都能安全回零，也不能据此断言 SDK 轨迹实机起点已吻合。配置首尾一致只是必要条件。

`ArmJointTrajSdkMove` 每次执行前设置手臂外部模式 2、MPC `ArmOnly` / `DirectToWbc`；结束时尝试恢复 MPC `NoControl` / `ThroughFullBodyMpc`，前后各等待 0.5 秒。它不恢复启动前的手臂模式、不切回模式 0，也不物理复位。阶段 2–5 除 3 秒轨迹发送外还有约 1 秒模式等待及调用开销；发送结束或恢复请求返回不是实测到位/恢复确认。

两条路径的完整对比见 [`09-控制路径与选型`](../03-接口详细文档/09-控制路径与选型.md)。

---

## 四、参数从 `board.json` 读取

| 黑板键 | 内容 |
|---|---|
| `arm_zero_joint_angles` | 双臂全零，单个 14 维目标，阶段 1 使用 |
| `arm_zero_to_forward_traj` | 阶段 2 轨迹：全零 → 前伸 |
| `arm_forward_to_up_traj` | 阶段 3 轨迹：前伸 → 上举 |
| `arm_up_to_spread_traj` | 阶段 4 轨迹：上举 → 展开 |
| `arm_spread_to_zero_traj` | 阶段 5 轨迹：展开 → 全零 |
| `arm_motion_duration_sec` | 每阶段手臂动作时间，默认 `3.0` 秒 |
| `head_left_yaw_deg` / `head_right_yaw_deg` | 左右转 yaw |
| `head_up_pitch_deg` / `head_down_pitch_deg` | 抬头和低头 pitch |
| `head_center_yaw_deg` / `head_center_pitch_deg` | 正中角度，也用于补齐单轴动作 |

手臂目标在 `board.json` 中是原生 JSON 数组。阶段 1 通过 `READ_BOARD` 把单个目标传给 `TimedArmJoint`：

```json
"joint_angles": {
  "source": "READ_BOARD",
  "board_key": "arm_zero_joint_angles"
}
```

`TimedArmJoint.joint_angles` 与左右分臂参数互斥；运行时读取 14D 数组后拆为左右各 7D。发送节点使用 `desire_time`，并配置 `actual_time_output_key=<label>__actual_time`；随后 `WaitSeconds.duration_sec` 通过 `READ_BOARD` 读此输出。两节点包在 `memory=true` 的 Sequence 中，避免等待期间重复发令。

阶段 2 至 5 把轨迹传给 `ArmJointTrajSdkMove`。`use_board_trajectory` 必须设为 `false`，节点才会使用 `joint_traj` 参数，而不是去读黑板上固定的 `ArmJointTrajectories` 键：

```json
"use_board_trajectory": {
  "value": "false",
  "source": "CUSTOM",
  "data_type": "string"
},
"joint_traj": {
  "source": "READ_BOARD",
  "board_key": "arm_zero_to_forward_traj"
}
```

修改姿态或动作时间只需编辑 `board.json`，改完重启行为树生效。修改时保持：

- 每个关键点**严格为 14 个元素**；
- 相邻阶段**首尾相接**：每条轨迹的第一个点等于上一阶段的最后一个点，阶段 2 的第一个点等于 `arm_zero_joint_angles`。

干跑会检查每个关键点是否为 14 个元素，但不检查首尾是否相接，也不检查轨迹是否至少有两个点。运行前需人工核对后两项；不要用真机试验不连续的轨迹。

---

## 五、运行

### 5.1 干跑

```bash
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/instructional_case_02 \
  --dry-run --tick-once
```

预期输出（关键行）：

```text
[apps][dry-run] 已加载树，根节点: five_arm_head_phases
[apps][dry-run] tick 后根状态: Status.SUCCESS
```

以 `tick 后根状态` 判断是否通过。当前启动器在 dry-run 根状态为 `Status.FAILURE` 时仍退出 `0`，不能只凭进程退出码判断通过。

### 5.2 仿真或真机

按 [06 第五节](06-头部顺序动作-instructional_case_01.md#52-仿真或真机) 启动控制器和 TF 转发，并确认 TimedCmd、Arm SDK、LowLevel SDK 及模式服务就绪。**本案例阶段 1 显式设置 `prepare_arm_control=false`，`TimedArmJoint` 不切换手臂模式**，必须先确认外部控制模式允许 TimedCmd 接管；后续 SDK 阶段的模式切换不能补足此前的条件。确认当前姿态、限位和空间允许本次动作后，在终端 3 运行：

```bash
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/instructional_case_02
```

> ⚠️ **真机**：运行前按 [02-执行命令 · 真机运行前安全检查](02-执行命令.md#真机运行前安全检查) 逐项确认。本案例双臂会前伸、上举和侧向展开 90°，确认双臂和头部的完整运动范围内没有人员、线缆或其他障碍物。

阶段 2 至 5 每次开始时可能出现下面两条 `WARNING`。第一条说明当前固件主要消费控制流；第二条说明模式服务不可用、仅更新本地状态。不能直接把它们判作“无影响”，需结合其他错误、控制状态与实际动作验证：

```text
[WARNING] ...base_sdk_manager: set_mpc_mode: mode 在当前固件构建下无实际控制效果（仅 flow 生效），详见方法 docstring
[WARNING] kuavo-humanoid-sdk: [Core] Manipulation MPC control mode service not available, updating local stateonly
```

---

## 六、验证方式

| 层次 | 判据 |
|------|------|
| 加载 | 干跑打印 `根节点: five_arm_head_phases` |
| 单拍 tick | `tick 后根状态: Status.SUCCESS` |
| 仿真 / 真机 | 检查五阶段调用顺序，结合实际反馈/观察记录头臂动作、轨迹起点和跳变情况；退出码为 `0` 且日志无错误只是程序结果，不证明到位或安全 |
| 收尾 | 确认 MPC 控制流及手臂模式；场景不恢复启动前状态、不自动复位 |

---

## 七、动手改一改

| 改什么 | 怎么改 | 预期现象 |
|---|---|---|
| 手臂动得更慢 | `arm_motion_duration_sec` 改为 `5.0` | TimedCmd 期望/SDK 发送时长改为 5 秒；到位需另验，阶段还含等待与调用开销 |
| 换一个姿态 | 同时修改 `arm_zero_to_forward_traj` 的第二个点和 `arm_forward_to_up_traj` 的第一个点（两处一致，各保持 14 个元素） | 阶段 2 的手臂姿态变化，阶段 3 从新姿态平滑起动 |
| 给轨迹加一个中间点 | 在 `arm_up_to_spread_traj` 的两个点之间插入一个 14 维关键点 | 阶段 4 的手臂先经过中间点再展开，总时间不变 |
| 观察子节点顺序的影响 | 在阶段 2–5 的 `Parallel` 中把手臂节点移到 `HeadControlSdkMove` 前面 | 头部要等 SDK 发送循环返回才下发；阶段 1 非阻塞等待不产生这种延迟 |

---

## 八、下一步

- [08-头手躯干串行-instructional_case_03](08-头手躯干串行-instructional_case_03.md)：每阶段依次下发头部、躯干 4D 位姿和双臂 14D 关节目标
- [04-预期输出](04-预期输出.md)：成功与失败的输出形态
