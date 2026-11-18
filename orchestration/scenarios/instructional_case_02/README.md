# instructional_case_02：手臂与头部并行动作

本案例在 `case_01` 的顺序树基础上加入双臂控制，演示如何把五个
`Parallel` 组合成一个 `Sequence`。每个阶段先下发头部目标，再执行一组手臂
关节动作。

所有命令均需在仓库根目录执行。建议先执行 dry-run 检查配置，再连接真机。

## 树结构

```text
Sequence
├── Parallel：双臂全零   + 低头
├── Parallel：双臂前伸   + 左转
├── Parallel：双臂上举   + 抬头
├── Parallel：双臂展开   + 右转
└── Parallel：双臂回全零 + 头部正中
```

每个 `Parallel` 使用 `success_on_all`，子节点顺序固定为头部在前、手臂在后。
`Parallel` 本身不是多线程，而是按顺序 tick 子节点。`HeadControlSdkMove`
快速下发目标角后返回，随后手臂节点执行定时等待或 SDK 轨迹发送循环；此时
头部和手臂的物理动作可以重叠，但节点不检测两者是否实测到位。如果把手臂节点放在
前面，它的等待会阻塞本次 tick，导致头部命令延后。

`success_on_all` 表示头部和手臂节点都成功后，本阶段才成功；任一节点失败，
本阶段及外层 `Sequence` 都会失败。这里不需要 `Async` 包裹。

## 手臂目标

全树共 16 个节点：1 个 `Sequence`、5 个 `Parallel`、10 个叶子节点。
关节顺序为 `[左臂 J0-J6, 右臂 J0-J6]`，数值按默认 Adapter `angle_unit=deg`
使用度；`arm_motion_duration_sec=3` 是规划期望/轨迹发送时长，不是实测到位保证。

| 阶段 | 手臂节点 | 手臂目标 | 头部目标 |
|---|---|---|---|
| 1 | `ArmJointTimedMove` | `[0, 0, 0, 0, 0, 0, 0] × 2` | 低头：`yaw=0°`, `pitch=20°` |
| 2 | `ArmJointTrajSdkMove` | `[-30, 0, 0, 0, 0, 0, 0] × 2` | 左转：`yaw=20°`, `pitch=0°` |
| 3 | `ArmJointTrajSdkMove` | `[0, 0, 0, -90, 0, 0, 0] × 2` | 抬头：`yaw=0°`, `pitch=-20°` |
| 4 | `ArmJointTrajSdkMove` | 左臂 J1=`90°`，右臂 J1=`-90°`，其余为 `0°` | 右转：`yaw=-20°`, `pitch=0°` |
| 5 | `ArmJointTrajSdkMove` | `[0, 0, 0, 0, 0, 0, 0] × 2` | 正中：`yaw=0°`, `pitch=0°` |

本案例用到两种手臂关节接口：

| 节点 | 底层调用 | 插值位置 | 输入 |
|---|---|---|---|
| `ArmJointTimedMove` | `hardware.send_arm_joint_timed()` | TimedCmd 服务端，从**当前位置**规划到目标 | 单个 14 维目标 |
| `ArmJointTrajSdkMove` | `hardware.send_arm_joint_traj_sdk()` | 客户端，50 Hz 线性插值后逐帧下发 | 至少两个 14 维关键点 |

`ArmJointTrajSdkMove` 不读取手臂当前位置，只在给定的关键点之间插值，所以：

- 轨迹的**第一个点必须等于手臂当前姿态**，否则手臂会从当前位置跳到第一个点；
- 轨迹至少要有两个点。只给一个点时接口只打印告警、手臂不动，节点仍返回成功。

因此阶段 2 至 5 的轨迹都写成 `[上一阶段姿态, 本阶段姿态]`。阶段 1 开始时手臂
姿态未知，所以使用 `ArmJointTimedMove` 请求 TimedCmd 从当前位置规划回零。
技能只等待服务返回的 `actual_time`（无该值时使用 3 秒），未配置额外 `settle_time`
（默认 0）；不检查回零误差。首尾关键点一致只保证配置连续，不能保证实机已到达起点，
也不能保证从任意姿态回零都安全。

`ArmJointTrajSdkMove` 每次执行前设置手臂外部控制模式 2、MPC `ArmOnly` 与
`DirectToWbc` 控制流，结束时尝试恢复 MPC `NoControl` / `ThroughFullBodyMpc`，
前后各等待 0.5 秒。它不恢复启动前的手臂模式，也不切回模式 0 或物理复位；
恢复结果也不等同于实测确认。阶段 2 至 5 除 3 秒发送时长外还有约 1 秒模式等待及
调用开销，不应按精确 3 秒验收到位。

本案例使用 `config/app_config.yaml` 的全局配置。默认配置会初始化本案例需要的
TimedCmd、Arm SDK 和 LowLevel SDK manager，但初始化请求不代表环境就绪。
**阶段 1 的 `ArmJointTimedMove` 不自行切换手臂模式**：真机前需确认处于允许 TimedCmd
接管的外部控制模式，且控制器、ROS/TF、SDK 与模式服务可用；后续 SDK 阶段的模式切换
不能替代阶段 1 的前置条件。环境准备见 [执行命令](../../../docs/04-简单案例运行/02-执行命令.md)。

## 从 board 加载参数

手臂目标和头部角度集中放在 `board.json`。手臂目标是原生 JSON 数组，加载后
仍是列表：

| board 键 | 内容 |
|---|---|
| `arm_zero_joint_angles` | 双臂全零，单个 14 维目标，阶段 1 使用 |
| `arm_zero_to_forward_traj` | 阶段 2 轨迹：全零 → 前伸 |
| `arm_forward_to_up_traj` | 阶段 3 轨迹：前伸 → 上举 |
| `arm_up_to_spread_traj` | 阶段 4 轨迹：上举 → 展开 |
| `arm_spread_to_zero_traj` | 阶段 5 轨迹：展开 → 全零 |
| `arm_motion_duration_sec` | 每阶段手臂动作时间，默认 3 秒 |
| `head_left_yaw_deg` / `head_right_yaw_deg` | 左右转 yaw |
| `head_up_pitch_deg` / `head_down_pitch_deg` | 抬头和低头 pitch |
| `head_center_yaw_deg` / `head_center_pitch_deg` | 正中角度，也用于补齐单轴动作 |

阶段 1 通过 `READ_BOARD` 把单个目标传给 `ArmJointTimedMove`：

```json
"joint_angles": {
  "source": "READ_BOARD",
  "board_key": "arm_zero_joint_angles"
}
```

阶段 2 至 5 把轨迹传给 `ArmJointTrajSdkMove`。`use_board_trajectory` 必须设为
`false`，节点才会使用 `joint_traj` 参数，而不是去读黑板上固定的
`ArmJointTrajectories` 键：

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

头部参数使用相同方式。例如左转阶段：

```json
"yaw_deg": {
  "source": "READ_BOARD",
  "board_key": "head_left_yaw_deg"
}
```

修改姿态或动作时间时只需编辑 `board.json`，重启行为树后生效。修改时保持：

- 每个关键点严格为 14 个元素；
- 相邻阶段首尾相接：每条轨迹的第一个点等于上一阶段的最后一个点，阶段 2 的
  第一个点等于 `arm_zero_joint_angles`。

dry-run 会检查每个关键点是否为 14 个元素，但不检查首尾是否相接，也不检查轨迹
是否至少有两个点。运行前需人工核对后两项；不要用真机试验不连续的轨迹。

## 运行

先做无 ROS、不会驱动机器人的结构校验：

```bash
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/instructional_case_02 \
  --dry-run --tick-once
```

确认输出中加载的是 `instructional_case_02/py_tree.json`，且显示
`tick 后根状态: Status.SUCCESS`。当前启动器在 dry-run 根状态为 `Status.FAILURE`
时仍退出 `0`，不能只凭退出码判断通过。

完成配置与运动范围检查后，再进行真机运行：

```bash
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/instructional_case_02
```

实机运行前确认双臂和头部的完整运动范围内没有人员、线缆或其他障碍物。

阶段 2 至 5 每次开始时，日志里可能出现下面两条 `WARNING`。它们来自 MPC 控制
模式的设置，不能仅凭这两条告警断言轨迹已执行或控制流正确；需结合其余错误、
控制状态和实际反馈判断，尤其第二条说明服务不可用、仅更新本地状态：

```text
[WARNING] ...base_sdk_manager: set_mpc_mode: mode 在当前固件构建下无实际控制效果（仅 flow 生效），详见方法 docstring
[WARNING] kuavo-humanoid-sdk: [Core] Manipulation MPC control mode service not available, updating local stateonly
```

## 文件

| 文件 | 用途 |
|---|---|
| `py_tree.json` | 五阶段主行为树，通过 `READ_BOARD` 引用动作参数 |
| `py_tree_child.json` | 空子树集合，本案例不使用子树 |
| `board.json` | 手臂回零目标、四条手臂关节轨迹、头部角度和动作时间 |
