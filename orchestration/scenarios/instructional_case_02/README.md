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
快速下发目标角后返回，随后 `ArmJointTimedMove` 开始手臂动作并等待完成；此时
头部已经开始运动，因此头部和手臂的物理动作可以重叠。如果把手臂节点放在
前面，它的等待会阻塞本次 tick，导致头部命令延后。

`success_on_all` 表示头部和手臂节点都成功后，本阶段才成功；任一节点失败，
本阶段及外层 `Sequence` 都会失败。这里不需要 `Async` 包裹。

## 手臂目标

`ArmJointTimedMove` 每次只下发一个 14 维关节目标，不使用多关键点轨迹。
关节顺序为 `[左臂 J0-J6, 右臂 J0-J6]`，单位为度，每阶段期望执行时间为 3 秒。

| 阶段 | 手臂目标 | 头部目标 |
|---|---|---|
| 1 | `[0, 0, 0, 0, 0, 0, 0] × 2` | 低头：`yaw=0°`, `pitch=20°` |
| 2 | `[-30, 0, 0, 0, 0, 0, 0] × 2` | 左转：`yaw=20°`, `pitch=0°` |
| 3 | `[0, 0, 0, -90, 0, 0, 0] × 2` | 抬头：`yaw=0°`, `pitch=-20°` |
| 4 | 左臂 J1=`90°`，右臂 J1=`-90°`，其余为 `0°` | 右转：`yaw=-20°`, `pitch=0°` |
| 5 | `[0, 0, 0, 0, 0, 0, 0] × 2` | 正中：`yaw=0°`, `pitch=0°` |

`ArmJointTimedMove` 调用 `hardware.send_arm_joint_timed()`，由 TimedCmd 从
当前位置平滑规划到单个目标角。Adapter 默认使用角度制。

本案例使用 `config/app_config.yaml` 的全局配置。
默认配置会初始化本案例需要的 TimedCmd 和 LowLevel SDK manager。

## 从 board 加载参数

四组手臂关节角和五个阶段使用的头部角度集中放在 `board.json`。第一、第五阶段
复用同一组全零手臂目标。手臂目标是
原生 JSON 数组，加载后仍是 `List[float]`：

| board 键 | 内容 |
|---|---|
| `arm_zero_joint_angles` | 双臂全零，阶段 1 和阶段 5 共用 |
| `arm_forward_joint_angles` | 双臂前伸 |
| `arm_up_joint_angles` | 双臂上举 |
| `arm_spread_joint_angles` | 双臂展开 |
| `arm_motion_duration_sec` | 每阶段手臂动作期望时间，默认 3 秒 |
| `head_left_yaw_deg` / `head_right_yaw_deg` | 左右转 yaw |
| `head_up_pitch_deg` / `head_down_pitch_deg` | 抬头和低头 pitch |
| `head_center_yaw_deg` / `head_center_pitch_deg` | 正中角度，也用于补齐单轴动作 |

主树通过 `READ_BOARD` 把数组传给 `ArmJointTimedMove`：

```json
"joint_angles": {
  "source": "READ_BOARD",
  "board_key": "arm_forward_joint_angles"
}
```

头部参数使用相同方式。例如左转阶段：

```json
"yaw_deg": {
  "source": "READ_BOARD",
  "board_key": "head_left_yaw_deg"
}
```

修改姿态或动作时间时只需编辑 `board.json`，保持每组手臂数组严格为
14 个元素；重启行为树后生效。

## 运行

先做无 ROS、不会驱动机器人的结构校验：

```bash
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/instructional_case_02 \
  --dry-run --tick-once
```

确认输出中加载的是 `instructional_case_02/py_tree.json` 后，再进行真机运行：

```bash
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/instructional_case_02
```

实机运行前确认双臂和头部的完整运动范围内没有人员、线缆或其他障碍物。

## 文件

| 文件 | 用途 |
|---|---|
| `py_tree.json` | 五阶段主行为树，通过 `READ_BOARD` 引用动作参数 |
| `py_tree_child.json` | 空子树集合，本案例不使用子树 |
| `board.json` | 四组手臂关节目标、头部角度和动作时间 |
