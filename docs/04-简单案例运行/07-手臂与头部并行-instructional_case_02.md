# 07 手臂与头部并行-instructional_case_02

[返回文档首页](../README.md) · [04-简单案例运行](../04-简单案例运行/) · [03-接口详细文档](../03-接口详细文档/)

本文中的相对路径一律相对 **LeTools 仓库根目录**；各命令需在仓库根目录下执行。

**难度**：⭐⭐

**适合读者**：跑通了 [06-头部顺序动作](06-头部顺序动作-instructional_case_01.md)，想让多个部位同时动作的人。

**前置知识**：`Sequence`、`memory`、`READ_BOARD`（见 [06](06-头部顺序动作-instructional_case_01.md)）。

**核心概念**：用 `Parallel` 组合头部和手臂动作；`Parallel` 按顺序 tick 子节点，子节点的排列顺序决定物理动作能否重叠；用 TimedCmd 让手臂平滑运动到单个关节目标。

**学习目标**：

1. 看懂「`Sequence` 套 5 个 `Parallel`」的阶段式结构；
2. 理解为什么头部节点必须排在手臂节点前面；
3. 会用 `board.json` 中的 14 维数组修改双臂姿态。

**由简到难进阶路线**：[06](06-头部顺序动作-instructional_case_01.md)（头部顺序动作）→ **本篇**（头部 + 双臂）→ [08](08-头手躯干并行-instructional_case_03.md)（加入躯干，多 planner 同步）→ [09](09-头手躯干与底盘并行-instructional_case_04.md)（加入底盘）。

---

## 一、场景定位

| 维度 | 值 |
|------|----|
| 场景目录 | `orchestration/scenarios/instructional_case_02/` |
| 主树 | `py_tree.json`，根 `Sequence(five_arm_head_phases)`，共 **16 个节点** |
| 子树 | `py_tree_child.json` 为空集合，本案例不使用子树 |
| 黑板 | `board.json`：四组手臂关节目标、头部角度、动作时间 |
| 硬件配置 | 无 `hardware_config.json`，使用 `config/app_config.yaml` 的全局配置（默认会初始化本案例需要的 TimedCmd 与 LowLevel SDK 管理器） |
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
├── HeadControlSdkMove   下发头部目标后立即返回
└── ArmJointTimedMove    下发手臂目标并等待运动完成
```

### 为什么头部要排在前面

`Parallel` 不是多线程，而是在每一拍里按顺序 tick 子节点：

1. `HeadControlSdkMove` 先下发头部目标并返回，头部开始运动；
2. `ArmJointTimedMove` 随后下发手臂目标并等待手臂运动完成。

这样头部和手臂的物理动作可以重叠。如果把手臂节点放在前面，它的等待会阻塞本拍，头部命令要等手臂动作结束后才下发。

`success_on_all` 表示两个子节点都成功后本阶段才成功；任一子节点失败，本阶段和外层 `Sequence` 都会失败。本案例不需要用 `Async` 包裹。

---

## 三、手臂目标

`ArmJointTimedMove` 调用 `hardware.send_arm_joint_timed()`，由 TimedCmd 从当前位置平滑规划到**单个** 14 维关节目标，不使用多关键点轨迹。

- 关节顺序：`[左臂 J0–J6, 右臂 J0–J6]`；
- 单位：度（随适配器的 `angle_unit` 配置，默认度）；
- 每阶段期望执行时间：3 秒（`arm_motion_duration_sec`）。

| 阶段 | 手臂目标 | 头部目标 |
|---|---|---|
| 1 | 双臂全零 | 低头：`yaw=0°`，`pitch=20°` |
| 2 | 双臂 J0=`-30°`，其余 `0°`（前伸） | 左转：`yaw=20°`，`pitch=0°` |
| 3 | 双臂 J3=`-90°`，其余 `0°`（上举） | 抬头：`yaw=0°`，`pitch=-20°` |
| 4 | 左臂 J1=`90°`、右臂 J1=`-90°`，其余 `0°`（展开） | 右转：`yaw=-20°`，`pitch=0°` |
| 5 | 双臂全零 | 正中：`yaw=0°`，`pitch=0°` |

---

## 四、参数从 `board.json` 读取

| 黑板键 | 内容 |
|---|---|
| `arm_zero_joint_angles` | 双臂全零，阶段 1 和阶段 5 共用 |
| `arm_forward_joint_angles` | 双臂前伸 |
| `arm_up_joint_angles` | 双臂上举 |
| `arm_spread_joint_angles` | 双臂展开 |
| `arm_motion_duration_sec` | 每阶段手臂动作的期望时间，默认 `3.0` 秒 |
| `head_left_yaw_deg` / `head_right_yaw_deg` | 左右转 yaw |
| `head_up_pitch_deg` / `head_down_pitch_deg` | 抬头和低头 pitch |
| `head_center_yaw_deg` / `head_center_pitch_deg` | 正中角度，也用于补齐单轴动作 |

手臂目标在 `board.json` 中是原生 JSON 数组，主树通过 `READ_BOARD` 把它传给 `ArmJointTimedMove`：

```json
"joint_angles": {
  "source": "READ_BOARD",
  "board_key": "arm_forward_joint_angles"
}
```

修改姿态或动作时间只需编辑 `board.json`，每组手臂数组必须**严格为 14 个元素**；改完重启行为树生效。

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

### 5.2 仿真或真机

按 [06 第五节](06-头部顺序动作-instructional_case_01.md#52-仿真或真机) 的三终端方式启动控制器和 TF 转发，在终端 3 运行：

```bash
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/instructional_case_02
```

> ⚠️ **真机**：运行前按 [02-执行命令 · 真机运行前安全检查](02-执行命令.md#真机运行前安全检查) 逐项确认。本案例双臂会前伸、上举和侧向展开 90°，确认双臂和头部的完整运动范围内没有人员、线缆或其他障碍物。

---

## 六、验证方式

| 层次 | 判据 |
|------|------|
| 加载 | 干跑打印 `根节点: five_arm_head_phases` |
| 单拍 tick | `tick 后根状态: Status.SUCCESS` |
| 仿真 / 真机 | 五个阶段依次执行，每阶段头部与手臂同时开始运动；进程退出码为 `0`，日志无 `ERROR` |

---

## 七、动手改一改

| 改什么 | 怎么改 | 预期现象 |
|---|---|---|
| 手臂动得更慢 | `arm_motion_duration_sec` 改为 `5.0` | 每阶段手臂用 5 秒到位 |
| 换一个姿态 | 修改 `arm_forward_joint_angles` 中的元素（保持 14 个） | 阶段 2 的手臂姿态变化 |
| 观察子节点顺序的影响 | 在某个 `Parallel` 中把 `ArmJointTimedMove` 移到 `HeadControlSdkMove` 前面 | 该阶段头部要等手臂动作结束才开始转动 |

---

## 八、下一步

- [08-头手躯干并行-instructional_case_03](08-头手躯干并行-instructional_case_03.md)：加入躯干，用一次 TimedCmd 请求让多个部位同步完成
- [04-预期输出](04-预期输出.md)：成功与失败的输出形态
