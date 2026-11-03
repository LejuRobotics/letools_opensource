# 08 头手躯干并行-instructional_case_03

[返回文档首页](../README.md) · [04-简单案例运行](../04-简单案例运行/) · [03-接口详细文档](../03-接口详细文档/)

本文中的相对路径一律相对 **LeTools 仓库根目录**；各命令需在仓库根目录下执行。

**难度**：⭐⭐⭐

**适合读者**：跑通了 [07-手臂与头部并行](07-手臂与头部并行-instructional_case_02.md)，想让手臂和躯干在同一时刻同步完成动作的人。

**前置知识**：`Parallel` 的执行顺序与子节点排列（见 [07](07-手臂与头部并行-instructional_case_02.md)）。

**核心概念**：用 `MultiTimedCommandMove` 在**一次** TimedCmd 请求中提交多个 planner；区分行为树 `Parallel` 的「顺序 tick」与 TimedCmd `is_sync` 的「同步完成」。

**学习目标**：

1. 看懂「头部节点 + 多 planner 节点」的阶段结构；
2. 会查 planner 编号与 `cmd_vec` 格式，自行组合 planner；
3. 理解为什么不能用 `Async` 把手臂和躯干拆成两个线程分别下发。

**由简到难进阶路线**：[07](07-手臂与头部并行-instructional_case_02.md)（头部 + 双臂）→ **本篇**（头部 + 双臂 + 躯干）→ [09](09-头手躯干与底盘并行-instructional_case_04.md)（加入底盘）。

---

## 一、场景定位

| 维度 | 值 |
|------|----|
| 场景目录 | `orchestration/scenarios/instructional_case_03/` |
| 主树 | `py_tree.json`，根 `Sequence(four_head_arm_torso_phases)`，共 **13 个节点** |
| 子树 | `py_tree_child.json` 为空集合，本案例不使用子树 |
| 黑板 | `board.json`：头部角度、三组 TimedCmd 命令、动作时间 |
| 硬件配置 | 无 `hardware_config.json`，使用 `config/app_config.yaml` 的全局配置 |
| 运动部位 | 头部、双臂、下肢 / 躯干 |

---

## 二、树结构

```text
Sequence(four_head_arm_torso_phases)  memory=true
├── Parallel(phase_1_home)          头回正 + 双臂全零 + 躯干基准姿态
├── Parallel(phase_2_turn_left)     头左转 + 双臂左向姿态 + 躯干左转
├── Parallel(phase_3_turn_right)    头右转 + 双臂右向姿态 + 躯干右转
└── Parallel(phase_4_return_home)   头回正 + 双臂全零 + 躯干回基准姿态
```

每个 `Parallel`（`success_on_all`）有两个子节点，头部节点在前：

```text
HeadControlSdkMove       下发头部目标后立即返回
MultiTimedCommandMove    一次 send_timed_multi_commands() 提交 planner 3 / 8 / 9，并等待运动结束
```

本案例的三个 planner：

| planner | 内容 | `cmd_vec` 维度 |
|---:|---|---:|
| `3` | 下肢 / 躯干关节角 | 4 |
| `8` | 左臂关节角 | 7 |
| `9` | 右臂关节角 | 7 |

### 两层「并行」的区别

| 层次 | 机制 | 效果 |
|---|---|---|
| 行为树 `Parallel` | 每拍按子节点顺序依次 tick | 头部节点先下发并返回，头部运动与后面的 TimedCmd 运动重叠 |
| TimedCmd `is_sync=true`（节点默认） | 服务端对同一请求中的多个 planner 统一规划 | planner 3 / 8 / 9 **同时完成** |

`MultiTimedCommandMove` 会等待这组运动结束，所以头部节点必须排在它前面。

### 为什么不用 `Async`

`Async` 只改变行为树的线程调度，不能把多次 TimedCmd 调用合成一次。如果手臂和躯干在两个线程里分别调用 TimedCmd，它们会争用同一个控制器主循环，可能返回 `main loop is updating the command and is busy`。需要多个部位同步运动时，把它们放进同一个 `MultiTimedCommandMove` 的命令组。

---

## 三、`MultiTimedCommandMove` 的配置

### 3.1 命令格式

`commands` 是一个数组，每条命令包含 `planner_index` 和 `cmd_vec`，可选 `desire_time` 覆盖节点的默认执行时间：

```json
[
  {"planner_index": 3, "cmd_vec": [29.0, -41.11, 32.21, 20.0]},
  {"planner_index": 8, "cmd_vec": [0, 0, 0, -90, 0, 30, 0], "desire_time": 4.0},
  {"planner_index": 9, "cmd_vec": [0, 0, 0, -90, 0, 30, 0]}
]
```

节点会校验每个 planner 的向量维度，同一组中不允许重复的 planner。

### 3.2 planner 编号

planner 编号表示运动规划类型，不是关节序号：

| planner | `cmd_vec` 格式 |
|---:|---|
| `0` | 底盘世界坐标系目标 `[x, y, yaw]` |
| `1` | 底盘本体坐标系目标 `[x, y, yaw]` |
| `2` | 躯干局部位姿 `[x, z, yaw, pitch]` |
| `3` | 下肢 / 躯干关节 `[j0, j1, j2, j3]` |
| `4` / `5` | 左 / 右臂世界系末端 `[x, y, z, yaw, pitch, roll]` |
| `6` / `7` | 左 / 右臂局部系末端 `[x, y, z, yaw, pitch, roll]` |
| `8` / `9` | 左 / 右臂 7 个关节角 |

位置单位为米；角度遵循适配器的 `angle_unit` 配置，默认是度。

其他组合示例。躯干位姿 + 双臂关节（`2 / 8 / 9`）：

```json
[
  {"planner_index": 2, "cmd_vec": [0.2, 1.0, 0.0, 0.0]},
  {"planner_index": 8, "cmd_vec": [0, 0, 0, 0, 0, 0, 0]},
  {"planner_index": 9, "cmd_vec": [0, 0, 0, 0, 0, 0, 0]}
]
```

下肢关节 + 双臂局部末端位姿（`3 / 6 / 7`）：

```json
[
  {"planner_index": 3, "cmd_vec": [0, 0, 0, 0]},
  {"planner_index": 6, "cmd_vec": [0.3, 0.2, 0.1, 0, 0, 0]},
  {"planner_index": 7, "cmd_vec": [0.3, -0.2, 0.1, 0, 0, 0]}
]
```

### 3.3 其他节点参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `desire_time` | `3.0` | 默认执行时间（秒），本案例从黑板 `motion_duration_sec` 读取 |
| `is_sync` | `true` | 各 planner 同时完成 |
| `focus_ee` | `false` | `true`：专注手臂末端解算，躯干可能随动；`false`：尽量不动躯干，在手臂工作区间内解算，精度可能不高。`focus_z` 自动设为相同值 |
| `prepare_arm_control` | `true` | 命令中含 planner ≥ 2 时，下发前先把手臂控制模式切到外部控制 |
| `settle_time` | `0.5` | 运动结束后的额外等待（秒） |

任一焦点或模式设置失败时，本次运动命令不会下发。干跑不调用硬件。

---

## 四、当前动作参数

| 阶段 | 黑板命令组 | 头部 `[yaw, pitch]` | 左 / 右臂 `[J0..J6]` | 下肢 / 躯干 `[j0..j3]` |
|---|---|---|---|---|
| 1：初始归零 | `home_commands` | `[0, 0]` | `[0, 0, 0, 0, 0, 0, 0]` | `[0, 0, 0, 0]` |
| 2：向左 | `forward_and_left_commands` | `[20, 0]` | `[0, 0, 0, -90, 0, 30, 0]` | `[29.00, -41.11, 32.21, 20]` |
| 3：向右 | `forward_and_right_commands` | `[-20, 0]` | `[0, 0, 0, -90, 0, -30, 0]` | `[29.00, -41.11, 32.21, -20]` |
| 4：回零 | `home_commands` | `[0, 0]` | `[0, 0, 0, 0, 0, 0, 0]` | `[0, 0, 0, 0]` |

角度单位均为度，TimedCmd 期望时间为 3 秒（`motion_duration_sec`）。阶段 2 完成后直接执行阶段 3，中间不回零。修改动作或 planner 组合只需编辑 `board.json` 中的三组命令，改完重启行为树生效。

---

## 五、运行

### 5.1 干跑

```bash
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/instructional_case_03 \
  --dry-run --tick-once
```

预期输出（关键行）：

```text
[apps][dry-run] 已加载树，根节点: four_head_arm_torso_phases
[apps][dry-run] tick 后根状态: Status.SUCCESS
```

### 5.2 仿真或真机

按 [06 第五节](06-头部顺序动作-instructional_case_01.md#52-仿真或真机) 的三终端方式启动控制器和 TF 转发，在终端 3 运行：

```bash
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/instructional_case_03
```

> ⚠️ **真机**：运行前按 [02-执行命令 · 真机运行前安全检查](02-执行命令.md#真机运行前安全检查) 逐项确认。本案例躯干会升降并左右转动，确认头部、双臂和躯干的完整运动范围内没有人员、线缆或障碍物。

---

## 六、验证方式

| 层次 | 判据 |
|------|------|
| 加载 | 干跑打印 `根节点: four_head_arm_torso_phases` |
| 单拍 tick | `tick 后根状态: Status.SUCCESS` |
| 仿真 / 真机 | 每阶段双臂与躯干同时到位，头部与之重叠运动；进程退出码为 `0`，日志无 `ERROR` |
| 常见失败 | 日志出现维度错误或重复 planner 的报错时，检查 `board.json` 中对应命令组的 `cmd_vec` 长度与 `planner_index` |

---

## 七、动手改一改

| 改什么 | 怎么改 | 预期现象 |
|---|---|---|
| 放慢动作 | `motion_duration_sec` 改为 `5.0` | 每阶段手臂和躯干用 5 秒同步到位 |
| 改手臂姿态 | 把 `forward_and_left_commands` 中 planner `8`、`9` 的 J5 从 `30` 改为 `45` | 阶段 2 的双臂姿态变化，躯干动作不变 |
| 换 planner 组合 | 把 `forward_and_left_commands` 改为 3.2 中的 `2 / 8 / 9` 示例 | 躯干改为按位姿控制 |

---

## 八、下一步

- [09-头手躯干与底盘并行-instructional_case_04](09-头手躯干与底盘并行-instructional_case_04.md)：在同一结构上加入 JiBot 底盘旋转
- [`09-控制路径与选型`](../03-接口详细文档/09-控制路径与选型.md)：标准接口、SDK 直调、TimedCmd 三条控制路径的取舍
