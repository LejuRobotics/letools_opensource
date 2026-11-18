# 08 头手躯干串行-instructional_case_03

[返回文档首页](../README.md) · [04-简单案例运行](../04-简单案例运行/) · [03-接口详细文档](../03-接口详细文档/)

本文中的相对路径一律相对 **LeTools 仓库根目录**；命令需在仓库根目录执行。

**适合读者**：已了解 [07-手臂与头部并行](07-手臂与头部并行-instructional_case_02.md)，想用现有节点分别控制躯干位姿和双臂关节的人。

**核心概念**：阶段内用带记忆的 `Sequence` 依次下发头部、躯干和手臂目标；区分躯干 4D **位姿**与手臂 14D **关节角**，以及定时等待与实测到位。

**学习目标**：

1. 看懂四个阶段、每阶段三个叶子节点的串行结构；
2. 会修改 planner `2` 的 `[x, z, yaw, pitch]` 与独立的 14D 双臂目标；
3. 理解模式准备、等待与返回值的边界。

**进阶路线**：[07](07-手臂与头部并行-instructional_case_02.md)（头部 + 双臂并行）→ **本篇**（头部 → 躯干 → 双臂串行）→ [09](09-头手躯干与底盘并行-instructional_case_04.md)（独立的多 planner 并行与 JiBot 示例）。

---

## 一、场景定位

| 维度 | 值 |
|---|---|
| 场景目录 | `orchestration/scenarios/instructional_case_03/` |
| 主树 | 根 `Sequence(four_head_arm_torso_phases)`，共 **17 个节点**：5 个 `Sequence` + 12 个叶子 |
| 子树 | `py_tree_child.json` 为空集合 |
| 黑板 | `board.json`：三组躯干位姿命令、三组双臂关节目标、头部角度及两项时长，共 12 个键 |
| 硬件配置 | 无 `hardware_config.json`，使用 `config/app_config.yaml` 的全局配置 |
| 运动部位 | 头部、躯干、双臂；无底盘任务 |

## 二、树结构与执行边界

```text
Sequence(four_head_arm_torso_phases)  memory=true
├── Sequence(phase_1_home)          头回正 → 躯干 home → 双臂 home
├── Sequence(phase_2_turn_left)     头左转 → 躯干 left → 双臂 left
├── Sequence(phase_3_turn_right)    头右转 → 躯干 right → 双臂 right
└── Sequence(phase_4_return_home)   头回正 → 躯干 home → 双臂 home
```

每个阶段也是 `memory=true`，固定按以下顺序执行：

```text
HeadControlSdkMove     下发头部 yaw/pitch，不查询到位
  → MultiTimedCommandMove  仅 planner 2 躯干位姿，请求成功后定时等待
  → ArmJointTimedMove     14D 双臂关节目标，请求成功后定时等待
```

躯干节点成功后才执行双臂节点，不用 `Parallel` 或 `Async` 把两个 TimedCmd 请求并发。
`ArmJointTimedMove` 经 `send_arm_joint_timed()` 将 14D 目标提交给双臂关节规划器；
它不是 SDK 多关键点轨迹节点。

两个 TimedCmd 技能都等待服务返回的 `actual_time + settle_time`（无 `actual_time` 时回退到请求时长），
没有读取实际关节或位姿误差。头部只是下发目标，因此它的物理运动仍可能与躯干重叠。
**串行是节点/请求顺序，不是三部位都经传感器确认到位后才切换。**

## 三、参数与单位

### 3.1 躯干只用 planner 2

`torso_*_commands` 各只有一条命令。例如左向目标：

```json
[{"planner_index": 2, "cmd_vec": [0.16, 1.10, 30.0, 0.0]}]
```

`cmd_vec` 格式为 **`[x, z, yaw, pitch]`**。前两项是米，后两项按当前默认
Adapter `angle_unit=deg` 使用度，不是四个关节角。节点校验 planner 编号、维度、
数值有限性与时长，但不检查该位姿在实际机器人上是否可达或无碰撞。

### 3.2 手臂目标独立配置

`arm_home_joint_angles`、`arm_left_joint_angles`、`arm_right_joint_angles`
各为 14 个角度，顺序是 `[左臂 J0–J6, 右臂 J0–J6]`，传给 `ArmJointTimedMove.joint_angles`。
本案例不把手臂目标放进躯干命令组，也不添加新的动作节点或技能。

| 阶段 | 躯干 board 键 | 手臂 board 键 | 头部 `[yaw, pitch]`（度） | 躯干 `[x, z, yaw, pitch]`（米、米、度、度） | 左/右臂各自 `[J0..J6]`（度） |
|---|---|---|---|---|---|
| 1：home | `torso_home_commands` | `arm_home_joint_angles` | `[0, 0]` | `[0.12, 0.97, 0, 0]` | `[0, 0, 0, 0, 0, 0, 0]` |
| 2：left | `torso_left_commands` | `arm_left_joint_angles` | `[20, 0]` | `[0.16, 1.10, 30, 0]` | `[0, 0, 0, -90, 0, 30, 0]` |
| 3：right | `torso_right_commands` | `arm_right_joint_angles` | `[-20, 0]` | `[0.16, 1.10, -30, 0]` | `[0, 0, 0, -90, 0, -30, 0]` |
| 4：home | `torso_home_commands` | `arm_home_joint_angles` | `[0, 0]` | `[0.12, 0.97, 0, 0]` | `[0, 0, 0, 0, 0, 0, 0]` |

`home` 是配置的基准位姿，不是躯干全零或专用复位。阶段 2 后直接进入阶段 3。
上述值与当前 `board.json` 一致；头部/手臂及躯干角度均按度填写，不能只改 Adapter 单位而保留这些数值。

### 3.3 时间、模式与焦点

| 配置 | 当前值 | 作用 |
|---|---|---|
| `motion_duration_sec` | `3.0` | 分别传给躯干 `desire_time` 和手臂 `total_time`，每阶段是两段运动请求 |
| `motion_settle_time_sec` | `0.5` | 躯干与手臂各自额外等待 |
| 躯干节点 `is_sync` | `true` | 只作用于当前请求；不能同步后面的独立手臂节点 |
| 躯干节点 `prepare_arm_control` | `true` | 下发前调用 `set_arm_control_mode(2)`，不做物理复位 |
| 躯干节点 `focus_ee` | `false` | 下发前设置 `focus_ee=false`，`focus_z` 同值；不禁止显式躯干运动 |

任一模式或焦点设置失败都不下发该节点的 TimedCmd，但已生效的副作用不自动回滚。
`ArmJointTimedMove` 不自行切模式，本树依赖前面的躯干节点准备模式。
树结束不自动恢复原手臂模式或焦点，也不复位。两段 3 秒请求加各自 0.5 秒等待
只是名义安排，服务返回的规划时间及调用/tick 开销可能更长，不能写成“整个阶段 3 秒到位”。

## 四、修改配置

参数通过 `READ_BOARD` 从 `board.json` 读取。只改姿态或时长不需要改树结构；改完重启行为树。

- 修改躯干：编辑对应 `torso_*_commands[0].cmd_vec`，保持 4D 位姿格式与米/度单位。
- 修改双臂：编辑对应 `arm_*_joint_angles`，保持 14D；左右臂分开核对关节顺序和限位。
- 修改时长：`motion_duration_sec` 影响每阶段两次请求，`motion_settle_time_sec` 影响两次额外等待。
- 多 planner 同请求的例子另见 [case04](09-头手躯干与底盘并行-instructional_case_04.md#四头手和躯干动作)，不要把它误当作本案例当前结构。

用于 [case05 的 01→04 顺序联测](../../orchestration/scenarios/instructional_case_05/README.md) 时，还需手动更新 case05 的 JSON 中对应的子树和带前缀 board 参数；它不依赖同步脚本，也不是运行时直接读取本案例文件。

## 五、运行

### 5.1 干跑

```bash
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/instructional_case_03 \
  --dry-run --tick-once
```

预期关键输出：

```text
[apps][dry-run] 已加载树，根节点: four_head_arm_torso_phases
[apps][dry-run] tick 后根状态: Status.SUCCESS
```

需同时检查场景路径和根状态。当前 dry-run 即使根为 `FAILURE` 也可能退出 0，不能只看退出码。
此检查不连接硬件，不验证可达性、碰撞、模式或实测到位。

### 5.2 仿真或真机

按 [06 第五节](06-头部顺序动作-instructional_case_01.md#52-仿真或真机) 启动控制器与 TF 转发。
确认 ROS、LowLevel SDK、TimedCmd、模式及焦点服务可用，且不存在其他任务同时控制这些部位，再运行：

```bash
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/instructional_case_03
```

**真机前提**：按 [真机运行前安全检查](02-执行命令.md#真机运行前安全检查) 核对位姿可达性、限位与碰撞；
清空头、臂、躯干运动范围，安排急停监护。失败或中断后先确认实际姿态及模式，不要把重新运行当作自动恢复。

## 六、验证方式

| 层次 | 判据与边界 |
|---|---|
| 配置 | 17 个节点、四个阶段均为 `Sequence`；躯干仅 planner `2`、手臂目标各 14D |
| 干跑 | 根节点名称正确且 `tick 后根状态: Status.SUCCESS` |
| 调用顺序 | 每阶段头部请求后为躯干请求及等待，再为双臂请求及等待 |
| 实机效果 | 测试人员结合状态反馈/现场观察确认目标姿态与限位，并记录偏差；根 `SUCCESS`、日志或 `sleep` 结束不替代该项 |
| 收尾 | 人工确认手臂模式与焦点；场景没有自动恢复保证 |

## 七、下一步

- [09-头手躯干与底盘并行-instructional_case_04](09-头手躯干与底盘并行-instructional_case_04.md)：独立使用 planner `3/8/9` 同请求及 JiBot 相对导航，不是本树简单加一个底盘节点。
- [case05：01→04 顺序联测](../../orchestration/scenarios/instructional_case_05/README.md)：测试部门统一入口。
- [控制路径与选型](../03-接口详细文档/09-控制路径与选型.md)：SDK 与 TimedCmd 路径的取舍。
