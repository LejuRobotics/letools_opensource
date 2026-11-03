# 09 头手躯干与底盘并行-instructional_case_04

[返回文档首页](../README.md) · [04-简单案例运行](../04-简单案例运行/) · [03-接口详细文档](../03-接口详细文档/)

本文中的相对路径一律相对 **LeTools 仓库根目录**；各命令需在仓库根目录下执行。

**难度**：⭐⭐⭐

**适合读者**：跑通了 [08-头手躯干并行](08-头手躯干并行-instructional_case_03.md)，想把底盘运动编排进同一棵树、并在真机 JiBot 底盘上运行的人。

**前置知识**：`Parallel` 的执行顺序、`MultiTimedCommandMove`（见 [08](08-头手躯干并行-instructional_case_03.md)）。

**核心概念**：用一个持续 `RUNNING` 的守护节点管理底盘控制权；`success_on_one` 让主序列结束时自动终止守护节点；区分 JiBot 相对导航与 TimedCmd 底盘 planner。

**学习目标**：

1. 看懂「控制权守护 + 动作序列」的外层结构；
2. 理解底盘节点为什么要排在 `MultiTimedCommandMove` 前面；
3. 知道「只提交不等待」的底盘节点依赖时间窗口，以及时间窗口不够时该怎么做。

**由简到难进阶路线**：[08](08-头手躯干并行-instructional_case_03.md)（头部 + 双臂 + 躯干）→ **本篇**（加入底盘）→ [10](10-搬箱场景-grasp_mtbf.md)（导航、识别、抓取、放置的完整链路）。

---

## 一、场景定位

| 维度 | 值 |
|------|----|
| 场景目录 | `orchestration/scenarios/instructional_case_04/` |
| 主树 | `py_tree.json`，根 `Parallel(jibot_control_guard_and_motion)`，共 **18 个节点** |
| 子树 | `py_tree_child.json` 为空集合，本案例不使用子树 |
| 黑板 | `board.json`：沿用 [08](08-头手躯干并行-instructional_case_03.md) 的头部与 TimedCmd 参数，另加底盘参数 |
| 硬件配置 | 无 `hardware_config.json`；JiBot 由 `config/app_config.yaml` 中的 `navigation_chassis_backend: jibot` 选择 |
| 运动部位 | 头部、双臂、下肢 / 躯干、JiBot 底盘 |
| 运行环境 | 干跑、**真机（JiBot 底盘）**。MuJoCo 仿真中没有 JiBot 导航服务，底盘节点无法执行 |

---

## 二、树结构

```text
Parallel(jibot_control_guard_and_motion)  policy=success_on_one
├── NodeEnableVelControl  give_control_to_jibot_navigation   把底盘控制权交给 JiBot 导航，结束时恢复
└── Sequence(four_head_arm_torso_chassis_phases)  memory=true
    ├── Parallel(phase_1_home)         头回正 + 双臂 / 躯干归零
    ├── Parallel(phase_2_turn_left)    头左转 + JiBot 相对左转 + 双臂 / 躯干左向姿态
    ├── Parallel(phase_3_turn_right)   头右转 + JiBot 相对右转 + 双臂 / 躯干右向姿态
    └── Parallel(phase_4_return_home)  头回正 + JiBot 相对回正 + 双臂 / 躯干回零
```

### 外层：控制权守护

- `NodeEnableVelControl(enable=false)` 在动作期间把底盘速度控制权交给 JiBot 导航模块，并一直保持 `RUNNING`；
- 外层 `Parallel` 使用 `success_on_one`：四阶段序列完成后根节点成功，同时终止守护节点；`restore_on_stop=true` 使守护节点在被终止时恢复运行前的控制权；
- 如果守护节点执行失败，根节点也会失败，不会把序列成功误判为整个案例成功。

### 阶段内：子节点顺序

有底盘动作的阶段按以下顺序排列：

```text
HeadControlSdkMove           下发头部目标后立即返回
BaseMoveRelativeJibotMove    调用 /move_base/base_move 提交底盘任务后立即返回
MultiTimedCommandMove        提交手臂与躯干目标，并等待规划运动结束
```

`Parallel` 依次 tick 子节点，所以底盘实际运动会与手臂、躯干运动重叠。如果把底盘节点放在 `MultiTimedCommandMove` 后面，TimedCmd 的等待会让底盘任务延后下发。

---

## 三、JiBot 相对运动

`BaseMoveRelativeJibotMove` 底层调用 `hardware.base_move_relative_jibot()`：`x`、`y` 是相对当前本体坐标系的位移（米），`theta` 是相对当前朝向的旋转量（**弧度**，不受 `angle_unit` 影响）。

| 阶段 | 相对 `theta`（rad） | 约合角度 | 累计朝向 |
|---|---:|---:|---:|
| 1：初始归零 | 不下发底盘任务 | 0° | 0° |
| 2：向左 | `+0.349066` | +20° | +20° |
| 3：向右 | `-0.698132` | -40° | -20° |
| 4：回正 | `+0.349066` | +20° | 0° |

三次旋转的增量之和为零，正常完成后底盘回到起始朝向。本案例只原地旋转（`x = y = 0`）。

底盘参数（`board.json`）：

| 黑板键 | 默认值 | 说明 |
|---|---:|---|
| `base_relative_x_m` / `base_relative_y_m` | `0.0` | 相对位移（米） |
| `base_turn_left_rad` / `base_turn_right_rad` / `base_return_center_rad` | `0.349066` / `-0.698132` / `0.349066` | 三个阶段的相对旋转量 |
| `base_angular_velocity_rad_s` | `0.35` | 角速度（rad/s） |
| `base_angle_threshold_rad` | `0.05` | 角度到达阈值（rad） |

### 时间窗口的限制

`BaseMoveRelativeJibotMove` **只提交任务，不查询是否到达**。本案例把最大一段旋转（40°，约 0.7 rad）控制在约 2 秒内完成，依靠同一阶段 `MultiTimedCommandMove` 约 3.5 秒的等待（3 秒执行 + 0.5 秒 `settle_time`）让底盘转完。

如果调大旋转量或调低角速度，底盘可能在下一阶段开始时还没转完。这种情况应改用带任务 ID 和到达检查的编排：提交后用 `CheckArrivedJibotMove` 等待到达，参数见 [`11-节点清单`](../03-接口详细文档/11-节点清单.md)。

### 不要和 TimedCmd 底盘 planner 混淆

| 接口 | 是否使用 planner 编号 | 运动含义 | 角度单位 |
|---|---|---|---|
| `BaseMoveRelativeJibotMove` | 否 | JiBot 相对当前位姿运动 | 弧度 |
| `MultiTimedCommandMove` 的 planner `0` | 是 | TimedCmd 世界坐标系底盘目标 | 随 `angle_unit`，默认度 |
| `MultiTimedCommandMove` 的 planner `1` | 是 | TimedCmd 本体坐标系底盘目标 | 随 `angle_unit`，默认度 |

本案例的底盘动作走 JiBot 接口，没有使用 planner `0` / `1`。

---

## 四、头、手和躯干动作

与 [08](08-头手躯干并行-instructional_case_03.md) 相同：`MultiTimedCommandMove` 在一次请求中同步提交 planner `3`（下肢 / 躯干）、`8`（左臂）、`9`（右臂）。

| 阶段 | 头部 `[yaw, pitch]`（度） | 左 / 右臂 `[J0..J6]`（度） | 下肢 / 躯干 `[j0..j3]`（度） |
|---|---|---|---|
| 1 | `[0, 0]` | `[0, 0, 0, 0, 0, 0, 0]` | `[0, 0, 0, 0]` |
| 2 | `[20, 0]` | `[0, 0, 0, -90, 0, 30, 0]` | `[29.00, -41.11, 32.21, 20]` |
| 3 | `[-20, 0]` | `[0, 0, 0, -90, 0, -30, 0]` | `[29.00, -41.11, 32.21, -20]` |
| 4 | `[0, 0]` | `[0, 0, 0, 0, 0, 0, 0]` | `[0, 0, 0, 0]` |

---

## 五、运行

### 5.1 干跑

```bash
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/instructional_case_04 \
  --dry-run --tick-once
```

预期输出（关键行）：

```text
[apps][dry-run] 已加载树，根节点: jibot_control_guard_and_motion
[apps][dry-run] tick 后根状态: Status.SUCCESS
```

### 5.2 真机

1. 确认 `config/app_config.yaml` 中 `navigation_chassis_backend` 为 `jibot`，且 JiBot 导航服务可用：`rosservice list | grep move_base`；
2. 下位机启动真机控制器，宿主机按 [02-执行命令 · 运行前环境准备](02-执行命令.md#运行前环境准备仿真与真机共用) 准备终端，并确认 TF 转发已启动；
3. 运行：

```bash
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/instructional_case_04
```

> ⚠️ **真机**：运行前按 [02-执行命令 · 真机运行前安全检查](02-执行命令.md#真机运行前安全检查) 逐项确认。本案例底盘会原地左右旋转，同时头部、双臂和躯干运动；`BaseMoveRelativeJibotMove` 的避障默认关闭，确认机器人四周以及头部、双臂和躯干的完整运动范围内没有人员、线缆或障碍物。

---

## 六、验证方式

| 层次 | 判据 |
|------|------|
| 加载 | 干跑打印 `根节点: jibot_control_guard_and_motion` |
| 单拍 tick | `tick 后根状态: Status.SUCCESS` |
| 真机 | 底盘依次左转、右转、回正并回到起始朝向；日志中出现 JiBot `/move_base/base_move` 的提交信息；进程退出码为 `0` |
| 排查 | 只看到 TimedCmd 日志、看不到 `/move_base/base_move` 时，先确认实际加载的场景路径是 `instructional_case_04` |

---

## 七、动手改一改

| 改什么 | 怎么改 | 注意 |
|---|---|---|
| 转得更慢 | `base_angular_velocity_rad_s` 改为 `0.25` | 40° 一段约需 2.8 秒，仍在约 3.5 秒的时间窗口内 |
| 转得更大 | 三个 `theta` 同比例放大，保持总和为零 | 超出时间窗口时，需改用到达检查（见第三节） |
| 加一点平移 | `base_relative_x_m` 改为非零 | 每阶段都会平移，累计位移不为零；务必确认场地空间 |

---

## 八、下一步

- [10-搬箱场景-grasp_mtbf](10-搬箱场景-grasp_mtbf.md)：导航、识别、抓取、搬运、放置的完整链路
- [`03-adapters-硬件适配层`](../03-接口详细文档/03-adapters-硬件适配层.md)：JiBot 接口与标准导航接口
