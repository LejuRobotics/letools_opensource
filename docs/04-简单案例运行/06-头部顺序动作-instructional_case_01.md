# 06 头部顺序动作-instructional_case_01

[返回文档首页](../README.md) · [04-简单案例运行](../04-简单案例运行/) · [03-接口详细文档](../03-接口详细文档/)

本文中的相对路径一律相对 **LeTools 仓库根目录**；各命令需在仓库根目录下执行。

**难度**：⭐

**适合读者**：跑通了 [01-最小可运行示例](01-最小可运行示例.md) 和 [03-行为树最小示例](03-行为树最小示例.md)，想第一次让机器人按行为树真正动起来的人。

**前置知识**：知道 `Sequence` 会依次执行子节点；知道 `board.json` 是黑板初始值。

**核心概念**：顺序树 `Sequence`、带记忆执行 `memory=true`、用 `READ_BOARD` 从黑板读取参数、用等待节点控制动作节奏。

**学习目标**：

1. 看懂一棵 11 个节点的顺序树如何让头部依次左右、上下转动；
2. 理解「下发目标即返回」的动作节点为什么要配合等待节点；
3. 只改 `board.json` 就能调整动作幅度和停留时间。

**由简到难进阶路线**：[03](03-行为树最小示例.md)（1 个节点）→ **本篇**（头部顺序动作）→ [07](07-手臂与头部并行-instructional_case_02.md)（加入双臂并行）→ [08](08-头手躯干并行-instructional_case_03.md)（加入躯干，多 planner 同步）→ [09](09-头手躯干与底盘并行-instructional_case_04.md)（加入底盘）。

---

## 一、场景定位

| 维度 | 值 |
|------|----|
| 场景目录 | `orchestration/scenarios/instructional_case_01/` |
| 主树 | `py_tree.json`，根 `Sequence(simple_head_move_sequence)`，共 **11 个节点** |
| 子树 | `py_tree_child.json` 为空集合，本案例不使用子树 |
| 黑板 | `board.json`，7 个键：头部角度与停留时间 |
| 硬件配置 | 无 `hardware_config.json`，使用 `config/app_config.yaml` 的全局配置 |
| 运动部位 | 仅头部 |

---

## 二、树结构

```text
Sequence(simple_head_move_sequence)  memory=true
├── HeadControlSdkMove  turn_head_left                    左转 20°
├── WaitSeconds         hold_left                         停留 1 s
├── HeadControlSdkMove  turn_head_right                   右转 20°
├── WaitSeconds         hold_right                        停留 1 s
├── HeadControlSdkMove  return_head_to_center_after_yaw   回正
├── HeadControlSdkMove  look_up                           抬头 20°
├── WaitSeconds         hold_up                           停留 1 s
├── HeadControlSdkMove  look_down                         低头 20°
├── WaitSeconds         hold_down                         停留 1 s
└── HeadControlSdkMove  return_head_to_center_after_pitch 回正
```

| 节点 | 作用 |
|---|---|
| `HeadControlSdkMove` | 通过 SDK 下发一个头部 yaw / pitch 目标角，**调用成功即返回**，不等待头部实际到达 |
| `WaitSeconds` | 在指定时间内保持 `RUNNING`，让下一个头部目标延后下发 |

因为 `HeadControlSdkMove` 不等待到位，连续下发的目标会立刻覆盖上一个目标。本案例在两个方向之间插入 `WaitSeconds`，留出运动和观察时间。头部转速由底层控制器决定，该节点没有速度参数。

根节点 `memory=true` 表示：某个子节点仍在 `RUNNING` 时，下一拍从这个子节点继续，不会重新执行前面已经成功的头部命令。省略 `memory` 时默认为 `false`，树会每拍从第一个节点重新开始，详见 [`05-orchestration-行为树编排`](../03-接口详细文档/05-orchestration-行为树编排.md)。

---

## 三、角度约定

| 动作 | yaw | pitch |
|---|---:|---:|
| 左转 | `+20°` | `0°` |
| 右转 | `-20°` | `0°` |
| 抬头 | `0°` | `-20°` |
| 低头 | `0°` | `+20°` |
| 回正 | `0°` | `0°` |

参数单位为**度**，适配器内部转换为弧度后下发。SDK 对头部的限位为 yaw ±80°、pitch ±25°，超出时打印警告并截断到限位，不会报错。

---

## 四、参数从 `board.json` 读取

头部角度和停留时间集中放在 `board.json`：

| 黑板键 | 默认值 | 用途 |
|---|---:|---|
| `head_left_yaw_deg` | `20.0` | 左转 yaw |
| `head_right_yaw_deg` | `-20.0` | 右转 yaw |
| `head_up_pitch_deg` | `-20.0` | 抬头 pitch |
| `head_down_pitch_deg` | `20.0` | 低头 pitch |
| `head_center_yaw_deg` | `0.0` | 正中 yaw，上下动作复用 |
| `head_center_pitch_deg` | `0.0` | 正中 pitch，左右动作复用 |
| `head_hold_duration_sec` | `1.0` | 每个方向的停留时间（秒） |

主树通过 `READ_BOARD` 引用这些值，例如左转节点的 yaw：

```json
"yaw_deg": {
  "source": "READ_BOARD",
  "board_key": "head_left_yaw_deg"
}
```

`board.json` 在启动时加载一次。修改动作幅度或节奏只需编辑 `board.json`，不用改树结构；改完**重启行为树**生效。

---

## 五、运行

### 5.1 干跑（不连 ROS、不驱动机器人）

```bash
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/instructional_case_01 \
  --dry-run --tick-once
```

预期输出（关键行）：

```text
[apps] 子树集合已加载：.../instructional_case_01/py_tree_child.json (count=0)
[apps][dry-run] 已加载树，根节点: simple_head_move_sequence
[apps][dry-run] tick 后根状态: Status.SUCCESS
```

干跑时动作节点和等待节点都直接返回 `SUCCESS`，所以一拍即可走完整棵树。判据见 [04-预期输出](04-预期输出.md)。

### 5.2 仿真或真机

仿真和真机都需要 3 个终端：**终端 1** 控制器 → **终端 2** TF 转发 → **终端 3** 行为树。每个新开的宿主机终端都要先完成 [02-执行命令 · 运行前环境准备](02-执行命令.md#运行前环境准备仿真与真机共用)。容器的创建、进入和控制器编译见 [安装步骤 1.1](../02-环境安装/03-安装步骤.md#11-环境部署)。

**终端 1：启动控制器**（最先启动，保持运行）

仿真在 Docker 容器内执行：

```bash
cd /root/kuavo_ws
echo "$ROBOT_VERSION"          # 必须与目标机器人一致，不一致先 export 正确值
source devel/setup.zsh
roslaunch humanoid_controllers load_kuavo_mujoco_sim_wheel.launch
```

真机在下位机执行 `roslaunch humanoid_controllers load_kuavo_real_wheel.launch`，按提示操作后保持运行。

成功标志：仿真出现 MuJoCo 窗口；`rostopic list` 能看到控制器话题。

**终端 2：在宿主机启动 TF 转发**（控制器启动之后）

```bash
cd ~/letools_opensource/infrastructure/ros_packages
source /opt/ros/noetic/setup.bash
source devel/setup.bash
./start_tf_republisher.sh
```

成功标志：打印 `✅ 服务启动成功！`，且 `rosservice list | grep republish_tfs` 有输出。SDK 初始化时会等待该服务，缺少它时 SDK 初始化失败。

**终端 3：在宿主机运行行为树**

```bash
cd ~/letools_opensource
source /opt/ros/noetic/setup.bash
source infrastructure/ros_packages/devel/setup.bash
export PYTHONPATH="$PWD/infrastructure/ros_packages/src/kuavo_common/python:$PWD:$PYTHONPATH"

python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/instructional_case_01
```

> ⚠️ **真机**：运行前按 [02-执行命令 · 真机运行前安全检查](02-执行命令.md#真机运行前安全检查) 逐项确认，并确认头部转动范围内没有障碍物。

---

## 六、验证方式

| 层次 | 判据 |
|------|------|
| 加载 | 干跑打印 `根节点: simple_head_move_sequence`，`子树集合已加载… (count=0)` |
| 单拍 tick | `tick 后根状态: Status.SUCCESS` |
| 仿真 / 真机 | 头部依次左转、右转、回正、抬头、低头、回正；进程退出码为 `0`，日志无 `ERROR` |
| 假成功的识别 | 日志中出现 `SDK 未初始化，请先调用 initialize()` 时，头部动作并未执行，即使行为树显示成功；先检查终端 2 的 TF 转发和终端 3 的 `PYTHONPATH` |

---

## 七、动手改一改

| 改什么 | 怎么改 | 预期现象 |
|---|---|---|
| 转得更大 | `head_left_yaw_deg` 改为 `40.0`、`head_right_yaw_deg` 改为 `-40.0` | 左右转动幅度变大 |
| 停得更久 | `head_hold_duration_sec` 改为 `3.0` | 每个方向停留 3 秒 |
| 去掉等待 | 在 `py_tree.json` 中删除 `hold_left` 节点 | 左转目标刚下发就被右转目标覆盖，几乎看不到左转 |

改完重启行为树，先干跑确认仍能加载，再在仿真或真机上观察。

---

## 八、下一步

- [07-手臂与头部并行-instructional_case_02](07-手臂与头部并行-instructional_case_02.md)：在顺序树里加入 `Parallel`，让头部和双臂同时动作
- [05-常见错误](05-常见错误.md)：报错与处置
