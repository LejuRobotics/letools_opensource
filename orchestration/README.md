# orchestration 编排层

PyTrees 行为树运行入口、节点库与场景配置。阶段 1 默认场景为 **`studio_smoke_v1`**（底盘 → 腿 → 臂冒烟树）。


## 新手理解：orchestration 编排层是干什么的

`orchestration` 可以理解成机器人任务的“导演层”。它不亲自控制电机，也不直接负责 ROS 话题发布，而是负责把多个动作安排成一个完整任务流程。

例如一个简单任务：

```text
等待人工确认
↓
底盘向前移动
↓
腿部/躯干调整
↓
手臂执行动作
↓
任务结束
```

这些步骤由行为树组织，`orchestration` 就负责加载行为树、创建节点、周期性 tick，并根据节点返回状态决定下一步。

### 完整执行链路

```text
python3 orchestration/main.py
↓
读取 board.json 和 py_tree.json
↓
BehaviorTreeFactory 根据 JSON 创建行为树
↓
JSON 中的节点名映射到 orchestration/nodes/ 下的 Python 节点
↓
节点解析参数，并调用对应 Skill 或 Hardware 方法
↓
Skill 调用 hardware 标准接口
↓
Adapter / SDK / ROS 控制机器人
```

### 目录怎么理解

```text
orchestration/
├── main.py              # 编排入口，负责启动行为树
├── engine/              # 行为树工厂、控制器、tick 主循环
├── nodes/               # 行为树叶子节点，连接 JSON 和 Skill/Hardware
├── scenarios/           # 场景 JSON，描述一棵树怎么长
├── tasks/               # 任务级配置或旧任务入口
├── services/            # 黑板等运行时服务
└── shared_hardware.py   # 创建并共享 hardware，避免每个节点重复初始化机器人
```

### dry-run 和真机模式的区别

```text
--dry-run
└── 不连接 ROS 和硬件，主要检查 import、JSON、节点加载、行为树结构是否正确。

真机模式
└── 会初始化 ROS 节点和 hardware，节点会真正调用 Skill/Adapter 控制机器人。
```

所以新手第一次看行为树，建议先用：

```bash
python3 orchestration/main.py --dry-run --tick-once
```

确认树能加载，再考虑真机运行。

### 和 Skill / Adapter 的关系

```text
orchestration
└── 负责“什么时候做哪个动作”

skills
└── 负责“某个动作具体怎么做”

adapters / hardware
└── 负责“怎么把动作变成机器人能执行的底层调用”
```

一句话总结：

```text
orchestration 不是单个动作，而是把多个动作编排成完整任务。
```

---
---

## `main.py` 使用说明

**路径**: `LeTools/orchestration/main.py`  
**ROS 节点名**: `behavior_tree_main`（与 embodied 一致）

在 **LeTools 仓库根目录** 下执行（或先 `cd` 到该目录）。

### 环境准备

```bash
export PYTHONPATH=/path/to/LeTools:$PYTHONPATH
cd /path/to/LeTools

# 按项目文档 source（示例，路径以实机为准）
source infrastructure/ros_packages/devel/setup.bash
source /path/to/embodied/devel/setup.bash   # pytrees_actions msg/srv

export ROBOT_VERSION=45   # 与 5W 轮臂文档一致
```

实机前可选预检（不跑行为树）：

```bash
python3 apps/test_kuavo_5w_app/verify_phase1_standard_methods.py
```

---

### 快速开始

| 场景 | 命令 |
|------|------|
| 无 ROS / 无机器人，检查能否加载树 | `python3 orchestration/main.py --dry-run` |
| 干跑 + 单次 tick | `python3 orchestration/main.py --dry-run --tick-once` |
| **真机冒烟（推荐）** | `python3 orchestration/main.py` |
| 跑完后保持 ROS 节点 | `python3 orchestration/main.py --spin` |

默认树与黑板：

| 文件 | 默认路径 |
|------|----------|
| 行为树 | `orchestration/scenarios/studio_smoke_v1/py_tree.json` |
| 黑板 | `orchestration/scenarios/studio_smoke_v1/board.json` |

场景说明见 [scenarios/studio_smoke_v1/README.md](./scenarios/studio_smoke_v1/README.md)。

---

### 命令行参数

```bash
python3 orchestration/main.py [-h] [--dry-run] [--tick-once] [--spin]
                            [--tree TREE] [--board BOARD]
```

| 参数 | 说明 |
|------|------|
| `--dry-run` | 不 `rospy.init_node`、不连硬件；设置 `STUDIO_DRY_RUN=1`，节点走桩逻辑 |
| `--tick-once` | 仅与 `--dry-run` 合用；加载后对树执行 **一次** `tick` |
| `--spin` | 树结束后执行 `rospy.spin()`，进程常驻（Ctrl+C 退出）；**默认不开启** |
| `--tree` | 指定 `py_tree.json` 路径 |
| `--board` | 指定 `board.json` 路径 |

查看帮助：

```bash
python3 orchestration/main.py -h
```

---

### 运行流程（真机模式）

```text
rospy.init_node("behavior_tree_main")
  → 加载 board.json
  → 注册 stop / pause / resume 服务
  → BehaviorTreeController 50Hz tick 执行树
  → 根节点 SUCCESS / FAILURE
  → 释放 shared_hardware
  → 进程退出（除非 --spin）
```

默认树步骤（需人工 **按 Enter** 后继续）：

1. `WaitForEnter`
2. `ChassisShortMove` → `chassis_velocity`
3. `LegShortMove` → `leg_control`
4. `MoveArmBaseJointTrajectories` → `arm_control`

---

### 进程退出与退出码

**默认行为**：树跑完后 **自动退出**（不 `spin`），便于脚本/CI 判断结果。

| 根节点终态 | 日志示例 | 退出码 |
|------------|----------|--------|
| `SUCCESS` | `[BehaviorTree] 完成: SUCCESS` → `studio_smoke 完成，进程退出 (0)` | `0` |
| `FAILURE` | `行为树 FAILURE，进程退出 (1)` | `1` |
| 其它 / 加载失败 | `行为树未达终态 ... (2)` | `2` |

使用 `--spin` 时：树结束后仍保持节点，可用下面 ROS 服务或 Ctrl+C 结束。

---

### ROS 服务（真机模式）

| 服务 | 类型 | 作用 |
|------|------|------|
| `/stop_behavior_tree` | `std_srvs/Empty` | 停止主循环 |
| `/pause_behavior_tree` | `std_srvs/Empty` | 暂停 tick |
| `/resume_behavior_tree` | `std_srvs/Empty` | 恢复 tick |

仅在真机路径（非 `--dry-run`）下注册；长期联调可加 `--spin` 保持进程。

---

### 指定其它场景树

```bash
python3 orchestration/main.py \
  --tree orchestration/scenarios/my_scene/py_tree.json \
  --board orchestration/scenarios/my_scene/board.json
```

`--tree` 文件必须存在，否则启动前 `exit 1`。

---

### 干跑模式说明

- 设置环境变量 `STUDIO_DRY_RUN=1`（由 `--dry-run` 自动设置）。
- 运动类节点（如 `ChassisShortMove`、`LegShortMove`）不调用 `IHardware`，直接返回 SUCCESS。
- 用于 CI / 新机器检查：**import、节点索引、JSON 解析** 是否正常。
- `WaitForEnter` 在无 TTY 时读 EOF 也会 SUCCESS，故 `--dry-run --tick-once` 常得到根节点 SUCCESS。

---

### 常见问题

| 现象 | 可能原因 | 建议 |
|------|----------|------|
| 手臂步骤日志后长时间无输出 | Adapter 内等待 `/lb_arm_joint_reach_time` + 默认 sleep（约 8s） | 属阻塞等待，非主循环卡死；结束后应打印「完成」并退出 |
| 跑完仍不退出 | 使用了 `--spin` | 去掉 `--spin`，或 Ctrl+C |
| 节点 import 失败 | 未设置 `PYTHONPATH` 或未 source embodied | 见上文「环境准备」 |
| 树 FAILURE | 某节点返回 FAILURE | 查该节点 `feedback_message` 与 rospy 日志 |

---

### 相关代码与文档

| 路径 | 说明 |
|------|------|
| [engine/behavior_tree_controller.py](./engine/behavior_tree_controller.py) | 50Hz 主循环、终态检测与退出 |
| [engine/behavior_tree_factory.py](./engine/behavior_tree_factory.py) | JSON → 树、节点动态加载 |
| [shared_hardware.py](./shared_hardware.py) | 全局 `IHardware` 单例 |
| [nodes/](./nodes/) | 行为节点实现 |
| [scenarios/](./scenarios/) | 场景 `py_tree.json` / `board.json` |
| [../docs/PHASE1_SIMPLE_TREE_TASK_PLAN.md](../docs/PHASE1_SIMPLE_TREE_TASK_PLAN.md) | 阶段 1 任务与 DoD |
| [../docs/modules/MODULE_orchestration.md](../docs/modules/MODULE_orchestration.md) | 编排层开发指南 |

---

**维护**: `main.py` 行为变更时同步更新本文「命令行参数」「退出码」两节。


