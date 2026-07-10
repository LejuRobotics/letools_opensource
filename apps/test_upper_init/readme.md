# 行为树 JSON 启动器使用说明

## 简介

轻量级机器人行为树启动工具，仅负责从 JSON 加载并运行行为树，**所有业务逻辑由行为树配置定义**。支持离线验证（无 ROS）和 ROS 环境运行。

> 编排层主入口 `orchestration/main.py` 默认场景为 `studio_smoke_v1`，详见 [orchestration/README.md](../../orchestration/README.md)。本启动器是**通用、场景无关**的入口，建议新场景优先使用本启动器。

---

## 与 `orchestration/main.py` 的区别

| 维度 | `run_behavior_tree_json.py`(本启动器) | `orchestration/main.py` |
|---|---|---|
| 路径 | `apps/test_upper_init/` | `orchestration/` |
| 默认场景 | 无(必须 `--scenario`) | `studio_smoke_v1` |
| `--scenario` 自动发现 | ✅ | ❌(需 `--tree` + `--board`) |
| 子树加载 | 单文件 `py_tree_child.json` | 同左 |
| 适用 | 通用 / CI / 多场景复用 | 阶段 1 默认烟测 |
| 干跑机制 | `--dry-run` | `--dry-run`(等价) |

---

## 快速使用

### 1. 前置要求

- Python 3.8+
- 运行真实机器人需 ROS 环境(`source infrastructure/ros_packages/devel/setup.bash`)
- 项目根目录执行

### 2. 基本命令

```bash
# 指定场景运行(最常用)
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/<your_scenario>

# 干跑验证(无 ROS、不连硬件)
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/<your_scenario> --dry-run

# 干跑 + 单次 tick
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/<your_scenario> --dry-run --tick-once

#跑全部 6 个场景，每个场景全部 6 组动作
python3 apps/test_upper_init/run_all_dismantle_box.py

# 只跑场景 1 和 3，每个场景只跑第 1、3 组动作
python3 apps/test_upper_init/run_all_dismantle_box.py  \
  --scenario orchestration/scenarios/<your_scenario> --scenarios 1,3 --action-groups 1,3

#  指定场景子集 + 动作组子集
python3 apps/test_upper_init/run_all_dismantle_box.py  \
   --scenarios 2,4,6 --action-groups 1


```

---

## 核心参数

| 参数 | 作用 |
|---|---|
| `--scenario DIR` | 场景文件夹，自动从该目录取 `py_tree.json` / `py_tree_child.json` / `board.json` |
| `--tree PATH` | 显式指定主树路径(优先级高于 `--scenario` 默认值) |
| `--subtrees PATH` | 显式指定子树集合文件(单文件 `py_tree_child.json`) |
| `--board PATH` | 显式指定黑板 JSON |
| `--dry-run` | 离线验证：不初始化 ROS，不连硬件；需 `py_trees` 可用 |
| `--dry-run --tick-once` | 干跑并执行一次 tick |
| `--spin` | 树跑完后 `rospy.spin()`(Ctrl+C 退出) |
| `--parallel-load` | 启用并行构树(可能引发 import 死锁，默认关闭) |
| `--ros-node NAME` | ROS 节点名，默认 `behavior_tree_main` |

---

## 场景目录规范

```
your_scenario/
├── py_tree.json            # 必需 - 主树
├── py_tree_child.json      # 必需 - 子树集合(单文件,key 为子树文件名)
└── board.json              # 必需 - 黑板
```

`py_tree_child.json` 是**唯一**受支持的子树加载机制:文件是 dict,key 是子树文件名(如 `demo_walk.json`),value 是子树完整 JSON(含 `tree` / `interface` 字段)。`py_tree.json` 节点中通过 `"name": "demo_walk.json"` 引用,**`name` 必须与 `py_tree_child.json` 的 key 完全一致**。

`BehaviorTreeFactory` 在 `subtree_config` dict 中按 key 查找子树,找不到时直接 `KeyError` 退出(不会"宽松"地从同目录 glob)。

```
your_scenario/
├── py_tree.json                            # 主树，内嵌引用若干 demo_*.json
├── board.json
├── demo_single_tag_walk_and_pick.json      # 子树 1
├── demo_single_tag_lift.json               # 子树 2
└── demo_single_tag_back_and_reset.json     # 子树 3
```

适用场景：案例可拆成 3+ 棵独立子树，且各自有 README/测试价值时。

---

## 三链运行模式

每个场景都应在以下三链上验证：

### 链 1：干跑(无 ROS、无硬件)

```bash
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario <DIR> --dry-run --tick-once
```

- 节点走桩逻辑(读 `STUDIO_DRY_RUN=1`)
- 验证：import、JSON 解析、节点索引、初始 tick 不出 FAILURE
- 退出码 0 = 通过；1 = FAILURE 出现

### 链 2：MuJoCo 仿真

需先**手动**启动 MuJoCo 仿真器(在 `kuavo-ros-opensource` 仓库)，然后：

```bash
python3 apps/test_upper_init/run_behavior_tree_json.py --scenario <DIR>
```
- 关联文件夹如下（目录下有相应readme）
```
orchestration
├──orchestration/engine          
├──orchestration/nodes    
└──orchestration/scenarios            
```
```
skills 
└──skills/atomic/refactored_sdk            
```


- 节点调用 `IHardware` 桩
- 验证：节点时序、blackboard 流转、`/robot_tag_info` 等话题可读

### 链 3：真机 + 兜底

需先启动下位机控制器(`load_kuavo_real.launch`)，然后同上条命令。

- 节点调用真实硬件
- 验证：闭环控制、安全降级(如 `real_tag_timeout_sec` 触发 fake tag 注入)

---

## 退出码

| 退出码 | 含义 |
|---|---|
| `0` | 根节点 SUCCESS |
| `1` | 根节点 FAILURE |
| `2` | 异常 / 其他状态 / 未达终态 |

`--spin` 模式下不退出，按 Ctrl+C 退出后不保证退出码。

---

## 排错清单

| 现象 | 可能原因 | 建议 |
|---|---|---|
| `[apps] 主树 py_tree.json 不存在` | `--scenario` 路径错或漏参数 | 检查 cwd 和参数 |
| `[apps] 子树集合文件不存在，忽略` | `--subtrees` 指向错误路径 | 该警告通常可忽略，除非预期有子树 |
| 节点 FAILURE: `KeyError` on blackboard | 节点未声明 READ access | 查 `orchestration/nodes/node_*.py` 的 `__init__` 中 `register_key` |
| `dry-run` 通过但真机 FAILURE | 干跑不调 `IHardware` | 看节点 `feedback_message`，可能硬件未连接 |
| `board.json missing key: X` | 新增 input 但 board 未同步 | 同步更新 `board.json` 字段 |
| 节点一直 RUNNING 不退出 | 等待某 blackboard key 永不到达 | 检 `NodeWaitForBlackboard` 超时配置或上游节点是否写入 |

---

## 简单说明

- 退出码：`0=执行成功`，`1=执行失败`，`2=异常/其他状态`
- 若需单独指定主树/子树/黑板文件,可使用 `--tree` / `--subtrees` / `--board` 参数,优先级高于 `--scenario`
- `board.json` 支持两种结构:扁平 dict 或分组 list(启动器自动识别)
- 子树 JSON 的 `interface.inputs` 字段描述该子树需要的外部输入
- 节点类自动从 `orchestration/nodes/` 发现:snake_case 文件名 → PascalCase 类名
- 详细编排机制见 [orchestration/README.md](../../orchestration/README.md)
- 完整链路示例见各 scenario 自带 readme(如 [refactored_sdk_atomic_v1 § 完整链路示例](../../scenarios/refactored_sdk_atomic_v1/readme.md#完整链路示例))

---

## v0 已知约束(refactored_sdk_single_tag_pick_v0 专属)

v0 阶段以下两处为**骨架占位**,真机集成测试时需替换为真实实现:

| 位置 | 当前行为 | 替换为 |
|---|---|---|
| `NodeTagToArmGoal.update()` | 写 `placeholder=[None]`,让下游 `NodeWheelArm` 的 `if l and r:` 通过 | 真实逆运动学输出(参考源版 `nodes.py:188-360`) |
| `NodeComputePickGoal.update()` | 访问 `tag.pose_in_world.x / .y / .z`(`Pose6D` 是 dataclass,无 `.pos` 属性) | 保持(已与 `core/domain/pose.py` 对齐) |

---

## 简单说明