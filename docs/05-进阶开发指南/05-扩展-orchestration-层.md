# 05 扩展 orchestration 层

本文中的相对路径（如 `orchestration/`、`apps/test_upper_init/run_behavior_tree_json.py`、`skills/`）
一律相对 **LeTools 仓库根目录**；各命令需在仓库根目录下执行。

`orchestration` 把原子技能串成行为树：**用 JSON 描述流程、用 Python 描述单个动作**。

> 落层判定与依赖方向见 `01-扩展总则与分层规范`。`orchestration` 不得 import `apps`（现有例外见 E3，清单见 [`01-04-系统架构与分层`](../01-介绍/04-系统架构与分层.md#四已知分层例外e1--e2--e3)）。

## 一、行为树运行机制

LeTools 默认以 **50Hz**（约每 20ms 一次）对整棵树执行一次 `tick`：

```text
tick（根节点）
  │
Sequence（策略：全部按序做成）
  │
  ├──────────────┬─────────────────┐
叶子 WaitForEnter  子树 look_for_tag  子树 walk_to_tag
（真正驱动硬件）    （内部可再挂 Parallel）
  │
SUCCESS / FAILURE / RUNNING
  ↑ 状态沿父节点向上冒泡，父节点按策略决定整段结果
```

**三类节点**（按职责分）：

| 类型 | 作用 | 代表实现 |
|---|---|---|
| 复合节点（Composite） | 只决定策略（子节点如何被调度与汇总），不直接发关节指令 | `Sequence` / `Selector` / `Parallel` |
| 装饰器（Decorator） | 有且仅有一个子节点，只改写其调度方式或返回值 | `Async` / `ForEach` / `RepeatUntil` / `RunIfIndex` / `PressureDropGuard` |
| 叶子节点（Leaf / Action） | 调用 Skill / 硬件接口，是真正驱动硬件的一层 | `HeadControlSdkMove` / `LegJointSdkMove` / `WaitForEnter` |

仓库惯例：**主树根节点几乎都是 `Sequence`**；`Parallel` 多出现在子树中（如找 Tag、腿臂协同）。

**三种状态**（`py_trees.common.Status`）：

| 状态 | 含义 | 树的行为 |
|---|---|---|
| `SUCCESS` | 本节点已完成 | 父节点按自身策略决定是否推进/结束 |
| `FAILURE` | 本节点失败 | 父节点按自身策略决定是否换分支或整体失败 |
| `RUNNING` | 本节点仍在进行中 | 父节点保持该子节点，等待下一拍继续 tick |

长时间 `RUNNING` 通常是技能在等待硬件到位（如运动插值中），并非死循环。

叶子节点典型写法：`is_finished()` 为真返回 `SUCCESS`，否则返回 `RUNNING`（参考 `orchestration/nodes/head_control_sdk_move.py` 的 `update()`）。

### memory 与策略选型

顺序脚本是「代码执行到某一行卡住」；行为树是「**每拍重新从根走下来，但 `Sequence` 在 `memory=True` 时会跳过已经成功的子节点**」。若 `memory=False`，每一拍都从第一个子节点重新检查——这适合需要实时响应环境变化的条件判定，但在搬箱主流程里不能这么设，否则底盘走完下一拍又会回头等待 Enter。

选哪一种复合节点：

```text
这些步骤必须一个接一个？            → Sequence
失败了是否有备选替代方案？          → Selector
几个动作要同时推进、谁先完成有规定？ → Parallel（再选 policy）
只需改写「成功/失败」判定或调度方式？ → 装饰器（Decorator）
```

`policy` 与 `memory` 是复合节点的**参数**，不是新的节点类型。

- `Parallel` 的 `policy` 有两种：`success_on_one`（任一成功即整体成功，用于并发找 Tag）、`success_on_all`（全部成功才算成功，用于腿臂并行）。
- **`Parallel` 不代表硬件真并行**：它默认单线程依次 tick；有阻塞动作（`time.sleep`、同步等 ROS）时必须显式叠加 `Async` 装饰器，否则仍是顺序执行。腿臂并行必须 `success_on_all` + `Async`。

## 二、节点基类契约

`orchestration/nodes/base_node.py` 定义 `BaseAction(Behaviour)`，**新 Action 节点应继承此类**：

```python
class BaseAction(Behaviour):
    def __init__(self, name, label, namespace, params):
        super().__init__(name=name)
        self.label = label
        self.namespace = namespace
        self.params = params
        self.global_blackboard = self.attach_blackboard_client()
```

节点生命周期（`py_trees` 约定）：

- `initialise()`：**首次进入该节点时触发一次**。解析 `params`、实例化 Skill 对象。
- `update()`：**每一拍 tick 触发一次**。调用 `skill.execute()` 并查询执行状态，返回 `SUCCESS` / `FAILURE` / `RUNNING`。
- `terminate()`：**离开该节点时触发**，处理收尾清理。

每个公开节点以 `@define_manifest` 声明（定义在 `orchestration/utils/manifest_decorators.py`）。manifest 是节点被 JSON 树引用时的注册契约，用于前端渲染节点面板与参数表单、生成 `node_library.json`、运行时参数校验。

## 三、黑板读写

黑板（Blackboard）是全局键值存储区，用于在节点间传递中间变量（如 `walk_goal`、`latest_tag_4`），避免把运行时数据硬编码进行为树结构。

`BaseAction` 在构造时 `attach_blackboard_client()` 挂上全局黑板。JSON 侧读取黑板用 `source: READ_BOARD`；参数名与黑板键不一致时，用 `board_key` 显式指定黑板键：

```json
"target_tag_id": {
  "value": "",
  "source": "READ_BOARD",
  "data_type": "int",
  "board_key": "pick_tag_id"
}
```

黑板初始值来自各场景的 `board.json`（顶层 key 即黑板键名）。下表为各场景的**核心/示例键，非穷尽列表**，完整键请直接读对应 `board.json`：

| 场景 | board.json 顶层键（示例） |
|---|---|
| `studio_smoke_v1` / `refactored_sdk_atomic_v1` | `ArmJointTrajectories` |
| `refactored_sdk_arm_v1` | `EETrajLocalForward`、`EETrajWorldForward`、`EETrajWorldUp`、`ArmJointTrajectories` |
| `refactored_sdk_single_tag_pick_v0` | `pick_tag_id`、`tag_ids`、`stand_in_tag_pos`、`stand_in_tag_euler`、`box_width`、`box_behind_tag`、`box_beneath_tag`、`box_left_tag`、`head_search_yaws`、`head_search_pitchs`、`tag_scan_timeout`、`chassis_walk_mode` |
| `wheel_arm_single_tag_pick_v1` | `pick_tag_id`、`tag_ids`、`box_width`、`box_behind_tag`、`box_beneath_tag`、`box_left_tag`、`hand_pitch_degree`、`arm_control_type`、`use_virtual_tag`、`virtual_tag_pose_in_odom` |
| `grasp_mtbf` | `repeat_count`、`mtbf_stop_requested`、`navigation_config`、`pick_tag_id`、`place_tag_id`、`tag_ids`、`use_virtual_tag`、`fallback_to_virtual`、`require_fresh_tag`、`real_tag_timeout_sec`、`box_size`、`pick_position_offset` |
| `dismantle_box` / `dismantle_box_demo` / `dismantle_box_demo_action` / `jibot_nav_v1` | 顶层为空对象 `{}`（参数在节点内或运行时注入） |

## 四、场景目录与四类配置文件

场景目录位置：`orchestration/scenarios/<场景名>/`。

```text
<scene>/
├── py_tree.json            # 【必填】主行为树定义文件
├── py_tree_child.json      # 【有子树时必填】子树库定义文件
├── board.json              # 【建议配置】黑板初始参数
├── hardware_config.json    # 【可选】场景级硬件参数覆盖
└── readme.md               # 【建议配置】场景开发说明
```

启动脚本 `--scenario <目录>` 会自动加载同目录下的 `py_tree.json`、`py_tree_child.json` 和 `board.json`。

四类文件的定位**完全不同**，不要混用：

| 文件 | 定位 | 结构 |
|---|---|---|
| `py_tree.json` | **任务顺序**（主树） | 顶层 `tree`（必填）、`interface`（建议） |
| `py_tree_child.json` | **可复用段落的仓库**（子树库） | 顶层为 KV 字典：`文件名 -> 子树定义`；主树用 `.json` 结尾的 `name` 引用；工厂加载时 deepcopy |
| `board.json` | **运行时数据与物理参数/标定常量** | 扁平 Dict，**顶层 key 即黑板键名** |
| `hardware_config.json` | **场景级硬件参数覆盖**（**不是**行为树字段） | 真机路径下、创建 `IHardware` 前 merge 进硬件工厂配置 |

设计原则：**物理参数与标定常量写 `board.json`，控制逻辑流写 `py_tree.json`。**

### 4.1 `py_tree.json` — 主树

顶层字段：

| 字段 | 含义 | 必填 |
|---|---|---|
| `tree` | 树的根节点定义 | 是 |
| `interface` | 对外可配置参数的声明（`inputs` / `outputs` / `custom` / `process`） | 建议 |

`tree` 的节点字段：

| 字段 | 含义 |
|---|---|
| `name` | 复合节点名 / 子树文件名（`.json` 结尾）/ 叶子节点类名 |
| `label` | 自由命名，日志与前端显示用 |
| `params` | 节点参数 |
| `childs` | 子节点数组；叶子节点为 `[]` |
| `childBoard` | 子黑板项；叶子节点为 `[]` |

### 4.2 `py_tree_child.json` — 子树库

顶层采用 `文件名 -> 子树定义` 的 KeyValue 字典：

```json
{
  "look_for_tag.json": { "tree": { "name": "Parallel", "label": "look_tag", "params": {}, "childs": [] } },
  "walk_to_tag.json":  { "tree": { "name": "Sequence", "label": "walk_tag", "params": {}, "childs": [] } }
}
```

主树中通过节点 `name` 引用（`name` 必须以 `.json` 结尾，且完全匹配键名）：

```json
{
  "name": "look_for_tag.json",
  "label": "find_box_subtree",
  "params": {
    "target_tag_id": { "value": "${pick_tag_id}", "source": "INPUT", "data_type": "int" }
  },
  "childs": []
}
```

同一子树可被主树引用多次；引擎工厂在加载时会自动 deepcopy，保证各子树实例间状态相互隔离。

### 4.3 `board.json` — 黑板初始值

推荐扁平 JSON Dict 结构：

```json
{
  "pick_tag_id": 4,
  "use_virtual_tag": true,
  "box_length": 0.4
}
```

### 4.4 `hardware_config.json` — 可选硬件覆盖

非行为树本身字段。真机路径下、创建 `IHardware` 前会 merge 进硬件工厂配置。例如：

```json
{
  "skip_force_publishers": true,
  "sdk_managers_whitelist": ["timed", "arm", "low"]
}
```

## 五、JSON 怎么写

### 5.1 节点通用字段

```json
{
  "name": "HeadControlSdkMove",
  "label": "look_left_30",
  "params": {},
  "childs": []
}
```

### 5.2 `params` 参数结构

```json
"yaw_deg": { "value": "30.0", "source": "CUSTOM", "data_type": "float" }
```

`data_type` 支持：`float` / `int` / `string` / `bool` / `list` / `json`。数组类型直接写在 `value` 中：

```json
"head_search_yaws": { "value": [12, 0, -12, 0], "source": "CUSTOM", "data_type": "list" }
```

`source` 取值：固定值用 `CUSTOM`、从黑板读用 `READ_BOARD`、宏替换后为 `RESOLVED`。

### 5.3 主树 `interface.inputs` 与 `${宏}`

在 `py_tree.json` 头部声明对外可配置变量：

```json
{
  "internal_name": "pick_tag_id",
  "type": "int",
  "required": false,
  "default": 4,
  "description": "取货侧箱子 Tag ID"
}
```

加载时变量取值的优先级：**命令行启动覆盖参数 > `board.json` / 黑板已有值 > `interface` 设定的 `default`**。解析出的值会自动替换树中 `"value": "${pick_tag_id}"` 且 `source: INPUT` 的节点参数。因此日常修改 Tag ID 或箱子尺寸，**只需改 `board.json`**。

未在 `interface.inputs` 声明的 `${var}` 会产生「变量未定义」告警且不会被替换。

### 5.4 `name` 的三种含义

```text
"name": "Sequence"          → 复合节点
"name": "Async"             → 装饰器（必须包含 childs）
"name": "look_for_tag.json" → 引用子树（Key 必须定义在 py_tree_child.json 中）
"name": "CalcLegMove"       → 叶子节点（必须能映射到 orchestration/nodes/ 下的 Python 类）
```

叶子节点类名匹配规则：`"HeadControlSdkMove"` 会自动匹配 `head_control_sdk_move.py`（驼峰转蛇形）或 `HeadControlSdkMove.py`。推荐统一使用 Python 类的原本名称。

## 六、从 JSON 到机器人动作的数据流

以控制头部转动至 `yaw=30°` 为例：

```text
JSON 配置:
   "name": "HeadControlSdkMove"
   "params.yaw_deg.value": "30.0"
        ↓ 工厂根据类名动态 import
orchestration/nodes/head_control_sdk_move.py
   initialise(): 解析 Params，创建 HeadControlSdk Skill
   update():     调用 skill.execute()，按结果返回 SUCCESS/RUNNING
        ↓
skills/atomic/refactored_sdk/head_control_sdk.py
   hardware.control_head_sdk(yaw=30.0, pitch=0.0)
        ↓
Adapter → SDK / ROS 话题 → 驱动头部物理关节
```

核心法则：**JSON 管编排，Skill 管动作，Adapter 管底层通信。**

## 七、DryRun 机制

`--dry-run` 的作用：

- 设置环境变量 `STUDIO_DRY_RUN=1`；
- **不初始化 ROS 节点** `init_node`，**不创建 `IHardware`**；
- 运动节点与 `WaitForEnter` 等阻塞节点**直接返回 `SUCCESS`**；
- 主要目的：快速校验依赖 import、类名拼写、JSON 结构与子树 Key 是否合法。

`--tick-once` 与 `--dry-run` 合用，只 tick 一拍。

**成功判定**：dry-run 时以输出中的 `tick 后根状态: Status.SUCCESS` 为准；退出码只反映加载是否成功（加载失败为非 0，加载成功后总是 0）。非 dry-run 运行时，退出码按根节点终态：`SUCCESS=0`、`FAILURE=1`、其它 `=2`。

```bash
# 【Dry-Run 语法检查】只校验 JSON 与节点加载，只 tick 一拍
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/studio_smoke_v1 \
  --dry-run --tick-once

# 【仿真或真机实跑】（需预先启动 ROS 及运控站立链）
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/refactored_sdk_single_tag_pick_v0
```

行为树依赖 Python 包 `py_trees`（注意是复数），Ubuntu 20.04 / ROS Noetic / Python 3.8 下请锁定 `py_trees==2.2.3`。

## 八、新建场景标准流程

1. **复制基础模板**：复制 `studio_smoke_v1` 或 `refactored_sdk_single_tag_pick_v0` 目录，**勿直接从产线真机树改动**。
2. **配置黑板参数**：修改 `board.json` 中的 Tag ID、位姿目标及几何尺寸。
3. **编排主树结构**：调整 `py_tree.json` 的 `childs` 数组顺序，确保使用的节点 `name` 均已实现。
4. **抽离重复逻辑**：将多次复用的动作段落抽离至 `py_tree_child.json`。
5. **执行 DryRun 校验**：

```bash
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/my_custom_demo \
  --dry-run --tick-once
```

6. **仿真验证与真机迁移**：在 MuJoCo 仿真中验证通过后，方可迁至真机运行。
7. **写 `readme.md`**：说明场景目标、依赖的前提（ROS/运控站立）与运行命令。

## 九、新增动作节点

若现有节点无法满足需求，需按三层依次添加：

```text
skills/atomic/refactored_sdk/<feature>.py   →  <Feature>Skill
orchestration/nodes/<feature>_move.py       →  <Feature>Move
场景 JSON 的 name                            →  "<Feature>Move"
```

1. **新增技能 Skill**：在 `skills/atomic/refactored_sdk/<feature>.py` 中定义 `<Feature>Params` 与继承自 `SkillBase` 的 `<Feature>Skill`；
2. **新增节点 Node**：在 `orchestration/nodes/<feature>_move.py` 中继承 `BaseAction` 实现 `<Feature>Move`，加 `@define_manifest`，在 `initialise()` 建 Skill、`update()` 调 `execute()`；
3. **在 JSON 中引用**：在场景 JSON 的 `name` 中填 `<Feature>Move`。

## 十、样例场景阅读顺序

推荐按以下顺序依次阅读仓库内置场景，**切勿直接修改产线复杂行为树**：

1. `scenarios/studio_smoke_v1/`（冒烟测试）：最小主树结构（Enter 确认 → 底盘移动 → 蹲下 → 抬臂）。
2. `scenarios/refactored_sdk_atomic_v1/`（原子技能演示）：学习子树引用、`Parallel` 与 `Async` 并发使用。
3. `scenarios/refactored_sdk_single_tag_pick_v0/`（单 Tag 取放）：包含子树拆分、`${宏}` 替换、虚拟 Tag 注入。
4. `scenarios/grasp_mtbf/`（循环搬箱）：用公共节点编排地图导航、Tag 识别、关键点计算与八阶段抓放，可配置轮数。

## 十一、研发设计原则与易错点

**设计原则**

1. **策略先行**：顺序做成用 `Sequence`；容错备选用 `Selector`；并发推进用 `Parallel`。
2. **职责分离**：任务流程归 JSON 配置文件，控制算法与状态机归 Skill 模块。
3. **数据解耦**：物理参数与标定常量写在 `board.json`，控制逻辑流写在 `py_tree.json`。
4. **主树精简**：主树应能在单屏内阅读完毕，复杂子流程全部抽离至 `py_tree_child.json`。
5. **分步验证**：严格遵循 `Dry-Run → 物理仿真 → 真机实测` 的递进测试流程。
6. **避免并发误区**：`Parallel` 仅代表状态汇总策略，涉及阻塞性动作必须显式叠加 `Async` 装饰器。
7. **保持记忆标志**：主流程的 `Sequence` 节点应维持 `memory: True`，避免每拍重复触发已完成节点。

**易错点**

1. **把 `Parallel` 当成「硬件真并行」**：需显式叠加 `Async` 装饰器。
2. **主流程 `Sequence` 误设 `memory=False`**：会出现「底盘走完下一拍又回头等待 Enter」。
3. **子树引用名不匹配**：`name` 必须以 `.json` 结尾且与 `py_tree_child.json` 的 key 完全一致。
4. **叶子节点类名拼写错误**：`name` 需精确匹配 `orchestration/nodes/` 下的类名。
5. **`params.source` 用错**：固定值用 `CUSTOM`、从黑板读用 `READ_BOARD`、宏替换后为 `RESOLVED`。
6. **`interface.inputs` 未声明就写 `${var}`**：只会产生告警且不被替换。
7. **在真机上直接改产线行为树**：应先 `--dry-run` → 仿真 → 真机。
8. **`py_trees` 版本不一致**：锁 `2.2.3`，并确保入口先导入 `py_trees_compat`。

## 十二、常用节点速查

| 文件 | 类名 | 对应 Skill | 使用场景 |
|---|---|---|---|
| `wait_for_enter.py` | `WaitForEnter` | `wait_for_enter` | 全部场景（人工确认） |
| `wait_seconds.py` | `WaitSeconds` | `wait_seconds` | 定时等待 |
| `chassis_short_move.py` | `ChassisShortMove` | `chassis_velocity` | `studio_smoke_v1` |
| `leg_short_move.py` | `LegShortMove` | `leg_control` | `studio_smoke_v1` |
| `move_arm_base_joint_trajectories.py` | `MoveArmBaseJointTrajectories` | 旧版 `arm_control` | `studio_smoke_v1` |
| `head_control_sdk_move.py` | `HeadControlSdkMove` | `head_control_sdk` | `refactored_sdk_atomic_v1` |
| `arm_joint_traj_sdk_move.py` | `ArmJointTrajSdkMove` | `arm_joint_traj_sdk` | `refactored_sdk_atomic_v1`、`refactored_sdk_arm_v1` |
| `arm_reset_sdk_move.py` | `ArmResetSdkMove` | `arm_reset_sdk` | `refactored_sdk_arm_v1` |
| `leg_joint_sdk_move.py` | `LegJointSdkMove` | `leg_joint_sdk` | `refactored_sdk_atomic_v1` |
| `base_pose_local_move.py` | `BasePoseLocalMove` | `base_pose_local` | `refactored_sdk_atomic_v1` |
| `chassis_navigation_move.py` | `ChassisNavigationMove` | — | 导航类场景 |
| `node_wait_for_blackboard.py` | `NodeWaitForBlackboard` | — | 等待黑板（找 Tag 并行） |
| `node_head_sequence.py` | `NodeHeadSequence` | — | 扫头序列 |

装饰器节点：`Async` / `ForEach` / `RepeatUntil` / `RunIfIndex` / `PressureDropGuard`（均位于 `orchestration/nodes/`）。

已废弃/未使用：`move_head.py`（`MoveHead`，未使用）、`skill_node.py`（遗留抽象框架，已废弃）。

## 十三、相关篇位

- 测试与故障排查：`05-07-测试与调试`
- 节点与技能的接口细节：`03-接口详细文档`（orchestration / skills 篇）
- 技能侧扩展方法：`05-04-扩展-skills-层`
