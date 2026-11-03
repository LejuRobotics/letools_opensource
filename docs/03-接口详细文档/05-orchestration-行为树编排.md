# 05 orchestration 行为树编排

本文中的相对路径（如 `orchestration/`、`apps/test_upper_init/run_behavior_tree_json.py`）
一律相对 **LeTools 仓库根目录**；各命令需在仓库根目录下执行。

> **核心法则：JSON 管编排，Skill 管动作，Adapter 管底层通信。**

orchestration 层是任务的「导演层」：它加载场景 JSON、把节点拼成 py_trees 行为树、以固定周期 tick，并根据节点返回的状态决定下一步。它自己不控制电机。

## 05.01 命名空间与类索引

| 点分路径 | 实体 | 说明 |
|---|---|---|
| `orchestration.main` | `main()` | 行为树主入口（ROS 节点名 `behavior_tree_main`），默认场景 `studio_smoke_v1` |
| `orchestration.engine.behavior_tree_factory` | `BehaviorTreeFactory`、`ParamsWrapper` | **核心工厂**：JSON → py_trees 树，节点动态发现与实例化 |
| `orchestration.engine.behavior_tree_controller` | `BehaviorTreeController` | **运行控制器**：50Hz 主循环 `tick()`、终态检测、stop/pause/resume 服务 |
| `orchestration.engine.py_trees_compat` | — | py_trees 0.7.x / 2.x Blackboard API 兼容层，**必须先于**工厂导入 |
| `orchestration.engine.behavior_tree_engine` | — | **遗留**早期顺序执行引擎，已被工厂 + 控制器取代 |
| `orchestration.nodes.*` | 各 `<Feature>Move` | 行为树叶子节点（薄封装） |
| `orchestration.nodes.base_node` | `BaseAction` | 所有新节点的基类（继承 `py_trees.behaviour.Behaviour`） |
| `orchestration.nodes.base` | → `BaseAction` | 兼容层，重导出 |
| `orchestration.nodes.skill_node` | `SkillNode` | **遗留**抽象框架，已废弃 |
| `orchestration.nodes.async_decorator` | `Async` | 装饰器：子节点放后台线程 tick |
| `orchestration.nodes.for_each` | `ForEach` | 装饰器：对黑板列表逐项执行同一子树 |
| `orchestration.nodes.repeat_until` | `RepeatUntil` | 装饰器：重复子节点直到黑板条件满足 |
| `orchestration.nodes.run_if_index` | `RunIfIndex` | 装饰器：按索引门控 |
| `orchestration.nodes.pressure_drop_guard` | `PressureDropGuard` | 装饰器：压力下降保护 |
| `orchestration.utils.manifest_decorators` | `define_manifest` / `get_manifest` / `get_all_manifests` / `export_manifests_to_dict` / `NodeManifest` | 节点元数据注册与导出 |
| `orchestration.shared_hardware` | `get_shared_hardware()` / `set_hardware_config()` / `reset_shared_hardware()` | 全局硬件单例 |
| `orchestration.parser.task_parser` | `TaskParser` | task.yaml 解析（旧任务入口） |
| `orchestration.services.blackboard_service` | — | 黑板 ROS 服务（**已停用**，见下） |

## 05.02 类与方法详表

### 引擎

| 路径 | API | 签名 | 说明 | 所属层 |
|---|---|---|---|---|
| Core | `BehaviorTreeFactory` | `BehaviorTreeFactory(blackboard_client, tree_dir=None, enable_parallel_loading=True, subtree_json_path=None, action_group_filter=None)` | 构造工厂并加载子树配置 | `orchestration.engine.behavior_tree_factory` |
| Core | `load_tree_from_json` | `load_tree_from_json(json_file, override_params=None) -> py_trees.trees.BehaviourTree` | 从 `py_tree.json` 构建整棵树 | 同上 |
| Core | `reload_subtree_config` | `reload_subtree_config() -> None` | 重新从磁盘读子树配置 | 同上 |
| Core | `load_tree_by_skill` | `load_tree_by_skill(end_effector, skill_name, override_params=None) -> BehaviourTree` | 按末端执行器 + 技能名加载预置树 | 同上 |
| 查询 | `list_available_skills` | `list_available_skills(end_effector=None) -> Dict[str, List[str]]` | 列出可用技能 | 同上 |
| 查询 | `validate_skill` | `validate_skill(end_effector, skill_name) -> bool` | 校验预置技能是否存在 | 同上 |
| Core | `BehaviorTreeController` | `BehaviorTreeController(behavior_tree_core: BehaviorTreeFactory)` | 绑定工厂 | `orchestration.engine.behavior_tree_controller` |
| Core | `start_behavior_tree` | `start_behavior_tree(tree_json_path, blackboard_client=None) -> py_trees.common.Status` | 启动 50Hz 主循环，返回根节点终态 | 同上 |
| Core | `load_tree_only` | `load_tree_only(tree_json_path, blackboard_client=None)` | **干跑**：仅加载树不启动循环 | 同上 |
| Core | `init_services` | `init_services() -> None` | 注册 `/stop_behavior_tree`、`/pause_behavior_tree`、`/resume_behavior_tree`（仅 ROS 环境） | 同上 |
| Core | `stop_behavior_tree` / `pause_behavior_tree` / `resume_behavior_tree` | `stop_behavior_tree(req=None)` | 服务回调 | 同上 |

> **黑板 ROS 服务已停用**：`orchestration/services/blackboard_service.py` 在 `BehaviorTreeController.__init__` 里的调用是**刻意注释掉的**（`pytrees_actions` 未编译，注册必然失败），不是漏写。

### 节点基类与装饰器

| 路径 | API | 签名 | 说明 | 所属层 |
|---|---|---|---|---|
| Core | `BaseAction` | `BaseAction(name, label, namespace, params)` | 叶子节点基类，继承 `py_trees.behaviour.Behaviour` | `orchestration.nodes.base_node` |
| Core | `setup` | `setup(**kwargs)` | py_trees 生命周期钩子 | 同上 |
| Core | `initialise` | `initialise()` | 每个节点子类覆写：解析 params → 建 Skill → `skill.initialize()` | 同上 |
| Core | `Async` | `Async(name, child: Behaviour, tick_hz=50.0)` | 把子节点放后台线程 tick | `orchestration.nodes.async_decorator` |
| Core | `ForEach` | `ForEach(name, child, source_key, target_key, index_key="")` | 遍历黑板列表 | `orchestration.nodes.for_each` |
| Core | `RepeatUntil` | `RepeatUntil(name, child, condition_key, condition_path="", expected_value=True, wait_for_child_completion=False, check_before_iteration=False, max_iterations=0)` | 重复直到黑板条件满足或完成指定轮数 | `orchestration.nodes.repeat_until` |
| Core | `RunIfIndex` | `RunIfIndex(name, child, index=1, nav_point_index_key="nav_point_index")` | 按索引门控（`0`/未设=全执行；`3`=只跑 index 3；`"1,3,5"`=跑列表内索引） | `orchestration.nodes.run_if_index` |
| Core | `PressureDropGuard` | 见源码 | 压力下降保护 | `orchestration.nodes.pressure_drop_guard` |

### 节点元数据装饰器

`@define_manifest` 把节点元数据挂到类的 `_manifest` 属性，并注册进全局 `_MANIFEST_REGISTRY`：

| 路径 | API | 签名 | 说明 |
|---|---|---|---|
| Core | `define_manifest` | `define_manifest(label="", category=None, description="", params=None, inputs=None, outputs=None, version="1.0.0", author="", tags=None, tree_type="")` | 类装饰器 |
| 查询 | `get_manifest` | `get_manifest(cls_or_name) -> Optional[NodeManifest]` | 按类或类名取 manifest |
| 查询 | `get_all_manifests` | `get_all_manifests() -> Dict[str, Dict[str, Any]]` | 取全部注册项 |
| 查询 | `export_manifests_to_dict` | `export_manifests_to_dict() -> Dict[str, Any]` | 导出 `node_library.json` 结构（含 `version` / `generated_at` / `node_count` / `nodes`） |

`params` 里每个参数是 `{"name", "type", "default", "description", ...}`，`type` 取值见 `VALID_TYPES`：`int` / `float` / `string` / `bool` / `intArr` / `floatArr` / `stringArr` / `json` / `pose`。

### 硬件单例

| 路径 | API | 签名 | 说明 |
|---|---|---|---|
| Core | `get_shared_hardware` | `get_shared_hardware() -> IHardware` | 线程安全单例；懒加载硬件（约 0.5s 初始化开销延迟到首次调用）；**初始化失败不缓存半成品** |
| Core | `set_hardware_config` | `set_hardware_config(config: dict)` | 首次取硬件**之前**设置配置覆盖；硬件已初始化时 `raise RuntimeError("硬件已初始化，无法更改配置")` |
| Core | `reset_shared_hardware` | `reset_shared_hardware()` | 关闭并清空单例（主入口在树跑完后调用） |

## 05.03 参数说明

### 场景目录结构

```text
orchestration/scenarios/<场景名>/
├── py_tree.json          # 必填：主行为树
├── py_tree_child.json    # 有子树时必填：子树库
├── board.json            # 建议配置：黑板初始值
├── hardware_config.json  # 可选：场景级硬件参数覆盖
└── README.md             # 建议配置：场景说明
```

### 四类配置文件定位

| 文件 | 定位 | 例子 |
|---|---|---|
| `py_tree.json` | **流程**：树的形状 | 顺序、分支、并发 |
| `py_tree_child.json` | **复用**：可被主树引用多次的子树 | `look_for_tag.json` |
| `board.json` | **数据**：运行时变量初值 | Tag ID、箱子尺寸、虚拟标签开关 |
| `hardware_config.json` | **裁剪**：场景级硬件覆盖 | `skip_force_publishers`、`sdk_managers_whitelist` |

### `py_tree.json` 顶层字段

| 字段 | 含义 | 必填 |
|---|---|---|
| `tree` | 根节点定义 | 是 |
| `interface` | 对外可配置参数声明（`inputs` / `outputs` / `custom` / `process`） | 建议 |

### 树节点字段

| 字段 | 含义 |
|---|---|
| `name` | 复合节点名 / 子树文件名（`.json` 结尾）/ 叶子节点类名 |
| `label` | 自由命名，日志与前端显示 |
| `params` | 节点参数 |
| `childs` | 子节点数组；叶子节点为 `[]` |
| `childBoard` | 子黑板项；叶子节点为 `[]` |

### `name` 的四种解析

```text
"name": "Sequence"            → 复合节点（只支持 Sequence / Selector / Parallel）
"name": "Async"               → 装饰器（必须包含 childs）
"name": "look_for_tag.json"   → 引用子树（key 必须定义在 py_tree_child.json）
"name": "CalcLegMove"         → 叶子节点（映射到 orchestration/nodes/ 下的类）
```

叶子节点的类名匹配由 `BehaviorTreeFactory._build_node_index()` 建立：文件名 `arm_joint_traj_sdk_move.py` → `_snake_to_pascal("arm_joint_traj_sdk_move")` → 类名 `ArmJointTrajSdkMove`。**推荐直接用 Python 类名。**

### 参数结构（`params` 的 value/source/data_type）

```json
"yaw_deg": {
  "value": "30.0",
  "source": "CUSTOM",
  "data_type": "float"
}
```

- `data_type` 支持 `float` / `int` / `string` / `bool` / `list` / `json`。
- 列表可直接写在 `value`：`"value": [12, 0, -12, 0]`，`"data_type": "list"`。
- `source` 取值：`CUSTOM`（固定值）、`READ_BOARD`（从黑板读，可用 `board_key` 指定键名）、`INPUT`（宏替换）、`RESOLVED`（替换后）。

### `interface.inputs` 与 `${宏}`

主树头部声明对外变量：

```json
{ "internal_name": "pick_tag_id", "type": "int", "required": false,
  "default": 4, "description": "取货侧箱子 Tag ID" }
```

取值优先级：**命令行启动覆盖参数 > board.json / 黑板已有值 > interface 的 default**。

解析出的值会替换树中 `"value": "${pick_tag_id}"` 且 `source: INPUT` 的节点参数。因此**日常修改 Tag ID 或箱子尺寸，只改 `board.json` 即可**。

### 复合节点策略（`Sequence` / `Selector` / `Parallel`）

| 复合节点 | 语义 | 关键参数 |
|---|---|---|
| `Sequence` | 顺序执行子节点，全部成功才成功 | `memory`（省略时默认 `False`） |
| `Selector` | 依次尝试子节点，直到一个成功 | `memory`（省略时默认 `False`） |
| `Parallel` | 并发 tick 子节点，按 `policy` 汇总 | `policy`：`success_on_all` / `success_on_one` |

- `memory=True` 时 `Sequence` 会**跳过已成功的子节点**；`memory=False` 每拍都从第一个子节点重新检查。主流程必须**显式**设置 `memory=True`（省略时为 `False`），否则会出现「底盘走完下一拍又回头等待 Enter」。
- `source: CUSTOM` 的值会**原样**传给复合节点，`data_type` 不做转换（`behavior_tree_factory.py` 的 `_create_composite_node` 直接读取 `memory`）。因此：
  - 推荐写成 JSON 布尔值：`"memory": {"value": true, "source": "CUSTOM", "data_type": "bool"}`；
  - 仓库中常见的字符串写法 `"value": "True"` 可以生效，因为非空字符串为真；
  - **字符串 `"False"` 同样为真，等同于 `memory=True`**。要关闭 memory，写 JSON 布尔值 `false` 或省略该参数。
- `policy` / `memory` 是复合节点的**参数**，不是新的节点类型。

### 装饰器

| 装饰器 | 文件 | 关键参数 | 用途 |
|---|---|---|---|
| `Async` | `async_decorator.py` | `tick_hz`（默认 `50.0`） | 子节点放后台线程 tick，配合 `Parallel` 实现编排层真并行启动 |
| `ForEach` | `for_each.py` | `source_key`、`target_key` | 对黑板列表逐项执行同一子树 |
| `RepeatUntil` | `repeat_until.py` | `condition_key`（必填）、`condition_path`、`expected_value`（默认 `true`）、`wait_for_child_completion`、`check_before_iteration`、`max_iterations`（默认 `0`） | 重复成功子树直到条件满足或达到成功轮数上限；子树失败立即返回 FAILURE |
| `RunIfIndex` | `run_if_index.py` | `index`（默认 `1`）、`nav_point_index_key`（默认 `nav_point_index`） | 按索引门控 |
| `PressureDropGuard` | `pressure_drop_guard.py` | — | 压力下降保护 |

`RepeatUntil.max_iterations` 必须为非负整数，`0` 表示无次数上限（保留既有默认行为），`N>0` 表示子树成功完成 N 轮后返回 `SUCCESS`，不是 tick 数或尝试次数。条件和次数上限任一满足即可退出；失败轮不计数、不重试。`iterations` 保存本次调用已经成功完成的轮数，装饰器重新进入时归零。

默认条件可中断运行中的子树；若要完整执行当前轮再停止，启用 `wait_for_child_completion=true`，并用 `check_before_iteration=true` 在首轮/下一轮启动前检查停止条件。次数上限只在子树成功完成时检查，不中断半轮。JSON 中 `max_iterations` 可使用 `READ_BOARD`，构树时解析，修改轮数需重新加载；停止条件从运行时黑板读取。已有场景不传新参数仍保持条件循环，无次数限制。

> `Async` 只保证**编排层**不阻塞。它**不保证底层硬件真能同时执行两条控制链路**；多线程下黑板与 ROS/SDK 调用的线程安全需自行评估。

### `hardware_config.json`

非行为树字段。真机路径下、创建 `IHardware` 之前会被 merge 进硬件工厂配置：

```json
{ "skip_force_publishers": true, "sdk_managers_whitelist": ["timed", "arm", "low"] }
```

## 05.04 返回值

### 节点状态（三态）

每个节点每拍返回 `py_trees.common.Status` 之一：

| 状态 | 含义 | 树的行为 |
|---|---|---|
| `SUCCESS` | 本节点已完成 | 父节点按自身策略决定推进 / 结束 |
| `FAILURE` | 本节点失败 | 父节点按自身策略决定换分支 / 整体失败 |
| `RUNNING` | 仍在进行 | 父节点保持该子节点，下一拍继续 tick |

叶子节点的典型写法：`skill.is_finished()` 为真 → `SUCCESS`，否则 `RUNNING`。长时间 `RUNNING` 通常是技能在等硬件到位（如运动插值中），**不是死循环**。

### 进程退出码（`orchestration/main.py`）

主入口在非 dry-run 运行时把根节点终态映射为进程退出码：

| 根节点终态 | 日志 | 退出码 |
|---|---|---|
| `SUCCESS` | `studio_smoke 完成，进程退出 (0)` | `0` |
| `FAILURE` | `行为树 FAILURE，进程退出 (1)` | `1` |
| 其它（未达终态） | `行为树未达终态 (%s)，进程退出 (2)` | `2` |
| 找不到行为树 JSON | `错误: 找不到行为树 JSON: …` | `1` |

> **两个不同的系统，别混用**：这里的退出码是 `orchestration/main.py` 的进程退出码。`--dry-run` 时不适用：加载失败以非 0 退出，加载成功后总是 0，tick 结果看输出中的「首次 tick 后根状态」。二开脚本里常见的「读 `tree.log` 最后一行 `完成: SUCCESS|FAILURE`」是另一套判定，两者判据不同。

## 05.05 异常

行为树加载期会把结构问题**打印成明确错误**，而不是抛异常给调用方：

| 报错 | 触发条件 | 处理 |
|---|---|---|
| `Unknown composite node type` | `name` 写成不支持的复合类型 | 只用 `Sequence` / `Selector` / `Parallel` |
| `[V3.0 Lite Interface Error] 必填参数缺失` | `interface` 必填项既无 override、也无黑板值、又无 default | 补 override / 黑板值 / default |
| `[V3.0 Lite Type Error]` | 值无法转换为声明的 `data_type` | 核对值格式 |
| `[V3.0 Lite Enum Error]` | 值不在 `options` 允许列表 | 改用允许值之一 |
| `[V3.0 Lite Macro] 警告: 变量未在 interface 中定义` | 用了 `${var}` 但没在 `interface.inputs` 声明 | 补声明或改变量名 |
| 主树 `py_tree.json` 不存在 | `--scenario` 目录下缺文件 | 核对目录与文件名 |
| `root 不存在`（dry-run 加载失败） | 树构建失败（类名拼错 / 子树 key 不存在） | 按上面的报错定位 |
| py_trees 版本不兼容 | 未先导入兼容层 | 确认入口先 `import py_trees_compat` |

`orchestration/nodes/skill_node.py` 的 `SkillNode` 与 `orchestration/engine/behavior_tree_engine.py` 都是**遗留**实现，不要在新代码里使用。

## 05.06 调用示例

### 运行

```bash
cd /path/to/LeTools
export PYTHONPATH=$PWD:$PYTHONPATH
export ROBOT_VERSION=62  # 62 夹爪轮臂示例；按目标机型调整

# 干跑：只校验 import / JSON / 节点类名，不连 ROS 与硬件
python3 orchestration/main.py --dry-run --tick-once

# 真机（默认场景 studio_smoke_v1）
python3 orchestration/main.py

# 指定场景
python3 orchestration/main.py \
  --tree orchestration/scenarios/refactored_sdk_atomic_v1/py_tree.json \
  --board orchestration/scenarios/refactored_sdk_atomic_v1/board.json
```

命令行参数：

| 参数 | 说明 |
|---|---|
| `--dry-run` | 不 `rospy.init_node`、不连硬件；设置 `STUDIO_DRY_RUN=1` |
| `--tick-once` | 仅与 `--dry-run` 合用；加载后只 tick 一次 |
| `--spin` | 树结束后 `rospy.spin()` 常驻（默认关闭） |
| `--tree` / `--board` | 指定主树 / 黑板 JSON 路径 |

`apps/test_upper_init/run_behavior_tree_json.py` 是功能更全的启动器，额外支持 `--scenario`、`--subtrees`、`--hardware-config`、`--action-groups`（逗号分隔动作组，如 `1,3,5`）、`--parallel-load`。

### 最小主树

```json
{
  "tree": {
    "name": "Sequence",
    "label": "demo_seq",
    "params": { "memory": { "value": "True", "source": "CUSTOM", "data_type": "string" } },
    "childs": [
      { "name": "WaitForEnter", "label": "wait_enter",
        "params": { "message": { "value": "按 Enter 开始", "source": "CUSTOM", "data_type": "string" } },
        "childs": [], "childBoard": [] }
    ],
    "childBoard": []
  },
  "interface": { "inputs": [], "outputs": [], "custom": [], "process": [] }
}
```

### 新建场景标准流程

1. **复制基础模板**：复制 `studio_smoke_v1` 或 `refactored_sdk_single_tag_pick_v0`，不要直接改产线真机树。
2. **配置黑板参数**：改 `board.json` 的 Tag ID、位姿目标、几何尺寸。
3. **编排主树**：调整 `py_tree.json` 的 `childs` 顺序，确认每个 `name` 都已实现。
4. **抽离重复逻辑**：多次复用的段落写进 `py_tree_child.json`。
5. **DryRun 校验**：`python3 orchestration/main.py --tree <场景>/py_tree.json --dry-run --tick-once`。
6. **仿真验证 → 真机迁移**：仿真通过后才上真机。

### 新增一个动作节点（三层）

1. **技能**：`skills/atomic/refactored_sdk/<feature>.py` 定义 `<Feature>Params` 与 `<Feature>Skill`（见 `04-skills-原子技能.md`）。
2. **节点**：`orchestration/nodes/<feature>_move.py` 继承 `BaseAction`，实现 `<Feature>Move`。
3. **JSON**：场景里 `"name": "<Feature>Move"`。

节点模板：

```python
class XxxMove(BaseAction):
    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._skill = None
        self._dry_done = False

    def initialise(self):
        # 解析 params → 构造 SkillParams → 建 Skill → skill.initialize()
        ...

    def update(self):
        # skill.execute() / skill.is_finished() → SUCCESS / FAILURE / RUNNING
        ...
```

### 环境分档

| 档位 | 启动方式 | 能验证什么 |
|---|---|---|
| 干跑 | `--dry-run [--tick-once]` | 树能否加载、节点类名是否匹配、参数与黑板连线是否通 |
| MuJoCo 仿真 | 启动 ROS + MuJoCo 后正常运行 | 关节 / 手臂 / 头 / 躯干 / 底盘的运动链路、IK 与轨迹下发 |
| 真机 | 同上 + 下位机 | 全量（真实相机 / 末端 / 力控） |

两点澄清：

1. **干跑不是仿真**——它把运动节点整体短路，不触发 IK 与轨迹下发。干跑通不代表能仿真。
2. **仿真不是缩小的真机**——JiBot 底盘的三个节点走 `leju_mobile_base_msgs` 的 `BaseMove` / `MoveToTarget` / `CheckArrived` ROS 服务，仿真环境没有这套服务，含 JiBot 节点的场景仿真下无法直接运行。

判定场景属于哪一档，**按节点类名扫**（不要按文件名）：

```bash
grep -rho '"name"[[:space:]]*:[[:space:]]*"[^"]*"' \
  orchestration/scenarios/<场景>/*.json | sort -u
```

- 出现 `*JibotMove` → 真机 + JiBot；
- 出现 `*Percep` / `*Tag*` / `Apriltag*` / `Inference*` → 需真实相机或外部视觉服务；
- 力控 / 真空 / 爪类 → 需真实末端；
- 只有 `*SdkMove` → 仿真 / 真机均可；
- 任意组合 → 均可干跑。

`dismantle_box_demo_action` 的子树叫 `demo_jibot_nav.json`，但里面全是纯 SDK 节点，实际不依赖 JiBot——这就是「按类名判」的反例。

## 05.07 节点清单与口径说明

90 / 56 / 57 三个数字口径，以及 56 个公共节点的**作用 / 参数 / 底层**逐节点说明，已独立成篇：**[11-节点清单.md](./11-节点清单.md)**。
