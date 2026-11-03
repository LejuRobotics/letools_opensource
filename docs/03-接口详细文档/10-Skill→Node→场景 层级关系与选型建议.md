# 10 Skill→Node→场景 层级关系与选型建议

> 新增一个能力时，最先要回答的不是「代码怎么写」，而是「**它该落在哪一层**」。落错层会导致后面每一步都别扭：要么在 `py_tree.json` 里堆逻辑，要么让 Skill 依赖行为树状态。
>
> 分工：本篇讲「**选哪个层级**」，[`09-控制路径与选型.md`](./09-控制路径与选型.md) 讲「**选哪条路径**（标准 / SDK / TimedCmd）」。**选型时先定层级，再定路径。**

## 10.01 三层是什么

```text
Skill（实现 ISkill）        ← 「一个动作」的封装，可被任何调用方复用
  ↑ 被调用
Node（继承 Behaviour）      ← 「动作在行为树里的一次实例」，薄封装
  ↑ 被 JSON 引用
场景（py_tree.json）        ← 「什么时候、按什么顺序调用哪个 Node」
```

| 层 | 代码位置 | 基类 | 输入 | 输出 | 是否知道行为树 |
|---|---|---|---|---|---|
| Skill | `skills/atomic/refactored_sdk/<feature>.py` | `SkillBase`（实现 `ISkill`） | `SkillParams` | `Result` + `is_finished()` | **不知道**（零 py_trees 依赖） |
| Node | `orchestration/nodes/<feature>_move.py` | `BaseAction`（`orchestration.nodes.base_node`，继承 `py_trees.behaviour.Behaviour`） | `(name, label, namespace, params)` dict | `py_trees.common.Status` | 知道 |
| 场景 | `orchestration/scenarios/<场景>/py_tree.json` | — | — | — | 就是它 |

三层的职责边界：

- **Skill** 只做业务逻辑：参数校验 → 调 `hardware.xxx()` → 判断何时结束。**不直接发 ROS 话题、不直接碰 SDK、不 import py_trees。**
- **Node** 只做翻译：把 JSON 参数转成 `XxxParams`、把 `Result` + `is_finished()` 转成 `Status.SUCCESS/FAILURE/RUNNING`。**不含业务逻辑。**
- **场景** 只做编排：顺序 / 分支 / 并发 / 传参。**不写代码。**

Node 的翻译逻辑就是这三行（`orchestration/nodes/head_control_sdk_move.py` 逐字）：

```python
def initialise(self):
    skill_params = HeadControlSdkParams(
        yaw_deg=float(self.params.get("yaw_deg", 0.0)),
        pitch_deg=float(self.params.get("pitch_deg", 0.0)),
    )
    self._skill = HeadControlSdkSkill(hardware=get_shared_hardware())
    result = self._skill.initialize(skill_params)

def update(self):
    if self._skill.is_finished():
        return Status.SUCCESS
    result = self._skill.execute()
    if not result.success:
        self.feedback_message = result.message or "head_control_sdk failed"
        return Status.FAILURE
    return Status.RUNNING
```

节点不自己造硬件 —— 从 `orchestration.shared_hardware.get_shared_hardware()` 取单例，保证整棵树共用同一份硬件对象。

## 10.02 决策树：新增能力落在哪一层

```text
要加的东西是什么？
│
├─ 只是「换参数」？（同一个动作，改数值/换目标）
│   → 只改场景 JSON
│     例：把某个节点 yaw_deg 从 20 改成 -20；换一个 tag_id
│     不写任何 Python。
│
├─ 需要「新的编排结构」？（组合已有节点，但顺序/并发/分支是新的）
│   → 只改场景 JSON + py_tree_child.json
│     例：新增一条「先动头再看 tag」的子树
│     不写任何 Python。
│
├─ 需要「一个新的动作」？
│   ├─ 这个动作会被多个场景 / 多次复用？
│   │   → 写 Skill（+ 对应的 Node）
│   └─ 只在当前场景出现一次，且是一次性验证？
│       → 写 apps/ 下的验证脚本，直接调 hw.xxx()
│         （不进行为树，不写 Skill/Node）
│
└─ 需要「一个新的控制原语」？（hw 上还没有这个方法）
    → 先改 adapters 层（见 03-adapters-硬件适配层.md），
      再回到上面判断要不要 Skill/Node。
```

四层落点的判据：

| 落点 | 什么时候 | 改动量 |
|---|---|---|
| 纯改场景 JSON | 动作集合不变，只是参数或组合方式变 | 0 行 Python |
| 新增 Node | 已有 Skill，但行为树里还没有它的节点 | 1 个薄文件 |
| 新增 Skill + Node | 新动作，且需要被行为树复用 | 2 个文件 |
| 只写 apps/ 脚本 | 一次性验证，不进生产流程 | 1 个脚本，不进行为树 |
| 改 adapters | 新的硬件控制原语 | 改 mixin（见 `03`） |

**「能复用则沉到 Skill」的含义**：只要一个动作可能被第二个场景用到，就先写成 Skill（哪怕当前只有一个场景用）。反过来，只在一个场景里出现、且逻辑是「A 做完做 B」这种纯编排，就**不要**写成一个 Skill —— 那是场景 JSON 的职责。

## 10.03 落错层的典型症状

| 症状 | 说明落错在哪 | 正确落点 |
|---|---|---|
| `py_tree.json` 里出现同一串 5 个节点被复制三遍 | 编排复用没做 | 抽成 `py_tree_child.json` 子树 |
| Skill 里 `import py_trees` | 逻辑写进了 Skill | 挪到 Node |
| Skill 里 `rospy.Publisher(...)` 直接发话题 | 越过了 adapters | 挪到 adapters 的 mixin |
| Node 的 `update()` 里判断「电流是否超阈值」 | 业务逻辑写进了 Node | 挪到 Skill |
| 场景里为了「换个固定角度」新建了一个 Node 类 | Node 粒度太细 | 用 `params`，纯改 JSON |
| 一个 Skill 里塞了「抓取 + 搬运 + 放置」三步 | Skill 粒度太粗 | 拆成三个 Skill，用场景串联 |
| Node 的 `__init__` 里 `LejuWheeledArmHardware()` | 绕开了共享单例 | 用 `get_shared_hardware()` |

## 10.04 新增一个能力的完整流程（Skill + Node）

以「新增一个『把头部转回原位』的动作」为例：

```text
① adapters 层确认能力存在
   hw.control_head_sdk(yaw=0, pitch=0) 已在 → 无需改 adapters

② 写 Skill：skills/atomic/refactored_sdk/head_control_sdk.py
   （若已有同类技能，直接复用，跳到 ③）

③ 写 Node：orchestration/nodes/head_control_sdk_move.py
   - @define_manifest(label=..., category=[...], tree_type=...,
                      params=[{"name": "yaw_deg", "type": "float", ...}])
   - 继承 BaseAction
   - initialise()：params → XxxParams → Skill(hardware=get_shared_hardware())
   - update()：is_finished() → SUCCESS / execute() → FAILURE 或 RUNNING

④ 场景 JSON 里引用
   { "name": "HeadControlSdkMove", "label": "head_home",
     "params": { "yaw_deg": {"value": "0.0", "source": "CUSTOM", "data_type": "float"} },
     "childs": [], "childBoard": [] }
```

关键点：

- **`name` 写类名**（`HeadControlSdkMove`），工厂会把它解析成 `orchestration/nodes/head_control_sdk_move.py` —— 索引构建时同时注册 snake_case 与 PascalCase 两种键（`_snake_to_pascal`）。
- **不要往 `skills/atomic/refactored_sdk/__init__.py` 加聚合导入**：行为树构建阶段会并行 import 叶子节点模块，聚合导入会放大 import 锁竞争，某些 Python 版本/线程时序下触发 `_DeadlockError`。
- 参数在 `@define_manifest` 里声明一次，前端节点面板与运行时校验都读它；`params=[...]` 的 `default` 类型要写成字符串（如 `"0.0"`），JSON 里 `value` 也是字符串。

## 10.05 三层与三条控制路径的交叉

层级与路径是**正交**的两个维度：

| | 层级（本篇，`10`） | 路径（[`09-控制路径与选型.md`](./09-控制路径与选型.md)） |
|---|---|---|
| 决定 | 代码放哪、由谁调度 | 动作最终怎么落到硬件 |
| 例 | 「写成 Skill 还是只写脚本」 | 「用 `send_ee_pose` 还是 `send_ee_pose_sdk`」 |
| 变化频率 | 加新能力时 | 单个动作内部 |

两者的连接点在 **Skill 内部**：

```python
# Skill 决定「调哪个适配器方法」= 路径选择
result = getattr(self.hardware, "send_arm_ee_world_timed", None)
```

`refactored_sdk/` 目录名下**同时**存在 `_sdk` 技能、`_timed` 技能和标准接口技能（`arm_ee_pose_jibot.py` 走 `send_ee_pose`）—— 目录名不代表路径。因此：

> **先按本篇定层级（Skill / Node / 场景），再按 `09` 在那个 Skill 里定路径。**

反过来也成立：一个 Skill 只要没规定死路径，就可以被不同 Node 复用在不同场景里 —— 这正是「能复用则沉到 Skill」的价值。

## 10.06 场景配置层能表达什么（不写代码的边界）

改场景 JSON 能做的：

| 能做 | 怎么做 |
|---|---|
| 换参数 | 改 `params.<name>.value` |
| 换顺序 | 调 `childs` 数组顺序（`Sequence` / `Selector`） |
| 加并发 | 套 `Parallel` + `Async` |
| 加重复 / 条件 | 套 `ForEach` / `RepeatUntil` / `RunIfIndex` |
| 复用子流程 | 抽到 `py_tree_child.json`，用 `"xxx.json"` 引用 |
| 换初值数据 | 改 `board.json` |
| 裁剪硬件 | 改 `hardware_config.json` |
| 参数从哪来 | 改 `params.<name>.source`（`CUSTOM` / `READ_BOARD` / `INPUT` / `RESOLVED`） |

改场景 JSON **不能**做的（必须动 Python）：

- 新增一个动作语义；
- 改变一个已有动作的内部行为；
- 跨节点的实时数据流计算（那属于 Skill 或黑板）。

场景文件格式、`name` 的四种解析、`${宏}` 优先级见 [`05-orchestration-行为树编排.md`](./05-orchestration-行为树编排.md)；`board.json` / `hardware_config.json` 逐字段见 [`08-工具层-脚本与配置.md`](./08-工具层-脚本与配置.md) 的 `08.03`。

## 10.07 速查表

| 我要做的事 | 改哪里 | 参考 |
|---|---|---|
| 换个角度 / 换个 tag / 换个速度 | `py_tree.json` 的 `params` | 本篇 `10.02` |
| 把一段重复流程抽出来 | `py_tree_child.json` + 主树引用 | `05.03` |
| 让两个动作并行 | `Parallel` + `Async` | `05.04` |
| 加一个能被复用的新动作 | 新增 Skill + Node | 本篇 `10.04` |
| 只是验证一下，不进生产 | `apps/` 下写脚本 | `06-apps-测试与示例应用.md` |
| 动作里要精确控时 | Skill 里改用 `_timed` 方法 | `09-控制路径与选型.md` |
| 动作里要 SDK 专有能力 | Skill 里改用 `_sdk` 方法 | `09-控制路径与选型.md` |
| hw 上还没有这个方法 | 先改 adapters mixin | `03-adapters-硬件适配层.md` |
| 加一种新末端执行器 | 先改 drivers，再改 adapters | [`07-drivers-与-SDK.md`](./07-drivers-与-SDK.md) |

> 相关：路径选型见 [`09-控制路径与选型.md`](./09-控制路径与选型.md)；Skill 基类与生命周期见 [`04-skills-原子技能.md`](./04-skills-原子技能.md)；节点的 manifest 与工厂解析见 [`05-orchestration-行为树编排.md`](./05-orchestration-行为树编排.md)。
