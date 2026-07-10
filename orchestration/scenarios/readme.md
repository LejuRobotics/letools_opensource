# 场景 JSON 编写指南

以 `refactored_sdk_atomic_v1` 为例。

---

## 一、JSON 文件写法

每个场景目录包含三个文件：

```
your_scenario/
├── board.json           ← 全局黑板初始数据
├── py_tree_child.json   ← 可复用子树模板集
└── py_tree.json         ← 主行为树（顶层流程）
```

### 1.1 board.json

黑板是行为树的全局共享存储。扁平 dict 格式，顶层 key 直接作为黑板键名，值可以是任意 JSON 原生类型。

```json
{
  "ArmJointTrajectories": {
    "times": [0.0, 3.0],
    "q_frames": [
      [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
      [-30.0, 20.0, 15.0, -45.0, 25.0, 10.0, -35.0, -30.0, -20.0, -15.0, -45.0, -25.0, -10.0, -35.0]
    ]
  }
}
```

节点中通过 `self.global_blackboard.<KeyName>` 读取。

### 1.2 py_tree.json（主树）

顶层 `tree` 节点定义执行流程。`name` 以 `.json` 结尾 → 子树引用；否则 → 原子节点类名。

```json
{
  "tree": {
    "name": "Sequence",
    "label": "refactored_sdk_atomic_v1_seq",
    "params": {
      "memory": { "value": "True", "source": "CUSTOM", "data_type": "string" }
    },
    "childs": [
      { "name": "WaitForEnter",  "label": "wait_enter",    "params": { ... }, "childs": [], "childBoard": [] },
      { "name": "demo_cmd_pose_base.json",     "label": "...", "params": {}, "childs": [], "childBoard": [] },
      { "name": "demo_leg_arm_parallel.json",  "label": "...", "params": {}, "childs": [], "childBoard": [] },
      { "name": "demo_head_control.json",      "label": "...", "params": {}, "childs": [], "childBoard": [] },
      { "name": "ArmResetSdkMove",             "label": "...", "params": {}, "childs": [], "childBoard": [] }
    ],
    "childBoard": []
  },
  "interface": { "inputs": [], "outputs": [], "custom": [], "process": [] }
}
```

### 1.3 py_tree_child.json（子树模板）

以子树文件名作 key，每个子树内结构与主树相同。

```json
{
  "demo_cmd_pose_base.json": {
    "tree": {
      "name": "Sequence",
      "label": "demo_cmd_pose_base_seq",
      "params": { "memory": { "value": "True", "source": "CUSTOM", "data_type": "string" } },
      "childs": [ ... ],
      "childBoard": []
    }
  },
  "demo_head_control.json": { ... },
  "demo_leg_arm_parallel.json": { ... }
}
```

### 1.4 节点 JSON 结构速查

#### 叶子节点（原子 Action）

```json
{
  "name": "BasePoseLocalMove",       // 必须 = orchestration/nodes/ 中的类名
  "label": "forward_0p5m",           // 自由命名，区分同类型实例
  "params": {
    "x": {
      "value": "0.5",                 // 参数值
      "source": "CUSTOM",             // CUSTOM = 固定值 / READ_BOARD = 从黑板读
      "data_type": "float"            // float / int / string / bool / json
    }
  },
  "childs": [],                       // 叶子节点必须为空
  "childBoard": []                    // 叶子节点必须为空
}
```

#### 复合节点

| `name` | 行为 | 特有 params |
|--------|------|------------|
| `"Sequence"` | 顺序执行所有 childs | `memory` (string, "True"/"False") |
| `"Selector"` | 依次尝试 childs，直到一个成功 | `memory` |
| `"Parallel"` | 同时 tick 所有 childs | `policy` ("success_on_all" / "success_on_one") |

#### Async 装饰器 + Parallel（实现真并行）

`Async` 是自定义装饰器（`orchestration/nodes/async_decorator.py`），在独立线程中 tick 子节点。配合 Parallel 使用可实现腿臂同时运动：

```json
{
  "name": "Parallel",
  "label": "phase1",
  "params": { "policy": { "value": "success_on_all", "source": "CUSTOM", "data_type": "string" } },
  "childs": [
    {
      "name": "Async",
      "label": "leg_async",
      "params": { "tick_hz": { "value": "50.0", "source": "CUSTOM", "data_type": "float" } },
      "childs": [
        { "name": "LegJointSdkMove", "label": "leg_zero", "params": { ... }, "childs": [], "childBoard": [] }
      ],
      "childBoard": []
    },
    {
      "name": "Async",
      "label": "arm_async",
      "params": { "tick_hz": { "value": "50.0", "source": "CUSTOM", "data_type": "float" } },
      "childs": [
        { "name": "ArmJointTrajSdkMove", "label": "arm_spread", "params": { ... }, "childs": [], "childBoard": [] }
      ],
      "childBoard": []
    }
  ],
  "childBoard": []
}
```

#### 数组参数

直接在 `value` 中写 JSON 数组：

```json
"joint_traj": {
  "value": [[0.0, 0.0, ...], [-30.0, 20.0, ...]],
  "source": "CUSTOM",
  "data_type": "string"
}
```

### 1.5 JSON 字段规则

| 字段 | 规则 |
|------|------|
| `name` | 原子节点 = 节点类名（精确匹配大小写）；子树引用 = `"文件名.json"` |
| `label` | 任意字符串，日志/前端显示用 |
| `params.<key>.source` | `"CUSTOM"` 固定值 / `"READ_BOARD"` 从黑板读 / `"RESOLVED"` 宏替换后 |
| `params.<key>.data_type` | `float` / `int` / `string` / `bool` / `json` |
| `childs` | 叶子节点 = `[]`，复合节点 = 子节点数组 |
| `childBoard` | 叶子节点 = `[]` |

---

## 二、原子技能 → 节点类 → JSON（示例：head_control_sdk）

以 `skills/atomic/refactored_sdk/head_control_sdk.py` 为例，说明一个功能如何从技能写到 JSON。

### 第 1 层：原子技能 HeadControlSdkSkill

**文件**：[skills/atomic/refactored_sdk/head_control_sdk.py](skills/atomic/refactored_sdk/head_control_sdk.py)

```python
from skills.base.skill_base import SkillBase

# ① 参数 dataclass —— 字段 = JSON 中可传的参数
@dataclass
class HeadControlSdkParams(SkillParams):
    skill_name: str = "head_control_sdk"
    yaw_deg: float = 0.0        # 偏航角（度）
    pitch_deg: float = 0.0      # 俯仰角（度）
    timeout: float = 30.0

# ② 技能实现 —— 继承 SkillBase
class HeadControlSdkSkill(SkillBase):
    def __init__(self, hardware: IHardware):
        super().__init__(name="head_control_sdk")
        self.hardware = hardware

    def on_initialize(self, params: HeadControlSdkParams) -> Result:
        self.params = params
        self._done = False
        return Result.ok()

    def on_execute(self) -> Result:
        result = self.hardware.control_head_sdk(   # ← 最终调 adapter
            yaw=float(self.params.yaw_deg),
            pitch=float(self.params.pitch_deg),
        )
        self._done = True
        return result

    def on_is_finished(self) -> bool:
        return self._done
```

技能只做：**校验参数 → 调 adapter → 标记完成**。不关心行为树和 JSON。

### 第 2 层：原子节点 HeadControlSdkMove

**文件**：[orchestration/nodes/head_control_sdk_move.py](orchestration/nodes/head_control_sdk_move.py)

```python
from orchestration.nodes.base_node import BaseAction  # 继承 py_trees.Behaviour
from orchestration.shared_hardware import get_shared_hardware

# ③ @define_manifest —— 声明节点参数（供前端/文档用）
@define_manifest(
    label="头部控制（SDK）",
    category=["motion", "head"],
    tree_type="studio_smoke",
    description="调用 hardware.control_head_sdk(yaw, pitch)",
    params=[
        {"name": "yaw_deg",   "type": "float", "default": "0.0", "description": "偏航角（度）"},
        {"name": "pitch_deg", "type": "float", "default": "0.0", "description": "俯仰角（度）"},
    ],
)
class HeadControlSdkMove(BaseAction):
    def initialise(self):
        # ④ 从 JSON params 取值 → 构造技能参数
        skill_params = HeadControlSdkParams(
            yaw_deg=float(self.params.get("yaw_deg", 0.0)),
            pitch_deg=float(self.params.get("pitch_deg", 0.0)),
        )
        # ⑤ 创建技能，注入硬件单例
        self._skill = HeadControlSdkSkill(hardware=get_shared_hardware())
        self._skill.initialize(skill_params)

    def update(self):
        # ⑥ 每个 tick：检查完成 → 执行 → 返回状态
        if self._skill.is_finished():
            return Status.SUCCESS
        result = self._skill.execute()
        if not result.success:
            return Status.FAILURE
        return Status.RUNNING
```

节点是 **JSON ↔ 技能** 的桥梁。`self.params.get("yaw_deg")` 拿到的就是 JSON 中 `"yaw_deg"` 的 `"value"`。

### 第 3 层：JSON 中使用

在树的 `childs` 中写：

```json
{
  "name": "HeadControlSdkMove",    // ⑦ 必须 = 节点类名
  "label": "center",               // ⑧ 自由取名
  "params": {
    "yaw_deg": {                   // ⑨ key 必须 = self.params.get("yaw_deg")
      "value": "0.0",
      "source": "CUSTOM",
      "data_type": "float"
    },
    "pitch_deg": {
      "value": "0.0",
      "source": "CUSTOM",
      "data_type": "float"
    }
  },
  "childs": [],
  "childBoard": []
}
```

同一个节点类可实例化多次，传不同参数：

```json
{ "name": "HeadControlSdkMove", "label": "center",       "params": { "yaw_deg": {"value": "0.0"},  ... } },
{ "name": "HeadControlSdkMove", "label": "look_left_30", "params": { "yaw_deg": {"value": "30.0"}, ... } },
{ "name": "HeadControlSdkMove", "label": "look_right_30","params": { "yaw_deg": {"value": "-30.0"},... } },
{ "name": "HeadControlSdkMove", "label": "scan_center",  "params": { "yaw_deg": {"value": "0.0"},  ... } }
```

### 数据流全景

```
JSON                               Node                             Skill
─────────────────────────────────────────────────────────────────────────────
"name": "HeadControlSdkMove" ──→  importlib 导入类
"params": {
  "yaw_deg": {"value":"30.0"} ──→  self.params.get("yaw_deg") → 30.0
                                     ↓
                                   HeadControlSdkParams(         ──→  params.yaw_deg=30.0
                                     yaw_deg=30.0, pitch_deg=0.0
                                   )
                                     ↓
                                   HeadControlSdkSkill(hardware)  ──→  self.hardware
                                     ↓
                                   skill.initialize(params)       ──→  on_initialize()
                                     ↓
                                   skill.execute()                ──→  on_execute():
                                                                         hardware.control_head_sdk(
                                                                           yaw=30.0, pitch=0.0
                                                                         )
                                                                           ↓
                                                                      Adapter → SDK → 机器人
```

### 三层对应关系统一表

| 层级 | 文件 | 关键代码 | 作用 |
|------|------|---------|------|
| JSON | `py_tree_child.json` | `"name": "HeadControlSdkMove"` | 声明用哪个节点类 |
| JSON | `py_tree_child.json` | `"yaw_deg": {"value": "30.0"}` | 声明传入的参数值 |
| 节点 | `head_control_sdk_move.py` | `class HeadControlSdkMove` | JSON 的 `name` **必须等于**这个类名 |
| 节点 | `head_control_sdk_move.py` | `self.params.get("yaw_deg", 0.0)` | 从 JSON 取值，key 必须匹配 |
| 节点 | `head_control_sdk_move.py` | `HeadControlSdkParams(yaw_deg=...)` | 构造技能参数对象 |
| 技能 | `head_control_sdk.py` | `class HeadControlSdkParams` | 参数 dataclass，定义字段与类型 |
| 技能 | `head_control_sdk.py` | `hardware.control_head_sdk(...)` | 最终调用的 adapter 方法 |

### 新增功能的标准步骤

以 `head_control_sdk` 为模板，新增一个功能需要改 3 个地方：

```
□ 1. 创建原子技能
     skills/atomic/refactored_sdk/<功能名>.py
       ├── <功能名>Params(SkillParams)   ← dataclass，字段 = 可传参数
       └── <功能名>Skill(SkillBase)
             ├── on_initialize() → 校验参数
             ├── on_execute()    → self.hardware.<adapter方法>(...)
             └── on_is_finished()→ return self._done

□ 2. 创建节点类
     orchestration/nodes/<功能名>_move.py
       ├── @define_manifest(params=[...])   ← key 必须与 Params 字段对应
       └── class <功能名>Move(BaseAction)
             ├── initialise() → self.params.get("xxx") → Params → Skill
             └── update()     → skill.execute() → 返回 Status

□ 3. 在 JSON 中使用
     └── { "name": "<功能名>Move",
           "params": { "xxx": {"value":"1.0", "source":"CUSTOM", "data_type":"float"} } }
```

**关键约束**：

| 约束 | 说明 |
|------|------|
| `JSON.name` = `Node 类名` | 精确匹配、含大小写；不是文件名，不是技能类名 |
| `JSON.params 的 key` = `Node 中 self.params.get("xxx")` | 节点取值的 key 即 JSON 中 params 的 key |
| `JSON params` → `Node._parse_params` → `SkillParams` | 值从 JSON → ParamsWrapper → dataclass 字段 |
| 叶子节点 `childs` 和 `childBoard` | 必须为 `[]` |
