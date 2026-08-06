# refactored_sdk_atomic_v1 场景构建说明

## 概述

本场景是一个综合演示案例，展示了机器人的**底盘 → 腿臂并行 → 头部 → 手臂复位**全流程运动能力。本文档从底层到顶层，逐层说明三个 JSON 配置文件的构建链路。

## 整体架构

```
┌─────────────────────────────────────────────────────────┐
│  行为树 JSON 编排层 (py_tree.json + py_tree_child.json)  │  ← 你在这里定义"做什么、按什么顺序"
├─────────────────────────────────────────────────────────┤
│  原子节点层 (orchestration/nodes/*.py)                    │  ← 每个节点 = py_trees Behaviour
├─────────────────────────────────────────────────────────┤
│  原子技能层 (skills/atomic/refactored_sdk/*.py)          │  ← 每个技能 = 对 IHardware 的一次调用封装
├─────────────────────────────────────────────────────────┤
│  硬件适配层 (adapters/hardware/leju_wheeled/*.py)        │  ← IHardware 接口实现，多 Mixin 组装
├─────────────────────────────────────────────────────────┤
│  SDK 管理层 (adapters/hardware/leju_wheeled/services/sdk_manager/) │  ← ArmSDKManager / LowLevelSDKManager
├─────────────────────────────────────────────────────────┤
│  Kuavo Humanoid SDK (kuavo_humanoid_sdk)                 │  ← robot_sdk.control.* 底层 API
└─────────────────────────────────────────────────────────┘
```

---

## 第一层：Adapter（硬件适配层）

**代码位置**：[adapters/hardware/](adapters/hardware/)

适配层是整个系统的硬件抽象入口。通过**工厂模式**创建硬件实例：

```python
# orchestration/shared_hardware.py
from adapters.hardware.factory import HardwareFactory
_hardware = HardwareFactory.create_hardware(config={'robot_type': 'leju_wheeled'})
```

`LejuWheeledArmHardware` 通过 **Mixin 多继承** 组装能力：

| Mixin | 职责 | 关键 SDK 方法 |
|-------|------|--------------|
| `SDKControlMixin` | SDK 直调（_sdk 后缀方法） | `control_head_sdk()`, `send_arm_joint_traj_sdk()`, `send_leg_joint_sdk()`, `send_base_position_local_sdk()`, `arm_reset()` |
| `BaseControlMixin` | 底盘速度/位置控制 | `send_base_pose()` |
| `ArmControlMixin` | 手臂末端/关节控制 | `send_arm_joint_trajectory()` |
| `TorsoControlMixin` | 躯干位姿控制 | `send_torso_pose()` |
| `LifecycleMixin` | 初始化/关闭 + SDK Manager 创建 | `initialize()`, `shutdown()` |

**调用链路**（以底盘相对位姿为例）：

```
send_base_pose(x=0.5, y=0, yaw=0, frame=LOCAL)
  → BaseControlMixin.send_base_pose()
    → _low_level_sdk_manager.control_base_position_local(target_pos=(0.5, 0, 0))
      → robot_sdk.control.control_base_position_local(...)   # Kuavo SDK 底层
```

---

## 第二层：Atomic Skill（原子技能层）

**代码位置**：[skills/atomic/refactored_sdk/](skills/atomic/refactored_sdk/)

每个原子技能是对 IHardware 某个方法的**薄封装**，继承自 `SkillBase` → `ISkill`。

### SkillBase 生命周期

```
initialize(params) → on_initialize()     # 校验参数
    ↓
execute()          → on_execute()        # 调用 hardware.xxx()（每个 tick 调用一次）
    ↓
is_finished()      → on_is_finished()    # 判别是否完成
```

### 本场景涉及的 7 个原子技能

| 技能文件 | 技能类 | 调用的 Adapter 方法 | 作用 |
|---------|--------|-------------------|------|
| [base_pose_local.py](skills/atomic/refactored_sdk/base_pose_local.py) | `BasePoseLocalSkill` | `hardware.send_base_pose(frame=LOCAL)` | 底盘本体系相对位姿移动 |
| [arm_joint_traj_sdk.py](skills/atomic/refactored_sdk/arm_joint_traj_sdk.py) | `ArmJointTrajSdkSkill` | `hardware.send_arm_joint_traj_sdk(joint_traj, total_time)` | 14 关节手臂轨迹 |
| [leg_joint_sdk.py](skills/atomic/refactored_sdk/leg_joint_sdk.py) | `LegJointSdkSkill` | `hardware.send_leg_joint_sdk(joint_angles, total_time)` | 4 关节腿部控制 |
| [head_control_sdk.py](skills/atomic/refactored_sdk/head_control_sdk.py) | `HeadControlSdkSkill` | `hardware.control_head_sdk(yaw, pitch)` | 头部 2 轴控制 |
| [arm_reset_sdk.py](skills/atomic/refactored_sdk/arm_reset_sdk.py) | `ArmResetSdkSkill` | `hardware.arm_reset()` | 手臂安全归位 |
| [torso_reset_sdk.py](skills/atomic/refactored_sdk/torso_reset_sdk.py) | `TorsoResetSdkSkill` | `hardware.reset_torso_to_initial()` | 躯干复位 |
| [wait_seconds.py](skills/atomic/refactored_sdk/wait_seconds.py) | `WaitSecondsSkill` | `time.sleep()` | 等待指定秒数 |
| [wait_for_enter.py](skills/atomic/refactored_sdk/wait_for_enter.py) | `WaitForEnterSkill` | `input()` | 等待键盘输入 |

### 技能实现示例：ArmJointTrajSdkSkill

```python
class ArmJointTrajSdkSkill(SkillBase):
    def on_initialize(self, params: ArmJointTrajSdkParams) -> Result:
        # 校验：必须 14 个关节，每个点必须是 list
        for point in params.joint_traj:
            if len(point) != 14: return Result.fail(...)
        self._done = False
        return Result.ok()

    def on_execute(self) -> Result:
        # 调适配器：一次调用，即刻 fire-and-forget
        result = self.hardware.send_arm_joint_traj_sdk(
            joint_traj=self.params.joint_traj,
            total_time=float(self.params.total_time),
        )
        self._done = True   # 单次执行，不需要循环
        return result

    def on_is_finished(self) -> bool:
        return self._done   # 一次执行后即视为完成
```

---

## 第三层：Atomic Node（原子节点层）

**代码位置**：[orchestration/nodes/](orchestration/nodes/)

每个原子节点是 `py_trees.behaviour.Behaviour` 的子类（通过 `BaseAction`），负责将技能接入行为树引擎。

### 节点基类：BaseAction

```python
class BaseAction(Behaviour):
    def __init__(self, name, label, namespace, params):
        super().__init__(name=name)
        self.label = label        # 用户可读标签
        self.params = params      # 从 JSON 解析的参数 (ParamsWrapper)
        self.global_blackboard = self.attach_blackboard_client()
```

### 节点生命周期（py_trees 标准）

```
__init__()        → 仅保存引用，不做初始化
initialise()       → 每次执行开始前调用：创建 Skill + SkillParams + initialize
update()           → 每个 tick 调用：skill.execute() → 返回 Status
terminate(status)  → （继承自 Behaviour）
```

### 节点实现模式（以 BasePoseLocalMove 为例）

```python
class BasePoseLocalMove(BaseAction):
    def initialise(self):
        # 1. 从 params（来自 JSON）构建 Skill 参数
        skill_params = BasePoseLocalParams(
            x=float(self.params.get("x", 0.5)),
            y=float(self.params.get("y", 0.0)),
            yaw=float(self.params.get("yaw", 0.0)),
            frame=FrameType.LOCAL,
        )
        # 2. 创建技能实例，注入硬件单例
        self._skill = BasePoseLocalSkill(hardware=get_shared_hardware())
        # 3. 初始化技能（校验参数）
        self._skill.initialize(skill_params)

    def update(self):
        # 4. 每个 tick：执行 → 判断状态
        if self._skill.is_finished():
            return Status.SUCCESS
        result = self._skill.execute()
        if not result.success:
            return Status.FAILURE
        return Status.RUNNING
```

### 节点元数据：@define_manifest 装饰器

每个节点通过装饰器声明其元数据，这些元数据用于：
- 前端编辑器渲染节点面板和参数表单
- 动态生成 `node_library.json` 供前端使用

```python
@define_manifest(
    label="底盘相对位姿（本体坐标）",         # 前端显示名称
    category=["motion", "chassis"],           # 分类
    tree_type="studio_smoke",                  # 行为树类型标签
    description="调用 hardware.send_base_pose(frame=LOCAL)",
    params=[
        {"name": "x",   "type": "float", "default": "0.5", "description": "前后位移（m）"},
        {"name": "y",   "type": "float", "default": "0.0", "description": "左右位移（m）"},
        {"name": "yaw", "type": "float", "default": "0.0", "description": "偏航（deg）"},
    ],
)
class BasePoseLocalMove(BaseAction):
    ...
```

### 本场景涉及的节点类

| 节点类 | 文件 | 对应技能 |
|--------|------|---------|
| `BasePoseLocalMove` | [base_pose_local_move.py](orchestration/nodes/base_pose_local_move.py) | `BasePoseLocalSkill` |
| `ArmJointTrajSdkMove` | [arm_joint_traj_sdk_move.py](orchestration/nodes/arm_joint_traj_sdk_move.py) | `ArmJointTrajSdkSkill` |
| `LegJointSdkMove` | [leg_joint_sdk_move.py](orchestration/nodes/leg_joint_sdk_move.py) | `LegJointSdkSkill` |
| `HeadControlSdkMove` | [head_control_sdk_move.py](orchestration/nodes/head_control_sdk_move.py) | `HeadControlSdkSkill` |
| `ArmResetSdkMove` | [arm_reset_sdk_move.py](orchestration/nodes/arm_reset_sdk_move.py) | `ArmResetSdkSkill` |
| `TorsoResetSdkMove` | [torso_reset_sdk_move.py](orchestration/nodes/torso_reset_sdk_move.py) | `TorsoResetSdkSkill` |
| `WaitSeconds` | [wait_seconds.py](orchestration/nodes/wait_seconds.py) | `WaitSecondsSkill` |
| `WaitForEnter` | [wait_for_enter.py](orchestration/nodes/wait_for_enter.py) | `WaitForEnterSkill` |

### 节点 — 技能 — 适配器调用关系

```
JSON 参数 params
    │
    ▼
Atomic Node (Behaviour)
    │  initialise(): 读取 params → 构建 SkillParams
    │  update():    调用 skill.execute()
    ▼
Atomic Skill (SkillBase)
    │  on_execute(): 调用 hardware.xxx()
    ▼
IHardware 接口 (LejuWheeledArmHardware)
    │  send_arm_joint_traj_sdk() / send_leg_joint_sdk() / ...
    ▼
SDK 管理服务（Adapter 内部：ArmSDKManager / LowLevelSDKManager）
    │  move_joint_traj_auto() / move_wheel_lower_joint_auto() / ...
    ▼
Kuavo Humanoid SDK (robot_sdk.control.*)
    │  底层电机/传感器 API
    ▼
    机器人硬件
```

---

## 第四层：JSON 编排层（py_tree.json + py_tree_child.json + board.json）

### 4.1 board.json — 全局黑板初始数据

**文件**：[board.json](board.json)

黑板（Blackboard）是 `py_trees` 的共享状态机制，所有节点都可读写。

```json
{
  "ArmJointTrajectories": {
    "times": [0.0, 3.0],
    "q_frames": [
      [0.0, 0.0, ..., 0.0],                     // 14个关节，起始位姿（全0=归位）
      [-30.0, 20.0, 15.0, ..., -35.0]            // 目标位姿（展开状态）
    ]
  }
}
```

- `ArmJointTrajectories` 被 `ArmJointTrajSdkMove` 节点通过黑板读取
- 当节点的 `use_board_trajectory = "true"` 时，`q_frames` 作为关节轨迹使用
- 运行时由 `apply_flat_board_json()` 将整个 dict 写入全局黑板

### 4.2 py_tree.json — 主行为树

**文件**：[py_tree.json](py_tree.json)

主树定义了**顶层执行流程**。每条记录代表一个行为树节点。

#### JSON 节点结构

```json
{
  "name": "节点类名 或 xxx.json（子树引用）",
  "label": "用户可读标签",
  "params": {
    "参数名": {
      "value": "参数值",
      "source": "CUSTOM",      // CUSTOM=固定值, INPUT=需宏替换, READ_BOARD=从黑板读取
      "data_type": "类型"       // int/float/string/bool/intArr/floatArr/json
    }
  },
  "childs": [],                 // 子节点列表（Composite 类型才有）
  "childBoard": []              // 子树黑板参数（可选）
}
```

#### 本场景主树结构

```json
{
  "tree": {
    "name": "Sequence",                              // 根节点：顺序执行
    "label": "refactored_sdk_atomic_v1_seq",
    "params": { "memory": { ... } },
    "childs": [
      { "name": "WaitForEnter", ... },               // ① 等待 Enter 确认
      { "name": "demo_cmd_pose_base.json", ... },    // ② 子树：底盘位姿
      { "name": "demo_leg_arm_parallel.json", ... },  // ③ 子树：腿臂并行
      { "name": "demo_head_control.json", ... },      // ④ 子树：头部控制
      { "name": "ArmResetSdkMove", ... }              // ⑤ 手臂安全复位
    ]
  }
}
```

**设计要点**：
- `name` 为 `xxx.json` 的节点 → 子树引用，由 `_handle_subtree()` 从 `py_tree_child.json` 中查找并深拷贝构建
- `name` 为 Python 类名的节点 → 原子 Action，由 `_create_node_instance()` 动态导入
- `source: "CUSTOM"` 的参数 → 直接使用 `value`，不经过黑板/宏替换
- 所有节点的 `childs: []` + `childBoard: []` 表示叶子节点

### 4.3 py_tree_child.json — 子树定义集

**文件**：[py_tree_child.json](py_tree_child.json)

包含 3 个子树定义，每个子树是一个独立可复用的行为树模板。

#### 子树 1：demo_cmd_pose_base.json（底盘位姿移动）

```
Sequence (memory=true)
  ├─ BasePoseLocalMove label="forward_0p5m"    → x=0.5,  y=0.0, yaw=0.0
  ├─ BasePoseLocalMove label="backward_0p3m"   → x=-0.3, y=0.0, yaw=0.0
  ├─ BasePoseLocalMove label="left_0p3m"       → x=0.0,  y=0.3, yaw=0.0
  ├─ BasePoseLocalMove label="right_0p3m"      → x=0.0,  y=-0.3, yaw=0.0
  ├─ BasePoseLocalMove label="rotate_ccw_90deg"→ x=0.0,  y=0.0, yaw=90.0
  ├─ BasePoseLocalMove label="rotate_cw_90deg" → x=0.0,  y=0.0, yaw=-90.0
  ├─ BasePoseLocalMove label="full_rotation_360deg" → x=0.0, y=0.0, yaw=360.0
  └─ BasePoseLocalMove label="combined"        → x=0.3,  y=0.0, yaw=45.0
```

#### 子树 2：demo_leg_arm_parallel.json（腿臂并行运动）

```
Sequence (memory=true)
  ├─ Parallel (success_on_all, "phase1")         ← 腿+臂 同时归零/展开
  │   ├─ Async → LegJointSdkMove   (leg_zero_3s)   腿部回零
  │   └─ Async → ArmJointTrajSdkMove (arm_home_to_spread_3s) 手臂展开
  ├─ WaitSeconds (hold_1s_1)
  ├─ Parallel (success_on_all, "phase2")         ← 腿+臂 同时目标/回收
  │   ├─ Async → LegJointSdkMove   (leg_target_3s)  腿部目标角度
  │   └─ Async → ArmJointTrajSdkMove (arm_bend_to_home_3s) 手臂回收
  ├─ Parallel (success_on_all, "phase1_repeat")  ← 重复 phase1
  │   ├─ Async → LegJointSdkMove   (leg_zero_3s_r2)
  │   └─ Async → ArmJointTrajSdkMove (arm_home_to_spread_3s_r2)
  ├─ WaitSeconds (hold_1s_2)
  ├─ Parallel (success_on_all, "phase2_repeat")  ← 重复 phase2
  │   ├─ Async → LegJointSdkMove   (leg_target_3s_r2)
  │   └─ Async → ArmJointTrajSdkMove (arm_bend_to_home_3s_r2)
  └─ TorsoResetSdkMove (torso_reset_after_leg_arm)
```

> **Async 装饰器的作用**：`orchestration/nodes/async_decorator.py` 是一个自定义 py_trees Decorator，在独立线程中 tick 子节点。当两个 Async 节点放在一个 `Parallel (success_on_all)` 中时，**腿和臂可以真正同时运动**（而非常规的交替 tick）。

#### 子树 3：demo_head_control.json（头部扫视）

```
Sequence (memory=true)
  ├─ HeadControlSdkMove (center)       → yaw=0,   pitch=0
  ├─ WaitSeconds (wait_1s_0)
  ├─ HeadControlSdkMove (look_left_30) → yaw=30,  pitch=0
  ├─ WaitSeconds (wait_1s_1)
  ├─ HeadControlSdkMove (look_right_30)→ yaw=-30, pitch=0
  ├─ WaitSeconds (wait_1s_2)
  ├─ HeadControlSdkMove (look_up_20)   → yaw=0,   pitch=20
  ├─ WaitSeconds (wait_1s_3)
  ├─ HeadControlSdkMove (look_down_20) → yaw=0,   pitch=-20
  ├─ WaitSeconds (wait_1s_4)
  ├─ HeadControlSdkMove (scan_left)    → yaw=30,  pitch=0
  ├─ WaitSeconds (wait_1p5s_1)
  ├─ HeadControlSdkMove (scan_right)   → yaw=-30, pitch=0
  ├─ WaitSeconds (wait_1p5s_2)
  ├─ HeadControlSdkMove (scan_center)  → yaw=0,   pitch=0
  └─ WaitSeconds (wait_1p5s_3)
```

---

## 完整链路示例

以下以 `demo_leg_arm_parallel.json` 子树中 `leg_zero_3s` 节点为例，展示从 JSON 到硬件的完整调用链：

```
py_tree_child.json
  │  {"name": "LegJointSdkMove", "label": "leg_zero_3s",
  │   "params": {"j0": {"value": "0.0", "source": "CUSTOM"}, ...}}
  │
  ▼
BehaviorTreeFactory._build_tree_recursive()
  │  解析 name="LegJointSdkMove" → 非复合节点、非子树 → _create_node_instance()
  │  通过 _build_node_index() 找到模块: orchestration.nodes.leg_joint_sdk_move
  │  importlib.import_module → LegJointSdkMove 类
  │
  ▼
LegJointSdkMove.__init__(name="LegJointSdkMove", label="leg_zero_3s", params=ParamsWrapper)
  │  self.params.get("j0") = 0.0
  │  self.params.get("total_time") = 3.0
  │
  ▼ (行为树第一次 tick 该节点)
LegJointSdkMove.initialise()
  │  LegJointSdkParams(joint_angles=[0.0, 0.0, 0.0, 0.0], total_time=3.0)
  │  LegJointSdkSkill(hardware=get_shared_hardware())
  │  skill.initialize(params) → on_initialize() → 校验通过
  │
  ▼ (每个后续 tick)
LegJointSdkMove.update()
  │  skill.execute() → on_execute()
  │
  ▼
LegJointSdkSkill.on_execute()
  │  self.hardware.send_leg_joint_sdk(joint_angles=[0,0,0,0], total_time=3.0)
  │
  ▼
SDKControlMixin.send_leg_joint_sdk()
  │  角度转换：用户角度 → 弧度 → 度（SDK 内部格式）
  │  self._low_level_sdk_manager.move_wheel_lower_joint_auto(
  │      target_deg=[0.0, 0.0, 0.0, 0.0], total_time=3.0
  │  )
  │
  ▼
LowLevelSDKManager.move_wheel_lower_joint_auto()
  │  自动管理 MPC 模式 (ArmOnly ↔ FullBody)
  │  从当前关节位置插值到目标，100Hz 频率下发 3 秒
  │
  ▼
robot_sdk.control.control_wheel_lower_joint(...)
  │  → 电机驱动 → 机器人腿部执行
```

## 如何新增一个场景

1. **确认原子节点已存在** — 检查 `orchestration/nodes/` 下是否有对应 Python 类
2. **如需新节点**：
   - 在 `skills/atomic/refactored_sdk/` 创建原子技能（封装 adapter 调用）
   - 在 `orchestration/nodes/` 创建节点类（继承 BaseAction，调用技能）
3. **创建场景目录**：`orchestration/scenarios/<your_scenario>/`
4. **编写 board.json**：定义全局初始参数（如轨迹数据、配置常量）
5. **编写 py_tree_child.json**（可选）：定义可复用的子树组合
6. **编写 py_tree.json**：定义主行为树执行流程，引用子树或原子节点
7. **运行验证**：
   ```bash
   # 离线验证（无需 ROS）
   python3 apps/test_upper_init/run_behavior_tree_json.py \
       --scenario orchestration/scenarios/<your_scenario> --dry-run --tick-once

   # 真机运行（需要 ROS）
   python3 apps/test_upper_init/run_behavior_tree_json.py \
       --scenario orchestration/scenarios/<your_scenario>
   ```

## 关键设计原则

| 原则 | 说明 |
|------|------|
| **关注点分离** | JSON 只管编排（"做什么"），Python 节点只管执行（"怎么做"），Skill 只管适配器调用 |
| **薄节点** | 节点类只做参数解析和状态传递，不包含运动逻辑 |
| **单一职责** | 每个原子技能只封装一次 IHardware 调用 |
| **深拷贝隔离** | 子树 JSON 在构建时深拷贝，避免同一子树多处引用的配置污染 |
| **参数三源** | `CUSTOM`(固定值) → `INPUT`(需宏替换) → `READ_BOARD`(黑板读取) |
| **宏替换** | `${var}` 语法在加载阶段解析，将 interface 输入映射到节点参数 |
