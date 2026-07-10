# orchestration/nodes — 行为树叶子节点

每个文件是一个 py_trees Behaviour 叶子节点，作为 JSON 行为树中的可执行动作单元。
节点是**薄封装**——只负责参数解析和生命周期管理，实际业务逻辑委托给 `skills/` 层。

## 文件清单

### 手臂控制

| 文件 | 类名 | 对应 Skill | 场景使用 |
|------|------|-----------|---------|
| `arm_ee_traj_local_sdk_move.py` | `ArmEeTrajLocalSdkMove` | `arm_ee_traj_local_sdk` | `arm_v1` |
| `arm_ee_traj_world_sdk_move.py` | `ArmEeTrajWorldSdkMove` | `arm_ee_traj_world_sdk` | `arm_v1` |
| `arm_joint_traj_sdk_move.py` | `ArmJointTrajSdkMove` | `arm_joint_traj_sdk` | `atomic_v1` `arm_v1` |
| `arm_reset_sdk_move.py` | `ArmResetSdkMove` | `arm_reset_sdk` | `arm_v1` |
| `move_arm_base_joint_trajectories.py` | `MoveArmBaseJointTrajectories` | 旧版 arm_control | `smoke_v1` |

### 底盘

| 文件 | 类名 | 对应 Skill | 场景使用 |
|------|------|-----------|---------|
| `base_pose_local_move.py` | `BasePoseLocalMove` | `base_pose_local` | `atomic_v1` |
| `chassis_short_move.py` | `ChassisShortMove` | `chassis_velocity` (旧版) | `smoke_v1` |

### 头部 / 下肢

| 文件 | 类名 | 对应 Skill | 场景使用 |
|------|------|-----------|---------|
| `head_control_sdk_move.py` | `HeadControlSdkMove` | `head_control_sdk` | `atomic_v1` |
| `move_head.py` | `MoveHead` | 旧版 head_control | **未使用** ❌ |
| `leg_joint_sdk_move.py` | `LegJointSdkMove` | `leg_joint_sdk` | `atomic_v1` |
| `leg_short_move.py` | `LegShortMove` | `leg_control` (旧版) | `smoke_v1` |

### 编排工具

| 文件 | 类名 | 对应 Skill | 场景使用 |
|------|------|-----------|---------|
| `wait_for_enter.py` | `WaitForEnter` | `wait_for_enter` | 全部场景 |
| `wait_seconds.py` | `WaitSeconds` | `wait_seconds` | `atomic_v1` |
| `async_decorator.py` | `Async` | 无（装饰器） | `atomic_v1` |

### 基础设施

| 文件 | 类名 | 说明 |
|------|------|------|
| `base_node.py` | `BaseAction` | 所有新节点基类，继承 `py_trees.behaviour.Behaviour`，提供 `params` / `global_blackboard` / DRY_RUN 支持 |
| `base.py` | → `BaseAction` | 兼容层，重导出 `BaseAction` |
| `skill_node.py` | `SkillNode` `ISkill` | **遗留** 抽象基类框架，已废弃 ❌ |

## 新节点模板

```python
class XxxMove(BaseAction):
    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._skill = None
        self._dry_done = False

    def initialise(self):
        # 解析 params → 构造 SkillParams → 创建 Skill → skill.initialize()
        ...

    def update(self):
        # skill.execute() / skill.is_finished() → Status.SUCCESS/FAILURE/RUNNING
        ...
```

## 与 JSON 的映射

JSON 中 `"name"` 字段即为节点类名。Factory 通过 `_build_node_index()` 自动发现：
- 文件名 `arm_joint_traj_sdk_move.py` → 类名 `ArmJointTrajSdkMove`
- 转换规则：`_snake_to_pascal("arm_joint_traj_sdk_move")` → `ArmJointTrajSdkMove`


