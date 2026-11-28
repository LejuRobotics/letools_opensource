# orchestration/nodes — 行为树叶子节点

每个文件是一个 py_trees Behaviour 叶子节点，作为 JSON 行为树中的可执行动作单元。
节点是**薄封装**——只负责参数解析和生命周期管理，实际业务逻辑委托给 `skills/` 层。

## 文件清单

### 手臂控制

| 文件 | 类名 | 对应 Skill | 场景使用 |
|------|------|-----------|---------|
| `timed_arm_joint.py` | `TimedArmJoint` | `timed_arm_joint` | `instructional_case_02` 阶段 1、`03` 各阶段、`05` 快照 |
| `arm_joint_traj_sdk_move.py` | `ArmJointTrajSdkMove` | `arm_joint_traj_sdk` | `atomic_v1`、`arm_v1`、`instructional_case_02` 阶段 2–5、`05` 快照 |
| `arm_reset_sdk_move.py` | `ArmResetSdkMove` | `arm_reset_sdk` | `arm_v1` |
| `move_arm_base_joint_trajectories.py` | `MoveArmBaseJointTrajectories` | 旧版 arm_control | `smoke_v1` |

### 底盘

| 文件 | 类名 | 对应 Skill | 场景使用 |
|------|------|-----------|---------|
| `base_pose_local_move.py` | `BasePoseLocalMove` | `base_pose_local` | `atomic_v1` |
| `chassis_short_move.py` | `ChassisShortMove` | `chassis_velocity` (旧版) | `smoke_v1` |
| `base_move_relative_jibot_move.py` | `BaseMoveRelativeJibotMove` | `base_move_relative_jibot_sdk` | `instructional_case_04`、`05` 快照等 |
| `node_enable_vel_control.py` | `NodeEnableVelControl` | 无（直接调用权限接口） | `instructional_case_04`、`05` 快照等 |

### 头部 / 下肢

| 文件 | 类名 | 对应 Skill | 场景使用 |
|------|------|-----------|---------|
| `head_control_sdk_move.py` | `HeadControlSdkMove` | `head_control_sdk` | `atomic_v1`、`instructional_case_01` 至 `05` |
| `move_head.py` | `MoveHead` | 旧版 head_control | **未使用** ❌ |
| `leg_joint_sdk_move.py` | `LegJointSdkMove` | `leg_joint_sdk` | `atomic_v1` |
| `leg_short_move.py` | `LegShortMove` | `leg_control` (旧版) | `smoke_v1` |

### 通用 TimedCmd

| 文件 | 类名 | 对应 Skill | 场景使用 |
|------|------|-----------|---------|
| `multi_timed_command_move.py` | `MultiTimedCommandMove` | `multi_timed_command` | `instructional_case_03`（仅 planner 2 位姿）、`04`（3/8/9 关节）、`05` 快照 |

### 编排工具

| 文件 | 类名 | 对应 Skill | 场景使用 |
|------|------|-----------|---------|
| `wait_for_enter.py` | `WaitForEnter` | `wait_for_enter` | 全部场景 |
| `wait_seconds.py` | `WaitSeconds` | `wait_seconds` | `atomic_v1`、`instructional_case_01`、`02`、`03`、`05`（含手臂独立等待与案例间 2 秒等待） |
| `async_decorator.py` | `Async` | 无（装饰器） | `atomic_v1` |

### 示教场景返回边界

- case03 为每阶段 `HeadControlSdkMove → MultiTimedCommandMove → Sequence(TimedArmJoint, WaitSeconds)` 串行，非多部位同步树；case04 独立使用并行结构。
- 头部 SDK 不查到位；躯干 TimedCmd 技能仍按规划时长等待；TimedArmJoint 只发送，后续独立 WaitSeconds 非阻塞等待，SDK 关节轨迹节点完成发送循环；这些 `SUCCESS` 均不是实测到位。
- `MultiTimedCommandMove` 默认准备模式 2 并设置焦点，结束不恢复；教学 case02/03/05 显式设置 `TimedArmJoint.prepare_arm_control=false`，不切模式（新节点默认 true）。SDK 轨迹收尾仅尝试恢复 MPC 默认模式/流，不恢复原手臂模式。
- case04 的 `BaseMoveRelativeJibotMove` 只提交底盘任务，不向黑板输出 `task_id`、不查询到达；`NodeEnableVelControl` 的权限恢复是 best-effort。
- [case05：01→04 顺序联测](../scenarios/instructional_case_05/README.md) 复用上述现有节点的子树，相邻案例间各插入公共 `WaitSeconds` 等待 2 秒；不新增动作技能、到位检查或复位。

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
