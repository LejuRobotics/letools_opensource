# 04 skills 原子技能

> Skill 是**可以被行为树复用的机器人动作单元**。它比 `hardware.xxx()` 更接近业务动作，又比一整棵行为树更小。

skills 层主要负责业务逻辑：参数校验、调用硬件接口、判断何时结束。总表中列出的 `ArmEePoseJibotSkill` 等技能存在话题直发例外；判断调用路径请看各技能实现，不要仅凭所在目录推断。

## 04.01 命名空间与类索引

| 点分路径 | 实体 | 说明 |
|---|---|---|
| `skills.base.skill_base` | `SkillBase` | 所有技能的基类（实现 `ISkill`，附带超时与异常兜底） |
| `skills.atomic.refactored_sdk` | 40 个顶层技能文件 | **当前主力**：扁平结构、一文件一技能；具体控制路径见下方总表 |
| `skills.atomic.manipulation.*` | 14 个目录 | **旧版**技能（`studio_smoke_v1` 场景使用） |
| `skills.atomic.motion.chassis_velocity` / `move_to_pose` | 2 个技能 | **旧版**运动技能 |
| `skills.atomic.perception.*` | `camera_capture` / `read_qrcode` / `pallet_*` / `box_frame` | 感知技能（非 SDK 路径） |
| `skills.atomic.grasp_skill` | `GraspSkill` | 旧版抓取（**不继承** `SkillBase`） |
| `orchestration.shared_hardware` | `get_shared_hardware()` | 硬件单例注入点 |

### 数量口径

| 口径 | 数量 |
|---|---|
| `refactored_sdk/` 顶层 `.py`（不含 `__init__.py`） | **40** |
| `manipulation/*/skill.py` | 14 |
| `motion/*/skill.py` | 2 |
| `perception/*/skill.py` | 4（另有若干 `algorithm.py` 纯计算模块） |

### `SkillBase` —— 技能基类

`skills.base.skill_base.SkillBase` 继承 `core.interfaces.i_skill.ISkill`（`ISkill` 的五个成员全是 `@abstractmethod`，见 `02-core-接口与数据模型.md`）。`SkillBase` 把它们实现为「模板方法 + 钩子」，子类只写钩子。

| 路径 | API | 签名 | 说明 | 所属层 |
|---|---|---|---|---|
| Skill | `name` | `name -> str` | 技能唯一标识（构造时传入，property） | `skills.base.skill_base` |
| Skill | `initialize` | `initialize(params: SkillParams) -> Result` | 记录 `params` 与 `_start_time`，转发给 `on_initialize` | `skills.base.skill_base` |
| Skill | `execute` | `execute() -> Result` | **先查超时**，再 `try: on_execute() except → Result.fail(str(e))` | `skills.base.skill_base` |
| Skill | `cancel` | `cancel() -> Result` | 打 WARNING 日志后转发 `on_cancel` | `skills.base.skill_base` |
| Skill | `is_finished` | `is_finished() -> bool` | 转发 `on_is_finished` | `skills.base.skill_base` |
| — | `on_initialize` | `on_initialize(params) -> Result` | 钩子，默认 `Result.ok()` | 子类覆写 |
| — | `on_execute` | `on_execute() -> Result` | 钩子，默认 `raise NotImplementedError` | 子类覆写 |
| — | `on_cancel` | `on_cancel() -> Result` | 钩子，默认 `Result.ok()` | 子类覆写 |
| — | `on_is_finished` | `on_is_finished() -> bool` | 钩子，默认 `False` | 子类覆写 |

`SkillBase.execute()` 内部实现（逐字）：

```python
def execute(self) -> Result:
    if time.time() - self._start_time > self._params.timeout:
        logger.error(f"Skill [{self.name}] timed out.")
        return Result.fail("Timeout")
    try:
        return self.on_execute()
    except Exception as e:
        logger.exception(f"Skill [{self.name}] execution error: {e}")
        return Result.fail(str(e))
```

两条硬性后果：

1. **超时是按 `initialize` 时刻算的**，不是按「一次 execute 的耗时」。等待类技能必须放大 `timeout`（见 `04.03`）。
2. **`on_execute` 抛任何异常都被吞成 `Result.fail(str(e))`**，不会向上传播。调试时看 ERROR 日志里的 traceback 而不是靠 except 捕获。

### `hardware` 注入约定

多数 `refactored_sdk` 硬件技能在构造时收一个 `IHardware`，并把它存到 `self.hardware`：

```python
class HeadControlSdkSkill(SkillBase):
    def __init__(self, hardware: IHardware):
        super().__init__(name="head_control_sdk")
        self.hardware = hardware
```

行为树节点通过全局单例注入：`orchestration.shared_hardware.get_shared_hardware()`（懒加载 + 线程安全，初始化失败不会缓存半成品对象）。纯编排类技能（`WaitSecondsSkill`、`WaitForEnterSkill`）不接硬件，构造无参。

多数硬件技能用 `getattr(self.hardware, "method_name", None)` 取方法，取不到时返回 `Result.fail("Hardware does not implement xxx()")`。纯计算、话题直发和嵌套子树技能有自己的调用方式，以源文件和对应节点为准。

## 04.02 类与方法详表

### refactored_sdk 技能总表

「Params」列是该技能的参数 dataclass；「Adapter 调用」列是它真正打到的适配器方法。

| 技能文件 | Skill 类 | Params | Adapter 调用 | 功能 |
|---|---|---|---|---|
| `arm_ee_traj_local_sdk.py` | `ArmEETrajLocalSdkSkill` | `ArmEETrajLocalSdkParams` | `send_arm_ee_traj_sdk(frame='base_link')` | 手臂末端轨迹（本体坐标系） |
| `arm_ee_traj_world_sdk.py` | `ArmEETrajWorldSdkSkill` | `ArmEETrajWorldSdkParams` | `send_arm_ee_traj_sdk(frame='world')` | 手臂末端轨迹（世界坐标系） |
| `arm_joint_timed.py` | `ArmJointTimedSkill` | `ArmJointTimedParams` | `send_arm_joint_timed()` | 双臂 14 关节单目标（TimedCmd） |
| `multi_timed_command.py` | `MultiTimedCommandSkill` | `MultiTimedCommandParams` | `set_arm_control_mode(2)`（可选） + `set_focus_ee()` + `set_focus_z()` + `send_timed_multi_commands()` | 可配置的多 planner 单次组合控制（如 2/8/9、3/6/7）；`prepare_arm_control=true` 默认在含 planner 2–9 时准备外部控制模式，失败不发令，不复位、不自动切回 0；`focus_ee=true` 专注末端解算，躯干可能随动，`false` 尽可能不动躯干并在手臂工作区间内解算，精度可能降低；`focus_z` 跟随 `focus_ee` |
| `arm_joint_traj_sdk.py` | `ArmJointTrajSdkSkill` | `ArmJointTrajSdkParams` | `send_arm_joint_traj_sdk()` | 手臂 14 关节多关键点轨迹 |
| `arm_reset_sdk.py` | `ArmResetSdkSkill` | `ArmResetSdkParams` | `arm_reset()` | 手臂归位 |
| `arm_ee_dual_timed.py` | `ArmEEDualTimedSkill` | `ArmEEDualTimedParams` | `send_arm_ee_world_timed` / `_local_timed` | 双臂单次末端位姿（双 planner 并发） |
| `arm_ee_single_timed.py` | `ArmEESingleTimedSkill` | `ArmEESingleTimedParams` | `send_left/right_arm_ee_*_timed` | 单臂单次末端位姿 |
| `arm_ee_burst_timed.py` | `ArmEEBurstTimedSkill` | `ArmEEBurstTimedParams` | `send_timed_single_command` 循环 | 在线连发航点（可双臂互不干扰） |
| `arm_ee_local_relative_timed.py` | `ArmEELocalRelativeTimedSkill` | `ArmEELocalRelativeTimedParams` | 相对补偿 TimedCmd | 双臂末端 local 相对补偿 |
| `arm_ee_timed_cmd.py` | `ArmEETimedCmdSkill` | `ArmEETimedCmdParams` | `send_timed_multi_commands` | TimedCmd 末端位姿（多航点 index 切换） |
| `arm_ee_offline_traj.py` | `ArmEEOfflineTrajSkill` | `ArmEEOfflineTrajParams` | `set_offline_trajectory` + `enable_offline_trajectory` | 离线时间最优轨迹 |
| `arm_ee_pose_jibot.py` | `ArmEePoseJibotSkill` | `ArmEePoseJibotParams` | `send_ee_pose` / `send_both_ee_poses` | 末端位姿（**话题直发**，非 SDK） |
| `arm_ee_setup_jibot.py` | `ArmEESetupJibotSkill` | `ArmEESetupJibotParams` | MPC 前置 + `set_focus_ee` + 预建 Publisher | 末端控制前置设置 |
| `base_move_relative_jibot.py` | `BaseMoveRelativeJibotSkill` | `BaseMoveRelativeJibotParams` | `base_move_relative_jibot()` | JiBot 底盘相对移动 |
| `base_move_to_target_jibot.py` | `BaseMoveToTargetJibotSkill` | `BaseMoveToTargetJibotParams` | `base_move_to_target_jibot()` | JiBot 底盘绝对目标移动 |
| `base_pose_local.py` | `BasePoseLocalSkill` | `BasePoseLocalParams` | `send_base_pose(frame=LOCAL)` | 底盘相对位姿（适配器内部等到达） |
| `chassis_stop.py` | `ChassisStopSkill` | `ChassisStopParams` | `enable_vel_control_jibot()` | 底盘停止（切换速度控制权限） |
| `chassis_servo_calc.py` | `ChassisServoCalcSkill` | `ChassisServoCalcParams` | 无（纯计算） | 视觉偏差 → 位置增量 / 速度指令 |
| `chassis_servo_exec.py` | `ChassisServoExecSkill` | `ChassisServoExecParams` | `base_move_relative_jibot()` / `send_base_velocity_tick()` / `send_base_cmd_vel_tick()` | 伺服指令下发底盘（异步，等到达由节点做） |
| `chassis_servo_judge.py` | `ChassisServoJudgeSkill` | `ChassisServoJudgeParams` | 无（纯计算） | 收敛 / 继续 / 超限失败判决 |
| `check_arrived_jibot.py` | `CheckArrivedJibotSkill` | `CheckArrivedJibotParams` | `check_arrived_jibot()` | JiBot 到达检查 |
| `check_distance_to_target.py` | `CheckDistanceToTargetSkill` | `CheckDistanceToTargetParams` | 读黑板 / 状态 | 检查到目标距离 |
| `head_control_sdk.py` | `HeadControlSdkSkill` | `HeadControlSdkParams` | `control_head_sdk()` | 头部（SDK 直调） |
| `leg_joint_sdk.py` | `LegJointSdkSkill` | `LegJointSdkParams` | `send_leg_joint_sdk()` | 下肢关节（SDK） |
| `leg_joint_timed.py` | `LegJointTimedSkill` | `LegJointTimedParams` | `send_leg_joint_timed()` | 下肢关节（TimedCmd，服务端 Ruckig） |
| `torso_reset_sdk.py` | `TorsoResetSdkSkill` | `TorsoResetSdkParams` | `reset_torso_to_initial()` | 躯干复位（ROS Service） |
| `bimanual_joint_trajectory.py` | `BimanualJointTrajectorySkill` | `BimanualJointTrajectoryParams` | `send_arm_joint_traj_sdk()` | 合并左右臂为 14 关节同步轨迹 |
| `single_tag_joint_pick_plan.py` | `SingleTagJointPickPlanSkill` | 同名 Params | IK + 关节轨迹 | 按 Tag 位姿规划双臂抓取关节轨迹 |
| `grasp_ring.py` | `GraspRingSkill` | `GraspRingParams` | `send_ee_pose` + `control_end_effector` | 抓圆环：读位姿 → FK → 预抓 → 下抓 → 抬起 |
| `place_ring.py` | `PlaceRingSkill` | `PlaceRingParams` | 同上 | 放圆环 |
| `vacuum_485.py` | `Vacuum485Skill` | `Vacuum485Params` | `adapters.vacuum_485` | 气泵继电器（吸/吹/断电） |
| `vacuum_control.py` | `VacuumControlSkill` | `VacuumControlParams` | `adapters.vacuum_control` | 气泵吸/放 |
| `pressure_drop_detection.py` | `PressureDropDetectionSkill` | 同名 Params | 压力传感器 | 压力骤降检测 |
| `inference_control_skill.py` | `InferenceControlSkill` | `InferenceControlParams` | 外部推理服务 | VLA 推理控制 |
| `wait_seconds.py` | `WaitSecondsSkill` | `WaitSecondsParams` | 无（纯编排） | 等待指定秒数 |
| `wait_for_enter.py` | `WaitForEnterSkill` | `WaitForEnterParams` | 无（纯编排） | 等待人工按 Enter |
| `palletizing_line_mpc.py` | `PalletizingLineMpcSkill` | `PalletizingLineMpcParams` | `set_offline_trajectory` + `enable_offline_trajectory` | 码垛直线搬运（MPC 离线轨迹，默认 10 段） |
| `palletizing_line_mpc_restore.py` | `PalletizingLineMpcRestoreSkill` | `PalletizingLineMpcRestoreParams` | 继承原 MPC 调用；另调 `get_arm_joint_positions` + `get_leg_joint_positions` + `send_arm_joint_trajectory` + `send_leg_joint_command` | 搬运前记录双臂与下肢 18 关节，完成、失败或异常后恢复并读回验证；默认 `prepare_pose=False` |
| `palletizing_line_ik.py` | `PalletizingLineIkSkill` | `PalletizingLineIkParams` | `solve_ik` + `publish_arm_joint_setpoint`（逐帧）；预备姿态 `send_arm_joint_trajectory` | 码垛直线搬运（逐帧 IK，默认 `ik_rate=40 Hz`，默认 10 段直线；`approach_distance=0` 时 8 段） |

`palletizing_line_ik` 的「10 段」是默认值：`build_palletizing_path()` 在 `approach_distance > 0` 时生成 10 段（含接近 `approach` 与撤离 `retreat` 两段）；`approach_distance=0` 时这两段不生成，只剩 8 段。路径段数随该参数变化，不是固定常量。

`palletizing_line_ik` 里三处下发要分开读，阻塞语义各不相同：

- **IK 逐帧**（直线段内每一帧）：`solve_ik` 求出 14 关节角后，用非阻塞的 `publish_arm_joint_setpoint` 逐帧发布，默认 `ik_rate=40 Hz`。它只发设定点、不等待到位，所以能按帧周期跑。
- **预备姿态**（执行前，技能内）：走阻塞的 `send_arm_joint_trajectory`。首帧 q0 与冻结姿态都依赖它已经到位。
- **最终关节还原**（执行后，`apps` 侧）：验收脚本 teardown 用同一个阻塞接口把选中的关节还原，不在技能内部完成。

`palletizing_line_mpc` 原技能仍只做 `set_offline_trajectory` + `enable_offline_trajectory`，结束还原由入口负责。`palletizing_line_mpc_restore` 通过继承复用同一搬运实现，在原技能的每个返回路径外增加 18 关节恢复；恢复命令之后必须重新读取双臂和下肢角度，超过 `restore_arm_tolerance_deg` / `restore_leg_tolerance` 即返回失败，不能把发送成功当作恢复到位。

### manipulation 旧版技能

旧版技能目录结构是「一个技能一个目录」（`skill.py` + 可选 `params.py`），基类不统一：部分继承 `SkillBase`，部分直接实现 `ISkill`，参数也有独立 dataclass。

| 目录 | Skill 类 | 基类 | 说明 |
|---|---|---|---|
| `arm_control/` | `ArmControlSkill` | `SkillBase` | 手臂关节轨迹（度），对齐 `test_arm_joint.py` |
| `arm_trajectory/` | `ArmTrajectorySkill` | `ISkill` | 自定义关节名 + 时间点 + 角度表 |
| `head_control/` | `HeadControlSkill` | `SkillBase` | 头部单步 |
| `leg_control/` | `LegControlSkill` | `SkillBase` | 腿部控制 |
| `leg_joint_control/` | `LegJointControlSkill` | `ISkill` | 腿部关节 |
| `pick/` | `PickSkill` | `SkillBase` | 抓取（视觉可选） |
| `place/` | `PlaceSkill` | `ISkill` | 放置 |
| `pos_base_control/` | `PosBaseControlSkill` | `ISkill` | 本体系位姿 |
| `pos_world_control/` | `PosWorldControlSkill` | `ISkill` | 世界系位姿 |
| `simple_two_arm_publisher_joint/` | `TwoArmJointSweepSkill` | `ISkill` | 双臂关节扫掠 |
| `simple_two_arm_publisher_local/` | `SimpleTwoArmPublisherLocalSkill` | `ISkill` | 双臂本体系发布（带自定义 IK 参数） |
| `torso_pose_control/` | `TorsoPoseControlSkill` | `ISkill` | 躯干位姿序列 |
| `two_arm_hand_pose_control/` | `TwoArmHandPoseControlSkill` | `ISkill` | 双臂手部位姿（带自定义 IK 参数） |
| `vel_control/` | `VelControlSkill` | `ISkill` | 底盘速度 |

### 新旧架构对比

| 方面 | `refactored_sdk/`（新） | `manipulation/` `motion/`（旧） |
|---|---|---|
| 基类 | 统一 `SkillBase` | 部分 `SkillBase`，部分自定义 `ISkill` |
| 硬件路径 | 按技能选择 SDK、TimedCmd、话题、服务或纯计算 | 混合 ROS 话题 / 服务 / SDK |
| 参数 | `@dataclass Params(SkillParams)` | 各自定义 |
| 目录结构 | 扁平，一文件一技能 | 嵌套，一技能一目录 |
| 对应节点 | `orchestration/nodes/` 统一薄封装 | 部分在 `nodes/`，部分自建 |
| 使用场景 | `refactored_sdk_atomic_v1` / `_arm_v1` | `studio_smoke_v1` |

**新技能一律写进 `refactored_sdk/`。**

## 04.03 参数说明

所有技能参数都继承 `core.domain.skill_params.SkillParams`：

| 字段 | 类型 | 默认 | 说明 |
|---|---|---|---|
| `skill_name` | `str` | `""` | 技能标识 |
| `timeout` | `float` | `30.0` | **执行超时（秒）**，从 `initialize` 起算 |
| `retry_count` | `int` | `3` | 失败重试次数（由上层使用） |
| `extra_kwargs` | `Dict[str, Any]` | `{}` | 扩展参数 |

### 高频技能参数（逐字段）

**末端 TimedCmd 族**

| Params | 字段 | 类型 | 默认 | 说明 |
|---|---|---|---|---|
| `ArmEEDualTimedParams` | `frame` | `str` | `'world'` | `world` → planner 4+5；`local` / `base_link` → 6+7 |
| | `left_pose` / `right_pose` | `List[float]` | 见下 | 6 维 `[x, y, z, yaw, pitch, roll]`（**度**） |
| | `desire_time` | `float` | `2.0` | 期望执行时长（秒） |
| | `focus_ee` / `focus_z` | `bool` | `False` | 是否抢占笛卡尔 / Z 轴跟踪焦点 |
| | `prepare_arm_control` / `release_arm_control` | `bool` | `True` | 是否自动做手臂外部控制前置 / 收尾 |
| | `timeout` | `float` | `30.0` | |
| `ArmEESingleTimedParams` | `side` | `str` | `'left'` | `left` / `right` |
| | `frame` | `str` | `'world'` | |
| | `pose` | `List[float]` | 6 维默认姿态 | 度 |
| | `desire_time` | `float` | `3.0` | |
| `ArmEEBurstTimedParams` | `side` / `frame` | `str` | `'left'` / `'world'` | |
| | `waypoints` | `List[List[float]]` | 内置默认航点 | 逐点连发 |
| `ArmEELocalRelativeTimedParams` | `offset_x` / `offset_y` / `offset_z` | `float` | 见 manifest | local 相对偏移 |
| | `fixed_yaw` / `fixed_pitch` / `fixed_roll` | `float` | `0.0` / `-90.0` / `0.0` | 姿态锁死值 |
| `ArmEEOfflineTrajParams` | `traj` | `List[List[float]]` | — | 轨迹点 |
| | `times` | `List[float]` | — | 时间戳 |
| | `total_time` | `float` | — | 总时长 |

**SDK 轨迹族**

| Params | 字段 | 类型 | 默认 | 说明 |
|---|---|---|---|---|
| `ArmEETrajLocalSdkParams` / `ArmEETrajWorldSdkParams` | `left_traj` / `right_traj` | `List[List[float]]` | 内置默认 | 关键点列表 |
| | `total_time` | `float` | `3.0` | 自动插值总时长 |
| | `timeout` | `float` | `60.0` | 轨迹类超时更宽松 |
| `ArmJointTrajSdkParams` | `joint_traj` | 多关键点 14 关节角 | — | |
| | `total_time` | `float` | `3.0` | |
| `ArmJointTimedParams` | `joint_angles` | `List[float]` | 14 个 `0.0` | 单个双臂目标，顺序为 `[左臂 7, 右臂 7]` |
| | `desire_time` | `float` | `3.0` | TimedCmd 期望执行时间 |
| | `settle_time` | `float` | `0.0` | 运动完成后的额外等待 |
| `BimanualJointTrajectoryParams` | `left_arm_joint_traj` / `right_arm_joint_traj` | `Optional[List[List[float]]]` | `None` | 单臂轨迹 |
| | `bimanual_traj_14d` | `Optional[List[List[float]]]` | `None` | 直接给 14 维 |
| | `total_time` | `float` | `3.0` | |

**底盘 / JiBot**

| Params | 字段 | 类型 | 默认 |
|---|---|---|---|
| `BaseMoveRelativeJibotParams` | `x` / `y` / `theta` | `float` | 见 manifest |
| | `options` | `MoveToTargetOptions` | 避障与到达阈值 |
| `BasePoseLocalParams` | `x` / `y` / `yaw` | `float` | — |
| `ChassisStopParams` | — | — | 无额外字段 |
| `CheckArrivedJibotParams` | `task_id` / `blocking` / `timeout` | — | 见 manifest |

**`ChassisServoExecParams`**

位置模式（`ServoCommand.mode='position'`）用前四行字段走 `base_move_relative_jibot`；速度模式（`mode='velocity'`）用后三行字段决定发到哪条通道。

| 字段 | 类型 | 默认 | 说明 |
|---|---|---|---|
| `position_threshold` | `float` | `0.08` | 位置模式到达容差(m)：剩余距离小于它判到达、一步不走，单步位移须明显大于它 |
| `angle_threshold` | `float` | `0.1` | 位置模式到达容差(rad)：同理 |
| `linear_velocity` / `angular_velocity` | `float` | `0.15` / `0.25` | 位置模式底盘速度(m/s、rad/s) |
| `allow_rotation` | `bool` | `True` | 位置模式是否允许旋转 |
| `command_channel` | `str` | `'humanoid'` | 速度模式通道：`humanoid` = 现有 `/cmd_vel`（`send_base_velocity_tick`）；`chassis_direct` = 底盘原生话题（`send_base_cmd_vel_tick`）。位置模式不看此字段 |
| `cmd_vel_topic` | `str` | `'/move_base/base_cmd_vel'` | 仅 `command_channel='chassis_direct'` 生效：底盘原生速度话题 |
| `publish_rate_hz` | `float` | `100.0` | 仅 `chassis_direct` 生效；本技能**不启动流**，该值只由节点透传给适配器的 `start_cmd_vel_stream` |

速度模式的通道分派与兜底：

- `humanoid`：每帧单发一次 `/cmd_vel`，与引入 `command_channel` 之前逐字一致。
- `chassis_direct`：每帧向 `cmd_vel_topic` 单发一次 `base_cmd_vel`。行为树里走这条通道时**不经过技能的这段分支**——`NodeChassisServoExec` 自己起后台流并只更新目标速度（见 `11-节点清单.md` 的 `NodeChassisServoExec` 一节）；技能这边只覆盖直接调用技能的场合。
- 未知取值：不静默，打 ERROR 后本帧仍按 `humanoid` 下发，但返回失败，`error_code='INVALID_COMMAND_CHANNEL'`。
- 零速兜底跟当前通道走：`chassis_direct` 发 `cmd_vel_topic`，其余发 `/cmd_vel`，不会把零速发到另一条通道上。

**头部 / 下肢 / 躯干**

| Params | 字段 | 类型 | 默认 | 说明 |
|---|---|---|---|---|
| `HeadControlSdkParams` | `yaw_deg` / `pitch_deg` | `float` | `0.0` | **度** |
| `LegJointSdkParams` / `LegJointTimedParams` | `joint_angles` | `List[float]` | — | 4 关节 |
| `TorsoResetSdkParams` | — | — | 无额外字段 | |

**抓取 / 末端工具**

| Params | 字段 | 类型 | 默认 |
|---|---|---|---|
| `GraspRingParams` | `q_pre` | `List[float]` | 14 维预抓关节角 |
| | `grasp_offset_z` / `pre_grasp_offset_z` / `ee_to_tip_z` | `float` | `0.05` / `0.15` / `-0.1` |
| | `ring_name` | `str` | `'ring_01'` |
| | `gripper_pre_position` / `gripper_close_position` | `float` | `50.0` / `0.0` |
| `Vacuum485Params` / `VacuumControlParams` | `action` | `str` | `'suck'` |
| `InferenceControlParams` | `inference_config_path` | `str` | 内置默认 |
| | `policy_type` | `str` | `'act'` |
| | `pretrained_path` / `task_prompt` | `str` | `''` / `'Pick Place'` |
| | `max_episode_steps` | `int` | `200` |
| | `device` / `inference_env` | `str` | `'cuda'` / `'sim'` |

**纯编排工具**

| Params | 字段 | 类型 | 默认 | 说明 |
|---|---|---|---|---|
| `WaitSecondsParams` | `duration_sec` | `float` | `1.0` | 等待秒数；`timeout` 已设 `120.0` |
| `WaitForEnterParams` | `message` | `str` | — | 提示语 |

### `timeout` 的坑（高频踩雷）

`SkillBase.execute()` **每次调用**都会检查 `now - start_time > timeout`，超时直接 `Result.fail("Timeout")` 并打 ERROR。因此：

- 轨迹类技能（`*_sdk`）默认给到 `60.0`；
- 在技能内部运行整棵子树的流程型技能需要给到小时级（如 `7200.0`）；
- **自己写等待类技能时必须同步放大 `timeout`**。典型反例：`WaitSeconds` 场景里若要等 86400 秒，就必须同时把 `timeout` 设成 `max(duration + 60, 120)`，否则第一次检查就超时，行为树直接 FAILURE。

### 干跑短路

`WaitSecondsSkill` / `WaitForEnterSkill` 等纯编排技能会读 `STUDIO_DRY_RUN` 环境变量（`1` / `true` / `yes`）：

```python
_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")
```

干跑时直接返回 `Result.ok()`、`on_is_finished()` 返回 `True`。运动类技能与节点同理。

## 04.04 返回值

- **`initialize` / `execute` / `cancel` 返回 `Result`**。`execute()` 在技能还在进行中时也返回 `Result.ok()`（**`ok` 不等于已完成**），完成与否看 `is_finished()`。
- **`is_finished() -> bool`**。判定技能终态只看它。
- 参数类型不对时 `on_initialize` 返回 `Result.fail("Invalid parameters for XxxSkill")`——这是 `isinstance` 检查，不是异常。
- 适配器没有对应方法时返回 `Result.fail("Hardware does not implement xxx()")`。

标准驱动循环（`apps/test_kuavo_5w_skills/_scaffold.py` 的 `run_skill` 即此形状）：

```python
result = skill.initialize(params)
if not result.success:
    return False
deadline = time.time() + params.timeout
while not skill.is_finished():
    if time.time() > deadline:
        return False                     # 超时
    result = skill.execute()
    if not result.success:
        return False                     # 失败
    time.sleep(tick_interval)            # 默认 0.05s（20Hz）
```

## 04.05 异常

技能层**几乎不向外抛异常**，全部收敛为 `Result`：

| 情形 | 表现 |
|---|---|
| `params` 类型不对 | `Result.fail("Invalid parameters for XxxSkill")` |
| 适配器无该方法 | `Result.fail("Hardware does not implement xxx()")` |
| 超时 | `Result.fail("Timeout")` + ERROR 日志 |
| `on_execute` 内抛任何异常 | 被 `SkillBase.execute()` 捕获 → `Result.fail(str(e))` + `logger.exception` traceback |
| 适配器方法返回失败 | 原样透传该 `Result` |

`SkillExecutionError`（定义于 `core.common.exceptions`）是技能层的语义化异常类型，供上层在需要抛异常时使用；标准路径上不会看到它。

## 04.06 调用示例

### 直接构造运行一个技能

以下示例需要已启动的机器人或对应仿真，以及已经完成 ROS 和 SDK 环境初始化；`control_head_sdk` 的参数单位为度。先检查硬件初始化结果，再运行技能。

```python
from adapters.hardware.factory import HardwareFactory
from skills.atomic.refactored_sdk.head_control_sdk import (
    HeadControlSdkParams, HeadControlSdkSkill,
)

hw = HardwareFactory.create_hardware({"robot_type": "leju_wheeled"})
result = hw.initialize()
if not result.success:
    raise RuntimeError(result.message)

try:
    skill = HeadControlSdkSkill(hardware=hw)
    params = HeadControlSdkParams(yaw_deg=15.0, pitch_deg=-5.0, timeout=10.0)
    result = skill.initialize(params)
    if not result.success:
        raise RuntimeError(result.message)

    while not skill.is_finished():
        result = skill.execute()
        if not result.success:
            raise RuntimeError(result.message)
finally:
    hw.shutdown()
```

### 无硬件调用示例：等待技能

`WaitSecondsSkill` 不需要硬件，可用它先练习 `initialize → execute → is_finished` 的调用顺序。若设置了 `STUDIO_DRY_RUN=1`，该技能会立即完成；下面的例子应在未设置该变量的普通终端运行。

```python
import time
from skills.atomic.refactored_sdk.wait_seconds import (
    WaitSecondsParams, WaitSecondsSkill,
)

skill = WaitSecondsSkill()
result = skill.initialize(WaitSecondsParams(duration_sec=2.0, timeout=5.0))
if not result.success:
    raise RuntimeError(result.message)
while not skill.is_finished():
    result = skill.execute()
    if not result.success:
        raise RuntimeError(result.message)
    time.sleep(0.05)
```

行为树中的硬件节点由 `orchestration.shared_hardware.get_shared_hardware()` 注入适配器。上面的直接调用示例只展示两种已核对源码的技能；总表中的其他技能须逐个补齐必需参数、依赖、运行环境与失败判定后，才能称为“按文档复现”。

### 新增一个技能（标准流程）

```text
1. 先确认 hardware 有没有对应能力（send_*_sdk / send_*_timed / send_*）。
2. 在 skills/atomic/refactored_sdk/ 新增 xxx.py，继承 SkillBase。
3. 定义 @dataclass XxxParams(SkillParams)，字段带默认值。
4. on_execute() 里 getattr(self.hardware, "method", None) 取方法并调用。
   不要自己写 rospy.Publisher / ServiceProxy。
5. 要放进行为树，再去 orchestration/nodes/ 写一个薄节点
   （负责 JSON 参数解析 + py_trees 状态转换）。
6. 不要往 skills/atomic/refactored_sdk/__init__.py 加聚合导入 ——
   行为树构建阶段会并行 import 叶子节点模块，聚合导入会放大 import 锁竞争，
   某些 Python 版本/线程时序下触发 _DeadlockError。
```

> 相关：本篇提到的适配器方法签名见 [`03-adapters-硬件适配层.md`](./03-adapters-硬件适配层.md)；`SkillParams` 在 core 中的定义见 [`02-core-接口与数据模型.md`](./02-core-接口与数据模型.md)；技能如何被节点与场景组织见 [`10-Skill→Node→场景 层级关系与选型建议.md`](./10-Skill→Node→场景%20层级关系与选型建议.md)；技能层真机测试见 [`06-apps-测试与示例应用.md`](./06-apps-测试与示例应用.md)。
