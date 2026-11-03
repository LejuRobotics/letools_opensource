# 02 core 接口与数据模型

core 层是 LeTools 的**共同语言层**：它定义接口、领域数据模型和枚举，本身不直接控制机器人，也不依赖 ROS / SDK。

> **读法**：先看 `02.01` 的类索引建立全景，再按需查 `02.02` 的方法详表。`02.03`–`02.05` 是参数的坑（单位、枚举、返回形态、异常），`02.06` 汇总可复制示例。

## 02.01 命名空间与类索引

| 点分路径 | 实体 | 说明 |
|---|---|---|
| `core.interfaces.i_hardware` | `IHardware` | 硬件抽象接口（所有 Adapter 必须实现） |
| `core.interfaces.i_skill` | `ISkill` | 技能接口（生命周期契约） |
| `core.interfaces.i_camera` | `ICamera` | 相机接口 |
| `core.interfaces.i_perception` | `IPerception` | 感知接口（依赖注入 `ICamera`） |
| `core.domain.result` | `Result` | 统一操作结果封装 |
| `core.domain.pose` | `Pose6D`、`TransformMatrix` | 6D 位姿与 4×4 齐次变换矩阵 |
| `core.domain.enums` | `FrameType`、`MPCControlMode`、`ArmSide` | 坐标帧 / MPC 模式 / 手臂侧别 |
| `core.domain.skill_params` | `SkillParams`、`MoveToPoseParams`、`PickObjectParams` | 技能参数基类与常用子类 |
| `core.domain.end_effector` | `GripperCommand`、`DualGripperCommand`、`HandFingerCommand`、`SG100HandCommand`、`EndEffectorType` | 末端执行器指令与类型 |
| `core.domain.chassis_options` | `MoveToTargetOptions` | 底盘导航选项 |
| `core.domain.ruckig_params` | `RuckigParams` | Ruckig 在线轨迹规划器参数 |
| `core.domain.camera` | `CameraInfo`、`CameraFrame`、`DepthData`、`PointCloudData`、`CameraStatus`、`CameraIntrinsics`、`CameraExtrinsics` | 相机数据模型 |
| `core.domain.perception` | `TagDetection`、`ObjectDetection`、`PerceptionResult` | 感知数据模型 |
| `core.domain.joint_state` / `observation` / `tag` / `task` / `trajectory` | 关节状态、观测、标签、任务、轨迹模型 | 其余领域模型 |

## 02.02 类与方法详表

### `IHardware` —— 硬件抽象接口

`core.interfaces.i_hardware.IHardware` 是抽象基类（`ABC`）。它规定的不是「机器人有什么」，而是「Adapter 必须提供哪些方法」。方法分十组：

| 路径 | API | 签名 | 说明 | 所属层 |
|---|---|---|---|---|
| 标准 | `initialize` | `initialize() -> Result` | 初始化连接（ROS 节点 / SDK / 相机 / 状态管理器） | `IHardware` 抽象 |
| 标准 | `shutdown` | `shutdown() -> Result` | 断开连接并释放资源 | `IHardware` 抽象 |
| 查询 | `is_connected` | `is_connected -> bool` | 当前是否已连接（property） | `IHardware` 抽象 |
| 标准 | `send_base_velocity` | `send_base_velocity(vx, vy, vyaw, frame=LOCAL) -> Result` | 底盘速度控制 | `IHardware` 抽象 |
| 标准 | `send_base_pose` | `send_base_pose(x, y, yaw, frame=WORLD) -> Result` | 底盘位置控制 | `IHardware` 抽象 |
| 标准 | `move_chassis_relative` | `move_chassis_relative(x, y, theta, options=None) -> Result` | 本体系相对导航（可选能力，默认 `fail`） | `IHardware` 抽象 |
| 标准 | `move_chassis_to_target` | `move_chassis_to_target(x, y, theta, options=None) -> Result` | 地图系绝对导航（可选能力） | `IHardware` 抽象 |
| 标准 | `check_chassis_arrived` | `check_chassis_arrived(task_id, blocking=True, timeout=20.0) -> Result` | 导航到达查询 | `IHardware` 抽象 |
| 标准 | `set_chassis_external_control` | `set_chassis_external_control(enable) -> Result` | 导航 / 外部控制权切换 | `IHardware` 抽象 |
| 查询 | `get_chassis_external_control_state` | `get_chassis_external_control_state(timeout=3.0) -> Result` | 读当前控制权 | `IHardware` 抽象 |
| 标准 | `send_torso_pose` | `send_torso_pose(pose: Pose6D) -> Result` | 躯干相对基座位姿 (x, z, pitch, yaw) | `IHardware` 抽象 |
| 标准 | `get_torso_initial_pose` | `get_torso_initial_pose() -> Result` | 读躯干初始位姿（`data={position, euler}`） | `IHardware` 抽象 |
| 标准 | `send_ee_pose` | `send_ee_pose(side, pose: Pose6D, frame=WORLD) -> Result` | 单臂末端位姿 | `IHardware` 抽象 |
| 标准 | `send_both_ee_poses` | `send_both_ee_poses(left_pose, right_pose, frame=WORLD) -> Result` | 双臂末端同时控制 | `IHardware` 抽象 |
| 标准 | `send_arm_joint_trajectory` | `send_arm_joint_trajectory(positions[14], time_sec=0.0) -> Result` | 双臂 14 关节轨迹，阻塞等待到位 | `IHardware` 抽象 |
| 标准 | `publish_arm_joint_setpoint` | `publish_arm_joint_setpoint(positions_deg[14]) -> Result` | 双臂 14 关节设定点（左 7 右 7，单位度，无 `time_sec`），**非阻塞**：只发布设定点，不 sleep、不等到位；`success` 仅表示 publish 调用完成 | `IHardware` 抽象 |
| 查询 | `get_arm_joint_positions` | `get_arm_joint_positions() -> Result` | 当前 14 个手臂关节角，`data` 为 14 维列表，单位**弧度**（不随 `angle_unit` 变化） | `IHardware` 抽象 |
| 标准 | `send_leg_joint_command` | `send_leg_joint_command(positions[4]) -> Result` | 腿/躯干 4 关节 | `IHardware` 抽象 |
| Timed | `send_timed_base_pose` | `send_timed_base_pose(x, y, yaw, desire_time, frame=WORLD) -> Result` | 指定时长底盘位姿 | `IHardware` 抽象 |
| Timed | `send_timed_torso_pose` | `send_timed_torso_pose(x, z, yaw, pitch, desire_time) -> Result` | 指定时长躯干位姿 | `IHardware` 抽象 |
| Timed | `send_timed_leg_joint` | `send_timed_leg_joint(joint_angles[4], desire_time) -> Result` | 指定时长腿关节 | `IHardware` 抽象 |
| Timed | `send_timed_left_arm_joint` / `send_timed_right_arm_joint` | `send_timed_*_arm_joint(joint_angles[7], desire_time) -> Result` | 单臂 7 关节时序 | `IHardware` 抽象 |
| Timed | `send_timed_left_arm_ee_world` / `send_timed_right_arm_ee_world` / `send_timed_left_arm_ee_local` / `send_timed_right_arm_ee_local` | `send_timed_*_arm_ee_{world,local}(pose[6], desire_time=3.0) -> Result` | 单臂末端位姿 `[x, y, z, yaw, pitch, roll]`（planner_index 4/5/6/7；位置米，姿态按 `angle_unit`） | `IHardware` 抽象 |
| Timed | `send_timed_multi_commands` | `send_timed_multi_commands(commands: List[dict], is_sync=False) -> Result` | 多规划器并发（含 `planner_index`/`desire_time`/`cmd_vec`） | `IHardware` 抽象 |
| Timed | `set_ruckig_planner_params` | `set_ruckig_planner_params(planner_index, is_sync, vel/acc/jerk_max, ...) -> Result` | Ruckig 限值设置 | `IHardware` 抽象 |
| Timed | `set_offline_trajectory` / `enable_offline_trajectory` | `set_offline_trajectory(trajectories: List[dict]) -> Result` | 预置 / 启停离线定时轨迹 | `IHardware` 抽象 |
| Timed | `check_ik_accessibility` | `check_ik_accessibility(is_left, is_local, is_whole_body, pose_desired[6], ...) -> Result` | IK 可达性检查 | `IHardware` 抽象 |
| IK | `solve_ik` | `solve_ik(request: IkSolveRequest) -> Result` | 双臂 IK 求解（一次求左右臂），`data` 为 `core.domain.ik.IkSolution` | `IHardware` 抽象 |
| 查询 | `get_transform` | `get_transform(target_frame, source_frame, timeout=0.2) -> Result` | TF 位姿查询，`data` 为 `core.domain.transform.Transform` | `IHardware` 抽象 |
| 查询 | `check_joint_path` | `check_joint_path(start_q14, goal_q14, sampling_deg=3.0, timeout=5.0) -> Result` | 14 关节直线插值路径的碰撞校验（关节角为弧度） | `IHardware` 抽象 |
| 标准 | `set_mpc_mode` | `set_mpc_mode(mode: MPCControlMode) -> Result` | 向 ROS 服务提交模式值；当前固件不据此切换控制分支（见下方说明） | `IHardware` 抽象 |
| 标准 | `enable_quick_mode` | `enable_quick_mode(enable: bool) -> Result` | 手臂/下肢快速模式（绕过 MPC） | `IHardware` 抽象 |
| 标准 | `set_focus_ee` / `set_focus_z` | `set_focus_ee(focus_ee=True) -> Result` | 笛卡尔跟踪焦点 latch | `IHardware` 抽象 |
| 标准 | `control_end_effector` | `control_end_effector(side, cmd) -> Result` | 夹爪 / 灵巧手统一控制 | `IHardware` 抽象 |
| 标准 | `control_head` | `control_head(yaw, pitch) -> Result` | 头部姿态 | `IHardware` 抽象 |
| 标准 | `arm_reset` | `arm_reset() -> Result` | 手臂归位 | `IHardware` 抽象 |
| 标准 | `set_ee_force` / `set_ee_force_both` / `clear_ee_force` | `set_ee_force(side, force_kg=(0,0,0), torque=(0,0,0)) -> Result` | 末端期望力（内部 kg×9.8→N） | `IHardware` 抽象 |
| 标准 | `set_external_wrench` / `clear_external_wrench` | `set_external_wrench(side, force_n=(0,0,0), torque=(0,0,0)) -> Result` | 仿真外力（N） | `IHardware` 抽象 |
| 标准 | `enable_force_empty_detect` | `enable_force_empty_detect(enable) -> Result` | 挥空检测开关 | `IHardware` 抽象 |
| 标准 | `set_contact_force_params` | `set_contact_force_params(transition_time, interpolation_speed) -> Result` | 接触力插值参数 | `IHardware` 抽象 |
| 查询 | `get_reach_time` | `get_reach_time(topic_type: str) -> Optional[float]` | 指令预计到达时间 | `IHardware` 抽象 |
| 查询 | `get_mpc_observation` / `get_mpc_control_mode` / `get_body_acceleration` / `get_joint_torque` / `get_ee_poses` | `get_*() -> Optional[...]` | 状态反馈 | `IHardware` 抽象 |
| 查询 | `get_ee_pose_reach_error` | `get_ee_pose_reach_error(is_left: bool) -> Result` | 末端位姿静差（`data['err_vector']` 6 维） | `IHardware` 抽象 |

> **实现新适配器**：除下文五个「可选能力」导航方法外，表中方法均为 `@abstractmethod`（共 50 个，以 `core/interfaces/i_hardware.py` 为准），漏实现任何一个都会在实例化时抛 `TypeError`。

> **「可选能力」的含义**：`move_chassis_relative` 等五个导航方法在 `IHardware` 里给的是**默认失败实现**（`return Result.fail("当前 Hardware Adapter 不支持底盘相对导航")`），而不是 `@abstractmethod`。没有导航底盘的 Adapter 不必实现它们。

> **`set_mpc_mode` 的当前固件行为**：服务会接收并回显 `MPCControlMode` 值，但控制主循环不按该值选择分支。调用本身仍会访问 ROS 服务，现有流程保留它用于兼容与软暂停守卫；不能凭返回成功判断控制分支已经切换。新流程应按目标能力使用 `enable_quick_mode` / `set_arm_quick_mode` / `set_arm_control_mode`，详见 `09-控制路径与选型.md`。

### `ISkill` —— 技能接口

| 路径 | API | 签名 | 说明 | 所属层 |
|---|---|---|---|---|
| Skill | `name` | `name -> str` | 技能唯一标识（property） | `core.interfaces.i_skill` |
| Skill | `initialize` | `initialize(params: SkillParams) -> Result` | 注入并校验参数 | `core.interfaces.i_skill` |
| Skill | `execute` | `execute() -> Result` | 执行核心逻辑（可被多次 tick 调用） | `core.interfaces.i_skill` |
| Skill | `cancel` | `cancel() -> Result` | 中断技能，确保进入安全状态 | `core.interfaces.i_skill` |
| Skill | `is_finished` | `is_finished() -> bool` | 是否已结束（无论成功或失败） | `core.interfaces.i_skill` |

`ISkill` 是 `ABC`，五个成员全部 `@abstractmethod`。实际编写技能时通常继承 `skills.base.skill_base.SkillBase`（它实现了 `ISkill` 并额外提供超时检查），详见 `04-skills-原子技能.md`。

### `ICamera` / `IPerception`

| 路径 | API | 签名 | 说明 | 所属层 |
|---|---|---|---|---|
| 标准 | `initialize` | `ICamera.initialize(config: dict) -> Result` | 初始化相机 | `core.interfaces.i_camera` |
| 标准 | `shutdown` | `ICamera.shutdown() -> Result` | 关闭相机 | `core.interfaces.i_camera` |
| 查询 | `is_connected` | `ICamera.is_connected() -> bool` | 是否连接 | `core.interfaces.i_camera` |
| 查询 | `get_camera_frame` | `get_camera_frame(camera_name="camera") -> Optional[CameraFrame]` | RGB uint8 HWC + 深度 | `core.interfaces.i_camera` |
| 查询 | `get_synchronized_camera_frame` | `get_synchronized_camera_frame(camera_name="camera") -> Optional[CameraFrame]` | 时间戳配对的 RGBD（可选能力，默认 `None`） | `core.interfaces.i_camera` |
| 查询 | `wait_for_next_synchronized_camera_frame` | `wait_for_next_synchronized_camera_frame(camera_name="camera", timeout_sec=2.0) -> Result` | 等下一组同步 RGBD（不支持时 `fail(error_code="RGBD_SYNC_UNSUPPORTED")`） | `core.interfaces.i_camera` |
| 查询 | `get_depth_data` / `get_point_cloud` / `get_camera_info` / `get_camera_status` | `get_*(camera_name="camera") -> Optional[...]` | 深度 / 点云 / 参数 / 状态 | `core.interfaces.i_camera` |
| 标准 | `start_camera` / `stop_camera` | `start_camera(camera_name="camera") -> bool` | 启停指定相机 | `core.interfaces.i_camera` |
| 标准 | `initialize` | `IPerception.initialize(camera: ICamera, config=None) -> bool` | 注入相机并初始化感知 | `core.interfaces.i_perception` |
| 标准 | `shutdown` | `IPerception.shutdown() -> None` | 释放感知资源 | `core.interfaces.i_perception` |
| 查询 | `get_tag_detections` | `get_tag_detections() -> List[TagDetection]` | AprilTag 检测结果 | `core.interfaces.i_perception` |
| 查询 | `get_object_detections` | `get_object_detections() -> List[ObjectDetection]` | 通用目标检测（预留，当前返回空） | `core.interfaces.i_perception` |
| 查询 | `get_latest_result` | `get_latest_result() -> Optional[PerceptionResult]` | 最近一次完整感知结果 | `core.interfaces.i_perception` |
| 查询 | `get_camera_frame` / `get_point_cloud` / `get_depth_data` / `get_camera_info` / `get_camera_status` | 同 `ICamera` 同名方法 | 委托给注入的 `ICamera` | `core.interfaces.i_perception` |

### 领域数据模型

| 路径 | API | 签名 | 说明 | 所属层 |
|---|---|---|---|---|
| Core | `Result` | `Result(success, message, data=None, error_code=None)` | 统一返回；工厂 `Result.ok()` / `Result.fail()` | `core.domain.result` |
| Core | `Pose6D` | `Pose6D(x=0, y=0, z=0, yaw=0, pitch=0, roll=0)` | 6D 位姿；位置米、姿态**始终弧度** | `core.domain.pose` |
| Core | `TransformMatrix` | `TransformMatrix(matrix: np.ndarray)` | 4×4 齐次变换矩阵，非 4×4 抛 `ValueError` | `core.domain.pose` |
| Core | `SkillParams` | `SkillParams(skill_name="", timeout=30.0, retry_count=3, extra_kwargs={})` | 技能参数基类 | `core.domain.skill_params` |
| Core | `MoveToPoseParams` | 继承 `SkillParams`，加 `target_pose` / `speed_factor=0.5` / `tolerance_pos=0.01` | 移动到指定位姿 | `core.domain.skill_params` |
| Core | `PickObjectParams` | 继承 `SkillParams`，加 `object_label="box"` / `approach_distance=0.1` / `gripper_force=10.0` | 抓取物体 | `core.domain.skill_params` |
| Core | `GripperCommand` | `GripperCommand(position=0.0, velocity=50.0, effort=1.0)` | 二指夹爪：位置 [0,100]（0 张开 / 100 闭合） | `core.domain.end_effector` |
| Core | `DualGripperCommand` | `DualGripperCommand(left_position=None, right_position=None, velocity=50.0, effort=1.0)` | 双侧夹爪单次请求给不同位置 | `core.domain.end_effector` |
| Core | `HandFingerCommand` | `HandFingerCommand(positions=[0.0]*6)` | 灵巧手 6 指，各 [0,100] | `core.domain.end_effector` |
| Core | `MoveToTargetOptions` | `MoveToTargetOptions(avoid_enabled=False, avoid_distance=0.5, linear_velocity=0.15, angular_velocity=0.25, position_threshold=0.08, angle_threshold=0.1, allow_rotation=True)` | 底盘导航选项 | `core.domain.chassis_options` |
| Core | `RuckigParams` | `RuckigParams(velocity_max, acceleration_max, jerk_max, velocity_min=None, acceleration_min=None)`；工厂 `create_chassis_params` / `create_arm_joint_params` / `create_ee_cartesian_params` | 在线轨迹规划限值 | `core.domain.ruckig_params` |
| Core | `TagDetection` | `TagDetection(tag_id, pose_in_world, pose_in_camera=None, size=0.0, confidence=1.0, timestamp=0.0, corner_points=None, frame_id="base_link")` | AprilTag 检测 | `core.domain.perception` |
| Core | `ObjectDetection` | `ObjectDetection(label, confidence, bbox=None, pose_in_camera=None)` | 通用物体检测 | `core.domain.perception` |
| Core | `PerceptionResult` | `PerceptionResult(success, objects=[], tags=[], message="")` | 感知统一返回 | `core.domain.perception` |
| Core | `CameraFrame` / `DepthData` / `PointCloudData` / `CameraInfo` / `CameraStatus` / `CameraIntrinsics` / `CameraExtrinsics` | 相机数据模型 | 帧 / 深度 / 点云 / 参数 / 状态 / 内参 / 外参 | `core.domain.camera` |

## 02.03 参数说明

### `Pose6D` 的单位与顺序（最易错）

- 位置 `x/y/z`：**米**；姿态 `yaw/pitch/roll`：**始终弧度**——**不受** Adapter 的 `angle_unit` 配置影响，与「默认用度」的普通方法参数不同。
- 欧拉角顺序是 **`[x, y, z, yaw, pitch, roll]`**，即 yaw-pitch-roll（ZYX），与底层测试脚本一致。注意这与 `check_ik_accessibility` 期望的 `[x, y, z, roll, pitch, yaw]` 顺序不同。
- 内置方法：

```python
p = Pose6D(x=0.3, y=0.2, z=0.8, yaw=0.0, pitch=0.0, roll=0.0)
p.to_list()        # [0.3, 0.2, 0.8, 0.0, 0.0, 0.0]
p.to_quaternion()  # (qx, qy, qz, qw)

# 用「度」输入时显式声明
p2 = Pose6D.from_euler(0.3, 0.2, 0.8, yaw=0, pitch=-90, roll=0, degrees=True)
```

### `angle_unit` 与 `_to_rad()`

Adapter 创建时可通过 `config` 指定角度单位：

```python
hw = HardwareFactory.create_hardware(config={'robot_type': 'leju_wheeled'})              # 默认：度
hw = HardwareFactory.create_hardware(config={'robot_type': 'leju_wheeled', 'angle_unit': 'rad'})
```

内部统一走 `_to_rad()` 转换（度→弧度 ×π/180，弧度→弧度不变）。**例外**：`send_arm_joint_trajectory`（发布到 `/kuavo_arm_traj`）、`publish_arm_joint_setpoint`（同样发布到 `/kuavo_arm_traj`）和 `send_leg_joint_command`（发布到 `/lb_leg_traj`）把角度直接下发，不做 `_to_rad()`，控制器期望度数，因此即使配了 `angle_unit='rad'`，传给这三个方法的仍应是**度数**。

### 枚举取值

| 枚举 | 取值 | 说明 |
|---|---|---|
| `FrameType` | `KEEP_CURRENT = 0`、`WORLD = 1`、`LOCAL = 2`、`JOINT_SPACE = 5` | 坐标系类型。`WORLD` 基于 odom，`LOCAL` 为 `base_link`，`JOINT_SPACE`=5 用于关节空间（不做 IK） |
| `ArmSide` | `LEFT = "left"`、`RIGHT = "right"`、`BOTH = "both"` | 手臂侧别 |
| `MPCControlMode` | `NO_CONTROL = 0`、`ARM_ONLY = 1`、`BASE_ONLY = 2`、`BASE_ARM = 3`、`ARM_EE_ONLY = 4` | 当前固件接收并回显该值，但不按其选择实际控制分支；`get_mpc_control_mode()` 读到值也不代表分支已切换 |
| `EndEffectorType` | `LEJU_CLAW`、`QIANGNAO_HAND` | 末端执行器类型 |
| `GripperStatus` | （见 `core.domain.end_effector`） | 夹爪状态枚举 |
| `SG100ControlMode` | `JOINT_POSITION` 等 | SG100 灵巧手控制模式，默认 `JOINT_POSITION`，`enable_mask=0x07FF` 表示全部 11 关节 |

> 以上描述针对仓库记录的固件行为；更换运控版本时应重新验证。`set_mpc_mode()` 调用并非空操作，但模式值不应作为实际控制分支的开关。三条控制路径见 [`09-控制路径与选型.md`](./09-控制路径与选型.md)。

### `SkillParams` 的 `timeout`

`SkillParams.timeout` 默认 `30.0` 秒，`SkillBase.execute()` **每次**调用都会检查 `now - start > timeout`，超时返回 `Result.fail("Timeout")` 并打 ERROR 日志。因此对「等待」类技能要显式放大超时，例如 `WaitSeconds` 场景里 `duration_sec=86400` 必须同时给 `timeout=max(dur+60, 120)`，否则必然超时退化成 FAILURE。

## 02.04 返回值

- **`Result`**：绝大多数动作类方法返回它。判定只看 `.success`；`.message` 用于日志，`.error_code` 用于分支处理，`.data` 携带查询结果。
- **`Optional[...]`**：查询类方法在「还没数据」时返回 `None` 而不是 `fail`。典型：`get_ee_poses() -> Optional[Dict]`、`get_camera_frame() -> Optional[CameraFrame]`、`get_mpc_observation() -> Optional[Dict]`。**调用前必须判空**：

```python
poses = hw.get_ee_poses()
if poses is None:
    logger.warning("尚无末端位姿反馈")
else:
    left = poses.get("left")
```

- **`bool`**：`is_connected`、`is_finished`、`IPerception.initialize`、`ICamera.start_camera`。
- **`None`**：`IPerception.shutdown()`。

`Result.ok()` / `Result.fail()` 的结构与示例见 `01-阅读说明.md`。

## 02.05 异常

| 异常 | 触发条件 | 典型位置 |
|---|---|---|
| `KuavoFrameworkError` | 框架基础异常（父类） | `core.common.exceptions` |
| `HardwareConnectionError` | 硬件连接失败或断开 | Adapter 生命周期 |
| `SkillExecutionError` | 技能执行中逻辑错误或超时 | 技能层 |
| `ConfigurationError` | 配置缺失或格式错误 | `HardwareFactory.create_hardware` |
| `KinematicsError` | 逆解等运动学解算失败 | IK / 规划相关 |
| `ValueError` | 领域模型输入非法，如 `TransformMatrix` 非 4×4 | `core.domain.pose` |

## 02.06 调用示例

```python
from core.domain.result import Result
from core.domain.pose import Pose6D
from core.domain.enums import ArmSide, FrameType
from core.domain.end_effector import GripperCommand

# 1) 造一个位姿（姿态用弧度）
left_pose = Pose6D(x=0.3, y=0.2, z=0.8, yaw=0.0, pitch=0.0, roll=0.0)

# 2) 单臂末端位姿（标准接口，自动阻塞等待完成）
result = hw.send_ee_pose(side=ArmSide.LEFT, pose=left_pose, frame=FrameType.WORLD)
if not result.success:
    print(f"失败: {result.message}")

# 3) 夹爪闭合（position=100 闭合，effort=1.0A）
result = hw.control_end_effector(
    side=ArmSide.LEFT,
    cmd=GripperCommand(position=100.0, velocity=50.0, effort=1.0),
)

# 4) 查询类：先判空
poses = hw.get_ee_poses()
if poses is None:
    print("尚无末端反馈")
```

下一步：硬件实现与完整 API 清单见 [`03-adapters-硬件适配层.md`](./03-adapters-硬件适配层.md)；模式值与实际控制路径的区别见 [`09-控制路径与选型.md`](./09-控制路径与选型.md)。
