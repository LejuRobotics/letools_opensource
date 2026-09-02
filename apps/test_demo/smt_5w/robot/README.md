# 商超机器人 SDK

`supermarket.robot` 是商超业务与 ROS/机器人驱动之间的能力层。状态机和各业务
Agent 只依赖本包公开的稳定接口，不直接创建 ROS publisher、subscriber、service
proxy，也不再依赖旧 `psdk` 包。

本包有两个重要设计目标：

- 在没有 ROS 的开发机上也能安全导入，便于单元测试。
- 把机器人型号差异集中在运行时与工厂中，避免业务状态机出现大量型号分支。

## 1. 文件职责

| 文件 | 职责 | 主要公开对象 |
| --- | --- | --- |
| `__init__.py` | SDK 公共入口；统一导出业务允许使用的类、工厂和异常 | `build_robot_io`、`build_motion_controller`、`ArmController`、`QRRecognizer`、`ObjectPositionTracker` |
| `robot_io.py` | 延迟加载 ROS 依赖；管理节点、消息类型、话题、服务、TF 和 readiness 超时；隔离人形与轮臂差异 | `RobotIO`、`HumanoidRobotIO`、`WheelRobotIO`、`build_robot_io` |
| `arm.py` | 双臂 IK、真实关节反馈、关节单位转换、插值轨迹、安全校验和控制模式切换 | `ArmController`、`IkSolution` |
| `motion.py` | 底盘/步态运动、横移、转向、接近二维码以及失败后的停车 | `HumanoidMotionController`、`WheelMotionController`、`build_motion_controller` |
| `qr.py` | 二维码订阅、去旧数据、扫描头部轨迹、复扫和横向对齐 | `QRRecognizer`、`QRObservation` |
| `vision.py` | YOLO 检测订阅、线程安全缓存、新鲜度检查和限时等待 | `ObjectPositionTracker`、`ObjectObservation` |
| `end_effector.py` | 强脑灵巧手和乐聚夹爪的统一命令、配置校验和有界服务调用 | `EndEffectorController`、`normalize_end_effector_type` |
| `geometry.py` | 无 ROS 的几何纯函数，包括四元数、角度归一化和扫描点生成 | `yaw_from_quat`、`rotate_vector_by_quat`、`scan_head_points` |

`EndEffectorController` 已通过 `Robot.end_effector` 注入 `TaskContext`。默认商超流程中的
水瓶/包裹抓取和放置释放均统一走新 SDK；尚未注册的球体演示仍属于 legacy 边界。

## 2. 依赖方向

依赖只允许从上层指向下层：

```text
SupervisorAgent / 业务 Agent
             |
             v
      supermarket.robot.__init__
             |
             +---- arm.py ------------+
             +---- motion.py ---------+
             +---- qr.py -------------+----> RobotIO 接口 ----> robot_io.py ----> ROS / TF / 驱动
             +---- vision.py ---------+                              |
             +---- end_effector.py ---+                              v
             |                                                    geometry.py
             +---- geometry.py <------------- qr.py              （纯 Python）
```

维护时遵循以下规则：

- `agents.py` 和 `state_machine.py` 不直接导入 `rospy` 或机器人消息类型。
- 控制模块通过 `robot_io` 使用 ROS，不各自初始化 ROS node。
- 机器人类型分支放在 `build_robot_io()`、`build_motion_controller()` 或对应
  RobotIO 子类中，不放进业务 Agent。
- `geometry.py` 保持无副作用、无 ROS 依赖，供各控制模块复用。
- ROS 服务和发布者等待必须有超时，不能永久阻塞状态机。

## 3. 真实模式的构建流程

真实入口位于 `supermarket/run_supermarket.py::build_real_context()`，当前构建顺序
如下：

1. `load_config()` 读取 YAML，并调用
   `supermarket.configuration.apply_robot_defaults()` 复制、校验和补齐默认配置。
2. `build_robot_io(robot_type, ...)` 根据 `robot_type` 创建
   `HumanoidRobotIO` 或 `WheelRobotIO`，在此时才加载 ROS 依赖并初始化节点。
3. `build_motion_controller(robot_io, config)` 根据同一个 RobotIO 创建对应运动控制器。
4. 用同一个 RobotIO 构造 `QRRecognizer`、`ArmController` 和
   `ObjectPositionTracker`。
5. 将 `RobotIO`、motion、arm、QR、vision 和 end-effector 组装成一个轻量 `Robot`；
   保留的 `capture` 合约槽为 `None`，默认流程不再创建第二套 legacy 动作发布器。
6. `_make_context()` 只把 `Robot` 注入 `TaskContext`，再由 `SupervisorAgent` 串行调度。

对应关系为：

```text
supermarket.yaml
      |
      v
apply_robot_defaults
      |
      v
build_robot_io -------+----> QRRecognizer
      |               +----> ObjectPositionTracker
      |               +----> ArmController ----> 统一 IK 服务
      v
build_motion_controller
      |
      v
Robot(io/motion/arm/qr/vision/end_effector/capture)
      |
      v
TaskContext -> SupervisorAgent
```

不要调整为“先创建各控制器、最后初始化 ROS”。二维码和视觉订阅者、手臂反馈订阅
都要求 ROS node 已初始化。

## 4. 调用示例

### 4.1 构建真实机器人能力

业务入口优先从包入口导入，不要从实现文件跨层取私有辅助函数：

```python
from supermarket.robot import (
    ArmController,
    ObjectPositionTracker,
    QRRecognizer,
    build_motion_controller,
    build_robot_io,
)

robot_io = build_robot_io(
    config.get("robot_type", "humanoid"),
    params=config,
    node_name="supermarket_state_machine",
    init_node=True,
)
motion = build_motion_controller(robot_io, params=config)
qr = QRRecognizer(robot_io, config)
arm = ArmController(robot_io, config)
vision = ObjectPositionTracker(robot_io=robot_io)
```

构建入口随后把这些对象统一组装为 `Robot`。所有控制能力共享一个
`robot_io`，业务层只通过 `context.robot.*` 访问，因此节点、话题配置、服务超时和
机器人型号始终一致。

### 4.2 双臂 IK 与连续轨迹

放置动作必须同时提供左右手目标和姿态。多段 IK 使用唯一的 `solve_pose()` 入口，把
上一段的弧度解显式用作下一段 q0：

```python
approach = arm.solve_pose(
    left_xyz=[0.35, 0.20, 0.10],
    right_xyz=[0.25, -0.18, 0.08],
    left_quat=[0.0, 0.0, 0.0, 1.0],
    right_quat=[0.0, 0.0, 0.0, 1.0],
    label="放置接近点",
)

place = arm.solve_pose(
    left_xyz=[0.38, 0.20, 0.06],
    right_xyz=[0.25, -0.18, 0.08],
    left_quat=[0.0, 0.0, 0.0, 1.0],
    right_quat=[0.0, 0.0, 0.0, 1.0],
    label="放置终点",
    q0_joints=approach.seed_joints_rad,
)

arm.move_joints_interpolated(approach.trajectory_joints_deg, duration=2.0)
arm.move_joints_interpolated(
    place.trajectory_joints_deg,
    duration=1.5,
    start_joints=approach.trajectory_joints_deg,
)
```

注意：`seed_joints_rad` 是 IK q0 使用的弧度，`trajectory_joints_deg` 是
`/kuavo_arm_traj` 使用的角度，不能混用。业务代码只使用结构化的
`solve_pose()` 结果。

### 4.3 运动、二维码和视觉

```python
# 扫描指定二维码；失败会抛出明确的 QR 异常，不返回含糊的旧缓存。
pose = qr.scan(target_id=3)
motion.walk_to_qr(pose, approach_distance=0.01)
refined_pose = qr.scan_after_walk(target_id=3, motion=motion)

# 只读取新鲜的 YOLO 结果；没有结果时在限定时间内等待。
position = vision.wait_for_position(object_id=0, timeout=2.0, max_age=0.5)
```

轮臂还提供按时长和步数插值的躯干相对移动接口；人形控制器不提供此方法。
驱动话题接收的是绝对位姿，因此控制器会读取开环状态，再向绝对目标插值：

```python
motion.move_torso_relative_xyz(
    dx=0.10,
    dy=0.0,       # 轮臂没有躯干 y 自由度
    dz=-0.05,
    duration=2.0,
    steps=100,
)
```

多段移动若要始终相对同一个固定起点，应复用同一份初始位姿：

```python
initial_pose = robot_io.get_torso_open_loop_pose()
motion.move_torso_relative_xyz(dx=0.0, dz=0.50, initial_pose=initial_pose)
motion.move_torso_relative_xyz(dx=0.20, dz=0.50, initial_pose=initial_pose)
motion.move_torso_relative_xyz(dx=0.0, dz=0.0, initial_pose=initial_pose)
```

每一帧都向 `/cmd_lb_torso_pose` 发布完整的绝对 `x/z/yaw/pitch` 位姿；完成后
等待 `/lb_torso_pose_reach_time`。未显式传入 `yaw`、`pitch` 时保持参考姿态。
调用期间不能混发 `/lb_leg_traj`；软件控制暂停期间发布的新消息会被驱动丢弃。

调用方必须检查布尔返回值或捕获 `RobotControlError`、`MotionError`、`QRError`、
`VisionError` 等明确异常。失败后不要继续执行后续机械动作。

## 5. 配置与可测试性

- 话题名、服务名和 readiness 超时由配置覆盖；默认值集中在
  `RosTopics`、`RosServices`、`RosTimeouts`。
- `apply_robot_defaults()` 不原地修改调用方传入的原始配置。
- ROS 依赖在 RobotIO 实例化时延迟加载，因此下面的导入在无 ROS 环境也应成功：

```python
from supermarket.robot import ObjectPositionTracker, build_robot_io
from supermarket.robot.geometry import normalize_angle
```

- 单元测试应向控制器注入 fake RobotIO，不要依赖 ROS master。
- 业务状态机变化还必须补充或更新状态机单元测试。

## 6. 扩展新的机器人类型

新增机器人（例如 `crawler`）时按以下顺序扩展：

1. 在 `robot_io.py` 新增 `CrawlerRobotIO(RobotIO)`，只覆写与现有机器人
   确有差异的能力，例如位姿变换、站立命令或控制模式切换。
2. 在 `_normalize_robot_type()` 注册规范名称和允许的别名。
3. 在 `build_robot_io()` 增加 `crawler` 到 RobotIO 的映射。
4. 如果运动学不同，在 `motion.py` 新增 `CrawlerMotionController`，实现业务实际使用
   的统一接口：`stop()`、`stance()`、`turn_180()`、`walk_to_qr()` 和
   `lateral_adjust()`。
5. 在 `build_motion_controller()` 注册新的控制器。
6. 如果双臂 IK 请求格式不同，优先在新的 RobotIO 中提供消息/参数构造能力；只有
   求解策略本身不同才在 `arm.py` 增加小范围分支，不能把型号判断扩散到 Agent。
7. 为 RobotIO、motion、TF 变换和 IK 返回转换补 fake-RobotIO 单元测试。
   真实上机前检查话题、服务、单位和超时配置。

完成这些步骤后，`run_supermarket.py` 和业务状态机通常不需要增加新型号分支。

## 7. 当前 `yolo_object_capture` 迁移边界

默认商超主场景的水瓶和包裹均使用 v2 抓取契约，直接复用扫描阶段选中的坐标，并通过
`RobotIO`、`ArmController` 和 `EndEffectorController` 执行动作。剩余边界如下：

- `yolo_object_capture.yolo_cylinder_capture`：已完成 v2 迁移；两段 IK、轨迹、头部和
  末端动作均走新 API，并把安全持物末态写回放置链。
- `yolo_object_capture.yolo_sphere_capture`：仍是独立 legacy 演示脚本，尚未实现
  `pick_and_hold()`，默认商超配置不再注册它。
- `yolo_object_capture.utils.tools.KuavoMotionController`：仅未迁移的独立球体演示仍使用；
  默认商超启动不再创建该控制器。
- `yolo_object_capture.yolo_package_capture` 已使用 v2 点目标接口；它复用扫描阶段
  选中的坐标，通过唯一的 `ArmController.solve_pose()` 入口连续求解预抓取、抓取和
  抬升目标，再调用 `move_joints_interpolated()` 和
  `EndEffectorController`。最终 IK seed/关节会交给后续放置链，不属于上述 legacy 边界。
- `yolo_object_capture.yolo_sphere_capture` 当前没有商超要求的 `pick_and_hold()`，
  仍只能独立运行；在补齐 v1/v2 抓取契约前不能作为有效的商超 `capture_module`。
- 放置后的末端释放已统一调用 `EndEffectorController`，不再回退旧末端工具。

后续只需迁移球体演示：补齐 v2 `pick_and_hold()`，改用正式能力层，并在实机验证后
删除仅由球体使用的 legacy 工具。

因此，开发时不能直接删除 `yolo_object_capture`，也不要新增从
`supermarket.robot` 反向依赖 legacy 包的代码。
