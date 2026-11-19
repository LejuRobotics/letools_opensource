# refactored_sdk 原子技能集合

从可跑通测试脚本抽取的 Adapter 原子技能，每个技能封装了 **一个** `hardware.xxx()` 调用。

路径基准：技能清单中的文件名相对 `skills/atomic/refactored_sdk/`；仅写文件名的 `*_mixin.py` 相对 `adapters/hardware/leju_wheeled/mixins/`；节点文件位于 `orchestration/nodes/`。其余带目录的项目路径均相对 LeTools 仓库根目录，命令从仓库根目录执行。人工分层测试入口位于 `apps/internal/{adapters,skills,nodes}/`，不要与框架实现目录混淆。

## 架构约定

```
Skill (本层)         →  Node (orchestration/nodes/)  →  JSON 行为树
    │
    └── hardware.xxx()     ← IHardware 接口
         └── _xxx_manager  ← Adapter 内部 SDK 管理服务
              └── robot_sdk.control.xxx()  ← Kuavo 底层 SDK
```

所有需硬件依赖的技能通过 `get_shared_hardware()` 获取 `IHardware` 单例注入；纯编排工具技能无硬件依赖。

---

## 技能清单

### 1. 手臂控制

| 技能文件 | 功能 | 调用的 Adapter 方法 | 底层链路 |
|---------|------|-------------------|---------|
| `arm_ee_traj_local_sdk.py` | 手臂末端轨迹（本体坐标系 `base_link`） | `hardware.send_arm_ee_traj_sdk(frame='base_link')` | `sdk_control_mixin.py` → `ArmSDKManager.move_eef_traj_auto()` → `robot_sdk.control.control_robot_end_effector_pose()` |
| `arm_ee_traj_world_sdk.py` | 手臂末端轨迹（世界坐标系 `world`） | `hardware.send_arm_ee_traj_sdk(frame='world')` | 同上 |
| `arm_joint_timed.py` | 双臂 14 关节单目标控制（TimedCmd） | `hardware.send_arm_joint_timed()` | 一次下发 `[左臂 J0-J6, 右臂 J0-J6]`，等待规划运动完成 |
| `arm_joint_traj_sdk.py` | 手臂 14 关节轨迹（多关键点，自动 MPC） | `hardware.send_arm_joint_traj_sdk()` | `sdk_control_mixin.py` → `ArmSDKManager.move_joint_traj_auto()` → `robot_sdk.control.control_arm_joint_positions()` |
| `set_arm_joint.py` | 双臂关节单目标发送（固定deg） | `hardware.set_arm_joint(joint)` | `ArmMixin` → 已有 `LowLevelSDKManager.control_arm_joint_positions()` → 原生SDK |
| `set_arm_joint_trajectory.py` | 原生双臂带时间点轨迹发送（固定deg） | `hardware.set_arm_joint_trajectory(times, q_frames)` | `ArmMixin` → `LowLevelSDKManager.control_arm_joint_trajectory()` → 原生SDK |
| `get_arm_joint.py` | 双臂14维关节反馈（固定deg） | `hardware.get_arm_joint()` | `ArmMixin` → `LowLevelSDKManager.get_arm_joint()` → SDK state |
| `set_arm_control_mode.py` | 设置手臂模式（整数0/1/2） | `hardware.set_arm_control_mode(control_mode)` | `ArmMixin` → `LowLevelSDKManager.set_arm_control_mode()` → SDK control |
| `get_arm_control_mode.py` | 查询手臂模式及名称 | `hardware.get_arm_control_mode()` | `ArmMixin` → `LowLevelSDKManager.get_arm_control_mode()` → SDK state |
| `set_arm_pose.py` | 双臂末端单目标（xyz米、ZYX欧拉deg） | `hardware.set_arm_pose(left_pose, right_pose, frame='local')` | `ArmMixin` → 已有末端SDKManager调用 → SDK control |
| `get_arm_pose.py` | 双臂末端反馈（m/deg，TF/FK可选） | `hardware.get_arm_pose(frame='local', source='tf', fk_frame=None)` | `ArmMixin` → `LowLevelSDKManager.get_arm_pose()` → SDK tools/FK |
| `get_arm_ik_result.py` | 双臂目标位姿求14维关节解（deg，不发送运动） | `hardware.get_arm_ik_result(left_pose, right_pose, frame='local', ik_frame=..., joint=None)` | `ArmMixin` → `LowLevelSDKManager.get_arm_ik_result()` → SDK arm_ik |
| `get_arm_fk_result.py` | 给定14维关节计算双臂位姿（m/deg，不读反馈） | `hardware.get_arm_fk_result(joint, frame='local', fk_frame=...)` | `ArmMixin` → `LowLevelSDKManager.get_arm_fk_result()` → SDK arm_fk |
| `arm_reset_sdk.py` | 双臂归位到初始姿态 | `hardware.arm_reset()` | `sdk_control_mixin.py` → `ArmSDKManager.arm_reset()` → `robot_sdk.control.arm_reset()` |

**参数说明**：
- 末端轨迹点格式：`[x, y, z, qx, qy, qz, qw]`（位置 + 四元数）
- 关节角度格式：14 个 float，顺序 `[左臂J0-J6, 右臂J0-J6]`（用户角度单位），镜像规则 J0/J3/J6 保持原值，J1/J2/J4/J5 取反
- `arm_joint_timed.py` 接收一个 14 维目标姿态，不是多关键点轨迹；角度遵循 Adapter 的 `angle_unit`，默认是度

### 新双臂关节接口

- `SetArmJointParams(joint=...)`：必填14维角度（deg），左臂7个、右臂7个；无默认运动目标。复用现有单目标SDKManager实现，但不复用旧入口的 `angle_unit` 语义。
- `SetArmJointTrajectoryParams(times=..., q_frames=...)`：必填非空时间点和N×14角度（deg）。`times` 为相对轨迹起点的秒数，必须大于0、严格递增，与每帧一一对应；保留不均匀时间间隔，不将轨迹压缩为 `total_time`。
- 两个技能正常返回后保存 `Result`，重复 execute 不重复发送。成功表示单次SDK调用成功，`data=None`，不等待到位。`timeout` 是技能执行超时，不是运动时长。
- Node 为 `SetArmJoint` / `SetArmJointTrajectory`，读取同名参数，支持列表、JSON字符串和 `READ_BOARD` 运行时绑定。dry-run 校验输入但不初始化硬件。示例结构：`{"name": "SetArmJointTrajectory", "params": {"times": ..., "q_frames": ...}}`，省略号需由实际人工确认数据替换。
- 独立的 `arm_mixin.py`（`ArmMixin`）通过 `convert_angle` 转换deg到rad，Manager使用rad直调原生SDK。注册到现有Hardware，不更改生命周期或旧SDKControlMixin接口。
- 原生SDK单目标会裁剪超限角度；原生轨迹会拒绝超限帧，且要求stance状态。SDK内部可能切到ExternalControl，新封装不额外设置/恢复MPC、不插值、不提供到位保证；相应物理限位由SDK处理。
- 旧 `send_arm_joint_positions_sdk`、`send_arm_joint_traj_sdk`、旧技能和场景均保留原行为，特别是旧轨迹仍走自动MPC/重采样路径，不能当作新接口的同义方法。
- Mock验证见 `tests/test_set_arm_joint.py` 和 `adapters/hardware/leju_wheeled/services/sdk_manager/tests/test_low_level_sdk_manager.py`，不下发真机指令。

### 双臂关节与末端反馈

- `get_arm_joint()`：无业务输入，返回 `Result.data` 为14维反馈角（固定deg）。直接读low Manager的SDK state，旧 `get_arm_joint_positions()` 仍返回rad且依赖arm Manager，不修改其调用和单位。
- `set_arm_pose(left_pose, right_pose, frame='local')`：左右必须同时提供 `[x,y,z,yaw,pitch,roll]`，位置m、欧拉角deg，不接受单臂缺省或默认位姿补齐。Mixin转rad及xyzw四元数再调用已有Manager末端控制；成功只表示发送完成。
- `get_arm_pose(frame='local', source='tf', fk_frame=None)`：返回 `left_pose/right_pose` 两个6维m/deg列表以及 `frame/source`。local映射base_link，world映射odom（不是map）；默认TF读 `zarm_l7_end_effector/zarm_r7_end_effector`。source只接受tf/fk，失败不静默换来源。
- FK读取当前SDK关节反馈（rad）后调用原生 `arm_fk`；必须提供确认后的 `fk_frame`，它应与FK模型输出参考系一致。若和请求系不同，通过SDK TF变换后返回；缺少FK服务或TF返回失败。FK可配置自定义工具偏移，TF与FK对应的末端点可能不同，使用前须确认，不能仅凭名称当成相同实测。
- 不将deg塞进Core的 `Pose6D`；其内部仍使用rad。公开末端接口使用普通6维列表，四元数只在Adapter/Manager边界使用。
- 对应技能/节点为 `GetArmJoint`、`SetArmPose`、`GetArmPose`。单次执行后缓存结果；getter Node按 `output_key` 写黑板（默认arm_joint/arm_pose），初始化清空输出，失败不留下旧值；dry-run成功不伪造反馈，输出为None。
- 反馈均不保证首帧或新鲜度；SDK FK需要可用 `/ik/fk_srv`（上游说明涉及WithIK初始化），当前不改全局SDK初始化。新接口不增加自动到位等待、模式恢复、单臂控制或生命周期。
- 离线验证见 `tests/test_arm_feedback_pose.py` 和low Manager测试。原生joint轨迹在当前轮式仿真的实际执行仍待验证，不能以Mock通过证明物理有效。

### 双臂IK/FK纯计算

- `get_arm_ik_result(left_pose, right_pose, frame='local', ik_frame=None, joint=None)`：左右目标为6维`[x,y,z,yaw,pitch,roll]`（m/deg），必填`ik_frame`为模型基准对应的整机TF frame；可选14维`joint`作为q0（deg），返回`Result.data`为14维关节解（deg）。不发送运动、不自动切模式；未知基准或TF缺失直接失败。
- `get_arm_fk_result(joint, frame='local', fk_frame=None)`：给定14维deg关节计算双臂位姿（m/deg），不读实时关节。输出结构与get_arm_pose相同，包含left_pose/right_pose/frame/source='fk'/fk_frame。现有get_arm_pose(source='fk')只读一次反馈后复用该计算函数。
- 当前v62上半身模型的内部base_link实际对应整机waist_yaw_link，不能仅看同名base_link就认为坐标相同。模型基准及eef_z_offset工具偏移必须人工确认；local=整机base_link，world=odom。跨系计算使用当前TF，不保证模型结果与实测独立或同步。
- 新IK通过low Manager调用SDK原生arm_ik，显式传KuavoIKParams（当前SDK下层不能处理params=None）；SDK默认求解参数不等于旧solve_ik的精度、constraint_mode、主备回退、跳变检查和历史yaw补偿。**旧solve_ik/IkSolveRequest/IkSolution保持不动**，码垛调用及rad/四元数契约不改，不能无感替换成新IK。旧SDK直调业务也暂不自动改写。
- 对应Skill/Node为GetArmIkResult、GetArmFkResult；结果单次缓存，Node写output_key（默认arm_ik_result/arm_fk_result），dry-run仅校验、输出None。原生IK/FK依赖IK服务和模型配置；本次不更改SDK初始化或上游模型。
- 六个独立计算示例在apps/internal的adapters/skills/nodes下；顶部填写参数，纯计算结果不会自动发送，输入空时不初始化硬件。测试见tests/test_arm_kinematics.py和low Manager测试。

### 手臂控制模式

- `get_arm_control_mode()` 无业务输入，通过ArmMixin → LowLevelSDKManager → SDK `state.arm_control_mode()` 查询，返回 `Result.data={"mode": int, "name": str}`。取值0/ArmFixed、1/AutoSwing、2/ExternalControl；不暴露SDK枚举，不切换模式。
- 原生SDK服务查询失败可能返回缓存，因此success不代表控制器在线确认；None、非法返回或异常转为Result.fail。不将这项查询当作运动完成或可靠的在线门禁。
- `GetArmControlModeSkill` 单次执行后缓存结果，重新initialize再查询；`GetArmControlMode` 节点写入output_key（默认arm_control_mode），初始化清空旧值，dry-run不查询并保持None。
- `set_arm_control_mode(control_mode)` 统一为 ArmMixin → LowLevelSDKManager → 原生SDK，0调用set_fixed_arm_mode、1调用set_auto_swing_arm_mode、2调用set_external_control_arm_mode。不再直调ROS服务或fallback；上层位置参数/`control_mode=`签名保留，需初始化low Manager（白名单仅arm/timed或skip_sdk_managers的旧配置须相应调整）。SDK带碰撞检查和模式查询，已处于目标模式时可能跳过切换，模式1不承诺重复触发归位。返回仍为Result、成功data=None，但不是原服务超时/重置行为的逐项等价替代。设置/查询不保证到位，不额外切MPC或恢复模式。
- `SetArmControlModeSkill` / `SetArmControlMode` 复用已有Hardware setter，必填 `control_mode`，只接受整数0/1/2，无默认模式。正常返回后缓存Result、不重复切换，重新initialize后可再次执行；节点支持 `control_mode__board_key` 运行时黑板绑定，dry-run校验参数但不初始化硬件或切换。成功不表示物理归位，不自动恢复模式。
- 三层设置/查询示例在 `apps/internal/{adapters,skills,nodes}/{set,get}_arm_control_mode.py`。setter顶部 `CONTROL_MODE=None`，由人填写后运行，模式1可能触发运动；示例不额外复位、不切MPC、不自动恢复。测试见 `tests/test_arm_control_mode.py` 及low Manager测试。

### 2. 底盘控制

| 技能文件 | 功能 | 调用的 Adapter 方法 | 底层链路 |
|---------|------|-------------------|---------|
| `base_pose_local.py` | 底盘本体坐标系相对位姿移动 | `hardware.send_base_pose(x, y, yaw, frame=LOCAL)` | `base_control_mixin.py` → 适配器内部等待到达时间反馈 |

### 3. 头部与下肢

| 技能文件 | 功能 | 调用的 Adapter 方法 | 底层链路 |
|---------|------|-------------------|---------|
| `head_control_sdk.py` | 头部偏航/俯仰控制（旧入口保留） | `hardware.control_head_sdk(yaw, pitch)` | `sdk_control_mixin.py` → `LowLevelSDKManager.control_head()` → `robot_sdk.control.control_head()` |
| `set_head_pose.py` | 单次发送头部目标角（度） | `hardware.set_head_pose(yaw, pitch)` | `head_mixin.py` → `LowLevelSDKManager.set_head_pose()` → SDK control |
| `get_head_pose.py` | 单次读取头部反馈角（度） | `hardware.get_head_pose()` | `head_mixin.py` → `LowLevelSDKManager.get_head_pose()` → SDK state |
| `leg_joint_sdk.py` | 下肢 4 关节控制 | `hardware.send_leg_joint_sdk()` | `sdk_control_mixin.py` → `LowLevelSDKManager.move_wheel_lower_joint_auto()` → `robot_sdk.control.control_leg_joint_positions()` |

### 头部姿态输入输出

- `SetHeadPoseParams(yaw=..., pitch=...)`：两个角度均必填，单位固定为度，参数名不带单位后缀；不受 Adapter 的 `angle_unit` 影响。技能仅发送一次，不等待到位，成功返回 `Result` 且 `data=None`，失败透传错误信息。
- `GetHeadPoseParams()`：无业务输入；成功返回 `Result.data={"yaw": float, "pitch": float}`，单位固定为度。读取 SDK 的传感器反馈缓存，不是上一次目标角。每次技能初始化后读取一次；再次获取需重新初始化技能。
- 调用链为 Node → Skill → IHardware → HeadMixin → LowLevelSDKManager → SDK。Mixin 使用 math_utils.convert_angle 转换度/弧度，Manager 同名方法直接调用 SDK，不复用旧 control_head。原 Hardware 生命周期和 BaseSDKManager 初始化流程保持不变。
- SDK 缓存没有首帧标记或时间戳，初始化默认零值可能被读出，也无法识别断流后的旧数据。本次仅验证返回值为两维有限角度，不提供反馈有效性/新鲜度保证，不可仅凭一次读取证明到位。
- 行为树节点为 `SetHeadPose` / `GetHeadPose`；getter 的 `output_key` 默认 `head_pose`，将结果字典写入黑板。读取开始清空该键，失败不留下上一次结果；dry-run 不访问硬件、不伪造反馈，输出保持 `None`。
- 旧接口、旧技能节点和场景保留；IHardware 新方法默认返回不支持，不增加抽象方法要求。
- 离线测试见 tests/test_head_pose.py、tests/test_head_pose_app_scripts.py 和 LowLevelSDKManager 的测试，不发送真机指令。

### 4. 通用 TimedCmd

| 技能文件 | 功能 | 调用的 Adapter 方法 | 底层链路 |
|---------|------|-------------------|---------|
| `multi_timed_command.py` | 可配置的多 planner 单次组合控制 | `hardware.set_arm_control_mode(2)`（可选） + `hardware.set_focus_ee()` + `hardware.set_focus_z()` + `hardware.send_timed_multi_commands()` | 将 `commands` 中的 planner 通过一次请求下发，可组合 2/8/9、3/8/9、3/6/7 等 |

`MultiTimedCommandParams.prepare_arm_control` 默认 `True`：包含躯干或手臂 planner 2–9 时，每次发令前调用 `set_arm_control_mode(2)` 并检查返回结果，失败则不下发 TimedCmd。纯底盘 planner 0/1 不切换手臂模式；设为 `False` 时由调用方保证外部控制模式。此处没有独立的模式查询，只确认切换接口成功。不做手臂复位，不在结束后自动切回模式 0，便于连续动作和持箱；释放控制应由场景显式安排。

`focus_ee` 默认 `False`，`focus_z` 内部跟随其值，二者不代替手臂控制模式准备。`MultiTimedCommandMove` 透传以上参数；dry-run 仅初始化并验证命令，不执行模式切换。直接调用 Adapter 的 `send_timed_multi_commands()` 不会经过技能前置准备。

### 5. 编排工具（无硬件依赖）

| 技能文件 | 功能 | 实现方式 |
|---------|------|---------|
| `wait_for_enter.py` | 等待用户按 Enter 后继续 | `input()` 阻塞，DRY_RUN 环境自动跳过 |
| `wait_seconds.py` | 阻塞等待指定秒数 | `time.sleep()`，DRY_RUN 环境自动跳过 |

---

## 对应测试脚本

新增/重构的 11 条上肢接口（三层共 33 个脚本）见[双臂接口分层测试](../../../apps/internal/_arm_test.md)（9 条）与[头部接口分层测试](../../../apps/internal/_head_test.md)（2 条）。两份说明中的命令均从仓库根目录执行，目标参数由操作人员填写；不要批量运行或将 Mock 通过视为真机验收。

| 技能 | 测试脚本 |
|------|---------|
| `arm_ee_traj_local_sdk` | `apps/test_kuavo_5w_refactored/sdk/02_arm/test_arm_ee_traj_local.py` |
| `arm_ee_traj_world_sdk` | `apps/test_kuavo_5w_refactored/sdk/02_arm/test_arm_ee_traj_world.py` |
| `arm_joint_traj_sdk` | `apps/test_kuavo_5w_refactored/sdk/02_arm/test_arm_joint_traj.py` |
| `arm_reset_sdk` | `apps/test_kuavo_5w_refactored/sdk/02_arm/test_arm_reset.py` |
| `base_pose_local` | `apps/test_kuavo_5w_refactored/sdk/01_base_control/test_cmd_pose_base.py` |
| `head_control_sdk` | `apps/test_kuavo_5w_refactored/sdk/06_services/test_head_control.py` |
| `leg_joint_sdk` | `apps/test_kuavo_5w_refactored/sdk/02_lower_body/test_leg_joint.py` |

---

## 关键源码位置

| 层级 | 路径 |
|------|------|
| 旧 SDK 控制入口 | `adapters/hardware/leju_wheeled/mixins/sdk_control_mixin.py` |
| 新双臂接口 | `adapters/hardware/leju_wheeled/mixins/arm_mixin.py` |
| 新头部接口 | `adapters/hardware/leju_wheeled/mixins/head_mixin.py` |
| 底盘适配器 | `adapters/hardware/leju_wheeled/mixins/base_control_mixin.py` |
| Arm SDK 管理器 | `adapters/hardware/leju_wheeled/services/sdk_manager/arm_sdk_manager.py` |
| LowLevel SDK 管理器 | `adapters/hardware/leju_wheeled/services/sdk_manager/low_level_sdk_manager.py` |
| 硬件工厂 | `adapters/hardware/factory.py` |
| 共享硬件单例 | `orchestration/shared_hardware.py` |
| 编排节点（薄封装） | `orchestration/nodes/` |
| 行为树场景 | `orchestration/scenarios/scenarios_internal/refactored_sdk_arm_v1/` |
