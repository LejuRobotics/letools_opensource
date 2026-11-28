# refactored_sdk 原子技能集合

从测试脚本抽取的原子技能，基于 `IHardware` 组合发令和模式准备；具体副作用以各技能参数为准。

路径基准：技能清单中的文件名相对 `skills/atomic/refactored_sdk/`；仅写文件名的 `*_mixin.py` 相对 `adapters/hardware/leju_wheeled/mixins/`；节点文件位于 `orchestration/nodes/`。其余带目录的项目路径均相对 LeTools 仓库根目录，命令从仓库根目录执行。人工分层测试入口位于 `apps/test_internal/{adapters,skills,nodes}/`，不要与框架实现目录混淆。

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

#### 六种新控制技能的完成语义

`set_arm_joint/pose`、`set_arm_joint_traj/pose_traj`、`timed_arm_joint/pose` 及对应 Node 只执行一次发送调用并缓存结果，不包含反馈到位检查。`is_finished()` 表示已有调用结果，节点 SUCCESS 不保证机械臂到位；不会自动调用 `get_arm_joint()` / `get_arm_pose()` 轮询反馈。

Timed 技能透传服务结果，不根据 `actual_time` 等待机械运动完成；同步 SDK 轨迹仍阻塞当前 tick，发送调用返回后即完成技能，不追加末帧验收。`timeout` 仍为技能生命周期超时，不是运动时长或到位等待时长。既有模式准备、focus 设置及 Adapter 反馈查询接口保持不变，详见下文各接口说明。

| 技能文件 | 功能 | 调用的 Adapter 方法 | 底层链路 |
|---------|------|-------------------|---------|
| `arm_joint_traj_sdk.py` | 手臂 14 关节轨迹（多关键点，自动 MPC） | `hardware.send_arm_joint_traj_sdk()` | `sdk_control_mixin.py` → `ArmSDKManager.move_joint_traj_auto()` → `robot_sdk.control.control_arm_joint_positions()` |
| `set_arm_joint.py` | 双臂关节单目标发送（固定deg） | `hardware.set_arm_joint(joint)` | `ArmMixin` → 已有 `LowLevelSDKManager.control_arm_joint_positions()` → 原生SDK |
| `set_arm_joint_traj.py` | ArmAPI按总时间重采样双臂关节轨迹（deg） | `hardware.set_arm_joint_traj(joint_traj, total_time)` | `ArmMixin` → LowLevelSDKManager → ArmAPI.move_joint_traj |
| `set_arm_pose_traj.py` | ArmAPI轮式双臂末端轨迹（m/deg） | `hardware.set_arm_pose_traj(left_traj, right_traj, total_time, ...)` | `ArmMixin` → LowLevelSDKManager → ArmAPI.move_eef_traj_wheel_mpc |
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

### 新双臂关节接口

- `SetArmJointParams(joint=...)`：必填14维角度（deg），左臂7个、右臂7个；无默认运动目标。复用现有单目标SDKManager实现，但不复用旧入口的 `angle_unit` 语义。
- 单目标技能正常返回后保存 `Result`，重复 execute 不重复发送。成功表示单次SDK调用成功，`data=None`，不等待到位。`timeout` 是技能执行超时，不是运动时长。
- 单目标节点为 `SetArmJoint`，读取joint参数，支持列表、JSON字符串及READ_BOARD运行时绑定；dry-run校验参数但不访问硬件。
- 独立的 `arm_mixin.py`（`ArmMixin`）通过 `convert_angle` 转换deg到rad，Manager使用rad直调原生SDK。注册到现有Hardware，不更改生命周期或旧SDKControlMixin接口。
- 原生SDK单目标会裁剪超限角度。SDK内部可能切到ExternalControl，新封装不额外设置/恢复MPC、不插值、不提供到位保证；相应物理限位由SDK处理。
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
- 离线验证见 `tests/test_arm_feedback_pose.py` 和low Manager测试。Mock通过不证明实际运动或物理解算精度。

### 双臂IK/FK纯计算

- `get_arm_ik_result(left_pose, right_pose, frame='local', ik_frame=None, joint=None)`：左右目标为6维`[x,y,z,yaw,pitch,roll]`（m/deg），必填`ik_frame`为模型基准对应的整机TF frame；可选14维`joint`作为q0（deg），返回`Result.data`为14维关节解（deg）。不发送运动、不自动切模式；未知基准或TF缺失直接失败。
- `get_arm_fk_result(joint, frame='local', fk_frame=None)`：给定14维deg关节计算双臂位姿（m/deg），不读实时关节。输出结构与get_arm_pose相同，包含left_pose/right_pose/frame/source='fk'/fk_frame。现有get_arm_pose(source='fk')只读一次反馈后复用该计算函数。
- 当前v62上半身模型的内部base_link实际对应整机waist_yaw_link，不能仅看同名base_link就认为坐标相同。模型基准及eef_z_offset工具偏移必须人工确认；local=整机base_link，world=odom。跨系计算使用当前TF，不保证模型结果与实测独立或同步。
- 新IK通过low Manager调用SDK原生arm_ik，显式传KuavoIKParams（当前SDK下层不能处理params=None）；SDK默认求解参数不等于旧solve_ik的精度、constraint_mode、主备回退、跳变检查和历史yaw补偿。**旧solve_ik/IkSolveRequest/IkSolution保持不动**，码垛调用及rad/四元数契约不改，不能无感替换成新IK。旧SDK直调业务也暂不自动改写。
- 对应Skill/Node为GetArmIkResult、GetArmFkResult；结果单次缓存，Node写output_key（默认arm_ik_result/arm_fk_result），dry-run仅校验、输出None。原生IK/FK依赖IK服务和模型配置；本次不更改SDK初始化或上游模型。
- 六个独立计算示例在apps/test_internal的adapters/skills/nodes下；顶部填写参数，纯计算结果不会自动发送，输入空时不初始化硬件。测试见tests/test_arm_kinematics.py和low Manager测试。

### 控制器端MultiCmd关节目标

- `timed_arm_joint(left_arm_joint=None, right_arm_joint=None, desire_time=None, is_sync=True)`：左右各7维deg，至少提供一侧；None表示不提交，空列表/错误维度/非有限值失败。desire_time必须为大于0的有限秒数。固定deg，不受旧angle_unit影响。
- 调用链为Node → Skill → IHardware → 新 `TimedCmdMixin`（timedcmd_mixin.py）→ 已有TimedRosManager.send_multi_commands → `/mobile_manipulator_timed_multi_cmd`。左臂只发planner8，右臂只发planner9，双臂在同一请求中发8+9，角度在新Mixin中只转换一次到rad。
- `is_sync=True`统一本次规划器时长，**不是Python等待机械完成**。Result.data透传服务的actual_time；不是到位实测，也不是硬速度上限。服务返回失败不伪装成功。
- 未提交的另一臂不补零、不读反馈来合成保持指令，不停止或覆盖其已有轨迹；并发/之前未结束的运动仍可继续。运动Adapter不切模式；Skill/Node默认 `prepare_arm_control=True`，先请求 `set_arm_control_mode(2)` 成功后提交目标，False时跳过。失败/异常缓存且不发令，不经模式1、不自动恢复或重试。切2可能激活残留目标，调用前须确认控制器状态，不能当作清空、停止或实时模式确认。
- 与 `set_arm_joint_traj`（原生ArmAPI客户端100Hz重采样）分开；ROS1 旧双臂 send_arm_joint_timed 已退役，send_timed_multi_commands 保留；ROS2 同名兼容接口保留。需ROS timed Manager；SDK low仅用于模式准备。
- Skill/Node为TimedArmJoint；节点还支持互斥的14D `joint_angles` 输入（运行时拆分），以及 `actual_time_output_key` 输出时长（字段缺失（含无返回数据）回退本次desire_time），供独立WaitSeconds读取。发送技能不等待。其他参数同名，节点支持JSON及运行时黑板输入，dry-run仅校验不访问硬件。示例在apps/test_internal三层目录，现场参数以脚本为准；Adapter仅需ROS timed，Skill/Node默认模式准备另需low Manager。不提供focus参数。离线测试见tests/test_timed_arm_joint.py。

### 控制器端MultiCmd末端目标

- `timed_arm_pose(left_pose=None, right_pose=None, desire_time=None, frame='local', is_sync=True)`：至少提供一侧6维 `[x,y,z,yaw,pitch,roll]`，固定m/deg；时长必填大于0。None表示不提交该臂，不补默认位姿、不停止其既有运动。
- `local/base_link`对应左planner6右planner7；`world/odom`对应左4右5。未知frame直接失败。位置保持米，只有姿态deg→rad，独立于旧angle_unit配置。
- 复用TimedRosManager的multi服务，双臂仅一次请求；is_sync仅表示规划时长同步，返回actual_time但不等待到位。运动Adapter不切模式、不改控制流、不自动归位或停止。
- Skill/Node默认 `prepare_arm_control=True`，先调用 `set_arm_control_mode(2)`，返回成功后才设置focus并提交目标；设False可跳过，由调用方负责准备。模式准备失败/异常会缓存失败，不设置focus、不提交目标，不经模式1、不自动切回或重试。Skill/Node示例启用ROS timed + SDK low Manager（模式入口需low），Adapter仍只需ROS timed。切2可能激活控制器已有目标，调用前必须人工确认无危险残留；此选项不是清空/停止机制，也不保证SDK缓存与实机模式一致。
- `TimedArmPoseParams` 中的 `focus_ee`、`focus_z` 分别接受bool或None，默认None不修改已有焦点；显式值由Skill按ee→z调用独立Adapter话题封装，再调用纯运动 `timed_arm_pose`，不作为MultiCmd请求参数。ee=True末端优先（躯干可能参与）、False躯干优先；z为Z焦点跟随开关。设置失败不发令，失败Result原样透传；已生效的设置不回滚、结束不恢复，两次发布与服务请求非原子，也不保证控制器已应用。focus是全局持久状态，单臂请求也可能影响全身，False/False不保证躯干绝对不动。不新增独立focus Skill/Node。
- 对应 `TimedArmPoseSkill` / `TimedArmPose`，支持运行时黑板输入与dry-run校验。旧末端Timed接口保留，不改写旧场景。三层人工示例为 `apps/test_internal/{adapters,skills,nodes}/timed_arm_pose.py`，Adapter示例只发目标，Skill/Node示例支持focus；现场已填写的目标与时长以文件为准；测试见 `tests/test_timed_arm_pose.py`。

### SDK ArmAPI定时执行

- `set_arm_joint_traj(joint_traj, total_time)`：至少两帧14维角度（固定deg），Mixin转rad，LowLevelSDKManager复用RobotSDK惰性创建原生ArmAPI，调用 `move_joint_traj(asynchronous=False, mpc_ctrl_mode=ArmOnly)`。SDK按总时间100Hz重采样关键点，不接收每帧时间；第一帧应由调用方明确提供，不自动读取反馈或插入当前状态。
- `set_arm_pose_traj(left_traj, right_traj, total_time, frame='local', direct_to_wbc=True, back_default=False)`：双臂必须等长、至少两帧6维`[x,y,z,yaw,pitch,roll]`（m/deg），转换为米/xyzw四元数后调用原生 `move_eef_traj_wheel_mpc(asynchronous=False, mpc_ctrl_mode=ArmOnly)`。默认local，支持world/odom。这里只支持轨迹，不将单个目标自动补成两帧。
- `total_time` 必填有限且至少0.01秒，避免SDK不足一个100Hz周期仅下发首点。`direct_to_wbc=True`要求SDK切DirectToWbc；False并不主动切回ThroughFullBodyMpc。`back_default=True`结束时请求NoControl/ThroughFullBodyMpc，不是恢复调用前状态，也不切回手臂模式0/1；默认False，异常时不承诺恢复。
- 两者均由SDK准备ExternalControl/ArmOnly，直接使用原生ArmAPI；不替换旧ArmSDKManager自研重采样和TimedCmd服务。新轨迹不接收每帧times，不调用 `/mobile_manipulator_timed_single_cmd`。旧的本仓原生带时间点轨迹封装已经删除，不保留兼容别名。
- 同步调用阻塞Adapter/Skill及Node当前tick，节点不适合要求可取消的Parallel。SDK的同步返回值是None，不按bool判断；Result.success仅表示ArmAPI调用未抛异常，不保证模式切换或每帧发送的返回值成功。SDK内部不汇总这些bool，也不保证实测到位，因此不提供arrived或actual_time。SDK重采样实际耗时可能长于total_time。
- 末端四元数沿用SDK的分量线性插值，不改为SLERP；封装仅对关键点归一化并对齐相邻等价符号，避免q/-q插出零值，不是完整笛卡尔路径或姿态速度规划。
- 对应Skill/Node为SetArmJointTraj、SetArmPoseTraj，单次执行缓存结果；dry-run仅校验不访问硬件。SkillBase超时只在调用前检查，不能中途取消同步SDK循环。SDK ArmAPI执行池由Manager关闭时释放，不自动复位机器人；同一Manager不要并发执行轨迹或shutdown。
- 三层人工脚本在 `apps/test_internal/{adapters,skills,nodes}/set_arm_joint_traj.py` 和 `set_arm_pose_traj.py`，顶部参数留空，时长None，无命令行参数。验证见 `tests/test_timed_arm_api.py` 及low Manager测试，仅Mock，不下发指令。

### 手臂控制模式

- `get_arm_control_mode()` 无业务输入，通过ArmMixin → LowLevelSDKManager → SDK `state.arm_control_mode()` 查询，返回 `Result.data={"mode": int, "name": str}`。取值0/ArmFixed、1/AutoSwing、2/ExternalControl；不暴露SDK枚举，不切换模式。
- 原生SDK服务查询失败可能返回缓存，因此success不代表控制器在线确认；None、非法返回或异常转为Result.fail。不将这项查询当作运动完成或可靠的在线门禁。
- `GetArmControlModeSkill` 单次执行后缓存结果，重新initialize再查询；`GetArmControlMode` 节点写入output_key（默认arm_control_mode），初始化清空旧值，dry-run不查询并保持None。
- `set_arm_control_mode(control_mode)` 统一为 ArmMixin → LowLevelSDKManager → 原生SDK，0调用set_fixed_arm_mode、1调用set_auto_swing_arm_mode、2调用set_external_control_arm_mode。不再直调ROS服务或fallback；上层位置参数/`control_mode=`签名保留，需初始化low Manager（白名单仅arm/timed或skip_sdk_managers的旧配置须相应调整）。SDK带碰撞检查和模式查询，已处于目标模式时可能跳过切换，模式1不承诺重复触发归位。返回仍为Result、成功data=None，但不是原服务超时/重置行为的逐项等价替代。设置/查询不保证到位，不额外切MPC或恢复模式。
- `SetArmControlModeSkill` / `SetArmControlMode` 复用已有Hardware setter，必填 `control_mode`，只接受整数0/1/2，无默认模式。正常返回后缓存Result、不重复切换，重新initialize后可再次执行；节点支持 `control_mode__board_key` 运行时黑板绑定，dry-run校验参数但不初始化硬件或切换。成功不表示物理归位，不自动恢复模式。
- 三层设置/查询示例在 `apps/test_internal/{adapters,skills,nodes}/{set,get}_arm_control_mode.py`。setter顶部 `CONTROL_MODE=None`，由人填写后运行，模式1可能触发运动；示例不额外复位、不切MPC、不自动恢复。测试见 `tests/test_arm_control_mode.py` 及low Manager测试。

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

### 躯干单帧输入（新增，不替换旧路径）

| Skill / Node | 必填输入 | SDK 链路 |
|---|---|---|
| `SetTorsoJointSkill` / `SetTorsoJoint` | `joint=[knee,leg,waist_pitch,waist_yaw]`，4维deg | `TorsoMixin` → `LowLevelSDKManager.control_wheel_lower_joint` → SDK同名方法 |
| `SetTorsoPoseSkill` / `SetTorsoPose` | `pose=[x,z,yaw,pitch]`，4维m/deg | `TorsoMixin` → `LowLevelSDKManager.control_torso_6dof` → SDK `control_torso_pose` |

- 参数默认空，不提供默认物理目标；拒绝维度错误、bool、NaN/inf。公共单位不受 `angle_unit` 影响。
- 关节保持deg下发；位姿映射SDK `(x,0,z,0,pitch_rad,yaw_rad)`，固定y/roll为0并将角度转rad。参考语义沿用控制器当前配置，不提供frame切换或TF变换。
- 使用已初始化的 `low` Manager。只调用一次并缓存Result（包括失败/异常）；成功data=None，仅表示SDK报告发送成功。Skill不自动切模式/focus、不插值、不sleep、不查询反馈、不保证单帧能完成运动。取消不撤销已发送目标。
- Node支持JSON数组与运行时READ_BOARD；dry-run只校验，不获取硬件。已有SDK/Timed/ROS接口、Skill和场景均保留，不能将带插值的旧动作直接换成单帧节点。
- 离线回归：`tests/test_set_torso.py`，全部使用Mock，不下发真机指令。
- 三层人工入口：`apps/test_internal/{adapters,skills,nodes}/set_torso_joint.py` 和 `set_torso_pose.py`，目标默认空，先校验后初始化硬件，仅启用low Manager；说明见 `apps/test_internal/_torso_test.md`。

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

新增/重构的 11 条上肢接口（三层共 33 个脚本）见[双臂接口分层测试](../../../apps/test_internal/_arm_test.md)（9 条）与[头部接口分层测试](../../../apps/test_internal/_head_test.md)（2 条）。两份说明中的命令均从仓库根目录执行，目标参数由操作人员填写；不要批量运行或将 Mock 通过视为真机验收。

| 技能 | 测试脚本 |
|------|---------|
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
| 行为树场景 | `orchestration/scenarios/scenarios_internal/refactored_sdk_atomic_v1/` |

## 躯干 Timed 控制补充

| IHardware方法 | Skill / Node | 必填目标 | planner |
|---|---|---|---|
| `timed_torso_joint(joint, desire_time, is_sync=True)` | `TimedTorsoJointSkill` / `TimedTorsoJoint` | `[knee,leg,waist_pitch,waist_yaw]`，4维deg | 3 |
| `timed_torso_pose(pose, desire_time, is_sync=True)` | `TimedTorsoPoseSkill` / `TimedTorsoPose` | `[x,z,yaw,pitch]`，4维m/deg | 2 |

- 均由 `TimedCmdMixin` → `TimedRosManager.send_multi_commands()` 发一次MultiCmd请求（即便只有一个planner），不经旧的全局angle_unit换算路径。关节4维全部转rad，位姿只转换yaw/pitch；pose不暴露y/roll、不做TF变换。
- `desire_time` 必填大于0有限秒数，`is_sync` 必须为bool，仅表示规划时长同步；目标默认空。Adapter不切模式/focus，不等待，不使用LowLevelSDKManager发送运动目标。
- Skill与Timed手臂保持一致：`prepare_arm_control=True` 默认先通过现有模式接口请求模式2（需要low Manager），失败不发目标；False跳过准备。切2可能激活控制器残留手臂目标，操作者须先确认安全。不自动清空目标、改focus/MPC或恢复模式。
- Skill缓存完整Result及异常失败，不重复发令、不sleep、不检查到位；服务返回 `actual_time` 是规划时长，不是实际反馈或停止证明。
- Node支持运行期READ_BOARD与dry-run参数校验，可选 `actual_time_output_key` 将规划时长写黑板供独立WaitSeconds使用。初始化清空旧输出，只有缺字段时回退desire_time，显式null/bool/负数/非有限值失败；节点自身不等待。dry-run不获取硬件，输出请求时长仅供编排干跑，不伪造实测反馈。
- 人工脚本按Adapter/Skill/Node三层放在 `apps/test_internal/` 对应子目录，公共配置与结果校验统一为 `_timed_test.py`，说明见 `apps/test_internal/_timed_test.md`；两个手臂Timed入口也使用该公共模块。旧SDK/Timed接口和场景不删除、不迁移。

## 躯干反馈与SDK关节轨迹

- `get_torso_joint()`：经 `TorsoMixin → LowLevelSDKManager` 读取SDK `joint_state.position[:4]`（rad），Adapter转换为固定deg的 `[knee,leg,waist_pitch,waist_yaw]`。不受angle_unit影响。缺失、短数组或非法值失败，不补零；缓存不保证首帧/新鲜度，不用于证明到位。
- `set_torso_joint_traj(joint_traj, total_time)`：每帧4维deg，必须是嵌套列表，至少一个关键帧；total_time必填有限数且至少0.01秒（SDK 100Hz采样间隔，非物理限值）。没有默认目标或时长。
- 链路为 `TorsoMixin → LowLevelSDKManager → TorsoAPI.move_wheel_lower_joint(asynchronous=False)`。Manager先校验参数/反馈，再调用SDK。SDK会再次读取状态、自动插入当前起点、请求MPC ArmOnly、均匀分配关键帧并100Hz重采样发送。不额外切手臂模式或focus，也不自动恢复MPC。前置反馈检查不提供原子快照或新鲜度保证。
- SDK同步返回None，Result.success仅表示调用未抛异常，data=None。SDK内部不汇总模式切换和逐帧发送的bool，因此不能保证逐帧接收或实测到位。不提供actual_time；阻塞当前tick，Skill超时/cancel无法中断同步SDK循环，不适合要求可取消的Parallel。
- Skill缓存完整成功或失败结果，重新初始化后才再次读取/执行。`GetTorsoJoint` 的output_key默认torso_joint，初始化先清空旧输出，失败或dry-run保持None。`SetTorsoJointTraj` 支持运行期黑板轨迹/时长与dry-run校验；干跑不访问硬件。
- TorsoAPI使用原有low Manager的RobotSDK，惰性创建；shutdown释放其线程池，不与同步执行并行调用。新接口为IHardware可选方法，旧控制链和场景均保留。
- 人工入口在 `apps/test_internal/{adapters,skills,nodes}/get_torso_joint.py` 与 `set_torso_joint_traj.py`；只启用low Manager，具体见 `apps/test_internal/_torso_test.md`。自动化回归均使用Mock，不执行真机入口。

## MultiCmd ROS管理器迁移

四个新 `timed_arm_joint/pose`、`timed_torso_joint/pose` 及共用 `send_timed_multi_commands` 均由 `TimedRosManager` 直接调用 `/mobile_manipulator_timed_multi_cmd`。新管理器位于 `services/ros_manager/timed_ros_manager.py`，不继承BaseSDKManager、不构造SDK。ROS节点仍由Hardware初始化；Manager初始化只检查ROS就绪，服务发现发生在发送时（5秒，不是整个调用硬截止）。每次调用只发一次，不自动重试、不sleep或切模式。

配置分离：`ros_managers_whitelist=["timed"]` 开启Timed ROS后端（默认同时启用timed、torso、mpc），`skip_ros_managers=True` 或空ROS白名单关闭。`skip_sdk_managers` 仅跳过SDK，不再代表禁用ROS Manager。纯Timed Adapter使用 `sdk_managers_whitelist=[]`；带模式准备的Skill/Node使用 `["low"]`。ROS与SDK白名单中的timed不是同一个管理器。

旧TimedCmdManager的SDK SingleCmd、Ruckig/离线轨迹/IK可达性路径本次保留，其send_multi_commands仅兼容转发到TimedRosManager，协议构造不再重复。旧sdk白名单仍可使用，不强迫旧场景迁移；已有默认硬件配置会同时保留旧SDK能力并初始化新ROS后端。旧共用入口仍按angle_unit转换，新四接口固定m/deg；Result/actual_time、planner与is_sync语义不变。关闭Manager不停止或撤销控制器目标。

## 躯干当前姿态、初始姿态与复位

| 接口 | 后端 | 成功data |
|---|---|---|
| `get_torso_pose()` | TorsoMixin → LowLevelSDKManager → SDK TF查询 | `{pose:[x,z,yaw,pitch],frame:'base_link',link:'waist_yaw_link',source:'tf'}` |
| `get_torso_pose_initial()` | TorsoMixin → TorsoRosManager → 初始姿态服务 | `{pose:[x,z,yaw,pitch],frame:None,source:'initial_service'}` |
| `reset_torso()` | TorsoMixin → TorsoRosManager → 复位服务 | `{accepted:True,estimated_time:float或None}` |

两种pose公开单位m/deg。当前位姿只查waist_yaw_link相对base_link，不用目标或初始姿态作fallback，不保证TF缓存新鲜度；忽略反馈y/roll只返回四分量，并非将原始TF强制置零。初始服务响应无frame字段，保留来源信息，不伪称做过base_link变换或读取当前实测；服务angular按ROS角度rad转换为公共deg。

复位通过 `/mobile_manipulator_reset_torso` 的SetBool(True)提交，初始查询通过 `/mobile_manipulator_get_torso_initial_pose` 的getLbTorsoInitialPose(isNeed=True)。TorsoRosManager不依赖SDK、不自行init_node，服务发现5秒不代表整调用硬超时，无自动重试。复位成功只代表接受；服务message可解析为非负有限时间时填estimated_time，否则None，不将已接受误报为失败。

两个查询Node的output_key默认torso_pose/torso_pose_initial，初始化清空、失败/dry-run保持None；Skill单次缓存Result，重复tick不重复查询/复位。ResetTorso不切额外模式，不等待或检查到位。旧命名get_torso_initial_pose/reset_torso_to_initial及旧技能均保留。

配置：get_torso_pose需SDK low；初始查询/复位仅需 `ros_managers_whitelist=["torso"]` 且SDK白名单可为空。ROS管理器默认启用timed+torso+mpc，空白名单/skip_ros_managers可禁用。三层人工脚本在apps/test_internal对应目录，reset_torso.py默认CONFIRM_RESET=False，在初始化硬件前拦截，必须人工确认现场安全；关闭硬件不等于撤销已接受的复位。

## 躯干Ruckig持久规划限制

`timed_torso_ruckig_params(mode, velocity_max, acceleration_max, jerk_max, velocity_min=None, acceleration_min=None, is_sync=True)` 经 `TorsoMixin → TimedRosManager` 调用 `/mobile_manipulator_set_ruckig_planner_params`，不初始化SDK。

- mode必填：pose对应planner2，四维[x,z,yaw,pitch]；joint对应planner3，四维[knee,leg,waist_pitch,waist_yaw]。不接受其他planner，不按Timed手臂编号套用。
- 三个max均必填4维正有限数，两个min若显式填写必须为4维负有限数；省略时明确取对应-max，允许不对称幅值。拒绝bool、NaN/inf、零、错维度，不提供默认物理限制。
- 公开速度/加速度/jerk单位依次为m/s、m/s²、m/s³或deg/s、deg/s²、deg/s³；pose前两维为线量、后两维角量，joint全为角量。Adapter将所有角量导数统一转rad制，包括下界，不受angle_unit影响。
- is_sync为规划同步配置，不是Python等待运动完成。服务成功消息透传，data=None；失败/异常返回失败，不自动重试。服务发现5秒不代表整个调用硬超时。
- 参数会持续影响后续规划，不仅当前动作；这是规划限制，不是硬件安全限值。不发送目标、不切模式/focus、不自动恢复。Skill每次初始化仅设置一次并缓存成功或失败；Node支持运行期黑板和dry-run校验，不增加输出键。
- 三层人工入口：`apps/test_internal/{adapters,skills,nodes}/timed_torso_ruckig_params.py`。MODE默认空，三个max默认空，先校验再初始化硬件；仅启用ROS timed，SDK白名单为空。操作者确认物理限制后手动执行，不自动配置真机。旧Ruckig接口不改动。

## 独立MPC ROS接口

六接口通过MpcMixin → MpcRosManager，不依赖SDK：

| 设置/查询 | 输入/返回字段 | 值与名称 |
|---|---|---|
| set/get_mpc_control_mode | mode | 0 NoControl、1 ArmOnly、2 BaseOnly、3 BaseArm |
| set/get_mpc_control_flow | flow | 0 ThroughFullBodyMpc、1 DirectToWbc |
| set/get_mpc_frame | frame | 0 KeepCurrentFrame、1 WorldFrame、2 LocalFrame、3 VRFrame、4 ManipulationWorldFrame |

setter只接受严格整数（不接受bool/浮点/字符串，Node JSON解析后再校验），不接受旧ARM_EE_ONLY=4作为mode。旧Core枚举与旧set_mpc_mode保持不变。新setter成功data=None，仅表示服务接受，不额外查询、切手臂模式、改变其他项或恢复。frame=0表示保持当前，不要求回显为0。

getter每次Skill初始化只调用一次查询服务，成功data为对应字段与name的字典，如 `{mode:1,name:'ArmOnly'}`。服务拒绝/异常/非法返回值直接失败，不回退本地缓存。Node输出键默认mpc_control_mode/mpc_control_flow/mpc_frame，初始化清空，失败/dry-run保持None。重复execute/tick缓存完整Result，不重发；服务发现5秒不代表整个调用硬截止。

按用户确认，旧 `get_mpc_control_mode()->Optional[int]` 改名为 `get_mpc_control_mode_cached()`，保留旧缓存/ROS2查询行为并更新调试脚本。新的同名getter返回Result；ROS2目前明确返回不支持新ROS1服务契约，其他旧能力不变。外部未纳入仓库的旧调用者需自行改用cached名称。

ROS管理器默认启用timed、torso、mpc；可用ros_managers_whitelist=["mpc"]只启用本组，sdk_managers_whitelist=[]。不会自行init_node或初始化SDK。当前固件对模式的实际消费能力须现场确认，服务接受/回显不是安全隔离、停止或控制行为生效证明。SDK已有模式调用路径不被此次替换，旧场景保留。

人工入口在apps/test_internal的adapters/skills/nodes下，六个同名脚本。setter TARGET_MODE/TARGET_FLOW/TARGET_FRAME默认None，必须人工确认后填写；getter不发送设置请求；Node人工入口拒绝dry-run。公共配置与校验见apps/test_internal/_mpc_test.py。
