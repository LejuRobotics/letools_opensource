# 双臂接口分层测试

路径基准：本文所有文件路径均相对 LeTools 仓库根目录，命令也从仓库根目录执行。脚本按头部模板组织，不接受命令行参数，每层自行展示初始化、测试调用和关闭。

## 测试脚本

| 接口 | Adapter | Skill | Node |
|---|---|---|---|
| 设置关节目标 | `apps/internal/adapters/set_arm_joint.py` | `apps/internal/skills/set_arm_joint.py` | `apps/internal/nodes/set_arm_joint.py` |
| 设置原生关节轨迹 | `apps/internal/adapters/set_arm_joint_trajectory.py` | `apps/internal/skills/set_arm_joint_trajectory.py` | `apps/internal/nodes/set_arm_joint_trajectory.py` |
| 读取关节反馈 | `apps/internal/adapters/get_arm_joint.py` | `apps/internal/skills/get_arm_joint.py` | `apps/internal/nodes/get_arm_joint.py` |
| 查询控制模式 | `apps/internal/adapters/get_arm_control_mode.py` | `apps/internal/skills/get_arm_control_mode.py` | `apps/internal/nodes/get_arm_control_mode.py` |
| 设置控制模式 | `apps/internal/adapters/set_arm_control_mode.py` | `apps/internal/skills/set_arm_control_mode.py` | `apps/internal/nodes/set_arm_control_mode.py` |
| 设置末端位姿 | `apps/internal/adapters/set_arm_pose.py` | `apps/internal/skills/set_arm_pose.py` | `apps/internal/nodes/set_arm_pose.py` |
| 读取末端位姿 | `apps/internal/adapters/get_arm_pose.py` | `apps/internal/skills/get_arm_pose.py` | `apps/internal/nodes/get_arm_pose.py` |
| 计算双臂IK | `apps/internal/adapters/get_arm_ik_result.py` | `apps/internal/skills/get_arm_ik_result.py` | `apps/internal/nodes/get_arm_ik_result.py` |
| 计算双臂FK | `apps/internal/adapters/get_arm_fk_result.py` | `apps/internal/skills/get_arm_fk_result.py` | `apps/internal/nodes/get_arm_fk_result.py` |

表格中的 Adapter / Skill / Node 均为人工测试入口，不是框架实现文件。共用文件 `apps/internal/_arm_test.py` 只保留硬件配置和参数/结果校验，不管理生命周期、不发送运动指令。关节和位姿校验分别使用 `validate_joint_target`、`validate_pose_target` 以及 `check_joint_result`、`check_pose_result`，避免同名函数含义不清。

## 人工参数

- `set_arm_joint`：填写 `TARGET_JOINT`，14个关节角（deg），顺序为左臂7个、右臂7个。
- `set_arm_joint_trajectory`：填写 `TRAJECTORY_TIMES`、`TRAJECTORY_FRAMES`。时间为相对轨迹起点的秒数，大于0且严格递增；每个时间点对应一帧14维关节角（deg），不是分段时长。
- `get_arm_joint`：无业务输入，返回14维关节反馈角（deg）。
- `get_arm_control_mode`：无业务输入，返回mode与name，0/ArmFixed、1/AutoSwing、2/ExternalControl；Node顶部OUTPUT_KEY默认arm_control_mode。SDK可能返回缓存，不能当作控制器在线确认。getter示例只查询不切换。
- `set_arm_control_mode`：填写顶部 `CONTROL_MODE`，默认None，必须为整数0固定、1AutoSwing、2外部控制。统一通过ArmMixin → LowLevelSDKManager → 原生SDK，不直接调用ROS服务，必须初始化low Manager。上层签名保留；SDK可能跳过已处于目标模式的切换，模式1不承诺重复触发重置且可能运动。不自动切回、不额外复位或切MPC，不以设置成功证明到位。
- `set_arm_pose`：填写 `LEFT_POSE`、`RIGHT_POSE`，均为 `[x,y,z,yaw,pitch,roll]`，位置m、ZYX欧拉角deg。必须同时提供双臂，不补默认值、不支持单臂。
- 末端接口的 `FRAME` 默认 `"local"`，即base_link；`"world"` 对应odom，不是map。也接受base_link/odom别名。
- `get_arm_pose`：选择 `SOURCE="tf"` 或 `"fk"`，默认tf，失败不自动换来源。输出包含left_pose、right_pose（m/deg）及frame/source。
- 选择fk时必须人工确认并填写 `FK_FRAME`，表示FK模型输出基准对应的TF frame；不同参考系需要可用TF变换。FK工具偏移也需确认，不能假定与TF末端点相同。
- `get_arm_ik_result`：填写LEFT_POSE/RIGHT_POSE（6维m/deg）、FRAME及IK_FRAME，可选JOINT作14维deg初值。不运动，输出14维deg解；不是当前关节反馈。
- `get_arm_fk_result`：填写JOINT（14维deg）、FRAME和FK_FRAME。不读实时关节、不运动，输出双臂m/deg位姿。模型frame必须与整机TF基准对应；当前v62上半身模型的内部base_link对应整机waist_yaw_link，另需确认工具偏移。
- 新IK直调SDK，不带旧solve_ik的补偿、精度和连续性策略；旧码垛业务暂不迁移。IK/FK依赖服务和模型，Mock通过不证明求解物理准确。
- getter Node 的 `OUTPUT_KEY` 默认arm_joint/arm_pose/arm_ik_result/arm_fk_result，可在顶部修改；测试从指定黑板键读取输出。

目标参数为空时失败退出，不初始化硬件。操作人员已经填写的目标不会因辅助文件合并而改变，运行前仍须逐个检查。脚本不接收命令行参数，不提供推荐的真机运动数值。

## 人工运行

仅由操作人员配置好ROS/SDK、确认现场安全和实际关节范围后，从仓库根目录按需单独运行。例如以下为Adapter层入口：

```bash
python3 apps/internal/adapters/set_arm_joint.py
python3 apps/internal/adapters/set_arm_joint_trajectory.py
python3 apps/internal/adapters/get_arm_joint.py
python3 apps/internal/adapters/get_arm_control_mode.py
python3 apps/internal/adapters/set_arm_control_mode.py
python3 apps/internal/adapters/set_arm_pose.py
python3 apps/internal/adapters/get_arm_pose.py
python3 apps/internal/adapters/get_arm_ik_result.py
python3 apps/internal/adapters/get_arm_fk_result.py
```

Skill、Node同样从仓库根目录运行，以 `python3` 加上表格对应的完整相对路径，例如 `python3 apps/internal/skills/get_arm_joint.py` 或 `python3 apps/internal/nodes/get_arm_joint.py`。只选需要的脚本，不批量发指令。Node人工验证前取消 `STUDIO_DRY_RUN`。成功退出码0，参数、初始化或调用失败为1。

## 行为边界

调用链为 Node → Skill → IHardware → ArmMixin → LowLevelSDKManager → SDK。只启用low Manager，跳过相机、末端执行器、StateManager和力控发布器。除显式set_arm_control_mode测试外，示例不额外切换模式，不复位手臂或躯干、不扫描、不等待到位；SDK内部仍可能切换ExternalControl，原初始化依赖保持不变。

关节单目标可能被SDK限幅，原生轨迹超限可能被拒绝；轨迹保留全部时间点，不走旧自动MPC重采样路径。轨迹发送后脚本释放自身硬件引用，不验证整个轨迹物理完成或进程退出后控制器的执行行为。当前轮式仿真原生轨迹不动的问题仍待核实，不能以发送成功证明轨迹可执行。

读取TF或SDK缓存不保证首帧、新鲜度或物理到位。FK是根据关节反馈计算的位姿，不是第二套独立传感器。所有示例只调用一次，没有自动重试或来源回退。

离线验证见 `tests/test_arm_joint_app_scripts.py`、`tests/test_arm_pose_app_scripts.py`，仅使用Mock，不下发真实指令。
