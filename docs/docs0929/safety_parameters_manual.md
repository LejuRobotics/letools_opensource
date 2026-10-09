# LeTools 安全参数手册（V62 实机核查稿）

[返回 README](../README.md) · [仿真开发指南](仿真开发指南.md) · [FAQ 自检手册](FAQ_常见问题.md)

> 本稿同时整理开源基线和 2026-09-19 现场 V62 实机的只读核查结果。配置、URDF及运行接口中的原始值不自动等于经过安全认证的推荐值、警告值或禁止值。凡标记“待确认/待实测”的内容，禁止用于正式安全验收。本文不记录登录密码。

## 1. 适用基线与结论

| 项目 | 当前核查结果 |
|---|---|
| LeTools本地基线 | 分支 `dev`，commit `ca1baf54cc314df1bce60e6ac773fb6dcbd60587`；工作区有未提交修改 |
| SDK版本 | 以 `scripts/kuavo_humanoid_sdk_tools/sdk_version.env` 为准 |
| 开源对照基线 | `kuavo-ros-opensource` Tag `1.4.5`，commit `d824a7b877b41d0b9e6a9ffe947e050d89ea9681` |
| 现场上位机仓库 | `/media/data/kuavo_ros_application`，分支 `yzh/merge_master`，commit `d29bea338d7241c9286d07af8abf1e9b254d7028`，dirty |
| 现场LeTools仓库 | `/media/data/LeTools`，分支 `feature/hs_carton`，commit `331b677a0c34beae67e4d80b5d5d251b89fa75b5`，dirty |
| 现场下位机控制仓库 | `/home/lab/zwl/kuavo-ros-control`，分支 `chaiduo/0427`，commit `55bd1742a97dd4740ff8ac6a1b6379bea227019e`，dirty |
| 当前核查机型 | ROS参数 `/robot_version=62`；systemd环境 `ROBOT_VERSION=62`；配置 `ROBOT_MODULE=LUNBI_V62` |
| 当前末端配置 | 左右均为 `qibeng`（气泵），来自现场 `kuavo_v62/kuavo.json` |
| 核查时间 | 2026-09-19（现场运行快照） |
| 机器人实际型号、序列号、硬件版本 | 待现场填写 |
| 文档负责人/实机标定负责人 | 待指定 |

本次代码核查结论：

- 下位机活动服务 `ocs2_h12pro_monitor.service` 明确设置 `ROBOT_VERSION=62`，且ROS参数再次确认版本62；
- 现场 V62 配置中存在位置、速度、峰值速度、堵转力矩、峰值力矩和保护时间窗口；
- 现场 V62 配置的控制/规划模型实际引用 `biped_s49/urdf/drake/*`，同时仓库另有 `biped_s62.urdf`，两者不可混用；
- 上位机当前 `load_robot_head_wheel.launch` 默认选择 `biped_s42_head.urdf`，该文件用于当前RViz/头部应用，不能作为V62控制限位依据；
- 仓库没有“折叠状态/伸展状态”两套独立安全阈值；
- 仓库没有推荐/警告/禁止三级数值；
- 仓库没有经过验收的末端最大线速度和急停制动距离；
- 运行配置、S62整机URDF和V62所引用的S49 Drake URDF数值不同，正式交付前必须由控制负责人裁决安全边界。

## 2. 参数来源和优先级

现场下位机当前控制仓库根目录为：

```bash
export KUAVO_ROOT=/home/lab/zwl/kuavo-ros-control
export ROBOT_VERSION=62
```

| 来源 | 仓库内路径 | 用途 | 能否直接作为正式安全值 |
|---|---|---|---|
| 现场V62运行配置 | `src/kuavo_assets/config/kuavo_v62/kuavo.json` | 当前ROS参数 `/kuavo_configuration` 的来源；硬件保护和控制参数 | 不能，需确认单位、生效开关并实测 |
| V62配置引用的控制模型 | `src/kuavo_assets/models/biped_s49/urdf/drake/biped_v3_all_joint.urdf`、`biped_v3_arm.urdf` | 当前V62配置声明的规划/控制模型 | 不能等同于硬件保护阈值 |
| 仓库中的S62整机URDF | `src/kuavo_assets/models/biped_s62/urdf/biped_s62.urdf` | V62机械模型候选/对照 | 当前配置未直接引用，不能据此认定生效 |
| 现场控制服务 | `/etc/systemd/system/ocs2_h12pro_monitor.service` | 工作空间、ROS master、控制方案和机器人版本 | 可确认当前运行环境，不能提供安全阈值 |
| 上位机显示模型 | `/media/data/kuavo_ros_application/src/ros_robotModel/biped_s42/urdf/biped_s42_head.urdf` | 当前RViz/头部应用显示 | 不是V62控制限位来源 |
| 开源基线真机启动文件 | `src/humanoid-control/humanoid_controllers/launch/load_kuavo_real.launch` | 1.4.5基线中的保护开关 | 只能用于对照，不能证明现场服务采用相同参数 |
| 关节保护说明 | `docs/7常见问题与故障排查/关节保护简介.md` | 字段语义、日志和已知风险 | 作为机制说明，不作为认证报告 |

安全值必须同时具备机器人型号、硬件版本、关节名称、单位、字段路径、数据来源、测试工况、测试日期和审批记录。

## 3. 三级阈值定义与当前映射

| 等级 | 交付定义 | 期望软件行为 | 当前仓库是否已有正式数值 |
|---|---|---|---|
| 推荐值 | 日常开发、演示和连续运行范围 | 正常运行并记录 | 否 |
| 警告值 | 接近已验证边界 | 告警、降速或要求确认 | 否 |
| 禁止值 | 不允许下发或继续运行 | 拒绝、停止并进入安全处置 | 否 |

仓库字段不能直接按名称套成三级：

| 仓库字段 | 已知作用 | 与三级标注的关系 |
|---|---|---|
| `min/max_joint_position_limits` | 命令位置截断 | 只能作为候选禁止边界，V62手臂当前值过宽，需复核 |
| `joint_velocity_limits` | 命令截断/速度保护 | 候选警告或禁止边界，单位存在冲突，需确认 |
| `joint_peak_velocity_limits` | 峰值速度字段 | 候选禁止边界，实际使用链路需确认 |
| `joint_torque_limits` | 堵转保护阈值 | 长时间阈值，不等于推荐最大力矩 |
| `joint_peak_torque_limits` | 峰值扭矩保护/截断 | 短时间极限，不等于允许持续使用值 |
| `joint_limit_torque_ratio` | 部分版本用于关节限位附近的力矩倍率 | 现场V62配置中不存在该字段；不得沿用其他版本的 `0.3` |

正式三级数值必须由硬件、安全和控制负责人共同批准。

## 4. V62 双臂关节限位

### 4.1 现场V62配置实际引用的Drake手臂边界

现场 `kuavo_v62/kuavo.json` 的 `arm_urdf` 实际指向 `biped_s49/urdf/drake/biped_v3_arm.urdf`，而不是 `biped_s62/urdf/drake/biped_v3_arm.urdf`。以下为该实际引用文件的原始值。弧度是文件原值，角度仅为便于阅读的近似换算。

| 关节 | 下限 rad（约deg） | 上限 rad（约deg） | URDF effort | URDF velocity | 状态 |
|---|---:|---:|---:|---:|---|
| 左臂 J1 `zarm_l1_joint` | -3.142（-180°） | 1.571（90°） | 66.67 | 18.8 | 规划模型原值，待确认安全等级 |
| 左臂 J2 `zarm_l2_joint` | -0.349（-20°） | 2.094（120°） | 75.0 | 8.0 | 同上 |
| 左臂 J3 `zarm_l3_joint` | -1.571（-90°） | 1.571（90°） | 57.0 | 7.5 | 同上 |
| 左臂 J4 `zarm_l4_joint` | -2.618（-150.0°） | 0.000（0.0°） | 75.0 | 8.0 | 同上 |
| 左臂 J5 `zarm_l5_joint` | -1.571（-90.0°） | 1.571（90.0°） | 14.1 | 17.5 | 同上 |
| 左臂 J6 `zarm_l6_joint` | -1.309（-75.0°） | 0.698（40.0°） | 14.1 | 17.5 | 同上 |
| 左臂 J7 `zarm_l7_joint` | -0.698（-40.0°） | 0.698（40.0°） | 14.1 | 17.5 | 同上 |
| 右臂 J1 `zarm_r1_joint` | -3.142（-180°） | 1.571（90°） | 66.67 | 18.8 | 同上 |
| 右臂 J2 `zarm_r2_joint` | -2.094（-120°） | 0.349（20°） | 75.0 | 8.0 | 同上 |
| 右臂 J3 `zarm_r3_joint` | -1.571（-90°） | 1.571（90°） | 57.0 | 7.5 | 同上 |
| 右臂 J4 `zarm_r4_joint` | -2.618（-150.0°） | 0.000（0.0°） | 75.0 | 8.0 | 同上 |
| 右臂 J5 `zarm_r5_joint` | -1.571（-90.0°） | 1.571（90.0°） | 14.1 | 17.5 | 同上 |
| 右臂 J6 `zarm_r6_joint` | -0.698（-40.0°） | 1.309（75.0°） | 14.1 | 17.5 | 同上 |
| 右臂 J7 `zarm_r7_joint` | -0.698（-40.0°） | 0.698（40.0°） | 14.1 | 17.5 | 同上 |

### 4.2 S62整机URDF与实际引用模型的差异

同一现场仓库中的 `biped_s62.urdf` 与V62配置实际引用的S49 Drake手臂URDF并非完全一致。例如 J1：

| 来源 | lower | upper | effort | velocity |
|---|---:|---:|---:|---:|
| `biped_s62.urdf` 左/右J1 | -3.14159 | 1.57080 | 66.0 | 18.8 |
| V62配置实际引用的 `biped_s49/.../biped_v3_arm.urdf` 左/右J1 | -3.14159 | 1.57080 | 66.67 | 18.8 |

两者J1角度一致，但力矩字段已有差异；其他文件还存在更明显的速度、力矩差异。因此，表4.1只能作为当前配置引用的规划模型基线，不能直接宣称为真机禁止值。发布前必须由控制负责人说明S49模型复用于V62的设计意图，并裁决各字段的优先级。

### 4.3 V62运行配置中的双臂保护原始值

V62共有20个配置项：前4项为轮式躯干机构相关关节，索引4–17为左右臂J1–J7，最后2项为头部。下表按现场数组索引4–17提取双臂值。此前将最后两个头部项 `200/1800` 误映射为手臂速度的做法已经纠正。

现场配置中，14个手臂项的 `min_joint_position_limits/max_joint_position_limits` 均为 `-180/+180`。该范围明显比实际引用URDF更宽，只能记录为命令配置原值，不能作为允许手臂转动到±180°的依据。

| 关节 | `joint_velocity_limits` | `joint_peak_velocity_limits` | `joint_torque_limits` | `joint_peak_torque_limits` |
|---|---:|---:|---:|---:|
| 左J1 | 12.0 | 18.8 | 46.67 | 66.67 |
| 左J2 | 6.0 | 8.0 | 52.5 | 75.0 |
| 左J3 | 6.5 | 7.5 | 39.9 | 57.0 |
| 左J4 | 6.0 | 8.0 | 52.5 | 75.0 |
| 左J5 | 10.0 | 7.0 | 9.87 | 14.1 |
| 左J6 | 10.0 | 7.0 | 9.87 | 14.1 |
| 左J7 | 10.0 | 7.0 | 9.87 | 14.1 |
| 右J1 | 12.0 | 18.8 | 46.67 | 66.67 |
| 右J2 | 6.0 | 8.0 | 52.5 | 75.0 |
| 右J3 | 6.5 | 7.5 | 39.9 | 57.0 |
| 右J4 | 6.0 | 8.0 | 52.5 | 75.0 |
| 右J5 | 10.0 | 7.0 | 9.87 | 14.1 |
| 右J6 | 10.0 | 7.0 | 9.87 | 14.1 |
| 右J7 | 10.0 | 7.0 | 9.87 | 14.1 |

风险说明：现场配置没有在字段旁声明单位，URDF按规范通常使用rad/s，但运行数组可能处于电机空间或关节空间。部分手臂常规速度字段（J5–J7为10）还高于峰值字段（7），语义不能仅凭字段名推断。正式手册必须由控制负责人确认单位、坐标空间和保护实现，确认前不得写入推荐/警告/禁止栏。

非手臂项保留原始索引，避免在没有控制器映射证据时猜测物理关节名称：

| 数组索引 | 当前分类 | 位置下限/上限 | 常规/峰值速度 | 常规/峰值力矩 | 映射状态 |
|---:|---|---:|---:|---:|---|
| 0 | 轮式躯干机构 | -5.5 / 83 | 6.0 / 22.5 | 467.6 / 668 | 物理关节名待控制器映射确认 |
| 1 | 轮式躯干机构 | -161 / 5.5 | 6.0 / 22.5 | 467.6 / 668 | 同上 |
| 2 | 轮式躯干机构 | -25 / 181 | 6.0 / 22.5 | 186.9 / 267 | 同上 |
| 3 | 轮式躯干机构 | -190 / 190 | 6.0 / 22.5 | 186.9 / 267 | 同上 |
| 18 | 头部项A | -30 / 30 | 200 / 1800 | 140 / 200 | 与具体头部关节的次序待确认 |
| 19 | 头部项B | -25 / 25 | 200 / 1800 | 140 / 200 | 同上 |

### 4.4 折叠与伸展状态待补表

现场配置已确认以下姿态字段，但没有把它们定义为“折叠态/伸展态”安全判定：

| 字段 | 现场V62原值（deg，左右臂各J1–J7） | 可确认含义 |
|---|---|---|
| `init_arm_pos` | 左 `[20,0,0,-30,0,0,0]`；右同左 | 初始化手臂姿态 |
| `walk_arm_pose` | 左 `[20,0,0,-35,0,0,0]`；右同左 | 行走摆臂基准姿态 |
| `calibration_safe_pose` | 左 `[-60,0,30,-30,0,0,0]`；右 `[-60,0,-30,-30,0,0,0]`，另有2个头部项 | 标定前安全姿态候选，不等于运行安全阈值 |
| `arm_calibration_velocity` | `15.0` | 标定速度字段；单位和安全等级待控制负责人确认 |

不得仅通过姿态名称把上述数组认定为折叠态或伸展态。两状态仍需由碰撞包络、末端工具、躯干高度及互锁条件共同定义。

| 状态 | 判定条件 | 角度推荐/警告/禁止 | 速度推荐/警告/禁止 | 力矩推荐/警告/禁止 |
|---|---|---|---|---|
| 折叠臂 | 待明确由哪3个折叠机构关节及区间判定 | 待碰撞包络和实机标定 | 待实测 | 待实测 |
| 伸展臂 | 待明确状态切换和互锁条件 | 待碰撞包络和实机标定 | 待实测 | 待实测 |

## 5. 保护机制、参数路径与修改方法

### 5.1 保护字段

V62配置文件：

```text
$KUAVO_ROOT/src/kuavo_assets/config/kuavo_v62/kuavo.json
```

| 字段 | V62当前值/形式 | 说明 |
|---|---|---|
| `peak_protection` | `0.1` s | 峰值扭矩连续超限时间窗口 |
| `locked_rotor_protection` | `2` s | 堵转扭矩连续超限时间窗口 |
| `speed_protection` | `0.05` s | 速度连续超限时间窗口 |
| `joint_limit_torque_ratio` | 现场配置中不存在 | 其他版本可能存在；禁止把开源基线或其他机型的值复制到V62 |
| `min/max_joint_position_limits` | 20项数组 | 命令位置截断边界 |
| `joint_velocity_limits` | 20项数组 | 常规速度截断/保护字段 |
| `joint_peak_velocity_limits` | 20项数组 | 峰值速度字段 |
| `joint_torque_limits` | 20项数组 | 堵转保护阈值 |
| `joint_peak_torque_limits` | 20项数组 | 峰值扭矩保护阈值 |
| `VelocityLimit` | `[1.2, 0.6, 50]` | 底盘/躯干命令速度候选上限；三个分量和单位待确认 |
| `cmd_vel_step` | `[0.1, 0.05, 5]` | 命令步进候选值；三个分量和单位待确认 |
| `kuavo_wheel_torso_limit` | `z_range=[0.74,0.84]`、`x_range=[0.1,0.2]`、`pitch_range=[0,20]` | 轮式躯干工作区原值；坐标系、单位和安全等级待确认 |

### 5.2 现场生效链路与保护开关

现场活动服务为：

```text
/etc/systemd/system/ocs2_h12pro_monitor.service
```

2026-09-19只读检查确认其状态为 `active/running`，关键环境如下：

```text
KUAVO_CONTROL_SCHEME=ocs2
KUAVO_ROS_CONTROL_WS_PATH=/home/lab/zwl/kuavo-ros-control
ROBOT_VERSION=62
ROS_MASTER_URI=http://kuavo_master:11311
```

该服务未显式传入 `joint_protect_enable` 或 `cmd_truncation_enable`。开源1.4.5对照基线的 `load_kuavo_real.launch` 中曾见以下默认值：

```xml
<arg name="joint_protect_enable" default="false" />
<arg name="cmd_truncation_enable" default="false" />
```

这只能证明开源对照文件的默认值，不能证明现场OCS2服务当前保护状态。不得在未核对阈值、单位和急停措施前直接在客户真机启用，也不得为“验证”而重启现场服务。后续应由控制负责人在维护窗口通过节点参数、控制器启动日志和源码调用链确认。

开源基线的候选启动形式如下，仅用于受控环境说明，不是现场直接操作命令：

```bash
roslaunch humanoid_controllers load_kuavo_real.launch \
  joint_protect_enable:=true \
  cmd_truncation_enable:=true
```

实际启动命令如果由其他launch文件转发，必须用 `rosparam get` 和启动日志确认参数最终值。

### 5.3 安全修改流程

1. 确认 `ROBOT_VERSION=62`、机器人序列号、硬件版本和下位机commit；
2. 备份 `kuavo_v62/kuavo.json` 并记录校验值；
3. 标注所改数组索引对应的物理关节，禁止凭数组位置猜测；
4. 确认单位、持续/峰值含义和保护开关；
5. 先做JSON解析、离线检查和MuJoCo验证；
6. 经安全负责人批准后，在隔离区、低速、低负载条件下实机验证；
7. 每次只改一类参数，保存原值、修改值、日志和结果；
8. 未通过验收立即回滚，禁止通过扩大阈值消除告警。

JSON格式检查：

```bash
python3 -m json.tool \
  "$KUAVO_ROOT/src/kuavo_assets/config/kuavo_v62/kuavo.json" >/dev/null
```

## 6. 末端最大线速度

仓库没有找到可作为正式安全上限的左右末端线速度配置。URDF的关节 `velocity` 是关节速度，不是末端笛卡尔线速度，不能直接转换成单一安全值。

| 状态 | 末端类型 | 负载 | 推荐值 | 警告值 | 禁止值 | 单位 |
|---|---|---:|---:|---:|---:|---|
| 折叠臂 | 左右均为气泵 `qibeng` | 待称重 | 待实测 | 待实测 | 待实测 | m/s |
| 伸展臂 | 左右均为气泵 `qibeng` | 待称重 | 待实测 | 待实测 | 待实测 | m/s |

测试必须记录机器人状态、末端工具、负载、轨迹方向、控制周期、采样频率、最大值、重复次数和停止方式。

## 7. 急停与制动距离

### 7.1 已确认的停止入口

| 入口 | 类型 | 当前代码行为 | 安全性质 |
|---|---|---|---|
| 物理急停 | 硬件开关 | 产品文档说明切断/控制驱动器电源 | 最终行为需对应硬件版本确认 |
| `/stop_robot` | ROS Topic，`std_msgs/Bool` | 控制器和遥控节点使用 | 软件停止，不等于物理急停 |
| `/websocket_sdk_srv/stop_robot` | ROS Service，`std_srvs/Trigger` | 现场运行时存在 | WebSocket SDK软件停止，不等于物理急停 |
| `/bezier/stop_plan_arm_trajectory` | ROS Service，`std_srvs/Trigger` | 现场运行时存在 | 停止手臂轨迹规划，不等于整机急停 |
| `/cmd_vel` | ROS Topic，`geometry_msgs/Twist` | 现场运行时存在；相关脚本可持续发布零速度 | 软件零速覆盖，不等于断电 |
| `/cmd_vel_world` | ROS Topic，候选为`geometry_msgs/Twist` | 开源源码中存在，本次现场快照未观察到 | 使用前必须复核，不能作为当前急停入口 |
| 遥控器BACK或组合键 | 人机输入 | 部分节点停止底盘或杀死程序 | 需确认所用遥控器和启动方式 |

### 7.2 已知风险

仓库《关节保护简介》明确指出：当前部分电机的保护动作是失能/零力矩，高速运动时存在风险；文档同时建议增加整机急停并考虑渐进式制动。因此，软件失能、零速度覆盖和物理急停必须分别测试，不能共用一个“制动距离”。

### 7.3 制动距离实测表

| 停止类型 | 臂状态 | 初始底盘速度 | 末端负载 | 地面 | 控制周期 | 制动距离 | 重复次数 | 最大值 |
|---|---|---:|---:|---|---:|---:|---:|---:|
| 物理急停 | 折叠 | 待定 | 待定 | 待定 | 待定 | 待实测 | ≥10（待审批） | 待实测 |
| 物理急停 | 伸展 | 待定 | 待定 | 待定 | 待定 | 待实测 | ≥10（待审批） | 待实测 |
| `/stop_robot` | 折叠/伸展 | 待定 | 待定 | 待定 | 待定 | 待实测 | ≥10（待审批） | 待实测 |
| 零速度覆盖 | 折叠/伸展 | 待定 | 待定 | 待定 | 100Hz发布 | 待实测 | ≥10（待审批） | 待实测 |

重复次数和工况必须由安全测试方案批准，上表中的“≥10”仅为草案建议，不是认证标准。

## 8. 实时数据订阅/发布接口

### 8.1 现场已确认接口

下表已于2026-09-19在下位机ROS master中通过 `rostopic type` 或 `rosservice type` 实际确认。这里只证明当时接口存在及类型正确，不证明接口频率、数据单位或停止效果已经通过安全验收。

| 接口 | 消息/服务类型 | 方向（相对控制器） | 用途 |
|---|---|---|---|
| `/stop_robot` | `std_msgs/Bool` | 发布/消费 | 软件停止信号 |
| `/cmd_vel` | `geometry_msgs/Twist` | 发布/消费 | 底盘局部坐标速度 |
| `/move_base/base_cmd_vel` | `geometry_msgs/Twist` | 发布 | 底盘调度速度 |
| `/move_base/robot_status` | `leju_mobile_base_msgs/RobotStatus` | 订阅 | 底盘状态 |
| `/move_base/status_code` | `std_msgs/UInt64` | 订阅 | 底盘状态码 |
| `/hardware_status` | `leju_mobile_base_msgs/HardwareStatus` | 订阅 | 底盘硬件状态 |
| `/omni_bot/joint_states` | `sensor_msgs/JointState` | 订阅 | 轮式底盘关节状态 |
| `/kuavo_arm_traj` | `sensor_msgs/JointState` | 发布/消费 | 手臂轨迹接口 |
| `/bezier/arm_traj` | `trajectory_msgs/JointTrajectory` | 发布/消费 | 贝塞尔手臂轨迹 |
| `/bezier/arm_traj_state` | `kuavo_msgs/planArmState` | 订阅 | 手臂规划状态 |
| `/robot_action_state` | `h12pro_controller_node/RobotActionState` | 订阅 | 遥控/动作状态 |
| `/websocket_sdk_srv/stop_robot` | `std_srvs/Trigger` | Service | WebSocket SDK软件停止 |
| `/bezier/stop_plan_arm_trajectory` | `std_srvs/Trigger` | Service | 停止贝塞尔手臂轨迹 |
| `/bezier/plan_arm_trajectory` | `kuavo_msgs/planArmTrajectoryBezierCurve` | Service | 规划贝塞尔手臂轨迹 |

现场检查时，`/enable_wbc_arm_trajectory_control` 和 `/enable_mm_wbc_arm_trajectory_control` 均不存在。客户端不得假设所有开源文档中的服务在当前启动模式下可用。

### 8.2 开源基线候选接口

以下接口可从1.4.5源码中找到，但本次现场快照未逐项确认，使用前必须再次查询：

| 接口 | 候选类型 | 用途 |
|---|---|---|
| `/joint_cmd` | `kuavo_msgs/jointCmd` | 关节位置、速度、力矩等命令 |
| `/share_memory/sensor_data_raw` | `kuavo_msgs/sensorsData` | 原始关节/传感器状态 |
| `/joint_states` | `sensor_msgs/JointState` | 标准关节状态 |
| `/hand_wrench/left_hand`、`/hand_wrench/right_hand` | `geometry_msgs/WrenchStamped` | 左右末端力/力矩 |
| `/monitor/frequency/wbc`、`/monitor/time_cost/wbc` | `std_msgs/Float64` | WBC频率和耗时 |
| `/humanoid_controller/get_controller_list` | 以现场查询为准 | 查询控制器 |

运行时核验：

```bash
rosnode list
rostopic list
rosservice list
rostopic type /stop_robot
rostopic type /kuavo_arm_traj
rostopic hz /omni_bot/joint_states
rostopic echo -n 1 /hardware_status
rosservice type /websocket_sdk_srv/stop_robot
rosservice info /bezier/stop_plan_arm_trajectory
```

订阅原始状态前，应使用 `rosmsg show` 确认字段定义。不同消息层中的 `joint_q/joint_v/joint_current/joint_torque` 可能处于电机空间或关节空间，使用前必须确认坐标和单位。

## 9. 常见连接异常和超时排查

### 9.0 本次实机网络边界

| 层级 | 地址/接口 | 已确认职责 |
|---|---|---|
| 上位机 | 无线地址 `192.168.0.153`；内部有线地址 `192.168.26.12` | 运行LeTools、相机、视觉推理、RViz及应用层ROS节点 |
| 下位机 | 内部有线地址 `192.168.26.1` | 运行ROS master、OCS2/H12Pro、手臂轨迹和底层控制服务 |
| ROS master | `http://kuavo_master:11311` | 由下位机服务环境配置提供 |

上位机到下位机的SSH和ROS通信依赖 `192.168.26.0/24` 内部网络。排查时先区分“开发电脑到上位机”“上位机到下位机”“ROS节点到ROS master”三段链路，不要只用一次 `ping` 判断整条链路正常。

### 9.1 ROS master不可用

```bash
echo "$ROS_MASTER_URI"
echo "$ROS_IP"
echo "$ROS_HOSTNAME"
rosnode list
```

容器和宿主机联合运行时应访问同一个ROS master，Docker通常使用host网络。不要同时设置互相冲突的 `ROS_IP` 和 `ROS_HOSTNAME`。

现场可从上位机执行以下只读检查：

```bash
ip route get 192.168.26.1
ping -c 3 192.168.26.1
ssh lab@192.168.26.1 'systemctl is-active ocs2_h12pro_monitor.service'
```

密码不得写入脚本、仓库、命令历史或本手册。

### 9.2 Topic存在但无数据

```bash
rostopic info /share_memory/sensor_data_raw
rostopic hz /share_memory/sensor_data_raw
rosnode info <发布节点>
```

- 无publisher：上游节点未启动或已经退出；
- 有publisher但频率为0：检查硬件、共享内存和控制器首个错误；
- 类型不一致：重新加载同一版本消息工作空间，排除旧 `devel`；
- 频率异常：同时查看 `/monitor/frequency/wbc` 和控制器CPU占用。

### 9.3 Service不存在或调用超时

```bash
rosservice list | grep humanoid_controller
rosservice info /humanoid_controller/get_controller_list
rosnode info <服务节点>
```

如果服务端节点不存在，增加客户端timeout无效。先处理roslaunch中的第一个 `ERROR/FATAL` 或退出的 `REQUIRED process`。

### 9.4 日志位置和保护日志关键字

| 日志 | 位置/命令 |
|---|---|
| ROS/roslaunch | `~/.ros/log/latest/` |
| 控制器终端 | 保存完整启动输出 |
| systemd | `journalctl -u <服务名> -n 300 --no-pager` |
| Docker | `docker logs --tail 300 <容器名>` |
| catkin | 工作空间 `logs/<包名>/` |

关节保护常见关键字：

```text
Trigger peak protection
set motor <编号> disable
motor status map
[orig] tau <编号> value <原值> -> [max] <截断值>
```

出现保护触发时，保存触发前后至少10秒的控制器日志、关节状态、命令值和机器人视频。不得只清除日志后重试。

## 10. 发布前必须补齐的验证

- [x] 运行版本已由ROS参数和systemd环境双重确认为V62；
- [x] 当前控制仓库、分支、commit及dirty状态已记录；
- [x] 当前末端配置已确认左右均为气泵 `qibeng`；
- [x] V62配置实际引用S49 Drake模型的事实已记录；
- [x] 2026-09-19运行时Topic/Service名称和类型已抽样复核；
- [ ] 机器人产品型号、序列号和硬件修订号已从铭牌/资产系统填写；
- [ ] 折叠/伸展状态判定条件及互锁逻辑已定义；
- [ ] S62整机URDF、S49 Drake模型和 `kuavo_v62/kuavo.json` 的差异已由负责人裁决；
- [ ] 所有角度、速度和力矩字段的单位、空间及索引已确认；
- [ ] 推荐/警告/禁止三级数值已通过评审和实机验证；
- [ ] 保护开关的默认值和客户部署值已记录；
- [ ] 末端最大线速度已按状态、方向、工具和负载实测；
- [ ] 物理急停、软件停止和零速覆盖的制动距离已分别实测；
- [ ] 全量实时Topic/Service已在最终交付启动模式下复核并归档类型、频率和样例；
- [ ] 超时、节点退出和通信中断测试已经通过；
- [ ] 回滚流程和原始配置备份已验证；
- [ ] 文档负责人、控制负责人、硬件负责人和安全负责人已签字。

在以上阻断项完成前，本文件只能作为“代码核查稿”和标定模板，不能宣称完成正式安全参数交付。

## 11. 客户回复覆盖状态

| 客户要求 | 本次已补齐 | 仍未完成及原因 |
|---|---|---|
| 折叠臂/伸展臂各关节角度、最大角速度、最大力矩、末端最大线速度、急停距离 | 已记录现场V62运行配置、实际引用URDF、14个手臂关节原始边界、姿态字段、末端类型和停止入口 | 折叠/伸展判定、末端线速度和制动距离必须在批准工况下实测 |
| 推荐值/警告值/禁止值三级标注 | 已定义三级含义并标出可作为候选边界的仓库字段 | 仓库原值未经安全评审，不能自动转换为三级正式阈值 |
| 参数名称、配置路径、字段和修改方法 | 已补齐现场下位机控制仓库、V62配置、实际S49 Drake模型、S62整机URDF、systemd服务和安全修改流程 | 数组索引0–3、18–19对应的物理关节名称仍需控制器负责人确认 |
| 实时订阅/发布接口及连接超时排查 | 已实机确认主要Topic/Service类型，并补充上位机—下位机—ROS master三段链路 | 最终交付启动模式下仍需做全量接口、频率、超时和停止效果验收 |

本次已将能够通过实机只读证据确认的待定项改为确定值；涉及人身安全、硬件修订、单位语义、碰撞包络或动态制动效果的项目继续保留为阻断项。
