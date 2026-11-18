# instructional_case_03：头部、躯干与手臂串行动作

本案例按 `home → left → right → home` 执行四个阶段。每阶段都是带记忆的
`Sequence`，依次执行现有的 `HeadControlSdkMove`、`MultiTimedCommandMove` 和
`ArmJointTimedMove`。躯干命令与手臂命令分开提交，不同时调用 TimedCmd。

所有命令均需在仓库根目录执行。先 dry-run 检查配置，再由测试人员确认真机条件。

## 树结构与返回边界

```text
Sequence(four_head_arm_torso_phases, memory=true)
├── Sequence(phase_1_home)：头回正 → 躯干 home → 双臂 home
├── Sequence(phase_2_turn_left)：头左转 → 躯干 left → 双臂 left
├── Sequence(phase_3_turn_right)：头右转 → 躯干 right → 双臂 right
└── Sequence(phase_4_return_home)：头回正 → 躯干 home → 双臂 home
```

共 **17 个节点**：1 个根 `Sequence`、4 个阶段 `Sequence`、12 个叶子节点。
每阶段的叶子顺序如下：

| 节点 | 当前配置与行为 |
|---|---|
| `HeadControlSdkMove` | 下发头部 yaw/pitch 目标；不查询头部是否到位 |
| `MultiTimedCommandMove` | 一次 `send_timed_multi_commands()` 请求仅含 **planner 2**：躯干 4D 位姿；请求成功后等待 `actual_time + settle_time` |
| `ArmJointTimedMove` | 调用 `send_arm_joint_timed()` 下发 **14D 双臂关节目标**；请求成功后等待 `actual_time + settle_time` |

这里的“串行”是节点执行与 TimedCmd 请求的串行：躯干节点成功后才运行手臂节点。
头部没有到位等待，实际头部运动仍可能与后续躯干动作重叠。两个 TimedCmd 技能都只是
按返回的规划时长 `sleep`；无 `actual_time` 时使用请求时长作为回退值，不读取实际关节或
位姿误差。节点或根树 `SUCCESS` 不是实测到位或安全证明。

## 当前动作参数

躯干 planner `2` 的 `cmd_vec` 是 **`[x, z, yaw, pitch]`**：`x/z` 单位为米，
`yaw/pitch` 在本案例默认 Adapter `angle_unit=deg` 下为度。头部和手臂角度也按度填写；
若改 Adapter 角度单位，必须同步核对全部角度值，参数名本身不会强制转换单位。

| 阶段 | 头部 `[yaw, pitch]`（度） | 躯干 `[x, z, yaw, pitch]`（米、米、度、度） | 左/右臂各自 `[J0..J6]`（度） |
|---|---|---|---|
| 1：home | `[0, 0]` | `[0.12, 0.97, 0, 0]` | `[0, 0, 0, 0, 0, 0, 0]` |
| 2：left | `[20, 0]` | `[0.16, 1.10, 30, 0]` | `[0, 0, 0, -90, 0, 30, 0]` |
| 3：right | `[-20, 0]` | `[0.16, 1.10, -30, 0]` | `[0, 0, 0, -90, 0, -30, 0]` |
| 4：home | `[0, 0]` | `[0.12, 0.97, 0, 0]` | `[0, 0, 0, 0, 0, 0, 0]` |

`home` 是配置的基准位姿，不是躯干全零，也不是专门的复位动作。阶段 2 后直接进入
阶段 3，中间不回 home。双臂数组顺序为 `[左臂 7 关节, 右臂 7 关节]`。

| board 键 | 内容 |
|---|---|
| `torso_home_commands` / `torso_left_commands` / `torso_right_commands` | 各仅一条 planner `2` 命令 |
| `arm_home_joint_angles` / `arm_left_joint_angles` / `arm_right_joint_angles` | 各一个 14D 双臂关节目标 |
| `motion_duration_sec` | `3.0`，分别作为躯干 `desire_time` 和手臂 `total_time` |
| `motion_settle_time_sec` | `0.5`，躯干与手臂各自额外等待 |
| `head_left_yaw_deg` / `head_right_yaw_deg` | `20.0` / `-20.0` |
| `head_center_yaw_deg` / `head_center_pitch_deg` | 均为 `0.0` |

例如左向躯干命令：

```json
[{"planner_index": 2, "cmd_vec": [0.16, 1.10, 30.0, 0.0]}]
```

每阶段分别申请两段 3 秒运动，而非整个阶段只用 3 秒；实际等待取决于各自返回的
`actual_time`，再各加 0.5 秒，并有调用和 tick 开销。

## 模式与真机前提

- 每个躯干节点显式配置 `is_sync=true`、`focus_ee=false`、`prepare_arm_control=true`。
  此处只有一个 planner，`is_sync` 不表示它会和后面的手臂节点同步。
- `MultiTimedCommandMove` 下发前调用 `set_arm_control_mode(2)`，再设置
  `focus_ee=false`、`focus_z=false`。任一步失败都不下发该节点的 TimedCmd；已经产生的
  模式/焦点副作用不会自动回滚。`focus_ee=false` 也不表示禁止显式躯干目标运动。
- `ArmJointTimedMove` 本身不切控制模式；本树的先行躯干节点负责准备外部控制模式。
  树结束不会自动切回模式 0、恢复原焦点或执行物理复位。
- 使用 `config/app_config.yaml` 全局配置，无场景级 `hardware_config.json`。非 dry-run
  需要控制器、ROS/TF、LowLevel SDK、TimedCmd 及模式/焦点服务可用；运行准备见
  [执行命令](../../../docs/04-简单案例运行/02-执行命令.md)。
- 真机前须人工核对这些位姿的可达性、限位、碰撞与运动范围，清空人员、线缆及障碍物并
  安排急停监护。串行、请求成功或等待时间过去都不能替代这些检查。

## 运行

先做无 ROS、不会驱动机器人的加载与单拍检查：

```bash
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/instructional_case_03 \
  --dry-run --tick-once
```

核对场景路径、根节点 `four_head_arm_torso_phases` 和
`tick 后根状态: Status.SUCCESS`。当前启动器 dry-run 即使根为 `FAILURE` 也可能退出 0，
不能只看退出码。dry-run 不能验证实机模式、可达性、碰撞或到位。

完成现场检查后，由测试人员运行：

```bash
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/instructional_case_03
```

修改上述参数后重启场景生效；若用于 [case05 的 01→04 顺序联测](../instructional_case_05/README.md)，
还需手动更新其 JSON 中对应的子树与带前缀 board 参数；不依赖同步脚本。

## 文件

| 文件 | 用途 |
|---|---|
| `py_tree.json` | 四阶段串行主树，参数通过 `READ_BOARD` 引用 |
| `py_tree_child.json` | 空子树集合 |
| `board.json` | 头部角度、躯干 4D 位姿命令、双臂 14D 关节目标和时长 |
