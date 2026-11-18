# instructional_case_04：头、手、躯干与 JiBot 底盘并行动作

本案例独立使用四阶段并行结构：头部 SDK 目标、一次 TimedCmd **planner `3/8/9`**
组合请求，以及阶段 2–4 的 JiBot 相对旋转。它与当前 case03 的
“头部 → planner 2 躯干位姿 → 14D 双臂目标”串行结构不同，不能共用躯干命令格式。

所有命令在仓库根目录执行。先 dry-run，再由测试人员确认真机条件；普通 MuJoCo 环境
没有本案例所需的 JiBot 导航服务。

## 树结构与控制权

```text
Parallel(jibot_control_guard_and_motion, success_on_one)
├── NodeEnableVelControl(enable=false, restore_on_stop=true)
└── Sequence(four_head_arm_torso_chassis_phases, memory=true)
    ├── Parallel：头回正 + 双臂/躯干关节归零
    ├── Parallel：头左转 + JiBot 相对左转 + 双臂/躯干左向目标
    ├── Parallel：头右转 + JiBot 相对右转 + 双臂/躯干右向目标
    └── Parallel：头回正 + JiBot 相对回正 + 双臂/躯干关节归零
```

共 **18 个节点**。阶段 `Parallel` 均为 `success_on_all`；外层使用 `success_on_one`，
因为权限守护正常时一直 `RUNNING`，动作序列成功后根树成功并终止守护节点。

`NodeEnableVelControl` 在初始化时尝试把底盘控制权交给 JiBot 导航，终止时尽力恢复原权限：

- 硬件没有 `enable_vel_control_jibot` 方法时跳过并保持 `RUNNING`，不代表权限已正确切换；
- 调用切换接口失败时返回 `FAILURE`，根树会失败；但 `Parallel` 会先 tick 各分支，
  **不是失败即阻止同拍其他分支下发命令的安全门**；
- 本场景未配置 `restore_to`。原权限读不到时仍可能切换成功，收尾跳过恢复；
- 恢复只是 best-effort：恢复失败只记录告警，不会将已经成功的根树改判为失败。

因此运行前、运行后均需人工确认权限。序列节点实际返回 `FAILURE` 后不进入后续阶段，
不等于所有硬件异常都会被可靠反馈，也不等于已提交任务已停止。

## 阶段内顺序与返回语义

阶段 1 只有头部和 TimedCmd 节点；阶段 2–4 按以下顺序 tick：

```text
HeadControlSdkMove
BaseMoveRelativeJibotMove
MultiTimedCommandMove
```

头部只下发目标、不查询到位。底盘调用 `/move_base/base_move`，任务被接受后即返回，
不等待到达。随后 TimedCmd 一次提交双臂与躯干目标，技能等待服务返回的
`actual_time + 0.5 s`（无该值时回退到 `motion_duration_sec`）。这里的等待是 `sleep`，
没有实际位姿/关节误差检查。物理运动可以重叠，但请求顺序或 `SUCCESS` 不证明同步到位。
把底盘节点放到 TimedCmd 后面只会延后提交任务，不会增加到达检查。

## JiBot 相对运动

`x/y` 是本体系相对位移（米），`theta` 是相对当前朝向的旋转增量（弧度，
不受 Adapter `angle_unit` 影响）。当前 `x=y=0`：

| 阶段 | 相对 `theta`（rad） | 约合角度 | 理想累计朝向变化 |
|---|---:|---:|---:|
| 1 | 不下发底盘任务 | — | 0° |
| 2 | `+0.349066` | +20° | +20° |
| 3 | `-0.698132` | -40° | -20° |
| 4 | `+0.349066` | +20° | 0° |

这只是命令增量的理想累计。只有每段执行符合预期时才可能回到起始朝向；
本树不查询到达，增量和为零也不能证明回正。默认角速度 `0.35 rad/s`，角度阈值
`0.05 rad`，均由 `board.json` 配置；避障沿用节点默认 `false`。

最大一段按 `0.698132 / 0.35` 粗算约 2 秒，**不是完成时间上限**。同阶段 TimedCmd
名义等待约 3.5 秒也不是底盘到达条件；加减速、调度或导航异常都可能使前一任务尚未结束
就提交下一任务，树收尾还可能在底盘在途时恢复权限。验收需要逐任务到达证据时，应另行
编排保存返回 `task_id` 并查询到达/超时；本节点当前不把 `task_id` 写入黑板，不能只增加
一个未绑定真实任务 ID 的检查节点就认为解决。

本案例的底盘不是 TimedCmd planner `0/1`：后者分别是世界系/本体系底盘目标，
角度随 Adapter 单位（默认度），与这里固定弧度的 JiBot 接口不同。

## 头、手和躯干参数

`MultiTimedCommandMove` 的每个命令组都包含：

| planner | 内容 | 维度与单位 |
|---:|---|---|
| `3` | 下肢/躯干关节角 | 4D，度 |
| `8` | 左臂关节角 | 7D，度 |
| `9` | 右臂关节角 | 7D，度 |

| 阶段 | board 命令组 | 头部 `[yaw, pitch]`（度） | 左/右臂各自 `[J0..J6]`（度） | 躯干 `[j0..j3]`（度） |
|---|---|---|---|---|
| 1 | `home_commands` | `[0, 0]` | `[0, 0, 0, 0, 0, 0, 0]` | `[0, 0, 0, 0]` |
| 2 | `forward_and_left_commands` | `[20, 0]` | `[0, 0, 0, -90, 0, 30, 0]` | `[29.00, -41.11, 32.21, 20]` |
| 3 | `forward_and_right_commands` | `[-20, 0]` | `[0, 0, 0, -90, 0, -30, 0]` | `[29.00, -41.11, 32.21, -20]` |
| 4 | `home_commands` | `[0, 0]` | `[0, 0, 0, 0, 0, 0, 0]` | `[0, 0, 0, 0]` |

这些角度按默认 Adapter `angle_unit=deg` 填写；修改单位配置时必须同步核对数值。
`motion_duration_sec=3.0` 是期望规划时长。节点默认 `is_sync=true` 请求三个 planner
同步规划，但不证明实际同时到位；默认 `settle_time=0.5`。

默认 `prepare_arm_control=true` 在每次请求前调用 `set_arm_control_mode(2)`，随后
设置 `focus_ee=false`、`focus_z=false`；任一步失败则不下发该节点的 TimedCmd。
已产生的副作用不回滚，结束不切回手臂模式 0、不恢复原焦点、不复位。
底盘权限守护与这些手臂/躯干模式是不同机制。

## 运行与验收

使用 `config/app_config.yaml` 全局配置，`navigation_chassis_backend: jibot`；无
场景级 `hardware_config.json`。

```bash
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/instructional_case_04 \
  --dry-run --tick-once
```

核对场景路径、根节点 `jibot_control_guard_and_motion` 和
`tick 后根状态: Status.SUCCESS`，不能只看退出码（当前 dry-run 根失败也可能退出 0）。
dry-run 不检查 JiBot 服务、权限、可达性、碰撞或实际到位。

真机需先按 [执行命令](../../../docs/04-简单案例运行/02-执行命令.md) 准备控制器、ROS/TF、
SDK、TimedCmd 及模式/焦点服务，确认 JiBot `/move_base/base_move` 可用、底盘权限可读取
和切换、无其他在途任务。清空机器人四周和头/臂/躯干运动范围，核对限位并安排急停监护后，
由测试人员运行：

```bash
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/instructional_case_04
```

日志中的三个 JiBot 任务提交只能证明请求被接受，不能证明到达；验收须单独记录实际姿态、
底盘任务状态以及最终控制权。失败或中断后不能假定已停止或已恢复，应先人工确认。

[case05：01→04 顺序联测](../instructional_case_05/README.md) 会复用本场景的现有行为与边界，
相邻案例间各用公共 `WaitSeconds` 等待 2 秒；固定间隔不替代到达检查，也不会额外补模式恢复或物理复位。

## 文件

| 文件 | 用途 |
|---|---|
| `py_tree.json` | JiBot 权限守护及四阶段并行主树 |
| `py_tree_child.json` | 空子树集合 |
| `board.json` | 头部角度、planner `3/8/9` 命令组和 JiBot 相对运动参数 |
