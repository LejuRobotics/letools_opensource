# instructional_case_04：头、手、躯干与 JiBot 底盘并行动作

本案例复制 `instructional_case_03` 的四阶段动作，并在左转、右转和回正阶段加入
JiBot 底盘相对旋转。头部、底盘、双臂和躯干的参数均从 `board.json` 加载。

所有命令均需在仓库根目录执行。建议先执行 dry-run 检查配置，再连接真机。

## 树结构

```text
Parallel（success_on_one）
├── NodeEnableVelControl：把底盘控制权交给 JiBot 导航，结束时恢复
└── Sequence
    ├── Parallel：头回正 + 双臂全零 + 躯干基准姿态
    ├── Parallel：头左转 + JiBot 相对左转 + 双臂/躯干左向姿态
    ├── Parallel：头右转 + JiBot 相对右转 + 双臂/躯干右向姿态
    └── Parallel：头回正 + JiBot 相对回正 + 双臂/躯干回零
```

最外层 `Parallel` 使用 `success_on_one`：权限节点持续为 `RUNNING`，四阶段序列完成后
根节点成功，并终止权限节点，从而恢复运行前的底盘控制权。
如果权限节点执行失败，根节点也会失败，不会继续把序列成功误判为整个案例成功。

## 阶段内并行顺序

有底盘动作的阶段按下面的子节点顺序排列：

```text
HeadControlSdkMove
BaseMoveRelativeJibotMove
MultiTimedCommandMove
```

行为树的 `Parallel` 会依次 tick 子节点。`BaseMoveRelativeJibotMove` 调用
`/move_base/base_move` 提交任务后立即返回；`MultiTimedCommandMove` 随后提交手臂与
躯干目标，并等待规划运动结束。因此底盘实际运动会与手臂、躯干运动重叠。
如果把底盘节点放在 `MultiTimedCommandMove` 后面，TimedCmd 的等待会导致底盘延后下发。

头部节点仍放在最前面，保持与 `case_03` 相同的可靠执行顺序。本案例不需要 `Async`。

## JiBot 相对运动

本案例使用 `BaseMoveRelativeJibotMove`，底层调用
`hardware.base_move_relative_jibot()`。`x`、`y` 是相对当前本体坐标系的位移，
单位为米；`theta` 是相对当前朝向的旋转量，单位为弧度。

| 阶段 | 相对 `theta` | 约合角度 | 累计朝向 |
|---|---:|---:|---:|
| 1：初始归零 | 不下发底盘任务 | 0° | 0° |
| 2：向左 | `+0.349066` | +20° | +20° |
| 3：向右 | `-0.698132` | -40° | -20° |
| 4：回正 | `+0.349066` | +20° | 0° |

三次旋转的增量和为零，正常完成后底盘回到起始朝向。默认角速度为
`0.35 rad/s`，角度到达阈值为 `0.05 rad`。这些参数都可以在 `board.json` 中修改。

`BaseMoveRelativeJibotMove` 只负责提交任务，不查询到达状态。本示例把最大旋转段控制在
约 2 秒，并利用同阶段 TimedCmd 的约 3.5 秒等待窗口完成底盘动作。若把旋转量调大或
速度调低，应改用带任务 ID 和到达检查的导航编排，不能继续依赖这个时间窗口。

### 不要和 TimedCmd 底盘 planner 混淆

| 接口 | 是否使用 planner index | 运动含义 | 角度单位 |
|---|---:|---|---|
| `BaseMoveRelativeJibotMove` | 否 | JiBot 相对当前位姿运动 | 弧度 |
| `MultiTimedCommandMove` 的 planner `0` | 是，`0` | TimedCmd 世界坐标系底盘目标 | Adapter `angle_unit`，默认度 |
| `MultiTimedCommandMove` 的 planner `1` | 是，`1` | TimedCmd 本体坐标系底盘目标 | Adapter `angle_unit`，默认度 |

因此，planner `0` 确实是 TimedCmd 的底盘运动，但本案例的 JiBot 底盘动作没有使用
planner `0`；它使用独立的 `BaseMoveRelativeJibotMove` 节点。

## 头、手和躯干动作

`MultiTimedCommandMove` 在一次 `send_timed_multi_commands()` 请求中同步提交：

| planner | 内容 | `cmd_vec` 维度 |
|---:|---|---:|
| `3` | 下肢/躯干关节角 | 4 |
| `8` | 左臂关节角 | 7 |
| `9` | 右臂关节角 | 7 |

| 阶段 | 头部 `[yaw, pitch]`（度） | 左/右臂 `[J0..J6]`（度） | 下肢/躯干 `[j0..j3]`（度） |
|---|---|---|---|
| 1 | `[0, 0]` | `[0, 0, 0, 0, 0, 0, 0]` | `[0, 0, 0, 0]` |
| 2 | `[20, 0]` | `[0, 0, 0, -90, 0, 30, 0]` | `[29.00, -41.11, 32.21, 20]` |
| 3 | `[-20, 0]` | `[0, 0, 0, -90, 0, -30, 0]` | `[29.00, -41.11, 32.21, -20]` |
| 4 | `[0, 0]` | `[0, 0, 0, 0, 0, 0, 0]` | `[0, 0, 0, 0]` |

## 品牌配置

JiBot 由全局配置 `config/app_config.yaml` 中的
`navigation_chassis_backend: jibot` 选择。本案例不额外提供 `hardware_config.json`。
`NodeEnableVelControl(enable=false)` 会在动作期间把控制权交给 JiBot 导航模块。

## 运行

先做无 ROS、不会驱动机器人的结构校验：

```bash
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/instructional_case_04 \
  --dry-run --tick-once
```

确认输出中加载的是 `instructional_case_04/py_tree.json` 后，再进行真机运行：

```bash
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/instructional_case_04
```

真机运行前确认机器人四周、头部、双臂和躯干的完整运动范围内没有人员、线缆或障碍物。
运行日志中应出现 JiBot 的 `/move_base/base_move` 提交信息；如果只看到 TimedCmd
相关日志，先检查实际加载的场景路径是否仍是 `instructional_case_03`。

## 文件

| 文件 | 用途 |
|---|---|
| `py_tree.json` | JiBot 权限守护及四阶段主行为树 |
| `py_tree_child.json` | 空子树集合，本案例不使用子树 |
| `board.json` | 头、手、躯干和底盘动作参数 |
