# instructional_case_03：头部、手臂与躯干并行动作

本案例在 `case_02` 的基础上加入躯干，演示如何用一个可配置的
`MultiTimedCommandMove` 同时提交多个 TimedCmd planner。主树由四个
`Parallel` 组成，所有动作参数均从 `board.json` 加载。

所有命令均需在仓库根目录执行。建议先执行 dry-run 检查配置，再连接真机。

## 树结构

```text
Sequence
├── Parallel：头回正 + 双臂全零 + 躯干基准姿态
├── Parallel：头左转 + 双臂左向姿态 + 躯干左转
├── Parallel：头右转 + 双臂右向姿态 + 躯干右转
└── Parallel：头回正 + 双臂全零 + 躯干回基准姿态
```

每个 `Parallel` 有两个子节点，并保持头部节点在前：

```text
HeadControlSdkMove
MultiTimedCommandMove
```

`HeadControlSdkMove` 先快速下发头部目标。随后 `MultiTimedCommandMove` 用一次
`send_timed_multi_commands()` 请求提交 board 中的一组命令。当前案例配置的是：

| planner | 内容 | `cmd_vec` 维度 |
|---:|---|---:|
| `3` | 下肢/躯干关节角 | 4 |
| `8` | 左臂关节角 | 7 |
| `9` | 右臂关节角 | 7 |

节点默认设置 `is_sync=true`，三个 planner 由 TimedCmd 服务端统一规划、同时完成。
头部目标已在此前下发，因此头部与这组 TimedCmd 动作可以重叠。

这里有两层“并行”，含义不同：行为树的 `Parallel` 会按子节点顺序逐个 tick；
TimedCmd 的 `is_sync=true` 则让同一请求中的 `3/8/9` planner 按同一结束时间规划。
`MultiTimedCommandMove` 会等待这组运动结束，所以头部节点必须放在它前面。

本案例不使用 `Async`。`Async` 只改变行为树线程调度，并不能把多次 TimedCmd
调用合成一次。若手臂和躯干在两个线程里分别调用 TimedCmd，它们会竞争同一个
控制器主循环，可能返回 `main loop is updating the command and is busy`。

## 配置其他组合

`MultiTimedCommandMove` 不写死 `3/8/9`。每条命令只需要两个字段：

```json
{
  "planner_index": 3,
  "cmd_vec": [29.0, -41.11, 32.21, 20.0]
}
```

所有命令放在同一个数组中。比如躯干位姿加双臂关节使用 `2/8/9`：

```json
[
  {"planner_index": 2, "cmd_vec": [0.2, 1.0, 0.0, 0.0]},
  {"planner_index": 8, "cmd_vec": [0, 0, 0, 0, 0, 0, 0]},
  {"planner_index": 9, "cmd_vec": [0, 0, 0, 0, 0, 0, 0]}
]
```

下肢关节加双臂局部末端位姿使用 `3/6/7`：

```json
[
  {"planner_index": 3, "cmd_vec": [0, 0, 0, 0]},
  {"planner_index": 6, "cmd_vec": [0.3, 0.2, 0.1, 0, 0, 0]},
  {"planner_index": 7, "cmd_vec": [0.3, -0.2, 0.1, 0, 0, 0]}
]
```

planner 编号表示运动规划类型，不是关节序号。编号和向量格式如下：

| planner | 格式 |
|---:|---|
| `0` | 底盘世界坐标系目标 `[x, y, yaw]` |
| `1` | 底盘本体坐标系目标 `[x, y, yaw]` |
| `2` | 躯干局部位姿 `[x, z, yaw, pitch]` |
| `3` | 下肢/躯干关节 `[j0, j1, j2, j3]` |
| `4` / `5` | 左/右臂世界系末端 `[x, y, z, yaw, pitch, roll]` |
| `6` / `7` | 左/右臂局部系末端 `[x, y, z, yaw, pitch, roll]` |
| `8` / `9` | 左/右臂 7 个关节角 |

位置单位为米；角度遵循 Adapter 的 `angle_unit` 配置，默认是度。这里的
planner `0/1` 属于 TimedCmd 接口，不是 `case_04` 使用的 JiBot 相对导航接口；
后者的 `theta` 固定使用弧度。

节点会校验每个 planner 的向量维度，也不允许同一组中重复 planner。每条命令可以额外写
`"desire_time": 4.0` 覆盖节点的默认执行时间。

节点参数 `focus_ee` 控制本次下发前的笛卡尔跟踪焦点，默认 `false`：

- `true`：专注手臂末端解算，躯干可能随动；
- `false`：尽可能不动躯干，在手臂工作区间内解算，精度可能不高。

技能会将 `focus_z` 设置为与 `focus_ee` 相同的值，不支持在节点上分别配置。任一焦点设置失败时，本次运动命令不会下发；dry-run 不调用硬件。

## 当前动作参数

| 阶段 | board 命令组 | 头部 `[yaw, pitch]` | 左/右臂 `[J0..J6]` | 下肢/躯干 `[j0, j1, j2, j3]` |
|---|---|---|---|---|
| 1：初始归零 | `home_commands` | `[0, 0]` | `[0, 0, 0, 0, 0, 0, 0]` | `[0, 0, 0, 0]` |
| 2：向左 | `forward_and_left_commands` | `[20, 0]` | `[0, 0, 0, -90, 0, 30, 0]` | `[29.00, -41.11, 32.21, 20]` |
| 3：向右 | `forward_and_right_commands` | `[-20, 0]` | `[0, 0, 0, -90, 0, -30, 0]` | `[29.00, -41.11, 32.21, -20]` |
| 4：回零 | `home_commands` | `[0, 0]` | `[0, 0, 0, 0, 0, 0, 0]` | `[0, 0, 0, 0]` |

当前默认配置下角度单位均为度，TimedCmd 期望时间为 3 秒。修改动作或 planner 组合时只需编辑
`board.json` 中的 `home_commands`、`forward_and_left_commands` 和
`forward_and_right_commands`。阶段 2 完成后会直接执行阶段 3，中间不会先回零。

## 运行

先做无 ROS、不会驱动机器人的结构校验：

```bash
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/instructional_case_03 \
  --dry-run --tick-once
```

确认输出中加载的是 `instructional_case_03/py_tree.json` 后，再进行真机运行：

```bash
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/instructional_case_03
```

实机运行前确认头部、双臂及升降躯干的完整运动范围内没有人员、线缆或障碍物。

## 文件

| 文件 | 用途 |
|---|---|
| `py_tree.json` | 四阶段主行为树 |
| `py_tree_child.json` | 空子树集合，本案例不使用子树 |
| `board.json` | 头部参数与每阶段的 TimedCmd 命令组 |
