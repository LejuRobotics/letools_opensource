# instructional_case_05：01 → 04 顺序联测

## 用途与执行顺序

供测试部门在同一次行为树运行中，按顺序执行四个教学案例各一遍。

```text
Sequence(memory=True)
├── instructional_case_01：头部顺序动作与停留
├── WaitSeconds：等待 2 秒
├── instructional_case_02：头部 + 双臂分阶段并行
├── WaitSeconds：等待 2 秒
├── instructional_case_03：每阶段头部 → 躯干位姿 → 双臂关节串行
├── WaitSeconds：等待 2 秒
└── instructional_case_04：头部、躯干/双臂与 JiBot 底盘分阶段并行
```

- 前一个案例根节点返回 `SUCCESS` 后，等待 2 秒再进入下一个；案例或等待节点返回 `FAILURE` 后不进入后续案例。
- 三处间隔复用公共 [WaitSeconds](../../nodes/wait_seconds.py)，`duration_sec=2.0` 直接配置在主树中；等待期间不推进下一案例。共增加 6 秒间隔，首例前和末例后不额外等待；实际切换还受 tick 调度影响。
- 不循环、不自动重试、不跳过失败案例、不在案例之间额外复位。固定等待不等于实测到位检查。
- 共 4 个子树、3 个案例间等待节点；展开后 66 个节点。保留各案例原有的动作参数、内部执行顺序和收尾逻辑。
- 全程复用同一个硬件适配器。案例之间不会重启控制器，也不会恢复到独立启动时的全部硬件模式。
- `SUCCESS` 是现有节点的执行结果，**不是所有部位实测到位、使能正常或无碰撞的验收证明**；硬件异常只有被节点识别并返回失败时，才能阻断后续案例。

## 文件与配置来源

本目录仅保留三个 JSON 和本 README，无生成脚本或运行前同步步骤。

| 文件 | 作用 |
|---|---|
| [py_tree.json](py_tree.json) | 根顺序树，依次引用 01～04，并在相邻案例间配置 2 秒等待 |
| [py_tree_child.json](py_tree_child.json) | 四个原始案例主树的静态副本，不新增运动逻辑 |
| [board.json](board.json) | 46 个黑板参数，按案例加前缀隔离 |
| [README.md](README.md) | 配置、运行和验收说明 |

黑板键与节点标签使用 `case_01__`、`case_02__`、`case_03__`、`case_04__` 前缀。例如：

- `case_02__arm_motion_duration_sec`：case02 手臂各段时长。
- `case_03__torso_left_commands`：case03 的躯干位姿目标。
- `case_04__motion_duration_sec`：case04 的多规划器期望时长。

case03 与 case04 的 `motion_duration_sec`、头部同名参数彼此独立，不能通过修改一个案例的参数改变另一个案例。

### 配置维护

**05 直接读取本目录的三个 JSON，不会在启动时自动重新读取 01～04，也不依赖同步脚本。**

- 调整联测参数：修改本目录 [board.json](board.json) 中对应案例前缀的键。
- 调整案例内部结构：修改 [py_tree_child.json](py_tree_child.json) 中对应子树，保留节点标签和 `READ_BOARD` 键的案例前缀。
- 案例间隔位于 [py_tree.json](py_tree.json) 的三个 `WaitSeconds.duration_sec`，当前均为 `2.0` 秒。
- 若修改原始 01～04 并希望联测保持一致，需手动更新 05 对应的子树和黑板参数；不要直接用无前缀的单项配置覆盖联测配置。目录外的离线回归会比对当前单项与联测副本，帮助发现遗漏。

改完后重启场景生效；先运行下方离线回归和 dry-run，再核对真机条件。

## 测试前核对

本轮合并没有调整 01～04 的目标或增加安全控制。真机执行前由现场负责人确认：

1. 四个单项案例已在本机型号、固件和当前配置下分别验证；尤其要确认从 case02 的 SDK 路径切到 case03 TimedCmd、再从 case03 躯干位姿切到 case04 躯干关节路径的连续过渡。
2. 控制器、ROS 环境和 TF 转发按[单项案例运行说明](../../../docs/04-简单案例运行/06-头部顺序动作-instructional_case_01.md#52-仿真或真机)准备完毕；JiBot 导航服务可用，机器人运动范围清空，有急停监护。
3. 使用与原案例一致的适配器角度配置 `deg`。头部、手臂关节、case03 的躯干 yaw/pitch、case04 的躯干关节均填度；JiBot `theta`、角速度、角度阈值仍是弧度制。
4. case03 的躯干是 **planner 2 的 `[x米, z米, yaw度, pitch度]`**，不是四关节角。当前目标为：

   | 目标 | 4D 位姿 |
   |---|---|
   | home / return home | `[0.12, 0.97, 0.0, 0.0]` |
   | left | `[0.16, 1.10, 30.0, 0.0]` |
   | right | `[0.16, 1.10, -30.0, 0.0]` |

5. case04 仍使用 **planner 3 的四关节角**与 planner 8、9 同步下发；其 home 是全零关节目标，不能把它与 case03 的 home 位姿视为同一个经过实测确认的姿态。
6. 确认当前使能、故障和控制模式。case02 第一阶段 `ArmJointTimedMove` 不负责切换手臂外部控制模式，需要启动前已满足控制器的外部控制条件；后续 SDK 段会自行管理其控制流。case03/04 的 `MultiTimedCommandSkill` 会请求外部模式 2 与 `focus_ee=False`、`focus_z=False`。
7. case05 没有开关快速模式、清故障或重新使能的步骤。此前残留的快速模式不能因换了场景而假定已清除；按现场流程确认后再执行，不要带着未知状态重跑。

## 执行命令

以下命令均在 LeTools 根目录执行。

### 1. 离线检查（不运动）

```bash
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/instructional_case_05 \
  --dry-run --tick-once
```

预期看到子树数量 `count=4`，以及：

```text
[apps][dry-run] tick 后根状态: Status.SUCCESS
```

**检查打印的根状态，不只看进程退出码。** 当前 CLI 在 dry-run 根为 `FAILURE` 时也可能以 `0` 退出。dry-run 会跳过真实等待、运动、服务和动力学，不能代替真机验收。

### 2. 真机联测（会运动）

只有完成上面的现场核对后，由操作人员执行：

```bash
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/instructional_case_05
```

不要使用后台循环自动重跑。异常时按现场急停流程处理；行为树停止或 Python 进程退出并不保证已提交的底盘任务或其他硬件运动已停止。

## 观察与记录

| 顺序 | 应观察的动作 | 重点记录 |
|---|---|---|
| 01 | 左看 → 右看 → 回中 → 抬头 → 低头 → 回中，四次 1 秒停留 | 方向、幅度、停留与实测回中；头部请求成功日志不代表到位 |
| 02 | 双臂零位 → 前伸 → 上举 → 展开 → 零位，分别配合头部动作 | 第 1 阶段 TimedCmd；后 4 段 SDK 两关键帧轨迹；各段默认 3 秒 |
| 03 | home → left → right → home；每阶段头部 → 躯干 → 手臂 | 躯干位姿和手臂各自发令、各自等待；期望时长 3 秒，额外等待 0.5 秒；无跨部位同批发送 |
| 04 | 上身分阶段动作，底盘提交相对旋转 `+20° → -40° → +20°` | 提交成功、实际转动、到达情况、收尾权限状态，分别记录 |

case04 的底盘节点**只提交任务，不等待到达**。增量角之和为零不保证实际回正；阶段里的上身等待也不等价于底盘到位检查。守护节点与动作分支处于 `Parallel`，不是顺序安全门；权限恢复是 best-effort，读不到原状态或恢复失败时要人工确认，根 `SUCCESS` 不能证明权限已恢复。

记录运行时间、机器人/固件版本、代码版本及 board 配置、根结果、失败案例/节点、服务错误、实测动作和急停情况。树内标签带案例前缀，子树名为 `instructional_case_01.json`～`instructional_case_04.json`。不要只用“根 SUCCESS”作为测试通过标准。

## 离线回归

```bash
python3 -m pytest orchestration/nodes/tests/test_instructional_case_05.py -q -s
```

覆盖：目录仅含四个文件、单项与子树副本一致、01→04 顺序、三个公共等待节点各等待 2 秒后才放行、每例只执行一遍、参数隔离、案例或等待失败后阻断后续案例、case03 空目标阻断、JiBot 正常路径权限恢复，以及 dry-run 不接触硬件。等待时序用模拟时钟验证，运动使用模拟硬件，不证明实际机械安全或控制器状态正确。

## 单项案例说明

- [case01：头部顺序动作](../instructional_case_01/README.md)
- [case02：手臂与头部分阶段并行](../instructional_case_02/README.md)
- [case03：头部、躯干位姿与手臂串行](../instructional_case_03/README.md)
- [case04：头手躯干与 JiBot 底盘并行](../instructional_case_04/README.md)
