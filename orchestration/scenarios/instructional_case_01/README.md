# instructional_case_01：头部顺序动作

本案例从最简单的顺序树开始：依次向头部下发左右、上下和回正目标。通过本案例
可以了解 `Sequence`、board 参数引用和等待节点的基本用法。

所有命令均需在仓库根目录执行。建议先执行 dry-run 检查配置，再连接真机。

## 动作流程

主树是一个带记忆的 `Sequence`：

```text
左转 20°
  -> 等待 1 秒
  -> 右转 20°
  -> 等待 1 秒
  -> 回正
  -> 抬头 20°
  -> 等待 1 秒
  -> 低头 20°
  -> 等待 1 秒
  -> 回正
```

`memory=true` 表示树遇到正在执行的节点时，下一个 tick 会从该节点继续，而不会
重新执行前面已经成功的头部命令。

使用的叶子节点：

| 节点 | 作用 |
|---|---|
| `HeadControlSdkMove` | 下发一个头部 yaw/pitch 目标角 |
| `WaitSeconds` | 在指定时间内保持 `RUNNING`，让下一个头部目标延后下发 |

`HeadControlSdkMove` 负责下发目标角，调用成功后就会返回；它不等待头部实际到达目标。
因此，本案例使用 `WaitSeconds` 留出运动和观察时间。头部速度由底层控制器决定，
该节点目前没有单独的速度参数。等待从节点启动时计时，不从实测到位时计时。
全树共 11 个节点（根 `Sequence`、6 次头部目标、4 次等待）；左右动作后的回正与抬头
之间没有专门等待，最后回正后也没有等待。不能据此保证会观察到一次完整的中间回正，
也不能把根 `SUCCESS` 当作最终头部到位。

## 角度约定

| 动作 | yaw | pitch |
|---|---:|---:|
| 左转 | `+20°` | `0°` |
| 右转 | `-20°` | `0°` |
| 抬头 | `0°` | `-20°` |
| 低头 | `0°` | `+20°` |
| 回正 | `0°` | `0°` |

本案例按 Adapter 默认 `angle_unit=deg` 填值。参数名虽以 `_deg` 结尾，实际仍由
Adapter 的单位配置解释；若修改该配置，必须同步核对数值。
本案例使用 `config/app_config.yaml` 的全局硬件配置，不需要单独的
`hardware_config.json`。

## 从 board 加载参数

头部角度和停留时间集中放在 `board.json`：

| board 键 | 默认值 | 用途 |
|---|---:|---|
| `head_left_yaw_deg` | `20.0` | 左转 yaw |
| `head_right_yaw_deg` | `-20.0` | 右转 yaw |
| `head_up_pitch_deg` | `-20.0` | 抬头 pitch |
| `head_down_pitch_deg` | `20.0` | 低头 pitch |
| `head_center_yaw_deg` | `0.0` | 正中 yaw，也供上下动作复用 |
| `head_center_pitch_deg` | `0.0` | 正中 pitch，也供左右动作复用 |
| `head_hold_duration_sec` | `1.0` | 每个方向停留时间 |

主树通过 `READ_BOARD` 引用这些值。例如左转节点的 yaw：

```json
"yaw_deg": {
  "source": "READ_BOARD",
  "board_key": "head_left_yaw_deg"
}
```

修改动作时只需编辑 `board.json`，不用改行为树结构；重启行为树后生效。

## 运行

先做无 ROS、不会驱动机器人的结构校验：

```bash
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/instructional_case_01 \
  --dry-run --tick-once
```

核对加载路径和 `tick 后根状态: Status.SUCCESS`，不能只看退出码（当前 dry-run
即使根为 `FAILURE` 也可能退出 0）。dry-run 不验证实际到位或运动安全。
完成 [运行环境与安全检查](../../../docs/04-简单案例运行/02-执行命令.md)，确认控制器、
ROS/TF 与 LowLevel SDK 就绪后，再由测试人员进行真机运行：

```bash
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/instructional_case_01
```

实机运行前确认头部转动范围内没有人员、线缆或障碍物，并安排急停监护。
当前头部 Adapter 将底层 `Result` 按对象真值判断，底层失败也可能被报告为成功；
出现 `SDK 未初始化` 等错误时不能判为通过，需核对实际反馈/动作。

全部单项检查完成后，可使用 [case05：01→04 顺序联测](../instructional_case_05/README.md)。

## 文件

| 文件 | 用途 |
|---|---|
| `py_tree.json` | 主行为树，通过 `READ_BOARD` 引用动作参数 |
| `py_tree_child.json` | 空子树集合，本案例不使用子树 |
| `board.json` | 头部角度与停留时间 |
