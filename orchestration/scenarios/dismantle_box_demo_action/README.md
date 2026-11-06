# dismantle_box_demo_action：上身抓取姿态动作演示

本场景只演示手臂、头部和下肢的配合，以及一段双臂末端 TimedCmd 轨迹。它沿用导航示例的子树名称，但树中没有底盘导航、到达检查或真空泵动作，不能据此认定完成了抓箱。

所有命令在 LeTools 仓库根目录执行。实际动作参数为固定值，实机运行前须检查空间和关节限位。

## 动作流程

```text
按 Enter 开始
  → 手臂关节轨迹、头部下俯和下肢姿态 1 并行
  → 等待 → 双臂末端定时轨迹 → 等待
  → 下肢姿态 2 → 等待 → 下肢姿态 3 → 等待
  → 头部回正 → 等待 → 按 Enter 结束
```

主树为带记忆的 `Sequence`，引用 `py_tree_child.json` 中的 `demo_jibot_nav.json`。子树里的 `Parallel` 使用 `success_on_all`，只有三个准备动作均成功才进入末端轨迹。

| 节点 | 作用 |
|---|---|
| `ArmJointTrajSdkMove` | 下发手臂关节轨迹 |
| `HeadControlSdkMove` | 低头与回正 |
| `LegJointSdkMove` | 三段下肢姿态 |
| `ArmEeTimedCmdMove` | 双臂末端定时轨迹 |
| `WaitSeconds`、`WaitForEnter` | 延时与人工确认 |

## 参数与运行

本目录没有 `board.json`；角度、轨迹、时间和末端坐标系写在 `py_tree_child.json` 中。`ArmEeTimedCmdMove` 的 `frame` 当前为 `local`。

```bash
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/dismantle_box_demo_action --dry-run
```

`--dry-run` 仅检查加载。上身控制器就绪、动作范围已核对后，才执行：

```bash
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/dismantle_box_demo_action
```

## 文件

| 文件 | 用途 |
|---|---|
| `py_tree.json` | 主树入口 |
| `py_tree_child.json` | 并行准备与后续上身动作 |
