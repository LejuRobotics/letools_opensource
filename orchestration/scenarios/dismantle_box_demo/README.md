# dismantle_box_demo：单点导航与吸附动作演示

本场景把底盘导航、手臂与头部下肢准备、双臂末端轨迹和真空吸附接成一条单点流程。它使用固定目标点与固定轨迹，是动作联调示例，不负责从视觉结果生成抓取位姿。

以下命令均在 LeTools 仓库根目录执行。实际运行前须核对固定坐标、末端轨迹及真空泵连接。

## 动作流程

```text
按 Enter 开始 → 底盘相对移动并等待
  → 并行提交导航、手臂关节轨迹、头部下俯和下肢姿态 1
  → 检查底盘到达 → 等待 → 双臂末端 TimedCmd 轨迹
  → 真空吸附 → 下肢姿态 2、3 → 头部回正
  → 导航返回初始点并检查到达 → 真空释放
  → 按 Enter 结束
```

主树是带记忆的 `Sequence`，实际动作写在 `demo_jibot_nav.json` 子树中。`Parallel` 的 `success_on_all` 只表示四个分支都成功才继续；底盘导航分支提交任务后，后续仍由 `CheckArrivedJibotMove` 检查到达。

| 节点 | 作用 |
|---|---|
| `BaseMoveRelativeJibotMove`、`BaseMoveToTargetJibotMove` | 初始移动及目标点导航 |
| `CheckArrivedJibotMove` | 等待导航任务到达 |
| `ArmJointTrajSdkMove`、`HeadControlSdkMove`、`LegJointSdkMove` | 准备和复位上身姿态 |
| `ArmEeTimedCmdMove` | 下发双臂末端定时轨迹 |
| `VacuumControlMove` | 吸附及释放 |
| `WaitSeconds`、`WaitForEnter` | 动作间等待和人工确认 |

## 参数与运行

本目录没有 `board.json`；目标点、关节角和末端轨迹都在 `py_tree_child.json` 的 `CUSTOM` 参数中。修改轨迹时应同时核对坐标系和机器人可达范围。

先做配置加载检查，`--dry-run` 不证明导航、吸附或机械动作可以完成：

```bash
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/dismantle_box_demo --dry-run
```

在导航、上身控制和真空泵服务就绪后运行：

```bash
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/dismantle_box_demo
```

## 文件

| 文件 | 用途 |
|---|---|
| `py_tree.json` | 主树入口，等待开始并调用子树 |
| `py_tree_child.json` | 单点导航、并行准备、吸附与返回的完整动作 |
