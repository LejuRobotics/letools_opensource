# dismantle_box：六点导航与上身动作演示

本场景依次访问六个固定导航点，在每个点执行手臂、头部和下肢姿态动作，再返回固定初始点。它演示 `Sequence`、子树引用和到达检查；树中没有相机识别、吸附或实际拆箱动作。

以下命令在 LeTools 仓库根目录执行。坐标与关节角直接写在子树配置中，连接实机前须核对它们与现场环境匹配。

## 动作流程

```text
按 Enter 开始 → 底盘相对移动并等待
  → 对导航点 1～6 逐点执行：
     导航 → 检查到达 → 手臂轨迹 → 头部下俯
     → 下肢姿态 1、2、3（中间等待）→ 头部回正
     → 返回初始点并检查到达
  → 按 Enter 结束
```

主树是带记忆的 `Sequence`，引用 `py_tree_child.json` 中的 `demo_jibot_nav.json`。因此等待或动作返回 `RUNNING` 时，不会重新执行此前成功的步骤。每个导航点的动作都是顺序执行；本场景没有并行动作分支。

| 节点 | 作用 |
|---|---|
| `WaitForEnter` | 在开始和结束时等待人工确认 |
| `BaseMoveRelativeJibotMove` | 执行初始相对移动 |
| `BaseMoveToTargetJibotMove`、`CheckArrivedJibotMove` | 提交目标点并确认到达 |
| `ArmJointTrajSdkMove` | 下发手臂关节轨迹 |
| `HeadControlSdkMove`、`LegJointSdkMove` | 调整头部和下肢姿态 |
| `WaitSeconds` | 在部分动作之间留出等待时间 |

## 参数与运行

本目录没有 `board.json`；导航坐标、角度、等待时间等均写在 `py_tree_child.json` 的节点参数中。修改前先确认坐标系、角度单位和机器人工作空间。

先检查树能否加载。`--dry-run` 不连接 ROS，也不验证实际运动：

```bash
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/dismantle_box --dry-run
```

确认 JiBot 导航、头部、手臂和下肢控制服务可用后，才运行实机命令：

```bash
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/dismantle_box
```

## 文件

| 文件 | 用途 |
|---|---|
| `py_tree.json` | 主树：开始确认、子树引用 |
| `py_tree_child.json` | 六个导航点及上身动作的具体顺序和参数 |
