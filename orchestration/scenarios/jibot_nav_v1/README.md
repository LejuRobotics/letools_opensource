# jibot_nav_v1：六点 JiBot 导航与头部下肢动作

本场景按固定顺序访问六个导航点，并在前五个点执行头部俯仰和下肢抬起、复位动作。它用于联调 JiBot 导航及上身动作的顺序控制，不包含箱体识别或抓取。

命令在 LeTools 仓库根目录执行。目标点坐标为示例固定值，实机运行前须确认地图、起点和运动空间。

## 动作流程

```text
按 Enter 开始 → 底盘相对移动并等待
  → 逐个访问导航点 1～5：
     导航 → 检查到达 → 低头 → 下肢抬起 → 等待
     → 下肢复位 → 头部回正 → 等待
  → 导航到第 6 点并检查到达 → 按 Enter 结束
```

主树为带记忆的 `Sequence`，通过 `demo_jibot_nav.json` 引入子树。前五个点的动作顺序一致；第六点只有导航和到达检查。

| 节点 | 作用 |
|---|---|
| `BaseMoveRelativeJibotMove` | 初始相对移动 |
| `BaseMoveToTargetJibotMove`、`CheckArrivedJibotMove` | 提交目标点并等待到达 |
| `HeadControlSdkMove` | 低头与回正 |
| `LegJointSdkMove` | 下肢抬起与复位 |
| `WaitSeconds`、`WaitForEnter` | 动作间等待和人工确认 |

## 参数与运行

本目录没有 `board.json`；导航坐标、头部角度、下肢关节角和等待时间均写在 `py_tree_child.json` 中。子树中的 `theta_unit` 为 `deg`。这些点位不是适用于所有地图的默认导航路线。

先做不连接 ROS 的加载检查：

```bash
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/jibot_nav_v1 --dry-run
```

`--dry-run` 不验证控制器服务或底盘实际到达。确认导航及头部、下肢服务可用后运行：

```bash
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/jibot_nav_v1
```

## 文件

| 文件 | 用途 |
|---|---|
| `py_tree.json` | 主树入口，等待开始并调用子树 |
| `py_tree_child.json` | 六点导航及前五点的头部、下肢动作 |
