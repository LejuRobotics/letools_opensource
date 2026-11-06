# refactored_sdk_arm_v1：手臂三种轨迹接口顺序演示

本场景依次演示双臂末端局部坐标轨迹、世界坐标轨迹和手臂关节轨迹，前后各执行一次手臂复位。通过它可以比较三类 SDK 动作节点在行为树中的编排方式。

命令在 LeTools 仓库根目录执行。轨迹坐标为示例值，实机运行前须确认坐标系、可达范围和周围空间。

## 动作流程

```text
按 Enter 开始 → 手臂复位
  → 局部坐标末端轨迹：前移、返回
  → 世界坐标末端轨迹：前移、上抬、返回
  → 手臂关节轨迹：展开、弯曲、回初始姿态
  → 手臂复位
```

主树是带记忆的 `Sequence`，依次引用 `demo_arm_ee_local.json`、`demo_arm_ee_world.json` 和 `demo_arm_joint.json` 三个子树；各子树内部也是顺序执行。

| 节点 | 作用 |
|---|---|
| `WaitForEnter` | 开始前人工确认 |
| `ArmResetSdkMove` | 开始和结束时复位手臂 |
| `ArmEeTrajLocalSdkMove` | 下发局部坐标系双臂末端轨迹 |
| `ArmEeTrajWorldSdkMove` | 下发世界坐标系双臂末端轨迹 |
| `ArmJointTrajSdkMove` | 下发手臂关节轨迹 |

## 参数来源

目录里有 `board.json`，保存了末端和关节轨迹样例；**当前树的参数却均为 `CUSTOM`，没有通过 `READ_BOARD` 使用这些键**。因此仅修改 `board.json` 不会改变本场景当前执行的轨迹；要改实际动作，应核对 `py_tree_child.json` 中相应节点的参数。

## 运行

先检查树和子树能否加载：

```bash
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/refactored_sdk_arm_v1 --dry-run
```

该检查不验证实际轨迹。确认 SDK 和手臂控制器就绪后执行：

```bash
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/refactored_sdk_arm_v1
```

## 文件

| 文件 | 用途 |
|---|---|
| `py_tree.json` | 主树和三个子树的调用顺序 |
| `py_tree_child.json` | 三类轨迹子树及当前执行参数 |
| `board.json` | 轨迹样例数据，当前树未引用 |
