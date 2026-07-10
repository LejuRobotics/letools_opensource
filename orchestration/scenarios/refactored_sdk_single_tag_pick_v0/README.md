# `refactored_sdk_single_tag_pick_v0` 综合案例

> 单 tag 抓取全流程,验证 refactored SDK + PyTrees + 三链运行模式的可复用模板。

| 字段 | 值 |
|---|---|
| Case 名 | `refactored_sdk_single_tag_pick_v0` |
| 路径 | `orchestration/scenarios/refactored_sdk_single_tag_pick_v0/` |
| 启动器 | [`run_behavior_tree_json.py`](../../../apps/test_upper_init/run_behavior_tree_json.py) |
| 默认树 | `py_tree.json`(主) + `py_tree_child.json`(3 棵子树内嵌) |
| 黑板 | `board.json`(22 字段) |
| 状态 | 28/28 单元测试 PASS,集成测试通过(2026-06-13) |

---

## 目标

机器人识别面前**单个 AprilTag**,用单臂**走到 + 抓 + 抬起 + 退回 + 复位**。整个过程**不依赖外部真实 tag**,支持**干跑 / MuJoCo / 真机**三链运行,真机模式下若真实 tag 30s 内未找到则**降级注入 fake tag** 继续执行。

---

## 节点清单(9 节点 + 2 utils)

| 类型 | 节点 | 角色 |
|---|---|---|
| 🆕 NEW | `NodeHeadSequence` | 扫头序列(多 yaw × 多 pitch,循环重扫,超时降级) |
| 🆕 NEW | `NodePercep` | 标签检测写到 blackboard(`latest_tag_<id>`,带 version 计数) |
| 🆕 NEW | `NodeWaitForBlackboard` | 三链行为: 干跑/虚拟/真机+超时降级 |
| 🔁 PORTED | `NodeComputePickGoal` | 由 fake tag 公式 + box 偏移算底盘目标 |
| 🔁 PORTED | `NodeTagToArmGoal` | 由 tag + box + lift 算臂轨迹(支持 pick / lift 两套 keypoints) |
| 🔁 PORTED | `NodeWheelWalk` | 底盘走(支持 `cmd_pos_world` / `cmd_pos` / `cmd_vel` 三模式) |
| 🔁 PORTED | `NodeWheelArm` | 臂走(支持 `joint` / `eef` 两模式) |
| 🆕 NEW | `EndEffectorClose` | 夹爪闭合(left/right/both) |
| 🆕 NEW | `SetBackWalkGoal` | 把底盘目标重置为 `(0,0,0)` |
| 🆕 UTIL | `generate_pick_keypoints` | 算抓取轨迹 → `Tuple[List[Pose], List[Pose]]` |
| 🆕 UTIL | `generate_lift_keypoints` | 算抬起轨迹 → `Tuple[List[Pose], List[Pose]]` |

> 标记:🆕=本 case 新增; 🔁=从早期代码移植; 🆕 UTIL=纯函数工具。

---

## 子树角色(4 棵,合并在 `py_tree_child.json` 中)

| 子树 key | 行为 | 关键节点 |
|---|---|---|
| `py_tree.json` | 顶层 Sequence,4 子项 | `head_search_parallel` + 3 棵 demo |
| `demo_single_tag_walk_and_pick.json` | 走到抓取点 → 抓 | `NodeComputePickGoal` → `NodeWheelWalk` → `NodeTagToArmGoal` → `NodeWheelArm` → `EndEffectorClose` |
| `demo_single_tag_lift.json` | 抬起物体 | `NodeTagToArmGoal`(keypoints_source=lift) → `NodeWheelArm` |
| `demo_single_tag_back_and_reset.json` | 并行: 底盘回原点 + 头/臂/躯干复位 | `SetBackWalkGoal` + `NodeWheelWalk` ‖ `HeadControlSdkMove` + `ArmResetSdkMove` + `TorsoResetSdkMove` |

> 3 棵子树**全部内嵌在 `py_tree_child.json`** 中(单文件,key=文件名),这是 `BehaviorTreeFactory` 唯一支持的子树机制。`py_tree.json` 节点 `"name"` 字段引用 key 名,二者必须严格一致。

**主树时序**:

```
Sequence(single_tag_pick_v0)
├─ Parallel(head_search_parallel)         # 三件事并行直到任一成功
│  ├─ NodeHeadSequence(扫头)
│  ├─ NodePercep(检测)
│  └─ NodeWaitForBlackboard(等真 tag / 注入 fake)
├─ demo_single_tag_walk_and_pick
├─ demo_single_tag_lift
└─ demo_single_tag_back_and_reset
```

---

## 启动命令(3 链)

### 链 1：干跑

```bash
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/refactored_sdk_single_tag_pick_v0 \
  --dry-run --tick-once
```

### 链 2：MuJoCo 仿真

需先手动启动 MuJoCo(`roslaunch humanoid_controllers load_kuavo_mujoco_sim_wheel.launch`),然后:

```bash
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/refactored_sdk_single_tag_pick_v0
```

### 链 3：真机

需先启动下位机(`roslaunch humanoid_controllers load_kuavo_real_wheel.launch`),然后同上条命令。

### 单元测试

```bash
pytest apps/test_upper_init/tests/test_refactored_sdk_single_tag_pick_v0.py -v
```

> 1 个 subprocess 集成测试 `test_scenario_dry_run_tick_once_succeeds` 默认 deselected,需手动跑确认全链路不依赖 mock 节点。

---

## board.json 22 字段速查

| 字段 | 类型 | 默认 | 用途 |
|---|---|---|---|
| `pick_tag_id` | int | 1 | 要抓的目标 tag id |
| `tag_ids` | list[int] | [1] | 扫头期间期望检测到的 tag 集合 |
| `stand_in_tag_pos` | list[float] | [0,0,0.55] | 机器人站立位置在 tag 局部系下的偏置 |
| `stand_in_tag_euler` | list[float] | [0,0,0] | 同上,欧拉角 |
| `box_width` | float | 0.35 | 物体宽度,用于决定抓取间距 |
| `box_behind_tag` | float | 0.0 | 物体在 tag 后方(odom 系) |
| `box_beneath_tag` | float | 0.0 | 物体在 tag 下方(odom 系) |
| `box_left_tag` | float | 0.0 | 物体在 tag 左侧(odom 系) |
| `head_search_yaws` | list[float] | [12,0,-12,0] | 扫头 yaw 序列(度) |
| `head_search_pitchs` | list[float] | [-15,0,15,0] | 扫头 pitch 序列(度) |
| `tag_scan_timeout` | float | 30.0 | 扫头超时秒数 |
| `chassis_walk_mode` | str | `cmd_pos_world` | `cmd_pos_world` / `cmd_pos` / `cmd_vel` |
| `arm_control_type` | str | `joint` | `joint` / `eef` |
| `end_effector_side` | str | `both` | `left` / `right` / `both` |
| `gripper_position` | int | 100 | 夹爪闭合位置 |
| `gripper_effort` | float | 1.0 | 夹爪力矩 |
| `z_lift` | float | 0.2 | 抬起高度(米) |
| `hand_pitch_degree` | float | 0.0 | 末端 pitch 角(度) |
| `use_virtual_tag` | bool | false | 强制走 fake tag 路径 |
| `real_tag_timeout_sec` | float | 30.0 | 真 tag 超时后注入 fake |
| `virtual_tag_pose_in_odom` | list[float] | [1,0,0,0,0,0] | fake tag 在 odom 系的 6 元组 pose |

---

## 关键设计决策

### 1. 为什么三链?

| 链 | 价值 | 失败代价 |
|---|---|---|
| 干跑 | CI/无硬件可验证 | import 错误、JSON 错误 |
| MuJoCo | 验证节点时序、blackboard 流转 | 仿真器启动耗时 |
| 真机 | 验证闭环控制 | 物理安全 |

不在干跑上验证的:硬件调用细节、视觉算法真值、动力学。

### 2. 为什么用 fake tag + 30s 兜底?

- 干跑无法触发视觉算法 → 走 fake
- MuJoCo 视觉模块可能不识别 tag → 走 fake
- 真机视觉失败(光照/遮挡)→ 走 fake,继续验证**后续逻辑**

公式: `fake_tag_pose = virtual_tag_pose_in_odom - stand_in_tag_*`(把"机器人站立位置"局部系的 tag 反推到 odom)。

### 3. 为什么把 `head_search` 和 `percep` 并行?

扫头是慢动作(逐个 yaw/pitch),等扫完再 percep 会**浪费时间**。两者并发在 `Parallel(success_on_one)` 下,任意一个成功就推进。

### 4. 为什么 `back_and_reset` 用 `Parallel(success_on_all)`?

底盘回原点和头/臂/躯干复位**互不依赖**,可并行。但**都成功**才算完成,所以用 `success_on_all` 而非 `success_on_one`。

### 5. 为什么 `NodeTagToArmGoal` 用 `keypoints_source` 参数而非两个独立节点?

抓取轨迹(pick)和抬起轨迹(lift)共享 tag + box + hand_pitch_degree 输入,仅终值不同(目标点 vs 抬起点)。**用同一节点 + 参数切换**比拆成两个节点更省配置。

---

## 下一步(v1 候选差异点)

- **多 tag 场景**: 把 `tag_ids` 扩展为 2+,`NodePercep` 需支持"任一 tag 触发"
- **动态抓取点**: `box_*` 不再是常量,而是订阅 `/detected_object_size` 动态读取
- **双臂**: `end_effector_side=both` 已经是路径,但需新增双臂协同节点(目前仅顺序)
- **失败回滚**: 当前链路任一 FAILURE 即终止,v1 可加 `Recovery` 行为树机制

---

## 相关文件

| 路径 | 用途 |
|---|---|
| [board.json](./board.json) | 22 字段黑板 |
| [py_tree.json](./py_tree.json) | 主树 |
| [py_tree_child.json](./py_tree_child.json) | 3 棵 demo 子树内嵌集合 |
| [apps/test_upper_init/tests/test_refactored_sdk_single_tag_pick_v0.py](../../../apps/test_upper_init/tests/test_refactored_sdk_single_tag_pick_v0.py) | 11 单元测试 |
| [apps/test_upper_init/readme.md](../../../apps/test_upper_init/readme.md) | 启动器使用说明 |
| [orchestration/README.md](../../README.md) | 编排层总览 |
| [orchestration/nodes/](../../nodes/) | 9 节点实现 |
| [core/util/keypoints.py](../../../core/util/keypoints.py) | 2 工具函数 |
