# 10 搬箱场景-grasp_mtbf

[返回文档首页](../README.md) · [04-简单案例运行](../04-简单案例运行/) · [03-接口详细文档](../03-接口详细文档/)

本文中的相对路径一律相对 **LeTools 仓库根目录**；各命令需在仓库根目录下执行。

**难度**：⭐⭐⭐

**适合读者**：已经跑通 [09](09-头手躯干与底盘并行-instructional_case_04.md)，想看一条可循环的「导航 → 识别 → 抓取 → 搬运 → 放置」整链的人。

**前置知识**：组合节点、黑板（[03](03-行为树最小示例.md)）；`MultiTimedCommandMove` 多 planner 下发（[08](08-头手躯干并行-instructional_case_03.md)）；AprilTag 的基本概念（[机器人基础通识](../01-介绍/01-机器人基础通识.md)）。

**核心概念**：`grasp_mtbf` 是 MTBF（Mean Time Between Failures，平均故障间隔时间）搬箱测试场景。除一个计算关键点的场景节点外，全部使用公共节点：底盘用 `ChassisNavigationMove`，躯干与双臂统一用 `MultiTimedCommandMove`，外层用 `RepeatUntil` 控制轮数。

**学习目标**：

1. 看懂「循环装饰器 → 单轮 Sequence → 抓/放子树」三层结构；
2. 知道关键点如何由 Tag 位置、偏置和箱体尺寸算出并覆盖黑板命令；
3. 会按现场修改相机、二维码（AprilTag）和导航点，并能在导航途中加入途经点；
4. 会用干跑验证修改，并知道真机运行前还缺什么。

**由简到难进阶路线**：[09](09-头手躯干与底盘并行-instructional_case_04.md)（头、手、躯干与底盘并行）→ **本篇**（主树 17 节点 + 2 子树，可循环）。

---

## 一、场景定位

| 维度 | 值 |
|------|----|
| 场景目录 | `orchestration/scenarios/grasp_mtbf/` |
| 主树 | `py_tree.json`，根 `RepeatUntil(mtbf)`，内含单轮 `Sequence(mtbf_round)` |
| 子树 | `py_tree_child.json`，**2 棵**：`pick_one.json` / `place_one.json`，各含 4 个运动节点 |
| 黑板 | `board.json`（40 个键：轮数、导航点、Tag、箱体尺寸、八阶段命令与时间等） |
| 硬件配置 | 无 `hardware_config.json` |
| 场景节点 | `orchestration/nodes/grasp_mtbf/compute_key_points.py`（`ComputeKeyPoints`） |
| 场景说明 | [场景 readme](../../orchestration/scenarios/grasp_mtbf/readme.md)（参数与规则的权威说明） |

---

## 二、树结构

主树 `py_tree.json` 共 17 个节点，整棵树展开子树后共 **25 个节点**（含根）、19 个叶子；单轮 Sequence 有 11 个直接子节点：

```text
RepeatUntil(mtbf)                    max_iterations ← board.repeat_count
└── Sequence(mtbf_round)
    ├── MultiTimedCommandMove        move_to_initial
    ├── ChassisNavigationMove        nav_to_pick
    ├── MultiTimedCommandMove        move_to_pick_observe
    ├── Parallel(success_on_one)     pick_tag_detection
    │   ├── NodeWaitForBlackboard    wait_for_pick_tag
    │   └── NodePercep               detect_pick_tag
    ├── ComputeKeyPoints             compute_pick_key_points (phase=pick)
    ├── pick_one.json                pick_align → pick_approach → pick → pick_lift
    ├── ChassisNavigationMove        nav_to_place
    ├── Parallel(success_on_one)     place_tag_detection
    │   ├── NodeWaitForBlackboard    wait_for_place_tag
    │   └── NodePercep               detect_place_tag
    ├── ComputeKeyPoints             compute_place_key_points (phase=place)
    ├── place_one.json               place_above → place → place_release → place_retreat
    └── MultiTimedCommandMove        move_to_initial
```

每轮都会重新导航、重新检测 Tag、重新计算关键点。

---

## 三、关键机制

| 机制 | 说明 |
|------|------|
| 轮数 | `repeat_count`：`1` 单轮，`N` 为 N 轮，`0` 无限循环；只统计成功完成的整轮 |
| 停止 | 运行时黑板 `mtbf_stop_requested=true` 时在当前轮结束后停止，不是急停 |
| 失败 | 任一导航、检测、计算或运动失败，整树立即 `FAILURE`，不重试、不自动放箱 |
| 检测 | 严格模式：只接受本阶段开始后采集的新 Tag；5 秒未检测到则失败，不注入虚拟目标 |
| 关键点 | `ComputeKeyPoints` 用 Tag XYZ + 位置偏置 + 箱体尺寸计算八个阶段的双臂 XYZ 与共享躯干，保留 board 中各阶段的欧拉角 |
| 运动 | `MultiTimedCommandMove` 每次初始化重读黑板命令；发令前切换手臂外部控制模式 2，失败不下发 |
| 途经点 | 可在单轮 Sequence 中插入额外 `ChassisNavigationMove` 绕行，方法见 [5.4 节](#54-导航点与途经点) |

具体公式、阶段偏置和参数含义以[场景 readme](../../orchestration/scenarios/grasp_mtbf/readme.md) 为准。

---

## 四、运行命令

```bash
cd /path/to/LeTools
export PYTHONPATH=$(pwd):$PYTHONPATH

# 干跑：不访问硬件，使用虚拟 Tag
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/grasp_mtbf --dry-run --tick-once

# 真机：去掉 --dry-run；需先启动控制器和 TF 转发，并完成下方检查
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/grasp_mtbf
```

运行前先按 [02-执行命令 · 运行前环境准备](02-执行命令.md#运行前环境准备仿真与真机共用) 完成工作空间、`PYTHONPATH` 和 TF 转发的准备。

**仿真说明**：默认 `board.json` 等待真实 Tag（`use_virtual_tag=false`、`fallback_to_virtual=false`、`require_fresh_tag=true`），没有相机检测的 MuJoCo 环境会在检测阶段返回 `FAILURE`。另外，导航目标 `pick_point` / `place_point` 是现场标定的固定坐标。因此默认配置不能直接在仿真中跑通；仿真前需要改用虚拟 Tag，并按仿真环境调整导航点。

> ⚠️ **真机**：运行前按 [02-执行命令 · 真机运行前安全检查](02-执行命令.md#真机运行前安全检查) 逐项确认。本场景默认配置风险较高，务必先核对 `board.json`：
> - `navigation_config.avoid_enabled=false`：导航时**不避障**；
> - `navigation_config.linear_velocity=1.00`：导航线速度较高，首次运行建议先降低；
> - `pick_point` / `place_point`：为现场标定坐标，换场地必须重新标定；
> - `repeat_count=2`：默认循环两轮，首次运行建议设为 `1`。

**验证方式**：

| 层次 | 判据 |
|------|------|
| 加载 | 打印 `根节点: mtbf`，`子树集合已加载… (count=2)` |
| 单拍 tick | `repeat_count=1` 时为 `Status.SUCCESS`；大于 1 或为 0 时第一拍完成一轮后为 `Status.RUNNING`，不表示失败 |
| 回归测试 | `python3 -m pytest -p no:cacheprovider orchestration/nodes/tests/test_grasp_mtbf.py -q` |

> **真机前须知**：当前没有夹爪开闭节点，`pick` / `place_release` 只是双臂目标阶段。Tag 位姿由 `ar_control` 从相机光学坐标系转换到 `base_link` 后发布，`ComputeKeyPoints` 直接按 BASE 系使用该坐标，不再做其他变换。
> 干跑和单元测试通过不代表实机可达性、碰撞和夹持已经验证。

---

## 五、按现场修改

换场地、换箱子或换标签时，按下表找到要改的文件。`board.json` 和 `py_tree.json` 在行为树启动时加载，`config/*.yaml` 在硬件初始化时读取，**改完都要重新启动行为树进程**才生效。

| 想改什么 | 改哪里 | 详见 |
|---|---|---|
| 相机分辨率、帧率，是否启动 AprilTag 检测 | `config/camera_config.yaml` | 5.2 |
| 二维码的 ID、尺寸、家族 | `config/apriltag_config.yaml`（检测器参数与标签同在一个文件）、场景 `board.json` 与 `py_tree.json` | 5.3 |
| 抓取 / 放置目标相对二维码的位置、箱子尺寸 | 场景 `board.json`：`pick_position_offset`、`place_position_offset`、`box_size` | 5.3 |
| 抓取点、放置点、导航速度与避障 | 场景 `board.json`：`navigation_config` | 5.4 |
| 导航途中经过的途经点 | 场景 `board.json` 与 `py_tree.json` | 5.4 |
| 循环轮数 | 场景 `board.json`：`repeat_count` | 第三节 |

### 5.1 检测链路

```text
头部相机（Orbbec，config/camera_orbbec.launch 启动）
  → /camera/color/image_raw
  → AprilTag 检测（config/apriltag_continuous.launch，参数来自 config/apriltag_config.yaml）
  → ar_control：相机光学坐标系 camera_color_optical_frame → base_link
  → /robot_tag_info
  → NodePercep 写黑板 latest_tag_<id> → NodeWaitForBlackboard 写 latest_tag_<id>_xyz
  → ComputeKeyPoints 读取 latest_tag_<id>_xyz 计算八个阶段的关键点
```

以上进程都由硬件适配器在行为树启动时自动拉起，不需要手动启动。

### 5.2 相机

本场景用**头部相机**检测二维码，腕部相机不参与。相机参数在 `config/camera_config.yaml` 的 `camera` 段：

| 字段 | 当前值 | 说明 |
|---|---|---|
| `enable_head` | `true` | 必须为 `true`，否则没有图像输入 |
| `color_width` / `color_height` / `color_fps` | `1280` / `800` / `15` | 彩色图分辨率与帧率，深度图对应 `depth_*` |
| `launch_apriltag` | `true` | 必须为 `true`；为 `false` 时适配器不创建感知模块，`NodePercep` 无法取得检测结果 |
| `apriltag_topic` | `/robot_tag_info` | Tag 结果话题，一般不需要改 |
| `enable_wrist_camera` | `false` | 腕部相机总开关，本场景不需要 |

注意事项：

- **头部相机型号**：当前启动文件 `config/camera_orbbec.launch` 固定启动 Orbbec Gemini 330 系列，AprilTag 订阅 `/camera/color/image_raw`。`camera_config.yaml` 中的 `head_camera_type` 不影响这条链路；换用其他型号的头部相机时，需要同时修改该启动文件，并保证彩色图话题和光学坐标系名称与上面一致。
- **坐标系**：`ar_control` 依赖 `camera_color_optical_frame` 到 `base_link` 的 TF。相机安装位置或角度变化后，必须更新对应的 TF，否则 Tag 坐标会整体偏移，关键点随之偏移且**不会报错**。
- **分辨率**：降低分辨率可以减轻计算负担，但会降低远距离、小尺寸二维码的检出率；修改后先按 5.3 末尾的方法确认能稳定检出。

### 5.3 二维码（AprilTag）

本场景用两个二维码：**抓取 Tag** 贴在待搬的箱子上，**放置 Tag** 贴在放置位置。默认 `pick_tag_id=1`、`place_tag_id=2`。

**换成其他 ID 或尺寸的二维码**，以把抓取 Tag 从 `1` 换成 `5` 为例，按顺序完成三处修改：

1. **`config/apriltag_config.yaml`**：检测器参数与待检测标签写在同一个文件，首次使用先从模板复制：

   ```bash
   cp config/apriltag_config.yaml.template config/apriltag_config.yaml
   ```

   然后在 `standalone_tags` 中登记新标签。`size` 是**黑框外沿**的边长（米），不含白边，建议用卡尺测量：

   ```yaml
   standalone_tags:
     [
       {id: 5, size: 0.10, name: 'tag_5'},
       {id: 2, size: 0.10, name: 'tag_2'},
     ]
   ```

   尺寸写错不会报错，但检测出的距离会按比例偏差（实物 0.18 m 写成 0.09 m，距离就差一倍）。同一文件的 `tag_family` 要与打印的标签一致，模板默认 `tag16h5`，实际以现场实物为准。

2. **场景 `board.json`**：

   | 键 | 修改为 |
   |---|---|
   | `pick_tag_id` | `5` |
   | `tag_ids` | `[5, 2]`，必须同时包含抓取和放置两个 ID |

3. **场景 `py_tree.json`**：把 `compute_pick_key_points` 节点读取的黑板键从 `latest_tag_1_xyz` 改为 `latest_tag_5_xyz`：

   ```json
   "tag_xyz": {"source": "READ_BOARD", "board_key": "latest_tag_5_xyz"}
   ```

   这个键名由等待节点按 `latest_tag_<id>_xyz` 生成，**不会随 `pick_tag_id` 自动变化**。只改前两步时，干跑会以 `FAILURE` 结束，日志为 `关键点计算失败: latest_tag_1_xyz 未就绪`。放置 Tag 同理，对应 `place_tag_id` 和 `compute_place_key_points` 节点。

**二维码贴的位置与目标不重合时**，用位置偏置修正，不需要改代码：

| 键 | 当前值 | 含义 |
|---|---|---|
| `pick_position_offset` | `[0.05, 0.0, 0.0]` | 抓取目标相对抓取 Tag 的位置偏置 `[x, y, z]`（米，BASE 系） |
| `place_position_offset` | `[-0.4, 0.0, -0.4]` | 放置目标相对放置 Tag 的位置偏置 |
| `box_size` | `[0.34, 0.58, 0.26]` | 箱子沿 BASE 系的尺寸 `[x, y, z]`（米），换箱子时修改 |

关键点公式和各阶段的附加偏置见[场景 readme](../../orchestration/scenarios/grasp_mtbf/readme.md)。

**检测相关参数**：`real_tag_timeout_sec`（默认 `5.0` 秒）是每次等待新 Tag 的超时时间；检出不稳定时可适当调大，不要改用虚拟 Tag。`use_virtual_tag`、`fallback_to_virtual` 只用于干跑和仿真，真机必须为 `false`。

**确认能检出**：硬件初始化完成后（例如行为树已运行到导航阶段），在另一个终端查看检测结果：

```bash
rostopic echo -n 1 /robot_tag_info
```

输出中应包含新的 Tag ID，`frame_id` 为 `base_link`，位置数值与 Tag 相对机器人的实际距离相符。

### 5.4 导航点与途经点

#### 抓取点与放置点

导航参数集中在 `board.json` 的 `navigation_config`：

| 字段 | 当前值 | 说明 |
|---|---|---|
| `enable` | `true` | `false` 时跳过所有导航节点，原地执行动作 |
| `pick_point` / `place_point` | `[2.24, 1.377, 1.58825]` / `[3.272, 1.386, 1.58825]` | `map` 坐标系下的 `[x, y, theta]`，位置单位米，航向角单位**弧度** |
| `linear_velocity` / `angular_velocity` | `1.00` / `1.00` | 线速度（m/s）/ 角速度（rad/s），首次运行建议降低 |
| `avoid_enabled` | `false` | 是否启用底层避障 |
| `allow_rotation` | `true` | 是否允许底盘旋转 |
| `arrived_position_threshold` / `arrived_angular_threshold` | `0.03` / `0.08` | 到达判定阈值（米 / 弧度） |

`pick_point`、`place_point` 是现场地图上的坐标，换场地或重建地图后必须重新获取。获取方法（JiBot 底盘）：把机器人移到目标位置和朝向，读取定位：

```bash
rostopic echo -n 1 /move_base/amcl_pose
```

取 `pose.pose.position` 的 `x`、`y`；航向角由 `orientation` 换算，底盘只绕竖直轴旋转时 `theta = 2 * atan2(z, w)`：

```bash
python3 -c "import math; z, w = 0.7071, 0.7071; print(2 * math.atan2(z, w))"   # 把 z、w 换成实测值
```

#### 添加途经点

`pick_point` 和 `place_point` 之间需要绕开障碍物或走固定路线时，可以加入一个或多个途经点。途经点用公共节点 `ChassisNavigationMove` 实现，不需要改代码，分两步：

**第 1 步：在 `board.json` 的 `navigation_config` 中增加点位**。保留已有字段，加入新字段，格式与 `pick_point` 相同：

```json
"pick_to_place_via_1": [2.80, 1.00, 0.00],
"pick_to_place_via_2": [3.10, 1.20, 1.57]
```

字段名可以自定义，但必须与下一步节点中的 `target_key` 完全一致。上面的数值只是格式示例，必须换成用上面的方法在现场地图中实测的坐标。

**第 2 步：在 `py_tree.json` 的单轮 `Sequence(mtbf_round)` 中插入导航节点**。插在 `pick_one.json` 之后、`nav_to_place` 之前：

```text
Sequence(mtbf_round)
├── move_to_initial → nav_to_pick → move_to_pick_observe → 抓取 Tag 检测 → compute_pick_key_points → pick_one.json
├── nav_pick_to_place_via_1      ← 新增
├── nav_pick_to_place_via_2      ← 新增
├── nav_to_place → 放置 Tag 检测 → compute_place_key_points → place_one.json
└── move_to_initial
```

每个途经点节点的 JSON 如下，复制时只改 `label` 和 `target_key` 的值：

```json
{
  "name": "ChassisNavigationMove",
  "label": "nav_pick_to_place_via_1",
  "params": {
    "navigation_config": {"source": "READ_BOARD", "board_key": "navigation_config", "data_type": "json"},
    "target_key": {"source": "CUSTOM", "data_type": "string", "value": "pick_to_place_via_1"},
    "mode": {"source": "CUSTOM", "data_type": "string", "value": "map"}
  },
  "childs": [],
  "childBoard": []
}
```

说明：

- `Sequence` 会等当前途经点到达后再前往下一个点；任一途经点导航失败或超时（默认 60 秒），本轮立即失败，不会继续放置；
- 途经点沿用 `navigation_config` 中的速度和到达阈值。需要单独设置时，在该节点的 `params` 中加同名参数，节点参数优先于 `navigation_config`，例如持箱经过窄通道时减速：

  ```json
  "linear_velocity": {"source": "CUSTOM", "data_type": "float", "value": 0.3},
  "timeout": {"source": "CUSTOM", "data_type": "float", "value": 90.0}
  ```

- **回程不会自动反向经过这些点**。回程也要绕行时，另设 `place_to_pick_via_1` 等点位，把对应节点插在单轮末尾的 `move_to_initial` 之后；这样最后一轮也会经过回程途经点；
- 途经点只是人为指定的中间目标，不保证两点之间的路径没有障碍。`avoid_enabled=false` 时导航不避障；验证路线时要考虑持箱状态下的箱体、双臂外廓和转向空间，不能只看底盘中心线。

**验证**：先干跑确认树能加载，再用下面的命令按执行顺序列出所有导航节点及其坐标，确认途经点的位置和顺序正确：

```bash
S=orchestration/scenarios/grasp_mtbf python3 -c '
import json, os
d = os.environ["S"]
nav = next(i["value"] for i in json.load(open(d + "/board.json"))["process"] if i["key"] == "navigation_config")
for n in json.load(open(d + "/py_tree.json"))["tree"]["childs"][0]["childs"]:
    if n["name"] == "ChassisNavigationMove":
        k = n["params"]["target_key"]["value"]
        print(n["label"], k, nav.get(k, "<board 中缺少该点>"))
'
```

加入一个途经点后的输出示例：

```text
nav_to_pick pick_point [2.24, 1.377, 1.58825]
nav_pick_to_place_via_1 pick_to_place_via_1 [2.8, 1.0, 0.0]
nav_to_place place_point [3.272, 1.386, 1.58825]
```

出现 `<board 中缺少该点>` 说明节点的 `target_key` 与 `navigation_config` 中的字段名不一致；此时干跑也会以 `Status.FAILURE` 结束，导航节点的失败信息为 `navigation_config 缺少目标点配置`。回程途经点插在 `move_to_initial` 之后，同样会出现在列表中。

### 5.5 修改后的验证顺序

1. **干跑**：确认树能加载、Tag 键名正确（`repeat_count=1` 时预期 `Status.SUCCESS`），并用 5.4 的命令核对导航顺序与坐标；
2. **只检测**：真机上用 `rostopic echo -n 1 /robot_tag_info` 确认两个 Tag 都能稳定检出；
3. **单轮低速**：`repeat_count=1`，降低 `linear_velocity` / `angular_velocity`，有人看护急停，观察导航路线和双臂关键点是否合理；
4. 确认无误后再恢复速度和轮数。

---

## 六、下一步

- [场景 readme](../../orchestration/scenarios/grasp_mtbf/readme.md)：轮数、关键点公式与感知策略的完整说明
- [11-小件上料模仿学习-grasp_ring_pick_place](11-小件上料模仿学习-grasp_ring_pick_place.md)：用模仿学习模型完成抓取的小件上料案例
- [12-下一步](12-下一步.md)：本章回顾与后续阅读
