# 托盘位姿（AprilTag 反识别）

用场景里位置固定的 AprilTag 标定出「Tag ↔ 木托盘」的刚体变换，之后每帧由 Tag 位姿
反推木托盘位姿。

**只做两件事**：离线标定一次，运行时每帧算一次。检测、相机、TF、硬件、行为树引擎
一律复用，本模块不碰。

```text
PerceptionAdapter          拉起 apriltag_continuous.launch，订阅 /robot_tag_info
        ↓
NodePercep                 写黑板 latest_tag_<id> + latest_tag_<id>_version
        ↓
NodePalletPose  ← 本模块   读 tag 位姿 → 反推托盘位姿
        ↓
latest_pallet + _version   供抓取类节点消费
```

| 文件 | 作用 |
|------|------|
| `algorithm.py` | 纯函数：标定、反识别、时间窗。**零框架依赖**，可脱离环境回归 |
| `skill.py` | `PalletPoseSkill`，接进技能层；不接 `hardware` 参数（纯计算）|
| `orchestration/nodes/node_pallet_pose.py` | 行为树节点：读黑板 tag、写黑板托盘 |
| `apps/test_camera_internal/pallet_calibration/` | 标定工具（走真机相机 + tkinter 点角点）|
| `apps/test_kuavo_5w_skills/test_pallet_pose.py` | 算法层回归 + 技能层单测 |
| `orchestration/nodes/tests/test_node_pallet_pose.py` | 节点单测（进 CI）|

---

# 1. 接口

## 输入

| 键 | 类型 | 谁写的 |
|---|---|---|
| `latest_tag_<id>` | `TagDetection` | `NodePercep` |
| `latest_tag_<id>_version` | `int` | `NodePercep` |

`<id>` 是场景里配置的 `tag_ids`。**tag 版本号没变时不重复计算。**

## 输出

| 键 | 类型 |
|---|---|
| `latest_pallet`（键名由 `key` 参数定）| **`core.domain.pose.Pose6D`** |
| `latest_pallet_version` | `int`，每算一次自增 |

`Pose6D(x, y, z, yaw, pitch, roll)`：位置**米**、姿态**弧度**。

> ⚠️ **`Pose6D` 不带 `frame_id`** —— 进来的系就是出去的系（见 §6 第 3 条）。
> **下游拿到的位姿在哪个坐标系里，是调用方自己的约定。**

节点**构造时就预置初值**：`__init__` 里写 `latest_pallet = None` / `_version = 0`
（`register_key` 在 py_trees 2.x 里只注册权限、不创建值，不预置的话消费者
`getattr` 会 `KeyError`）。`__init__` **只在建树时执行一次**，不是每帧。

节点持续返回 `RUNNING`（与 `NodePercep` 同构，由 `Parallel` 父节点决定何时收）。

## 节点参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `tag_ids` | `[]` | 参与反识别的 tag id 列表，如 `[0, 1]` |
| `key` | `latest_pallet` | 写黑板的键名 |
| `window` | `0` | 时间窗宽度，`0` = 不平滑 |
| `config_path` | `config/pallet_tag.yaml` | 标定结果路径 |

标定结果**从 yaml 读，不写进场景 JSON** —— 矩阵太长、太容易改错，而且标定结果
属于现场数据，不该跟场景定义混在一起。

# 2. 标定一次

```bash
python3 apps/test_camera_internal/pallet_calibration/pallet_calibrate.py --help
```

采 **5 帧**，每帧点一次托盘台面四角，求解后写 `config/pallet_tag.yaml`。
`--dry-run` 可以只求解、打印自检、不落盘。

**标定期间 tag 板和木托盘都不能动**（相机可以动）。标定一次即可；**任一被移动都要重标**。

> ⚠️ **点击顺序必须固定**，工具会用原始点击顺序建系。
> **仓库里 `pick_pallet_corners.py:14` 那句「点击顺序无所谓」是错的**
> （`order_quad()` 全仓库零调用点）—— 详见
> `pallet_servo/NOTES.md` §5。

# 3. 运行前提（**不满足会直接报错**）

1. **`config/camera_config.yaml` 里 `launch_apriltag` 当前是 `false`。**
   此时 `lifecycle_mixin` **根本不构造** `PerceptionAdapter`
   （`adapters/hardware/leju_wheeled/mixins/lifecycle_mixin.py:135`），
   `hardware.perception` 是 `None`，`NodePercep` 会 `AttributeError`。
   要真跑必须改成 `true`。
   （本仓库没有替你改，因为那会影响现场部署行为。）
2. **`config/apriltag_tags.yaml` 里要有你要用的 tag id 和真实尺寸**。
   现有条目是 id 1–4 / size 0.06，**和 maduo 用的标签不同**。
   检测器本身支持逐 tag 尺寸。
3. **标定要有图形界面**（点角点用 tkinter）。

# 4. 快速测试

```bash
# 算法层回归 + 技能层单测（无需硬件/ROS）
python3 apps/test_kuavo_5w_skills/test_pallet_pose.py

# 节点单测（CI 会跑这条）
pytest orchestration/nodes/tests/test_node_pallet_pose.py -m unit -v
```

算法层回归拿的是 maduo 项目导出的**同一份输入**重算、和那边的**同一份输出**逐位
比对 —— 算法是从那边搬过来的，这是唯一能证明没搬错的手段。fixture 由
`maduo/export_regression_fixture.py` 生成。

**在场景里干跑一次**（验证能被工厂按类名解析、黑板键注册成功，不碰硬件）：

```bash
python3 apps/test_upper_init/run_behavior_tree_json.py \
    --scenario orchestration/scenarios/<场景目录> --dry-run --tick-once
```

# 5. 接进行为树

场景 JSON 里加一个节点（`"name"` 是**节点类名**，不是技能名）：

```json
{ "name": "NodePalletPose",
  "label": "pallet_pose",
  "params": {
    "tag_ids": { "value": [0, 1], "source": "CUSTOM", "data_type": "intArr" },
    "key":     { "value": "latest_pallet", "source": "CUSTOM", "data_type": "string" }
  },
  "childs": [], "childBoard": [] }
```

把它和 `NodePercep` 放在**同一个 `Parallel`** 里（`NodePercep` 提供 tag，本节点消费），
产出的 `latest_pallet` 就是一个 `Pose6D`。

# 6. 四个坑

1. **变换顺序。** `T_pallet_tag` 的方向是 `pallet ← tag`，运行时
   `T_sensor_pallet = T_sensor_tag @ inv(T_pallet_tag)`。框架
   `core/common/transform.py` 的 `transform_pose(pose, M)` 等于 `M @ pose`，
   **乘法顺序相反**，直接套用会静默给出转置的结果。本模块内部全走矩阵，不经过它。

2. **`T_pallet_tag` 与坐标系无关。** 它是 tag 和托盘两个物体之间的相对位姿，
   `inv(T_sensor_pallet) @ T_sensor_tag` 里的传感系被约掉了。所以用相机系标定出来的
   矩阵，运行时可以原样吃 base_link 系的 `pose_in_world`，**不需要重标**。
   唯一要求是被相除的两项在同一个系里。

3. **`pose_in_world` 的参考系在现有代码里是自相矛盾的**：`ar_control_node` 硬编码
   `--target_frame base_link`，但 `node_compute_pick_goal` 等消费者把它当 odom 用还再
   施加一次 `odom→base_link`。**本模块对这条混乱免疫**（进来的系就是出去的系），
   标定工具也按 `TagDetection.frame_id` 读而不写死。**别人接这套键时需自己留意。**

4. **欧拉角的注释自相矛盾（但数学是对的）。** `core/domain/pose.py` 的文档说顺序是
   yaw-pitch-roll (ZYX)，`transform.py` 写的是 `from_euler('xyz', [roll, pitch, yaw])`，
   两者等价。本模块内部一律用 4×4 矩阵，只在接口边界转 `Pose6D`，把歧义关在门外。

# 7. 已知边界

- **标定自检不是绝对精度。** 它给的是重复性和跨 tag 一致性，都只说明同一次标定内部
  自洽不自洽。绝对精度需要真值，这套方案给不出。
- **时间窗压不掉慢变误差。** 这套系统里占大头的是随视角缓慢变化的系统误差，任何时间
  窗都动不了它；时间窗只能压掉 fast 抖动。用 `window=0` 就没有这个问题也没有收益。
- **`T_pallet_tag` 是场景标定**，只在这个 tag 板 + 这个托盘摆位的组合下成立。
- **生产路径上 `self_check["n_frames"]` 其实是 tag 数、不是帧数**：`TagObs.frame` 在
  `pallet_calibrate.py` 里传的是 tag id、在测试里传的是帧名，于是 `by_frame` 按 tag
  分组、`n_frames` = tag 数（2）而不是帧数（5）。测试里因为传的是帧名，这个键是对的
  —— **所以测试永远发现不了。**
- ✅ **某个 tag id 没被 `NodePercep` 写过时，不再把整棵树打死**（2026-09-21 修）。

  **曾经的症状**：`node_pallet_pose.py` 用的是
  `getattr(self.global_blackboard, f"latest_tag_{tag_id}", None)`，而 py_trees 的
  `Client.__getattr__` 在**键注册了但从未被写过**时抛的是 **`KeyError`**，
  `getattr` 的默认值**接不住**（实测报错原文：`KeyError "client '...' tried to
  access '/latest_tag_<id>' but it does not yet exist on the blackboard"`）。
  而 py_trees **不接** `update()` 抛出的异常，于是**整棵树连每帧日志一起没**。

  **触发条件**（两个都会真发生）：① 某个 tag 在 `config/pallet_tag.yaml` 里标定过、
  也配进了本节点的 `tag_ids`，但**不在 `NodePercep` 的 `tag_ids` 里**（`NodePercep`
  只为自己那份列表预置初值）；② 树上压根没有写那些键的节点。

  **现在的行为**：照抄 `node_pallet_servo._read_blackboard()` 的写法接住
  `KeyError` 返回默认值（**只接 `KeyError`**，`AttributeError` 意味着键没注册、
  是本文件自己的 bug，照旧响亮抛出）。缺失的 id 会被**跳过一个、照常用其余能用的**，
  并且：

  - 每条缺失 id 记一条**一次性 WARNING**（按 id 去重，不是每 tick 一条）；
  - `feedback_message` 从"等 tag"改成点名"**id N 的 `latest_tag_<id>` 从未被写过
    （上游 tag_ids 没覆盖）**"——两者在界面上必须分得开，后者永远不会好。

  回归测试在 `orchestration/nodes/tests/test_node_pallet_pose.py`：
  一条墓碑（断言裸 `getattr` 必然抛 `KeyError`）+ 两条行为用例。
  另外那里也把 `latest_tag_<id>_version` 的 `int()` 换成了不抛的读法
  （上游写个字符串版本号，旧代码同样抛穿 `update()`）。

# 8. 下游：托盘伺服误差（`pallet_servo`）

`latest_pallet` 的下游是 `skills/atomic/perception/pallet_servo/`
（把托盘位姿与箱子观测变成图像空间的三个量），**不改本模块一行代码**。

真机接上之前有两件事**必须先定**，都是本模块的既有问题：

1. **真机路径的托盘系是左手系**（烘进 `T_pallet_tag` 的每帧 `det = −1`），而
   `matrix_to_pose6d()` 会把镜面**静默投影掉** —— `NodePalletPose` 写在黑板上的
   `Pose6D` **还原不回标定出来的那个系**。今天无害（没有别的消费者），接伺服就会
   吃到被镜像的位姿。
2. **两条路径的 `x=0` / `x=W` 是反的**：离线点点的参考边写 `["y=0","x=W"]`，
   tag 路径要写 `["y=0","x=0"]` —— 写错了不报错，只是伺服错了边。

**详见 `pallet_servo/NOTES.md` §6**（含三选一的处置方案）。
