# grasp_mtbf — 可配置轮数的搬箱流程

主树用公共 `RepeatUntil` 包装完整单轮流程，当前 [board.json](board.json) 配置 `repeat_count=2`，执行 **2 轮**。将 `repeat_count` 改为需要的轮数：`1` 为单轮，`0` 表示无限循环。主树为 [py_tree.json](py_tree.json)，抓取与放置动作由场景级 [py_tree_child.json](py_tree_child.json) 中的 `pick_one.json` / `place_one.json` 执行。

## 循环配置

- `repeat_count`：非负整数，当前配置 `2`；只计完整单轮子树返回 `SUCCESS` 的次数，不按 tick 或单个动作计数。通过 `max_iterations` 传给装饰器，构树时读取，修改后需重新加载树。
- `mtbf_stop_requested`：默认 `false`；运行时在同一进程的黑板中设为 `true` 请求结束。第一轮前或两轮之间请求停止时不启动下一轮；轮内请求停止则等待本轮结束（`wait_for_child_completion=true`、`check_before_iteration=true`）。仅修改磁盘 board 不会更新已运行的黑板；当前不新增外部停止接口。
- 达到轮数上限或停止条件满足时根返回 `SUCCESS`；任一导航、检测、计算或动作失败则立即返回 `FAILURE`，不自动重试或开始下一轮，失败轮不计数。
- 单轮末尾恢复 `initial` 后才计数；下一轮从 `initial → nav_to_pick` 开始，因此会从放置位置重新导航到抓取位置。每轮重新检测两个 Tag、计算关键点，严格新帧校验仍生效。
- 请求按轮停止不是急停：当前轮仍可能继续导航和动作；失败退出也不自动放箱或恢复。

## 执行顺序

```text
initial → nav_to_pick → pick_observe
  → 抓取 Tag 检测 → ComputeKeyPoints(pick)
  → pick_align → pick_approach → pick → pick_lift
  → nav_to_place
  → 放置 Tag 检测 → ComputeKeyPoints(place)
  → place_above → place → place_release → place_retreat
  → initial
```

当前主树和抓取/放置子树已移除 Enter 断点，阶段成功后自动进入下一阶段。

当前主树只有 `nav_to_pick` 和 `nav_to_place` 两个导航节点；每轮末尾恢复 `initial`，若需继续，下一轮的 `nav_to_pick` 返回抓取点。最终停止时不额外导航回抓取点。board 中的 `place_observe` 配置仍保留，但当前主树没有执行这个动作。

所有躯干和双臂动作统一使用公共 `MultiTimedCommandMove`。**当前没有夹爪开闭节点**；`pick` / `place_release` 仅表示双臂目标阶段，不代表实际夹紧或松开夹爪。

## 修改相机、二维码与导航途经点

按现场修改相机参数、换用其他 ID 或尺寸的二维码、在导航途中加入途经点的完整步骤，见文档
[10-搬箱场景-grasp_mtbf · 五、按现场修改](../../../docs/04-简单案例运行/10-搬箱场景-grasp_mtbf.md#五按现场修改)。

## 关键点规则

场景节点 `ComputeKeyPoints` 输入：

- `phase`：`pick` 或 `place`。
- `tag_xyz`：当前 Tag 的 `[x,y,z]`，运行时从 `latest_tag_1_xyz` / `latest_tag_2_xyz` 读取。
- `position_offset`：实际目标相对 Tag 的位置偏置，单位米。
- `box_size`：箱子沿 BASE 轴的 `[x,y,z]` 尺寸，单位米；当前公式使用 X/Y 尺寸，不使用箱体 Z 尺寸叠加抓取高度。

令 `target = tag_xyz + position_offset`，阶段偏置为 `(dx, dy, dz)`：

```text
末端 X = target.x + box_size.x / 2 + dx
左臂 Y = target.y + (box_size.y / 2 + dy)
右臂 Y = target.y - (box_size.y / 2 + dy)
末端 Z = target.z + dz
```

阶段偏置集中定义在 [compute_key_points.py](../../nodes/grasp_mtbf/compute_key_points.py)：

| 阶段 | dx | dy | dz | 含义 |
| --- | ---: | ---: | ---: | --- |
| `pick_align` | -0.15 | 0.15 | 0.00 | 双手张开，在目标后方对齐 |
| `pick_approach` | 0.00 | 0.15 | 0.00 | 保持张开，向前接近 |
| `pick` | 0.00 | 0.00 | 0.00 | 双手进入箱体两侧接触位置 |
| `pick_lift` | -0.15 | 0.00 | 0.15 | 保持间距，抬升并后撤 |
| `place_above` | -0.15 | 0.00 | 0.15 | 到放置位置斜上方 |
| `place` | 0.00 | 0.00 | 0.00 | 前进并下降至放置位置 |
| `place_release` | 0.00 | 0.15 | 0.00 | 双手向两侧打开 |
| `place_retreat` | -0.15 | 0.15 | 0.00 | 保持张开并后撤 |

### 躯干

抓取四阶段统一依据 **`pick` 末端**，放置四阶段统一依据 **`place` 末端**计算。各自计算节点运行时计算一次，各阶段复用同一组躯干命令：

```text
torso_x = 0.12 + 0.5 * (末端 X - 0.6)
torso_z = 1.0 + 0.5 * (末端 Z - 1.0)
yaw = pitch = 0
```

不依据 `pick_align`、`pick_approach`、`pick_lift` 或 `place_above` 的附加偏置改变躯干。

### 欧拉角与时间

[board.json](board.json) 包含八组动作初值和各自的 `<stage>_duration`。初值供构树使用，计算节点运行后覆盖 planner 2 的躯干目标及 planner 6/7 的双臂 XYZ，**保留每组原命令各自的左右臂欧拉角**。

- planner 2：`[x,z,yaw_deg,pitch_deg]`。
- planner 6/7：`[x,y,z,yaw_deg,pitch_deg,roll_deg]`，角度单位为度。
- `MultiTimedCommandMove` 每次动作初始化重新读取黑板命令，避免使用构树时的旧 XYZ。
- 各阶段默认时间为 3 秒，可分别修改对应 duration。
- `focus_ee` 控制是否专注末端解算，`focus_z` 内部保持与其一致。
- 公共运动节点默认 `prepare_arm_control=true`，本场景无需增加前置节点或修改 board：每组躯干/手臂动作发令前调用 `set_arm_control_mode(2)` 并检查切换结果，失败则返回 `FAILURE`，不下发该组运动、不执行后续树动作。模式 0 是保持当前位置，焦点参数不能代替模式准备。
- 模式准备不查询当前模式，不执行模式 1 的物理复位，动作结束后也不自动切回模式 0。连续动作和导航期间是否释放控制由场景安排；当前树未新增释放动作。若外部统一管理模式，可给运动节点设置 `prepare_arm_control=false`；dry-run 不执行模式切换。
- `is_sync` 控制同组 planner 是否同步完成；`motion_settle_time` 为额外稳定等待时间。

## 导航与感知开关

- `navigation_config.enable=false`：跳过导航，原地执行动作。
- `navigation_config.pick_point` / `place_point`：`[x,y,theta]`，位置单位米，航向角单位弧度；当前节点使用 `mode=map`。
- `use_virtual_tag=true`：跳过相机检测并注入 board 中的虚拟 Tag。
- `use_virtual_tag=false`：等待真实检测；超时策略由 `fallback_to_virtual` 控制。
- `fallback_to_virtual=false`（当前配置）：超过 `real_tag_timeout_sec=5.0` 后不注入虚拟目标，返回 `FAILURE`，不再执行该阶段的关键点计算和抓放动作。
- `fallback_to_virtual=true`：超时注入虚拟 Tag 并返回 `SUCCESS`，仅用于仿真；公共等待节点默认为 false。
- `require_fresh_tag=true`（当前配置）：进入等待时记录版本和采集时间边界，只有版本更新且采集时间不早于本阶段开始的有效 Tag 才能成功。旧缓存即使被感知节点反复写入也不会放行；公共节点默认 false。
- 检测 Parallel 内先 tick 等待节点、再 tick 感知节点，避免记录更新边界时错过首帧。新帧在下一次 tick 被等待节点接受。
- 检测时间戳使用 ROS 时钟，超时使用单调时钟；严格模式下零值、无效或未来时间戳不被接受。
- dry-run：不访问真实硬件，立即使用虚拟 Tag，不受 `fallback_to_virtual=false` 限制；显式 `use_virtual_tag=true` 同样保留虚拟模式。

**Tag 位姿由 `ar_control` 从相机光学坐标系转换到 `base_link` 后发布；`ComputeKeyPoints` 直接按 BASE 系使用 Tag XYZ 与位置偏置，不再做其他变换。相机安装位置变化后必须更新相机到 `base_link` 的 TF。**

`NodePercep` 未添加相机故障降级：感知适配器缺失或检测接口抛异常仍会报错，不会被转为虚拟目标。检测超时失败仅停止后续行为树动作，不自动执行放箱、回初始或其他恢复动作。

关闭导航或使用虚拟 Tag 不会在 dry-run 之外禁用躯干/双臂运动；真实硬件执行前仍须检查可达性、碰撞、夹持方案及超时虚拟值的安全性。

## 离线验证与运行

整树 dry-run：

```bash
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/grasp_mtbf --dry-run --tick-once
```

该入口的 `--tick-once` 只 tick 一次；`repeat_count=1` 时整轮 dry-run 返回 `SUCCESS`，当前配置 `repeat_count=2` 时第一拍完成一轮后返回 `RUNNING`，不表示失败。其他大于 1 或为 0 的配置同样返回 `RUNNING`（停止条件未满足时）。多轮执行由下面的回归测试驱动重复 tick 验证。

回归测试：

```bash
python3 -m pytest -p no:cacheprovider \
  orchestration/nodes/tests/test_repeat_until.py \
  orchestration/nodes/tests/test_grasp_mtbf.py -q
```

测试覆盖八组 XYZ、左右臂对称关系、Pick/Place 各自共享躯干、独立 Euler 保留、运行时 Tag 更新、非法尺寸、单轮/三轮完整动作顺序、命令读取、时间绑定、每轮拒绝旧 Tag 缓存并重新采集、停止请求的轮边界、第二轮运动失败及检测超时失败时的动作阻断。

实机入口为同一启动命令去掉 `--dry-run --tick-once`，需要已配置的 ROS/SDK 和硬件环境。离线验证通过不代表实机动作或真实感知链路已经验证。
