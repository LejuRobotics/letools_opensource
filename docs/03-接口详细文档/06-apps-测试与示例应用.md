# 06 apps 测试与示例应用

本文中的相对路径（如 `apps/`、`orchestration/scenarios/`）
一律相对 **LeTools 仓库根目录**；各命令需在仓库根目录下执行。

> `apps/` 是**真机 / 仿真环境下的验证脚本目录**，不是 pytest 单测目录。它按「测试层级」组织，每层用一个脚手架模块统一前置/后置，便于逐层定位问题到底出在 ROS、适配器、SDK 还是封装。

## 06.01 命名空间与类索引

| 点分路径 | 层级 | 接口方式 | 目的 |
|---|---|---|---|
| `apps.test_kuavo_5w_adapter` | **标准接口层** | `LejuWheeledArmHardware` **标准接口**（无后缀） | 适配器层方法验证 |
| `apps.test_kuavo_5w_sdk_adapter` | **SDK 路径层** | `HardwareFactory` → `_sdk` / `_timed` 后缀方法 | Factory 封装验证 |
| `apps.test_kuavo_5w_skills` | 技能层 | 构造 `XxxSkill(hardware=...)`，走技能生命周期 | Skill 封装验证 |
| `apps.test_kuavo_5w_orchestration` | 编排层 | 构造节点 `XxxMove(name, label, ns, params)`，硬件走单例 | Node 包装验证 |
| `apps.test_camera_adapter` | 相机 | `CameraAdapter` / `PerceptionAdapter` | 相机与感知适配器 |
| `apps.jibot_adapter` | 底盘 | `LejuWheeledArmHardware._jibot` 方法 | JiBot 协议适配 |
| `apps.jiateng_adapter` | 底盘 | 嘉腾底盘方法 | 嘉腾协议适配 |
| `apps.test_upper_init` | 编排入口 | 行为树启动器 | 加载场景 JSON 并运行 |
| `apps.test_demo` | 示例 | 场景演示 | `pickbox` / `smt_5w` |

### 与 pytest 的关系

`pytest.ini` 的 `testpaths` 只包含 `tests`、`core/common/tests`、`adapters/hardware/leju_wheeled/services/sdk_manager/tests`、`adapters/hardware/leju_wheeled/tests` —— **`apps/` 不在其中**。

因此：

- `apps/` 下的脚本是**需要 ROS / 仿真 / 真机的手工验证脚本**，通常直接 `python3 <脚本>` 运行；
- `pytest` 命令跑的是框架自身的单元测试（标记 `unit` / `integration` / `slow`），与 `apps/` 是两套体系。

### 脚本量（不含 `__init__.py` 与 `_scaffold.py`）

| 目录 | 脚本数 |
|---|---:|
| `test_kuavo_5w_sdk_adapter` | 39 |
| `test_kuavo_5w_adapter` | 28 |
| `test_kuavo_5w_skills` | 26 |
| `test_kuavo_5w_orchestration` | 24 |
| `test_camera_adapter` | 15 |
| `test_demo` | 13 |
| `jiateng_adapter` | 3 |
| `test_upper_init` | 4 |

## 06.02 类与方法详表

### 各层脚手架模块

每层一个 `_scaffold.py`，提供「建硬件 → 前置 → 跑 → 后置」的统一函数。

| 路径 | API | 签名 | 说明 | 所属层 |
|---|---|---|---|---|
| Core | `adapter_setup` | `adapter_setup(hardware, need_arm=False, mpc_mode=None, need_torso_reset=True, need_focus_ee=False)` | 标准接口层前置：躯干复位、（可选）MPC 模式与末端焦点 | `apps.test_kuavo_5w_adapter._scaffold` |
| Core | `adapter_teardown` | `adapter_teardown(hardware, need_arm=False, restore_mpc=True)` | 标准接口层后置：`arm_reset()`，可选恢复 MPC 为 `NoControl` | 同上 |
| 查询 | `check_services_available` | `check_services_available(service_names, timeout=3.0) -> bool` | 在 `setUpClass` 里提前探测 ROS 服务，避免后续逐个超时 | 同上 |
| Core | `factory_setup` | `factory_setup(hardware, need_arm_reset=False, need_torso_reset=True, focus_ee=None, focus_z=None, mpc_mode=None)` | SDK 路径层前置 | `apps.test_kuavo_5w_sdk_adapter._scaffold` |
| Core | `factory_teardown` | `factory_teardown(hardware, need_arm_reset=False, need_torso_reset=False)` | SDK 路径层后置 | 同上 |
| Core | `build_hardware` | `build_hardware(whitelist=None, skip_end_effector=True, skip_camera=True, skip_state_manager=True, skip_force_publishers=True)` | 技能层建 `IHardware`（默认只起被测子系统） | `apps.test_kuavo_5w_skills._scaffold` |
| Core | `skill_setup` | `skill_setup(hardware, need_arm=False, need_torso_reset=True)` | 技能层前置（躯干复位 → 手臂 `set_arm_control_mode(1)` 重置 → `set_arm_control_mode(2)` 切外部控制） | 同上 |
| Core | `skill_teardown` | `skill_teardown(hardware, need_arm=False)` | 技能层后置 | 同上 |
| Core | `run_skill` | `run_skill(skill: SkillBase, params: SkillParams, tick_interval=0.05) -> bool` | 跑完整技能生命周期，返回是否成功 | 同上 |
| Core | `set_hardware_config` | `set_hardware_config(whitelist=None, skip_end_effector=True, skip_camera=True, skip_state_manager=True, skip_force_publishers=True) -> None` | 在节点首次取硬件**之前**覆盖单例配置 | `apps.test_kuavo_5w_orchestration._scaffold` |
| Core | `node_setup` / `node_teardown` | `node_setup(need_arm=False, need_torso_reset=True) -> None` | 编排层前置 / 后置（作用于共享单例） | 同上 |
| Core | `run_node` | `run_node(node, tick_interval=0.05, max_ticks=2000) -> bool` | `initialise()` 后循环 `update()` 直到 SUCCESS/FAILURE | 同上 |
| Core | `check_jibot_services` / `jibot_setup` | `check_jibot_services(timeout=5.0)` | JiBot 服务探测与前置 | `apps.jibot_adapter._scaffold` |

### 测试脚本分布

| 目录 | 二级子目录 | 覆盖内容 |
|---|---|---|
| `test_kuavo_5w_adapter`（标准接口层） | `01_base_control`(4) `02_lower_body`(2) `03_arm_control`(4) `04_timed_commands`(10) `05_force_control`(1) `06_services`(3) `07_debug_feedback`(3) `08_end_effector`(1) | 标准接口全量；`04_timed_commands` 用了 `_timed` 方法属**架构违规**（保留不动，由 T3+T4 覆盖） |
| `test_kuavo_5w_sdk_adapter`（SDK 路径层） | 按 `_sdk` / `_timed` 方法族分组 | `hw.send_*_sdk()`、`hw.send_*_timed()` |
| `test_kuavo_5w_skills` | 硬件技能 / 定时指令类 / SDK 轨迹类 / 末端位姿类 | `test_arm_reset.py`、`test_head_control.py`、`test_base_pose_local.py`、`test_leg_joint.py`、`test_leg_joint_timed.py`、`test_wait_seconds.py`、`test_arm_ee_*.py`、`test_palletizing_line_mpc.py`、`test_palletizing_line_mpc_restore.py` |
| `test_kuavo_5w_orchestration` | 与 skills 目录一一对应的 `*_move.py` | `test_arm_reset_sdk_move.py`、`test_head_control_sdk_move.py`、`test_wait_seconds.py` 等 |
| `test_camera_adapter` | 按功能拆分 | `test_camera_init/frame/depth/pointcloud/status/shutdown/tf/rviz.py`、`test_camera_carton.py`、`test_perception_*.py`、`e2e_camera_perception.py` |
| `jibot_adapter` | — | `test_base_move.py`、`test_move_to_target.py`、`test_check_arrived.py`、`test_enable_vel_control.py` |
| `jiateng_adapter` | — | 嘉腾同名四项 |
| `test_upper_init` | — | `run_behavior_tree_json.py`（行为树通用启动器） |

### 两层适配器测试的职责边界

| | 标准接口层（`test_kuavo_5w_adapter`） | SDK 路径层（`test_kuavo_5w_sdk_adapter`） |
|---|---|---|
| 调什么 | `hw.send_base_velocity()` 等**无后缀**方法 | `hw.send_*_sdk()` / `hw.send_*_timed()` |
| 底层走 | Mixin 直发 ROS | Mixin → SDKManager → SDK |
| 失败时先怀疑 | 适配器或 Mixin 封装 | Adapter 调用的 SDK API 或 SDK 版本 |

**排查顺序**：先用 `rostopic list` / `rosservice list` 确认 ROS 与控制器正常；标准接口层失败 → 查 `LejuWheeledArmHardware` 或对应 Mixin；SDK 路径层失败 → 查 SDK 是否按锁定版本安装及 SDK 管理器初始化；手臂类失败 → 重点查手臂控制模式与外部控制器切换；命令成功但机器人不动 → 查状态反馈、`reach_time`、MPC 模式反馈与 ROS 日志。

## 06.03 参数说明

### 脚手架参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `need_arm` / `need_arm_reset` | `False` | 是否做手臂重置 + 切外部控制模式 |
| `need_torso_reset` | `True` | 是否 `reset_torso_to_initial()`（内部会 `sleep 2.0`） |
| `mpc_mode` | `None` | 传入 `MPCControlMode` 会提交模式值；`None` 不调用。当前固件不按该值选择控制分支，见 [`03-adapters-硬件适配层.md`](./03-adapters-硬件适配层.md) |
| `need_focus_ee` / `focus_ee` / `focus_z` | 见签名 | 是否设置笛卡尔 / Z 轴跟踪焦点 |
| `restore_mpc` | `True` | 标准接口层后置是否把 MPC 恢复为 `NoControl` |
| `whitelist` | `None` | `sdk_managers_whitelist`，如 `['arm']` / `['low']` / `['timed']`；`None` 表示不限制 |
| `skip_end_effector` / `skip_camera` / `skip_state_manager` / `skip_force_publishers` | 均 `True` | 精简开关，只起被测技能所需子系统 |
| `tick_interval` | `0.05` | 技能 / 节点循环间隔（秒），即 20Hz |
| `max_ticks` | `2000` | `run_node` 的最大 tick 数 |
| `timeout`（`check_services_available`） | `3.0` | 每个 ROS 服务的等待超时 |

### 运行方式

```bash
# 单个脚本（真机 / 仿真）
python3 apps/test_kuavo_5w_adapter/01_base_control/test_cmd_vel_base.py

# 模块批量
python3 -m unittest discover -s apps/test_kuavo_5w_adapter/01_base_control -p "test_*.py"

# 批量脚本（自带 roscore 检查与 source）
./apps/test_kuavo_5w_adapter/run_test.sh                 # 全部模块
./apps/test_kuavo_5w_adapter/run_test.sh 01_base_control # 指定模块
./apps/test_kuavo_5w_adapter/run_test.sh 01_base_control/test_cmd_vel_base.py  # 单个
```

### 相机脚本的四个开关

| 参数 | 作用 |
|---|---|
| `--keep-alive` | `test_camera_init.py` 启动相机后保持运行，供其它脚本 `--reuse` 复用 |
| `--reuse` | 复用已在运行的相机，跳过 3s launch 等待 |
| `--rviz` | 同时打开 rviz（`biped_s4_head.rviz`） |
| `--config` | 指定 `camera_config.yaml` |

```bash
# 终端 1：起相机并常驻
python3 apps/test_camera_adapter/test_camera_init.py --keep-alive
# 终端 2：反复跑各单项
python3 apps/test_camera_adapter/test_camera_frame.py --reuse
python3 apps/test_camera_adapter/test_camera_depth.py --reuse
```

这 15 个脚本逐个测什么、前置条件、配置字段与踩过的坑，见 [`API参考/03-相机适配器测试脚本.md`](./API参考/03-相机适配器测试脚本.md)。

### 推荐的新手验证顺序（标准接口层）

不要一上来全量跑，按风险从低到高：无 ROS 预检 → 底盘（`send_base_velocity` / `send_base_pose`）→ 模式与服务检查（`set_mpc_mode` 只验证兼容服务调用，`enable_quick_mode` 需单独验证控制效果）→ 躯干与腿部 → 手臂 → 状态反馈（`07_debug_feedback`）。

## 06.04 返回值

- **`run_skill` / `run_node` 返回 `bool`**：`True` 当且仅当技能 `is_finished()` 且末次 `execute()` 成功 / 节点 `update()` 返回 `SUCCESS`。
- **测试脚本用 `unittest.TestCase`**，成败以断言为准；断言失败信息里带上 `result.message`：

```python
result = self.hardware.send_base_velocity(vx=0.3, vy=0.0, vyaw=0.0)
self.assertTrue(result.success, f"失败: {result.message}")
```

- **`run_test.sh` 的退出码**：单文件模式下断言失败即非 0（脚本 `exit 1`）；模块/全量模式下单个脚本失败**只打印 `❌ 测试失败`，不中断后续**。
- **`test_upper_init/run_behavior_tree_json.py`**：非 dry-run 运行时，根节点终态映射为进程退出码 `SUCCESS=0` / `FAILURE=1` / 其它 `=2`；dry-run 时退出码只反映加载是否成功，tick 结果看 `tick 后根状态:` 一行。

## 06.05 异常

- **前置阶段失败不抛异常，只打警告**：脚手架的躯干复位、手臂模式切换失败时用 `logger.warning` 继续，脚本仍会往下跑。因此看到 `⚠ 手臂复位警告` 不代表脚本失败，要结合后续断言判断。
- **`check_services_available` 返回 `False` 而不是抛异常**，用于在 `setUpClass` 里提前短路。
- **`SkillBase.execute()` 会把技能内异常吞成 `Result.fail(str(e))`**，所以在 `apps` 里跑技能时，失败要看 ERROR 日志里的 traceback。
- 脚本自身的**环境依赖错误**（未 source ROS、未设 `PYTHONPATH`、`ROBOT_VERSION` 未设）会以 Python 异常形式直接抛出。

## 06.06 调用示例

### 标准接口层：适配器标准接口

```python
import unittest
from adapters.hardware.leju_wheeled.hardware import LejuWheeledArmHardware
from apps.test_kuavo_5w_adapter._scaffold import adapter_setup, adapter_teardown

class TestExample(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.hardware = LejuWheeledArmHardware()
        cls.hardware.initialize()
        adapter_setup(cls.hardware, need_arm=True)

    @classmethod
    def tearDownClass(cls):
        adapter_teardown(cls.hardware, need_arm=True)
        cls.hardware.shutdown()

    def test_01_base_velocity(self):
        result = self.hardware.send_base_velocity(vx=0.3, vy=0.0, vyaw=0.0)
        self.assertTrue(result.success, f"失败: {result.message}")
```

### 技能层

```python
from apps.test_kuavo_5w_skills._scaffold import build_hardware, skill_setup, run_skill
from skills.atomic.refactored_sdk.head_control_sdk import (
    HeadControlSdkParams, HeadControlSdkSkill,
)

hw = build_hardware(whitelist=['low'])
try:
    result = hw.initialize()
    if not result.success:
        raise RuntimeError(f"硬件初始化失败: {result.message}")
    skill_setup(hw, need_arm=False)

    skill = HeadControlSdkSkill(hardware=hw)
    ok = run_skill(skill, HeadControlSdkParams(yaw_deg=20.0, pitch_deg=0.0, timeout=10.0))
    print("成功" if ok else "失败")
finally:
    hw.shutdown()
```

`build_hardware` 只创建硬件实例，支持 `whitelist` 和 `skip_*` 参数，不接收 `need_arm`；调用 `skill_setup` 前须先完成 `initialize()`。`need_arm` 是 `skill_setup` 的参数，用于决定是否执行手臂重置。

### 编排层

```python
from orchestration.nodes.head_control_sdk_move import HeadControlSdkMove
from apps.test_kuavo_5w_orchestration._scaffold import (
    set_hardware_config, node_setup, run_node,
)

set_hardware_config(whitelist=['low'])     # 必须在节点首次取硬件之前
node_setup(need_arm=False)

node = HeadControlSdkMove("head", "look_left", "", {"yaw_deg": 20.0, "pitch_deg": 0.0})
print("成功" if run_node(node) else "失败")
```

### 行为树启动器

```bash
# 干跑：只校验加载
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/studio_smoke_v1 --dry-run --tick-once

# 真机 / 仿真
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/refactored_sdk_single_tag_pick_v0
```

> 相关：被验证的适配器方法见 [`03-adapters-硬件适配层.md`](./03-adapters-硬件适配层.md)；技能生命周期见 [`04-skills-原子技能.md`](./04-skills-原子技能.md)；行为树与场景见 [`05-orchestration-行为树编排.md`](./05-orchestration-行为树编排.md)。
