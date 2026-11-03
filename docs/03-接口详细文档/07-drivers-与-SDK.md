# 07 drivers 与 SDK

> 本篇是**参考篇**。drivers 层是 adapters 的底层支撑，**正常开发不需要直接调用**；SDK 是 drivers 的底层依赖，版本由 `scripts/` 的锁定清单统一管理。
>
> 需要写 `from drivers.leju.end_effector import LejuEndEffector` 时，先读 `03-adapters-硬件适配层.md` 的 `control_end_effector` 一节 —— 九成场景直接用适配器方法就够。

## 07.01 drivers 层是什么

| 维度 | 说明 |
|---|---|
| 定位 | 框架最底层：直接操作硬件 / ROS 话题 / 服务 / SDK |
| 上游 | 只被 `adapters/` 调用（`core/` **不依赖** drivers） |
| 抽象程度 | **最小抽象**：暴露硬件原始能力，不做业务逻辑 |
| 当前内容 | `LejuEndEffector`（末端执行器驱动）+ Kuavo Humanoid SDK（Git Submodule） |
| 未实现 | 相机驱动（`drivers/camera/` 目录尚不存在） |

源码仓库中的主要文件如下。`drivers/` 未列入闭源编译清单，开源产物保留其 `.py` 源码；SDK submodule 另行安装：

```text
drivers/__init__.py
drivers/leju/__init__.py
drivers/leju/end_effector.py
drivers/leju/kuavo_humanoid_sdk/        ← Git Submodule
```

> **注意**：`LejuEndEffector` 由上级 `drivers/__init__.py` 导出（`drivers/leju/__init__.py` 只导出 SDK 侧的 `KuavoRobot` / `KuavoRobotState` / `SDK_AVAILABLE`）；部分历史代码还写着 `from drivers.leju import LejuJointController, LejuEndEffector`（见 `adapters/hardware/leju_bipedal/hardware.py`），这两个名字在 `drivers/leju` 包级都取不到，该 import 行会直接 `ImportError` —— leju_bipedal 适配器属未完成分支，轮臂（`leju_wheeled`）才是现行路径。

谁在调 drivers：

| 调用方 | 调什么 |
|---|---|
| `adapters/hardware/leju_wheeled/hardware.py` | `from drivers.leju.end_effector import LejuEndEffector`（`hardware.py` 第 149 行，实例化于第 203 行：`LejuEndEffector(self.config)`） |
| `adapters/hardware/leju_bipedal/hardware.py` | `from drivers.leju import LejuJointController, LejuEndEffector`（当前包导出下会 `ImportError`，未完成分支） |
| `orchestration/nodes/tests/test_leju_claw_single_side_safety.py` | 单元测试直接构造驱动 |

除上述三处，**框架其余部分不引用 drivers**。

## 07.02 类与方法详表

### `LejuEndEffector` —— 末端执行器驱动

路径：`drivers.leju.end_effector.LejuEndEffector`

支持三类末端，由 `config['type']` 决定（缺省 `leju_claw`）：

| `EndEffectorType` | 值 | 说明 | 通信方式 |
|---|---|---|---|
| `LEJU_CLAW` | `leju_claw` | 乐聚二指夹爪 | ROS Service `/control_robot_leju_claw` |
| `QIANGNAO_HAND` | `qiangnao` | 强脑灵巧手 | ROS Topic `/control_robot_hand_position` |
| `SG100_HAND` | `sg100` | 黑漫 SG100 11-DOF 灵巧手 | ROS Topic `/sg100_hand_command` + `/sg100_hand_state` |
| `SUCTION_CUP` | `suction_cup` | 吸盘 | —（枚举已定义，本驱动未实现分支） |
| `NONE` | `none` | 无末端 | — |

| 路径 | API | 签名 | 说明 | 所属层 |
|---|---|---|---|---|
| 工厂 | `__init__` | `__init__(config: dict)` | 从 `config['type']` 解析末端类型；不建立任何连接 | `drivers.leju.end_effector` |
| 查询 | `last_error` | `last_error -> str` | 最近一次连接失败原因（property），供 Adapter 向上层传播 | 同上 |
| 标准 | `connect` | `connect() -> bool` | 建 Publisher / ServiceProxy / Subscriber 并等待注册；返回是否就绪 | 同上 |
| Core | `disconnect` | `disconnect() -> None` | 注销夹爪状态订阅 `_state_sub`、清缓存并置 `_connected=False`（SG100 状态订阅与各 Publisher 不注销） | 同上 |
| 标准 | `send_command` | `send_command(side: str, cmd: Union[GripperCommand, DualGripperCommand]) -> Result` | 通用夹爪指令（LEJU_CLAW 走服务） | 同上 |
| 标准 | `send_hand_command` | `send_hand_command(left_cmd: HandFingerCommand, right_cmd: HandFingerCommand) -> Result` | 强脑灵巧手指令（`robotHandPosition` 消息，一次发双侧） | 同上 |
| 标准 | `send_sg100_command` | `send_sg100_command(side: str, cmd: SG100HandCommand) -> Result` | SG100 单手命令；另一侧 `enable_mask=0`、位置列表为空 | 同上 |
| 查询 | `get_state` | `get_state() -> EndEffectorState` | SG100 返回**最近下发侧**的双侧缓存状态；其余返回 `_current_state` | 同上 |
| — | `_on_claw_state` | `_on_claw_state(msg) -> None` | `/leju_claw_state` 回调，解析失败仅 WARNING 不抛 | 同上 |
| — | `_on_sg100_state` | `_on_sg100_state(msg) -> None` | `/sg100_hand_state` 回调，双侧独立解析 | 同上 |

内部辅助（不供外部调用）：

| API | 签名 | 说明 |
|---|---|---|
| `_complete_claw_positions` | `(left_position, right_position) -> Result` | 用反馈补齐**未控制侧**，避免服务端把该侧默认置 0（张开） |
| `_get_claw_positions` | `(timeout=None) -> Result` | 返回缓存的左右位置；无反馈时等首帧，超时失败。`timeout` 默认取 `config['claw_state_timeout']`（`1.0`），非正数/非有限值直接失败 |

### Kuavo Humanoid SDK

路径：`drivers/leju/kuavo_humanoid_sdk/` （Git Submodule，仓库 `kuavo-ros-opensource`；安装版本以 `scripts/kuavo_humanoid_sdk_tools/sdk_version.env` 锁定的分支与 tag 为准，当前 `master` / `1.4.5`，对应 commit `d824a7b877b41d0b9e6a9ffe947e050d89ea9681`。`.gitmodules` 里的 `branch = dev` **不是版本权威源**，它只在执行 `git submodule update --remote` 时决定子模块的更新目标，不决定当前安装哪个版本；注意 `--remote` 会把子模块前移到 `dev` 分支的**末端**，末端并不等于上面 tag 指向的 commit —— 需要版本对齐时一律以 `sdk_version.env` 为准）

安装后实际 import 名是 `kuavo_humanoid_sdk`，源码位于：

```text
drivers/leju/kuavo_humanoid_sdk/src/kuavo_humanoid_sdk/kuavo_humanoid_sdk/
```

包入口 `kuavo_humanoid_sdk/__init__.py` 用 **懒加载**（`__getattr__` + `_LAZY_MODULES`）转发到五个子模块：`.msg`、`.interfaces`、`.kuavo`、`.kuavo_strategy`、`.kuavo_strategy.grasp_box` —— 所以 `from kuavo_humanoid_sdk import XXX` 只在真正访问属性时才 import。

SDK 自身的主要模块（以 submodule 仓库为准）：

| 模块 | 功能 |
|---|---|
| `interfaces/robot.py` | `KuavoSDK` / `KuavoRobot(RobotBase)` —— 入口类 |
| `interfaces/data_types.py` | SDK 侧数据类型 |
| `interfaces/end_effector.py` | SDK 侧末端执行器抽象 |
| `kuavo/core/core.py` | 核心控制 |
| `kuavo/core/leju_claw_control.py` | 二指夹爪 |
| `kuavo/core/dex_hand_control.py` | 灵巧手 |
| `kuavo/core/audio.py` / `microphone.py` / `llm_*.py` | 语音与对话 |
| `common/logger.py` | `SDKLogger` / `disable_sdk_logging` |
| `common/optional_deps.py` | 可选依赖（`[audio]` / `[vision]` / `[full]`） |

**LeTools 侧只用到 SDK 的 `TimedCmdAPI` 与 `robot_sdk.control.*`**（见 `adapters/.../services/sdk_manager/`），不直接使用 SDK 的语音、视觉、对话模块。

## 07.03 参数说明

### `LejuEndEffector` 的 `config`

| 键 | 默认 | 说明 |
|---|---|---|
| `type` | `'leju_claw'` | 末端类型，取值见 `EndEffectorType` |
| `claw_state_timeout` | `1.0` | 等待夹爪首帧状态的秒数（`_get_claw_positions` 用） |

`config` 直接来自适配器的硬件配置：`app_config.yaml` 的字段被场景级 `hardware_config.json`（或命令行 `--hardware-config` 指定的覆盖文件，见 `apps/test_upper_init/run_behavior_tree_json.py`）按 `config.update` 合并后，经 `HardwareFactory.create_hardware(config)` 传入；末端类型由合并结果中的 `type` 键决定。

### 命令类型

| 类型 | 定义位置 | 字段 |
|---|---|---|
| `GripperCommand` | `core.domain.end_effector` | `position`（行程占比 [0,100]，0=张开 100=闭合）、`velocity`（[0,100]）、`effort`（力矩/电流 A） |
| `DualGripperCommand` | 同上 | `left_position` / `right_position`（`None` 的一侧不参与本次控制）、`velocity`、`effort` |
| `HandFingerCommand` | 同上 | `positions`（6 个手指关节位置，[0,100]） |
| `SG100HandCommand` | 同上 | `positions`（11 关节，**rad**）、`control_mode`、`enable_mask`、`kp` / `kd` / `torque_ff`、`per_joint_modes`；`SG100_JOINT_COUNT = 11` |
| `EndEffectorState` | 同上 | `status: GripperStatus`、`current_position` / `current_velocity` / `current_effort`、`finger_positions`（仅灵巧手有效） |
| `GripperStatus` | 同上 | `ERROR=-1` / `UNKNOWN=0` / `MOVING=1` / `REACHED=2` / `GRABBED=3` |

**单位约定**：`GripperCommand.position` 与 `HandFingerCommand.positions` 为 [0,100] 归一化值（不是角度、不是毫米）；`SG100HandCommand.positions` 为弧度 rad；SG100 的 `kp` / `kd` 为各关节增益列表。`control_end_effector` 适配器方法把这些类型原样透传，具体见 `03-adapters-硬件适配层.md`。

## 07.04 返回值

| 方法 | 返回 |
|---|---|
| `connect()` | `bool`（`True` = 已建好所有通道） |
| `disconnect()` | `None` |
| `send_command` / `send_hand_command` / `send_sg100_command` | `Result` |
| `get_state()` | `EndEffectorState`（**不返回 `Result`**，这是本层少数例外） |
| `last_error` | `str`（连接失败原因） |

`connect()` 对 SG100 有一处特别逻辑：SG100 是话题接口，无法像 LEJU_CLAW 用 `wait_for_service` 探测对端，因此 `connect()` 只检查命令话题**是否有订阅者**，无订阅者时打 WARNING 但**不阻断**（驱动可能稍后启动）；真正的硬校验在 `send_sg100_command()` 下发时再做一次 —— 此时无订阅者直接 `Result.fail(...)`，因为 `publish` 到无订阅者的话题会**静默丢失**。

`send_hand_command()`（强脑 qiangnao）走的是**同一条口径**：下发前检查 `/control_robot_hand_position` 是否有订阅者，无订阅者直接 `Result.fail`，不再让调用方以为下发成功。此外 `robotHandPosition` 的左右手字段是 `uint8[]`，位置会先取整（`int(round(...))`）再发布 —— 传浮点会在序列化阶段报 `must be unsigned integer type`。

## 07.05 异常

本层**几乎不抛异常**，统一转 `Result`：

| 情形 | 表现 |
|---|---|
| 未连接就发命令 | `Result.fail("Driver not connected")` 或 `Result.fail("Hand driver not ready")` |
| 末端类型与命令不匹配 | `Result.fail(f"Gripper command not supported for type: ...")` / `"Current end effector is not Leju Claw"` |
| SG100 命令话题无订阅者 | `Result.fail("SG100 命令话题 /sg100_hand_command 无订阅者，请确认 SG100 驱动节点已启动")` |
| 强脑手命令话题无订阅者 | `Result.fail("Qiangnao hand driver not ready: 话题 /control_robot_hand_position 无订阅者，命令会被丢弃")` |
| `claw_state_timeout` 非正数 / 非有限值 | `Result.fail("claw_state_timeout positive number")` |
| 状态回调消息格式非法 | `logger.warning("Ignore invalid /leju_claw_state message: ...")`，**丢弃不抛** |

唯一会直接冒出的异常在 `connect()` 的 `except Exception` 里被收敛成 `bool = False` + `last_error` 字符串，附带提示：

```text
"{type(e).__name__}: {e}; 已检查生成包目录 {path_hint}。
 若从其他入口运行，请先 source infrastructure/ros_packages/devel/setup.bash"
```

也就是说：**拿不到 `last_error` 就说明连接其实成功了**。

## 07.06 调用示例

### 直接驱动（一般不需要，仅调试用）

```python
from drivers.leju.end_effector import LejuEndEffector
from core.domain.end_effector import GripperCommand

ee = LejuEndEffector({'type': 'leju_claw'})

if not ee.connect():
    print("连接失败:", ee.last_error)
    raise SystemExit(1)

result = ee.send_command(
    side='left',
    cmd=GripperCommand(position=50.0),  # 行程占比 50%；velocity/effort 使用默认值
)
print("夹爪控制成功" if result.success else f"失败: {result.message}")

ee.disconnect()
```

### 通过适配器（推荐）

```python
from core.domain.enums import ArmSide
from core.domain.end_effector import GripperCommand

# hw 为已创建并初始化的硬件适配器（已配置乐聚二指夹爪）——不要自己造 LejuEndEffector
result = hw.control_end_effector(
    side=ArmSide.LEFT,
    cmd=GripperCommand(position=100.0),  # 0 为张开，100 为闭合
)
print("夹爪控制成功" if result.success else f"失败: {result.message}")
```

`GripperCommand` 的字段为 `position`（行程占比 0–100）、`velocity`（速度 0–100，默认 50.0）和 `effort`（电流参数，单位 A，默认 1.0），不支持 `open_ratio` 或 `force`。适配器接口使用 `side=ArmSide.LEFT/RIGHT/BOTH` 和 `cmd` 命令对象，不接收 `action` 参数。

`control_end_effector` 的参数与返回值见 [`03-adapters-硬件适配层.md`](./03-adapters-硬件适配层.md) 的「末端执行器」小节。

### 安装 / 卸载 SDK

```bash
# 安装：从远端按版本锁定清单拉取 submodule（推荐，版本受控）
./scripts/install_sdk.sh

# 卸载
./scripts/remove_local_sdk.sh --yes
```

三个脚本的参数与退出码见 [`08-工具层-脚本与配置.md`](./08-工具层-脚本与配置.md) 的 `08.01`。

## 07.07 为什么不需要读本篇就能开发

三条理由：

1. **drivers 的唯一公共能力（末端执行器）已经被适配器封装**，`hw.control_end_effector(...)` 是稳定接口，直接调 `LejuEndEffector` 反而绕过了适配器的状态管理与日志。
2. **SDK 的版本由 `scripts/install_sdk.sh` 的 pin 清单锁定**，不由业务代码决定；业务代码只通过 SDKManager 间接访问 `TimedCmdAPI` / `robot_sdk.control.*`。
3. **drivers 层不做业务抽象**，它只做「协议透明转发」—— 任何业务语义（安全默认值、姿态单位换算、双臂补齐）都在 adapters 层。

需要**改** drivers 的场景只有两类：

- 新增一种末端执行器类型（在 `EndEffectorType` 加枚举 + 在 `LejuEndEffector` 加分支 + 在适配器 mixin 暴露方法）；
- 升级 / 降级 SDK 版本（改 pin 清单，不碰代码）。

> 相关：适配器暴露的末端执行器接口见 [`03-adapters-硬件适配层.md`](./03-adapters-硬件适配层.md)；SDK 的 `_sdk` 直调路径见 [`09-控制路径与选型.md`](./09-控制路径与选型.md)；安装脚本见 [`08-工具层-脚本与配置.md`](./08-工具层-脚本与配置.md)。
