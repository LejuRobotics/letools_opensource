# 03 扩展 adapters 层

`adapters` 把硬件能力封装成上层可以直接调用的标准接口，**一厂商一目录、一能力一 Mixin**。

> 落层判定与依赖方向见 `01-扩展总则与分层规范`。`adapters` 不得 import `skills` / `orchestration` / `apps`（现有例外为 E3 中的调试惰性 import，清单见 [`01-04-系统架构与分层`](../01-介绍/04-系统架构与分层.md#四已知分层例外e1--e2--e3)）。

## 一、Mixin 组合方式

`adapters/hardware/leju_wheeled/hardware.py` 是**组装点**。主类只做三件事：初始化状态字段、Mixin 多继承、重导出。具体方法全部落在 `mixins/` 下的单个 Mixin 里。

现有 Mixin 清单（`hardware.py` 的组装列表）：

| Mixin | 职责 |
|---|---|
| `LifecycleMixin` | 初始化与关闭流程 |
| `BaseControlMixin` | 底盘速度 / 位置控制（ROS 话题） |
| `TorsoControlMixin` | 躯干位姿控制 + 笛卡尔焦点设置 |
| `ArmControlMixin` | 手臂关节 / 末端位姿 / 腿部关节 |
| `TimedCommandMixin` | TimedCmd 路径（`_timed` 后缀 + `send_timed_*` + Ruckig / 离线轨迹 / IK） |
| `ForceControlMixin` | 末端期望力控制（ROS 话题，对齐 LBForceController） |
| `ModeServiceMixin` | MPC / 快速模式 / 手臂控制模式 |
| `EndEffectorMixin` | 末端执行器（夹爪 / 灵巧手） |
| `StateFeedbackMixin` | 状态查询接口 |
| `IkServiceMixin` | IK 求解（`solve_ik`）、TF 查询（`get_transform`）、MoveIt 关节路径校验（`check_joint_path`） |
| `SDKControlMixin` | SDK 直调路径（`_sdk` 后缀 + `control_head` / `arm_reset`） |
| `JibotChassisMixin` | 玖物（JiBot）品牌底盘（ROS 服务，`_jibot` 后缀） |
| `JiatengChassisMixin` | 嘉腾底盘（ROS 服务，`_jiateng` 后缀） |
| `NavigationChassisMixin` | 标准导航接口到所选底盘后端的**路由** |
| `CartonVisionMixin` | 纸箱视觉相关能力 |

**新增一种能力的做法**：新建 `mixins/<capability>_mixin.py`，在 `mixins/__init__.py` 中导出，并在 `hardware.py` 的组装清单与基类列表两处同时追加。**一个 Mixin 只负责一个职责域**，不要把不相关的方法塞进已有 Mixin。

三种控制方式（`hardware.py` 头部注释规定的命名约定）在同一能力上并存，新增方法时按后缀区分：

- **标准接口**（符合 `IHardware`，ROS 话题/服务，推荐上层使用）：无后缀，如 `control_head()`、`arm_reset()`；
- **SDK 直调**（高频循环/力控）：`_sdk` 后缀，如 `control_head_sdk()`、`send_base_velocity_sdk()`；
- **TimedCmd**（带时间参数的精确运动规划）：`_timed` 后缀，如 `send_base_velocity_timed()`。

标准方法内部可以调用 `_sdk` 或 `_timed` 方法，以复用逻辑。

## 二、factory 注册

`adapters/hardware/factory.py` 的 `HardwareFactory.create_hardware(config)` 按 `config['robot_type']` 分支创建适配器。当前已注册三个：

| `robot_type` | 实现类 |
|---|---|
| `leju_bipedal` | `LejuBipedalHardware` |
| `leju_wheeled` | `LejuWheeledArmHardware`（默认，见 `config/app_config.yaml`） |
| `mock` | `MockHardwareAdapter` |

**新增机器人型号**：在工厂里加一个 `elif` 分支，并把实现放在 `adapters/hardware/<vendor>/`。未知 `robot_type` 会抛 `ConfigurationError`，不要用 try/except 吞掉。`robot_type` 缺失同样直接报错（不是默认回退）。

## 三、品牌底盘范例：jiateng / jibot

两个底盘后端各占一个子目录（`mixins/jiateng/`、`mixins/jibot/`），分别提供 `_jiateng` / `_jibot` 后缀的厂商方法。

上层**不感知**底盘品牌。调用链固定为：

```text
上层 → NavigationChassisMixin.move_chassis_relative(...)   # 无厂商名的标准接口
     → 按 config/app_config.yaml 的 navigation_chassis_backend 路由
       → JiatengChassisMixin.base_move_relative_jiateng(...)
        或 JibotChassisMixin.base_move_relative_jibot(...)
```

路由表定义在 `mixins/navigation_chassis_mixin.py` 的 `_NAVIGATION_METHODS`，每个后端注册五个能力键：

| 能力键 | 含义 |
|---|---|
| `relative` | 本体坐标系相对移动 |
| `target` | map 坐标系绝对目标 |
| `arrived` | 到达检查 |
| `set_external` | 外部速度控制权开关 |
| `get_external` | 外部控制权状态读取 |

**新增底盘后端**：新增一个子目录 + 在 `_NAVIGATION_METHODS` 注册这五个能力键，然后在 `app_config.yaml` 的 `navigation_chassis_backend` 指向它。后端未实现某能力键时，路由层会返回 `Result.fail("未实现导航能力: ...")`，不会静默降级。

两个后端的服务路径与消息类型差异、`navigation_chassis_backend` 的取值口径，详见 `07-02-平台适配说明`。

## 四、接口边界

- 上层（`skills` / `orchestration`）**只经 `IHardware` 抽象接口调用**；
- 厂商方法（`_jiateng` / `_jibot` / `_sdk` / `_timed` 后缀）**不得溢出到上层**——它们是同层内的实现手段，不是跨层契约；
- 适配器方法返回 `Result`，不把 ROS 异常直接抛给上层。

已知越界（新增代码不得照做）：`skills/atomic/manipulation/*` 与部分 `refactored_sdk` 技能直接 import 了 `LejuWheeledArmHardware` / `adapters.vacuum_control` 等具体实现，见 `01-扩展总则与分层规范` 例外表 E1。

## 五、验证

`adapters` 层测试有两个既定入口（均已列入 `pytest.ini` 的 `testpaths`）：

- `adapters/hardware/leju_wheeled/tests/`
- `adapters/hardware/leju_wheeled/services/sdk_manager/tests/`

另有 `adapters/hardware/leju_wheeled/mixins/tests/` **未**列入 `testpaths`，直接 `pytest` 不会拾起，需要显式给路径。口径细节见 `05-07-测试与调试`。

新增 Mixin 时同步补 `mixins/tests/` 下的单测；需要真机的用例打 `-m integration` 并单独跑。
