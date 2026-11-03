# 06 扩展 apps 层

`apps/` 是**跑得起来的最小入口**：一个目录 = 一个可独立执行的验证脚本集合。

> 落层判定与依赖方向见 `01-扩展总则与分层规范`。`apps` 可以依赖所有下层；反过来，下层仅在调试路径的惰性 import 和个别独立配套脚本中引用 `apps`（E3，清单见 [`01-04-系统架构与分层`](../01-介绍/04-系统架构与分层.md#四已知分层例外e1--e2--e3)），生产路径严禁依赖它。

## 一、目录分工

`apps/` 下共 13 个目录，按「被测层」分一级目录、按「能力域」分二级目录：

| 目录 | 分工 |
|---|---|
| `test_kuavo_5w_adapter/` | Adapter 层硬件测试（`01_base_control` … `08_end_effector`） |
| `test_kuavo_5w_skills/` | 技能层测试 |
| `test_kuavo_5w_orchestration/` | 编排层节点测试（每个节点一个脚本） |
| `test_kuavo_5w_sdk_adapter/` | SDK 接口验收（`sdk/` 与 `timed/` 两套） |
| `test_camera_adapter/` | 相机与感知适配器测试（深度、点云、TF、AprilTag 等） |
| `test_demo/` | 演示脚本 |
| `test_upper_init/` | **JSON 行为树启动器**（`run_behavior_tree_json.py` 及批量运行脚本） |
| `jibot_adapter/` | 玖物底盘迁移测试 |
| `jiateng_adapter/` | 嘉腾底盘迁移测试 |

行为树场景的启动统一走 `apps/test_upper_init/run_behavior_tree_json.py`；层级的调用路径见 `05-05-扩展-orchestration-层`。

## 二、脚手架复用

新增脚本时**优先复用同目录下既有的启动/参数/日志脚手架**，而不是重写一遍。

- `apps/test_kuavo_5w_adapter/_scaffold.py`：ROS 初始化、配置装载、日志与硬件工厂的公共封装；目录内另有 `run_test.sh` 与 `config/`。
- `apps/test_kuavo_5w_sdk_adapter/_scaffold.py`：SDK 级测试的 handle 创建与 teardown（注意：这个 `_scaffold` 被 `adapters` / `orchestration` 的调试路径反向引用，见 `01-扩展总则与分层规范` E3，改动它影响面比想象中大）。
- `apps/jiateng_adapter/_scaffold.py`：`jiateng_setup()` / `read_vel_control_state()` / `assert_vel_control_state()` / `assert_no_active_navigation_task()`，以及五个必需服务的等待。
- `apps/jibot_adapter/_scaffold.py`：玖物侧对应封装。

**一个能力域一个脚本**：目录内已有 `test_*.py` 时新增同类能力，复制最近的同域脚本改 `name` 与期望输出，比新建框架更安全（`_scaffold` 的 import 顺序、ROS 节点名、服务等待超时都已调好）。

## 三、脚本写法约定

- **目的明确**：脚本头部注明测试目标、前置条件（ROS / 运控站立 / 真机 / 仿真）与运行命令；
- **日志统一**：用 `core.common.logger` 的 `init_logging()` + `get_logger(__name__)`，不写裸 `print()`；
- **异常与超时**：服务等待、运动等待都要有超时保护，失败时打印可定位的上下文；
- **判定清晰**：脚本自带明确的成功/失败输出，示例脚本以「能跑通并打印预期现象」为准；
- **不写业务动作控制逻辑**：`apps` 只做「准备环境 + 调用 + 观察」，动作实现属于 `skills` / `adapters`。参考 `run_behavior_tree_json.py` 的注释约定——它是薄启动器，执行交给 `orchestration`。

## 四、验证

- 硬件相关脚本需要仿真或真机，按 `pytest` marker 记为 `integration`；纯逻辑的记为 `unit`（两者都不在 `pytest.ini` 的默认 `testpaths` 里，`apps/*` 需显式给路径运行）；
- 行为树语法自检走 dry-run：`python3 apps/test_upper_init/run_behavior_tree_json.py --scenario <dir> --dry-run --tick-once`，判据为输出 `tick 后根状态: Status.SUCCESS`（dry-run 的退出码只反映加载是否成功）；
- 部署/运行前置条件与验收输出见 `02-环境安装` 与 `04-简单案例运行`。
