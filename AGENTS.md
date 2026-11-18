# AGENTS.md

本文件面向在本仓库工作的 AI 编码代理（Claude Code、Codex、Multica agent）。
其中的事实只来自仓库内文件：本文件、`.gitlab-ci.yml`、`pytest.ini`、`ci_scripts/`。
本文件入库，因此在每个任务 worktree 里都可见。

## 0. 硬边界（先读，违反即回退）

- **不 push、不开 MR。** 分支由人推送，代理只输出可直接复制的命令。
- **不下发真机指令。** 真机测试由人执行；物理数值只能由人给出。
- **一个分支同时只能被一个 worktree 检出。** ROS 的 `build/` `devel/` 在源树内，多任务并行必须一任务一个 `git worktree`，不能靠 `git checkout` 切分支。
- **不要 `cp -r` 整仓**做实验或备份：原地改 + `git checkout --` 还原，或 `git worktree add --detach`，或 `git archive | tar -x -C /tmp`。

## 1. 这是什么

LeTools 是乐聚夸父人形机器人的 Python 技能工具链框架，支持轮式（`leju_wheeled`）和足式（`leju_bipedal`）两种平台，提供从硬件驱动到行为树任务编排的分层抽象。
技术栈：**ROS 1 (Noetic)** + **PyTrees 行为树** + **Kuavo Humanoid SDK**（git 子模块，位于 `drivers/leju/kuavo_humanoid_sdk`）。

## 2. 环境

```bash
export PYTHONPATH=$PWD:$PYTHONPATH
source infrastructure/ros_packages/devel/setup.bash   # ROS 消息包 (kuavo_msgs, ocs2_msgs)
export ROBOT_VERSION=62                               # 5W 轮臂版本
```

- 子模块版本由 `scripts/kuavo_humanoid_sdk_tools/sdk_version.env` 锁定，运行 `scripts/install_sdk.sh` 安装。
- 测试前先 `source /opt/ros/noetic/setup.bash`，否则 `adapters/hardware/leju_wheeled/tests` 整个根 collection error。
- ROS 发行版：**Noetic**。`infrastructure/ros_packages/` 用 `catkin build` 编译；其中 `build.sh` 是 `kuavo_tf2_web_republisher` 的专用编译脚本。
- 子模块两个，来源由 `.gitmodules` 配置：`drivers/leju/kuavo_humanoid_sdk`（gitcode，branch dev）、`LeTools-Learning`（github，branch main）。

## 3. 怎么跑

主入口是 `apps/test_upper_init/run_behavior_tree_json.py`（场景无关，**必须显式传 `--scenario`**，没有默认场景）。

```bash
# 推荐入口（仓库根目录执行）
python3 apps/test_upper_init/run_behavior_tree_json.py --scenario orchestration/scenarios/<scene>
python3 apps/test_upper_init/run_behavior_tree_json.py --scenario orchestration/scenarios/<scene> --dry-run
python3 apps/test_upper_init/run_behavior_tree_json.py --scenario orchestration/scenarios/<scene> --dry-run --tick-once
python3 apps/test_upper_init/run_behavior_tree_json.py --scenario orchestration/scenarios/<scene> --spin
python3 apps/test_upper_init/run_behavior_tree_json.py --tree path/to/py_tree.json --board path/to/board.json

```

`--scenario` 自动取该目录下的 `py_tree.json` / `py_tree_child.json` / `board.json`；`--tree` / `--board` 优先级高于 `--scenario` 的默认取值。参数全表见 `apps/test_upper_init/readme.md`。

退出码（见 `run_behavior_tree_json.py:429-438`）：`0`=成功，`1`=失败，`2`=其他/未达终态。

干跑模式：`STUDIO_DRY_RUN=1`（由 `--dry-run` 自动设置）。运动节点直接返回 SUCCESS 而不调用 IHardware，用于 CI / import 验证。

验证：`pytest`（按 `pytest.ini` 的 4 个 `testpaths`）。CI 的 `test:unit` 用 19 个 `--deselect` 跳过 sdk_manager 用例（`test_arm_sdk_manager.py` 11 + `test_timed_cmd_manager.py` 8）→ **CI 绿 ≠ 本地全绿**，别把既有报错当成自己改坏。

## 4. 架构

```
drivers/           底层硬件通信
  leju/              end_effector.py、kuavo_humanoid_sdk（子模块）

infrastructure/    ROS 包（kuavo_msgs, ocs2_msgs, 相机驱动, YOLO 检测等）

core/              核心领域层 — 接口、数据模型、工具（无外部依赖）
  interfaces/        IHardware, ISkill, ICamera, IPerception（抽象契约）
  domain/            Pose6D, Result, JointState, Enums (FrameType, MPCControlMode, ArmSide 等)
  common/            Transform, Interpolator, Logger, ConfigLoader, MathUtils
                     （core/ 只有 interfaces/ domain/ common/ 三个子目录，没有 services/；
                       测试在 core/common/tests/。SDK 与状态管理都在 adapters 层）

config/            应用配置文件

adapters/          硬件适配器层 — 按机器人平台实现 IHardware
  hardware/
    factory.py       HardwareFactory：根据 config['robot_type'] 创建适配器
    leju_wheeled/    LejuWheeledArmHardware（主力适配器，基于 mixin 组合）
      mixins/        12 个 mixin：base_control, arm_control, torso_control,
                     end_effector, timed_command, state_feedback, mode_service,
                     sdk_control, lifecycle, force_control, ik_service,
                     navigation_chassis
      mixins/jiateng/、mixins/jibot/
                     平台专用底盘 mixin（JiatengChassisMixin、JibotChassisMixin）
      services/      sdk_manager/（BaseSDKManager, TimedCmdManager,
                     ArmSDKManager, LowLevelSDKManager）、state_manager.py
      camera_adapter.py   CameraAdapter(ICamera)
    leju_bipedal/    LejuBipedalHardware（初步框架）

skills/            原子技能库（操作、运动、感知）
  atomic/            按技能分目录，每个含 skill.py + 可选 params.py
  base/skill_base.py 技能基类
  manipulation/ motion/ perception/ policy_inference/  领域技能
  templates/         技能模板

orchestration/     PyTrees 行为树运行时
  main.py            通用入口见 apps/test_upper_init/run_behavior_tree_json.py
  engine/            BehaviorTreeFactory (JSON→树), BehaviorTreeController (50Hz tick)
  nodes/             行为节点 (ChassisShortMove, LegShortMove, MoveArmBaseJointTrajectories 等)
  scenarios/         树 + 黑板 JSON 配置（顶层可直接跑；scenarios_internal/ 下需显式指定）
  shared_hardware.py 全局 IHardware 单例，供所有树节点使用
  tasks/             任务级行为（arm_test, arm_trajectory, smt_tray）
  parser/            task_parser.py（场景 JSON 解析）
  services/          blackboard_service.py（黑板服务，树节点间共享数据）
  web_ui/            Web UI 场景配置 JSON（tree_main_web.json 等）
  studio_tree.py     Studio 树工具
  config/            编排层配置
  utils/             编排层工具

apps/              按测试层级组织的测试脚本
  test_kuavo_5w_sdk_adapter/  推荐的 SDK 接口测试
  test_camera_adapter/    相机接口测试
  jiateng_adapter/ jibot_adapter/   底盘接口测试
  test_kuavo_5w_orchestration/ test_kuavo_5w_skills/ test_upper_init*/   其余层级测试
```

## 5. 关键模式

- **Result 模式**：`IHardware` 的**动作类**方法返回 `core.domain.result.Result`（success/message/data）；纯取值访问器按实际类型返回（如 `is_connected() -> bool`、`get_ee_poses() -> Optional[Dict]`）。用 `Result.ok()` / `Result.fail()` 工厂方法 —— 不要用异常处理预期内的失败。
- **硬件工厂**：`HardwareFactory.create_hardware(config)` 按 `config['robot_type']` 选适配器，支持 `leju_wheeled`、`leju_bipedal`、`mock`，另有 `leju_wheeled_ros2` / `ros2_wheeled` 走 ROS2 分支（`adapters/hardware/factory.py:36`）。
- **Mixin 组合**：`LejuWheeledArmHardware` 由 12 个 mixin 组成（每个控制域一个，清单见 §4）。新增硬件能力应在 `adapters/hardware/leju_wheeled/mixins/` 下新建 mixin。
- **行为树节点**：`scenarios/*/py_tree.json` → `BehaviorTreeFactory` 从 `orchestration/nodes/` 动态导入节点类；类名从 JSON 的 `name`/`nodeType`/`className` 字段解析，子树（`.json` 结尾）从 `py_tree_child.json` 加载。
  - 新增树节点放 `orchestration/nodes/` —— 由 `_build_node_index()` 自动发现（snake_case 文件名 → PascalCase 类名）。
- **参数来源**（树 JSON 中）：`CUSTOM`（字面值）、`RESOLVED`（宏替换后的值）、`INPUT`（通过 `${var}` 从黑板读取）、`READ_BOARD`（显式黑板读取）。覆盖优先级：`override_params` > 黑板 > 默认值。
- **SDK 管理器**：`adapters/hardware/leju_wheeled/services/sdk_manager/` 封装 Kuavo SDK API（TimedCmdAPI 等）。`ArmSDKManager` / `TimedCmdManager` / `LowLevelSDKManager` 都继承 `BaseSDKManager`，细节见该目录 README.md；控制模式枚举 `MPCControlMode` 在 `core/domain/enums.py`。
- **日志**：用 `core.common.logger.get_logger(__name__)`，支持 Trace ID 和相对路径。生产代码路径中不要用 `print()`。

## 6. 重要约定

- Core 层必须**零外部依赖**（不引入 ROS、SDK 等）。
- Adapters 层实现 IHardware；Apps 层消费 IHardware —— 应用代码中绝不直接导入适配器实现。
- 所有公共方法需要类型注解；动作类方法返回 `Result`，纯取值访问器按其实际类型标注。
- 轮式适配器的 ROS 话题/服务定义在 `kuavo_msgs` 中（通过 catkin 在 `infrastructure/ros_packages/` 编译）。
- CI 的依赖清单是 `ci_scripts/requirements.txt`（受保护白名单，拒绝 `-e`/`-r`/URL/本地路径）；根目录 `requirements.txt` 只供开源产物/下游使用。

## 7. 文档在哪里

- `docs/` 是**公开面**，会同步上官网；每章篇目数被 `tests/test_docs_links.py:47` 的 `CHAPTER_ARTICLE_COUNTS` 硬断言锁住（合计 59 篇见 `:57`）——**新增篇章必须先改该常量**，否则 CI 红。
- `docs_internal/` 是内部文档，不随开源产物外发（`*_internal/` 在 CI 里被 rsync 排除）。新的内部文档放这里，不会触发守卫。
- 写含命令的讲义类文档时，遵守 `tests/test_docs_path_baseline.py` 的路径基准规则。

## 8. 收尾三查

1. `git status --porcelain -uall` 为空。
2. 未推送提交只允许落在本条交付分支上；分支名 + HEAD commit 写进交付说明。
3. 工作目录 < 5G；超了要写清是什么、何时删。
