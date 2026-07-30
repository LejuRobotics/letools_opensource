<div align="center">

<h1 align="center">LeTools</h1>

[![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)](https://www.python.org/)
[![ROS](https://img.shields.io/badge/ROS-Noetic-22314E.svg)](http://wiki.ros.org/noetic)
[![Leju](https://img.shields.io/badge/Leju-Robotics-orange)](https://www.lejurobot.com/zh)

📖 乐聚 Kuavo 机器人上位机侧 Python 技能工具链框架

</div>

> ## 📚 文档导航
>
> | 📖 文档 |  链接 |
> |:---|:---:|
> | 零基础完整学习教程 | [beginner_tutorial.md](docs/beginner_tutorial.md) |
> | 启动指南 | [user_guides.md](docs/user_guides.md) |
> | 主要身体部位 API 参考 | [upper_body_api.md](adapters/upper_body_api.md) |
> | 相机与底盘 API 参考 | [camera_chassis_api.md](adapters/camera_chassis_api.md) |

`LeTools` 把机器人能力拆成几层：

```text
apps 示例/测试脚本
    ↓
orchestration 行为树编排
    ↓
skills 原子技能
    ↓
adapters 硬件适配器
    ↓
core 接口/数据模型/SDK 管理
    ↓
drivers + infrastructure 底层 SDK / ROS / 相机 / 消息包
    ↓
仿真或真机
```

新手可以把它理解成一个"动作积木系统"：`core` 定义积木接口，`adapters` 把接口接到机器人，`skills` 封装单个动作，`orchestration` 把动作排成任务，`apps` 提供可直接运行的例子。

***

## ✨ 核心特性

|      特性      | 说明                                                                         |
| :----------: | :------------------------------------------------------------------------- |
|  🤖 **分层架构** | core 接口/数据模型 → adapters 硬件适配 → skills 原子技能 → orchestration 行为树编排 → apps 示例 |
| 🦾 **多部位控制** | 底盘运动、双臂 14 关节、腿部/躯干、头部、末端执行器、力控                                            |
| 📷 **相机/视觉** | Orbbec/RealSense 启动、RGB/深度/点云、AprilTag、YOLO、二维码读取                          |
| 🌳 **行为树编排** | JSON 加载动作流程，支持顺序、并行、等待、黑板参数、子树复用                                           |
| 🔌 **多控制路径** | 标准接口 · SDK 直调 · TimedCmd（Ruckig / IK / 离线轨迹）                               |
| 🚀 **仿真&真机** | 支持 dry-run 离线验证、MuJoCo 仿真与真机部署                                             |

***

## 📋 环境要求

推荐环境：

- Ubuntu 20.04
- ROS Noetic（ROS1 通信、消息、服务、TF）
- Python 3.8+
- catkin tools（推荐 `catkin build`）
- Kuavo SDK（`drivers/leju/kuavo_humanoid_sdk` 子模块）
- Docker / MuJoCo（可选，仿真调试需要）

***

## 🛠️ 安装
详细部署安装步骤请参考[user_guides.md](docs/user_guides.md) 

**1. 编译 ROS 工作空间**

```bash
cd ~/letools_opensource/infrastructure/ros_packages
source /opt/ros/noetic/setup.bash
catkin build
source devel/setup.bash
```

> 如果暂时不需要 RealSense 相机，且视觉相关模块编译失败，可以先跳过：
>
> ```bash
> catkin config --skiplist \
    detection_yolo_v8 \
    ar_control \
    kuavo_vision_object \
    kuavo_yolo_point2d \
    yolo_box_object_detection \
    yolo_button_object_detection \
    yolo_valve_object_detection \
    orbbec_camera \
    realsense2_camera \
    kuavo_camera
> catkin build
> ```

**2. 安装 SDK**

```bash
cd ~/letools_opensource
chmod +x scripts/install_sdk.sh
./scripts/install_sdk.sh
```

> 📌 SDK 版本由仓库自带的 `scripts/kuavo_humanoid_sdk_tools/sdk_version.env` 锁定，
> 记录当前 LeTools 配套的 kuavo-ros-opensource 分支与 tag（如 `master` / `1.4.4`），
> 每次 LeTools 发版时同步更新，确保用户安装到与当前版本匹配的 SDK。

SDK 安装成功后验证：

```bash
python3 -c 'from kuavo_humanoid_sdk import KuavoRobot; print("SDK Ready")'
```

<details>
<summary>📖 ROS 工作空间说明</summary>

ROS 相关内容在 `infrastructure/ros_packages/`，典型 catkin 工作空间结构：

```text
ros_packages/
├── src/       # ROS package 源码
├── build/     # 编译中间文件
├── devel/     # 编译产物和 setup.bash
└── logs/      # 编译日志
```

ROS 中常见概念：

| 概念      | 说明             | 常用命令                                      |
| ------- | -------------- | ----------------------------------------- |
| Node    | 独立运行的 ROS 程序   | `rosnode list`                            |
| Topic   | 持续发布/订阅的数据通道   | `rostopic list`, `rostopic echo /topic`   |
| Message | Topic 中传输的数据结构 | `rosmsg show <type>`                      |
| Service | 一次请求一次响应的接口    | `rosservice list`, `rosservice call /srv` |
| Action  | 带目标、反馈、结果的长任务  | `rostopic list` 查看 action 相关 topic        |
| TF      | 坐标变换系统         | `rosrun tf tf_echo base_link camera_link` |
| Launch  | 一次启动多个节点       | `roslaunch pkg file.launch`               |

</details>

> 🚀 仿真环境部署与运行见 [用户指南](docs/user_guides.md)

***

## 🚀 快速开始

### 📊 Step 1 — 离线验证行为树

没有机器人、没有仿真时，可以先 dry-run：

```bash
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/refactored_sdk_atomic_v1 \
  --dry-run --tick-once
```

***

### 🧠 Step 2 — 运行硬件示例

以"头部转动"为例，先加载环境，再运行示例脚本：

```bash
source /opt/ros/noetic/setup.bash
source infrastructure/ros_packages/devel/setup.bash
export PYTHONPATH=$(pwd):$PYTHONPATH
python3 apps/test_kuavo_5w_sdk_adapter/sdk/01_head/test_head_control.py
```

> 📘 示例脚本按 `sdk/01_head` \~ `sdk/06_feedback` 分类，详见 [示例目录](apps/test_kuavo_5w_sdk_adapter/README.md)

***

### 🔗 Step 3 — 理解调用链路

<details>
<summary>📝 单个动作如何被执行</summary>

以"头部转动"为例：

```text
apps/test_kuavo_5w_sdk_adapter/sdk/01_head/test_head_control.py
    ↓ 调用测试脚本
adapters.hardware.factory.HardwareFactory
    ↓ 创建 leju_wheeled 硬件适配器
LejuWheeledArmHardware
    ↓ 由 SDKControlMixin 提供 control_head_sdk()
adapters/hardware/leju_wheeled/services/sdk_manager/low_level_sdk_manager.py
    ↓ 调用 kuavo_humanoid_sdk
机器人仿真或真机头部执行动作
```

</details>

<details>
<summary>📝 行为树任务如何执行</summary>

以 `refactored_sdk_atomic_v1` 场景为例：

```text
apps/test_upper_init/run_behavior_tree_json.py
    ↓ 加载 scenario
orchestration/scenarios/refactored_sdk_atomic_v1/py_tree.json
    ↓ 主树引用子树
orchestration/scenarios/refactored_sdk_atomic_v1/py_tree_child.json
    ↓ BehaviorTreeFactory 动态导入节点类
orchestration/nodes/*.py
    ↓ 节点创建并调用 Skill
skills/atomic/refactored_sdk/*.py
    ↓ Skill 调用 hardware.xxx()
adapters/hardware/leju_wheeled/*.py
    ↓ SDK/ROS/TimedCmd
机器人执行动作
```

</details>

<details>
<summary>📝 三种硬件控制路径</summary>

`LejuWheeledArmHardware` 支持三类控制方式：

| 控制方式     | 方法特征                                          | 底层路径                                     | 适合场景                   |
| -------- | --------------------------------------------- | ---------------------------------------- | ---------------------- |
| 标准接口     | `send_base_pose`, `control_head`, `arm_reset` | ROS 话题/服务或封装后的 SDK                       | 普通应用、Skill、行为树         |
| SDK 直调   | `*_sdk`，如 `control_head_sdk`                  | Core SDK Manager -> `kuavo_humanoid_sdk` | 高频控制、SDK 示例、底层验证       |
| TimedCmd | `*_timed`, `send_timed_*`                     | TimedCmdManager -> ROS 服务                | 带时间规划、多规划器、Ruckig、离线轨迹 |

</details>

> 📘 **推荐阅读顺序：** 新手建议按以下顺序阅读和运行：
>
> 1. `apps/test_kuavo_5w_sdk_adapter/README.md`
> 2. `apps/test_kuavo_5w_sdk_adapter/sdk/01_head/`，先跑头部示例
> 3. `core/interfaces/i_hardware.py`，理解硬件能力边界
> 4. `adapters/hardware/leju_wheeled/hardware.py`，理解 mixin 组合方式
> 5. `skills/base/skill_base.py` 和 `skills/atomic/refactored_sdk/`
> 6. `orchestration/nodes/`，理解行为树节点如何调用技能
> 7. `orchestration/scenarios/refactored_sdk_atomic_v1/readme.md`
> 8. `apps/test_upper_init/run_behavior_tree_json.py`，学习 JSON 行为树启动

***

## 🗂️ 项目结构

```text
LeTools
├── core/                        # 核心领域层：接口、数据模型、通用工具
│   ├── interfaces/              # 抽象接口（i_hardware / i_skill / i_camera / i_perception）
│   ├── domain/                  # 数据模型（result / pose / joint_state / trajectory / ...）
│   └── common/                  # 通用工具（logger / config_loader / transform / ...）
├── adapters/                    # 硬件适配层
│   ├── hardware/
│   │   ├── factory.py           # 工厂模式，按 robot_type 创建适配器
│   │   ├── leju_wheeled/        # 乐聚轮臂（当前主力）：多 Mixin 组合 + 相机/感知适配器 + 服务
│   │   └── leju_bipedal/        # 乐聚足式（后续扩展）
│   └── vacuum_control/          # 真空吸盘控制适配器
├── drivers/                     # 底层驱动层
│   └── leju/
│       ├── end_effector.py      # 末端执行器底层封装
│       └── kuavo_humanoid_sdk/  # SDK submodule
├── skills/                      # 原子技能层
│   ├── base/skill_base.py       # 技能生命周期基类
│   └── atomic/
│       ├── refactored_sdk/      # 当前推荐的新 SDK 技能（一文件一技能）
│       ├── manipulation/        # 旧版操作技能
│       ├── motion/              # 旧版运动技能
│       └── perception/          # 感知技能
├── orchestration/               # 行为树编排层
│   ├── engine/                  # JSON -> PyTrees 工厂与控制器
│   ├── nodes/                   # 行为树节点（一个节点调用一个 Skill）
│   ├── scenarios/               # 行为树场景 JSON
│   ├── tasks/                   # 任务级 YAML 配置
│   ├── services/                # 黑板服务
│   ├── config/                  # 行为树配置（boards/skills/trees）
│   └── parser/                  # 任务解析器
├── apps/                        # 示例和测试入口
│   ├── test_kuavo_5w_sdk_adapter/   # 当前推荐的新接口验收测试
│   ├── test_kuavo_5w_internal/      # 偏底层 ROS API 测试
│   ├── test_kuavo_5w_adapter/       # Adapter 层硬件测试
│   ├── test_kuavo_5w_sdk_internal/  # SDK 级测试
│   ├── test_camera_adapter/         # 相机适配器测试
│   ├── test_camera_internal/        # 相机视觉测试
│   ├── test_upper_init/             # JSON 行为树启动器
│   ├── jibot_adapter/               # JiBot 上位机迁移测试（适配器层）
│   └── jibot_internal/              # JiBot 上位机迁移测试（内部接口）
├── infrastructure/              # ROS 基础设施
│   └── ros_packages/            # catkin 工作空间
├── ci_scripts_internal/         # CI/开源构建脚本
├── config/                      # 配置文件
├── scripts/                     # 安装与运行脚本
└── docs/                        # 文档
```

> 📘 一句话总结：`core` 定规则，`adapters` 接硬件，`drivers/infrastructure` 管底层通信，`skills` 封动作，`orchestration` 排任务，`apps` 负责把这些能力跑起来。

<details>
<summary>📖 目录与关键文件说明</summary>

### `core/` 核心领域层

`core` 不直接依赖 ROS 或机器人硬件，负责定义稳定的接口、数据模型和通用工具。

| 路径                                              | 作用                                                  |
| ----------------------------------------------- | --------------------------------------------------- |
| `core/interfaces/i_hardware.py`                 | 最重要的硬件统一接口，定义底盘、手臂、腿部、头部、力控、模式、反馈等能力                |
| `core/interfaces/i_skill.py`                    | 技能接口，规定技能执行、取消、状态查询等契约                              |
| `core/interfaces/i_camera.py`                   | 相机接口，包含采图、流、深度、点云、状态等能力                             |
| `core/interfaces/i_perception.py`               | 感知接口，包含二维码、AprilTag、目标检测、位姿估计等能力                    |
| `core/domain/result.py`                         | `Result` 统一返回对象，所有预期失败尽量用 `success/message/data` 表达 |
| `core/domain/pose.py`                           | `Pose6D` 位姿对象，表示 xyz + roll/pitch/yaw               |
| `core/domain/joint_state.py`                    | 关节状态数据结构                                            |
| `core/domain/trajectory.py`                     | 轨迹点、轨迹序列、插值相关数据结构                                   |
| `core/domain/ruckig_params.py`                  | Ruckig 速度/加速度/急动度限制参数                               |
| `core/domain/camera.py`                         | 相机配置、图像帧、相机状态等                                      |
| `core/domain/perception.py`, `tag.py`           | 感知检测结果、Tag 位姿、二维码/AprilTag 数据                       |
| `core/domain/end_effector.py`                   | 夹爪/灵巧手命令和状态                                         |
| `core/domain/enums.py`                          | 坐标系、MPC 模式、手臂侧别等枚举                                  |
| `core/common/logger.py`                         | 统一日志系统，支持 trace id 和文件日志                            |
| `core/common/config_loader.py`, `app_config.py` | YAML/应用配置加载                                         |
| `core/common/transform.py`, `math_utils.py`     | 位姿转换、数学工具                                           |
| `core/common/interpolator.py`                   | 轨迹插值器                                               |

### `adapters/` 硬件适配层

`adapters` 把 `core/interfaces/i_hardware.py` 中的抽象方法落到具体机器人平台。

| 路径                                                                             | 作用                                                                       |
| ------------------------------------------------------------------------------ | ------------------------------------------------------------------------ |
| `adapters/hardware/factory.py`                                                 | `HardwareFactory`，根据 `robot_type` 创建 `leju_wheeled` 或 `leju_bipedal` 适配器 |
| `adapters/hardware/leju_wheeled/hardware.py`                                   | 轮臂主适配器 `LejuWheeledArmHardware`，通过多 mixin 组合完整能力                         |
| `adapters/hardware/leju_wheeled/camera_adapter.py`                             | 相机适配器，封装图像、深度、点云、TF、状态读取                                                 |
| `adapters/hardware/leju_wheeled/perception_adapter.py`                         | 感知适配器，封装 AprilTag、二维码、目标检测结果                                             |
| `adapters/hardware/leju_wheeled/mixins/lifecycle_mixin.py`                     | 初始化/关闭流程，创建 SDK manager、相机、感知、状态管理器                                      |
| `adapters/hardware/leju_wheeled/mixins/base_control_mixin.py`                  | 底盘速度/位姿标准控制                                                              |
| `adapters/hardware/leju_wheeled/mixins/arm_control_mixin.py`                   | 手臂末端/关节、腿部关节等标准控制                                                        |
| `adapters/hardware/leju_wheeled/mixins/torso_control_mixin.py`                 | 躯干控制                                                                     |
| `adapters/hardware/leju_wheeled/mixins/timed_command_mixin.py`                 | TimedCmd、Ruckig 参数、离线轨迹、IK 可达性                                           |
| `adapters/hardware/leju_wheeled/mixins/sdk_control_mixin.py`                   | SDK 直调能力，提供 `*_sdk` 方法                                                   |
| `adapters/hardware/leju_wheeled/mixins/mode_service_mixin.py`                  | MPC、快速模式、手臂控制模式服务                                                        |
| `adapters/hardware/leju_wheeled/mixins/state_feedback_mixin.py`                | 状态反馈读取                                                                   |
| `adapters/hardware/leju_wheeled/mixins/end_effector_mixin.py`                  | 末端执行器控制                                                                  |
| `adapters/hardware/leju_wheeled/mixins/force_control_mixin.py`                 | 末端期望力、仿真外力、挥空检测                                                          |
| `adapters/hardware/leju_wheeled/mixins/jibot/chassis_mixin.py`                 | JiBot 底盘导航服务适配                                                           |
| `adapters/hardware/leju_bipedal/hardware.py`                                   | 足式机器人适配器入口，目前能力较少                                                        |
| `adapters/hardware/leju_wheeled/services/state_manager.py`                     | 机器人状态管理和缓存                                                               |
| `adapters/hardware/leju_wheeled/services/sdk_manager/base_sdk_manager.py`      | SDK 管理器基础类，负责 SDK 初始化、错误处理、日志等共性                                         |
| `adapters/hardware/leju_wheeled/services/sdk_manager/arm_sdk_manager.py`       | 手臂 SDK 操作封装                                                              |
| `adapters/hardware/leju_wheeled/services/sdk_manager/low_level_sdk_manager.py` | 头部、底盘、腿部、模式等低层 SDK 操作                                                    |
| `adapters/hardware/leju_wheeled/services/sdk_manager/timed_cmd_manager.py`     | TimedCmd、Ruckig、离线轨迹、IK 可达性等高级服务                                         |

### `drivers/` 底层驱动层

| 路径                                 | 作用                                     |
| ---------------------------------- | -------------------------------------- |
| `drivers/leju/end_effector.py`     | 乐聚末端执行器封装，给 adapter 的末端执行器 mixin 使用    |
| `drivers/leju/kuavo_humanoid_sdk/` | SDK submodule 目标目录，安装成功后包含底层 Kuavo SDK |
| `drivers/README.md`                | 驱动层说明                                  |

> 如果 SDK 子模块没有下载成功，`drivers/leju/kuavo_humanoid_sdk` 可能为空或不存在，SDK 控制脚本会 import 失败。

### `skills/` 原子技能层

`skills` 只做"一个动作"的业务封装，不负责行为树结构，也不直接处理 JSON。

| 路径                                                      | 作用                                                                    |
| ------------------------------------------------------- | --------------------------------------------------------------------- |
| `skills/base/skill_base.py`                             | 技能基类，提供 `initialize -> execute -> is_finished -> cancel` 生命周期、超时和异常处理 |
| `skills/atomic/refactored_sdk/`                         | 当前主力 SDK 技能，一文件一技能                                                    |
| `skills/atomic/refactored_sdk/base_pose_local.py`       | 底盘本体系相对位姿                                                             |
| `skills/atomic/refactored_sdk/head_control_sdk.py`      | 头部控制                                                                  |
| `skills/atomic/refactored_sdk/arm_joint_traj_sdk.py`    | 双臂 14 关节轨迹                                                            |
| `skills/atomic/refactored_sdk/arm_ee_traj_local_sdk.py` | 手臂末端本体系轨迹                                                             |
| `skills/atomic/refactored_sdk/arm_ee_traj_world_sdk.py` | 手臂末端世界系轨迹                                                             |
| `skills/atomic/refactored_sdk/arm_reset_sdk.py`         | 手臂复位                                                                  |
| `skills/atomic/refactored_sdk/leg_joint_sdk.py`         | 腿部关节控制                                                                |
| `skills/atomic/refactored_sdk/torso_reset_sdk.py`       | 躯干复位                                                                  |
| `skills/atomic/refactored_sdk/wait_seconds.py`          | 等待指定秒数                                                                |
| `skills/atomic/refactored_sdk/wait_for_enter.py`        | 等待用户按 Enter                                                           |
| `skills/atomic/manipulation/`                           | 旧版操作类技能：手臂、腿部、抓取、放置、躯干、速度等                                            |
| `skills/atomic/motion/`                                 | 旧版运动技能：底盘速度、移动到位姿                                                     |
| `skills/atomic/perception/`                             | 感知技能：相机捕获、二维码读取                                                       |
| `skills/atomic/grasp_skill.py`                          | 旧版抓取技能                                                                |

### `orchestration/` 行为树编排层

`orchestration` 负责从 JSON 构建 PyTrees 行为树，并把节点执行委托给 `skills`。

| 路径                                                         | 作用                                                                |
| ---------------------------------------------------------- | ----------------------------------------------------------------- |
| `orchestration/main.py`                                    | 旧/通用行为树入口，可 dry-run 或真机运行                                         |
| `orchestration/shared_hardware.py`                         | 全局硬件单例，行为树节点共享同一个 `IHardware` 实例                                  |
| `orchestration/studio_tree.py`                             | 行为树服务/Studio 侧入口和树处理逻辑                                            |
| `orchestration/engine/behavior_tree_factory.py`            | JSON -> PyTrees 的核心工厂，负责子树、参数、动态导入节点                              |
| `orchestration/engine/behavior_tree_controller.py`         | 行为树控制器，负责加载树、tick、终态和服务                                           |
| `orchestration/engine/behavior_tree_engine.py`             | 较基础的行为树运行封装                                                       |
| `orchestration/engine/py_trees_compat.py`                  | 兼容不同 py\_trees 版本差异                                               |
| `orchestration/nodes/base_node.py`                         | 行为节点基类，保存 label、params、blackboard 等                               |
| `orchestration/nodes/*_sdk_move.py`                        | 新版 SDK 原子节点，通常一个节点对应一个 `refactored_sdk` 技能                        |
| `orchestration/nodes/async_decorator.py`                   | 异步装饰器，让节点可在线程中执行，配合 Parallel 做真实并行                                |
| `orchestration/nodes/wait_seconds.py`, `wait_for_enter.py` | 等待节点                                                              |
| `orchestration/scenarios/`                                 | 行为树场景目录，每个场景包含 `py_tree.json`, `py_tree_child.json`, `board.json` |
| `orchestration/tasks/`                                     | 任务级 YAML 配置，如 arm\_test、arm\_trajectory、smt\_tray                 |
| `orchestration/services/blackboard_service.py`             | 黑板读写服务封装                                                          |
| `orchestration/utils/blackboard_utils.py`                  | JSON 黑板参数写入 py\_trees blackboard                                  |
| `orchestration/utils/manifest_decorators.py`               | 节点元数据装饰器，给前端/节点库使用                                                |
| `orchestration/utils/node_utils.py`                        | 节点库、参数、类型、manifest 工具函数                                           |
| `orchestration/web_ui/`                                    | 前端/Studio 使用的树和节点 JSON 数据                                         |

### `apps/` 示例和测试入口

`apps` 是新手最适合先看的目录，它不定义框架，而是展示"怎么跑"。

| 路径                                               | 作用                                                  |
| ------------------------------------------------ | --------------------------------------------------- |
| `apps/test_upper_init/run_behavior_tree_json.py` | 推荐的 JSON 行为树启动器                                     |
| `apps/test_kuavo_5w_internal/`                            | 偏底层 ROS API 测试，覆盖底盘、腿部、手臂、定时指令、力控、服务、反馈             |
| `apps/test_kuavo_5w_adapter/`                        | Adapter 层硬件测试，覆盖标准接口、服务、状态反馈和高级定时命令                 |
| `apps/test_kuavo_5w_sdk_internal/`                        | SDK 级测试，包含 SDK 初始化、TimedCmd API、低层 SDK、手臂控制研究       |
| `apps/test_kuavo_5w_sdk_adapter/`                 | 当前推荐的新接口验收测试，按 `sdk/01_head` 到 `sdk/06_feedback` 分类 |
| `apps/test_camera_adapter/`                      | 相机适配器测试：初始化、图像、深度、点云、TF、RViz、感知                     |
| `apps/test_camera_internal/`                       | 相机视觉测试：相机启动、AprilTag、RViz 显示                        |
| `apps/jibot_adapter/`                                    | JiBot 上位机迁移测试（适配器层）：底盘移动、目标点移动、到达检查、速度控制开关          |
| `apps/jibot_internal/`                                   | JiBot 上位机迁移测试（内部接口）：底盘移动、到达检查等底层脚本                    |

### `infrastructure/` ROS 基础设施

| 路径                                                           | 作用                                |
| ------------------------------------------------------------ | --------------------------------- |
| `infrastructure/ros_packages/`                               | catkin 工作空间，执行 `catkin build` 的地方 |
| `infrastructure/ros_packages/src/kuavo_msgs/`                | Kuavo 自定义 ROS 消息/服务               |
| `infrastructure/ros_packages/src/leju_mobile_base_msgs/`     | 移动底盘相关消息                          |
| `infrastructure/ros_packages/src/ocs2_msgs/`                 | OCS2/MPC 相关消息                     |
| `infrastructure/ros_packages/src/kuavo_camera/`              | Kuavo 相机辅助节点和脚本                   |
| `infrastructure/ros_packages/src/kuavo_tf2_web_republisher/` | TF 转发/网页端 TF 数据支持                 |
| `infrastructure/ros_packages/src/OrbbecSDK_ROS1/`            | Orbbec ROS1 驱动                    |
| `infrastructure/ros_packages/src/ros_sensor_integration/`    | RealSense 等传感器集成                  |
| `infrastructure/ros_packages/src/ros_vision/`                | AprilTag、YOLO 等视觉识别包              |
| `infrastructure/ros_packages/src/dynamic_biped/`             | 动态双足/SDK 示例和相关 ROS 包              |

### `config/` 配置文件

| 文件                                  | 作用                |
| ----------------------------------- | ----------------- |
| `config/app_config.yaml`            | 应用级配置，如机器人类型等     |
| `config/robot_config.yaml`          | 机器人参数配置           |
| `config/camera_config.yaml`         | 相机配置              |
| `config/camera_orbbec.launch`       | Orbbec 相机启动配置     |
| `config/apriltag_continuous.launch` | AprilTag 连续检测启动配置 |
| `config/apriltag_settings.yaml`     | AprilTag 检测参数     |
| `config/apriltag_tags.yaml`         | Tag 尺寸、ID 等配置     |
| `config/log_config.yaml`            | 日志配置              |

### `scripts/` 安装与运行脚本

| 文件                                                        | 作用                            |
| --------------------------------------------------------- | ----------------------------- |
| `scripts/install_sdk.sh`                                  | 一键初始化并安装 `kuavo_humanoid_sdk` |
| `scripts/kuavo_humanoid_sdk_tools/setup_sdk.sh`           | SDK 安装主流程                     |
| `scripts/kuavo_humanoid_sdk_tools/init_config.sh`         | SDK 配置初始化                     |
| `scripts/kuavo_humanoid_sdk_tools/sdk_config.sh.template` | SDK 配置模板                      |
| `scripts/run_arm_trajectory.sh`                           | 手臂轨迹运行脚本                      |
| `scripts/run_camera_demo.sh`                              | 相机 demo 启动脚本                  |
| `scripts/run_smt_tray.sh`                                 | SMT tray 场景启动脚本               |

### `docs/` 文档

| 路径                                                        | 作用                 |
| --------------------------------------------------------- | ------------------ |
| `docs/docs_internal/guides/SDK_Integration_Guide.md`      | SDK 安装和集成说明        |
| `docs/docs_internal/guides/LOGGING_*`                     | 日志系统说明             |
| `docs/docs_internal/guides/ROS_ADAPTER_BEST_PRACTICES.md` | ROS 适配器开发规范        |
| `docs/docs_internal/guides/CAMERA_HARDWARE_TEST_GUIDE.md` | 相机硬件测试指南           |
| `docs/docs_internal/architecture/`                        | 行为树、迁移蓝图、测试映射等架构文档 |

</details>

***

## 📚 功能模块导航

|        模块       | 可以做什么                                             | 常用入口                                                           |
| :-------------: | :------------------------------------------------ | :------------------------------------------------------------- |
|   🚗 **底盘运动**   | 底盘速度控制、相对位移、世界坐标移动、旋转、JiBot 导航服务调用                | `apps/test_kuavo_5w_sdk_adapter/sdk/04_base/`, `apps/jibot_adapter/`    |
|   🦾 **手臂控制**   | 双臂 14 关节轨迹、末端局部/世界坐标控制、手臂复位、IK 可达性检查              | `apps/test_kuavo_5w_sdk_adapter/sdk/02_arm/`                    |
|   🦿 **腿部/躯干**  | 腿部 4 关节控制、躯干 6DoF、躯干复位、腿臂并行动作                     | `apps/test_kuavo_5w_sdk_adapter/sdk/03_lower_body/`             |
|   🗣️ **头部控制**  | yaw/pitch 控制、头部居中、左右上下扫描                          | `apps/test_kuavo_5w_sdk_adapter/sdk/01_head/`                   |
|   📷 **相机/视觉**  | Orbbec/RealSense 启动、RGB/深度/点云、AprilTag、YOLO、二维码读取 | `apps/test_camera_adapter/`, `apps/test_camera_internal/`        |
|   ✋ **末端执行器**   | 夹爪/灵巧手控制、抓取、放置、双臂手部姿态                             | `drivers/leju/end_effector.py`                                 |
|    💪 **力控**    | 末端期望力、外力仿真、挥空检测、接触力参数                             | `adapters/hardware/leju_wheeled/mixins/force_control_mixin.py` |
|   🔄 **模式切换**   | MPC 模式、ArmOnly/FullBody、快速模式、手臂控制模式               | `apps/test_kuavo_5w_sdk_adapter/sdk/05_mode/`                   |
|   📊 **状态反馈**   | 关节状态、末端位姿、MPC 状态、力矩、到达时间、调试订阅                     | `apps/test_kuavo_5w_sdk_adapter/sdk/06_feedback/`               |
|   🌳 **行为树任务**  | 从 JSON 加载动作流程，支持顺序、并行、等待、黑板参数、子树复用                | `apps/test_upper_init/run_behavior_tree_json.py`               |
| 📦 **ROS 基础设施** | 消息定义、相机驱动、TF 转发、YOLO/AprilTag、ROS 包编译             | `infrastructure/ros_packages/`                                 |

<details>
<summary>🔧 底盘运动 — 代码位置</summary>

| 层级 | 文件/目录                                                                  | 作用                                                                      |
| -- | ---------------------------------------------------------------------- | ----------------------------------------------------------------------- |
| 接口 | `core/interfaces/i_hardware.py`                                        | 定义 `send_base_velocity`, `send_base_pose`, `send_timed_base_pose` 等底盘接口 |
| 数据 | `core/domain/pose.py`, `core/domain/chassis_options.py`                | 位姿、JiBot 移动参数                                                           |
| 适配 | `adapters/hardware/leju_wheeled/mixins/base_control_mixin.py`          | 标准底盘速度/位置控制                                                             |
| 适配 | `adapters/hardware/leju_wheeled/mixins/sdk_control_mixin.py`           | SDK 直调底盘速度、世界/局部位置                                                      |
| 适配 | `adapters/hardware/leju_wheeled/mixins/timed_command_mixin.py`         | 带时间底盘控制、Ruckig、多指令                                                      |
| 适配 | `adapters/hardware/leju_wheeled/mixins/jibot/chassis_mixin.py`         | JiBot 底盘服务：相对移动、目标点移动、到达检查                                              |
| 技能 | `skills/atomic/refactored_sdk/base_pose_local.py`                      | 底盘本体系相对位姿原子技能                                                           |
| 节点 | `orchestration/nodes/base_pose_local_move.py`, `chassis_short_move.py` | 行为树底盘节点                                                                 |
| 示例 | `apps/test_kuavo_5w_sdk_adapter/sdk/04_base/`                           | 底盘 SDK 验证脚本                                                             |
| 示例 | `apps/jibot_adapter/`                                                          | JiBot 上位机服务迁移测试                                                         |

</details>

<details>
<summary>🔧 手臂控制 — 代码位置</summary>

| 层级     | 文件/目录                                                                      | 作用                                                   |
| ------ | -------------------------------------------------------------------------- | ---------------------------------------------------- |
| 接口     | `core/interfaces/i_hardware.py`                                            | 定义末端位姿、双臂位姿、关节轨迹、IK 等接口                              |
| 数据     | `core/domain/joint_state.py`, `trajectory.py`, `pose.py`                   | 关节状态、轨迹点、位姿                                          |
| SDK 管理 | `adapters/hardware/leju_wheeled/services/sdk_manager/arm_sdk_manager.py`   | 手臂 SDK 调用封装                                          |
| SDK 管理 | `adapters/hardware/leju_wheeled/services/sdk_manager/timed_cmd_manager.py` | TimedCmd 手臂/躯干/底盘规划服务封装                              |
| 适配     | `adapters/hardware/leju_wheeled/mixins/arm_control_mixin.py`               | 标准手臂末端/关节控制                                          |
| 适配     | `adapters/hardware/leju_wheeled/mixins/sdk_control_mixin.py`               | `send_arm_joint_traj_sdk`, `arm_reset`, 头部/下肢 SDK 路径 |
| 技能     | `skills/atomic/refactored_sdk/arm_joint_traj_sdk.py`                       | 14 关节手臂轨迹技能                                          |
| 技能     | `skills/atomic/refactored_sdk/arm_ee_traj_local_sdk.py`                    | 手臂末端本体系轨迹技能                                          |
| 技能     | `skills/atomic/refactored_sdk/arm_ee_traj_world_sdk.py`                    | 手臂末端世界系轨迹技能                                          |
| 技能     | `skills/atomic/refactored_sdk/arm_reset_sdk.py`                            | 手臂复位技能                                               |
| 节点     | `orchestration/nodes/arm_*_sdk_move.py`                                    | 行为树手臂节点                                              |
| 示例     | `apps/test_kuavo_5w_sdk_adapter/sdk/02_arm/`                                | 手臂关节/末端/复位示例                                         |

</details>

<details>
<summary>🔧 腿部和躯干 — 代码位置</summary>

| 层级     | 文件/目录                                                                          | 作用                                            |
| ------ | ------------------------------------------------------------------------------ | --------------------------------------------- |
| 接口     | `core/interfaces/i_hardware.py`                                                | 定义腿部关节、躯干位姿、Timed 下肢/躯干接口                     |
| SDK 管理 | `adapters/hardware/leju_wheeled/services/sdk_manager/low_level_sdk_manager.py` | 低层 SDK 控制：头部、腿部、底盘、模式等                        |
| 适配     | `adapters/hardware/leju_wheeled/mixins/torso_control_mixin.py`                 | 标准躯干位姿和复位相关能力                                 |
| 适配     | `adapters/hardware/leju_wheeled/mixins/sdk_control_mixin.py`                   | `send_leg_joint_sdk`, `send_torso_pose_sdk` 等 |
| 技能     | `skills/atomic/refactored_sdk/leg_joint_sdk.py`                                | 腿部关节技能                                        |
| 技能     | `skills/atomic/refactored_sdk/torso_reset_sdk.py`                              | 躯干复位技能                                        |
| 节点     | `orchestration/nodes/leg_joint_sdk_move.py`, `torso_reset_sdk_move.py`         | 行为树腿部/躯干节点                                    |
| 示例     | `apps/test_kuavo_5w_sdk_adapter/sdk/03_lower_body/`                             | 腿部和躯干示例                                       |

</details>

<details>
<summary>🔧 头部控制 — 代码位置</summary>

| 层级 | 文件/目录                                                          | 作用                            |
| -- | -------------------------------------------------------------- | ----------------------------- |
| 接口 | `core/interfaces/i_hardware.py`                                | 定义 `control_head(yaw, pitch)` |
| 适配 | `adapters/hardware/leju_wheeled/mixins/sdk_control_mixin.py`   | `control_head_sdk` 调用底层 SDK   |
| 技能 | `skills/atomic/refactored_sdk/head_control_sdk.py`             | 头部 yaw/pitch 原子技能             |
| 节点 | `orchestration/nodes/head_control_sdk_move.py`, `move_head.py` | 行为树头部节点                       |
| 示例 | `apps/test_kuavo_5w_sdk_adapter/sdk/01_head/`                   | 头部控制示例                        |

</details>

<details>
<summary>🔧 相机和视觉 — 代码位置</summary>

| 层级    | 文件/目录                                                                   | 作用                    |
| ----- | ----------------------------------------------------------------------- | --------------------- |
| 接口    | `core/interfaces/i_camera.py`, `i_perception.py`                        | 相机采集、流、点云、感知检测接口      |
| 数据    | `core/domain/camera.py`, `perception.py`, `tag.py`                      | 图像帧、相机状态、感知结果、Tag 位姿  |
| 适配    | `adapters/hardware/leju_wheeled/camera_adapter.py`                      | 订阅/等待相机图像、深度、点云、TF、状态 |
| 适配    | `adapters/hardware/leju_wheeled/perception_adapter.py`                  | AprilTag/二维码/目标检测结果封装 |
| 技能    | `skills/atomic/perception/camera_capture/skill.py`                      | 相机捕获技能                |
| 技能    | `skills/atomic/perception/read_qrcode/skill.py`                         | 二维码读取技能               |
| ROS 包 | `infrastructure/ros_packages/src/OrbbecSDK_ROS1/`                       | Orbbec 相机驱动           |
| ROS 包 | `infrastructure/ros_packages/src/ros_sensor_integration/ros_realsense/` | RealSense 相机驱动        |
| ROS 包 | `infrastructure/ros_packages/src/ros_vision/`                           | AprilTag、YOLO、工业检测节点  |
| 示例    | `apps/test_camera_adapter/`                                             | 相机适配层测试               |
| 示例    | `apps/test_camera_internal/`                                              | 相机启动、AprilTag、RViz 显示 |

</details>

<details>
<summary>🔧 末端执行器、抓取和力控 — 代码位置</summary>

| 层级 | 文件/目录                                                                        | 作用                           |
| -- | ---------------------------------------------------------------------------- | ---------------------------- |
| 数据 | `core/domain/end_effector.py`                                                | 夹爪/灵巧手命令和状态                  |
| 驱动 | `drivers/leju/end_effector.py`                                               | 乐聚末端执行器底层封装                  |
| 适配 | `adapters/hardware/leju_wheeled/mixins/end_effector_mixin.py`                | `control_end_effector` 等统一入口 |
| 适配 | `adapters/hardware/leju_wheeled/mixins/force_control_mixin.py`               | 末端期望力、外力、挥空检测、接触力参数          |
| 技能 | `skills/atomic/grasp_skill.py`, `skills/atomic/manipulation/pick/`, `place/` | 抓取/放置相关技能                    |

</details>

<details>
<summary>🔧 模式切换和状态反馈 — 代码位置</summary>

| 层级   | 文件/目录                                                           | 作用                                           |
| ---- | --------------------------------------------------------------- | -------------------------------------------- |
| 数据   | `core/domain/enums.py`                                          | `MPCControlMode`, `ArmSide`, `FrameType` 等枚举 |
| 状态服务 | `adapters/hardware/leju_wheeled/services/state_manager.py`      | 机器人状态缓存、订阅和查询封装                              |
| 适配   | `adapters/hardware/leju_wheeled/mixins/mode_service_mixin.py`   | MPC/快速模式/手臂模式服务调用                            |
| 适配   | `adapters/hardware/leju_wheeled/mixins/state_feedback_mixin.py` | 到达时间、MPC 状态、末端位姿、力矩等反馈                       |
| 示例   | `apps/test_kuavo_5w_sdk_adapter/sdk/05_mode/`                    | 控制模式测试                                       |
| 示例   | `apps/test_kuavo_5w_sdk_adapter/sdk/06_feedback/`                | 状态反馈测试                                       |

</details>

***

## 🌳 行为树与扩展

<details>
<summary>📖 行为树 JSON 如何工作</summary>

一个标准场景目录通常包含：

```text
orchestration/scenarios/<scene_name>/
├── py_tree.json        # 主树
├── py_tree_child.json  # 子树集合
├── board.json          # 黑板初始数据
└── readme.md           # 场景说明
```

**三个 JSON 的分工：**

| 文件                   | 作用                            |
| -------------------- | ----------------------------- |
| `py_tree.json`       | 主流程，定义顶层顺序、并行、子树引用和 Action 节点 |
| `py_tree_child.json` | 子树库，复用一组动作流程                  |
| `board.json`         | 黑板初始数据，如手臂轨迹、全局参数、任务变量        |

`BehaviorTreeFactory` 会根据 JSON 节点的 `name` 字段做判断：

```text
name = "Sequence" / "Parallel" 等
    → 创建 py_trees 复合节点

name = "demo_xxx.json"
    → 在 py_tree_child.json 中寻找子树

name = "HeadControlSdkMove"
    → 在 orchestration/nodes/ 中动态导入同名 Python 类
```

一个节点的执行链路：

```text
JSON params
    ↓
orchestration/nodes/head_control_sdk_move.py
    ↓ 构造 HeadControlSdkParams
skills/atomic/refactored_sdk/head_control_sdk.py
    ↓ 调用 hardware.control_head_sdk()
adapters/hardware/leju_wheeled/mixins/sdk_control_mixin.py
    ↓ 调用 low_level_sdk_manager
adapters/hardware/leju_wheeled/services/sdk_manager/low_level_sdk_manager.py
    ↓ Kuavo SDK
机器人执行头部动作
```

</details>

<details>
<summary>📖 如何新增一个动作或任务</summary>

### 只新增任务流程

如果已有节点和技能，只需要新增 JSON 场景：

```bash
mkdir -p orchestration/scenarios/my_scene
cp orchestration/scenarios/refactored_sdk_atomic_v1/py_tree.json orchestration/scenarios/my_scene/
cp orchestration/scenarios/refactored_sdk_atomic_v1/py_tree_child.json orchestration/scenarios/my_scene/
cp orchestration/scenarios/refactored_sdk_atomic_v1/board.json orchestration/scenarios/my_scene/
```

修改 JSON 后先 dry-run：

```bash
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/my_scene \
  --dry-run --tick-once
```

### 新增一个原子动作

推荐流程：

1. 在 `core/interfaces/i_hardware.py` 确认是否已有硬件接口。
2. 如果没有，在 `IHardware` 增加抽象方法。
3. 在 `adapters/hardware/leju_wheeled/mixins/` 中实现具体硬件调用。
4. 在 `skills/atomic/refactored_sdk/` 新增一个 Skill，继承 `SkillBase`。
5. 在 `orchestration/nodes/` 新增一个行为树节点，继承 `BaseAction`。
6. 使用 `@define_manifest` 描述节点参数，方便前端/节点库识别。
7. 在 `orchestration/scenarios/<scene>/py_tree.json` 引用新节点。
8. 先 `--dry-run --tick-once`，再仿真，最后真机。

推荐命名：

```text
skills/atomic/refactored_sdk/my_action.py       # MyActionSkill
orchestration/nodes/my_action_move.py           # MyActionMove
```

</details>

***

## 💬 常见问题

<details>
<summary>❓ catkin build 报找不到 empy</summary>

```text
Unable to find either executable 'empy' or Python module 'em'
```

处理：

```bash
sudo apt install -y python3-empy
which empy3
python3 -c "import em; print(em.__file__)"
catkin config --cmake-args -DEMPY_EXECUTABLE=/usr/bin/empy3
catkin build
```

</details>

<details>
<summary>❓ 已安装 python3-empy，但报 No module named 'catkin_pkg'</summary>

如果日志中出现 `/home/dsy/anaconda3/bin/python3`，说明 conda Python 抢占了 ROS 编译环境。

```bash
conda deactivate
hash -r
which python3
python3 -c "import catkin_pkg; print(catkin_pkg.__file__)"
source /opt/ros/noetic/setup.bash
catkin config --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3 -DEMPY_EXECUTABLE=/usr/bin/empy3
catkin build
```

</details>

<details>
<summary>❓ realsense2_camera 编译失败</summary>

如果暂时不用 RealSense：

```bash
catkin config --skiplist realsense2_camera kuavo_camera kuavo_tf2_web_republisher
catkin build
```

需要 RealSense 时再安装 `librealsense2-dev librealsense2-utils`，如果 apt 找不到包，需要先配置 Intel RealSense 源。

</details>

<details>
<summary>❓ scripts/install_sdk.sh 找不到</summary>

`install_sdk.sh` 在项目根目录，不在 ROS 工作空间里。

```bash
cd ~/LeTools
chmod +x scripts/install_sdk.sh
./scripts/install_sdk.sh
```

</details>

<details>
<summary>❓ SDK submodule 克隆超时</summary>

典型错误：

```text
fatal: 无法访问 'https://gitcode.com/OpenLET/kuavo-ros-opensource.git/'：Operation timed out
SDK 目录不存在: drivers/leju/kuavo_humanoid_sdk/src/kuavo_humanoid_sdk
```

原因是网络访问 Gitee 子模块失败。先测试：

```bash
git ls-remote https://gitcode.com/OpenLET/kuavo-ros-opensource.git
```

如果也超时，需要换网络、配置代理，或使用团队提供的可访问镜像/压缩包。脚本失败后建议恢复主仓库状态：

```bash
cd ~/LeTools
git sparse-checkout disable 2>/dev/null || true
git submodule deinit -f drivers/leju/kuavo_humanoid_sdk 2>/dev/null || true
rm -rf drivers/leju/kuavo_humanoid_sdk
rm -rf .git/modules/drivers/leju/kuavo_humanoid_sdk
```

</details>

***

## 💬 支持与反馈

我们鼓励反馈问题，使之可被搜索、归档，也方便后来者复用。

### 📝 提交渠道

| 角色 | 提交方式 |
|:---|:---|
| 🧑‍💻 **外部用户** | 前往 [GitCode Issues](https://gitcode.com/OpenLET/letools_opensource/issues) 使用 Issue 模板填写完整信息 |
| 🏢 **乐聚员工** | 通过飞书的LeTools问题反馈表单提交 |

> 📋 **员工提交必填字段：** 问题提出人 · 问题类型 · LeTools 版本/环境 · 具体内容等

### 🔄 处理流程

| 阶段 | 说明 |
|:---|:---|
| 📝 **提交问题** | 通过上述对应渠道提交，附上完整信息 |
| 👤 **管理员分配** | 管理员在 **1 个工作日内** 评估优先级、指定责任人并设定时间节点 |
| 🔧 **处理与反馈** | 责任人在「工作进度/受理意见」中更新处理进展 |
| ✅ **解决归档** | 完成后填写「交付意见」并标记「是否解决」 |

## 📄 许可证

本项目由 [乐聚机器人](https://www.lejurobot.com/zh) 维护。
