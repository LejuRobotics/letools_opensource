# 12 真机小件上料模仿学习-grasp_ring_pick_place_real

[返回文档首页](../README.md) · [04-简单案例运行](../04-简单案例运行/) · [案例库](案例库/README.md) · [03-接口详细文档](../03-接口详细文档/)

**难度**：⭐⭐⭐⭐

**适合读者**：已完成 LeTools 基础环境搭建，想在 V62 轮臂真机上运行圆环抓取、搬运和放置案例的人。

**前置知识**：行为树与黑板（[03-行为树最小示例](03-行为树最小示例.md)）；头、手、躯干与底盘编排（[09-头手躯干与底盘并行](09-头手躯干与底盘并行-instructional_case_04.md)）；完整任务流程（[10-搬箱场景](10-搬箱场景-grasp_mtbf.md)）；模型与行为树衔接（[11-小件上料模仿学习](11-小件上料模仿学习-grasp_ring_pick_place.md)）；ROS 网络及 Python 虚拟环境的基本用法。

**核心概念**：下位机运行运控，上位机运行 LeTools 行为树及推理客户端，PI05 服务器加载模型权重并提供推理服务。

**学习目标**：

1. 完成下位机、上位机与 PI05 服务器的环境部署；
2. 在 `board.json` 中配置运行路径、各阶段动作和推理方式；
3. 检查行为树加载，并运行真机圆环抓取、搬运和放置流程。

**进阶路线**：[09](09-头手躯干与底盘并行-instructional_case_04.md)（多部位编排）→ [10](10-搬箱场景-grasp_mtbf.md)（完整搬运流程）→ [11](11-小件上料模仿学习-grasp_ring_pick_place.md)（仿真模仿学习）→ **本篇**（真机案例部署与运行）。

**路径基准**：上位机命令在 LeTools 仓库根目录执行；下位机命令在 `kuavo-ros-opensource` 仓库根目录执行；服务器命令在服务器的 `LeTools-Learning` 仓库根目录执行。代码块中出现 `cd` 时，后续命令按切换后的目录执行。

---

## 一、真机环境部署

将仓库按下面步骤部署到真机对应的下位机与上位机。

### 1.1 下位机

**拉取代码并进入仓库**

```bash
# 拉取tag1.4.5的kuavo-ros-opensource仓库并进入
git clone --depth 1 --branch 1.4.5 --single-branch https://gitcode.com/OpenLET/kuavo-ros-opensource.git && cd kuavo-ros-opensource
```

**应用真机圆环补丁**

将附件 `kuavo_1.4.5_ring_real.patch` 放到下位机仓库根目录，执行：

```bash
#应用真机圆环补丁
git apply --check kuavo_1.4.5_ring_real.patch
git apply kuavo_1.4.5_ring_real.patch
```

补丁会将 V62 左右末端设置为乐聚夹爪，并为可视化模块增加模拟夹爪反馈开关。

**关闭模拟夹爪反馈**

打开下位机仓库中的文件：

```text
# 启动文件路径
src/humanoid-control/humanoid_controllers/launch/load_kuavo_real_wheel.launch
```

在 `<launch>` 下方添加：

```xml
<param name="/publish_fake_claw_state" type="bool" value="false"/>
```

保存后，启动运控时可视化模块就不会再发布模拟夹爪状态。

在下位机仓库根目录，切换到 root 用户并编译：

```bash
#编译
sudo su
catkin clean -y
catkin init
catkin config -DCMAKE_ASM_COMPILER=/usr/bin/as -DCMAKE_BUILD_TYPE=Release
source installed/setup.bash
catkin build humanoid_controllers
```

### 1.2 上位机

上位机运行 LeTools 行为树和推理客户端，通过 ROS 与下位机通信。

**拉取代码并进入仓库**

```bash
# 拉取dev分支的LeTools仓库并进入，确认提交
git clone --depth 1 --branch dev --single-branch https://www.lejuhub.com/highlydynamic/LeTools.git && cd LeTools
git fetch --depth 1 origin 5f9fcbf4
git checkout 5f9fcbf4
```

**初始化子模块（下载 LeTools-Learning 仓库）**

```bash
# 初始化子模块
git submodule update --init --recursive
```

**进入 LeTools-Learning 仓库，检查并应用补丁**

附件中的 `letools_learning_ring_real.patch` 位于：

```text
# 补丁路径
orchestration/scenarios/grasp_ring_pick_place_real/配套源码/
```

```bash
# 检查应用补丁
cd LeTools-Learning

git apply --check \
  ../orchestration/scenarios/grasp_ring_pick_place_real/配套源码/letools_learning_ring_real.patch
git apply \
  ../orchestration/scenarios/grasp_ring_pick_place_real/配套源码/letools_learning_ring_real.patch

cd ..
```

**创建并激活 Python 环境**

应用 Learning 补丁后，返回 LeTools 仓库根目录执行：

```bash
# LeTools根目录下创建激活环境
source /opt/ros/noetic/setup.bash
bash LeTools-Learning/tools/ring_real/setup_env.sh
source .venv-ring-real/bin/activate
```

脚本在 LeTools 根目录创建 `.venv-ring-real` 虚拟环境，并安装行为树及 PI05 客户端所需依赖。该环境使用 Python 3.8，需已有 ROS Noetic、python3-venv 及机器人镜像配套的 Torch、torchvision、OpenCV 和 cv_bridge。脚本不安装 PI05 服务器或 ACT 独立推理环境。

**编译相关ros包**

```bash
# 编译相关ros包
cd infrastructure/ros_packages
catkin clean -y
catkin init
catkin config -DCMAKE_ASM_COMPILER=/usr/bin/as -DCMAKE_BUILD_TYPE=Release
catkin build
source devel/setup.bash
cd ../..
```

**安装 SDK 并验证**

```bash
# 安装 SDK 并验证
bash scripts/sdk_tools/install.sh
bash scripts/sdk_tools/version.sh
python3 -c 'from kuavo_humanoid_sdk import KuavoRobot; print("SDK Ready!")'
```

### 1.3 推理服务器

**拉取代码并进入仓库**

推理服务器使用最新版 `LeTools-Learning` 仓库，负责加载 PI05 模型权重并提供推理服务。服务器环境与机器人上位机的 ROS 客户端环境分别部署。

在服务器终端下载仓库：

```bash
# 下载LeTools-Learning仓库并下载其子模块
git clone --branch dev https://github.com/LejuRobotics/LeTools-Learning.git
cd LeTools-Learning
git submodule update --init --recursive
```

按仓库中 OpenPI 的安装说明准备 uv 和模型运行环境。后续启动命令使用 `--no-sync`，因此必须提前完成依赖安装。

下载本案例配套配置及完整模型权重，并将配置文件放到以下路径：

```text
# 客户端与服务端配置文件
configs/server/pi05_dagger_1015_mixed.yaml
configs/deploy/pi05_dagger_1015_mixed.yaml
```

第一份用于推理服务器，第二份用于独立客户端测试。模型权重需要单独下载，并将服务器配置中的权重路径修改为实际存放位置。核对服务端与客户端的连接地址、端口和模型输入输出配置。

## 二、真机执行

整体行为树流程为：

```text
预抓取 → 推理抓取 → 模型停止与控制交接 → 搬运及底盘去程 → 放置 → 后撤与归位 → 底盘返程
```

### 2.1 真机执行仓库参数修改

打开 LeTools 仓库中的场景配置：

```text
# 场景配置路径
orchestration/scenarios/grasp_ring_pick_place_real/board.json
```

按实际部署修改以下参数：

以下默认值取自当前案例的 `board.json`。路径按实际部署修改，动作姿态、距离和速度由现场人员确认。

| 参数 | 当前默认值 | data_type | 含义 |
|---|---|---|---|
| `expected_lower_repo` | `/home/lab/example/kuavo-ros-1.4.5-reference` | `string` | 下位机正在运行的运控仓库的绝对路径。 |
| `ros_python` | `/media/data/example/letools-env/letools-ros/bin/python` | `string` | 启动行为树和 PI05 客户端使用的 Python 解释器路径，必须与激活的虚拟环境一致。 |
| `pi05_sdk` | `existing` | `string` | 使用 pi05_paths.existing_sdk 指定的 SDK。 |
| `pi05_paths.existing_sdk` | `/media/data/example/LeTools/diagnostics/reference_sdk` | `string` | 上位机 SDK 根目录，其中应包含 kuavo_humanoid_sdk/，需按实际安装路径修改。 |
| `pi05_paths.kuavo_common_python` | `/media/data/example/LeTools/drivers/leju/kuavo_humanoid_sdk/kuavo_common/python` | `string` | SDK 配套的 kuavo_common/python 目录，需按实际路径修改。 |
| `pi05_paths.vendor_root` | `LeTools-Learning/tools/ring_real/vendor_pi05` | `string` | 行为树使用的 PI05 适配客户端源码目录。 |
| `pi05_paths.deploy_config` | `LeTools-Learning/tools/ring_real/vendor_pi05/configs/deploy/steam_tln_zhiqiang_pi05.yaml` | `string` | 行为树使用的 PI05 客户端配置文件；与独立 eval.py 的配置分别管理。 |
| `inference_backend` | `pi05` | `string` | 推理方式：`pi05`、`act` 或 skip。 |
| `act_python` | `/media/data/example/letools-env/act-jp5-test/bin/python` | `string` | ACT 推理使用的 Python 解释器绝对路径，仅使用 ACT 时需要配置。 |
| `act_runtime.root` | `/media/data/example/act-orin-standalone` | `string` | ACT 部署代码目录，需包含 standalone_policy.py 和 act_core.py。 |
| `act_runtime.checkpoint` | `/media/data/example/act-orin-standalone/pretrained_model` | `string` | ACT 完整检查点目录，保留权重、配置及其引用的预处理和后处理统计文件。 |
| `act_runtime.device` | `cuda` | `string` | ACT 推理设备，可设为 cuda 或 cpu。 |
| `prepare.arm_waypoints_deg` | 两个路点，见 `board.json` | `array<array<float>>` | 预抓取双臂关节路点，按列表顺序执行；每点前 7 个值为左臂、后 7 个为右臂，单位：度。 |
| `prepare.claw_percent` | `[5.0, 5.0]` | `array<float>` | 左右夹爪目标闭合比例，单位：%。当前保持夹爪状态，不下发此目标。 |
| `integrated.prepare_torso_deg` | `[0.202, -0.098, -0.048, -0.153]` | `array<float>` | 预抓取躯干四个关节的目标角度，单位：度。 |
| `integrated.head_yaw_pitch_deg` | `[0.0, 25.0]` | `array<float>` | 预抓取头部偏航和俯仰目标角度，单位：度；当前机器正俯仰为低头方向。 |
| `integrated.prepare_arm_speed_deg_s` | `90.0` | `float` | 预抓取手臂规划速度，单位：度/秒。 |
| `integrated.prepare_torso_speed_deg_s` | `15.0` | `float` | 预抓取躯干规划速度，单位：度/秒。 |
| `integrated.head_speed_deg_s` | `20.0` | `float` | 头部目标插值速度，单位：度/秒。 |
| `real_ring_options.keep_claws_current` | `true` | `bool` | 预抓取不发送双夹爪目标命令，保持当前夹爪状态；仍检查真实反馈。 |
| `pi05_runtime.ros_rate_hz` | `10` | `int` | PI05 模型观测和策略步频，单位：Hz，填写整数。 |
| `pi05_runtime.control_rate_hz` | `100` | `int` | PI05 动作插值和控制频率，单位：Hz，填写整数。 |
| `pi05_runtime.max_episode_steps` | `20000` | `int` | PI05 单次任务允许的最大模型步数。 |
| `pi05_runtime.task_prompt` | `grab the steel hoop and put it on the right plane` | `string` | 发送给模型的任务文本，应与使用的模型及任务一致。 |
| `grasp.timeout_enabled` | `true` | `bool` | 是否启用正常抓取超时计时。 |
| `grasp.timeout_s` | `50.0` | `float` | 模型推理就绪后允许等待抓取确认的时间，单位：秒。 |
| `grasp.startup_timeout_s` | `90.0` | `float` | PI05 客户端启动和推理就绪等待上限，单位：秒；启动超时结束任务。 |
| `grasp.timeout_outcome` | `timeout_skip` | `string` | 正常抓取超时后，只有模型清理和控制交接通过才继续后续流程，不标记抓取成功。 |
| `grasp.position_detector.enabled` | `true` | `bool` | 是否启用右夹爪位置稳定抓取检测。 |
| `grasp.position_detector.min_percent` | `75.0` | `float` | 抓取检测允许的右夹爪闭合比例下限，单位：%。 |
| `grasp.position_detector.max_percent` | `99.5` | `float` | 抓取检测允许的右夹爪闭合比例上限，单位：%。 |
| `grasp.position_detector.range_percent` | `2.0` | `float` | 连续检测窗口内，右夹爪位置最大值与最小值之差的上限，单位：百分点。 |
| `grasp.position_detector.hold_s` | `1.5` | `float` | 右夹爪位置持续满足范围和稳定条件的窗口时长，单位：秒。 |
| `grasp.position_detector.handoff_max_opening_change_percent` | `3.0` | `float` | 模型停止及交接期间，右夹爪闭合比例相对触发时允许下降的最大值，单位：百分点。 |
| `grasp.effort_detector.enabled` | `false` | `bool` | 是否启用努力值抓取检测；当前未启用。 |
| `grasp.feedback_max_age_s` | `0.3` | `float` | 夹爪反馈时效上限，单位：秒；反馈失效结束整轮任务。 |
| `limits.feedback_age_s` | `0.3` | `float` | 关节反馈时效上限，单位：秒；反馈中断或没有新的有效样本会触发整轮中断。 |
| `carry.arm_waypoints_deg` | 两个路点，见 `board.json` | `array<array<float>>` | 搬运双臂关节路点，单位：度；左臂是否先移动再保持由 carry.move_left_to_initial 控制。 |
| `carry.torso_deg` | `[0.202, -0.098, -0.048, -0.153]` | `array<float>` | 搬运时躯干四个关节的目标角度，单位：度。 |
| `integrated.carry_arm_speed_deg_s` | `90.0` | `float` | 搬运手臂规划速度，单位：度/秒。 |
| `integrated.carry_torso_speed_deg_s` | `10.0` | `float` | 搬运躯干规划速度，单位：度/秒。 |
| `real_ring_options.hold_left_current` | `true` | `bool` | 启用后续阶段的左臂保持策略，结合 carry.move_left_to_initial 和最终归位配置使用。 |
| `carry.move_left_to_initial` | `true` | `bool` | 搬运前先将左臂移动至配置的搬运路点，再保持。 |
| `carry.base_local_m_deg` | `[-1.0, -1.0, -90.0]` | `array<float>` | 底盘相对搬运起点的 [X, Y, 航向角] 目标；前两项单位：米，航向角单位：度。 |
| `integrated.base_linear_speed_m_s` | `0.4` | `float` | 底盘平移速度上限，单位：米/秒。 |
| `integrated.base_angular_speed_rad_s` | `0.6` | `float` | 底盘旋转速度上限，单位：弧度/秒。 |
| `carry.return_to_grasp` | `true` | `bool` | 后撤与归位完成并通过交接检查后，执行底盘返程，返回本轮记录的起点。 |
| `integrated.place_position_mode` | `relative_to_current` | `string` | 放置位置计算方式；当前使用相对当前位置模式。 |
| `integrated.place_forward_m` | `0.3` | `float` | 相对位置模式下右手的前伸距离，单位：米。 |
| `integrated.place_height_mode` | `fixed` | `string` | 放置高度模式：`fixed` 使用配置高度，`current` 保持当前高度。 |
| `integrated.place_height_m` | `0.8` | `float` | 固定高度模式下的右手放置目标高度，单位：米。 |
| `integrated.place_orientation_mode` | `fixed_zyx` | `string` | 放置末端姿态模式；当前使用配置的固定 ZYX 姿态。 |
| `integrated.place_yaw_pitch_roll_deg` | `[99.6315, -2.693, -29.8902]` | `array<float>` | 固定姿态模式下的末端 [偏航角, 俯仰角, 滚转角]，单位：度。 |
| `place.move_duration_s` | `1.5` | `float` | 放置前伸请求的规划时间，单位：秒。 |
| `place.open_percent` | `0.0` | `float` | 放置时右夹爪目标闭合比例，单位：%；`0` 表示张开。 |
| `integrated.release_wait_for_still` | `false` | `bool` | 是否在放置到位后的开爪前后等待手臂稳定；当前关闭该项等待，开爪时手臂可能仍在运动。 |
| `place.retreat_m` | `0.2` | `float` | 放置后右手后撤距离，单位：米。 |
| `place.retreat_duration_s` | `1.5` | `float` | 后撤请求的规划时间，单位：秒。 |
| `place.return_waypoints_deg` | 两个路点，见 `board.json` | `array<array<float>>` | 双臂归位路点，单位：度；最终双臂归位由 place.home_both_after_initial 控制。 |
| `integrated.return_arm_speed_deg_s` | `90.0` | `float` | 归位前段手臂路点的规划速度，单位：度/秒。 |
| `integrated.home_final_arm_speed_deg_s` | `90.0` | `float` | 最后一个双臂归位路点的规划速度，单位：度/秒。 |
| `place.home_both_after_initial` | `true` | `bool` | 后撤后经过 initial 姿态，再执行最终双臂归位。 |
| `integrated.home_torso_deg` | `[0.507, -0.336, -0.309, 0.0]` | `array<float>` | 归位躯干四个关节的目标角度，单位：度。 |
| `integrated.home_torso_speed_deg_s` | `8.0` | `float` | 归位躯干规划速度，单位：度/秒。 |
| `real_ring_options.no_soft_pause` | `true` | `bool` | 失败时不请求整机软暂停；故障仍结束本轮行为树，不支持恢复旧任务继续执行。 |

#### 配置运行路径

以下路径必须根据实际部署位置修改：

`expected_lower_repo`：下位机正在运行的运控仓库绝对路径。在下位机进入该仓库，执行 `pwd`，将输出填入此字段。以下配置均修改真机场景的 `board.json`。上位机命令在 LeTools 仓库根目录执行。`pi05_paths.vendor_root` 和`pi05_paths.deploy_config` 使用案例提供的相对路径，按默认目录部署时无需修改。

`ros_python`：上位机启动行为树使用的 Python 解释器。激活部署时创建的环境后执行以下命令，将输出填入此字段：

```bash
source .venv-ring-real/bin/activate
python -c 'import sys; print(sys.executable)'
```

使用 PI05 时，将 `pi05_sdk` 保持为 existing，并配置两个 SDK 路径。

在上位机已激活的环境中，查询 SDK 包根目录：

```bash
python - <<'PY'
import importlib.util
from pathlib import Path

spec = importlib.util.find_spec("kuavo_humanoid_sdk")
if spec is None or spec.origin is None:
    raise RuntimeError("未找到 SDK，请先完成 SDK 安装")
print(Path(spec.origin).resolve().parent.parent)
PY
```

将输出填入 `pi05_paths.existing_sdk`。

加载上位机 ROS 工作空间后，查询 SDK 配套代码：

```bash
source /opt/ros/noetic/setup.bash
source infrastructure/ros_packages/devel/setup.bash

rospack find kuavo_common
```

检查输出目录中是否存在 `python/robot_version.py`，将这个 python 目录的绝对路径填入 `pi05_paths.kuavo_common_python`。若上位机没有该目录，需将下位机运控仓库的 `src/kuavo_common/python` 复制到上位机，再填写复制后的目录路径。

#### 设置各阶段动作

根据现场机器人和物品摆放位置，修改 `board.json` 中以下字段。字段含义、数据类型和单位见上表。`py_tree.json` 和 `py_tree_child.json`保持案例默认配置。

预抓取：修改 `prepare.arm_waypoints_deg`、`integrated.prepare_torso_deg` 和`integrated.head_yaw_pitch_deg`。手臂路点按列表顺序执行，当前先经过 initial，再到预抓取姿态；左右臂均可配置。当前`real_ring_options.keep_claws_current`=true，预抓取保持双夹爪当前状态，不发送 `prepare.claw_percent` 中的目标，但仍检查夹爪反馈。

搬运：修改 `carry.arm_waypoints_deg`、`carry.torso_deg` 和`carry.base_local_m_deg`。底盘目标为相对本轮搬运起点的 [X, Y, 航向角]，前两项单位为米，航向角单位为度。当前`carry.move_left_to_initial`=true，搬运前先将左臂移动到配置的搬运路点，再保持。

放置：先确认放置位置、高度和姿态模式。当前配置为相对当前位置前伸、固定高度和固定 ZYX 姿态，对应修改`integrated.place_forward_m`、`integrated.place_height_m`、`integrated.place_yaw_pitch_roll_deg` 和 `place.move_duration_s`。当前 `integrated.release_wait_for_still`=false，开爪前后不等待该项手臂稳定检查，开爪时手臂可能仍在运动。

后撤：修改 `place.retreat_m` 和 `place.retreat_duration_s`，分别设置后撤距离和规划时间。

归位：修改 `place.return_waypoints_deg` 和`integrated.home_torso_deg`。当前`place.home_both_after_initial`=true，后撤后经过 initial 姿态，再执行最终双臂归位。

底盘返程：当前 `carry.return_to_grasp`=true，后撤与归位完成并通过交接检查后，返回本轮记录的底盘起点，无需另行填写返程坐标。

运动速度：修改参数表中对应阶段的速度字段。归位前段手臂路点使用`integrated.return_arm_speed_deg_s`，最后一个双臂归位路点使用`integrated.home_final_arm_speed_deg_s`。

左臂行为由 `real_ring_options.hold_left_current`、`carry.move_left_to_initial` 和 `place.home_both_after_initial`共同决定。当前搬运前先移动到配置路点，随后保持，最终按双臂归位配置执行。

#### 2.1.1 使用 PI05 推理

在 `board.json` 设置：

```json
"inference_backend": "pi05"
```

将`pi05_paths.existing_sdk` 和 `pi05_paths.kuavo_common_python` 修改为上位机实际存在的目录，SDK 应与下位机运控版本配套。

推理服务器按 2.2.2 启动。行为树会自行启动`pi05_paths.deploy_config` 指定配置的适配客户端，无需手动运行独立客户端。

当前上位机客户端默认连接 `localhost:5555`。如果通过 SSH 转发连接服务器，在上位机另开终端执行，替换用户名和服务器 IP，并保持终端运行：

```bash
# 端口映射
ssh -N -o ExitOnForwardFailure=yes \
  -L 127.0.0.1:5555:127.0.0.1:5555 \
  用户名@服务器IP
```

#### 2.1.2 使用 ACT 推理

在 `board.json` 设置：

```json
"inference_backend": "act"
```

同时修改

```text
act_python：ACT 环境的 Python 解释器绝对路径。
act_runtime.root：包含 standalone_policy.py 和 act_core.py 的部署代码目录。
act_runtime.checkpoint：完整 ACT 检查点目录，保留权重、配置及其引用的预处理和后处理统计文件。
act_runtime.device：推理设备，选择 cuda 或 cpu。
```

#### 2.1.3 跳过模型推理

在 `board.json` 设置：

```json
"inference_backend": "skip"
```

skip 不启动模型，也不等待 50 秒抓取超时，直接跳过抓取阶段，继续执行搬运、放置、后撤归位及已开启的底盘返程。它仍会执行机器人动作，不等同于 `--dry-run`。

### 2.2 真机执行运行命令

#### 2.2.1 下位机

在下位机 kuavo-ros-opensource 仓库根目录执行：

```bash
# 启动运控
sudo su
source devel/setup.bash

export ROBOT_VERSION=62
export ROS_MASTER_URI=http://192.168.26.1:11311
export ROS_IP=192.168.26.1
unset ROS_HOSTNAME

roslaunch humanoid_controllers load_kuavo_real_wheel.launch
```

按终端提示完成启动，并保持该终端运行。启动文件应已按部署步骤设置 `/publish_fake_claw_state=false`。

#### 2.2.2 推理服务器（使用 PI05 时必需）

在服务器的 `LeTools-Learning` 仓库根目录执行：

```bash
# 启动推理服务
CUDA_VISIBLE_DEVICES=0 PYTHONUNBUFFERED=1 uv run --no-sync --project kuavo_model/external_models/openpi python kuavo_server/launch.py openpi --config configs/server/pi05_dagger_1015_mixed.yaml
```

等待模型权重加载完成、服务进入就绪状态，并保持该终端运行。服务端监听端口以配置文件和启动输出为准。

#### 2.2.3 上位机

**激活环境**

在 LeTools 仓库根目录执行：

```bash
# 进入仓库并激活环境
source /opt/ros/noetic/setup.bash
source .venv-ring-real/bin/activate
source infrastructure/ros_packages/devel/setup.bash
export ROS_MASTER_URI=http://192.168.26.1:11311
export ROS_IP=192.168.26.12
unset ROS_HOSTNAME
```

以上`ROS_IP` 按现场网络修改。使用 PI05 时，先确认服务器推理服务和端口连接已准备完成。

**准备夹爪**

首次空载测试时，发送双爪打开命令：

```bash
# 发布夹爪打开命令
rosservice call /control_robot_leju_claw \
  "data:
    name: ['left_claw', 'right_claw']
    position: [5.0, 5.0]
    velocity: [50.0, 50.0]
    effort: [1.0, 1.0]"
```

查看夹爪反馈：

```bash
# 查看夹爪反馈
rostopic echo -n 1 /leju_claw_state
```

确认左右夹爪实际已打开，并检查两侧反馈状态。例如下面显示左右状态均为 2，实测位置分别为 10.66% 和 9.61%。服务返回 `success: True` 表示命令已接受，实际位置仍以反馈和现场观察为准。若任一侧状态为 0，先检查反馈再继续。

```yaml
# 夹爪反馈示例
state: [2, 2]      # 不再为0
data:
  name:
    - left_claw
    - right_claw
  position: [10.66, 9.61]
```

**检查行为树加载**

在上位机 LeTools 仓库根目录，完成环境激活和 ROS 网络配置后执行：

```bash
# 行为树dry-run
python -B -u apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/grasp_ring_pick_place_real \
  --dry-run --tick-once
```

如果输出包含：

```text
[apps][dry-run] tick 后根状态: Status.SUCCESS
```

表示行为树配置和节点加载通过。干跑不执行机器人动作，也不验证模型权重、服务器连接或真机完整流程。

**启动行为树**

确认机器人静止稳定、运动范围清空且现场可操作急停后，执行：

```bash
# 行为树启动命令
python -B -u apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/grasp_ring_pick_place_real
```

**停止行为树**

> **紧急情况使用急停按钮！**

需要主动结束任务时，可在行为树终端按 Ctrl+C，或在另一个已配置相同 ROS 网络的上位机终端调用停止服务：

```bash
# 行为树停止命令
rosservice call /stop_behavior_tree "{}"
```

`ROS_MASTER_URI` 和 `ROS_IP` 按现场网络修改。请求后请观察行为树终端输出，并确认机器人实际已停止。

## 三、测试指南

## 四、附件
