# 11 小件上料模仿学习-grasp_ring_pick_place

[返回文档首页](../README.md) · [04-简单案例运行](../04-简单案例运行/) · [03-接口详细文档](../03-接口详细文档/)

**难度**：⭐⭐⭐⭐

**适合读者**：已经跑通 07–10 篇，想完成一次「仿真采集数据 → 训练 ACT 模型 → 在行为树中调用模型」完整模仿学习流程的人。

**前置知识**：行为树的基本结构（[03-行为树最小示例](03-行为树最小示例.md)）；已运行过前序案例（[07-手臂与头部并行](07-手臂与头部并行-instructional_case_02.md)–[10-搬箱场景](10-搬箱场景-grasp_mtbf.md)）；仿真容器的启动与编译（[安装步骤 1.1](../02-环境安装/03-安装步骤.md#11-环境部署)）；Conda 的基本用法。

**核心概念**：行为树主程序运行在宿主机系统 Python 3.8（ROS Noetic）中，ACT 推理运行在独立的 Conda Python 3.12 环境（`LeTools-Learning`）中。

**学习目标**：

1. 准备两套 Python 环境，并应用下位机补丁；
2. 在 MuJoCo 中采集圆环抓取数据，转换为 LeRobot 数据集并训练 ACT 模型；
3. 单独验证模型推理，再通过 `run_behavior_tree_json.py` 启动主行为树，完成「预抓取 → 模型抓取 → 搬运 → 放置 → 归位」。

---

请先完成其他案例的环境搭建，并将上位机仓库 letools\-opensource 和下位机仓库 kuavo\-ros\-opensource 放在同一目录下。本文以 \~/example/ 为例；如果使用其他目录，请将后续命令中的路径替换为实际路径。



上位机使用配套源码；LeTools\-Learning 需要安装圆环仿真工具附件；kuavo\-ros\-opensource 需要应用下位机补丁。

# **场景定位**

|维度|值|
|---|---|
|场景目录|`orchestration/scenarios/grasp_ring_pick_place/`|
|运行环境|仅 MuJoCo 仿真（`ROBOT_VERSION=62`，biped_s62 机型）|
|任务|机器人从料箱中抓取圆环，底盘搬运到工作台，放置后归位|
|抓取方式|ACT 模仿学习模型|
|参与的仓库|`letools-opensource`（行为树、场景配置、统一入口）、`kuavo-ros-opensource`（MuJoCo、模型、场景、ROS 接口）、`LeTools-Learning`（数据转换、训练与推理，作为上位机子模块）|
|行为树入口|`apps/test_upper_init/run_behavior_tree_json.py`|
|运行配置|行为树入口与黑板使用 `board.json`，动作参数使用 `py_tree_child.json`，推理参数使用 `LeTools-Learning/tools/ring_sim/config/ring_deploy.yaml`|

## **行为树流程**

```text
grasp_ring_pick_place（Sequence）
├── pre_grasp：张开夹爪，低头，调整躯干与双臂到预抓取位
├── inference_control：启动 ACT 推理，结合夹爪反馈判断抓取状态
├── pose_after_grasp：双臂切换到搬运姿态
├── nav_and_lift：躯干回正，底盘移动至放置区域
└── place_ring：前伸放置、张开夹爪、后撤并归位
```

行为树主程序运行在宿主机系统 Python 3.8 中，运行到推理节点时使用 `paths.learning_python` 启动 ACT 推理子进程；仿真机器人运行在 Docker 容器中。

# **一、基础环境准备**

## **1\.1 运行环境预览**

|组件|版本|
|---|---|
|系统 Python（环境 A）|3\.8（ROS Noetic 依赖）|
|Conda Python（环境 B）|3\.12（LeTools\-Learning）|
|Conda 环境名|letools（独立于 base）|
|ROBOT\_VERSION|62（biped\_s62 机型）|
|PyTorch|2\.9\.1|
|LeRobot|0\.5\.2|
|kuavo\_humanoid\_sdk|1\.4\.5；基础环境已预装，使用配套源码。|
|Docker 镜像|kuavo\_opensource\_mpc\_wbc\_img\_v1\.3\.0\.tar\.gz（官方预编译）|
|NVIDIA 驱动|570（LeTools\-Learning 文档测试通过版本）|
|NVIDIA Container Toolkit|1\.19\.1\-1（官方文档）|
|数据采集运行环境|系统 Python 3\.8（不用 Conda）|
|系统 Python 依赖|numpy\>=1\.19\.5,\<1\.27\.0、scipy\>=1\.8,\<1\.11、colorlog、py\_trees\<2\.5|
|LeTools\-Learning|本案例验证过的完整源码版本|

本节（一、基础环境准备）使用系统 Python 3\.8 和 ROS Noetic，不要激活 Conda 环境。



## **1\.2 初始化子模块（下载 LeTools\-Learning 和 LeRobot）**

在 `letools-opensource/` 初始化 `LeTools-Learning` 和 LeRobot 等子模块：

```Bash
cd ~/example/letools-opensource
git submodule update --init --recursive LeTools-Learning
```



## **1\.3 应用一键安装包**

先下载文末附件 `圆环仿真一键安装包.zip`，解压到 `~/example/`：

脚本会依次应用 `kuavo_sim_runtime.patch` 和 `data_challenge_simulator.patch`，解压 `ring_sim_assets.tar.gz`（13 个模型资源），安装 `LeTools-Learning/tools/ring_sim/`，最后安装两套 Python 依赖。

先在 `~/example/` 下做检查，检查不通过不会修改任何文件：

```Bash
cd ~/example
bash install_all.sh --check
```

检查通过后执行安装：

```Bash
bash install_all.sh --install   #--install 会按顺序应用下位机补丁、解压模型资源、安装 Learning 圆环代码并安装两套依赖。
```

默认仓库为 `~/example/letools-opensource`、`~/example/kuavo-ros-opensource`，默认 Conda 为 `~/miniforge3/bin/conda`。仓库或 Conda 装在别处时，在两条命令前分别设置：

```Bash
export LETOOLS_ROOT=/你的上位机仓库路径
export KUAVO_ROOT=/你的下位机仓库路径
export CONDA_BIN=/你的conda可执行文件路径
```

安装不会覆盖上位机代码。脚本不会自动回滚已完成的步骤，失败后请保留输出并检查仓库状态，不要直接重复运行。

`install_all.sh --install` 已包含两套 Python 依赖安装。成功后，把脚本输出的 Python 路径填入 `board.json` 的 `paths.learning_python`。

仅当依赖安装失败时，再单独执行：

```Bash
bash ~/example/ring-sim-upload/install_ring_sim_dependencies.sh \
  --letools-root ~/example/letools-opensource \
  --kuavo-root ~/example/kuavo-ros-opensource \
  --conda-bin ~/miniforge3/bin/conda
```



# **二、项目框架**

|**仓库**|**职责**|
|---|---|
|letools\-opensource|行为树、场景配置、统一入口、跨仓库调度|
|`LeTools-Learning`|ACT 推理、采集/评测工具、数据转换与训练|
|kuavo\-ros\-opensource|MuJoCo、机器人/夹爪模型、场景、ROS 仿真接口|

```Bash
letools-opensource/
├── apps/test_upper_init/run_behavior_tree_json.py
├── orchestration/
│   ├── nodes/
│   │   └── inference_runner.py  # 推理子进程入口
│   └── scenarios/grasp_ring_pick_place/
│       ├── board.json          # 行为树黑板、跨仓库路径、推理配置与夹爪反馈
│       ├── hardware_config.json
│       ├── py_tree.json
│       └── py_tree_child.json  # 导航、预抓取和放置动作参数
└── LeTools-Learning/
    ├── tools/ring_sim/         # 圆环全部工具，由 1.3 附件安装
    │   ├── helperfunc.py       # collect：仿真采集与 rosbag 录制
    │   ├── data/convert.py     # convert：rosbag → LeRobot 数据集
    │   ├── train.py            # train：ACT 训练
    │   ├── infer.py            # infer：ACT 独立推理
    │   ├── deploy.py           # evaluate：批量评测
    │   ├── runtime/            # 圆环 Gym 环境与配置加载
    │   ├── runtime_config.py   # 读取 board.json 的 paths 与容器名
    │   └── config/             # ring_deploy.yaml、ring_data.yaml、train/
    ├── kuavo_data/             # convert 依赖（关节切片、平台配置）
    ├── kuavo_model/            # train 依赖（训练框架）
    └── kuavo_deploy/           # infer 依赖（策略加载、ArmMove）；通用推理入口
```



## **2\.1 配置文件分工**

|**文件**|**作用**|
|---|---|
|board\.json|行为树配置，包括跨仓库路径、模型路径、夹爪反馈和放置目标。填写模型路径时建议使用绝对路径。|
|py\_tree\_child\.json|预抓取、搬运、放置、后撤、归位和导航动作参数；WheelNavMove 的 mode 为 base（底盘自身坐标系）|
|tools/ring\_sim/config/ring\_deploy\.yaml|圆环仿真推理配置，包括模型路径、推理频率和预抓取姿态|
|tools/ring\_sim/config/train/lerobot/act\.yaml|圆环数据集路径和 ACT 训练配置|
|hardware\_config\.json|仿真模式跳过相机、末端和力控发布器，仅启用 timed、arm、low 管理器|



# **三、仿真环境部署**

## **3\.1 Docker 容器内编译**

完成 1\.3 下位机补丁应用后，按下方步骤在 Docker 容器内编译并启动。

进入`kuavo-ros-opensource`目录执行`./docker/run_with_gpu.sh kuavo_opensource_mpc_wbc_img:1.3.0`进入容器后，默认在仓库的映射目录`/root/kuavo_ws`，执行以下命令开始编译：

```Bash
cd ~/example/kuavo-ros-opensource
./docker/run_with_gpu.sh kuavo_opensource_mpc_wbc_img:1.3.0

# 进入容器后
cd /root/kuavo_ws
export ROBOT_VERSION=62
source /opt/ros/noetic/setup.zsh
source installed/setup.zsh

catkin config -DCMAKE_ASM_COMPILER=/usr/bin/as -DCMAKE_BUILD_TYPE=Release
catkin build humanoid_controllers humanoid_wheel_interface_ros mujoco_cpp kuavo_msgs \
  ocs2_msgs data_challenge_simulator
source devel/setup.zsh
```



## **3\.2 仿真启动与场景验证**

编译好kuavo\-ros\-opensource环境后，在容器内执行。若使用新开的容器终端，也需要重新设置机器人版本并加载环境，进入容器，运行

```Bash
cd /root/kuavo_ws
export ROBOT_VERSION=62
source devel/setup.zsh

roslaunch humanoid_controllers load_kuavo_mujoco_sim_wheel.launch \
    scene_file:=/root/kuavo_ws/src/kuavo_assets/models/biped_s62/xml/scene_loading_s62.xml
```

验证标准：编译无失败包；仿真窗口正常打开，并显示小件上料场景中的机器人、料箱、工作台和圆环。至此仅完成仿真场景验证，不代表数据采集、训练和推理链路均已验证。

![2026-08-25 11-12-40 的屏幕截图](../../orchestration/scenarios/grasp_ring_pick_place/配套源码/图片和附件/2026-08-25%2011-12-40%20的屏幕截图.png)

# **四****、****数据采集、转换与模型训练**

## **4\.1 仿真数据采集**

在场景 `board.json` 中确认 `paths.kuavo_root` 和 `paths.letools_root` 是自己的仓库路径。`container.name` 留空时，程序会根据仿真仓库路径自动推导容器名；如需指定容器，可在该字段填写容器名。

采集使用宿主机系统 Python。请先启动配套 Docker 容器，并关闭当前已经运行的 ROS Master。

采集脚本会修改场景中的圆环位置，并在每轮结束时停止仿真相关 ROS 进程。采集前请关闭本机其他 ROS 任务，并在独立的仿真环境中运行。

```Bash
source /opt/ros/noetic/setup.bash
cd ~/example/letools-opensource/LeTools-Learning
/usr/bin/python3 -m tools.ring_sim collect
```

按提示选择是否录制 rosbag，并输入采集次数。采集完成后，本次 bag 输出在 `paths.collection_bags_root/run_<采集时间>/`，例如 `~/data/kuavo_bags/run_2026-11-02_20-15-00/`。4\.2 转换时，把这个 run\_\.\.\. 目录作为 rosbag\.rosbag\_dir。

[simplescreenrecorder-2026-10-15_14.48.03.mkv](../../orchestration/scenarios/grasp_ring_pick_place/配套源码/图片和附件/simplescreenrecorder-2026-10-15_14.48.03.mkv)

## **4\.2 ****数据转换**

转换会读取 ROS 的 rosbag，先 source 一次 ROS 环境：

```Plain Text
source /opt/ros/noetic/setup.bash
conda activate letools
cd ~/example/letools-opensource/LeTools-Learning

python3 -m tools.ring_sim convert \
  rosbag.mode=normal \
  rosbag.rosbag_dir=<4.1 采集得到的 run_... 目录> \
  rosbag.target_dir=outputs/ring_dataset
```

把第一处 `<...>` 替换成 4.1 实际输出的目录，尖括号本身不要留下。

`outputs/ring_dataset` 是相对 `LeTools-Learning` 的路径，转换后的数据集保存在 `outputs/ring_dataset/lerobot`。如果该目录已存在，转换会先删除再重建；转换前请确认没有需要保留的数据。

转换完成后检查 `outputs/ring_dataset/lerobot/meta/info.json`，确认 `total_frames` 大于 0；如果为 0，不要继续训练。



## **4\.3 模型训练与权重保存**

先改 `tools/ring_sim/config/train/lerobot/act.yaml`：

```Bash
dataset:
  root: outputs/ring_dataset/lerobot

training:
  output_dir: outputs/train/ring_act
```

如果 4.2 使用默认输出目录，这两项保持默认即可；训练轮数、batch size 等也在这个文件里修改。改到别处时同步修改。

再执行：

```Bash
conda activate letools
cd ~/example/letools-opensource/LeTools-Learning
python3 -m tools.ring_sim train
```



# **五、仿真推理与行为树验证**

> 💡 **本章开始前请先关闭前面启动的仿真。**
>
> A 组在宿主机每个新终端执行一次；B 组在进入容器后的每个新终端执行一次。本章 5\.1、5\.2 的启动代码块默认终端环境已准备好，不再重复 source，也不再重复进入容器。

A：宿主机新终端：

```Bash
source /opt/ros/noetic/setup.bash
source ~/example/kuavo-ros-opensource/installed/setup.bash
source ~/example/letools-opensource/infrastructure/ros_packages/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export PYTHONPATH=~/example/letools-opensource/drivers/leju/kuavo_humanoid_sdk/src/kuavo_humanoid_sdk:~/example/letools-opensource/infrastructure/ros_packages/src/kuavo_common/python:$PYTHONPATH
```

最后一行把上位机子模块里的 `kuavo_humanoid_sdk` 1\.4\.5 和 `kuavo_common/python`（提供 `robot_version`）加到 `PYTHONPATH`。如果上位机子模块没有初始化（`drivers/leju/kuavo_humanoid_sdk/` 为空），改用：`export PYTHONPATH=~/example/kuavo-ros-opensource/src/kuavo_humanoid_sdk:~/example/letools-opensource/infrastructure/ros_packages/src/kuavo_common/python:$PYTHONPATH`。每开一个宿主机新终端都要重设一遍，不要写进 `~/.bashrc`。

B：容器内终端：

```Bash
cd /root/kuavo_ws
export ROBOT_VERSION=62
source /opt/ros/noetic/setup.zsh
source devel/setup.zsh
export LD_LIBRARY_PATH=/root/kuavo_ws/devel/lib:/opt/drake/lib:/root/.mujoco/mujoco210/bin:$LD_LIBRARY_PATH
```



## **5\.1 独立模型推理**

本节只验证训练好的 ACT 权重能否在仿真中从预抓取位开始推理，需要两个终端。

打开 `LeTools-Learning/tools/ring_sim/config/ring_deploy.yaml`，把 `inference.pretrained_path` 改成训练生成的模型检查点目录：

```YAML
inference:
  pretrained_path: <模型检查点目录>
```

把 `<模型检查点目录>` 替换成 4.3 训练输出目录下的 `checkpoints/<步数>` 目录，写绝对路径。



### **5\.1\.1 终端1：****容器内****启动仿真**

```Bash
roslaunch humanoid_controllers load_kuavo_mujoco_sim_wheel.launch \
  scene_file:=/root/kuavo_ws/src/kuavo_assets/models/biped_s62/xml/scene_loading_s62.xml \
  randomize_ring:=true \
  publish_fake_claw_state:=false \
  publish_measured_claw_state:=true
```



### **5\.1\.2终端2：****宿主机****运行模型推理**

```Bash
conda activate letools
cd ~/example/letools-opensource/LeTools-Learning
python3 -m tools.ring_sim infer --task run --config tools/ring_sim/config/ring_deploy.yaml
```

启动后程序会先初始化仿真环境、移动到预抓取位，再开始 ACT 推理。

[simplescreenrecorder-2026-10-16_19.14.40.mkv](../../orchestration/scenarios/grasp_ring_pick_place/配套源码/图片和附件/simplescreenrecorder-2026-10-16_19.14.40.mkv)



## **5\.2 小件上料行为树运行**

|终端|执行位置与环境|用途|
|---|---|---|
|1|Docker 内，zsh|启动仿真|
|2|宿主机，系统 ROS 环境|TF 转发|
|3|宿主机，系统 Python 3\.8|启动行为树|

行为树主程序不要在 `LeTools-Learning` 的 Conda Python 3\.12 中启动。运行到推理节点时，程序会另外启动 Conda 推理子进程。

**启动并验证行为树：**

若刚完成第 5\.1 节，请先退出终端 2 的推理程序，并在终端 1 关闭仿真，再启动本节场景。启动前，请在 `board.json` 中确认 `paths.learning_python`，并把 `pretrained_path` 指向本次训练生成的检查点目录；`board.json` 的值会覆盖 `ring_deploy.yaml` 的同名项。首次运行可先使用默认运动参数。



### **5\.2\.1 终端1：****容器内****启动仿真场景**

```Bash
roslaunch humanoid_controllers load_kuavo_mujoco_sim_wheel.launch \
  scene_file:=/root/kuavo_ws/src/kuavo_assets/models/biped_s62/xml/scene_loading_s62.xml \
  randomize_ring:=true \
  publish_fake_claw_state:=false \
  publish_measured_claw_state:=true
```

等待小件上料场景加载完成，保持该终端运行。



### **5\.2\.2 终端2：****宿主机****启动TF转发服务**

```Bash
cd ~/example/letools-opensource/infrastructure/ros_packages
./start_tf_republisher.sh
```

出现“服务启动成功”，并显示 `/republish_tfs` 后，保持该终端运行。



### **5\.2\.3 终端3：****宿主机****启动行为树**

等仿真和 TF 转发服务启动完成后，在新的宿主机终端运行行为树：

```Bash
cd ~/example/letools-opensource

# 可选：先检查场景和配置
/usr/bin/python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/grasp_ring_pick_place \
  --dry-run

# 正式启动
/usr/bin/python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/grasp_ring_pick_place
```

正式运行会执行场景动作。行为树运行到推理节点时，会使用前面指定的 Conda Python 启动模型推理子进程。

[simplescreenrecorder-2026-10-19_13.38.27.mkv](../../orchestration/scenarios/grasp_ring_pick_place/配套源码/图片和附件/simplescreenrecorder-2026-10-19_13.38.27.mkv)


# **验证方式**

|阶段|判据|
|---|---|
|环境与补丁|`install_all.sh --check` 通过；`--install` 执行完成并输出 Learning Python 路径；`LeTools-Learning/tools/ring_sim/` 存在|
|仿真部署|编译无失败包；仿真窗口正常打开，可见机器人、料箱、工作台和圆环|
|数据|采集得到 `paths.collection_bags_root/run_<时间>/`；转换后 `rosbag.target_dir/lerobot/meta/info.json` 中 `total_frames` 大于 0|
|模型|训练输出目录中存在 checkpoint；5\.1 独立推理能够完成圆环抓取|
|行为树|依次完成预抓取、模型抓取、搬运、放置与归位，且无节点长时间等待或报错|


# **下一步**

- [12-下一步](12-下一步.md)：本章回顾与后续阅读
- [05-扩展-orchestration-层](../05-进阶开发指南/05-扩展-orchestration-层.md)：在场景中新增行为树节点或子树
- `LeTools-Learning` 的其他用法（其他模型、真机数据采集与部署）参考该仓库官方文档



# **附件：**

下载：[圆环仿真一键安装包.zip](../../orchestration/scenarios/grasp_ring_pick_place/配套源码/圆环仿真一键安装包.zip)

两套 Python 依赖清单、下位机补丁和 Learning 的圆环代码都在这个压缩包里，按 1\.3 的步骤解压并安装即可；上位机代码使用本案例对应的 LeTools 版本。本安装包不包含训练好的 ACT 权重，权重需按第四章自行采集、转换、训练得到。

|文件或目录|说明|
|---|---|
|`install_all.sh`|`--check` 只检查不修改；`--install` 执行安装|
|`使用说明.md`|脚本行为、默认路径、环境变量、执行顺序|
|`ring-sim-upload/`|下位机补丁、模型资源、两套依赖清单、依赖安装脚本、补丁说明|
|`圆环案例/letools-learning-ring-tools.zip`|LeTools\-Learning 圆环代码，新增 `tools/ring_sim/`|

解压后 `~/example/` 下共四项：`ring-sim-upload/`、`圆环案例/`、`install_all.sh`、`使用说明.md`。表中其余路径均相对于解压后的 `~/example/` 目录。

## **附件覆盖范围**

本附件覆盖圆环案例所需的仿真、模型、Learning 代码和依赖；上位机代码由 LeTools 仓库提供。逐项核对如下：

|**位置**|**改动**|**是否在附件中**|
|---|---|---|
|kuavo\-ros\-opensource|`kuavo_sim_runtime.patch` 的 15 个文件（场景、模型、夹爪反馈、仿真接口、`SetObjectPosition.srv`）|是|
|kuavo\-ros\-opensource|`data_challenge_simulator.patch` 的 `src/data_challenge_simulator/` 整包（580 个文件）|是|
|kuavo\-ros\-opensource|`ring_sim_assets.tar.gz` 的 13 个模型与纹理文件|是|
|letools\-opensource|`board.json` 与 `inference_runner.py` 的圆环支持|否，由对应 LeTools 版本提供|
|LeTools\-Learning|`tools/ring_sim/` 全部代码与配置|是|
|两套 Python 依赖|两份 requirements 和安装脚本|是|
|训练好的 ACT 权重|`outputs/train/.../checkpoints/`|**否**，需按第四章自行训练|

按 1\.3 执行 `install_all.sh --install` 后无需再手工修改其他文件。只有当导入 SDK 报 `ModuleNotFoundError: No module named 'robot_version'` 时，才需要核对 `src/kuavo_humanoid_sdk/kuavo_humanoid_sdk/kuavo/robot_blockly.py` 中 `kuavo_common/python` 的相对路径层数是否为三层 `../`。
