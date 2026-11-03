# 11 小件上料模仿学习-grasp_ring_pick_place

[返回文档首页](../README.md) · [04-简单案例运行](../04-简单案例运行/) · [03-接口详细文档](../03-接口详细文档/)

**难度**：⭐⭐⭐⭐

**适合读者**：已经跑通 07–10 篇，想完成一次「仿真采集数据 → 训练 ACT 模型 → 在行为树中调用模型」完整模仿学习流程的人。

**前置知识**：行为树的基本结构（[03-行为树最小示例](03-行为树最小示例.md)）；已运行过前序案例（[07-手臂与头部并行](07-手臂与头部并行-instructional_case_02.md)–[10-搬箱场景](10-搬箱场景-grasp_mtbf.md)）；仿真容器的启动与编译（[安装步骤 1.1](../02-环境安装/03-安装步骤.md#11-环境部署)）；Conda 的基本用法。

**核心概念**：行为树主程序运行在宿主机系统 Python 3.8（ROS Noetic）中，ACT 推理运行在独立的 Conda Python 3.12 环境（`LeTools-Learning`）中；行为树运行到推理节点时，由 `InferenceControlSkill` 启动推理子进程。仿真、行为树配置和推理所需代码分别位于下位机、上位机和 `LeTools-Learning` 三个仓库。

**学习目标**：

1. 准备两套 Python 环境，并应用下位机补丁；
2. 在 MuJoCo 中采集圆环抓取数据，转换为 LeRobot 数据集并训练 ACT 模型；
3. 单独验证模型推理，再通过 `run_behavior_tree_json.py` 启动主行为树，完成「预抓取 → 模型抓取 → 搬运 → 放置 → 归位」。

---

请先完成其他案例的环境搭建，并将上位机仓库 letools\-opensource 和下位机仓库 kuavo\-ros\-opensource 放在同一目录下。本文以 \~/example/ 为例；如果使用其他目录，请将后续命令中的路径替换为实际路径。



上位机和 LeTools\-Learning 直接使用完整源码，不需要打补丁；只有 kuavo\-ros\-opensource 需要应用下位机补丁。补丁必须打在对应发布基线的同一提交上。

# **场景定位**

|维度|值|
|---|---|
|场景目录|`orchestration/scenarios/grasp_ring_pick_place/`|
|运行环境|仅 MuJoCo 仿真（`ROBOT_VERSION=62`，biped_s62 机型）|
|任务|机器人从料箱中抓取圆环，底盘搬运到工作台，放置后归位|
|抓取方式|ACT 模仿学习模型|
|参与的仓库|`letools-opensource`（行为树、场景配置、统一入口）、`kuavo-ros-opensource`（MuJoCo、模型、场景、ROS 接口）、`LeTools-Learning`（数据转换、训练与推理，作为上位机子模块）|
|行为树入口|`apps/test_upper_init/run_behavior_tree_json.py`|
|运行配置|行为树入口与黑板使用 `board.json`，动作参数使用 `py_tree_child.json`，推理参数使用 `LeTools-Learning/configs/deploy/deploy.yaml`|

## **行为树流程**

```text
grasp_ring_pick_place（Sequence）
├── pre_grasp：张开夹爪，低头，调整躯干与双臂到预抓取位
├── inference_control：启动 ACT 推理，结合夹爪反馈判断抓取状态
├── pose_after_grasp：双臂切换到搬运姿态
├── nav_and_lift：躯干回正，底盘移动至放置区域
└── place_ring：前伸放置、张开夹爪、后撤并归位
```

行为树主程序运行在宿主机系统 Python 3.8 中，运行到推理节点时使用 `paths.learning_python` 启动 ACT 推理子进程；仿真机器人运行在 Docker 容器中。`board.json` 保存行为树黑板值、跨仓库路径、模型入口、放置目标和夹爪反馈判据；`py_tree_child.json` 保存导航与动作参数；`deploy.yaml` 保存推理频率和预抓取参数。三份配置分别负责行为树、动作参数和推理参数。

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
|flash\-attn|本 ACT 流程不要求单独安装；其他模型按需安装|
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
git submodule update --init --recursive
```



## **1\.3 应用下位机补丁和资源**

下载附件 `ring-sim-upload.zip`，解压到 `~/example/`，确认得到 `~/example/ring-sim-upload/`。压缩包内已包含补丁、模型资源、安装脚本、依赖清单和说明文档。

|**补丁**|**应用目录**|**用途**|
|---|---|---|
|kuavo\_sim\_runtime\.patch|kuavo\-ros\-opensource|MuJoCo 场景、模型、夹爪反馈和仿真接口|
|ring\_sim\_assets\.tar\.gz|kuavo\-ros\-opensource 根目录|2F85、D405、ring4 和三张纹理，共 13 个文件|
|data\_challenge\_simulator\.patch|kuavo\-ros\-opensource|Task 1～4 和圆环采集|

```Bash
cd ~/example/kuavo-ros-opensource
git apply --check ../ring-sim-upload/patches/kuavo_sim_runtime.patch
git apply ../ring-sim-upload/patches/kuavo_sim_runtime.patch

git apply --check ../ring-sim-upload/patches/data_challenge_simulator.patch
git apply ../ring-sim-upload/patches/data_challenge_simulator.patch

tar -xzf ../ring-sim-upload/patches/ring_sim_assets.tar.gz -C .
```

git apply 失败通常表示仓库或提交基线不一致；不要用 \-3 或手工跳过失败文件。



## **1\.4 安装 Python 依赖**

系统 Python 3\.8 供行为树和采集使用，Learning Python 3\.12 供转换、训练和推理使用。两套依赖清单和安装脚本统一放在 `~/example/ring-sim-upload/` 目录。脚本会创建或复用 `letools` 环境并安装两套依赖，但不会自动修改场景 `board.json`。

```Bash
bash ~/example/ring-sim-upload/install_ring_sim_dependencies.sh \
  --letools-root ~/example/letools-opensource \
  --kuavo-root ~/example/kuavo-ros-opensource \
  --conda-bin ~/miniforge3/bin/conda
```

系统依赖：requirements/system\-python38\.txt（numpy、scipy、colorlog、py\_trees 等）。Learning 依赖：torch、LeRobot 等。 脚本只安装依赖并打印 Learning Python 路径，不会自动改配置；把输出路径填入场景 `board.json` 的 `paths.learning_python`。必须显式指定 Miniforge 的 conda；不要让脚本误用 PATH 里的其他 conda。



# **二、项目框架**

|**仓库**|**职责**|
|---|---|
|letools\-opensource|行为树、场景配置、统一入口、跨仓库调度|
|`LeTools-Learning`|ACT 推理、采集/评测工具、数据转换与训练|
|kuavo\-ros\-opensource|MuJoCo、机器人/夹爪模型、场景、ROS 仿真接口|

```Bash
letools-opensource/
├── apps/test_upper_init/run_behavior_tree_json.py  # 通用行为树入口
├── orchestration/
│   ├── nodes/
│   └── scenarios/grasp_ring_pick_place/
│       ├── board.json          # 行为树黑板、路径与夹爪反馈
│       ├── hardware_config.json
│       ├── py_tree.json
│       └── py_tree_child.json  # 导航、预抓取和放置动作参数
└── LeTools-Learning/
    ├── tools/ring_sim/            # 采集、bag 检查、独立评测
    ├── configs/{data,deploy,train}/
    ├── kuavo_data/                # 数据转换
    ├── kuavo_model/               # 训练
    └── kuavo_deploy/              # 推理
```



## **2\.1 配置文件分工**

|**文件**|**作用**|
|---|---|
|board\.json|行为树黑板、跨仓库 `paths`、`container`、模型入口、放置目标、`grasp_feedback`；相对路径以该文件所在目录为基准|
|py\_tree\_child\.json|预抓取、搬运、放置、后撤、归位和导航动作参数；WheelNavMove 的 mode 为 base，导航坐标直接在此修改|
|configs/deploy/deploy\.yaml|ACT 推理参数，包括 `ros_rate`、`control_rate` 和 `env.sim_pre_grasp` 预抓取初始姿态|
|configs/train/lerobot/act\.yaml|ACT 训练超参数|
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
cd ~/example/letools-opensource/LeTools-Learning
/usr/bin/python3 -m tools.ring_sim collect
```

按提示选择是否录制 rosbag，并输入采集次数。采集完成后，本次 bag 输出在 `paths.collection_bags_root/run_<采集时间>/`，例如 `~/data/kuavo_bags/run_2026-11-02_20-15-00/`。4\.2 转换时，把这个 run\_\.\.\. 目录作为 rosbag\.rosbag\_dir。

[simplescreenrecorder-2026-10-15_14.48.03.mkv](../../orchestration/scenarios/grasp_ring_pick_place/配套源码/图片和附件/simplescreenrecorder-2026-10-15_14.48.03.mkv)

## **4\.2 ****数据转换**

```Plain Text
conda activate letools
cd ~/example/letools-opensource/LeTools-Learning

python3 -m kuavo_data.CvtRosbag2Lerobot \
  rosbag.mode=normal \
  rosbag.rosbag_dir=<bag目录> \
  rosbag.target_dir=<数据集父目录>
```

转换后的数据集保存在 `<rosbag.target_dir>/lerobot`。当 `rosbag.target_dir` 填 paths\.dataset\_parent 时，就是 `paths.dataset_parent/lerobot`；命令里以实际传入的 `rosbag.target_dir` 为准。

输出目录已存在时会停止。



## **4\.3 模型训练与权重保存**

先改 `configs/train/lerobot/act.yaml`：

```Bash
dataset:
  root: <数据集父目录>/lerobot

training:
  output_dir: <训练输出目录>
```

再执行：

```Bash
conda activate letools
cd ~/example/letools-opensource/LeTools-Learning
python3 kuavo_model/train.py --policy act --config-root configs/train/lerobot
```



# **五、仿真推理与行为树验证**

A 组在宿主机每个新终端执行一次；B 组在进入容器后的每个新终端执行一次。本章 5\.1、5\.2 的启动代码块默认终端环境已准备好，不再重复 source，也不再重复进入容器。

A：宿主机新终端：

```Bash
source /opt/ros/noetic/setup.bash
source ~/example/kuavo-ros-opensource/installed/setup.bash
source ~/example/letools-opensource/infrastructure/ros_packages/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
```

B：容器内终端：

```Bash
cd /root/kuavo_ws
export ROBOT_VERSION=62
source /opt/ros/noetic/setup.zsh
source devel/setup.zsh
export LD_LIBRARY_PATH=/root/kuavo_ws/devel/lib:/opt/drake/lib:/root/.mujoco/mujoco210/bin:$LD_LIBRARY_PATH
```



## **5\.1 独立模型推理**

先关闭 3\.2 中仍在运行的仿真。本节只验证训练好的 ACT 权重能否在仿真中从预抓取位开始推理，需要两个终端。

在 configs/deploy/deploy\.yaml 中确认：

```YAML
inference:
  pretrained_path: <模型检查点目录>
  # eval_episodes 仅 auto_test 使用，独立推理不读取
```



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
python3 kuavo_deploy/eval.py configs/deploy/deploy.yaml
```

启动后选择 2\. run；程序会先初始化仿真环境、移动到预抓取位，再开始 ACT 推理。

[simplescreenrecorder-2026-10-16_19.14.40.mkv](../../orchestration/scenarios/grasp_ring_pick_place/配套源码/图片和附件/simplescreenrecorder-2026-10-16_19.14.40.mkv)



## **5\.2 小件上料行为树运行**

|终端|执行位置与环境|用途|
|---|---|---|
|1|Docker 内，zsh|启动仿真|
|2|宿主机，系统 ROS 环境|TF 转发|
|3|宿主机，系统 Python 3\.8|启动行为树|

行为树主程序不要在 `LeTools-Learning` 的 Conda Python 3\.12 中启动。运行到推理节点时，程序会另外启动 Conda 推理子进程。

**启动并验证行为树：**

若刚完成第 5\.1 节，请先退出终端 2 的推理程序，并在终端 1 关闭仿真，再启动本节场景。启动前，请按 2\.1 的 board\.json 说明填写模型检查点路径；首次运行可先使用默认运动参数。



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
|环境与补丁|`git apply --check` 无报错；依赖安装脚本执行完成，并输出 Learning 环境的 Python 路径|
|仿真部署|编译无失败包；仿真窗口正常打开，可见机器人、料箱、工作台和圆环|
|数据|采集得到 `paths.collection_bags_root/run_<时间>/`；转换后存在 `rosbag.target_dir/lerobot`|
|模型|训练输出目录中存在 checkpoint；5.1 独立推理能够完成圆环抓取|
|行为树|依次完成预抓取、模型抓取、搬运、放置与归位，且无节点长时间等待或报错|


# **下一步**

- [12-下一步](12-下一步.md)：本章回顾与后续阅读
- [05-扩展-orchestration-层](../05-进阶开发指南/05-扩展-orchestration-层.md)：在场景中新增行为树节点或子树
- `LeTools-Learning` 的其他用法（其他模型、真机数据采集与部署）参考该仓库官方文档



# **附件：**

下载：[ring-sim-upload.zip](../../orchestration/scenarios/grasp_ring_pick_place/配套源码/ring-sim-upload.zip)

两套 Python 依赖清单和安装脚本均在 `ring-sim-upload.zip` 内，按 1\.3、1\.4 的步骤解压并安装即可。

|文件或目录|类型|说明|
|---|---|---|
|`ring-sim-upload.zip`|完整附件包|解压到 `~/example/`，得到 `~/example/ring-sim-upload/`；包含下位机补丁、模型资源、依赖清单、安装脚本和说明文档|
|`patches/kuavo_sim_runtime.patch`|下位机补丁|在 `kuavo-ros-opensource` 中应用；包含 MuJoCo 场景、模型、夹爪反馈和仿真接口|
|`patches/data_challenge_simulator.patch`|下位机补丁|在 `kuavo-ros-opensource` 中应用；Task 1～4 和圆环采集使用，约 328 MB|
|`patches/ring_sim_assets.tar.gz`|模型资源包|解压到 `kuavo-ros-opensource` 根目录；包含 2F85、D405、ring4 和三张纹理，共 13 个文件|
|`requirements/system-python38.txt`|系统依赖清单|由安装脚本读取；供系统 Python 3\.8 使用|
|`requirements/learning-python312.txt`|Learning 依赖清单|由安装脚本读取；供 `letools` 环境使用，不安装 SDK|
|`install_ring_sim_dependencies.sh`|安装脚本|放在 `~/example/ring-sim-upload/`；创建或复用 `letools` 环境并安装两套依赖|
|`说明/圆环仿真补丁改动说明/`|说明附件|逐文件说明下位机补丁的改动位置和内容|
|`说明/圆环仿真话题与服务一览/`|附录附件|话题、服务和自查命令，作为附录 A 使用|
|`图片和附件/`|图文附件|文档中的 1 张场景截图和 3 个演示视频，仅用于阅读，不参与程序运行|

除 `ring-sim-upload.zip` 和“图片和附件/”外，其余路径均相对于解压后的 `ring-sim-upload/` 目录。
