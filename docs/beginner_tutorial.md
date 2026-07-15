# 零基础完整学习教程

本教程面向**完全零基础** 的读者：不了解 LeTools 分层架构，不会配置 ROS / Python 环境，也没跑过任何机器人测试脚本。按顺序阅读并操作，即可完成从「什么都不懂」到「能运行测试脚本控制机器人」的全流程。

**→ 阅读说明：**  建议**按顺序阅读** 。第一部分建立概念，第二至三部分完成环境准备，第四至六部分从简单到复杂逐步跑通测试脚本。若你已有 ROS 环境且只想跑某个具体测试，可适当跳过前面部分。

## 你将学到什么

完成本教程后，你将能够：

| 目标 | 内容 |
|---|---|
| **理解概念** | LeTools 是什么；分层架构（core → adapters → skills → orchestration → apps）怎么理解 |
| **准备环境** | 安装 Python、ROS Noetic、Kuavo SDK；克隆仓库并安装依赖 |
| **跑通第一个脚本** | 初始化硬件适配器，让机器人动起来（手臂复位 / 底盘移动） |
| **理解控制路径** | 标准接口、SDK 接口、TimedCmd 三条路径的区别与选型 |
| **进阶测试** | 运行离线轨迹、IK 可达性检查、Ruckig 参数调优等高级测试 |
| **相机/视觉** | 启动相机、获取图像、AprilTag 检测 |
| **行为树编排** | 用 JSON 加载动作流程，把多个动作排成任务 |

---

# 第一部分：概念入门（零基础必读）

在动手装环境、跑代码之前，先建立一点直觉，避免「不知道自己在跑什么」。

## 1.1 LeTools 是什么？

**通俗理解** ：LeTools 是乐聚 Kuavo 机器人的上位机 Python 工具链框架。它把"控制机器人做动作"这件事拆成几层积木：底层定义接口，中间层适配硬件，上层封装技能，最上面用行为树编排任务。

你不需要从头写 ROS 话题订阅、服务调用、关节限位检查这些底层逻辑——LeTools 已经帮你封装好了。你只需要告诉它"我想让左臂伸到这个位置"或"我想让底盘前进 0.5 米"，它会帮你把指令翻译成机器人能理解的格式。

## 1.2 分层架构怎么理解？

LeTools 把机器人能力拆成几层，可以理解成一个"动作积木系统"：

| 层 | 目录 | 通俗理解 |
|---|---|---|
| **apps** | `apps/` | 搭好的范例——可以直接照着玩的积木示例 |
| **orchestration** | `orchestration/` | 拼装图纸——把多个积木按顺序拼成一套动作流程 |
| **skills** | `skills/` | 单个积木——一块积木做一个动作（挥手、前进、抓取） |
| **adapters** | `adapters/` | 接口适配——把积木的卡扣对准具体机器人的接口 |
| **core** | `core/` | 积木规格——定义卡扣形状、尺寸、材质标准 |
| **drivers / infrastructure** | `drivers/` `infrastructure/` | 底座——ROS 消息、SDK 底层驱动，积木搭在上面的地基 |

**新手最关心的是 `apps/`** ：里面全是"复制就能跑"的测试脚本，每个脚本演示一个具体功能。

## 1.3 ROS 是什么？在 LeTools 里起什么作用？

**ROS（Robot Operating System）** 本身不是一个操作系统，而是一套机器人通信中间件。它提供了一套标准的"话题（Topic）"和"服务（Service）"机制，让不同程序之间可以互相发消息、互相调用。

**通俗理解** ：ROS 就像机器人内部的"微信群"——上位机（LeTools）在群里发一句"左臂伸到 (0.3, 0.2, 0.6)"，控制器（MPC/WBC）看到后去执行，执行完在群里回复"到位了"。双方不需要知道对方代码长什么样，只需要约定好消息格式。

**在 LeTools 里的作用** ：

| 层 | 怎么用 ROS |
|---|---|
| **infrastructure/** | 定义 ROS 消息格式（`.msg`/`.srv`），是"群消息"的模板 |
| **drivers/** | 订阅/发布 ROS 话题，是"群成员"的收发信客户端 |
| **adapters/** | 通过 ROS 服务（Service）给控制器下发指令 |
| **core/** | 订阅 ROS 话题读取机器人当前状态（关节角、位姿） |

> 💡 **新手认知**：你运行 LeTools 脚本时，脚本本质上是在"给 ROS 群里发消息"。如果群里没有控制器（MPC 节点没启动），消息就石沉大海——这就是为什么脚本会报 `ROS 服务不可用`。

## 1.4 Docker 是什么？在 LeTools 里起什么作用？

**Docker** 是一种容器化工具，它把"操作系统 + 依赖库 + 编译好的程序"打包成一个镜像（Image），你在任何机器上运行这个镜像，都能得到完全一致的环境。

**通俗理解** ：Docker 就像一个"预制机房"——里面已经装好了指定版本的 Ubuntu、ROS、编译器、依赖库，你拎包入住就能用，不用自己从零装系统、配环境、踩依赖冲突的坑。

**在 LeTools 里的作用** ：

| 场景 | Docker 的角色 |
|---|---|
| **仿真部署** | 官方提供预制镜像，内含 MuJoCo + 控制器，直接 `docker run` 就能启动仿真 |
| **环境一致性** | 不用担心"我这台电脑能跑、你那台不能跑"——镜像里环境完全一致 |
| **真机部署** | 真机上也可用 Docker 跑控制器，避免污染宿主机环境 |

> 💡 **新手认知**：LeTools 本身**不需要** Docker 也能跑——只要你的电脑装了 ROS + Python + SDK，脚本就能直接运行。Docker 主要用在"启动仿真控制器"和"真机部署控制器"这两个场景，帮你省去装环境的麻烦。如果你直连真机且控制器已在运行，可以完全不用 Docker。

## 1.5 三条控制路径

LeTools 提供三条不同的路径来控制机器人，理解它们的区别是后续选型的关键：

| 路径 | 通俗理解 | 特点 | 典型入口 |
|---|---|---|---|
| **标准接口（Adapter）** | 通过适配器统一接口调用 | 跨硬件兼容，推荐正式使用 | `apps/test_kuavo_5w_adapter/` |
| **SDK 接口** | 通过 Kuavo SDK 接口调用 | 封装完善，测试覆盖全面 | `apps/test_kuavo_5w_sdk_adapter/` |
| **TimedCmd（Ruckig）** | 通过时序指令 + Ruckig 插值 | 时间最优、运动平滑，适合定点运动 | `apps/test_kuavo_5w_sdk_adapter/04_timed_commands/` |

> 💡 **新手建议**：从 **标准接口（Adapter）** 路径开始，它的测试脚本最完整、错误提示最友好。

## 1.6 关键术语速查

| 术语 | 含义 |
|---|---|
| **ROS** | 机器人通信中间件，LeTools 和控制器之间通过它收发消息 |
| **Docker** | 容器化工具，用于打包一致的运行环境，主要在仿真/真机控制器部署时使用 |
| **MPC** | 模型预测控制，机器人全身运动控制器 |
| **WBC** | 全身平衡控制，保持机器人不摔倒 |
| **Ruckig** | 在线轨迹生成库，在速度/加速度/急动度限制下算时间最优轨迹 |
| **IK** | 逆运动学，把"末端位姿"反解成"关节角度" |
| **FK** | 正运动学，把"关节角度"正算成"末端位姿" |
| **planner_index** | 规划器索引，告诉控制器这段指令控制哪个部位（手臂/底盘/躯干） |
| **cmd_vec** | 命令向量，具体参数值（位置/角度/速度） |
| **dry-run** | 离线模拟运行，不连机器人也不发 ROS 指令 |

---

# 第二部分：环境准备

## 2.1 操作系统

推荐 **Ubuntu 20.04**。LeTools 依赖 ROS Noetic（ROS1），目前只支持 Linux。

## 2.2 安装 ROS Noetic

如果尚未安装 ROS，按 [ROS Noetic 官方安装指南](http://wiki.ros.org/noetic/Installation/Ubuntu) 操作。安装完成后验证：

```bash
source /opt/ros/noetic/setup.bash
roscore --help   # 能打印帮助说明即安装成功
```

## 2.3 安装 Python 依赖

LeTools 需要 Python 3.8+。Ubuntu 20.04 自带 Python 3.8，验证：

```bash
python3 --version   # 应输出 Python 3.8.x 或更高
```

安装项目所需的 Python 包：

```bash
sudo apt update
sudo apt install -y python3-pip python3-rosdep python3-catkin-tools
pip3 install numpy scipy pytrees
```

## 2.4 下载工具链项目
乐聚员工可通过 GitLab 下载工具链项目（letools）（dev 分支）
外部客户可通过开源社区下载工具链项目（letools_opensource）：git clone https://gitcode.com/OpenLET/letools_opensource.git

```bash
cd ~
git clone https://gitcode.com/OpenLET/letools_opensource.git
cd letools_opensource
```


### 前置准备

确保你的系统已安装 Python 3.8+ 和 ROS Noetic，并且 `numpy` 版本满足要求：

```bash
pip3 install --upgrade "numpy>=1.19.5,<1.27.0"
```

---

# 第三部分：启动仿真或连接真机

测试脚本需要有一个"被控制的对象"——要么是仿真，要么是真机。本部分按以下顺序操作：

1. 编译 ROS 工作空间 + 安装 SDK（上位机侧，仿真和真机都需要）
2. 启动仿真 或 连接真机（二选一）
3. 环境加载与验证

## 3.1 编译 ROS 工作空间

在项目根目录运行以下命令，编译 ROS workspace 中的包：

```bash
cd ~/letools_opensource/infrastructure/ros_packages
catkin build
source devel/setup.bash
```

> 运行仿真暂时不需要相机模块，且视觉相关的包编译失败，可以先跳过：
>
> ```bash
> cd ~/letools_opensource/infrastructure/ros_packages
> catkin config --skiplist \
>    detection_yolo_v8 \
>    ar_control \
>    kuavo_vision_object \
>    kuavo_yolo_point2d \
>    yolo_box_object_detection \
>    yolo_button_object_detection \
>    yolo_valve_object_detection \
>    orbbec_camera \
>    realsense2_camera \
>    kuavo_camera\
>    kuavo_tf2_web_republisher
> catkin build
> ```

编译成功后，`devel/setup.bash` 会生成。**每个终端都需要 source 它**（见 3.4 节）。

## 3.2 安装  SDK

在项目根目录运行以下命令，脚本将自动完成 Submodule 初始化、分支切换、配置生成及 SDK 安装：

```bash
cd ~/letools_opensource
# 若是拉取过 SDK 子模块，但未初始化，则需清理残留的 submodule 数据
rm -rf drivers/leju/kuavo_humanoid_sdk
# 若是没有拉取过SDK子模块，则运行安装脚本初始化
chmod +x scripts/install_sdk.sh
./scripts/install_sdk.sh   ## 运行时间较长，耐心等待
```

验证安装：

```bash
python3 -c 'from kuavo_humanoid_sdk import KuavoRobot; print("SDK Ready!")'
```

若输出 `SDK Ready!` 则代表安装成功。

> ⚠️ 如果 `drivers/leju/kuavo_humanoid_sdk` 为空或不存在，说明子模块没下载成功，重新执行 `./scripts/install_sdk.sh` 即可。

## 3.3 启动仿真或连接真机

### 3.3.1 选项 A：启动 MuJoCo 仿真

#### Docker 镜像部署

仓库地址：https://gitcode.com/OpenLET/kuavo-ros-opensource/tree/dev/

将此仓库下的 dev 分支下载到本地，根据 readme.md 文档跑通 docker 环境，注意 Ubuntu 20.04 需要赋予 **sudo 权限**。

- docker 镜像可以自行查看网上相关配置使用 `./docker/Dockerfile` 构建，或者下载已经编译好的镜像：

```bash
wget https://kuavo.lejurobot.com/kuavo_research_editiion/docker_images/kuavo_opensource_mpc_wbc_img_v1.3.0.tar.gz
```

- 执行以下命令导入容器镜像：

```bash
sudo docker load -i kuavo_opensource_mpc_wbc_img_v1.3.0.tar.gz
```

- 💡 （推荐）进入 [**kuavo-ros-opensource**](https://gitcode.com/OpenLET/kuavo-ros-opensource) 目录执行 `./docker/run.sh` 进入容器后，默认在仓库的映射目录 `/root/kuavo_ws`，执行以下命令开始编译：

```bash
catkin config -DCMAKE_ASM_COMPILER=/usr/bin/as -DCMAKE_BUILD_TYPE=Release  # Important! -DCMAKE_ASM_COMPILER=/usr/bin/as 为配置了ccache必要操作，否则可能出现找不到编译器的情况
source installed/setup.zsh  # 加载一些已经安装的ROS包依赖环境，包括硬件包等
catkin build humanoid_controllers  # 会编译所有依赖项，成功后进行下一步
```

> 💡 **提示**
> - 容器镜像内部默认使用 zsh
> - .sh 类型文件不能运行记得给权限，`chmod +x ./docker/run.sh`
> - 若启动后仿真卡动可以在同级目录运行 `./docker/run_with_gpu.sh`

```bash
# 4090 系列需要作如下配置，具体视自己显卡情况而定可自行搜索解决
sudo nvidia-ctk runtime configure --runtime=docker

distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
  && curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
     sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
     sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
```

### 配置机器人版本

机器人版本通过环境变量 `$ROBOT_VERSION` 设置，版本号涉及不同机器人模型、硬件设置等，需要和自己的机器人匹配。在终端执行 `echo $ROBOT_VERSION` 查看当前设置的版本号，如果没有设置，通过以下设置版本号（其中的 45 代表 4.5 版本，根据实际情况修改，这里我们常用 62）：

在当前终端执行（临时设置）：

```bash
export ROBOT_VERSION=62
```

将其添加到你的 `~/.bashrc` 或者 `~/.zshrc` 终端配置文件中：如执行：

```bash
echo 'export ROBOT_VERSION=62' >> ~/.bashrc
```

添加到 `~/.bashrc` 文件末尾，重启终端后生效。

### 使用 MuJoCo 仿真器

```bash
source devel/setup.zsh  # 正常情况下，则使用 source devel/setup.bash
roslaunch humanoid_controllers load_kuavo_mujoco_sim_wheel.launch  # 轮臂机器人请使用
```

仿真启动后，验证 ROS 服务是否可用：

```bash
rosservice list | grep mobile_manipulator
# 应能看到 /mobile_manipulator_mpc_control 等服务
```

### 3.3.2 选项 B：连接真机

#### 下位机部署与跑通

```bash
git clone https://gitcode.com/OpenLET/kuavo-ros-opensource

# 编译运行
cd kuavo-ros-opensource  # 仓库目录
catkin config -DCMAKE_ASM_COMPILER=/usr/bin/as -DCMAKE_BUILD_TYPE=Release  # Important!
source installed/setup.bash  # 加载一些已经安装的ROS包依赖环境，包括硬件包等
catkin build humanoid_controllers
```

> 💡 **提示**：5-10 分钟即可完成编译

运行：

```bash
cd kuavo-ros-opensource
sudo su
source devel/setup.bash
roslaunch humanoid_controllers load_kuavo_real_wheel.launch
# 运行后按照提示日志按 "o"，一直后台挂着即可
```

#### 上位机部署跑通

上位机 SSH 进去后参照仿真环境部署，跑通即可。

这里注意上位机主目录磁盘较小，推荐到 `/media/data` 中部署运行。

## 3.4 kuavo_tf2_web_republisher 服务启动

```bash
# 首先编译工作空间
cd ~/letools_opensource/infrastructure/ros_packages
catkin build  # 或者使用该目录写好的 build.sh
# 摄像头相关编译不通过可以不用管，不影响我们后续操作
source devel/setup.bash
# 这里记得用系统环境所在的 py 环境也就是 ros 所在，有虚拟环境记得做选择
./start_tf_republisher.sh
```

这里我们就可以在终端中看到服务启动了。

> ⚠️ **注意**：这个服务得一直在后台挂着，而且得在仿真开启之后启动！！！

## 3.5 运行示例程序

`apps/` 目录下有以下文件夹，里面的示例脚本一般都是可以的：

- `test_kuavo_5w_adapter` — 标准接口（Adapter）测试
- `test_kuavo_5w_sdk_adapter` — SDK 接口测试
- `test_camera_adapter` — 相机与视觉测试
- `jibot_adapter` — 移动底盘测试
- `test_upper_init` — 行为树编排示例

推荐示例：

- `apps/test_kuavo_5w_sdk_adapter/sdk/01_head/test_head_control.py`
- `apps/test_kuavo_5w_sdk_adapter/sdk/02_arm/test_arm_joint_traj.py`
- `apps/test_kuavo_5w_sdk_adapter/sdk/03_lower_body/test_leg_joint.py`
- `apps/test_kuavo_5w_adapter/01_base_control/test_cmd_pose_base.py`

以头部示例程序为例：

```bash
# 首先 source 一下环境变量
cd ~/letools_opensource/infrastructure/ros_packages
source devel/setup.bash
# 这里记得用系统环境所在的 py 环境也就是 ros 所在，有虚拟环境记得做选择
cd ~/letools_opensource/apps/test_kuavo_5w_sdk_adapter/sdk/01_head
python3 test_head_control.py
```

然后我们就可以在 MuJoCo 仿真中看到机器人头部按照预设的动作活动了。

> ⚠️ **安全提示**：第一次运行时确保机器人周围有足够空间，建议先在仿真里跑通再上真机。



# 第四部分：简单脚本测试

跑通 3.5 的示例后，可以按部位逐个测试机器人的基本运动能力。每个脚本都是独立可运行的。下面每个小节都会说明**输入什么参数**、**产生什么动作**，方便你对照源码修改。

## 4.1 动头部

```bash
cd ~/letools_opensource/infrastructure/ros_packages
source devel/setup.bash
cd ~/letools_opensource/apps/test_kuavo_5w_sdk_adapter/sdk/01_head
python3 test_head_control.py
```

调用 `control_head_sdk(yaw, pitch)`，参数单位为**度**：

| 用例 | 参数 | 动作 |
|---|---|---|
| 居中 | `yaw=0, pitch=0` | 头部回到正前方 |
| 左转 | `yaw=+30, pitch=0` | 头部向左转 30° |
| 右转 | `yaw=-30, pitch=0` | 头部向右转 30° |
| 抬头 | `yaw=0, pitch=+20` | 头部向上抬 20° |
| 低头 | `yaw=0, pitch=-20` | 头部向下低 20° |
| 扫描序列 | `yaw ∈ [30, -30, 0]` | 依次左→右→居中 |

在 MuJoCo 仿真中可以看到机器人头部按照预设的动作活动。

## 4.2 动手臂

```bash
cd ~/letools_opensource/apps/test_kuavo_5w_adapter/03_arm_control
python3 test_arm_ee_joint.py
```

调用 `send_arm_ee_joint_space(left_joints, right_joints)`，每参数为 7 维关节角（**度**），顺序为 `[shoulder_yaw, shoulder_pitch, shoulder_roll, elbow_pitch, wrist_yaw, wrist_pitch, wrist_roll]`。左右臂按 J0/J3/J6 同号、其余反号镜像。脚本内部自动设手臂控制模式为 2（外部控制）：

| 用例 | 左臂参数 | 右臂参数 | 动作 |
|---|---|---|---|
| 展开 | `[-30, 20, 15, -45, 25, 10, -35]` | `[-30, -20, -15, -45, -25, -10, -35]` | 双臂张开前伸 |
| 弯曲 | `[-20, 30, -25, -20, 40, -15, 25]` | `[-20, -30, 25, -20, -40, 15, 25]` | 双臂收拢弯曲 |
| 零位 | `[0]*7` | `[0]*7` | 所有关节回零位 |

适配器自动等待 `/lb_arm_joint_reach_time/left` 和 `/lb_arm_joint_reach_time/right` 反馈到达。

## 4.3 动底盘

```bash
cd ~/letools_opensource/apps/test_kuavo_5w_adapter/01_base_control
python3 test_cmd_pose_base.py
```

调用 `send_base_pose(x, y, yaw, frame=LOCAL)`，`x/y` 单位为**米**，`yaw` 单位为**度**，是相对当前位置的增量：

| 用例 | 参数 | 动作 |
|---|---|---|
| 前进 | `x=0.5, y=0, yaw=0` | 相对前进 0.5 m |
| 后退 | `x=-0.3, y=0, yaw=0` | 相对后退 0.3 m |
| 左平移 | `x=0, y=0.3, yaw=0` | 相对左移 0.3 m |
| 右平移 | `x=0, y=-0.3, yaw=0` | 相对右移 0.3 m |
| 逆时针转 | `x=0, y=0, yaw=90` | 相对逆时针转 90° |
| 顺时针转 | `x=0, y=0, yaw=-90` | 相对顺时针转 90° |
| 整圈 | `x=0, y=0, yaw=360` | 旋转一整圈 |
| 复合 | `x=0.3, y=0, yaw=45` | 前进 0.3 m 同时转 45° |

适配器会自动订阅 `/lb_cmd_pose_reach_time` 等待到达。无需设置 MPC 模式（`/cmd_pose` 优先级高）。

## 4.4 动躯干

```bash
cd ~/letools_opensource/apps/test_kuavo_5w_adapter/02_lower_body
python3 test_torso_pose.py
```

调用 `send_torso_pose(pose)`，`pose` 为 `Pose6D(x, y, z, yaw, pitch, roll)`，位置单位为**米**，角度单位为**弧度**，是相对于 `base_link` 的**绝对坐标**（非增量）。脚本启动时自动复位躯干并读取初始位姿，后续用例在初始位姿基础上叠加偏移：

| 用例 | 参数（相对初始位姿的偏移） | 动作 |
|---|---|---|
| 抬高 | `z += 0.3` | 躯干抬高 0.3 m |
| 前移 | `x += 0.2`（保持抬高） | 躯干前移 0.2 m |
| 右偏航 | `yaw = +0.524`（+30°） | 躯干绕 z 轴右转 30° |
| 左偏航 | `yaw = -0.524`（-30°） | 躯干绕 z 轴左转 30° |
| 前倾 | `pitch = -0.175`（-10°） | 躯干前倾 10° |
| 后仰 | `pitch = +0.524`（+30°） | 躯干后仰 30° |
| 复位 | 回到初始位姿 | 躯干恢复初始位置 |

脚本启动时会关闭 Z 轴焦点跟踪（`set_focus_z(False)`）并设 MPC 为 `BaseArm`，结束自动恢复。

---

# 第五部分：进阶测试

跑通基础运动后，可以尝试更复杂的测试。下面每个小节挑一个典型示例，说明**输入什么参数**、**产生什么效果**。

## 5.1 时序动作测试

时序指令（TimedCmd）通过 `/mobile_manipulator_timed_single_cmd` 服务下发**带期望执行时间**的单条指令，由控制器内部的 Ruckig 规划器在速度/加速度/急动度限制下算出时间最优轨迹。相比标准接口的"发完即走"，它能拿到控制器返回的 `actualTime`（实际执行时间），适合需要精确计时的定点运动。

```bash
cd ~/letools_opensource/apps/test_kuavo_5w_adapter/04_timed_commands
python3 test_cmd_pose_sequence.py
```

以测试「躯干位姿时序指令序列」为例，调用 `send_timed_torso_pose(...)` 逐条下发指令：

```python
result = self.hardware.send_timed_torso_pose(
    x=0.3,            # 前后位置（米）
    z=1.4,            # 高度（米）
    yaw=17.19,        # 偏航角（度）
    pitch=0.0,        # 俯仰角（度）
    desire_time=3.0   # 期望执行时间（秒）
)
```

参数说明：

| 字段 | 含义 |
|---|---|
| `planner_index` | 规划器索引：0=底盘世界系、1=底盘局部系、2=躯干、3=下肢关节、8/9=左/右臂关节 |
| `desire_time` | **期望执行时间**（秒），控制器返回 `actual_time`（实际所需时间） |
| `cmd_vec` | 命令向量，底盘 3 维 `[x, y, yaw]`，躯干 4 维 `[x, z, yaw, pitch]`，手臂关节 7 维 |

脚本用例（按顺序执行，每条等前一条完成）：

| 用例 | 时间 | 位姿 `[x, z, yaw, pitch]` | 动作 |
|---|---|---|---|
| 初始位置 | 3.0s | `[0.0, 1.4, 0.0, 0.0]` | 躯干回到中立位 |
| 向前倾斜 | 3.0s | `[0.3, 1.4, 0.0, 17.19]` | 躯干前移并前倾 ~17° |
| 向后移动 | 3.0s | `[-0.2, 1.4, 0.0, 0.0]` | 躯干后移 0.2m |
| 向上抬升 | 3.0s | `[0.0, 1.5, 0.0, 0.0]` | 躯干抬高 0.1m |
| 回到初始 | 3.0s | `[0.0, 1.4, 0.0, 0.0]` | 恢复中立位 |

效果：躯干按 5 个关键位姿依次运动，每段控制器用 Ruckig 插值并在 `actual_time` 内到达，脚本据此等待再发下一条。

同目录还有更完整的 `test_mixed_commands.py`，把底盘（`send_timed_base_pose`）、躯干（`send_timed_torso_pose`）、腿部（`send_timed_leg_joint`）串联成"前进→前倾→微蹲→后退→恢复"的 6 步协调序列；以及 `test_arm_joint_sequence.py`（双臂 7 关节时序）、`test_cmd_vel_sequence.py`（底盘速度时序）等。

> ⚠️ 时序指令的 `planner_index`（0=底盘世界系/1=底盘局部系/2=躯干/...）和离线轨迹的 `planner_index`（0=左臂/1=右臂/2=躯干）**编号体系不同**，切勿混用。

## 5.2 IK 可达性检查

逆运动学（IK）把"末端期望位姿"反解成"关节角度"，并预先检查目标是否可达。

```bash
cd ~/letools_opensource/apps/test_kuavo_5w_adapter/04_timed_commands
python3 test_ik_accessibility.py
```

以测试 1「左臂世界系可达位姿检查」为例，调用 `check_ik_accessibility(...)`：

```python
result = hardware.check_ik_accessibility(
    is_left=True,                          # True=左臂, False=右臂
    is_local=False,                        # True=局部系, False=世界系
    is_whole_body=False,                   # True=全身运动, False=仅手臂
    pose_desired=[0.4, 0.2, 0.3, 0.0, 0.0, 0.0],  # 6维 [x,y,z,roll,pitch,yaw] 米/弧度
    total_time_desired=1.0,                # 期望运动时长（秒）
    max_attempts=5,                        # IK 求解最大尝试次数
    linear_error_max=0.005,                # 线位移容差 5mm
    angular_error_max=0.05                 # 角位移容差 ~2.86°
)
```

效果：向控制器查询左臂能否到达 `(0.4, 0.2, 0.3)`，返回是否可达、线/角位移误差、最优关节角 `q_best`。测试 4 会故意给 `(2.0, 0.0, 2.0)` 超出工作空间，预期返回不可达。

> ⚠️ 脚本头部声明此功能底层服务 `/mobile_manipulator_ik_accessibility_check` 尚未实现，当前会被跳过，等待底层团队支持。

## 5.3 Ruckig 参数调优

Ruckig 的速度/加速度/急动度限制可以按规划器单独配置，影响所有后续时序指令的运动性能。

```bash
cd ~/letools_opensource/apps/test_kuavo_5w_adapter/04_timed_commands
python3 test_ruckig_params.py
```

以测试 1「底盘位置规划器」为例，调用 `set_ruckig_planner_params(...)`：

```python
result = hardware.set_ruckig_planner_params(
    planner_index=0,                       # 0=底盘世界系位置（3维 x,y,yaw）
    is_sync=True,                          # True=同步模式
    velocity_max=[0.2, 0.2, 0.2],          # 最大速度 [m/s, m/s, rad/s]
    acceleration_max=[2.0, 2.0, 1.5],      # 最大加速度 [m/s², m/s², rad/s²]
    jerk_max=[20.0, 15.0, 12.0]            # 最大急动度 [m/s³, m/s³, rad/s³]
)
```

各规划器的 `planner_index` 和维度：

| planner_index | 规划器 | 维度 | 单位 |
|---|---|---|---|
| 0 | 底盘世界系位置 | 3 `[x, y, yaw]` | m/s, m/s², m/s³ |
| 2 | 躯干笛卡尔 | 4 `[x, z, yaw, pitch]` | m/s, rad/s 混合 |
| 3 | 下肢关节 | 4 关节 | rad/s, rad/s², rad/s³ |
| 8 | 左臂关节 | 7 关节 | rad/s, rad/s², rad/s³ |
| 9 | 右臂关节 | 7 关节 | rad/s, rad/s², rad/s³ |

效果：参数越大运动越快越陡，越小越慢越平滑。测试 7 会发送一条左臂时序指令 `[-20,15,10,-35,20,8,-25]°`（`desire_time=3.0`）验证新参数是否生效。

---

# 第六部分：相机与视觉

## 6.1 启动相机

```bash
python3 apps/test_camera_adapter/test_camera_init.py
```

支持 Orbbec（头部）和 RealSense（腕部）两种相机。

## 6.2 AprilTag 检测

```bash
python3 apps/test_camera_adapter/test_perception_apriltag.py
```

AprilTag 是一种视觉基准标记，机器人可以用来定位和抓取目标物体。

## 6.3 RViz 可视化

```bash
python3 apps/test_camera_adapter/test_camera_rviz.py
```

RViz 可以查看相机的 RGB 图像、深度图、点云。

---

# 第七部分：行为树编排

行为树是一种把多个动作按逻辑排列的方式。你可以用 JSON 文件描述"先移动底盘 → 再伸手 → 等待 2 秒 → 抓取"，然后用一行命令加载执行。本部分以 `refactored_sdk_atomic_v1` 场景为例，解释行为树的工作架构与原理。

## 7.1 一个场景的目录结构

每个行为树场景是 `orchestration/scenarios/` 下的一个目录，包含三个 JSON 文件：

```text
orchestration/scenarios/refactored_sdk_atomic_v1/
├── py_tree.json         # 主树：顶层执行流程
├── py_tree_child.json   # 子树：可复用的动作组合
├── board.json           # 黑板：全局共享变量初始值
└── readme.md            # 场景说明
```

| 文件 | 作用 |
|---|---|
| `py_tree.json` | 主流程，定义顶层顺序、并行、子树引用和 Action 节点 |
| `py_tree_child.json` | 子树库，把一组动作打包成可复用模板 |
| `board.json` | 黑板初始数据，如手臂轨迹、全局参数、任务变量 |

## 7.2 分层架构

行为树从 JSON 到机器人执行，经过以下层次（自上而下）：

```text
JSON 编排层（py_tree.json + py_tree_child.json）   ← 你定义"做什么、按什么顺序"
    ↓
原子节点层（orchestration/nodes/*.py）               ← 每个节点 = py_trees Behaviour
    ↓
原子技能层（skills/atomic/refactored_sdk/*.py）      ← 每个技能 = 对 IHardware 的一次调用
    ↓
硬件适配层（adapters/hardware/leju_wheeled/*.py）    ← IHardware 接口实现
    ↓
Kuavo Humanoid SDK                                   ← robot_sdk.control.* 底层 API
```

**关注点分离**：JSON 只管编排（"做什么"），Python 节点只管执行（"怎么做"），Skill 只管适配器调用。

## 7.3 主树结构（py_tree.json）

`refactored_sdk_atomic_v1` 的主树是一个 `Sequence`（顺序执行），包含 5 个子节点：

```text
Sequence (memory=true)
  ├─ WaitForEnter            ① 等待用户按 Enter 确认
  ├─ demo_cmd_pose_base.json ② 子树：底盘位姿（前进/后退/平移/旋转/整圈/复合）
  ├─ demo_leg_arm_parallel.json ③ 子树：腿臂并行运动（两相位×两重复）
  ├─ demo_head_control.json  ④ 子树：头部扫视（居中/左/右/上/下/扫描）
  └─ ArmResetSdkMove         ⑤ 手臂安全复位
```

**节点类型的判别规则**（由 `BehaviorTreeFactory` 在加载时判断）：

| `name` 字段 | 判别 | 处理方式 |
|---|---|---|
| `Sequence` / `Parallel` / `Async` 等 | 复合节点 | 创建 py_trees 内置复合节点 |
| `xxx.json` | 子树引用 | 从 `py_tree_child.json` 查找并深拷贝构建 |
| `HeadControlSdkMove` 等 Python 类名 | 原子 Action | 动态 `importlib` 导入同名类 |

## 7.4 子树与并行（py_tree_child.json）

`py_tree_child.json` 定义了 3 个子树。以 `demo_leg_arm_parallel.json` 为例，展示行为树的核心能力——**并行执行**：

```text
Sequence
  ├─ Parallel (success_on_all) "phase1"        ← 腿+臂 同时归零/展开
  │   ├─ Async → LegJointSdkMove   (leg_zero_3s)        腿部 4 关节回零，3 秒
  │   └─ Async → ArmJointTrajSdkMove (arm_home_to_spread_3s) 手臂展开，3 秒
  ├─ WaitSeconds (1.0)
  ├─ Parallel (success_on_all) "phase2"        ← 腿+臂 同时目标/回收
  │   ├─ Async → LegJointSdkMove   (leg_target_3s)      腿部到 [14.90,-32.01,18.03,-45]°
  │   └─ Async → ArmJointTrajSdkMove (arm_bend_to_home_3s)  手臂回收归零
  ├─ Parallel "phase1_repeat"                  ← 重复 phase1
  ├─ WaitSeconds (1.0)
  ├─ Parallel "phase2_repeat"                  ← 重复 phase2
  └─ TorsoResetSdkMove                         ← 躯干复位
```

`Async` 是自定义装饰器（[async_decorator.py](../../orchestration/nodes/async_decorator.py)），在独立线程中 tick 子节点。两个 `Async` 放在 `Parallel (success_on_all)` 下，**腿和臂可以真正同时运动**，而非常规的交替 tick。

## 7.5 节点参数与黑板（board.json）

每个节点的 `params` 字段描述输入参数，参数有三种来源：

| `source` | 含义 |
|---|---|
| `CUSTOM` | 固定值，直接使用 `value` |
| `INPUT` | 需宏替换，将 interface 输入映射到节点参数 |
| `READ_BOARD` | 从黑板读取 |

以头部子树的 `look_left_30` 节点为例：

```json
{
  "name": "HeadControlSdkMove",
  "label": "look_left_30",
  "params": {
    "yaw_deg":   { "value": "30.0", "source": "CUSTOM", "data_type": "float" },
    "pitch_deg": { "value": "0.0",  "source": "CUSTOM", "data_type": "float" }
  }
}
```

`board.json` 定义黑板初始数据，所有节点可读写。本场景的手臂轨迹就存在黑板上：

```json
{
  "ArmJointTrajectories": {
    "times": [0.0, 3.0],
    "q_frames": [
      [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
      [-30.0, 20.0, 15.0, -45.0, 25.0, 10.0, -35.0, -30.0, -20.0, -15.0, -45.0, -25.0, -10.0, -35.0]
    ]
  }
}
```

当 `ArmJointTrajSdkMove` 节点的 `use_board_trajectory = "true"` 时，会从黑板读取 `q_frames` 作为 14 关节轨迹；`"false"` 则用节点自身的 `joint_traj` 参数。

## 7.6 从 JSON 到机器人的完整链路

以 `demo_leg_arm_parallel.json` 中 `leg_zero_3s` 节点为例，展示一次完整调用：

```text
py_tree_child.json
  │  {"name": "LegJointSdkMove", "params": {"j0":"0.0", ..., "total_time":"3.0"}}
  ▼
BehaviorTreeFactory._build_tree_recursive()
  │  解析 name="LegJointSdkMove" → 动态导入 orchestration.nodes.leg_joint_sdk_move
  ▼
LegJointSdkMove.initialise()       ← 行为树首次 tick 该节点
  │  从 params 构建 LegJointSdkParams(joint_angles=[0,0,0,0], total_time=3.0)
  │  创建 LegJointSdkSkill(hardware=get_shared_hardware()) 并 initialize
  ▼
LegJointSdkMove.update()           ← 每个后续 tick
  │  skill.execute() → on_execute()
  ▼
LegJointSdkSkill.on_execute()
  │  hardware.send_leg_joint_sdk(joint_angles=[0,0,0,0], total_time=3.0)
  ▼
SDKControlMixin.send_leg_joint_sdk()
  │  → LowLevelSDKManager.move_wheel_lower_joint_auto()
  │  → robot_sdk.control.control_wheel_lower_joint(...)
  ▼
机器人腿部执行
```

## 7.7 运行与编写自己的行为树

运行场景（先离线验证再真机）：

```bash
cd ~/letools_opensource
# 离线验证（无需 ROS）
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/refactored_sdk_atomic_v1 --dry-run --tick-once

# 真机运行（需要 ROS + 控制器）
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/refactored_sdk_atomic_v1
```

编写自己的场景：

1. 复制 `refactored_sdk_atomic_v1/` 目录到 `orchestration/scenarios/<your_scenario>/`
2. 修改 `board.json` 的黑板变量、`py_tree_child.json` 的子树、`py_tree.json` 的主流程
3. 节点参数改为你要的值（如底盘前进 1.0 米、手臂不同姿态）
4. 若需要新动作，在 `skills/atomic/refactored_sdk/` 加技能，在 `orchestration/nodes/` 加节点类
5. 先 `--dry-run --tick-once` 验证 JSON 结构正确，再仿真，最后真机

> 📘 详细构建说明见场景自带的 [readme.md](../../orchestration/scenarios/refactored_sdk_atomic_v1/readme.md)。

---

# 附录：常见问题

> 以下内容来自仓库 [README.md](../README.md) 的「常见问题」与「三种硬件控制路径」章节。

## Q1：`catkin build` 报找不到 empy

报错：`Unable to find either executable 'empy' or Python module 'em'`

```bash
sudo apt install -y python3-empy
which empy3
python3 -c "import em; print(em.__file__)"
catkin config --cmake-args -DEMPY_EXECUTABLE=/usr/bin/empy3
catkin build
```

## Q2：已安装 python3-empy，但报 `No module named 'catkin_pkg'`

如果日志中出现 anaconda 路径，说明 conda Python 抢占了 ROS 编译环境。

```bash
conda deactivate
hash -r
which python3
python3 -c "import catkin_pkg; print(catkin_pkg.__file__)"
source /opt/ros/noetic/setup.bash
catkin config --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3 -DEMPY_EXECUTABLE=/usr/bin/empy3
catkin build
```

## Q3：realsense2_camera 编译失败

如果暂时不用 RealSense：

```bash
catkin config --skiplist realsense2_camera kuavo_camera kuavo_tf2_web_republisher
catkin build
```

需要 RealSense 时再安装 `librealsense2-dev librealsense2-utils`，如果 apt 找不到包，需要先配置 Intel RealSense 源。

## Q4：`scripts/install_sdk.sh` 找不到

`install_sdk.sh` 在项目根目录，不在 ROS 工作空间里。

```bash
cd ~/LeTools
chmod +x scripts/install_sdk.sh
./scripts/install_sdk.sh
```

## Q5：SDK submodule 克隆超时

典型错误：`fatal: 无法访问 ... Operation timed out`、`SDK 目录不存在: drivers/leju/kuavo_humanoid_sdk/src/kuavo_humanoid_sdk`

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

## Q6：三种硬件控制路径怎么选？

`LejuWheeledArmHardware` 支持三类控制方式：

| 控制方式 | 方法特征 | 底层路径 | 适合场景 |
|---|---|---|---|
| **标准接口** | `send_base_pose`, `control_head`, `arm_reset` | ROS 话题/服务或封装后的 SDK | 普通应用、Skill、行为树 |
| **SDK 直调** | `*_sdk`，如 `control_head_sdk` | Core SDK Manager → `kuavo_humanoid_sdk` | 高频控制、SDK 示例、底层验证 |
| **TimedCmd** | `*_timed`, `send_timed_*` | TimedCmdManager → ROS 服务 | 带时间规划、多规划器、Ruckig、离线轨迹 |

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