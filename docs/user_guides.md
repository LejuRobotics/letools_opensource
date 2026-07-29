# 📖 LeTools 用户指南

| 📁 文档路径 | 🤖 适用机器人 | 📋 内容 |
|:---|:---|:---|
| `docs/user_guides.md` | Kuavo 系列（轮式/人形） | 仿真部署 · SDK 安装 · 真机部署 |

## 📑 目录

- [🖥️ 一、仿真环境部署与跑通](#%F0%9F%96%A5%EF%B8%8F-%E4%B8%80%E4%BB%BF%E7%9C%9F%E7%8E%AF%E5%A2%83%E9%83%A8%E7%BD%B2%E4%B8%8E%E8%B7%91%E9%80%9A)
  - [🐳 1. 环境部署](#%F0%9F%90%B3-1-%E7%8E%AF%E5%A2%83%E9%83%A8%E7%BD%B2)
  - [🛠️ 2. 工具链部署与跑通](#%F0%9F%9B%A0%EF%B8%8F-2-%E5%B7%A5%E5%85%B7%E9%93%BE%E9%83%A8%E7%BD%B2%E4%B8%8E%E8%B7%91%E9%80%9A)
  - [🧪 3. 测试问题](#%F0%9F%A7%AA-3-%E6%B5%8B%E8%AF%95%E9%97%AE%E9%A2%98)
- [🔧 二、真机部署跑通指南](#%F0%9F%94%A7-%E4%BA%8C%E7%9C%9F%E6%9C%BA%E9%83%A8%E7%BD%B2%E8%B7%91%E9%80%9A%E6%8C%87%E5%8D%97)
  - [🤖 1. 下位机部署与跑通](#%F0%9F%A4%96-1-%E4%B8%8B%E4%BD%8D%E6%9C%BA%E9%83%A8%E7%BD%B2%E4%B8%8E%E8%B7%91%E9%80%9A)
  - [🖥️ 2. 上位机部署跑通](#%F0%9F%96%A5%EF%B8%8F-2-%E4%B8%8A%E4%BD%8D%E6%9C%BA%E9%83%A8%E7%BD%B2%E8%B7%91%E9%80%9A)

---

> 📘 本文档涵盖从仿真环境到真机部署的完整流程，包括 Docker 镜像部署、MuJoCo 仿真验证、SDK 安装、以及真机跑通。


## 🖥️ 一、仿真环境部署与跑通

### 🐳 1. 环境部署

#### 📦 1.1 Docker 镜像部署

仓库地址：https://gitcode.com/OpenLET/kuavo-ros-opensource/tree/dev/
请下载与当前 LeTools 配套的 kuavo-ros-opensource 分支与 tag（如 master / 1.4.4），则将此仓库下的 master分支，tag为1.4.4,下载到本地，git clone -b 1.4.4 https://gitcode.com/OpenLET/kuavo-ros-opensource.git

根据 readme.md 文档跑通 docker 环境，注意 Ubuntu 20.04 需要赋予 **sudo 权限**。

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

#### 🤖 1.2 MuJoCo 仿真验证

- 配置机器人版本

机器人版本通过环境变量 `$ROBOT_VERSION` 设置，版本号涉及不同机器人模型、硬件设置等，需要和自己的机器人匹配。在终端执行 `echo $ROBOT_VERSION` 查看当前设置的版本号，如果没有设置，通过以下设置版本号（其中的 45 代表 4.5 版本，根据实际情况修改，这里我们常用 62）：

在当前终端执行（临时设置）：

```bash
export ROBOT_VERSION=62
```

将其添加到你的 `~/.bashrc` 或者 `~/.zshrc` 终端配置文件中：如执行：

```bash
echo 'export ROBOT_VERSION=45' >> ~/.bashrc
```

添加到 `~/.bashrc` 文件末尾，重启终端后生效。

- 使用 MuJoCo 仿真器

```bash
source devel/setup.zsh  # 正常情况下，则使用 source devel/setup.bash
roslaunch humanoid_controllers load_kuavo_mujoco_sim_wheel.launch  # 轮臂机器人请使用
roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch  # 启动控制器、mpc、wbc、mujoco仿真器
```

### 🛠️ 2. 工具链部署与跑通

#### 📦 2.1 SDK 部署
乐聚员工可通过 GitLab 下载工具链项目（letools）（dev 分支）
外部用户可通过开源社区下载工具链项目（letools_opensource）：git clone https://gitcode.com/OpenLET/letools_opensource.git

##### 🔧 2.1.1 前置准备

确保你的系统已安装 Python 3.8+ 和 ROS Noetic，并且 `numpy` 版本满足要求：

```bash
pip3 install --upgrade "numpy>=1.19.5,<1.27.0"
```

##### 🔨 2.1.2 编译

在项目根目录运行以下命令，编译 ROS workspace 中的包：

```bash
cd ~/letools_opensource/infrastructure/ros_packages
catkin build
source devel/setup.bash
```
如果在部署仿真环境的话，报视觉模块编译失败，则需跳过相关包，再重新编译
```bash
cd ~/letools_opensource/infrastructure/ros_packages
catkin config --skiplist \
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
catkin build
source devel/setup.bash
```

##### ⚡ 2.1.3 一键安装（推荐）

新开一个终端，在项目根目录运行以下命令，脚本将自动完成 Submodule 初始化、分支切换、配置生成及 SDK 安装：
>📌 SDK 版本由仓库自带的 `scripts/kuavo_humanoid_sdk_tools/sdk_version.env` 锁定，
> 记录当前 LeTools 配套的 kuavo-ros-opensource 分支与 tag（如 `master` / `1.4.4`），
> 每次 LeTools 发版时同步更新，请下载对应分支与tag的kuavo-ros-opensource，以确保安装到与当前版本匹配的 SDK。

```bash
cd ~/letools_opensource
# 若是拉取过 SDK 子模块，但未初始化，则需清理残留的 submodule 数据
rm -rf drivers/leju/kuavo_humanoid_sdk
# 若是没有拉取过SDK子模块，则运行安装脚本初始化
chmod +x scripts/install_sdk.sh
./scripts/install_sdk.sh   ## 运行时间较长，耐心等待
```

##### ✅ 2.1.4 验证安装

运行以下 Python 命令，若输出 `SDK Ready!` 则代表安装成功：

```bash
python3 -c 'from kuavo_humanoid_sdk import KuavoRobot; print("SDK Ready!")'
```

> 💡 **提示**：原文档路径为 `/kuavo/docs/guides/SDK_Integration_Guide.md`

#### 🔄 2.2 kuavo_tf2_web_republisher 服务启动

```bash
# 首先编译工作空间
cd letools_opensource/infrastructure/ros_packages
source devel/setup.bash
# 这里记得用系统环境所在的 py 环境也就是 ros 所在，有虚拟环境记得做选择
./start_tf_republisher.sh
```

这里我们就可以在终端中看到服务启动了。

> ⚠️ **注意**：这个服务得一直在后台挂着，而且得在仿真开启之后启动！！！

#### 🚀 2.3 运行示例程序

`letools_opensource/apps` 目录下有以下文件夹，里面的示例脚本一般都是可以的：

- `test_kuavo_5w_adapter`
- `test_kuavo_5w_sdk_adapter`

> 💡 **推荐示例**：

- `LeTools/apps/test_kuavo_5w_sdk_adapter/sdk/01_head/test_head_control.py`
- `LeTools/apps/test_kuavo_5w_sdk_adapter/sdk/02_arm/test_arm_joint_traj.py`
- `LeTools/apps/test_kuavo_5w_sdk_adapter/sdk/03_lower_body/test_leg_joint.py`
- `LeTools/apps/test_kuavo_5w_adapter/01_base_control/test_cmd_pose_base.py`

以头部示例程序为例：

```bash
# 首先 source 一下环境变量
cd LeTools/infrastructure/ros_packages
source devel/setup.bash
# 这里记得用系统环境所在的 py 环境也就是 ros 所在，有虚拟环境记得做选择
cd LeTools/apps/test_kuavo_5w_sdk_adapter/sdk/01_head
python3 test_head_control.py
```

然后我们就可以在 MuJoCo 仿真中看到机器人头部按照预设的动作活动了。

### 🧪 3. 测试问题

#### 💻 3.1 在 Ubuntu 上测试

（待补充）

#### 🎛️ 3.2 在上位机上测试

（待补充）

---

## 🔧 二、真机部署跑通指南

> 📘 **声明**：本文档基于从仿真验证完之后往真机部署时出现的一些踩坑汇总的一个标准化流程。

### 🤖 1. 下位机部署与跑通

#### 📦 1.1 部署

```bash
# 这里我们选择下载kuavo-ros-opensource
git clone https://gitcode.com/OpenLET/kuavo-ros-opensource.git

# 编译运行
cd kuavo-ros-opensource  # 仓库目录
catkin config -DCMAKE_ASM_COMPILER=/usr/bin/as -DCMAKE_BUILD_TYPE=Release  # Important!
source installed/setup.bash  # 加载一些已经安装的ROS包依赖环境，包括硬件包等
catkin build humanoid_controllers
```

> 💡 **提示**：5-10 分钟即可完成编译

#### 📥 1.2 SDK 下载

详细内容见：`src/kuavo_humanoid_sdk/README.md`

- 安装最新的 **正式版** Kuavo Humanoid SDK：

```bash
pip install kuavo-humanoid-sdk
```

> 💡 **可选功能依赖**（按需安装）：

```bash
# 仅语音/ASR相关功能
pip install kuavo-humanoid-sdk[audio]

# 仅视觉/YOLO相关功能
pip install kuavo-humanoid-sdk[vision]

# 全量功能（包含音频 + 视觉依赖）
pip install kuavo-humanoid-sdk[full]
```

- 安装最新的 **beta 版** Kuavo Humanoid SDK：

```bash
pip install --pre kuavo-humanoid-sdk
```

- 💡 （推荐）对于本地开发安装（可编辑模式）：

```bash
cd src/kuavo_humanoid_sdk
chmod +x install.sh
./install.sh
```

#### ▶️ 1.3 运行

```bash
cd kuavo-ros-opensource
sudo su
source devel/setup.bash
roslaunch humanoid_controllers load_kuavo_real_wheel.launch
# 运行后按照提示日志按 "o"，一直后台挂着即可
```

#### ✅ 1.4 跑通

下列两个文件夹中的脚本原则上都是可以的：

- `src/kuavo_humanoid_sdk/kuavo_humanoid_sdk/kuavo_strategy_pytree/pick_place_box` — SDK 版本
- `src/demo/test_kuavo_wheel_real` — 话题版本

以下述脚本为例：

`/src/kuavo_humanoid_sdk/kuavo_humanoid_sdk/kuavo_strategy_pytree/pick_place_box/case_wheel_test_arm.py`

```bash
# 新开一个终端，运行 tf
cd kuavo-ros-opensource
sudo su
source devel/setup.bash
rosrun kuavo_tf2_web_republisher kuavo_tf2_web_republisher
```

> ⚠️ **注意**：tf 上位机下位机启动一次即可

```bash
# 新开一个终端，运行脚本
cd kuavo-ros-opensource
sudo su
source devel/setup.bash
cd /src/kuavo_humanoid_sdk/kuavo_humanoid_sdk/kuavo_strategy_pytree/pick_place_box/
python3 case_wheel_test_arm.py
```

> 💡 **提示**：观察机器人手臂运行正常即可

### 🖥️ 2. 上位机部署跑通

上位机 SSH 进去后参照仿真环境部署，跑通即可。

这里注意上位机主目录磁盘较小，推荐到 `/media/data` 中部署运行。


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