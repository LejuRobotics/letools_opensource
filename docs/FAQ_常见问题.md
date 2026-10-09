# LeTools FAQ：常见问题与自检手册

[返回 README](../README.md) · [零基础教程](beginner_tutorial.md) · [仿真开发指南](仿真开发指南.md)

> 面向第一次部署 LeTools 的使用者。按工具链分块整理环境配置、依赖缺失、通信超时和运行失败等典型问题。每项均给出判断命令、修复命令、成功标准和日志位置。

## 0. 所有问题先做这三步

### 0.1 确认自己在哪个环境

LeTools 与 `kuavo-ros-opensource` 是两个项目；宿主机和 Docker 容器也是两个环境。命令不要混用。

```bash
whoami
hostname
pwd
python3 -c 'import sys; print(sys.executable)'
```

推荐目录关系：

```text
/home/<用户名>/letools_opensource
/home/<用户名>/kuavo-ros-opensource-1.4.5
```

Docker 中通常看到 `/root/kuavo_ws`，这是宿主机 `kuavo-ros-opensource` 的挂载路径，不要求目录名相同。不要把 LeTools 放进 `kuavo-ros-opensource` 内部，也不要把宿主机和容器生成的 `build/devel` 混用。

### 0.2 运行只读自检

```bash
cd /path/to/letools_opensource
bash scripts/check_environment.sh
```

严格模式会把警告也作为非零退出：

```bash
bash scripts/check_environment.sh --strict
echo $?
```

退出码：`0` 无阻断项；`1` 存在必须修复项；`2` 表示严格模式下仍有警告。

| 代码 | 含义 | 首选处理 |
|---|---|---|
| `E001` | 当前不是 Linux | 使用 Ubuntu 20.04；Windows 只用于阅读和静态编辑 |
| `E002` | 缺少 Python 或 Git | 用系统包管理器安装后重开终端 |
| `E003` | ROS Noetic 未安装 | 安装 ROS Noetic 并加载 `/opt/ros/noetic/setup.bash` |
| `E004` | Python 低于 3.8 | 使用 Ubuntu 20.04 的系统 Python 3 |
| `E005` | SDK 锁定文件缺失/不完整 | 恢复 `sdk_version.env`，不要手填临时版本 |
| `W001` | catkin 或 Docker 缺失 | 按编译 ROS、运行容器的实际需要安装 |
| `W002` | Conda 正在覆盖系统 Python | `conda deactivate` 后重新加载 ROS 环境 |
| `W003`–`W006` | NumPy、empy、消息包或 SDK 异常 | 按本文“环境与 SDK”章节处理 |
| `W007`–`W008` | ROS 工作空间未编译或机器人版本未设置 | 编译消息包；仿真前设置正确 `ROBOT_VERSION` |
| `W009`–`W010` | ROS master 或控制服务不可用 | 按“ROS 通信”章节逐层定位 |
| `W011` | GPU或容器工具链不可用 | dry-run 可忽略；GPU仿真按“Docker与GPU”处理 |
| `W012` | 项目盘剩余空间少于 10 GB | 清理缓存或扩容后再编译 |

### 0.3 只看第一个真实错误

- catkin：找第一个 `Failed <<<`，后续 `Abandoned <<<` 通常只是连带结果；
- roslaunch：找第一个退出的 `REQUIRED process`；
- Python：从 traceback 最后一行确认异常类型，再向上找第一个 LeTools 文件；
- 动态库：查看第一条 `not found`，不要用软链接伪装不同 ABI；
- 通信超时：先确认服务端节点是否存活，不要只反复增加 timeout。

## 1. 环境与 catkin 编译工具

### 1.1 Conda 抢占 ROS Python

**现象**：日志包含 Anaconda/Miniconda 路径，或出现 `No module named catkin_pkg`、`em`、`rospkg`。

**判断命令**

```bash
which python3
python3 -c 'import sys; print(sys.executable); print(sys.path)'
echo "$CONDA_PREFIX"
```

**解决命令**

```bash
conda deactivate
hash -r
source /opt/ros/noetic/setup.bash
cd ~/letools_opensource/infrastructure/ros_packages
catkin config --cmake-args \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DEMPY_EXECUTABLE=/usr/bin/empy3 \
  -DCMAKE_ASM_COMPILER=/usr/bin/as \
  -DCMAKE_BUILD_TYPE=Release
catkin build
```

**成功标准**：`which python3` 为 `/usr/bin/python3`，catkin Summary 中 `Failed: None`。

**日志位置**：`infrastructure/ros_packages/logs/<包名>/build.*.log`。

### 1.2 缺少 empy

**现象**：`Unable to find either executable 'empy' or Python module 'em'`。

```bash
sudo apt update
sudo apt install -y python3-empy
/usr/bin/python3 -c 'import em; print("empy ready")'
cd ~/letools_opensource/infrastructure/ros_packages
catkin config --cmake-args -DEMPY_EXECUTABLE=/usr/bin/empy3
catkin build
```

**成功标准**：输出 `empy ready`，重新编译不再出现 empy 错误。

### 1.3 缺少 gflags 或其他系统头文件

**现象**：`fatal error: gflags/gflags.h: No such file or directory`。

```bash
sudo apt update
sudo apt install -y libgflags-dev
dpkg -L libgflags-dev | grep gflags.h
catkin build
```

安装后通常不用清理已成功的软件包。若错误是其他 `.h` 文件，应先确认正确的软件包名称，不要随意把头文件复制到 `/usr/include`。

### 1.4 找不到 AprilTag、YOLO 或相机包

仅做 SDK、行为树或无视觉仿真时，可以跳过这些包：

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

不要跳过 `kuavo_tf2_web_republisher`。需要视觉功能时必须安装依赖并取消 skip，不能把 skip 后的成功当作视觉功能通过。

### 1.5 catkin 继承了已删除的旧工作空间

**现象**：`ValueError: Resultspace path '/旧路径/installed' does not exist`。

```bash
cd ~/letools_opensource/infrastructure/ros_packages
catkin config
test -f "$HOME/kuavo-ros-opensource-1.4.5/installed/setup.bash"
catkin config --extend "$HOME/kuavo-ros-opensource-1.4.5/installed"
catkin clean -y
catkin build
source devel/setup.bash
```

如果基线只在 Docker 的 `/root/kuavo_ws/installed`，宿主机不能直接引用该容器路径。应在同一环境编译，或在宿主机准备真实存在的安装空间。

## 2. LeTools SDK 安装工具

相关入口：`scripts/install_sdk.sh`、`scripts/install_local_sdk.sh`、`scripts/remove_local_sdk.sh`。

### 2.1 确认锁定版本

```bash
cd ~/letools_opensource
cat scripts/kuavo_humanoid_sdk_tools/sdk_version.env
```

版本以该文件为准，例如 `SDK_REPO_BRANCH="master"`、`SDK_REPO_TAG="1.4.5"`。Tag 是正式复现基线；即使 master 后续前进，也不能用新的 master HEAD 冒充 1.4.5。

### 2.2 子模块下载超时

**现象**：`fatal: unable to access ... timed out`。

```bash
git ls-remote https://gitcode.com/OpenLET/kuavo-ros-opensource.git
git submodule status drivers/leju/kuavo_humanoid_sdk
```

若第一条就超时，这是网络或代理问题。网络恢复后执行：

```bash
cd ~/letools_opensource
git submodule sync --recursive
git submodule update --init --recursive drivers/leju/kuavo_humanoid_sdk
chmod +x scripts/install_sdk.sh
./scripts/install_sdk.sh
```

**成功标准**：子模块目录非空，安装脚本退出码为 0。

### 2.3 SDK 安装后仍不能 import

**判断命令**

```bash
which python3
python3 -m pip show kuavo-humanoid-sdk
python3 -c 'import kuavo_humanoid_sdk; print(kuavo_humanoid_sdk.__file__)'
python3 -c 'import kuavo_msgs, ocs2_msgs; print("ROS messages ready")'
```

**解决命令**

```bash
conda deactivate
source /opt/ros/noetic/setup.bash
source ~/letools_opensource/infrastructure/ros_packages/devel/setup.bash
cd ~/letools_opensource
./scripts/install_sdk.sh
python3 -c 'from kuavo_humanoid_sdk import KuavoRobot; print("SDK Ready")'
```

**成功标准**：输出 `SDK Ready`，模块和 pip 来自同一 Python 环境。

**日志定位**：保存安装脚本完整输出，同时提供 `which python3` 和 `python3 -m pip --version`。

## 3. ROS master、Topic 与 Service 通信工具

### 3.1 ROS master 无法连接

**现象**：`Unable to communicate with master`、连接拒绝或持续超时。

```bash
echo "$ROS_MASTER_URI"
echo "$ROS_IP"
echo "$ROS_HOSTNAME"
rosnode list
```

本机 master 的常见配置：

```bash
export ROS_MASTER_URI=http://localhost:11311
roscore
```

宿主机与容器联合运行时，Docker 应使用 host 网络，双方必须访问同一个 ROS master。不要同时设置互相冲突的 `ROS_IP` 与 `ROS_HOSTNAME`。

**成功标准**：`rosnode list` 能返回节点列表。

**日志位置**：`~/.ros/log/latest/`；先看 roscore 和第一个退出节点。

### 3.2 Topic 没数据或频率过低

```bash
rostopic list
rostopic info /目标topic
rostopic type /目标topic
rostopic echo -n 1 /目标topic
rostopic hz /目标topic
rosnode info /发布节点
```

- Topic 不存在：服务端节点未启动或命名空间错误；
- Topic 存在但没有 publisher：上游节点已退出；
- 有 publisher 但没数据：检查设备、仿真时钟和节点日志；
- 类型不一致：重新加载同一工作空间的消息包，不要混用旧 `devel`。

### 3.3 Service 超时或不存在

```bash
rosservice list | grep '<服务关键字>'
rosservice info /目标service
rosservice type /目标service
rosnode info /服务节点
```

如果 `/humanoid_controller/get_controller_list` 不存在，先确认控制器节点是否存活，再核对 `kuavo-ros-opensource` 与 SDK 版本。反复调用客户端或增加 timeout 不能修复未启动的服务端。

## 4. Docker、NVIDIA GPU 与 MuJoCo 工具

### 4.1 `-v` 为什么不能单独执行

`-v` 是 `docker run` 的参数，不是命令。已有容器不会因为单独输入 `-v` 自动增加挂载。

```bash
docker inspect <容器名> --format '{{range .Mounts}}{{println .Source "->" .Destination}}{{end}}'
```

需要挂载 LeTools 时，应修改创建容器的 `docker run`/`docker/run.sh` 后重新创建，或让 LeTools 在宿主机运行、通过 ROS 连接容器。

### 4.2 GPU 是否已经配置

以下命令在宿主机执行：

```bash
nvidia-smi
nvidia-ctk --version
docker info | grep -i runtime
docker run --rm --gpus all nvidia/cuda:12.0.0-base-ubuntu20.04 nvidia-smi
```

Docker `Runtimes` 已包含 `nvidia` 且测试容器能看到 GPU，就无需重复安装。无 GPU 仍可进行编辑、静态检查、单元测试和 dry-run。

### 4.3 MuJoCo 卡顿

```bash
top
free -h
df -h
nvidia-smi
rostopic hz /clock
docker stats
```

依次确认 CPU/内存/磁盘是否耗尽、`/clock` 是否稳定、控制器是否重启、GPU 容器是否获得 GPU，以及是否同时运行视觉、录包或多个仿真实例。

### 4.4 OpenVINO 动态库版本不一致

**现象**：控制器需要 `libopenvino.so.2520`，环境只有 `libopenvino.so.2330`。

```bash
find /opt/intel /usr /usr/local -name 'libopenvino.so*' 2>/dev/null
ldd /root/kuavo_ws/devel/lib/libnodelet_controller.so | grep -E 'openvino|not found'
```

`2520` 与 `2330` 是不同 ABI。不要创建假软链接，应使用配套 Docker 镜像，或在当前环境清理并重新编译相关控制器。

### 4.5 `libkuavo_assets.so` 找不到

```bash
find /root/kuavo_ws -name 'libkuavo_assets.so*'
ldd /root/kuavo_ws/devel/lib/libnodelet_controller.so | grep 'not found'
cd /root/kuavo_ws
catkin build kuavo_assets -p 1
catkin build humanoid_interface_ros -p 1
catkin build humanoid_wheel_interface_ros -p 1
catkin build humanoid_controllers -p 1
```

单线程按依赖顺序补编译后，再查看最后一个包的 Summary 和链接日志。

## 5. 行为树、JSON 与场景工具

### 5.1 单场景和批量 dry-run

```bash
cd ~/letools_opensource
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/studio_smoke_v1 \
  --dry-run --tick-once
```

**成功标准**：退出码为 0，主树、子树和节点类能够正常加载。

### 5.2 节点类、子树或 board 引用失败

```bash
python3 -m json.tool orchestration/scenarios/<场景>/py_tree.json >/dev/null
python3 -m json.tool orchestration/scenarios/<场景>/py_tree_child.json >/dev/null
python3 -m json.tool orchestration/scenarios/<场景>/board.json >/dev/null
```

核对 Python 类名大小写、子树键名、`READ_BOARD` 的 `board_key`，以及三个 JSON 是否在同一场景目录。

### 5.3 dry-run 通过但 MuJoCo 失败

dry-run 不验证 ROS 服务、动态库、模型、控制器、运动结果或安全边界。继续检查：

```bash
rosnode list
rosservice list
rostopic hz /clock
grep -R "ERROR\|FATAL" ~/.ros/log/latest/ | head -50
```

再查看场景目录中的 README，确认该场景是否声明支持 MuJoCo。

### 5.4 JiBot 场景不能直接仿真

`jibot_nav_v1` 依赖 JiBot/Jarvis 调度服务，普通 MuJoCo 不提供该服务。应选择不依赖 JiBot 的场景，或按仿真环境已有的底盘节点完成适配；不能把 dry-run 通过视为调度服务已经可用。

## 6. 底盘、手臂与末端执行工具

### 6.1 嘉腾底盘不动，随后提示仍有任务执行

`move_base` 前 `/enable_vel_control` 仍为 `true` 时会与任务模式冲突。

```bash
rosservice list | grep enable_vel_control
rosservice call /enable_vel_control "data: false"
```

重新发送一次任务并检查到达状态。完整说明见[嘉腾底盘控制文档](../apps/jiateng_adapter/嘉腾底盘控制使用说明.md)。

### 6.2 手臂轨迹很慢或多次调用后越来越慢

```bash
rostopic hz /目标轨迹topic
pgrep -af humanoid_controller
```

已知 100 Hz 高频下发可能造成 MPC 积压。将上层频率降至 30 Hz，并在多个独立示例间留约 2 秒排空时间；同时检查实际执行时长、控制器 CPU 和错误日志。

### 6.3 离线轨迹不随 `desire_time` 变化

确认配套 `kuavo-ros-control` 版本，并提供轨迹时间戳、`desire_time`、控制器 commit、实际执行时长和第一条警告。不要通过反复发送轨迹掩盖底层时间修剪问题。

## 7. 相机、视觉与 TF 工具

### 7.1 相机初始化失败

```bash
lsusb
ldconfig -p | grep -E 'Orbbec|realsense'
find /usr /usr/local -name 'libOrbbecSDK.so*' 2>/dev/null
rostopic list | grep -E 'camera|image|depth'
```

Orbbec 常见问题是 `libOrbbecSDK.so.1.10` 损坏或链接缺失。应从正式安装包或同版本仓库恢复。只有 RGB、深度 Topic 持续有数据才算初始化成功。

### 7.2 能看到 AprilTag 但检测不到

```bash
cat config/apriltag_tags.yaml
rostopic hz /相机图像topic
rostopic echo -n 1 /相机内参topic
rosnode info /apriltag节点
```

核对 tag 家族、ID、实际边长、图像 Topic 和 `camera_info`。仓库示例可能按 10 cm 标签配置，实际尺寸不同时必须修改配置。

### 7.3 TF republisher 找不到包或 executable

```bash
cd ~/letools_opensource/infrastructure/ros_packages
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rospack find kuavo_tf2_web_republisher
./start_tf_republisher.sh
```

若 `rospack find` 失败，检查该包是否被 skip，以及当前终端是否加载了这个工作空间。

## 8. 开源版本、发布与兼容工具

### 8.1 文档写 dev，但开源仓没有该分支

开源使用者以仓库实际存在的 release/master 和 `scripts/kuavo_humanoid_sdk_tools/sdk_version.env` 为准。内部 dev 不是公开安装入口。

### 8.2 路径中出现 `.opensource_artifact`

这可能是旧版 `.pyc` 制成品残留：

```bash
cd ~/letools_opensource
find adapters core skills orchestration -name '*.pyc' -o -name '__pycache__'
git log --oneline -1
```

拉取修复后的正式版本并重新安装，不要把旧 `.pyc` 复制到新源码目录。

### 8.3 V63 或特殊底盘版本对齐后仍失败

特殊机型可能需要专用分支，但分支名和硬件范围必须由维护者确认。反馈时提供机器人型号、硬件版本、下位机 commit、LeTools commit、SDK tag 和缺失 Service。不要把未确认的 beta 分支直接用于真机。

## 9. 日志收集与问题反馈

### 9.1 收集基础信息

```bash
cd ~/letools_opensource
git branch --show-current
git rev-parse HEAD
python3 --version
rosversion -d
docker --version
nvidia-smi
```

反馈时只提供定位所需的版本和错误日志。发送前逐项检查并删除用户名、IP、令牌、图像、rosbag及其他客户数据；需要完整日志时由支持人员明确指定范围后再提供。

### 9.2 日志位置

| 工具 | 日志位置/命令 |
|---|---|
| catkin | `infrastructure/ros_packages/logs/<包名>/` |
| ROS/roslaunch | `~/.ros/log/latest/` |
| Docker | `docker logs --tail 300 <容器名>` |
| systemd | `journalctl -u <服务名> -n 300 --no-pager` |
| Python/LeTools | 保存完整 traceback，不要只截最后一行 |
| GPU | 保存 `nvidia-smi` 和 `docker info` 输出 |

### 9.3 反馈模板

```text
问题所属工具：环境/catkin/SDK/ROS/Docker/MuJoCo/行为树/底盘/手臂/相机/TF
机器人型号与硬件版本：
LeTools 分支与 commit：
SDK 分支/tag：
kuavo-ros-opensource tag 与 commit：
Docker 镜像名称/摘要：
ROBOT_VERSION：
执行环境：宿主机或容器
完整运行命令：
第一个 ERROR/FATAL/Failed：
相关日志路径：
是否稳定复现：
已经执行的排查命令及结果：
支持包文件名：
```

外部用户可前往 [GitCode Issues](https://gitcode.com/OpenLET/letools_opensource/issues) 提交问题。涉及真机运动、安全参数或急停时，应停止尝试并联系维护者，不要扩大阈值或无限重试绕过故障。
