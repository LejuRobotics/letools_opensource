# Frequently Asked Questions

> 📋 整理自 [飞书 LeTools 问题反馈表]，持续更新。
> 最后更新：2026-08-08

---

## 一、环境搭建

### Q: `catkin build` 报错，提示找不到某些包（如 `apriltag_ros`、`realsense2_camera` 等）

编译仿真环境不需要相机和视觉相关的包，可以先跳过：

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

> ⚠️ **注意**：`kuavo_tf2_web_republisher` 不要加入 skip 列表，否则后续 `start_tf_republisher.sh` 会启动失败。

### Q: `catkin build` 报 `Unable to find either executable 'empy' or Python module 'em'`

缺少 `python3-empy` 包：

```bash
sudo apt install -y python3-empy
catkin config --cmake-args -DEMPY_EXECUTABLE=/usr/bin/empy3
catkin build
```

### Q: 编译日志出现 anaconda 路径，报 `No module named 'catkin_pkg'`

conda 的 Python 抢占了 ROS 编译环境：

```bash
conda deactivate
hash -r
source /opt/ros/noetic/setup.bash
catkin config --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3 -DEMPY_EXECUTABLE=/usr/bin/empy3
catkin build
```

### Q: 真机上 catkin build 某些库编译失败

如果是从 Windows 下载压缩包再拖到机器人上解压的，可能会出现 git 指向丢失导致部分包编译失败。

**解决**：SSH 连上机器人后直接 `git clone`，或在本机 `git clone` 后复制到机器人。

---

## 二、SDK 安装

### Q: `scripts/install_sdk.sh` 执行失败，子模块克隆超时

典型错误：`fatal: 无法访问 ... Operation timed out`

先测试网络连通性：

```bash
git ls-remote https://gitcode.com/OpenLET/kuavo-ros-opensource.git
```

如果超时，需要换网络或配置代理。失败后建议恢复主仓库状态：

```bash
cd ~/LeTools
git sparse-checkout disable 2>/dev/null || true
git submodule deinit -f drivers/leju/kuavo_humanoid_sdk 2>/dev/null || true
rm -rf drivers/leju/kuavo_humanoid_sdk
rm -rf .git/modules/drivers/leju/kuavo_humanoid_sdk
```

### Q: SDK 安装成功但 `from kuavo_humanoid_sdk import KuavoRobot` 报 import 错误

SDK 子模块可能没下载成功。检查 `drivers/leju/kuavo_humanoid_sdk` 目录是否为空或不存在，如果是，重新执行：

```bash
cd ~/letools_opensource
rm -rf drivers/leju/kuavo_humanoid_sdk
chmod +x scripts/install_sdk.sh
./scripts/install_sdk.sh
```

### Q: SDK 安装后运行脚本提示找不到模块

确保 SDK 版本与下位机匹配。LeTools 配套的 kuavo-ros-opensource 分支与 tag 记录在 `scripts/kuavo_humanoid_sdk_tools/sdk_version.env` 中。当前配套版本为 **tag 1.4.5 (master 分支)**。确认下位机和 LeTools 的 SDK 版本一致。

---

## 三、开源版安装问题

### Q: 运行示例脚本报 `ModuleNotFoundError: No module named 'xxx'`，路径中包含 `.opensource_artifact`

这是 LeTools 开源版曾出现的问题：CI 流水线将 `.py` 编译为 `.pyc` 后分发了包含硬编码 CI 环境绝对路径的字节码文件。

**当前状态：已修复。** 如果你仍遇到此问题，请确保拉取了最新的 master 分支/版本，并确认不再有 `.pyc` 文件分发：

- `adapters/hardware/leju_wheeled/hardware.py` 应为源码
- `adapters/hardware/leju_wheeled/mixins/__init__.py` 应为源码
- `core/`、`skills/`、`orchestration/` 目录下均不应只有 `.pyc` 文件

### Q: 阅读文档时发现某些包或模块引用了 `_internal` 路径

开源版的 `adapters/` 层曾引用了内部模块。**已修复**，当前版本已移除这些引用。

---

## 四、版本兼容性

### Q: 运行 SDK 控制脚本提示 `/humanoid_controller/get_controller_list` 服务不可用，MPC 模式设置失败

**根因**：LeTools 版本与下位机 kuavo-ros-opensource 版本不匹配，导致该服务未被启动。

**解决**：确保版本对齐——
- 下位机 kuavo-ros-opensource 使用 **tag 1.4.5**
- LeTools 安装 **1.4.5** 对应的 SDK

验证方法：

```bash
# 查看下位机 commit
cd ~/kuavo-ros-opensource && git log --oneline -1

# LeTools 侧使用匹配的 commit
cd ~/letools_opensource && git log --oneline -1
```

### Q: 版本对齐后仍然不行，特别是嘉腾底盘 V63 机器

部分 V63 机型需使用 kuavo-ros-opensource 的特定分支（beta 分支或 `opensource/lb/add_v63_bybeta_20260512_new` 分支）。请根据机器人型号确认使用的分支。

---

## 五、仿真启动

### Q: 启动 MuJoCo 仿真报错或卡顿

- 确保已设置 `ROBOT_VERSION` 环境变量（常用 62 或 45）：
  ```bash
  export ROBOT_VERSION=62
  ```
- 如启动后卡顿，使用 GPU 版本启动：
  ```bash
  ./docker/run_with_gpu.sh
  ```
- NVIDIA 4090 系列需额外配置 NVIDIA Container Toolkit

---

## 六、手臂控制

### Q: 手臂末端轨迹执行非常慢，或多次调用后越来越慢

**根因**：下发频率过高（100Hz），机器人 MPC 节点处理不过来，导致线程阻塞。

**解决**：
1. 下发频率调整为 30Hz
2. 多个示例脚本之间加 `time.sleep(2)` 让之前的进程排空

### Q: 离线轨迹（`set_offline_trajectory`）执行速度不随 `desire_time` 变化

**根因**：底层控制器的时间修剪逻辑用了"绝对时间"而非"相对时间"，会把尚未执行的点当作过期点删掉，导致控制器直接追终点。

**解决**：已在 `MobileManipulatorReferenceManager.cpp` 中将绝对时间修剪改为自定义修剪，且每次 `set_offline_trajectory` 先清空旧缓存。请确认使用最新版 kuavo-ros-control。

---

## 七、底盘控制

### Q: 嘉腾底盘下发移动指令后底盘不动，到达检测超时；再次下发提示"仍有任务在执行"

**根因**：调用 `move_base` 前 `/enable_vel_control` 仍为 `true`，与嘉腾底盘控制模式冲突。

**解决**：在调用 `move_base` 前显式关闭速度控制：

```bash
rosservice call /enable_vel_control "data: false"
```

### Q: 嘉腾底盘相关哪里有完整文档？

参考 [嘉腾底盘使用文档](LeTools/apps/jiateng_adapter/嘉腾底盘控制使用说明.md)。

---

## 八、相机与视觉

### Q: 运行 `test_camera_init.py` 相机启动失败

**常见原因**：Orbbec 相机的动态库 `libOrbbecSDK.so.1.10` 文件损坏或符号链接缺失。

**解决**：
1. 从正常机器或 Git 仓库恢复损坏的动态库文件
2. 确保启用相机实际收到 RGB 和深度数据后才判定初始化成功

### Q: 运行 `test_perception_apriltag.py`，相机能看到二维码但检测不到

**解决**：检查 AprilTag 配置参数是否正确——

```bash
# 确认配置文件中的 tag 尺寸与实际使用的标签一致
cat config/apriltag_tags.yaml
```

测试所用标签为 **10cm × 10cm**，如实际标签尺寸不同需修改配置。

---

## 九、tf_republisher 启动

### Q: `./start_tf_republisher.sh` 提示找不到包或 executable

**常见原因**：
1. 之前 `catkin config --skiplist` 跳过了 `kuavo_tf2_web_republisher` → 解决方法：**不要跳过该包**
2. 不在 `infrastructure/ros_packages/` 目录下运行 → **必须在 ros_packages 目录下执行该脚本**

```bash
cd ~/letools_opensource/infrastructure/ros_packages
source devel/setup.bash
./start_tf_republisher.sh
```

---

## 十、文档使用

### Q: 文档中提到 dev 分支，但实际仓库没有这个分支

LeTools 开源版没有 dev 分支，使用 **master 分支**即可。文档中相关描述已修正。

### Q: 我是外部用户，如何反馈问题？

前往 [GitCode Issues](https://gitcode.com/OpenLET/letools_opensource/issues) 提交 Issue。

---

## 快速诊断清单

遇到问题时，按以下顺序排查：

- [ ] `ROBOT_VERSION` 是否正确设置？
- [ ] ROS 环境是否已 source：`source /opt/ros/noetic/setup.bash`
- [ ] 工作空间是否已 source：`source ~/letools_opensource/infrastructure/ros_packages/devel/setup.bash`
- [ ] LeTools 版本与下位机 kuavo-ros-opensource 版本是否对齐（当前：tag 1.4.5）？
- [ ] `kuavo_tf2_web_republisher` 是否在后台运行？
- [ ] 仿真是否已启动（`roslaunch humanoid_controllers load_kuavo_mujoco_sim_wheel.launch`）？
- [ ] SDK 是否安装成功：`python3 -c 'from kuavo_humanoid_sdk import KuavoRobot; print("SDK Ready!")'`
