# LeTools 零基础完整学习教程（升级版）

> 适用对象：没有 ROS 开发经验、第一次接触 LeTools 的使用者。
> 内容依据：`D:\leju_robot\LeTools` 当前仓库。
> 配套仿真文档：[《LeTools 仿真开发指南》](./仿真开发指南.md)
> 常见问题参考：[FAQ自检手册](./FAQ_常见问题.md)
> 环境自检：`bash scripts/check_environment.sh`

## 1. 学习目标

完成本教程后，你应能够：

1. 理解 LeTools 的代码分层；
2. 编译 ROS 工作空间并安装配套 SDK；
3. 使用 dry-run 检查行为树；
4. 根据需要进入 MuJoCo 仿真或连接真机；
5. 运行一个已有示例；
6. 知道发生错误时应检查哪些日志和配置。

推荐学习顺序：

```text
认识目录 -> 准备系统 -> 编译ROS -> 安装SDK -> dry-run -> MuJoCo -> 真机
```

仿真部署、Docker、MuJoCo和仿真/真机能力边界已独立到[《LeTools 仿真开发指南》](./仿真开发指南.md)。本教程只保留必要入口，避免两份文档重复维护。

## 2. LeTools是什么

LeTools是乐聚Kuavo机器人上位机侧的Python技能工具链。仓库中的主要调用关系为：

```text
apps 示例和测试
    ↓
orchestration 行为树编排
    ↓
skills 原子技能
    ↓
adapters 硬件适配
    ↓
drivers / infrastructure SDK与ROS
    ↓
MuJoCo仿真或真机
```

常用目录：

| 目录 | 用途 | 新手何时需要查看 |
|---|---|---|
| `apps/` | 可运行示例和测试入口 | 想运行单项功能时 |
| `orchestration/` | 行为树节点、场景和加载器 | 想运行或修改业务场景时 |
| `skills/` | 原子技能 | 想复用或开发动作能力时 |
| `adapters/` | 将统一接口连接到具体硬件 | 排查硬件调用链时 |
| `infrastructure/ros_packages/` | ROS工作空间 | 编译ROS消息和功能包时 |
| `scripts/` | SDK安装等工具脚本 | 安装或更新SDK时 |
| `docs/` | 使用说明 | 操作前先查阅 |

## 3. 准备环境

### 3.1 已确认的推荐环境

仓库根目录`README.md`列出：

- Ubuntu 20.04；
- ROS Noetic；
- Python 3.8+；
- catkin tools；
- Kuavo SDK；
- Docker/MuJoCo仅在仿真调试时需要。

### 3.2 步骤1：确认操作系统

**前置条件**

- 已进入Ubuntu终端。

**执行命令**

```bash
lsb_release -a
```

**命令说明**

- `lsb_release`：读取Linux发行版信息；
- `-a`：显示全部可用信息。

**预期输出**

输出中应包含Ubuntu版本信息。仓库推荐Ubuntu 20.04。

**调整建议**

如果不是Ubuntu 20.04，不要直接假定兼容，应先在测试环境验证ROS Noetic、Python和依赖包。

### 3.3 步骤2：确认ROS环境

**前置条件**

- 已安装ROS Noetic。

**执行命令**

```bash
source /opt/ros/noetic/setup.bash
rosversion -d
```

**命令说明**

- `source`：把ROS环境变量加载到当前终端；
- `rosversion -d`：显示当前ROS发行版。

**预期输出**

```text
noetic
```

如果出现“文件不存在”，说明ROS Noetic尚未安装或安装路径不同。

### 3.4 步骤3：确认Python版本

```bash
python3 --version
```

**预期输出**

应为Python 3.8或更高版本。实际兼容性仍取决于依赖包。

## 4. 获取并进入项目

### 4.1 步骤4：确认项目目录

**前置条件**

- 已完成LeTools仓库克隆。

**执行命令**

```bash
cd ~/letools_opensource
pwd
ls
```

**命令说明**

- `cd`：进入项目目录；
- `pwd`：显示当前完整路径；
- `ls`：列出当前目录文件。

**预期输出**

`ls`结果中至少应看到：

```text
README.md
apps
orchestration
skills
adapters
infrastructure
scripts
```

**调整建议**

如果你的仓库文件夹名称是`LeTools`，请把命令改成实际路径，例如：

```bash
cd ~/LeTools
```

不要机械复制不存在的`~/letools_opensource`路径。

## 5. 编译ROS工作空间

### 5.1 步骤5：进入ROS工作空间

**前置条件**

- 已进入LeTools根目录；
- ROS Noetic已经安装；
- 已安装catkin tools。

```bash
cd infrastructure/ros_packages
source /opt/ros/noetic/setup.bash
```

**预期结果**

命令没有报错，并且当前目录为`infrastructure/ros_packages`。

### 5.2 步骤6：编译

```bash
catkin build
```

**命令说明**

- `catkin build`：编译当前catkin工作空间中的ROS包。

**预期输出**

末尾应显示构建完成摘要；成功包不应标记为failed。

**调整建议**

如果暂时不使用视觉相关模块，可根据仓库README使用skiplist跳过：

```bash
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
```

`--skiplist`表示本次工作空间不编译列出的包。只有确认当前任务不需要这些包时才能跳过。

### 5.3 步骤7：加载编译结果

```bash
source devel/setup.bash
```

**预期结果**

命令无报错。该命令只对当前终端生效，新开终端后需要重新执行。

## 6. 安装配套SDK

### 6.1 当前配套版本

仓库文件`scripts/kuavo_humanoid_sdk_tools/sdk_version.env`当前记录：

```bash
SDK_REPO_BRANCH="master"
SDK_REPO_TAG="1.4.5"
```

普通用户不需要单独修改`sdk_config.sh`。只有临时调试其他版本时才使用本地覆盖。

### 6.2 步骤8：运行安装脚本

**前置条件**

- 已返回LeTools根目录；
- 网络和Git访问正常；
- 已确认当前仓库版本。

```bash
cd ../..
chmod +x scripts/install_sdk.sh
./scripts/install_sdk.sh
```

**命令说明**

- `cd ../..`：从`infrastructure/ros_packages`返回LeTools根目录；
- `chmod +x`：给安装脚本增加执行权限；
- `./scripts/install_sdk.sh`：按照仓库锁定版本安装SDK。

**预期输出**

安装脚本应完成SDK获取和安装，不应以错误码退出。

### 6.3 步骤9：验证SDK

```bash
python3 -c 'from kuavo_humanoid_sdk import KuavoRobot; print("SDK Ready")'
```

**参数说明**

- `python3 -c`：直接执行引号中的Python代码；
- `from ... import ...`：检查SDK模块能否导入。

**预期输出**

```text
SDK Ready
```

若出现`ModuleNotFoundError`，先检查安装脚本是否成功、Python解释器是否一致，再查阅仓库`docs/FAQ_常见问题.md`。

## 7. 首次运行：先做dry-run

### 7.1 步骤10：加载LeTools环境

每次打开新终端，按顺序执行：

```bash
cd ~/letools_opensource
source /opt/ros/noetic/setup.bash
source infrastructure/ros_packages/devel/setup.bash
export PYTHONPATH=$(pwd):$PYTHONPATH
```

**说明**

- 第1条进入项目；
- 第2条加载系统ROS；
- 第3条加载LeTools编译结果；
- 第4条把项目根目录加入Python模块搜索路径。

**调整建议**

把第1条路径替换成你的实际仓库路径。

### 7.2 步骤11：运行行为树dry-run

```bash
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/refactored_sdk_atomic_v1 \
  --dry-run --tick-once
```

**参数说明**

| 参数 | 含义 |
|---|---|
| `--scenario` | 指定场景目录 |
| `--dry-run` | 不初始化ROS、不连接硬件 |
| `--tick-once` | 构建行为树后只执行一次tick |

**预期输出**

- 主树能够加载；
- 日志显示根节点名称；
- 没有import、JSON或board字段错误；
- 进程退出码为0表示根节点SUCCESS。

**注意**

dry-run通过只说明代码结构和初始流程可加载，不代表仿真或真机运动已经验证。

## 8. 进入MuJoCo仿真

MuJoCo、Docker、仿真启动命令、最低硬件配置记录表、能力边界和常见问题均维护在独立文档：

> [打开《LeTools 仿真开发指南》](./仿真开发指南.md)

完成仿真指南中的步骤后，再回到本教程继续真机或业务示例学习。

## 9. 连接真机

> 真机操作有物理风险。首次运行必须保证机器人周边空间充足、急停可触达，并建议先完成dry-run和MuJoCo验证。

### 9.1 步骤12：加载ROS和LeTools环境

```bash
cd ~/letools_opensource
source /opt/ros/noetic/setup.bash
source infrastructure/ros_packages/devel/setup.bash
export PYTHONPATH=$(pwd):$PYTHONPATH
```

### 9.2 步骤13：确认ROS连接

**前置条件**

- 下位机控制器已经启动；
- 上位机与机器人网络连通；
- ROS环境变量由现场网络方案提供。

```bash
rosnode list
rostopic list
rosservice list
```

**预期输出**

三个命令应能返回节点、话题和服务列表，而不是连接ROS master失败。

由于实际机器人网络地址和启动方式没有在当前仓库中统一确定，本教程不填写IP和ROS master地址。

## 10. 运行已有示例

### 10.1 步骤14：头部控制示例

仓库README给出的示例入口为：

```bash
python3 apps/test_kuavo_5w_sdk_adapter/sdk/01_head/test_head_control.py
```

**前置条件**

- ROS和LeTools环境已加载；
- MuJoCo控制器或真机控制器已经运行；
- SDK验证已通过。

**预期结果**

脚本正常启动，控制后端能够收到头部控制请求。具体运动效果取决于当前示例参数、仿真或真机状态。

**安全建议**

真机首次运行前检查机器人周边空间和急停状态，不要直接使用未经确认的大幅度参数。

### 10.2 示例阅读顺序

仓库推荐：

1. `apps/test_kuavo_5w_sdk_adapter/README.md`；
2. `apps/test_kuavo_5w_sdk_adapter/sdk/01_head/`；
3. `core/interfaces/i_hardware.py`；
4. `adapters/hardware/leju_wheeled/hardware.py`；
5. `skills/base/skill_base.py`和`skills/atomic/refactored_sdk/`；
6. `orchestration/nodes/`；
7. `orchestration/scenarios/refactored_sdk_atomic_v1/readme.md`；
8. `apps/test_upper_init/run_behavior_tree_json.py`。

## 11. 修改自己的行为树

### 11.1 场景目录

```text
your_scenario/
├── py_tree.json
├── py_tree_child.json
└── board.json
```

### 11.2 修改后的验证顺序

```bash
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/your_scenario \
  --dry-run --tick-once
```

确认dry-run通过后，再按照[《LeTools 仿真开发指南》](./仿真开发指南.md)进行MuJoCo验证，最后安排真机验证。

## 12. 常见问题快速定位

| 现象 | 先检查 |
|---|---|
| `cd`提示目录不存在 | 使用`pwd`和`ls`确认实际仓库路径 |
| `catkin build`失败 | 查看失败包和`infrastructure/ros_packages/logs/` |
| `ModuleNotFoundError` | SDK安装、Python解释器、`PYTHONPATH` |
| dry-run提示主树不存在 | `--scenario`路径及当前工作目录 |
| 子树`KeyError` | 主树引用名与`py_tree_child.json`的key是否一致 |
| board字段缺失 | `board.json`与节点输入是否同步 |
| dry-run通过但仿真失败 | MuJoCo控制器、ROS服务、SDK版本 |
| ROS master连接失败 | 现场ROS网络变量和下位机状态 |

详细问题处理见仓库`docs/FAQ_常见问题.md`。

## 13. 完成检查表

- [ ] 能说明apps、orchestration、skills和adapters的关系；
- [ ] ROS Noetic环境能够加载；
- [ ] `catkin build`完成；
- [ ] SDK导入输出`SDK Ready`；
- [ ] dry-run能够加载指定场景；
- [ ] 已阅读独立仿真指南；
- [ ] MuJoCo适用场景完成仿真验证；
- [ ] 真机测试前已经确认急停和现场安全；
- [ ] 知道到哪里查看FAQ和日志。

## 14. 相关文档

- [《LeTools 仿真开发指南》](./仿真开发指南.md)
- 仓库根目录：`README.md`
- 仓库启动指南：`docs/user_guides.md`
- 仓库FAQ：`docs/FAQ_常见问题.md`
- 行为树启动器：`apps/test_upper_init/readme.md`




## 15. 关键命令失败后的下一步

| 命令 | 成功判断 | 失败后的第一步 |
|---|---|---|
| `catkin build` | Summary中Failed为None | 定位第一个`Failed <<<`，不要先处理后续Abandoned |
| `./scripts/install_sdk.sh` | 输出安装完成且SDK可导入 | 检查消息包、Python解释器和`sdk_version.env` |
| `--dry-run --tick-once` | 退出码0 | 检查节点类名大小写、子树键和board文件 |
| `roslaunch ...mujoco...` | 控制器、MuJoCo节点持续存活 | 检查第一个REQUIRED进程错误及动态库`not found` |

完整环境、Docker、OpenVINO和catkin路径问题统一查阅[仿真开发指南](./仿真开发指南.md)；常见错误代码见[FAQ](./FAQ_常见问题.md)。
