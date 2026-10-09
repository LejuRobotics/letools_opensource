# jibot_nav_v1_sim — 仿真可运行版导航场景

`jibot_nav_v1` 的**仿真版本**。把依赖 JiBot 调度服务的节点替换为基于 ROS **话题**的等价实现，
使这棵行为树可以直接在 MuJoCo 仿真环境下跑通，不需要 JiBot 底盘机。

- 原始场景：[`../jibot_nav_v1/`](../jibot_nav_v1/)（真机 + JiBot 底盘，保持不变）
- 本场景：`../jibot_nav_v1_sim/`（仿真）

---

## 一、总览：3 个终端，各干什么

下面 3 个终端**都在同一个容器里**，顺序不能颠倒（终端 2 要求终端 1 先起，终端 3 要求终端 2 先起）。

| 终端 | 干什么 | 关键命令 | 跑完能关吗 |
|---|---|---|---|
| 终端 1 | MuJoCo 仿真（下位机 `kuavo-ros-opensource`） | `roslaunch humanoid_controllers load_kuavo_mujoco_sim_wheel.launch` | ❌ 一直挂着 |
| 终端 2 | TF 转发服务 `kuavo_tf2_web_republisher` | `./start_tf_republisher.sh` | ❌ 一直挂着 |
| 终端 3 | 跑行为树（本仓库 `LeTools`） | `python3 apps/test_upper_init/run_behavior_tree_json.py --scenario ...` | ✅ 跑完自己退出 |

---

## 二、第 0 步：从宿主机进容器（3 个终端都要做）

容器是 `run_with_gpu.sh` 创建的**具名容器**（不是 `--rm`），关终端不会销毁，可以反复 `exec` 进去。

```bash
# 【宿主机】查容器名，形如 kuavo_container_GPU_7c70d3a3
docker ps --format '{{.Names}}'
```

然后在宿主机上开 **3 个终端**，每个都执行下面这条（把容器名换成你查到的）：

```bash
# 【宿主机】进入容器，得到容器内的 zsh
docker exec -it kuavo_container_GPU_7c70d3a3 zsh
```

> - 容器**默认 shell 是 zsh**，所以本文所有 setup 都用 **`.zsh` 结尾**的文件（`setup.zsh`）。
>   如果你手动用 bash 进容器，把 `.zsh` 换成 `.bash` 即可。
> - 镜像里的 `/root/.zshrc` 已经自动执行了 `source /opt/ros/noetic/setup.zsh`，
>   所以**每个新开的 zsh 里 `rospy` / `rostopic` / `roslaunch` 直接可用**，不用再手动 source ROS。
> - 但每个工作空间**自己的** `setup.zsh` 仍然必须 source，见下面各终端。
> - 你执行 `./docker/run_with_gpu.sh` 时已经自带一个 zsh，那个就算终端 1，再开两个终端 exec 进去即可。

---

## 三、终端 1：起 MuJoCo 仿真（必须最先起）

```bash
# 【终端 1 · 容器内】
cd /root/kuavo_ws
source /opt/ros/noetic/setup.zsh     # 容器 .zshrc 已自动执行，这里写一遍是保险
source devel/setup.zsh               # 下位机工作空间环境（catkin build 的产物）
roslaunch humanoid_controllers load_kuavo_mujoco_sim_wheel.launch
```

- **`devel/setup.zsh` 不是 `installed/setup.zsh`**：`installed/setup.zsh` 是编译前加载预装依赖用的，
  跑仿真要 source 编译产物 `devel/setup.zsh`（官方 `readme.md` 也是这么写的）。
- 第一次在容器里跑，先编译一次：`cd /root/kuavo_ws && source installed/setup.zsh && catkin build humanoid_controllers`
- **ROS master 由这条 `roslaunch` 自动拉起**，不需要另外开 `roscore`。
- 起成功后会出现 MuJoCo 窗口（以及 rviz 窗口），**这个终端不要关**。

**成功标志**：终端不再刷 error、MuJoCo 窗口里出现机器人。
另开一个容器终端执行 `rostopic list | grep cmd_pose_world`，能列出 `/cmd_pose_world` 就说明下位机底盘通道就绪。

---

## 四、终端 2：起 TF 转发服务（必须在终端 1 之后）

```bash
# 【终端 2 · 容器内】
cd /root/LeTools/infrastructure/ros_packages
source /opt/ros/noetic/setup.zsh     # 脚本会检查 $ROS_DISTRO，先 source 保险
source devel/setup.zsh               # LeTools 的 ros_packages 工作空间
./start_tf_republisher.sh
```

- 看到 `✅ 服务启动成功！` 才算成功，**这个终端保持不关**。
- 若提示 `未找到编译后的可执行文件`，先编译一次：`./build.sh`（或 `catkin build kuavo_tf2_web_republisher`）。
- 官方 `docs/user_guides.md` 2.2 节的要求是：**这个服务要在仿真开启之后再启动**，所以顺序是 终端 1 → 终端 2。
- 为什么必须起：SDK 初始化时会 `wait_for_service('/republish_tfs')`，拿不到就抛 `SDK 初始化失败`，
  后续每个 SDK 动作都报 `SDK 未初始化，请先调用 initialize()`——头部动作会**假成功**（打 ERROR 但返回 True），
  腿部动作返回 False 导致行为树 FAILURE。详见「十一、已知差异与风险」第 5 条。

**成功标志**：终端打印 `✅ 服务启动成功！`，且 `rosservice list | grep republish_tfs` 有输出。

---

## 五、终端 3：跑行为树（本仓库）

```bash
# 【终端 3 · 容器内】
cd /root/LeTools
export PYTHONPATH=/root/LeTools/infrastructure/ros_packages/src/kuavo_common/python:$PYTHONPATH

python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/jibot_nav_v1_sim
```

- `PYTHONPATH` 那一行**每开一个新终端都要执行一次**（原因见第六节第 2 条，建议直接写进 `.zshrc`）。
- 这条命令**不需要**额外 source 任何 setup：入口脚本 `core/common/ros_environment.py` 会自己把
  `infrastructure/ros_packages/devel/lib/python3/dist-packages` 加进 `sys.path`，
  `rospy` 由容器 `.zshrc` 提供。手动补一句 `source devel/setup.zsh` 也无害。
- 跑起来后机器人会：**后退 0.1 m → 依次走 6 个点位（每个点位做低头 / 抬腿 / 复位）→ 等回车结束**。

**离线自检**（不连 ROS，只验证行为树能否加载；不需要终端 1 / 2，CI 可跑）：

```bash
# 【终端 3 · 容器内】
cd /root/LeTools
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/jibot_nav_v1_sim --dry-run --tick-once
```

---

## 六、一次性准备

前两项是**环境问题**、不是本场景特有的，官方文档没写全，但**漏掉任意一项场景都会中途 FAILURE**。

### 1. 装 `py_trees`（任意终端）

```bash
# 【容器内 · 任意终端】
pip3 install "py_trees==2.2.3"
```

> 版本依据：`.gitlab-ci.yml` 装的是不锁版本的 `py_trees`；
> `module_internal/bin_planner/requirements-jetson.txt` 锁的是 `2.2.3`。
> ⚠️ `docs/beginner_tutorial.md` 里写的是 `pip3 install ... pytrees`，
> `pytrees` 是另一个不相干的包，**正确包名是 `py_trees`**（下划线）。
> 漏装的表现：`ModuleNotFoundError: No module named 'py_trees'`。

### 2. 让 SDK 找得到 `robot_version`（终端 3，或写进 `.zshrc`）

```bash
# 【终端 3 · 容器内】
export PYTHONPATH=/root/LeTools/infrastructure/ros_packages/src/kuavo_common/python:$PYTHONPATH

# 验证：应打印出 robot_version.py 的路径
python3 -c "import robot_version; print(robot_version.__file__)"
```

想省事就写进容器的 `.zshrc`（容器具名可复用，能留住，之后每个新终端自动生效）：

```bash
echo 'export PYTHONPATH=/root/LeTools/infrastructure/ros_packages/src/kuavo_common/python:$PYTHONPATH' >> /root/.zshrc
```

> **为什么必须手动加**：SDK 的 `robot_blockly.py` 通过
> `rospkg.RosPack().get_path('kuavo_common') + '/python'` 和一层相对路径回退去找 `robot_version`，
> 但 `kuavo_common` 的 `CMakeLists.txt` 既没有 `catkin_python_setup()`，也没有 install `python/` 目录，
> `installed/share/kuavo_common/` 和 `devel/share/kuavo_common/` 下只有 `cmake/` 和 `package.xml`。
> 两条回退路径都指向不存在的目录，结果是 `ModuleNotFoundError: No module named 'robot_version'`，
> 进而让整个 SDK 导入链断掉、`LowLevelSDKManager` 无法初始化。
> CI 里是靠 `.gitlab-ci.yml` 的 `KUAVO_COMMON_PY` 绕过的，用户文档里没写。

### 3. 确认 `ROBOT_VERSION`

```bash
# 【终端 1 · 容器内】起仿真前
echo $ROBOT_VERSION    # 应与你的机型一致，官方文档里常用 62
```

> 镜像的 `/root/.zshrc` 里有一行 `export ROBOT_VERSION=42`，**会覆盖** `run_with_gpu.sh` 传进来的
> `-e ROBOT_VERSION=62`（`.zshrc` 在交互 shell 启动时最后执行）。值不对就手动 `export ROBOT_VERSION=62`，
> 或直接改 `.zshrc`。改完要重启终端 1 的仿真才生效。

---

## 七、检查点速查

| 终端 | 成功标志 | 常见报错 → 原因 |
|---|---|---|
| 1 仿真 | MuJoCo 窗口出现机器人；`rostopic list` 有 `/cmd_pose_world` | 找不到 `humanoid_controllers` → 忘了 `source devel/setup.zsh` 或没 `catkin build` |
| 2 TF 服务 | `✅ 服务启动成功！`；`rosservice list` 有 `/republish_tfs` | `ROS 环境未配置` → 没 source `/opt/ros/noetic/setup.zsh`；`未找到编译后的可执行文件` → 先 `./build.sh` |
| 3 行为树 | 机器人按 6 个点位走动，日志无 ERROR | `No module named 'py_trees'` → 第六节第 1 条；`No module named 'robot_version'` → 第六节第 2 条；`SDK 初始化失败: ... /republish_tfs` → 终端 2 没起 |

---

## 八、节点替换对照

| 原场景节点 | 本场景节点 | 通信通道 | 说明 |
|---|---|---|---|
| `BaseMoveToTargetJibotMove` ×6 | `WheelNavMove` | 话题 `/cmd_pose_world` | map 绝对目标点 → 世界坐标系绝对位姿 |
| `CheckArrivedJibotMove` ×6 | 已删除 | — | `WheelNavMove` 在后台线程阻塞到移动返回，无需单独判到达 |
| `BaseMoveRelativeJibotMove` ×1 | `BasePoseLocalMove` | 话题 `/cmd_pose` | 本体坐标系相对位姿，语义一致 |
| `HeadControlSdkMove` / `LegJointSdkMove` / `WaitSeconds` / `WaitForEnter` | 不变 | — | 与底盘无关，仿真可直接用 |

---

## 九、参数

参数目前内联在 `py_tree_child.json` 的 `params` 里（与原场景 `jibot_nav_v1` 一致，`board.json` 为空）。
需要调参时直接改对应节点的 `value`：

- `nav_point_N`（`WheelNavMove`）：`x` / `y`（米，世界坐标系）、`yaw_deg`（度）、`timeout_sec`
- `backward_0_2m`（`BasePoseLocalMove`）：`x` / `y`（米，本体坐标系）、`yaw`（度）

`hardware_config.json` 说明：

```json
{
  "skip_force_publishers": true,
  "skip_camera": true,
  "skip_end_effector": true,
  "sdk_managers_whitelist": ["low"]
}
```

- `sdk_managers_whitelist: ["low"]`：本场景只用头部 / 腿部，二者都走 `LowLevelSDKManager`；
  不初始化 `timed` / `arm` 可以加快启动。
- 相机、末端执行器、力控发布器在本场景用不到，全部跳过。

---

## 十、已知差异与风险

1. **坐标系不同**：原场景的 `x/y` 是 JiBot 地图（`map`）坐标；仿真里 `/cmd_pose_world` 是世界坐标系，
   原点约为机器人启动位姿。数值沿用了原场景，若机器人走向不符合预期，改 `nav_point_N` 的 `x/y` 即可。
2. **`timeout_sec` 目前不生效**：`orchestration/nodes/wheel_nav_move.py` 只读取 `x` / `y` / `yaw_deg`，
   `timeout_sec` 是为与 `grasp_ring_pick_place` 场景写法保持一致而保留的占位参数。
3. **`/lb_leg_control_srv` 在仿真里不存在**：下位机 `motion_capture_ik/launch/ik_node.launch` 里
   腿部 IK 服务节点是被注释掉的，SDK 初始化时会等它 5 秒超时。实测**不影响**本场景的抬腿动作
   （`LegJointSdkMove` 走的是 `/lb_leg_traj` 话题），但属于下位机仿真侧的一个缺口。
4. **`/lb_cmd_pose_reach_time` 的数值可疑**：实测 `/cmd_pose`（0.1 m）和 `/cmd_pose_world`（0.844 m）
   报回的预计到达时间完全相同（`1.0285125970840454` s），距离差 8 倍却同值，疑为固定默认值。
   由于 `send_world_position`（`adapters/hardware/leju_wheeled/mixins/base_control_mixin.py:158`）
   是"按预计时间 + 0.5 s 盲等"，**不校验是否真的到位**，
   判断机器人是否走到位要靠肉眼看 MuJoCo 窗口，不能只看日志的"完成"。
5. **依赖 SDK 可用**：`HeadControlSdkMove` / `LegJointSdkMove` 需要 `kuavo_humanoid_sdk` 能正常导入并初始化，
   参见「六、一次性准备」第 2、3 条与「四、终端 2」。SDK 初始化失败时头部动作会**假成功**，
   务必区分日志里有没有 `SDK 未初始化，请先调用 initialize()` 这行 ERROR。
6. 本场景只覆盖"导航 + 躯干/头部动作"，不涉及视觉、抓取；视觉相关场景的仿真化不在本次范围内。
