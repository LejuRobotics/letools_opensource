# orchestration 层真机测试脚本（test_kuavo_5w_orchestration）

镜像源码层 `orchestration/nodes/`，测试粒度为「编排层」——构造 `BaseAction` 子类节点，
走 PyTrees 节点生命周期（`initialise()` → 循环 `update()` → `SUCCESS`/`FAILURE`），
节点内部通过 `get_shared_hardware()` 全局单例取硬件并驱动技能，体现分层架构中
**编排层**与技能层/适配器层的区别。

## 分层位置

```
drivers → core → adapters → skills → orchestration → apps
                                            ↑ 此层        ↑ 此层
```

- **skills 层**（`test_kuavo_5w_skills`）：直接构造 `XxxSkill(hardware=...)` 走技能生命周期
- **本层（编排层）**：构造节点 `XxxMove(name, label, ns, params)`，硬件由单例注入，验证节点包装正确性
- **T4（适配器层）**：直接调 `hardware.xxx()`，验证适配器方法本身

## 目录结构

```
apps/test_kuavo_5w_orchestration/
  __init__.py
  _scaffold.py                            # run_node() + set_hardware_config() + node_setup/teardown
  README.md                                # 本文件
  test_arm_reset_sdk_move.py              # ArmResetSdkMove（硬件节点，arm）
  test_head_control_sdk_move.py           # HeadControlSdkMove（硬件节点，low）
  test_base_pose_local_move.py            # BasePoseLocalMove（硬件节点，low）
  test_leg_joint_sdk_move.py              # LegJointSdkMove（硬件节点，low）
  test_wait_seconds.py                     # WaitSeconds（纯编排节点，无硬件）
  # —— 末端定时指令类（whitelist=['timed']）——
  test_arm_ee_single_timed_move.py        # 单航点定时移动（left-world/right-local）
  test_arm_ee_burst_timed_move.py         # 多航点连续定时移动
  test_arm_ee_offline_traj_move.py        # 离线轨迹（traj+times）
  test_arm_ee_timed_cmd_move.py           # 定时指令（local，左右臂单航点）
  # —— 末端 SDK 轨迹类（whitelist=['arm']）——
  test_arm_ee_traj_local_sdk_move.py      # 末端局部系轨迹
  test_arm_ee_traj_world_sdk_move.py      # 末端世界系轨迹
  test_arm_joint_traj_sdk_move.py         # 14 关节角轨迹（内联 joint_traj）
  # —— 末端位姿/前置设置类（jibot）——
  test_arm_ee_pose_jibot_move.py          # 末端位姿（whitelist=['low','arm']）
  test_arm_ee_setup_jibot_move.py         # 末端独立控制前置设置（whitelist=['timed']）
  # —— 底盘/躯干类（whitelist=[]）——
  test_base_move_relative_jibot_move.py   # 底盘相对移动（前进/旋转）
  test_base_move_to_target_jibot_move.py  # 底盘移动到世界系目标
  test_chassis_stop_move.py               # 底盘停止导航
  test_check_arrived_jibot_move.py        # 导航到达检查（非阻塞）
  test_torso_reset_sdk_move.py            # 躯干复位
  # —— 真空控制类（whitelist=[]）——
  test_vacuum_485_move.py                  # 气泵485继电器（吸/断电）
  test_vacuum_control_move.py             # 气泵控制（吸/放含破真空）
  # —— 纯编排类（无硬件，dry-run 可跑）——
  test_wait_for_enter.py                   # 按Enter继续（dry-run 可跑）
```

## 运行

```bash
cd /path/to/LeTools            # 替换为本机 LeTools 根目录

source infrastructure/ros_packages/devel/setup.bash


# 单独跑某个节点
python3 apps/test_kuavo_5w_orchestration/test_arm_reset_sdk_move.py
python3 apps/test_kuavo_5w_orchestration/test_head_control_sdk_move.py

# 纯编排节点（无硬件依赖，可不连机器人）
python3 apps/test_kuavo_5w_orchestration/test_wait_seconds.py
```

退出码：0=成功，1=失败（对齐 T4 与 `orchestration/main.py` 约定）。

## 新增节点测试

1. 复制任一 `test_xxx_move.py`（硬件节点照 `test_arm_reset_sdk_move.py`，纯编排照 `test_wait_seconds.py`）
2. 改 import 为新节点的类
3. 在 `test_xxx()` 内构造节点、调 `run_node(node)`
4. `main()` 里按子系统选 `set_hardware_config(whitelist=[...])`（`arm` / `low`）与 `node_setup(need_arm=...)`

## _scaffold.py 提供的工具

| 函数 | 作用 |
|------|------|
| `set_hardware_config(whitelist, skip_*...)` | 在节点取硬件前覆盖共享硬件配置（对齐 T4） |
| `node_setup(need_arm, need_torso_reset)` | 前置：躯干复位、手臂复位、切外部控制（作用于共享单例） |
| `node_teardown(need_arm)` | 后置：手臂/躯干复位 |
| `run_node(node, tick_interval, max_ticks)` | 跑节点生命周期 initialise→update→SUCCESS/FAILURE |

前置/后置逻辑与 T4 `factory_setup`/`factory_teardown` 一致（MPC 模式、躯干复位等安全管理），
但本层独立实现，作用于 `get_shared_hardware()` 单例（节点也是从该单例取硬件），不跨目录 import T4。

## 与 skills 层测试的区别

| 维度 | skills 层 | orchestration 层（本层） |
|------|-----------|--------------------------|
| 被测对象 | `XxxSkill`（SkillBase） | `XxxMove`（BaseAction 节点） |
| 硬件注入 | 构造时传 `hardware=...` | 节点内部 `get_shared_hardware()` 单例 |
| 生命周期 | `initialize`→`execute`→`is_finished` | `initialise`→`update`→`Status` |
| 配置方式 | `build_hardware(whitelist=...)` | `set_hardware_config(whitelist=...)` 覆盖单例 |
