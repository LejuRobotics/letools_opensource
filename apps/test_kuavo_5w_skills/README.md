# skills 层真机测试脚本（test_kuavo_5w_skills）

镜像源码层 `skills/`，测试粒度为「技能层」——调用 `SkillBase` 子类的生命周期
（`initialize(params)` → 循环 `execute()` + `is_finished()`），而非直接调适配器方法，
体现分层架构中**技能层**与**适配器层**（T4 `test_kuavo_5w_sdk_adapter`）的区别。

## 分层位置

```
drivers → core → adapters → skills → orchestration → apps
                                    ↑ 此层            ↑ 此层
```

- **T4（适配器层）**：直接调 `hardware.xxx()`，验证适配器方法本身
- **本层（技能层）**：构造 `XxxSkill(hardware=...)`，走技能生命周期，验证技能封装正确性

## 目录结构

```
apps/test_kuavo_5w_skills/
  __init__.py
  _scaffold.py                       # run_skill() + build_hardware() + skill_setup/teardown
  README.md                           # 本文件
  test_arm_reset.py                   # ArmResetSdkSkill（硬件技能，arm）
  test_head_control.py               # HeadControlSdkSkill（硬件技能，low）
  test_base_pose_local.py            # BasePoseLocalSkill（硬件技能，low）
  test_leg_joint.py                   # LegJointSdkSkill（硬件技能，low）
  test_leg_joint_timed.py             # LegJointTimedSkill（硬件技能，timed，服务端 Ruckig 规划）
  test_wait_seconds.py               # WaitSecondsSkill（纯编排技能，无硬件）
  # —— 末端定时指令类（whitelist=['timed']，need_arm=True）——
  test_arm_ee_single_timed.py        # 单航点定时移动（local/world）
  test_arm_ee_burst_timed.py         # 多航点连续定时移动
  test_arm_ee_offline_traj.py         # 离线轨迹（traj+times）
  test_arm_ee_timed_cmd.py           # 定时指令（local/world）
  # —— 末端 SDK 轨迹类（whitelist=['arm']，need_arm=True）——
  test_arm_ee_traj_local_sdk.py      # 末端局部系轨迹
  test_arm_ee_traj_world_sdk.py      # 末端世界系轨迹
  test_arm_joint_traj_sdk.py         # 14 关节角轨迹
  # —— 末端位姿/前置设置类（jibot）——
  test_arm_ee_pose_jibot.py           # 末端位姿（whitelist=['low','arm']，need_arm=True）
  test_arm_ee_setup_jibot.py         # 末端独立控制前置设置（whitelist=['timed']，need_arm=True）
  # —— 底盘/躯干类（whitelist=[]，need_arm=False）——
  test_base_move_relative_jibot.py   # 底盘相对移动（前进/旋转）
  test_base_move_to_target_jibot.py  # 底盘移动到世界系目标
  test_chassis_stop.py               # 底盘停止导航
  test_check_arrived_jibot.py        # 导航到达检查
  test_torso_reset_sdk.py            # 躯干复位
  # —— 真空控制类（whitelist=[]，need_arm=False）——
  test_vacuum_485.py                  # 气泵485继电器（吹/吸/断电）
  test_vacuum_control.py             # 气泵控制（吸/放含破真空）
  # —— 纯编排类（无硬件）——
  test_wait_for_enter.py             # 按Enter继续（dry-run 可跑）
```

## 运行

```bash
cd /path/to/LeTools            # 替换为本机 LeTools 根目录

source infrastructure/ros_packages/devel/setup.bash


# 单独跑某个技能
python3 apps/test_kuavo_5w_skills/test_arm_reset.py
python3 apps/test_kuavo_5w_skills/test_head_control.py

# 纯编排技能（无硬件依赖，可不连机器人）
python3 apps/test_kuavo_5w_skills/test_wait_seconds.py
# 或显式 dry-run：
STUDIO_DRY_RUN=1 python3 apps/test_kuavo_5w_skills/test_wait_seconds.py
```

退出码：0=成功，1=失败（对齐 T4 与 `orchestration/main.py` 约定）。

## 新增技能测试

1. 复制任一 `test_xxx.py`（硬件技能照 `test_arm_reset.py`，纯编排照 `test_wait_seconds.py`）
2. 改 import 为新技能的 `XxxSkill` / `XxxParams`
3. 在 `test_xxx()` 内构造技能、调 `run_skill(skill, params)`
4. `main()` 里按子系统选 `build_hardware(whitelist=[...])`（`arm` / `low`）与 `skill_setup(need_arm=...)`

## _scaffold.py 提供的工具

| 函数 | 作用 |
|------|------|
| `build_hardware(whitelist, skip_*...)` | 构建 IHardware（config 对齐 T4 真机约定） |
| `skill_setup(hardware, need_arm, need_torso_reset)` | 前置：躯干复位、手臂复位、切外部控制 |
| `skill_teardown(hardware, need_arm)` | 后置：手臂/躯干复位 |
| `run_skill(skill, params, tick_interval)` | 跑技能生命周期 initialize→execute→is_finished |

前置/后置逻辑与 T4 `factory_setup`/`factory_teardown` 一致（MPC 模式、躯干复位等安全管理），
但本层独立实现，不跨目录 import T4 的 `_scaffold`，避免 skills 层耦合适配器层测试。
