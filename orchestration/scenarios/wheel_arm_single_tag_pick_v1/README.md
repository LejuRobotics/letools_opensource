# wheel_arm_single_tag_pick_v1

本场景是源脚本 `case_wheel_test_arm.py` 迁移至 LeTools 框架后的标准行为树场景。

**核心业务边界**：聚焦于单 Tag 识别/注入、左右臂抓取关键点生成、双臂 Joint IK 规划求解、双臂 14 关节同步轨迹执行以及执行成功后的手臂复位。不包含放置（Place）、夹爪释放或独立抬升动作。

---

## 1. 架构与涉及文件清单

迁移采用分层解耦架构，严格遵循零侵入原有公共代码与适配器的原则。

### 1.1 方案与文档 (Docs)
- `LeTools/orchestration/scenarios/wheel_arm_single_tag_pick_v1/README.md`: 本场景使用与维护手册。

### 1.2 场景编排与配置 (Scenario Config)
- `py_tree.json`: 顶层行为树结构定义（`Sequence` 顺序执行 Tag 注入 -> 轨迹规划 -> 轨迹执行 -> 复位）。
- `py_tree_child.json`: 子树配置定义。
- `board.json`: 初始黑板参数契约（Tag ID、箱体尺寸、倾角、控制模式等）。
- `hardware_config.json`: 硬件适配器配置（使用 `leju_wheeled` 轮臂适配器）。

### 1.3 原子技能层 (Atomic Skills - `skills/atomic/refactored_sdk/`)
- `single_tag_joint_pick_plan.py` (`SingleTagJointPickPlanSkill`): 单 Tag 双臂抓取关节轨迹规划原子技能。负责几何关键点计算与逆运动学 (IK) 求解，输出左右臂独立轨迹。
- `bimanual_joint_trajectory.py` (`BimanualJointTrajectorySkill`): 双臂 14 关节同步轨迹执行原子技能。负责双臂轨迹校验、逐帧合并为 $N \times 14$ 格式并异步下发底层硬件适配器。

### 1.4 行为树执行节点 (BT Nodes - `orchestration/nodes/`)
- `node_inject_single_tag.py` (`NodeInjectSingleTag`): 负责 Tag 信息注入（支持虚拟 Tag 与感知 Tag），维护 `latest_tag_<id>` 及递增版本号 `latest_tag_<id>_version`。
- `node_source_tag_to_arm_goal_single_tag.py` (`NodeSourceTagToArmGoalSingleTag`): 行为树轻量代理节点，读取黑板 Tag 信息并调用 `SingleTagJointPickPlanSkill`，将规划所得轨迹写回黑板。
- `node_wheel_arm_single_tag.py` (`NodeWheelArmSingleTag`): 行为树轻量代理节点，读取黑板轨迹并调用 `BimanualJointTrajectorySkill` 下发硬件并监控执行状态。
- `arm_reset_sdk_move.py` (`ArmResetSdkMove`): 动作完成后的手臂标准复位（复用现有通用节点）。

### 1.5 场景算法与工具模块 (Utils - `utils/`)
- `utils/trajectory.py`: 双臂关节轨迹形状校验（7 维有效性检测）及左右臂 7+7=14 逐帧合并。
- `utils/keypoints_single_tag.py`: 单 Tag 抓取几何关键点计算（BASE 坐标系）。
- `utils/ik_result.py`: 逆运动学求解结果封装与校验。
- `utils/source_joint_planner.py`: 源脚本兼容的关节空间运动规划器。
- `utils/async_execution.py`: 异步硬件调用与状态管理包装。

### 1.6 自动化测试集 (Tests - `tests/`)
- `tests/conftest.py`: 测试环境准备与共享 Fixtures。
- `tests/test_config.py`: 场景 JSON 配置文件合法性与黑板变量映射校验。
- `tests/test_trajectory.py`: 轨迹合并与校验函数单测。
- `tests/test_node_inject_single_tag.py`: Tag 注入节点逻辑与黑板状态测试。
- `tests/test_single_tag_joint_pick_plan_skill.py`: 规划技能逻辑与异常分支单测。
- `tests/test_bimanual_joint_trajectory_skill.py`: 双臂轨迹下发技能单测。
- `tests/test_node_source_tag_to_arm_goal_single_tag.py`: Tag 目标生成节点与代理调用单测。
- `tests/test_node_wheel_arm_single_tag.py`: 轮臂轨迹下发节点状态机流转测试。
- `tests/test_source_joint_semantics.py` & `tests/test_source_joint_planner_extended.py`: 关节规划语义与扩展边界测试。
- `tests/test_async_execution.py`: 异步轨迹执行与取消机制测试。

### 1.7 运行入口与底层依赖 (Entry & Platform)
- `LeTools/apps/test_upper_init/run_behavior_tree_json.py`: 统一的行为树运行入口脚本。
- `LeTools/adapters/hardware/leju_wheeled/`: 轮臂硬件适配器及 SDK Mixin。
- `kuavo-ros-opensource`: 提供 MuJoCo 仿真物理引擎及全身运动控制器后端。

---

## 2. 启动与验证

### 2.1 结构干跑验证 (Dry-Run)

无需启动 ROS 节点或 MuJoCo 仿真，用于快速校验 JSON 结构、节点加载与黑板传参：

```bash
cd LeTools
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/wheel_arm_single_tag_pick_v1 \
  --dry-run --tick-once
```

> **当前验证状态（2026-08-27）**：干跑通过，根状态返回 `Status.SUCCESS`。此模式由 `STUDIO_DRY_RUN=1` 短路运动节点，仅证明 import 链路、JSON 加载与节点类名映射无误，**不**触发 IK 求解与轨迹下发。

### 2.2 MuJoCo 仿真运行

在 MuJoCo 仿真环境中完整运行，需要打开两个终端：

**终端 1：启动轮臂 MuJoCo 控制器**
```bash
roslaunch humanoid_controllers load_kuavo_mujoco_sim_wheel.launch
```


**终端 2：启动 LeTools 行为树**
```bash
cd LeTools
source infrastructure/ros_packages/devel/setup.bash

python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/wheel_arm_single_tag_pick_v1
```

> **当前验证状态（2026-08-27）**：端到端 MuJoCo 仿真完整通过，行为树根状态返回 `Status.SUCCESS`。本次真跑覆盖完整闭环：`_arm_sdk_manager` 初始化成功 → `SingleTagJointPickPlanSkill` 规划出 397 帧双臂轨迹 → `BimanualJointTrajectorySkill` 合并并下发 N×14 轨迹执行成功 → `ArmResetSdkMove` 手臂复位成功。此运行依赖 `kuavo_humanoid_sdk` 实际调用 IK 求解（`SourceIkCompat.arm_ik`）与轨迹下发（`move_joint_traj_auto`），证明 Skills 层委托与 `source_sdk_compat` 兼容层在真跑链路下工作正常。

---

## 3. 数据契约与安全机制

1. **轨迹维度格式**：
   - 黑板内 `left_arm_joint_traj` 和 `right_arm_joint_traj` 均为 `list[list[float]]`，每帧恰好 7 个有效弧度值。
   - 下发前通过 `combine_bimanual_joint_trajectories` 组合成 N×14 格式：前 7 维为左臂，后 7 维为右臂。
2. **Tag 状态门禁**：
   - 目标节点监听 `latest_tag_<id>_version`，版本未更新时维持 `RUNNING` 挂起状态。
3. **IK 异常阻断**：
   - 左右任一臂 IK 求解无解或失败时，直接阻断并不生成/不发送局部残缺轨迹。
4. **异步安全下发**：
   - 硬件执行采用非阻塞/异步轮询机制，支持执行中断与安全退出。
5. **硬件配置说明**：
   - `hardware_config.json` 指定 `leju_wheeled`。MuJoCo 不是 LeTools 内部的单独 Python adapter，而是由底层 ROS 控制器承载的物理仿真后端。
6. **SDK 依赖（真跑前提）**：
   - 真跑（非 dry-run）依赖 `kuavo_humanoid_sdk` 已安装。`IK 求解`（`SourceIkCompat.arm_ik` → `robot_sdk.arm.arm_ik`）与`轨迹下发`（`SourceJointTrajectoryCompat.move_joint_traj_auto` → `_arm_sdk_manager.move_joint_traj_auto`）均经源 SDK 兼容层调用该包；缺失时生命周期阶段会跳过 SDK 管理器初始化（`_arm_sdk_manager` 保持 `None`），运行时抛 `RuntimeError("ArmSDKManager 未初始化")` / `RuntimeError("RobotSDK 未初始化")`。
   - 安装方式：`bash scripts/install_local_sdk.sh`（从本地 `drivers/leju/kuavo_humanoid_sdk` 源码安装到当前 Python 环境），验证：`python3 -c "from kuavo_humanoid_sdk import KuavoRobot; print('SDK Ready')"`。
   - 当前 dry-run 模式下此依赖被 `STUDIO_DRY_RUN` 短路，不影响结构验证。
