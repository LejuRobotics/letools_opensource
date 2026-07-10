# studio_smoke_v1
## 分层结构

```text
studio_smoke_v1 冒烟场景
├── 场景配置层
│   ├── board.json          # 黑板变量与运行时参数
│   ├── py_tree.json        # 主行为树，描述执行顺序
│   └── py_tree_child.json  # 子树模板，复用局部流程
│
├── 编排入口层
│   └── orchestration/main.py            # 加载场景 JSON，创建并 tick 行为树
│
├── 行为节点层
│   ├── WaitForEnter                    # 等待人工确认
│   ├── ChassisShortMove                # 底盘短距离移动
│   ├── LegShortMove                    # 腿部/躯干动作
│   └── MoveArmBaseJointTrajectories    # 手臂关节轨迹
│
├── 原子技能层
│   ├── chassis_velocity                # 底盘速度技能
│   ├── leg_control                     # 腿部控制技能
│   └── arm_control                     # 手臂控制技能
│
└── 硬件适配层
    └── IHardware 标准方法              # send_base_velocity、send_leg_joint_command、send_arm_joint_trajectory
```

阶段 1 简单生产冒烟树：

`WaitForEnter` → `ChassisShortMove` → `LegShortMove` → `MoveArmBaseJointTrajectories`

主路径：编排薄节点 → Skill → **IHardware 标准方法**（对齐 `apps/test_kuavo_5w_app`）：

| 步骤 | Skill | 标准方法 | 参考测试 |
|------|-------|----------|----------|
| 底盘 | `chassis_velocity` | `send_base_velocity` | `01_base_control/test_cmd_vel_base.py` |
| 躯干/腿 | `leg_control` | `send_leg_joint_command` | `02_lower_body/test_leg_joint.py` |
| 手臂 | `arm_control` | `enable_quick_mode` + `send_arm_joint_trajectory` | `03_arm_control/test_arm_joint.py` |

## 运行

入口用法详见 [orchestration/README.md](../../README.md#mainpy-使用说明)。

```bash
export PYTHONPATH=/path/to/LeTools:$PYTHONPATH
cd /path/to/LeTools

# 预检三条标准方法（真机）
python3 apps/test_kuavo_5w_app/verify_phase1_standard_methods.py

# 无 ROS/实机
python3 orchestration/main.py --dry-run
python3 orchestration/main.py --dry-run --tick-once

# 实机（需 rospy、下位机、按 Enter）；跑完自动退出
python3 orchestration/main.py

# 跑完后保持 ROS 节点（旧 embodied 行为，需 Ctrl+C）
python3 orchestration/main.py --spin
```

## 锁定参数

| 项 | 值 |
|----|-----|
| 底盘 | vx=0.3 m/s，3s，本体系，结束零速 |
| 腿 | [14.90, -32.01, 18.03, 30.0] °（test_leg_joint 右膝弯曲） |
| 臂 | test_arm_joint 展开双臂 14 关节，time_sec=3.0 |

## 安全

实机前确认工作空间无障碍；底盘结束发零速；臂动前已 `enable_quick_mode(True)`。

