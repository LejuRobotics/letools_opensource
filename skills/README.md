# skills — 原子技能层

本目录是编排系统的**业务逻辑核心**，每个 Skill 封装一个独立的硬件动作或编排工具操作。


## 新手理解：Skill 是什么

`Skill` 可以理解成“机器人可复用的小动作/小能力”。它比底层硬件接口更接近业务动作，但又比完整任务更小。

例如：

```text
底盘移动       -> 一个 Skill
手臂运动       -> 一个 Skill
头部控制       -> 一个 Skill
等待几秒       -> 一个 Skill
读取二维码     -> 一个 Skill
抓取 / 放置    -> 可以由多个 Skill 组合而成
```

Skill 的位置大概是：

```text
orchestration 行为树
└── 决定什么时候执行哪个步骤
    ↓
skills 原子技能
└── 封装单个动作的业务逻辑
    ↓
hardware 标准接口
└── 负责真正调用 Adapter/SDK/ROS
```

### Skill 和 Node 的区别

很多新手容易把 `orchestration/nodes/` 和 `skills/` 混在一起，可以这样区分：

```text
Node（行为树节点）
└── 负责接入 py_trees 生命周期、解析 JSON 参数、返回 SUCCESS/FAILURE/RUNNING。

Skill（原子技能）
└── 负责具体业务动作，比如控制手臂、控制底盘、等待、读取感知结果。
```

也就是说，Node 更像“行为树接口层”，Skill 更像“动作实现层”。

### Skill 和 Hardware 的关系

Skill 通常不直接发 ROS 话题，也不直接操作底层 SDK，而是通过 `hardware` 调用统一接口：

```python
result = self.hardware.send_base_velocity(...)
result = self.hardware.control_head_sdk(...)
result = self.hardware.send_arm_joint_trajectory(...)
```

这样 Skill 不需要知道底层话题名、服务名、SDK 初始化细节。

### 一个最小 Skill 的理解方式

一个 Skill 通常包含三件事：

```text
1. 参数：这个动作需要什么输入
2. 执行：调用 hardware 做动作
3. 结束判断：什么时候算成功或失败
```

伪代码可以这样理解：

```python
class ExampleSkill(SkillBase):
    def on_initialize(self):
        # 检查参数是否合法
        return True

    def on_execute(self):
        # 调用 hardware 执行动作
        self.result = self.hardware.send_base_velocity(vx=0.2, vy=0.0, vyaw=0.0)
        return self.result

    def on_is_finished(self):
        # 简单动作可以执行一次就结束
        return True
```

真正开发时请参考 `skills/atomic/refactored_sdk/` 下的现有技能，因为那里是当前主力的新架构写法。

### 新增 Skill 的推荐步骤

```text
1. 先确认 hardware 里有没有对应能力
   例如 send_base_velocity、control_head_sdk、send_arm_joint_trajectory。

2. 在 skills/atomic/refactored_sdk/ 下新增一个 xxx.py
   让它继承 SkillBase。

3. 定义参数 dataclass
   把动作需要的输入写清楚。

4. 在 on_execute() 中调用 hardware 方法
   不要直接写 ROS Publisher 或 ServiceProxy。

5. 如果要放进行为树，再在 orchestration/nodes/ 下写一个薄节点
   让节点负责 JSON 参数解析和 py_trees 状态转换。
```

一句话总结：

```text
Skill 是可以被行为树复用的机器人动作单元。
```

## 架构分层

```
orchestration/nodes/  (薄封装：参数解析 + py_trees 生命周期)
    │
    ▼ 委托
skills/               (本层：纯业务逻辑)
    │
    ▼ 调用
adapters/hardware/    (硬件适配器)
    │
    ▼ 委托
adapters/hardware/leju_wheeled/services/   (SDK 管理器)
    │
    ▼ 调用
robot_sdk.control.*   (底层 SDK)
```

## 目录结构

```
skills/
├── base/
│   └── skill_base.py          ← 所有 Skill 的基类
│
├── atomic/
│   ├── refactored_sdk/        ← ✅ 当前主力：新架构统一 SDK 直调
│   │   ├── README.md
│   │   ├── arm_ee_traj_local_sdk.py
│   │   ├── arm_ee_traj_world_sdk.py
│   │   ├── arm_joint_traj_sdk.py
│   │   ├── arm_reset_sdk.py
│   │   ├── base_pose_local.py
│   │   ├── head_control_sdk.py
│   │   ├── leg_joint_sdk.py
│   │   ├── wait_for_enter.py
│   │   └── wait_seconds.py
│   │
│   ├── manipulation/          ← ⚠️ 旧版技能（studio_smoke_v1 场景使用）
│   │   ├── arm_control/       → 手臂关节控制
│   │   ├── arm_trajectory/    → 手臂轨迹
│   │   ├── head_control/      → 头部控制
│   │   ├── leg_control/       → 腿部控制
│   │   ├── leg_joint_control/ → 腿部关节
│   │   ├── pick/              → 抓取（pick）
│   │   ├── place/             → 放置（place）
│   │   ├── pos_base_control/  → 本体系位姿
│   │   ├── pos_world_control/ → 世界系位姿
│   │   ├── simple_two_arm_publisher_joint/    → 双臂关节发布
│   │   ├── simple_two_arm_publisher_local/    → 双臂本体系发布
│   │   ├── torso_pose_control/ → 躯干位姿
│   │   ├── two_arm_hand_pose_control/ → 双臂手部位姿
│   │   └── vel_control/       → 速度控制
│   │
│   ├── motion/                ← ⚠️ 旧版运动技能
│   │   ├── chassis_velocity/  → 底盘速度
│   │   └── move_to_pose/      → 移动到位姿
│   │
│   ├── perception/            ← ⚠️ 感知技能（非 SDK 路径）
│   │   ├── camera_capture/    → 相机捕获
│   │   └── read_qrcode/       → 二维码读取
│   │
│   └── grasp_skill.py         ← ⚠️ 旧版抓取（不继承 SkillBase）
```

## 新旧架构对比

| 方面 | refactored_sdk/ (新) | manipulation/motion/ (旧) |
|------|---------------------|--------------------------|
| 基类 | 统一继承 `SkillBase` | 部分继承 `SkillBase`，部分自定义 |
| 硬件路径 | 全部走 SDK 直调 `hardware.xxx_sdk()` | 混合：ROS topic / service / SDK |
| 参数 | `@dataclass Params extends SkillParams` | 各自定义 |
| 目录结构 | 扁平，一文件一技能 | 嵌套，一技能一目录 |
| 对应节点 | `orchestration/nodes/` 统一薄封装 | 部分在 `nodes/`，部分自建 |
| 场景 | `atomic_v1` `arm_v1` | `smoke_v1` |

## SkillBase 生命周期

```
initialize(params)  →  on_initialize()   校验参数，初始化状态
    │
    ▼
execute()           →  on_execute()       执行一次业务操作
    │                                       (50Hz tick 中可能多次调用)
    ▼
is_finished()       →  on_is_finished()   返回 True 表示终态
```

## 编写新 Skill 规范

1. 统一放在 `skills/atomic/refactored_sdk/` 下
2. 继承 `SkillBase`，构造 `@dataclass Params(SkillParams)`
3. 通过 `getattr(self.hardware, "method_name", None)` 获取 Adapter 方法
4. 加 `@define_manifest` 声明行为树元数据
5. 禁止在 `__init__.py` 做聚合导入（防止并行 import 死锁）

