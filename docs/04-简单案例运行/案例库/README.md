# 案例库：从目标找到可运行场景

[返回文档首页](../../README.md) · [04 章教程](../01-最小可运行示例.md) · [原子技能库](../../03-接口详细文档/原子技能库/README.md)

这里索引**已有教程覆盖的公开示例**。场景目录中的其他场景或实验脚本不因出现在仓库里就自动成为公开案例；运行前以对应教程的环境、命令和验证条件为准。

| 目标 | 教程 | 对应场景/能力 | 验证边界 |
|---|---|---|---|
| 学会加载、读树与看输出 | [最小可运行示例](../01-最小可运行示例.md)、[行为树最小示例](../03-行为树最小示例.md) | `studio_smoke_v1` 等教学配置 | 干跑只检查加载和 tick，不验证实体动作 |
| 单部位顺序动作 | [头部顺序动作](../06-头部顺序动作-instructional_case_01.md) | `instructional_case_01`，`Sequence` + 等待节点 | 仿真 / 真机须先启动控制器与 TF 转发 |
| 头部与双臂并行动作 | [手臂与头部并行](../07-手臂与头部并行-instructional_case_02.md) | `instructional_case_02`，阶段 1 为 TimedCmd 单目标，其余 4 段为 SDK 两关键点轨迹 | 先确认外部模式及轨迹起点；节点不验收到位 |
| 头部、躯干与双臂串行 | [头手躯干串行](../08-头手躯干串行-instructional_case_03.md) | `instructional_case_03`，每阶段 `HeadControlSdkMove → MultiTimedCommandMove`（planner 2 位姿）`→ ArmJointTimedMove`（14D 关节） | 两次 TimedCmd 按规划时长等待，非实测到位；结束不自动恢复模式 |
| 底盘与上身并行 | [头手躯干与底盘并行](../09-头手躯干与底盘并行-instructional_case_04.md) | `instructional_case_04`，JiBot 底盘相对旋转 | 需真机 JiBot 底盘；底盘任务只提交不检查到达 |
| 测试部门 01→04 顺序联测 | [case05 运行说明](../../../orchestration/scenarios/instructional_case_05/README.md) | `instructional_case_05`，顺序执行 01–04 子树，相邻案例间用公共 `WaitSeconds` 等待 2 秒，board 键按案例加前缀隔离 | 非 dry-run 包含 JiBot 真机动作；成功仅表示现有节点返回，不新增到达等待、模式恢复或复位 |
| Tag 与搬箱 | [搬箱场景](../10-搬箱场景-grasp_mtbf.md) | `grasp_mtbf` | 需要地图导航与真实 Tag；虚拟 Tag 只用于干跑和仿真 |
| 模仿学习抓取 | [小件上料模仿学习](../11-小件上料模仿学习-grasp_ring_pick_place.md) | `grasp_ring_pick_place`，ACT 模型 | 仅 MuJoCo 仿真；需要本案例补丁包与 LeTools-Learning 环境 |
| 真机模仿学习上料 | [真机小件上料模仿学习](../12-真机小件上料模仿学习-grasp_ring_pick_place_real.md) | `grasp_ring_pick_place_real`，PI05 / ACT 模型 | V62 轮臂真机；干跑只验证加载，配套配置与权重另行准备 |

新增公开案例时，在索引里写明**场景目录、输入配置、启动命令、运行环境、预期输出、失败排查以及复用的技能**。现有案例若缺其中一项，应在原教程补齐；不要把案例直接标成“标准技能”。单项能力的参数和调用见[原子技能库](../../03-接口详细文档/原子技能库/README.md)。
