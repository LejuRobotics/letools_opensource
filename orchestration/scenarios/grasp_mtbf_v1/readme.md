# grasp_mtbf_v1 — 实机搬箱场景

本场景沿用现有实机的 Tag、坐标变换、躯干限位、准备位和 JiBot 路线。下载目录的 `gazebo_test` 是仿真参考，没有覆盖到此场景。

## 修改边界

场景只使用 `orchestration/nodes/grasp_mtbf_v1/` 中的 `GraspMtbf*` 专用节点；JSON 中显式引用这些名字。公共 `adapters/`、`core/`、原有通用节点及其他场景均与目标 `dev` 保持一致。

`scene_io.py` 与 `jibot_io.py` 只封装本场景需要的 ROS 服务及反馈订阅，`ros_service_call.py` 提供请求总时限和取消检查。它们不修改共享硬件对象、公共类或 SDK；原有夹爪及底盘位置/速度接口仍调用公共硬件对象。服务互锁和故障状态由场景实例持有，不改变其他场景的调用策略。该隔离是代码范围隔离，不代表多个场景可以同时控制同一机器人。

## SDK 1.4.5 兼容

SDK 子模块与 `dev` 一致，保持官方 1.4.5 对应提交 `d824a7b877b41d0b9e6a9ffe947e050d89ea9681`，SDK 源码无修改。旧本地补丁提交不再作为子模块指针，避免流水线拉取远端不存在的提交。

该版本控制器固定使用 BaseArm。`GraspMtbfCalcLegMove` 不再请求或等待 ArmOnly，也不使用旧 NoControl 模式停机。手臂 0/1/2 控制服务与旧 MPC 模式是不同接口；本次没有新增手臂模式切换。

## 动作时序

- 预抓取准备与靠近改成 `Sequence`：先等待准备位的规划时长，再进入 `walk_to_tag`。这两个动作不再使用 Parallel/Async 包装。
- 找 Tag 的头部扫描、感知与黑板等待仍并行运行。
- JiBot 绝对导航提交任务号后，由 `GraspMtbfCheckArrived` 轮询；相对后退节点先确认到达，再允许后续控制权交接。提交、相对后退及控制权节点的取消会传到 ROS 请求发送边界。
- `GraspMtbfMoveArm` 在每个关键点下发后保持 RUNNING，按 `max(请求时长, actualTime) + settle_time` 等待，然后执行该索引的夹爪操作和下一点。`isSync=True` 只同步左右规划器时长，服务返回表示接受指令，不表示实际到位。
- 手臂服务调用放在后台；单次默认等待 6 秒响应，单个关键点最大时长默认 60 秒。超时、取消、失败或非法响应后节点失败，已开始动作时请求 `/enable_control false`，并禁止同进程手臂节点自动重试。迟到响应不会触发下一动作。
- 场景私有 ROS helper 为定时命令、Ruckig 参数和 JiBot 服务设置发现及响应的总时限；请求结果不确定时锁存故障。客户端超时不能撤销服务端已经收到的请求。

原有 `WaitForEnter` 和 `debug_break` 人工确认点保留。`GraspMtbfWheelWalk` 仍是位置指令提交节点，不是自动到达检测；靠近后的人工确认必须在底盘实际停稳后进行。JiBot 段继续使用 `GraspMtbfCheckArrived`。公共底盘位置接口自身有同步等待，到达检查单次服务调用也会有界等待；并非所有节点都完全非阻塞。

## 躯干检查及限制

- `control_base` 必须为 false。target 模式下躯干 x 使用 `fixed_torso_x`，不把 Tag 偏移直接作为前后位移。
- 保留 x/z 范围、姿态和 x 跳变量检查，等待电机错误码、当前躯干目标及底盘指令零速；发送前重新检查 x 跳变量，执行期间持续检查异常。
- 按返回的规划时长等待，再核对控制器躯干目标反馈。参数见节点中的 `ready_timeout`、`motion_timeout`、`settle_time`；旧 `mode_timeout` 仅作为就绪等待时限的兼容别名。
- `/move_base/base_cmd_vel` 是速度指令，`torso_target_6D` 是控制器目标；它们不是实际底盘锁止或躯干到位测量。现有状态缓存也不提供反馈年龄，不能据此证明物理静止。
- 故障停止请求可能失败。需现场确认控制器状态、排除故障后重新使能和启动；`/enable_vel_control false` 仅停发控制器速度，不能当作底盘物理锁止。

## 执行次数

当前入口执行一轮，没有循环装饰器；控制器默认 `max_iterations=1`。保留 `WaitForEnter` 和 `debug_break`，需要人工推进。`--spin` 只维持进程，不表示重复搬箱。

## 配置与运行

现场配置在 `board.json`、`hardware_config.json`；还需确认对应配置目录的 apriltag 和 camera 配置。主要黑板数据为 `latest_tag_<id>`、`walk_goal`、`is_walk_goal_new`、`ArmPoseAndWrench` 和 `ArmMoveResult`。

加载环境中的 ROS、官方 SDK 和项目 Python 依赖后执行结构干跑：

```bash
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/grasp_mtbf_v1 --dry-run --tick-once
```

实机入口为同一命令去掉 `--dry-run --tick-once`。本次验证为 mock 回归和整树 dry-run，不代表实机动作已验证。
