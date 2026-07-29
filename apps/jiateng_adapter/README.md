# 嘉腾底盘适配器测试

本目录用于验证 `LejuWheeledArmHardware` 的嘉腾底盘接口。嘉腾是仓库中
新增的一款底盘选择，与 JiBot 并列存在；这里使用独立的 `_jiateng`
方法，不调用 `_jibot` 实现。

话题控制和脚本使用示例见
[《嘉腾底盘控制使用说明》](./嘉腾底盘控制使用说明.md)。

## 文件

| 文件 | 用途 |
|---|---|
| `_scaffold.py` | 初始化 ROS、等待服务、读取任务状态和 AMCL 位姿 |
| `test_base_move.py` | 嘉腾相对移动、原地旋转、移动加旋转 |
| `test_move_to_target.py` | 嘉腾 map 绝对目标移动 |
| `test_enable_vel_control.py` | 外部 `/cmd_vel` 转发开关 |

## 嘉腾适配器接口

```python
hardware.base_move_relative_jiateng(...)
hardware.base_move_to_target_jiateng(...)
hardware.check_arrived_jiateng(...)
hardware.enable_vel_control_jiateng(...)
hardware.get_vel_control_state_jiateng(...)
```

对应 ROS 接口：

| 功能 | ROS 接口 | 类型 |
|---|---|---|
| 相对移动 | `/move_base/base_move` | `leju_mobile_base_msgs/BaseMove` |
| 绝对移动 | `/move_base/move_to_target` | `leju_mobile_base_msgs/MoveToTarget` |
| 到达查询 | `/move_base/check_arrived` | `leju_mobile_base_msgs/CheckArrived` |
| 当前任务 | `/move_base/robot_status` | `leju_mobile_base_msgs/RobotStatus` |
| AMCL 位姿 | `/move_base/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` |
| 外部速度开关 | `/enable_vel_control` | `std_srvs/SetBool` |
| 外部速度状态 | `/enable_vel_control_state` | `std_msgs/Bool` |

## 运行方式

进入仓库并加载本地消息包：

```bash
cd ~/letools_opensource
source infrastructure/ros_packages/devel/setup.bash
```

每个测试文件的 `main` 都使用显式 `suite.addTest(...)`。运行前只保留一个
未注释的用例，避免连续创建多个运动任务。不要使用 pytest 直接执行整个
文件，否则 pytest 会自动发现其中全部 `test_*` 方法。

相对移动：

```bash
python3 apps/jiateng_adapter/test_base_move.py
```

绝对位置：

```bash
python3 apps/jiateng_adapter/test_move_to_target.py
```

外部速度通道：

```bash
python3 apps/jiateng_adapter/test_enable_vel_control.py
```

## 外部速度通道

现场连接关系为：

```text
/cmd_vel
  → /nodelet_manager
  → /move_base/base_cmd_vel
  → /leju_node
  → 嘉腾底盘
```

`/enable_vel_control` 的实机语义：

- `data=false`：关闭外部 `/cmd_vel` 转发。
- `data=true`：开启外部 `/cmd_vel` 转发。
- 该开关不是嘉腾导航总开关；`data=true` 时 `/move_base` 仍可使用。
- 外部 `/cmd_vel` 与导航可能同时汇入底盘，测试时不要同时输出非零命令。
- 停止 `/cmd_vel` 连续发布后，底盘会通过速度超时机制停止。

当前 `/cmd_vel` 的注册发布者可能包括手柄节点。发布者存在不等于正在发送
速度，使用以下命令检查：

```bash
rostopic info /cmd_vel
timeout 3 rostopic hz /cmd_vel
```

## 测试结果判定

运动测试会检查：

1. 服务调用成功。
2. 返回非空 `task_id`。
3. 非零任务没有被下游判定为 `accepted_zero_displacement`。
4. `/move_base/check_arrived` 在超时内确认到达。

嘉腾下游可能缓存重复目标。重复执行相同目标时，即使请求距离非零，也可能
返回 `repeat_same_target=true` 和 `accepted_zero_displacement`；测试会把
这种情况判定为失败，而不是仅凭服务返回 `success` 认为底盘已经移动。

## 已知问题

- `/move_base/make_plan` 曾在实机调用时导致 `lqr_path_track_node`
  以 `SIGSEGV` 退出，因此本目录不包含该接口测试。
- `/enable_vel_control_state` 可能只在状态变化时发布；单独读取当前状态时
  可能等待超时。
- 运动服务返回成功只表示任务被接受，最终结果仍以指定 `task_id` 的
  `check_arrived` 响应为准。
