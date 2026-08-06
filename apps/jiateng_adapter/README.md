# 嘉腾底盘适配器测试

本目录用于验证 `LejuWheeledArmHardware` 的嘉腾底盘接口。嘉腾是仓库中
新增的一款底盘选择，与 JiBot 并列存在。

话题控制和脚本使用示例见
[《嘉腾底盘控制使用说明》](./嘉腾底盘控制使用说明.md)。

## 文件

| 文件 | 用途 |
|---|---|
| `_scaffold.py` | 初始化 ROS、读取状态、检查控制模式 |
| `test_base_move.py` | 嘉腾相对移动、原地旋转、移动加旋转 |
| `test_move_to_target.py` | 嘉腾 map 绝对目标移动 |
| `test_enable_vel_control.py` | 嘉腾导航与外部控制模式切换 |

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

每个测试文件的 `main` 都使用显式 `suite.addTest(...)`。根据现场空间和测试
目的，保留需要执行的用例。

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

## 控制模式

嘉腾导航与外部控制互斥：

| 状态 | 可用接口 | 禁止并发接口 |
|---|---|---|
| `false` | `/move_base/base_move`、`/move_base/move_to_target` | `/cmd_vel`、`/cmd_pose`、`/cmd_pose_world` |
| `true` | `/cmd_vel`、`/cmd_pose`、`/cmd_pose_world` | `/move_base/*` 导航任务 |

`data=true` 时，外部速度链路会抢占嘉腾导航源。此时 `/move_base` 服务可能
返回任务已受理，但底盘仅短暂动作后停住，任务保持运行状态。切换控制模式前
必须确认另一类任务已经结束。

导航前设置：

```bash
rosservice call /enable_vel_control "data: false"
```

外部控制前设置：

```bash
rosservice call /enable_vel_control "data: true"
```

测试脚本的控制模式行为：

| 脚本 | 所需模式 | 行为 |
|---|---|---|
| `test_base_move.py` | 嘉腾导航 `false` | 若当前为 `true`，打印 WARNING 后自动切换为 `false` |
| `test_move_to_target.py` | 嘉腾导航 `false` | 若当前为 `true`，打印 WARNING 后自动切换为 `false` |
| `test_enable_vel_control.py` | 无 | 显式切换模式，并保留切换结果 |

当前 `/cmd_vel` 的注册发布者可能包括手柄节点。发布者存在不等于正在发送
速度，使用以下命令检查：

```bash
rostopic info /cmd_vel
timeout 3 rostopic hz /cmd_vel
```

## 测试结果判定

`test_base_move.py` 和 `test_move_to_target.py` 使用嘉腾导航模式。若测试开始时
检测到外部控制模式，脚本会先输出 WARNING，再切换到
`/enable_vel_control=false`。

测试通过必须同时满足：

1. `/move_base` 服务成功受理请求。
2. 响应中包含非空 `task_id`。
3. `/move_base/check_arrived` 在超时时间内返回 `arrived=true`。

服务响应中的 `success=true` 仅表示任务已受理，不能单独证明底盘完成了运动；
最终结果以对应 `task_id` 的 `check_arrived` 响应为准。
