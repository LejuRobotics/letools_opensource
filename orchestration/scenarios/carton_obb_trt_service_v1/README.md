# carton_obb_trt_service_v1

一键启动共享纸箱 OBB TensorRT 推理服务和阻塞式底盘视觉伺服服务。推理服务常驻加载：

```text
skills/atomic/perception/carton_detect/models/carton_obb_fp16.engine
```

场景会先检查公共头部 RGB-D 相机：已有健康发布者时直接复用，完全未启动时拉起
Orbbec 驱动；同时确保 `head_camera_depth -> camera_link` 静态 TF 已存在，缺失时
由本场景补齐。随后 OBB 服务常驻订阅彩色话题并缓存最新帧，多个业务进程通过
`/carton_obb/infer` 调用同一个模型，避免重复启动相机、加载 engine 和占用显存。
推理服务只负责图像前处理、TensorRT 推理和旋转 NMS；`/carton_obb/servo`
复用该后端完成深度采样、目标锁定和底盘三自由度闭环。

## 目录

| 文件 | 作用 |
|---|---|
| `board.json` | 公共相机、TF 桥接、engine、推理与伺服服务配置 |
| `start_all.sh` | 复用/启动相机与 TF 桥接、清理残留节点、启动双服务并镜像日志 |

该场景不包含行为树，因此不创建无业务意义的 `py_tree.json` 和
`hardware_config.json`。

## 首次构建

服务消息或节点代码更新后执行：

```bash
cd <LeTools 仓库根>/infrastructure/ros_packages
catkin build box_detection_msgs carton_obb
source devel/setup.bash
```

## 一键启动

需要 ROS master 已运行：

```bash
cd <LeTools 仓库根>/orchestration/scenarios/carton_obb_trt_service_v1

# 只检查配置、engine、ROS 包和 master，不启动服务
bash start_all.sh --dry-run

# 正式启动，前台运行
bash start_all.sh
```

退出时只关闭本场景启动的相机和 TF 桥接；已有相机或 TF 发布器均保持运行。

推理服务部分等价于：

```bash
roslaunch carton_obb carton_obb_trt_service.launch \
  engine_path:=<LeTools 仓库根>/skills/atomic/perception/carton_detect/models/carton_obb_fp16.engine \
  image_topic:=/camera/color/image_raw
```

但额外提供：

- 从 `board.json` 统一读取配置；
- 自动 source ROS 与本仓库 catkin 环境；
- 复用健康的公共 RGB-D，或在完全未启动时拉起 Orbbec；
- 复用已有头部相机 TF，或补齐 `head_camera_depth -> camera_link`；
- 等待彩色和深度首帧后再加载 TensorRT；
- 检查 engine、消息类型和 ROS master；
- 停止上一轮残留的推理和伺服节点；
- 服务端拒绝超过 `max_frame_age` 的陈旧相机帧；
- 退出时只关闭本场景拥有的相机与 TF 进程；
- 将终端输出实时镜像到 `log/carton_obb_trt_service/`。

## 调用视觉伺服

三个请求参数分别对应最大角速度、最大线速度和 `base_link` 下的目标 X。其余参数使用
服务端默认值，目标 Y 默认为 `0m`：

```bash
rosservice call /carton_obb/servo \
  "{max_yaw_rate_deg_s: 4.0, max_linear_speed_m_s: 0.04, target_x_m: 1.10}"
```

调用会阻塞到伺服完成或超时。成功响应的 `carton_pose` 是完成时纸箱在
`waist_link` 下的坐标：

```yaml
success: true
message: "Servo aligned; waist_link xyz=(...)m"
carton_pose:
  header:
    frame_id: "waist_link"
  point:
    x: 0.0
    y: 0.0
    z: 0.0
```

成功结果还会锁存发布到 `/carton_obb/servo_result`。超时、异常或并发请求返回
`success: false`；所有结束路径都会发送零速度并恢复 `/enable_vel_control=True`。

## 验证

```bash
rosnode list | grep carton_obb_trt_service
rosnode list | grep carton_obb_servo_service
rosservice type /carton_obb/infer
rosservice info /carton_obb/infer
rosservice type /carton_obb/servo
rosservice info /carton_obb/servo
rosrun tf tf_echo base_link camera_color_optical_frame
```

## 配置

修改同目录 `board.json`：

| 键 | 默认值 | 说明 |
|---|---|---|
| `engine_path` | 仓库内 `carton_obb_fp16.engine` | 相对路径按仓库根解析 |
| `service_name` | `/carton_obb/infer` | 所有调用方使用的服务名 |
| `warmup` | `5` | 服务注册前的预热次数 |
| `python` | 空串 | 可指定包含 TensorRT 的 Python 解释器 |
| `max_frame_age` | `0.5` | 允许推理的最新帧最大接收年龄（秒） |
| `servo_service_name` | `/carton_obb/servo` | 阻塞式视觉伺服服务名 |
| `servo_script` | `skills/.../realtime_carton_obb_trt.py` | 视觉伺服服务端脚本 |
| `servo_result_topic` | `/carton_obb/servo_result` | 成功结果锁存话题 |
| `servo_result_frame` | `waist_link` | 成功响应坐标系 |
| `servo_depth_weight` | `0.3` | 拆垛抓箱选箱：深度权重（仅 `/carton_obb/servo`） |
| `servo_bottom_center_weight` | `0.5` | 拆垛抓箱选箱：画面下方中心权重（仅 `/carton_obb/servo`） |
| `servo_area_weight` | `0.2` | 拆垛抓箱选箱：面积权重（仅 `/carton_obb/servo`） |
| `start_camera_if_missing` | `true` | 没有相机发布者时自动启动 |
| `camera_launch` | `config/camera_orbbec.launch` | 公共头部相机 launch |
| `color_topic` | `/camera/color/image_raw` | 服务端最新帧来源 |
| `depth_topic` | `/camera/depth/image_raw` | 供下游复用的配准深度 |
| `camera_startup_timeout` | `20` | 等待 RGB-D 首帧超时（秒） |
| `publish_tf_bridge` | `true` | 缺少桥接时自动启动静态 TF |
| `tf_parent_frame` | `head_camera_depth` | 机器人侧相机安装 frame |
| `tf_child_frame` | `camera_link` | Orbbec 驱动根 frame |
| `tf_transform_args` | `0 0 0 0 0 0` | `x y z yaw pitch roll` |
| `tf_startup_timeout` | `10` | 等待桥接可查询的超时（秒） |

服务请求不再携带图像，只传 `confidence` 和 `nms_iou`。响应中的
`source_header.stamp` 表示实际参与推理的头部相机帧；需要深度或可视化的调用方必须
按该时间戳匹配本地缓存，避免跨帧使用检测框。
