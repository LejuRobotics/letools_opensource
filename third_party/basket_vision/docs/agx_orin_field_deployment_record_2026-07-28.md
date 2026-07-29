# AGX Orin 实机部署与箱体位姿验证记录（2026-07-28）

本文记录一次在 NVIDIA AGX Orin 上对 Basket Vision 的最小化实机部署。目标仅为验证箱体检测及 6D 位姿估计，不启动机器人运动、规划或控制模块。

## 1. 最终结论

本次部署已经完成以下验证：

- Orbbec Gemini 335L 能以纯传感器模式发布彩色图、深度图和相机内参。
- YOLO 与 GDRN 模型能够在 Orin GPU 上成功加载和推理。
- `/infer_basket_pose` 与 `/infer_top_basket_ids` 服务能够启动并触发推理。
- 实物 `basket_4622` 被正确框选，并输出了 `camera_color_optical_frame` 下的 6D 位姿。
- 静止箱体连续两次推理的位置差为 `12.2 mm`，旋转差为 `1.329°`。

当前唯一关键缺口是没有真实的
`base_link <- camera_color_optical_frame` TF。因此相机坐标系下的位姿可用，机器人底盘坐标系下的位姿不可用。当前服务实现会在 TF 缺失时最终返回 `success: False` 和空数组，但推理日志中的 `[CAM_POSE_6D]` 仍包含经过有效性检查的相机坐标位姿。

本次没有启动任何运动组件，也没有用未经标定的静态 TF 冒充实机坐标。

## 2. 实机与目录

| 项目 | 实测值 |
|---|---|
| 设备 | NVIDIA AGX Orin，aarch64 |
| Jetson Linux | R35.6.0 |
| Ubuntu / ROS | Ubuntu 20.04 / ROS 1 Noetic |
| CUDA | 11.4，`nvcc` 11.4.315 |
| 系统 Python | 3.8.10，`/usr/bin/python` |
| 工程目录 | `/home/leju_kuavo/zwl/third_party/basket_vision` |
| 隔离运行环境 | `/home/leju_kuavo/basket_vision_envs/gdrn38` |
| ROS 消息工作区 | `basket_vision_ws` |
| 相机光学坐标系 | `camera_color_optical_frame` |
| 期望机器人坐标系 | `base_link` |

`cv_bridge` 在系统 Python 中已经验证通过。部署过程中没有修改或替换系统 Python。

## 3. 初始检查及问题

最初执行的基础检查为：

```bash
source /opt/ros/noetic/setup.bash
python -V
command -v python
cat /etc/nv_tegra_release
nvcc --version
python -m pip check
```

系统环境中同时存在多个 OpenCV wheel，并且 `pip check` 报告了 `pyzed`/Cython 版本冲突和若干 aarch64 平台兼容警告。它们不代表 `cv_bridge` 已失效，但继续直接修改该环境会影响机器人已有软件，因此后续改用独立 Python 3.8 环境。

第一次诊断命令失败：

```text
bash: basket_vision_module/scripts/diagnose_jetson_orin_env.sh: No such file or directory
```

原因是当时所在目录或工程内容与预期不一致。脚本存在时，从工程根目录应使用：

```bash
cd /home/leju_kuavo/zwl/third_party/basket_vision
bash basket_vision_module/scripts/diagnose_jetson_orin_env.sh "$(command -v python)"
```

初始工程也没有可直接导入的 `basket_gdrnpp/detectron2/detectron2` 源码目录，因此不能依赖该路径，需要在隔离环境中安装可用的 Detectron2 CUDA wheel。

## 4. 实际部署过程

### 4.1 隔离 Python 环境

本次最终使用：

```bash
source /home/leju_kuavo/basket_vision_envs/gdrn38/bin/activate
python -V
python -c 'import sys; print(sys.executable)'
```

核心验证结果：

- Python 3.8.10；
- PyTorch `2.0.0+nv23.05`；
- `torch.cuda.is_available()` 为真；
- TorchVision CUDA NMS 可执行；
- Detectron2 0.6 CUDA 扩展可导入；
- `rospy`、`cv_bridge` 和生成的 ROS service 消息可导入。

隔离环境中补齐或调整的主要依赖包括：

- Detectron2 0.6 wheel；
- Pillow 9.5.0；
- scikit-image 0.21.0；
- chardet 5.2.0；
- rospkg 1.6.0；
- catkin-pkg 1.0.0；
- yapf 0.40.1。

剩余的 `pip check` 提示属于 Detectron2 的训练或可选依赖，没有阻塞本次推理。环境验收命令为：

```bash
cd /home/leju_kuavo/zwl/third_party/basket_vision
source /home/leju_kuavo/basket_vision_envs/gdrn38/bin/activate
bash basket_vision_module/scripts/diagnose_jetson_orin_env.sh \
  /home/leju_kuavo/basket_vision_envs/gdrn38/bin/python
```

预期结尾为：

```text
RESULT: PASS
```

### 4.2 编译 ROS service 消息

```bash
cd /home/leju_kuavo/zwl/third_party/basket_vision/basket_vision_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash

python - <<'PY'
from basket_vision_msgs.srv import InferBasketPose
print("basket_vision_msgs import ok")
PY
```

### 4.3 为实机兼容做的代码调整

| 文件 | 调整原因 |
|---|---|
| `basket_gdrnpp/core/gdrn_modeling/demo/predictor_gdrn.py` | 使用 Python 3.8 支持的 `Optional[np.ndarray]`；推理时延迟导入 `pyassimp`/渲染模块；禁止重复下载 timm 预训练权重 |
| `basket_gdrnpp/core/utils/my_checkpoint.py` | 兼容 Lightning 1.9 的 `_FabricModule`/`_LiteModule` 差异 |
| `basket_gdrnpp/core/gdrn_modeling/demo/box_configs/basket_5.yaml` | 指向实机已有权重 `model_final_5.pth.1` 和 `best_5.pt` |
| `basket_gdrnpp/core/gdrn_modeling/demo/inference_service_vis_2_mult_inst_10_shared.py` | 增加 `[CAM_POSE_6D]` 日志，输出类别、置信度、bbox、相机坐标 XYZ 和四元数 |

远端被修改文件在操作前已备份。

### 4.4 启动 ROS 和相机

本次 ROS master、Orbbec Gemini 335L 纯传感器节点和感知服务均运行在实机上。若 ROS master 尚未启动，可在独立终端执行：

```bash
source /opt/ros/noetic/setup.bash
roscore
```

相机使用机器人现有 Orbbec 驱动启动。本次具体的厂商 launch 参数没有完整保存在记录中，因此这里不填写推测命令；启动后的接口验收标准如下：

```bash
source /opt/ros/noetic/setup.bash
rostopic list | grep -E '/camera/color/image_raw|/camera/depth/image_raw|/camera/color/camera_info'
timeout 10 rostopic echo -n 1 /camera/color/image_raw >/dev/null && echo "color ok"
timeout 10 rostopic echo -n 1 /camera/depth/image_raw >/dev/null && echo "depth ok"
timeout 10 rostopic echo -n 1 /camera/color/camera_info >/dev/null && echo "camera info ok"
```

此步骤只启动传感器，不启动底盘、机械臂或全身控制。

### 4.5 启动最小感知服务

在新终端执行：

```bash
cd /home/leju_kuavo/zwl
source /opt/ros/noetic/setup.bash
source /home/leju_kuavo/zwl/third_party/basket_vision/basket_vision_ws/devel/setup.bash

export KUAVO_STUDIO_DIR=/home/leju_kuavo/zwl
export UV_ACTIVATE_SCRIPT=/home/leju_kuavo/basket_vision_envs/gdrn38/bin/activate

bash third_party/basket_vision/basket_vision_module/scripts/start_gdrn_inference.sh
```

启动脚本会等待 `/camera/color/image_raw`，然后加载 YOLO、GDRN 和两个模型权重。成功标志为：

```text
SharedBasketPoseServiceNode ready.
  service: /infer_basket_pose
  service: /infer_top_basket_ids
[gdrn] inference service is ready
```

验证服务存在：

```bash
rosservice list | grep -E '/infer_basket_pose|/infer_top_basket_ids'
```

### 4.6 触发一次实物识别

```bash
source /opt/ros/noetic/setup.bash
source /home/leju_kuavo/zwl/third_party/basket_vision/basket_vision_ws/devel/setup.bash

rosservice call /infer_basket_pose "{}"
rosservice call /infer_top_basket_ids "{}"
```

查看相机坐标位姿日志：

```bash
grep -R '\[CAM_POSE_6D\]' \
  /home/leju_kuavo/zwl/third_party/basket_vision/logs/gdrn_inference/
```

也可查看带检测框的图像：

```bash
rqt_image_view /basket_vision/viz_image
```

## 5. 本次实测结果

箱体放置不动时，第一次识别结果为：

| 字段 | 数值 |
|---|---|
| 类别 | `basket_4622` |
| 置信度 | `0.9262` |
| 坐标系 | `camera_color_optical_frame` |
| 位置 XYZ | `(-0.261740, 0.162322, 1.198416) m` |
| 四元数 XYZW | `(-0.010437, -0.234511, 0.972020, 0.008517)` |
| RPY | `(-27.14°, 0.93°, 178.77°)` |
| bbox XYXY | `(144.3, 214.1, 309.7, 394.9) px` |

第二次识别结果：

| 字段 | 数值 |
|---|---|
| 类别 | `basket_4622` |
| 置信度 | `0.8688` |
| 位置 XYZ | `(-0.259293, 0.165869, 1.209810) m` |
| 相对第一次的位置差 | `12.2 mm` |
| 相对第一次的旋转差 | `1.329°` |

检测可视化中，高置信度 bbox 覆盖了实际绿色箱体。以上数据证明检测、分类和相机坐标 6D 位姿链路能够运行，但一次或两次结果不足以证明绝对精度。

相机光学坐标系约定为：

- `X`：画面向右为正；
- `Y`：画面向下为正；
- `Z`：相机朝前为正；
- 单位：米；
- 姿态四元数顺序：`x, y, z, w`。

## 6. 当前 TF 限制

检查命令：

```bash
source /opt/ros/noetic/setup.bash
timeout 5 rosrun tf tf_echo base_link camera_color_optical_frame
```

本次没有得到该 TF。其影响为：

- `[CAM_POSE_6D]` 中的相机坐标位姿仍然有效；
- `poses_base_link` 无法生成；
- 当前 service 的响应流程要求 `base_link`，所以最终可能显示 `success: False` 和空数组；
- 不能把相机 XYZ 直接解释成机器人 XYZ，因为两套坐标轴方向不同。

仓库虽然提供 `start_basket_tf_fallback.sh`，但其默认外参来自特定机型的 URDF 零位。它可用于排查 TF 链路，不可作为这台实机的精度真值。本次验证没有启用该 fallback。

要获得可靠的 `base_link` 位姿，下一步应完成真实相机外参标定，并由 robot state publisher 或经确认的静态 TF 发布：

```text
base_link -> ... -> camera_link -> camera_color_optical_frame
```

## 7. 现实世界精度验证方法

### 7.1 先确认被比较的物理点

网络返回的是 CAD 模型坐标原点，不一定是箱体前表面。测量前应确认相应 `.ply` 模型原点；若模型原点位于箱体几何中心，激光测距仪测到前表面后，还需根据箱体朝向加上从前表面到中心的距离。

### 7.2 平移精度

1. 在相机上标记光学中心位置和镜头朝向。
2. 用水平仪和地面胶带画出相机光轴。
3. 先将箱体原点放在光轴上，此时预期 `X≈0`、`Y≈0`。
4. 分别把箱体沿相机 `X/Y` 方向移动已知距离，例如 `±0.20 m`。
5. 在 `Z=0.8/1.0/1.2/1.5 m` 等多个距离重复测量。
6. 每个位置静止调用 20 次，记录均值、标准差和最大误差。

建议记录表：

| 测点 | 尺量真值 XYZ (m) | 推理均值 XYZ (m) | 绝对误差 (m) | 20 次标准差 (m) |
|---|---|---|---|---|
| P1 |  |  |  |  |
| P2 |  |  |  |  |
| P3 |  |  |  |  |

### 7.3 朝向精度

1. 在箱体上固定标记“正面”和箱体局部 X 轴。
2. 在地面贴出 `0°、±30°、±45°、90°` 参考线。
3. 逐个角度静止采集 20 次。
4. 对称箱体可能存在多个等价朝向，误差应按模型对称性取最小等价角，而不是直接相减欧拉角。
5. 姿态比较优先使用两个四元数的最小旋转夹角，避免欧拉角跳变。

### 7.4 更可靠的独立真值

可将 AprilTag 或 ArUco 板刚性固定在箱体上，并准确测量“标签坐标系到箱体 CAD 原点”的刚体变换。使用已标定相机独立求出标签位姿，再与 GDRN 结果逐帧比较。标签尺寸应精确测量，板面保持平整；否则标签系统自身误差会被误认为 GDRN 误差。

若条件允许，光学动捕、全站仪或高精度机械测量夹具可提供更强的真值。

可先采用以下工程验收目标，之后按抓取容差收紧：

- 平移平均误差不超过 `2–3 cm`；
- 静止重复性标准差不超过 `1 cm`；
- 姿态误差不超过 `3–5°`；
- 检测类别和 bbox 在目标工作距离内稳定。

这些数值是建议验收线，不是本模型已经达到的精度声明。在真实 TF 标定完成前，只评估相机坐标系；不要评估或使用 `base_link` 坐标。

## 8. 停止与恢复

本次启动的都是非运动节点。停止时分别在以下终端按 `Ctrl-C`：

1. GDRN 推理服务；
2. Orbbec 相机节点；
3. 本次单独启动的 `roscore`。

启动脚本收到 `Ctrl-C` 后会终止其 Python 子进程。临时 SSH 公钥已从实机 `authorized_keys` 中移除，本地临时私钥也已删除。

## 9. 最后状态快照

本次远程操作结束前的最后一次检查中，ROS master、Orbbec 纯传感器节点和感知服务仍在运行。该描述只是当时状态，不保证机器重启或会话结束后的持续状态；重新测试前应按第 4 节逐项检查。
