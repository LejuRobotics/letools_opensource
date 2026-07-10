### 功能说明
## 分层结构

```text
detection_show AprilTag 可视化
├── 输入话题层
│   ├── /tag_detections           # apriltag_ros 输出的标签检测结果
│   ├── /camera/color/image_raw   # 相机彩色图像
│   └── /camera/color/camera_info # 相机内参
│
├── 可视化处理层
│   └── tag_detection_show.py     # 计算角点投影，在图像上绘制边框和 ID
│
└── 输出展示层
    ├── /tag_detections_image     # 发布绘制后的图像
    └── rqt_image_view            # 查看可视化结果
```
- 本脚本基于 apriltag_ros 的检测结果，只负责在图像上绘制 AprilTag 的边框和 ID，不依赖于 tags.yaml 中的 tag_size 参数，也不输出实际的物理位姿。
- 适合只关注 可视化效果（四边形边框和 ID） 的场景。

### 依赖环境

- rospy
- cv2 (opencv-python)
- numpy
- cv_bridge

### 使用方法
注意⚠：确保ros主从通信正常，且下位机启动仿真或者实机(因为二维码检测依赖tf树)

1. 启动相机与 apriltag_ros 检测
```bash
cd ~/kuavo_ros_application
source devel/setup.bash 
# 旧版4代, 4Pro
roslaunch dynamic_biped load_robot_head.launch use_orbbec:=false
# 标准版, 进阶版, 展厅版, 展厅算力版
roslaunch dynamic_biped load_robot_head.launch use_orbbec:=true
# Max版
roslaunch dynamic_biped load_robot_head.launch use_orbbec:=true enable_wrist_camera:=true
```

2. 新建一个终端,运行本脚本
```bash
cd ~/kuavo_ros_application
source devel/setup.bash 
python3 src/ros_vision/detection_apriltag/detection_show/tag_detection_show.py
```

3. 查看可视化结果
```bash
rqt_image_view 
```

4. 运行后，你会看到：

- 检测到的标签被绿色边框标出
- 标签 ID 会显示在中心位置
- 发布的图像话题：/tag_detections_image

### rqt结果展示

![](images/二维码可视化预览图.png)

### 原理说明

1. 输入数据

- 来自 /tag_detections 的 AprilTag 检测结果（包含 tag ID、位姿等信息）。
- 来自 /camera/color/image_raw 的相机图像。
- 来自 /camera/color/camera_info 的相机内参（fx, fy, cx, cy）。

2. 计算步骤

- 从检测结果获取 标签的中心位置和姿态。
- 构造一个虚拟的正方形（边长 = default_tag_size，默认 0.04m）。
- 将该正方形的 3D 四个角点变换到相机坐标系。
- 使用相机内参投影到图像平面，得到四个像素点。
- 在图像上绘制四边形轮廓 + 标签 ID。



3. 重点说明关于`tag_size`的问题

- 参数default_tag_size 只是一个 缩放参考值，用于计算角点相对位置。

- 在本脚本中，我们 不关心物理尺寸，只需要在图像上绘制边框，因此 tag_size 变成了一个无关紧要的参数。

- 为了保持计算逻辑一致，这里仍然定义了一个 default_tag_size（默认 0.04m），但它不会影响可视化结果。

- tag_size 合理范围
  - 下限：不能小到比相机像素精度还小。
  - 上限：不能大到不符合实际拍摄场景。
