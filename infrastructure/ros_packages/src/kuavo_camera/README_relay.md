# 相机话题转发节点使用说明

本文档介绍如何使用相机话题转发功能，将任意输入话题转发到 `/leju_camera/color/image_raw` 等固定话题。

## 方案说明

使用 ROS 标准工具 `topic_tools` 进行话题转发：
- **优点**：简单、轻量、零延迟
- **原理**：直接转发消息指针，几乎无性能损耗
- **Launch文件**：`camera_relay.launch`

---

## 使用方法

### 基本使用（默认配置）
```bash
# 默认配置：转发RGB和深度图像（不转发camera_info）
# 输入：/camera/color/image_raw, /camera/depth/image_rect_raw
# 输出：/leju_camera/color/image_raw, /leju_camera/depth/image_rect_raw
roslaunch kuavo_camera camera_relay.launch
```

### 自定义输入话题
```bash
# 从头部相机转发
roslaunch kuavo_camera camera_relay.launch \
    input_topic:=/head_camera/color/image_raw \
    input_depth_topic:=/head_camera/depth/image_rect_raw

# 从Orbbec相机转发（深度话题不带_rect）
roslaunch kuavo_camera camera_relay.launch \
    input_topic:=/camera/color/image_raw \
    input_depth_topic:=/camera/depth/image_raw
```

### 控制转发选项
```bash
# 只转发RGB图像（不转发深度）
roslaunch kuavo_camera camera_relay.launch relay_depth:=false

# 同时转发camera_info
roslaunch kuavo_camera camera_relay.launch relay_camera_info:=true

# 转发所有数据（RGB + 深度 + camera_info）
roslaunch kuavo_camera camera_relay.launch \
    relay_camera_info:=true \
    relay_depth:=true
```

### 完整示例
```bash
# 转发头部相机的所有数据到 leju_camera 命名空间
roslaunch kuavo_camera camera_relay.launch \
    input_topic:=/head_camera/color/image_raw \
    input_depth_topic:=/head_camera/depth/image_rect_raw \
    relay_camera_info:=true \
    relay_depth:=true
```

---

## 参数说明

### camera_relay.launch

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `input_topic` | `/camera/color/image_raw` | 输入RGB图像话题 |
| `input_depth_topic` | `/camera/depth/image_rect_raw` | 输入深度图像话题 |
| `output_topic` | `/leju_camera/color/image_raw` | 输出RGB图像话题（固定） |
| `relay_camera_info` | `false` | 是否转发camera_info话题 |
| `relay_depth` | `true` | 是否转发深度图像话题 |

**输出话题列表**：
- RGB图像：`/leju_camera/color/image_raw`（始终转发）
- RGB相机信息：`/leju_camera/color/camera_info`（当 `relay_camera_info=true`）
- 深度图像：`/leju_camera/depth/image_rect_raw`（当 `relay_depth=true`）
- 深度相机信息：`/leju_camera/depth/camera_info`（当 `relay_depth=true`）

---

## 验证转发是否成功

### 1. 查看话题列表
```bash
rostopic list | grep leju_camera
```

应该看到（根据配置）：
```
/leju_camera/color/image_raw                # RGB图像（始终存在）
/leju_camera/color/camera_info              # RGB相机信息（relay_camera_info=true时）
/leju_camera/depth/image_rect_raw           # 深度图像（relay_depth=true时）
/leju_camera/depth/camera_info              # 深度相机信息（relay_depth=true时）
```

### 2. 检查话题频率
```bash
# 检查RGB图像频率
rostopic hz /leju_camera/color/image_raw

# 检查深度图像频率
rostopic hz /leju_camera/depth/image_rect_raw
```

### 3. 查看图像
```bash
# 查看RGB图像
rosrun image_view image_view image:=/leju_camera/color/image_raw

# 查看深度图像
rosrun image_view image_view image:=/leju_camera/depth/image_rect_raw

# 使用 rqt_image_view（可以同时查看多个话题）
rosrun rqt_image_view rqt_image_view
```

### 4. 查看话题信息
```bash
rostopic info /leju_camera/color/image_raw
```

### 5. 查看节点状态
```bash
# 查看转发节点是否运行
rosnode list | grep relay

# 查看节点详细信息
rosnode info /camera_image_relay
rosnode info /camera_depth_relay
```

---

## 集成到现有Launch文件

如果需要在现有的launch文件中集成转发功能，可以这样做：

```xml
<launch>
    <!-- 启动原始相机 -->
    <include file="$(find realsense2_camera)/launch/rs_camera.launch" />
    
    <!-- 启动话题转发 -->
    <include file="$(find kuavo_camera)/launch/camera_relay.launch">
        <arg name="input_topic" value="/camera/color/image_raw" />
        <arg name="relay_camera_info" value="true" />
        <arg name="relay_depth" value="true" />
    </include>
    
    <!-- 其他节点... -->
</launch>
```

---

## 常见应用场景

### 场景1：默认RealSense相机转发（含深度）
```bash
# 适用于标准RealSense D435/D455相机
# 默认转发RGB和深度，不转发camera_info
roslaunch kuavo_camera camera_relay.launch
```

### 场景2：Orbbec相机转发
```bash
# Orbbec相机的深度话题不带_rect后缀
roslaunch kuavo_camera camera_relay.launch \
    input_topic:=/camera/color/image_raw \
    input_depth_topic:=/camera/depth/image_raw
```

### 场景3：头部相机转发（含相机信息）
```bash
# 转发头部相机的所有数据
roslaunch kuavo_camera camera_relay.launch \
    input_topic:=/head_camera/color/image_raw \
    input_depth_topic:=/head_camera/depth/image_rect_raw \
    relay_camera_info:=true
```

### 场景4：只转发RGB图像（不转发深度）
```bash
# 适用于只需要RGB数据的应用
roslaunch kuavo_camera camera_relay.launch \
    input_topic:=/camera/color/image_raw \
    relay_depth:=false
```

### 场景5：多相机切换
```bash
# 切换到左手腕相机
roslaunch kuavo_camera camera_relay.launch \
    input_topic:=/left_wrist_camera/color/image_raw \
    input_depth_topic:=/left_wrist_camera/depth/image_rect_raw

# 切换到右手腕相机
roslaunch kuavo_camera camera_relay.launch \
    input_topic:=/right_wrist_camera/color/image_raw \
    input_depth_topic:=/right_wrist_camera/depth/image_rect_raw
```

### 场景6：与AprilTag检测集成
```bash
# 在一个launch文件中同时启动相机、转发和AprilTag检测
```

```xml
<launch>
    <!-- 启动相机 -->
    <include file="$(find realsense2_camera)/launch/rs_camera.launch" />
    
    <!-- 转发到leju_camera命名空间 -->
    <include file="$(find kuavo_camera)/launch/camera_relay.launch">
        <arg name="input_topic" value="/camera/color/image_raw" />
        <arg name="relay_depth" value="true" />
    </include>
    
    <!-- AprilTag检测使用转发后的话题 -->
    <include file="$(find apriltag_ros)/launch/continuous_detection.launch">
        <arg name="camera_name" value="/leju_camera/color" />
        <arg name="image_topic" value="image_raw" />
    </include>
</launch>
```

---

## 故障排查

### 问题1：转发节点启动但没有数据
**检查步骤**：
1. 检查输入话题是否存在：
   ```bash
   rostopic list | grep camera
   ```

2. 检查输入话题是否有数据：
   ```bash
   rostopic hz /camera/color/image_raw
   rostopic hz /camera/depth/image_rect_raw
   ```

3. 检查话题名称是否正确（注意大小写和斜杠）：
   ```bash
   rostopic info /camera/color/image_raw
   ```

4. 检查转发节点是否正常运行：
   ```bash
   rosnode list | grep relay
   rosnode info /camera_image_relay
   ```

### 问题2：找不到 topic_tools 包
**解决方法**：
```bash
# ROS Noetic
sudo apt-get install ros-noetic-topic-tools

# ROS Melodic
sudo apt-get install ros-melodic-topic-tools

# ROS Kinetic
sudo apt-get install ros-kinetic-topic-tools
```

### 问题3：深度图像话题不匹配
**原因**：不同相机的深度话题命名不同
- RealSense: `/camera/depth/image_rect_raw`
- Orbbec: `/camera/depth/image_raw`

**解决方法**：
```bash
# 先查看实际的深度话题名称
rostopic list | grep depth

# 使用正确的话题名称
roslaunch kuavo_camera camera_relay.launch \
    input_depth_topic:=/camera/depth/image_raw
```

### 问题4：launch文件找不到
**检查方法**：
```bash
# 确认文件是否存在
ls ~/work/kuavo_ros_application/src/kuavo_camera/launch/camera_relay.launch

# 重新编译工作空间（如果需要）
cd ~/work/kuavo_ros_application
catkin_make

# 刷新环境变量
source devel/setup.bash
```

---

## 性能说明

`topic_tools relay` 的性能特点：
- **延迟**：几乎零延迟（< 0.1ms）
- **原理**：直接转发消息指针，不复制数据
- **CPU使用**：极低，几乎可忽略
- **适用场景**：所有实时性要求的应用

