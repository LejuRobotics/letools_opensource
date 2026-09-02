# SMT 料盘搬运使用教程

本文档说明如何使用轮臂版 SMT 料盘搬运脚本 `smt_scene.py` 完成料盘抓取与箱内放置流程。

---

## (一) 文件说明

| 文件 | 适用平台 | 作用 |
| --- | --- | --- |
| `smt_scene.py` | 轮臂机器人 | SMT 料盘搬运主程序（`waist_yaw_link` 坐标系） |
| `smt_pick_sim.py` | 轮臂机器人 | 料盘抓取仿真（手工填写二维码坐标，不控制底盘） |
| `smt_place_sim.py` | 轮臂机器人 | 料盘放置仿真（手工填写二维码坐标，不控制底盘） |
| `config/smt_scene.yaml` | 轮臂机器人 | 料盘二维码与箱子二维码配置 |
| `robot/` | 轮臂机器人 | 机器人能力层（底盘、手臂、二维码、夹爪） |

**流程概述**：扫描货架大码 → 接近货架分区 → 扫描料盘小码 → 单臂抓取料盘 → 后退转身 → 扫描箱子二维码 → 接近放置位置 → 放下料盘 → 复位

---

## (二) 编译

使用前需先编译上位机代码。下位机为轮臂控制器，无需额外编译。

### 上位机编译

```bash
cd ~/kuavo_ros_application
git checkout dev
catkin build apriltag_ros   # 优先编译 apriltag_ros
catkin build                # 编译所有功能包
```

---

## (三) 场景布置

### 轮臂版场景布置

- **初始站位**：轮臂底盘正对货架分区大码，初始距离不超过 **2 米**。
- **货架布置**：货架上粘贴大码（分区定位）和小码（料盘定位）。大码用于底盘粗接近，小码用于最终抓取定位。
- **箱子位置**：箱子放置在轮臂正前方，箱体二维码朝向机器人。箱子二维码高度建议距地面约 **0.5m ~ 1.0m**，具体根据轮臂手臂工作空间调整。
- **放置位置**：放置二维码与抓取二维码大致在**同一条直线上**，轮臂抓取后执行原地转身即可面对放置二维码，场景部署如下。

![货架二维码粘贴示意图](604c4d39f294e16055e2112d6c89b9ad.png)

### 二维码粘贴说明

- 二维码（AprilTag）族群 **36h11**，尺寸 **100mm**。
- 二维码下载网站：[https://chev.me/arucogen/](https://chev.me/arucogen/)
- **大码**：粘贴在货架分区正面中央，用于底盘粗接近。
- **小码**：粘贴在料盘正面，用于最终抓取定位。
- **箱码**：粘贴在箱子正面中央，用于放置定位。
- **修改 tag 尺寸**：在上位机工作空间 `kuavo_ros_application\src\ros_vision\detection_apriltag\apriltag_ros\config\tags.yaml` 中，将各 tag 的 `size` 修改为与你打印的二维码尺寸一致，此场景中都是使用的小码0.02m，大码0.1m，按照你布置的场景修改：

```yaml
standalone_tags:
  [
    {id: 0, size: 0.02, name: 'tag_0'},
    {id: 1, size: 0.02, name: 'tag_1'},
    {id: 2, size: 0.02, name: 'tag_2'},
    {id: 3, size: 0.1, name: 'tag_3'},
    {id: 4, size: 0.1, name: 'tag_4'},
    {id: 5, size: 0.1, name: 'tag_5'},
    {id: 6, size: 0.1, name: 'tag_6'},
    {id: 7, size: 0.1, name: 'tag_7'},
    {id: 8, size: 0.1, name: 'tag_8'},
    {id: 9, size: 0.1, name: 'tag_9'}
  ]
```

### 箱子与货架要求

- **料盘**：尺寸适中，便于单臂夹爪抓取。表面平整，便于二维码粘贴和识别。
- **箱子**：推荐使用塑料箱，尺寸根据实际需求选择。箱子不宜过重，建议不超过 2kg。
- **货架**：多层货架，每层粘贴大码；料盘放置于货架上，小码朝外。

【物料表：此处留空，需列出实际使用的箱子、料盘、货架、打印二维码等物料】

### 场景布置注意事项

1. 机器人与货架之间没有障碍物，确保手臂运动空间足够。
2. 地面平整，二维码识别时避免强光直射或反光干扰。
3. 机器人的安全工作空间（`base_link` 坐标系）：
   - x 方向：0.20m ~ 0.70m（机器人胸前范围）
   - y 方向：-0.25m ~ 0.25m（左右范围）
   - z 方向：0.10m ~ 0.60m（高度范围）
   料盘抓取中心必须落在此范围内，否则程序会报错中止。

---

## (四) 配置文件 smt_scene.yaml

配置文件所有距离单位为**米（m）**，速度单位为**米/秒（m/s）**。

```yaml
# 按列表顺序依次抓取料盘。tray_qr_id 是料盘上的 2 cm 小码，
# coarse_qr_id 是该料盘所属货架分区的大码；多个料盘可共用同一个大码。
trays:
  - {tray_qr_id: 7, coarse_qr_id: 0}

# 所有料盘共用的放置箱二维码 ID；不得与任何小码或大码 ID 相同。
box_qr_id: 1
```

### 4.1 二维码配置

| 参数 | 说明 |
| --- | --- |
| `trays[].tray_qr_id` | 料盘上粘贴的小码 ID，按实际使用的 AprilTag ID 填写 |
| `trays[].coarse_qr_id` | 货架分区上粘贴的大码 ID，按实际使用的 AprilTag ID 填写 |
| `box_qr_id` | 箱子正面粘贴的二维码 ID，按实际使用的 AprilTag ID 填写 |

**注意**：
- 三个 ID 不能相同，且必须是实际使用的二维码 ID。
- `trays` 列表按顺序执行，多个料盘可共用同一个大码（同一货架分区）。
- 若同一分区内有多个料盘，程序会在同一分区连续抓取，跳过大码复扫。

---

## (五) 运行步骤

### 下位机（轮臂）

打开终端，启动轮臂控制器：

```bash
cd ~/kuavo-ros-opensource
sudo su
source devel/setup.bash
roslaunch humanoid_controllers load_kuavo_real_wheel.launch
```

再开一个终端，启动：

```bash
cd ~/kuavo-ros-opensource
sudo su
source devel/setup.bash
rosrun ar_control ar_control_node.py
```

### 上位机

启动头部传感器。上位机需要修改传感器 `apriltag.launch` 文件，如下：

```xml
<node pkg="ar_control" type="ar_control_node.py" name="ar_control_node"
      args="--target_frame base_link --source_frame head_camera_color_optical_frame"
      output="screen"
      if="false"
      />
```

同时需要确定 `apriltag.launch` 中：

```bash
<launch>
    <arg name="launch_camera" default="true" />
    <arg name="head_camera_type" default="orbbec_camera" /> <!-- orbbec_camera, rs_camera,...-->
```

然后打开终端运行：

```bash
cd ~/kuavo_ros_application
source devel/setup.bash
roslaunch dynamic_biped apriltag.launch
```

上述程序启动后，在下位机开启终端使用 `rostopic echo /robot_tag_info` 来查看二维码识别的 x, y, z 轴是否准确，如果不准则进行限位标定再进行搬运任务，否则会对效果产生影响。

### 下位机程序

机器人站稳后，另启终端运行 SMT 料盘搬运程序：

```bash
cd ~/kuavo-ros-opensource
sudo su
source devel/setup.bash
cd src/demo/SMT_5w/
python3 smt_scene.py
```

---

## (六) 常用调参方法

⚠️⚠️⚠️ **(如果默认的参数不适合当前机器，则再根据情况进行调整)**

### 6.1 手伸不到料盘或箱子

优先调整 `smt_scene.py` 中的偏移参数：

```python
PICK_OFFSET_XYZ = [0.00, 0, 0.08]    # 相对二维码的实际抓取点偏移
PLACE_OFFSET_XYZ = [0.05, 0, 0.15]   # 相对二维码的实际放置点偏移
```

- **增大 x 值**：让手更往前伸。
- **调整 z 值**：改变抓取/放置高度。

### 6.2 接近二维码后距离不合适

调整 `smt_scene.py` 中的接近距离：

```python
SHELF_COARSE_APPROACH_DISTANCE_M = 0.45   # 大码接近距离
BOX_APPROACH_DISTANCE_M = 0.55            # 箱码接近后的底盘安全距离
PICK_BACKOFF_M = 0.30                     # 抓取后后退距离
PLACE_BACKOFF_M = 0.30                    # 放置后后退距离
```

- 机器人停得太远（手够不到）→ **减小** 对应值。
- 机器人停得太近（手臂无法伸展）→ **增大** 对应值。
- 接近箱子时底盘将箱子推前 → 增大 `BOX_APPROACH_DISTANCE_M`；每次建议增加
  `0.05m`，并确认放置目标仍在机械臂可达范围内。

### 6.3 料盘抓取位置左右偏移(一般不需要修改)

调整 `smt_scene.py` 中的对齐参数：

```python
LEFT_PICK_ALIGNMENT_Y_M = 0.253    # 左臂工作线 Y 值
RIGHT_PICK_ALIGNMENT_Y_M = -0.253  # 右臂工作线 Y 值
```

若发现夹爪仍有侧向偏差，只需分别微调这两个 Y 值。

### 6.4 二维码识别不到或识别不准

1. 确认机器人初始位置距离二维码不超过 2 米。
2. 确认二维码清晰无遮挡，光照充足。
3. 确认上位机 `tags.yaml` 中的二维码 size 与实际标签尺寸一致。
4. 料盘小码识别超时时间可调整 `QR_SCAN_TIMEOUT_S = 15.0`。

---

## (七) TF 树报错解决

如果出现以下报错：

```
[WARN] [1780909261.547880]: 无法转换 AprilTag ID (1,) 的位姿: Could not find a connection between
'base_link' and 'camera_color_optical_frame' because they are not part of the same tree.
Tf has two or more unconnected trees.
```

则进入上位机对应的启动传感器的 launch 文件中，将其中的 `camera` 修改为 `head_camera_depth`：

```xml
<node pkg="tf2_ros" type="static_transform_publisher" name="camera_to_real_frame"
      args="0 0 0 0 0 0 camera head_camera_link" />
```

修改为：

```xml
<node pkg="tf2_ros" type="static_transform_publisher" name="camera_to_real_frame"
      args="0 0 0 0 0 0 head_camera_depth head_camera_link" />
```
