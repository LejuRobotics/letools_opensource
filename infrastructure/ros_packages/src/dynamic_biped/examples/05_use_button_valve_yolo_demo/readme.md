## 关于此demo的阀门按钮识别，请查阅基础手册`/home/kuavo/kuavo_ros_application/docs/How_to_use_demo/05_阀门按钮识别使用手册.md`
## 分层结构

```text
05_use_button_valve_yolo_demo 阀门/按钮识别示例
├── 启动层
│   ├── one_start_valve_button_yolo.launch # 同时启动阀门识别和按钮识别
│   ├── yolo_button_object_detection        # 按钮检测节点
│   └── yolo_valve_object_detection         # 阀门检测节点
│
├── 感知层
│   └── YOLO/分割检测                       # 输出阀门、按钮等目标识别结果
│
└── 任务层
    └── 比赛/演示任务中的视觉感知输入
```
### 以下为启动指南
### 针对于2024人工智能打机器人比赛 -- 比赛任务（视觉感知阀门及按钮识别使用）
### 关于启动，一键使用launch启动
```bash
cd ~/kuavo_ros_application # 去到对应的目录

catkin build # 编译

roslaunch yolo_button_object_detection one_start_valve_button_yolo.launch  # 一键启动阀门识别+按钮识别的launch

roslaunch yolo_button_object_detection yolo_segment_detect.launch # 只启动按钮识别的launch
roslaunch yolo_valve_object_detection yolo_segment_detect.launch  # 只启动阀门识别的launch 
```
