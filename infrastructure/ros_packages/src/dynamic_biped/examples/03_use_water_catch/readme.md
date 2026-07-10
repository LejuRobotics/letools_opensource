## watch catch 视觉抓取水
## 分层结构

```text
03_use_water_catch 视觉抓水示例
├── 启动层
│   └── vision_catch_water_demo.launch # 一键启动示例流程
│
├── 规划层
│   └── moveit_interface_plan          # MoveIt 规划器与 Python API
│
├── 感知层
│   └── 视觉检测结果                    # 提供水瓶或目标物位置
│
└── 执行层
    └── 机器人手臂抓取动作
```
* 启动指南如下：
```bash
cd ~/kuavo_ros_application/
source ~/kuavo_ros_application/devel/setup.bash
roslaunch moveit_interface_plan vision_catch_water_demo.launch # 启动moveit规划器 + moveitPythonAPI视觉抓水案例
```



