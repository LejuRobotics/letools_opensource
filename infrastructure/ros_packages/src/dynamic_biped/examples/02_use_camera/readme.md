## 查看监视图像
## 分层结构

```text
02_use_camera 相机查看示例
├── 示例脚本层
│   └── camera_BGR82CV.py # 订阅相机图像并转换为 OpenCV 可显示格式
│
├── ROS 环境层
│   └── devel/setup.bash  # 加载相机话题、消息类型、依赖包环境
│
└── 视觉功能层
    └── 查看机器人相机监视图像
```
```bash
source ~/kuavo_ros_application/devel/setup.bash
python3 camera_BGR82CV.py
```



