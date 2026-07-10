## 机器人播放语音
## 分层结构

```text
01_use_music 语音示例
├── 示例脚本层
│   ├── playmusic.py    # 调用语音播放接口
│   └── recordmusic.py  # 调用录音接口并保存自定义音频
│
├── ROS 环境层
│   └── devel/setup.bash # 加载 kuavo_ros_application 的 ROS 工作空间环境
│
└── 机器人功能层
    ├── 播放预置语音
    └── 录制自定义语音
```
```bash
source ~/kuavo_ros_application/devel/setup.bash

# 下面语音任选一个
python3 playmusic.py Lily_Flower     # 机器人播放 茉莉花花语
python3 playmusic.py Rose_Flower     # 机器人播放 玫瑰花语
python3 playmusic.py Tulip_Flower    # 机器人播放 郁金香花语
python3 playmusic.py say_hello_kids  # 机器人播放 你好小朋友
python3 playmusic.py say_hello_lady  # 机器人播放 你好女士
python3 playmusic.py say_hello_sir   # 机器人播放 你好先生
python3 playmusic.py say_goodbye     # 机器人播放 再见
```

## 机器人录制语音
```bash
source ~/kuavo_ros_application/devel/setup.bash

python3 recordmusic.py 281 # 调用Kuavo Python API 录制音频, 同时自定义的名字为281
```
