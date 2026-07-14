#!/usr/bin/env python
import rospy
import sys  # 导入sys模块
sys.path.append("/home/kuavo/kuavo_ros_application/src/dynamic_biped")

from kuavoRobotSDK import kuavo

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: playmusic.py <music_filename>")
        sys.exit(1)

    music_filename = sys.argv[1]  # 获取命令行参数作为音乐文件名

    kuavo_robot = kuavo("kuavo_robot_3.5_awe")

    # 以参数作为名字录制音频，同时超时时间设置为10s
    result = kuavo_robot.set_robot_record_music(music_filename, 10) 

    print(result)
