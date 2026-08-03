#!/usr/bin/env python3
"""
CameraAdapter RGB 帧获取测试

验证 get_camera_frame("camera") 返回有效的 CameraFrame（含 RGB 图像）。
等待超时 15 秒，若超时视为失败。
"""

import sys
import os
import time
import signal
import argparse
from typing import Optional

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from adapters.hardware.leju_wheeled.camera_adapter import CameraAdapter


class CameraFrameTest:
    """RGB 帧获取测试"""

    def __init__(self):
        self.adapter: Optional[CameraAdapter] = None
        self._shutdown_flag = False
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, signum, frame):
        print("\n⚠️  收到终止信号，正在清理...")
        self._shutdown_flag = True
        if self.adapter:
            self.adapter.shutdown()

    def run(self, reuse=False):
        print("=" * 60)
        print("  CameraAdapter RGB 帧获取测试")
        print("=" * 60)
        print()

        try:
            self.adapter = CameraAdapter()
            config = {'enable_head': True, 'enable_wrist_camera': False}

            if reuse:
                import rospy
                if not rospy.core.is_initialized():
                    rospy.init_node('test_camera_frame', anonymous=True)
                print("   ♻️  重用模式：跳过 launch/TF/rviz，直接订阅话题")
                self.adapter._config = config
                self.adapter._setup_subscribers(config)
                self.adapter._is_connected = True
                self.adapter._initialized = True
            else:
                result = self.adapter.initialize(config)
                if not result.success:
                    print(f"   ❌ 初始化失败: {result.message}")
                    return False

            print("   等待相机帧数据（超时 15s）...")
            frame = None
            timeout = 15
            start_time = time.time()

            while frame is None and (time.time() - start_time) < timeout and not self._shutdown_flag:
                frame = self.adapter.get_camera_frame("camera")
                time.sleep(0.5)

            if frame is not None:
                color_shape = frame.color_image.shape if frame.color_image is not None else "None"
                depth_shape = frame.depth_image.shape if frame.depth_image is not None else "None"
                print(f"   ✅ 获取成功")
                print(f"      RGB 图像: {color_shape}")
                print(f"      深度图像: {depth_shape}")
                print(f"      时间戳: {frame.timestamp}")
                return True
            else:
                print("   ❌ 超时未获取到相机帧")
                return False

        except Exception as e:
            print(f"   ❌ 测试异常: {e}")
            return False

        finally:
            if self.adapter and not reuse:
                self.adapter.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reuse', action='store_true', help='重用已启动的相机，跳过 init/shutdown')
    args = parser.parse_args()

    test = CameraFrameTest()
    success = test.run(reuse=args.reuse)
    sys.exit(0 if success else 1)
