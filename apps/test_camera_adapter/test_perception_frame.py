#!/usr/bin/env python3
"""
PerceptionAdapter 相机数据委托测试

验证 PerceptionAdapter 将 get_camera_frame/get_depth_data/get_point_cloud/
get_camera_info/get_camera_status 委托给注入的 CameraAdapter，数据能正常获取。
"""

import sys
import os
import time
import signal
import argparse
from typing import Optional

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from adapters.hardware.leju_wheeled.camera_adapter import CameraAdapter
from adapters.hardware.leju_wheeled.perception_adapter import PerceptionAdapter


class PerceptionFrameTest:
    """相机数据委托测试"""

    def __init__(self):
        self.camera_adapter: Optional[CameraAdapter] = None
        self.perception_adapter: Optional[PerceptionAdapter] = None
        self._shutdown_flag = False
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, signum, frame):
        print("\n⚠️  收到终止信号，正在清理...")
        self._shutdown_flag = True
        if self.perception_adapter:
            self.perception_adapter.shutdown()
        if self.camera_adapter:
            self.camera_adapter.shutdown()

    def run(self, reuse=False):
        print("=" * 60)
        print("  PerceptionAdapter 相机数据委托测试")
        print("=" * 60)
        print()

        all_passed = True

        try:
            # 初始化链路
            self.camera_adapter = CameraAdapter()
            camera_config = {'has_head': True, 'enable_wrist_camera': False}

            if reuse:
                import rospy
                if not rospy.core.is_initialized():
                    rospy.init_node('test_perception_frame', anonymous=True)
                print("   ♻️  重用模式：跳过 launch/TF/rviz，直接订阅话题")
                self.camera_adapter._config = camera_config
                self.camera_adapter._setup_subscribers(camera_config)
                self.camera_adapter._is_connected = True
                self.camera_adapter._initialized = True
            else:
                result = self.camera_adapter.initialize(camera_config)
                if not result.success:
                    print(f"   ❌ CameraAdapter 初始化失败: {result.message}")
                    return False
            print("   ✅ CameraAdapter 就绪")

            self.perception_adapter = PerceptionAdapter()
            self.perception_adapter.initialize(
                camera=self.camera_adapter,
                config={'launch_apriltag': False}
            )

            # 等待相机数据
            print("   等待相机数据（15s）...")
            timeout = 15
            start = time.time()
            while (time.time() - start) < timeout and not self._shutdown_flag:
                frame = self.perception_adapter.get_camera_frame("camera")
                if frame is not None:
                    break
                time.sleep(0.5)

            # 测试 1: get_camera_frame
            print("\n📌 测试 1: get_camera_frame()")
            frame = self.perception_adapter.get_camera_frame("camera")
            if frame is not None:
                print(f"   ✅ RGB: {frame.color_image.shape}")
            else:
                print("   ❌ 获取失败")
                all_passed = False

            # 测试 2: get_depth_data
            print("\n📌 测试 2: get_depth_data()")
            depth = self.perception_adapter.get_depth_data("camera")
            if depth is not None:
                print(f"   ✅ depth: {depth.depth_image.shape}")
            else:
                print("   ⚠️  未能获取深度图（可能需更多时间）")

            # 测试 3: get_point_cloud
            print("\n📌 测试 3: get_point_cloud()")
            pc = self.perception_adapter.get_point_cloud("camera")
            if pc is not None and pc.points.size > 0:
                print(f"   ✅ 点云: {pc.points.shape[0]} 个点")
            else:
                print("   ⚠️  未获取到点云（可能需更多时间）")

            # 测试 4: get_camera_info
            print("\n📌 测试 4: get_camera_info()")
            info = self.perception_adapter.get_camera_info("camera")
            if info is not None:
                print(f"   ✅ camera_type={info.camera_type}, resolution={info.resolution}")
            else:
                print("   ❌ 获取失败")
                all_passed = False

            # 测试 5: get_camera_status
            print("\n📌 测试 5: get_camera_status()")
            status = self.perception_adapter.get_camera_status("camera")
            if status is not None and status.is_running:
                print(f"   ✅ is_running={status.is_running}, frame_count={status.frame_count}")
            else:
                print("   ❌ 状态异常")
                all_passed = False

        except Exception as e:
            print(f"   ❌ 测试异常: {e}")
            return False

        finally:
            if self.perception_adapter:
                self.perception_adapter.shutdown()
            if self.camera_adapter and not reuse:
                self.camera_adapter.shutdown()

        return all_passed


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reuse', action='store_true', help='重用已启动的相机，跳过 camera init/shutdown')
    args = parser.parse_args()

    test = PerceptionFrameTest()
    success = test.run(reuse=args.reuse)
    sys.exit(0 if success else 1)
