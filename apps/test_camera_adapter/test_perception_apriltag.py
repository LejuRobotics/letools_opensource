#!/usr/bin/env python3
"""
PerceptionAdapter AprilTag 检测 + 话题 publisher 校验

验证:
  - get_tag_detections() 惰性启动 AprilTag 节点
  - (如二维码在视野内) 检测到标签
  - /tag_detections 和 /robot_tag_info 话题有 publisher
"""

import sys
import os
import time
import signal
import subprocess
from typing import Optional

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from adapters.hardware.leju_wheeled.camera_adapter import CameraAdapter
from adapters.hardware.leju_wheeled.perception_adapter import PerceptionAdapter


class PerceptionApriltagTest:
    """AprilTag 检测 + 话题 publisher 校验"""

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

    def _check_topic_has_publisher(self, topic: str, timeout: float = 5.0):
        """检查话题是否有 publisher"""
        try:
            result = subprocess.run(
                ['rostopic', 'info', topic],
                capture_output=True, text=True, timeout=timeout
            )
            return 'Publishers:' in result.stdout and len(result.stdout.split('Publishers:')[1].strip()) > 0
        except Exception:
            return False

    def run(self):
        print("=" * 60)
        print("  PerceptionAdapter AprilTag 检测 + 话题校验")
        print("=" * 60)
        print()

        try:
            # 初始化链路
            self.camera_adapter = CameraAdapter()
            result = self.camera_adapter.initialize({
                'enable_head': True,
                'enable_wrist_camera': False,
            })
            if not result.success:
                print(f"   ❌ CameraAdapter 初始化失败: {result.message}")
                return False
            print("   ✅ CameraAdapter 初始化成功")

            self.perception_adapter = PerceptionAdapter()
            self.perception_adapter.initialize(
                camera=self.camera_adapter,
                config={'launch_apriltag': True}
            )
            print("   ✅ PerceptionAdapter 初始化成功")

            # 等待相机数据就绪
            print("   等待相机数据（10s）...")
            timeout = 10
            start = time.time()
            while (time.time() - start) < timeout and not self._shutdown_flag:
                frame = self.camera_adapter.get_camera_frame("camera")
                if frame is not None:
                    break
                time.sleep(0.5)

            # 测试 1: AprilTag 检测
            print("\n📌 测试 1: get_tag_detections()")
            print("   请将 AprilTag 二维码对准相机（15s 超时）")
            tags = None
            timeout = 15
            start = time.time()
            while not tags and (time.time() - start) < timeout and not self._shutdown_flag:
                tags = self.perception_adapter.get_tag_detections()
                time.sleep(0.5)

            if tags and len(tags) > 0:
                print(f"   ✅ 检测到 {len(tags)} 个二维码")
                for tag in tags:
                    print(f"      - ID: {tag.tag_id}, pos: ({tag.pose_in_world.x:.2f}, {tag.pose_in_world.y:.2f}, {tag.pose_in_world.z:.2f})")
            else:
                print("   ⚠️  未检测到二维码（请确保二维码在视野内）")

            # 测试 2: 话题 publisher 校验
            print("\n📌 测试 2: /tag_detections publisher 检查")
            has_pub_1 = self._check_topic_has_publisher('/tag_detections', timeout=5.0)
            if has_pub_1:
                print("   ✅ /tag_detections 有 publisher")
            else:
                print("   ❌ /tag_detections 无 publisher")
                return False

            print("\n📌 测试 3: /robot_tag_info publisher 检查")
            has_pub_2 = self._check_topic_has_publisher('/robot_tag_info', timeout=5.0)
            if has_pub_2:
                print("   ✅ /robot_tag_info 有 publisher")
            else:
                print("   ❌ /robot_tag_info 无 publisher")
                return False

            return True

        except Exception as e:
            print(f"   ❌ 测试异常: {e}")
            import traceback
            traceback.print_exc()
            return False

        finally:
            if self.perception_adapter:
                self.perception_adapter.shutdown()
            if self.camera_adapter:
                self.camera_adapter.shutdown()


if __name__ == '__main__':
    test = PerceptionApriltagTest()
    success = test.run()
    sys.exit(0 if success else 1)
