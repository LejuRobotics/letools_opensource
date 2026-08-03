#!/usr/bin/env python3
"""
CameraAdapter 点云获取 + 话题数据校验

验证:
  - get_point_cloud("camera") 返回有效 PointCloudData（点数 > 0）
  - rostopic echo /camera/depth_registered/points -n1 有数据输出（防假成功）
"""

import sys
import os
import time
import signal
import argparse
import subprocess
from typing import Optional

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from adapters.hardware.leju_wheeled.camera_adapter import CameraAdapter


class CameraPointCloudTest:
    """点云获取 + 话题校验测试"""

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

    def _check_rostopic_has_data(self, topic: str, timeout: float = 5.0):
        """检查 ROS 话题是否有数据发布"""
        try:
            result = subprocess.run(
                ['rostopic', 'echo', topic, '-n', '1'],
                capture_output=True, text=True, timeout=timeout
            )
            return result.stdout.strip() != '' and 'ERROR' not in result.stdout
        except subprocess.TimeoutExpired:
            return False
        except FileNotFoundError:
            print("   ⚠️  rostopic 命令不可用")
            return None

    def run(self, reuse=False):
        print("=" * 60)
        print("  CameraAdapter 点云获取 + 话题校验")
        print("=" * 60)
        print()

        try:
            self.adapter = CameraAdapter()
            config = {'enable_head': True, 'enable_wrist_camera': False}

            if reuse:
                import rospy
                if not rospy.core.is_initialized():
                    rospy.init_node('test_camera_pointcloud', anonymous=True)
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

            # 测试 1: get_point_cloud()
            print("📌 测试 1: get_point_cloud()")
            pc_data = None
            timeout = 20
            start_time = time.time()

            while pc_data is None and (time.time() - start_time) < timeout and not self._shutdown_flag:
                pc_data = self.adapter.get_point_cloud("camera")
                time.sleep(0.5)

            if pc_data is not None and pc_data.points.size > 0:
                num_points = pc_data.points.shape[0]
                has_colors = pc_data.colors is not None
                print(f"   ✅ 获取成功: {num_points} 个点, 含颜色: {has_colors}, frame_id: {pc_data.frame_id}")
            else:
                print("   ❌ 超时或点云为空")
                return False

            # 测试 2: rostopic echo 校验
            print("\n📌 测试 2: rostopic echo /camera/depth_registered/points -n1")
            has_data = self._check_rostopic_has_data('/camera/depth_registered/points', timeout=5.0)

            if has_data is None:
                print("   ⚠️  跳过（rostopic 不可用）")
            elif has_data:
                print("   ✅ /camera/depth_registered/points 有数据")
            else:
                print("   ❌ /camera/depth_registered/points 无数据")
                return False

            return True

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

    test = CameraPointCloudTest()
    success = test.run(reuse=args.reuse)
    sys.exit(0 if success else 1)
