#!/usr/bin/env python3
"""
CameraAdapter 状态/健康/性能查询测试

验证:
  - get_camera_status("camera") 返回 CameraStatus（is_running=True）
  - check_health("camera") 返回 healthy=True
  - get_performance_metrics("camera") 返回有效指标字典
"""

import sys
import os
import time
import signal
import argparse
from typing import Optional

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from adapters.hardware.leju_wheeled.camera_adapter import CameraAdapter


class CameraStatusTest:
    """状态/健康/性能查询测试"""

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
        print("  CameraAdapter 状态/健康/性能查询测试")
        print("=" * 60)
        print()

        all_passed = True

        try:
            self.adapter = CameraAdapter()
            config = {'has_head': True, 'enable_wrist_camera': False}

            if reuse:
                import rospy
                if not rospy.core.is_initialized():
                    rospy.init_node('test_camera_status', anonymous=True)
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

            # 等待几帧数据到达
            print("   等待数据...")
            timeout = 10
            start = time.time()
            while (time.time() - start) < timeout and not self._shutdown_flag:
                frame = self.adapter.get_camera_frame("camera")
                if frame is not None:
                    break
                time.sleep(0.5)

            # 测试 1: get_camera_status()
            print("\n📌 测试 1: get_camera_status()")
            status = self.adapter.get_camera_status("camera")
            if status is not None and status.is_running:
                print(f"   ✅ is_running={status.is_running}, frame_count={status.frame_count}, fps={status.fps:.1f}")
            else:
                print(f"   ❌ 状态异常: is_running={status.is_running if status else 'None'}")
                all_passed = False

            # 测试 2: check_health()
            print("\n📌 测试 2: check_health()")
            health = self.adapter.check_health("camera")
            if health['healthy']:
                print(f"   ✅ healthy={health['healthy']}, frame_count={health['frame_count']}")
            else:
                print(f"   ❌ 不健康: {health['reason']}")
                all_passed = False

            # 测试 3: get_performance_metrics()
            print("\n📌 测试 3: get_performance_metrics()")
            metrics = self.adapter.get_performance_metrics("camera")
            required_keys = ['frame_count', 'fps', 'avg_latency_ms', 'max_latency_ms', 'error_counts']
            missing = [k for k in required_keys if k not in metrics]
            if not missing:
                print(f"   ✅ frame_count={metrics['frame_count']}, fps={metrics['fps']:.1f}")
                print(f"      avg latency: {metrics['avg_latency_ms']}")
                print(f"      errors: {metrics['error_counts']}")
            else:
                print(f"   ❌ 缺少指标: {missing}")
                all_passed = False

        except Exception as e:
            print(f"   ❌ 测试异常: {e}")
            return False

        finally:
            if self.adapter and not reuse:
                self.adapter.shutdown()

        return all_passed


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reuse', action='store_true', help='重用已启动的相机，跳过 init/shutdown')
    args = parser.parse_args()

    test = CameraStatusTest()
    success = test.run(reuse=args.reuse)
    sys.exit(0 if success else 1)
