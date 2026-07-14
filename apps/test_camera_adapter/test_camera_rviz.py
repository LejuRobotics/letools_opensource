#!/usr/bin/env python3
"""
CameraAdapter rviz 启动/不启动验证

验证:
  - rviz=false（默认）: 不启动 rviz 节点
  - rviz=true: 启动 rviz 节点
"""

import sys
import os
import signal
import argparse
import subprocess
from typing import Optional

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from adapters.hardware.leju_wheeled.camera_adapter import CameraAdapter


class CameraRvizTest:
    """rviz 启动/不启动测试"""

    def __init__(self):
        self.adapter: Optional[CameraAdapter] = None
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, signum, frame):
        print("\n⚠️  收到终止信号，正在清理...")
        if self.adapter:
            self.adapter.shutdown()

    def _has_rviz_node(self):
        """检查是否存在 rviz 节点"""
        try:
            result = subprocess.run(
                ['rosnode', 'list'],
                capture_output=True, text=True, timeout=5
            )
            nodes = result.stdout.split('\n')
            return any('rviz' in n.lower() for n in nodes)
        except Exception:
            return False

    def run(self, reuse=False):
        print("=" * 60)
        print("  CameraAdapter rviz 启动/不启动测试")
        print("=" * 60)
        print()

        all_passed = True

        # --- 测试 1: rviz=false（默认）---
        print("📌 测试 1: rviz=false（默认）")
        try:
            self.adapter = CameraAdapter()
            config = {'has_head': True, 'enable_wrist_camera': False}

            if reuse:
                import rospy
                if not rospy.core.is_initialized():
                    rospy.init_node('test_camera_rviz', anonymous=True)
                print("   ♻️  重用模式：跳过 launch/TF/rviz，直接订阅话题")
                self.adapter._config = config
                self.adapter._setup_subscribers(config)
                self.adapter._is_connected = True
                self.adapter._initialized = True
            else:
                result = self.adapter.initialize(config)
                if not result.success:
                    print(f"   ❌ 初始化失败: {result.message}")
                    all_passed = False

            if all_passed and self._has_rviz_node():
                print("   ❌ rviz=false 但 rviz 节点存在")
                all_passed = False
            elif all_passed:
                print("   ✅ 未启动 rviz")
        except Exception as e:
            print(f"   ❌ 异常: {e}")
            all_passed = False
        finally:
            if self.adapter and not reuse:
                self.adapter.shutdown()

        print()

        # --- 测试 2: rviz=true ---
        print("📌 测试 2: rviz=true")
        try:
            self.adapter = CameraAdapter()
            config = {
                'has_head': True,
                'enable_wrist_camera': False,
                'rviz': True,
            }

            if reuse:
                import rospy
                if not rospy.core.is_initialized():
                    rospy.init_node('test_camera_rviz', anonymous=True)
                print("   ♻️  重用模式：跳过 launch/TF/rviz，直接订阅话题")
                self.adapter._config = config
                self.adapter._setup_subscribers(config)
                self.adapter._is_connected = True
                self.adapter._initialized = True
            else:
                result = self.adapter.initialize(config)
                if not result.success:
                    print(f"   ❌ 初始化失败: {result.message}")
                    all_passed = False

            if all_passed:
                if self._has_rviz_node():
                    print("   ✅ rviz 已启动")
                else:
                    print("   ❌ rviz=true 但未检测到 rviz 节点")
                    all_passed = False
        except Exception as e:
            print(f"   ❌ 异常: {e}")
            all_passed = False
        finally:
            if self.adapter and not reuse:
                self.adapter.shutdown()

        return all_passed


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reuse', action='store_true', help='重用已启动的相机，跳过 init/shutdown')
    args = parser.parse_args()

    test = CameraRvizTest()
    success = test.run(reuse=args.reuse)
    sys.exit(0 if success else 1)
