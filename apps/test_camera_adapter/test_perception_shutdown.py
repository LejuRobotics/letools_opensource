#!/usr/bin/env python3
"""
PerceptionAdapter 资源清理测试

验证:
  - shutdown() 终止 apriltag_process 和 ar_control_process 子进程
"""

import sys
import os
import signal
from typing import Optional

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from adapters.hardware.leju_wheeled.camera_adapter import CameraAdapter
from adapters.hardware.leju_wheeled.perception_adapter import PerceptionAdapter


class PerceptionShutdownTest:
    """PerceptionAdapter 资源清理测试"""

    def __init__(self):
        self.camera_adapter: Optional[CameraAdapter] = None
        self.perception_adapter: Optional[PerceptionAdapter] = None
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, signum, frame):
        print("\n⚠️  收到终止信号，正在清理...")
        if self.perception_adapter:
            self.perception_adapter.shutdown()
        if self.camera_adapter:
            self.camera_adapter.shutdown()

    def run(self):
        print("=" * 60)
        print("  PerceptionAdapter 资源清理测试")
        print("=" * 60)
        print()

        try:
            # 初始化链路
            self.camera_adapter = CameraAdapter()
            result = self.camera_adapter.initialize({
                'enable_head': True,
                'enable_wrist_camera': False
            })
            if not result.success:
                print(f"   ❌ CameraAdapter 初始化失败: {result.message}")
                return False

            self.perception_adapter = PerceptionAdapter()
            init_result = self.perception_adapter.initialize(
                camera=self.camera_adapter,
                config={'launch_apriltag': True}
            )
            if not init_result:
                print("   ❌ PerceptionAdapter 初始化失败")
                return False

            # 触发 AprilTag 节点启动（惰性启动）
            self.perception_adapter.get_tag_detections()

            print("   ✅ 初始化完成，AprilTag 节点已启动（如有 tag 在视野内）")

            # 测试: shutdown()
            print("\n📌 shutdown() 资源清理")
            self.perception_adapter.shutdown()

            # 验证子进程已终止
            apriltag_dead = (
                self.perception_adapter._ros_process is None or
                self.perception_adapter._ros_process.poll() is not None
            )
            ar_control_dead = (
                self.perception_adapter._ar_control_process is None or
                self.perception_adapter._ar_control_process.poll() is not None
            )

            print(f"   apriltag_process: {'✅ 已终止' if apriltag_dead else '❌ 仍在运行'}")
            print(f"   ar_control_process: {'✅ 已终止' if ar_control_dead else '❌ 仍在运行'}")

            if apriltag_dead and ar_control_dead:
                print("\n   ✅ 所有感知子进程已终止")
                return True
            else:
                print("\n   ❌ 部分子进程未终止")
                return False

        except Exception as e:
            print(f"   ❌ 测试异常: {e}")
            return False

        finally:
            if self.camera_adapter:
                self.camera_adapter.shutdown()


if __name__ == '__main__':
    test = PerceptionShutdownTest()
    success = test.run()
    sys.exit(0 if success else 1)
