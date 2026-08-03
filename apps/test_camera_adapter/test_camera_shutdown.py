#!/usr/bin/env python3
"""
CameraAdapter 资源清理测试

验证:
  - shutdown() 后 is_connected() 返回 False
  - shutdown() 终止所有子进程（launch + TF + rviz）
"""

import sys
import os
import signal
from typing import Optional

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from adapters.hardware.leju_wheeled.camera_adapter import CameraAdapter


class CameraShutdownTest:
    """资源清理测试"""

    def __init__(self):
        self.adapter: Optional[CameraAdapter] = None
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, signum, frame):
        print("\n⚠️  收到终止信号，正在清理...")
        if self.adapter:
            self.adapter.shutdown()

    def run(self):
        print("=" * 60)
        print("  CameraAdapter 资源清理测试")
        print("=" * 60)
        print()

        all_passed = True

        try:
            # 初始化
            self.adapter = CameraAdapter()
            config = {'enable_head': True, 'enable_wrist_camera': False}
            result = self.adapter.initialize(config)

            if not result.success:
                print(f"   ❌ 初始化失败: {result.message}")
                return False

            print("   ✅ 初始化成功")

            # 测试 1: 调用 shutdown()
            print("\n📌 测试 1: shutdown() 调用")
            shutdown_result = self.adapter.shutdown()

            if shutdown_result.success:
                print(f"   ✅ shutdown 成功: {shutdown_result.message}")
            else:
                print(f"   ❌ shutdown 失败: {shutdown_result.message}")
                all_passed = False

            # 测试 2: shutdown 后 is_connected() 应为 False
            print("\n📌 测试 2: is_connected() = False")
            if not self.adapter.is_connected():
                print("   ✅ is_connected() = False")
            else:
                print("   ❌ is_connected() 仍为 True")
                all_passed = False

            # 测试 3: 子进程已终止
            print("\n📌 测试 3: 子进程已终止")
            launch_dead = self.adapter._launch_process is None
            tf_dead = len(self.adapter._tf_processes) == 0
            rviz_dead = self.adapter._rviz_process is None

            print(f"   launch_process: {'✅ None' if launch_dead else '❌ 仍存在'}")
            print(f"   tf_processes: {'✅ 空' if tf_dead else '❌ 仍存在'}")
            print(f"   rviz_process: {'✅ None' if rviz_dead else '❌ 仍存在'}")

            if not (launch_dead and tf_dead and rviz_dead):
                all_passed = False

        except Exception as e:
            print(f"   ❌ 测试异常: {e}")
            return False

        return all_passed


if __name__ == '__main__':
    test = CameraShutdownTest()
    success = test.run()
    sys.exit(0 if success else 1)
