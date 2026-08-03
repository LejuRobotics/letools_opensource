#!/usr/bin/env python3
"""
PerceptionAdapter 初始化 + 依赖注入测试

验证:
  - PerceptionAdapter.initialize(camera=camera_adapter, config=config) 返回 True
  - 依赖注入正确：_camera 属性不为 None
"""

import sys
import os
import signal
from typing import Optional

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from adapters.hardware.leju_wheeled.camera_adapter import CameraAdapter
from adapters.hardware.leju_wheeled.perception_adapter import PerceptionAdapter


class PerceptionInitTest:
    """感知适配器初始化 + 依赖注入测试"""

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
        print("  PerceptionAdapter 初始化 + 依赖注入测试")
        print("=" * 60)
        print()

        try:
            # 1. 先初始化 CameraAdapter
            print("📌 步骤 1: 初始化 CameraAdapter")
            self.camera_adapter = CameraAdapter()
            camera_config = {'enable_head': True, 'enable_wrist_camera': False}
            result = self.camera_adapter.initialize(camera_config)

            if not result.success:
                print(f"   ❌ CameraAdapter 初始化失败: {result.message}")
                return False

            print("   ✅ CameraAdapter 初始化成功")

            # 2. 依赖注入初始化 PerceptionAdapter
            print("\n📌 步骤 2: 初始化 PerceptionAdapter（依赖注入）")
            self.perception_adapter = PerceptionAdapter()
            perception_config = {'launch_apriltag': True}
            init_result = self.perception_adapter.initialize(
                camera=self.camera_adapter,
                config=perception_config
            )

            if init_result:
                print("   ✅ PerceptionAdapter 初始化成功")
            else:
                print("   ❌ PerceptionAdapter 初始化失败")
                return False

            # 3. 验证依赖注入
            print("\n📌 步骤 3: 验证依赖注入")
            if self.perception_adapter._camera is not None:
                print(f"   ✅ camera 已注入: {type(self.perception_adapter._camera).__name__}")
            else:
                print("   ❌ camera 未注入")
                return False

            if self.perception_adapter._is_initialized:
                print("   ✅ _is_initialized = True")
            else:
                print("   ❌ _is_initialized = False")
                return False

            return True

        except Exception as e:
            print(f"   ❌ 测试异常: {e}")
            return False

        finally:
            if self.perception_adapter:
                self.perception_adapter.shutdown()
            if self.camera_adapter:
                self.camera_adapter.shutdown()


if __name__ == '__main__':
    test = PerceptionInitTest()
    success = test.run()
    sys.exit(0 if success else 1)
