#!/usr/bin/env python3
"""
CameraAdapter + PerceptionAdapter 端到端验证脚本

验证完整的相机感知链路：
- CameraAdapter 启动和话题订阅
- PerceptionAdapter 依赖注入和初始化
- AprilTag 检测功能
- 资源清理
"""

import sys
import os
import time
import signal
from typing import Optional

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from adapters.hardware.leju_wheeled.camera_adapter import CameraAdapter
from adapters.hardware.leju_wheeled.perception_adapter import PerceptionAdapter


class E2EValidator:
    """端到端验证器"""

    def __init__(self):
        self.camera_adapter: Optional[CameraAdapter] = None
        self.perception_adapter: Optional[PerceptionAdapter] = None
        self._shutdown_flag = False
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, signum, frame):
        print("\n⚠️  收到终止信号，正在清理...")
        self._shutdown_flag = True
        self.cleanup()

    def run(self):
        """运行完整验证流程"""
        print("=" * 70)
        print("  CameraAdapter + PerceptionAdapter 端到端验证")
        print("=" * 70)
        print()

        try:
            # 步骤 1: 初始化 CameraAdapter
            print("📷 步骤 1: 初始化 CameraAdapter")
            self.camera_adapter = CameraAdapter()
            camera_config = {
                'enable_head': True,
                'enable_wrist_camera': False,
            }
            result = self.camera_adapter.initialize(camera_config)
            if not result.success:
                print(f"   ❌ CameraAdapter 初始化失败: {result.message}")
                return False
            print("   ✅ CameraAdapter 初始化成功")
            print()

            # 步骤 2: 等待相机数据
            print("⏳ 步骤 2: 等待相机数据（10秒）")
            timeout = 10
            start = time.time()
            frame = None
            while (time.time() - start) < timeout and not self._shutdown_flag:
                frame = self.camera_adapter.get_camera_frame("camera")
                if frame:
                    break
                time.sleep(0.5)

            if frame:
                print("   ✅ 获取到相机帧")
            else:
                print("   ⚠️  未获取到相机帧（可能需要硬件）")
            print()

            # 步骤 3: 初始化 PerceptionAdapter（依赖注入）
            print("🔍 步骤 3: 初始化 PerceptionAdapter（依赖注入）")
            self.perception_adapter = PerceptionAdapter()
            perception_config = {
                'launch_apriltag': True
            }
            init_result = self.perception_adapter.initialize(
                camera=self.camera_adapter,
                config=perception_config
            )
            if not init_result:
                print("   ❌ PerceptionAdapter 初始化失败")
                return False
            print("   ✅ PerceptionAdapter 初始化成功")
            print()

            # 步骤 4: 验证相机数据委托
            print("🔗 步骤 4: 验证相机数据委托")
            delegated_frame = self.perception_adapter.get_camera_frame("camera")
            if delegated_frame:
                print("   ✅ 相机数据委托正常")
            else:
                print("   ⚠️  未获取到委托的相机帧")
            print()

            # 步骤 5: 测试 AprilTag 检测
            print("🏷️  步骤 5: 测试 AprilTag 检测（15秒）")
            print("   请将 AprilTag 二维码对准相机")
            timeout = 15
            start = time.time()
            tags = []
            while (time.time() - start) < timeout and not self._shutdown_flag:
                tags = self.perception_adapter.get_tag_detections()
                if tags:
                    break
                time.sleep(0.5)

            if tags:
                print(f"   ✅ 检测到 {len(tags)} 个 AprilTag")
                for tag in tags:
                    print(f"      - ID: {tag.tag_id}, 位置: ({tag.pose_in_world.x:.2f}, {tag.pose_in_world.y:.2f}, {tag.pose_in_world.z:.2f})")
            else:
                print("   ⚠️  未检测到 AprilTag（请确保二维码在视野内）")
            print()

            # 步骤 6: 获取最新感知结果
            print("📊 步骤 6: 获取最新感知结果")
            latest = self.perception_adapter.get_latest_result()
            if latest:
                print(f"   ✅ 感知结果: {'成功' if latest.success else '失败'}")
                if latest.tags:
                    print(f"      检测到 {len(latest.tags)} 个标签")
            else:
                print("   ⚠️  无感知结果")
            print()

            print("=" * 70)
            print("  ✅ 端到端验证完成")
            print("=" * 70)
            return True

        except Exception as e:
            print(f"❌ 验证过程中发生异常: {e}")
            import traceback
            traceback.print_exc()
            return False

        finally:
            self.cleanup()

    def cleanup(self):
        """清理资源"""
        print("\n🧹 清理资源...")
        if self.perception_adapter:
            self.perception_adapter.shutdown()
        if self.camera_adapter:
            self.camera_adapter.shutdown()
        print("✅ 清理完成")


if __name__ == '__main__':
    validator = E2EValidator()
    success = validator.run()
    sys.exit(0 if success else 1)
