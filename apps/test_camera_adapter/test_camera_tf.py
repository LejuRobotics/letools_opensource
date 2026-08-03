#!/usr/bin/env python3
"""
CameraAdapter TF 静态变换检查

验证 initialize() 后 2 个 static_transform_publisher 节点存在：
  - head_camera_depth → camera_link
  - camera → head_camera_link

下位机 URDF + orbbec_sensor_robot_enable.launch 提供其余 TF。
"""

import sys
import os
import signal
import argparse
import subprocess
from typing import Optional

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from adapters.hardware.leju_wheeled.camera_adapter import CameraAdapter


class CameraTFTest:
    """TF 静态变换检查"""

    def __init__(self):
        self.adapter: Optional[CameraAdapter] = None
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, signum, frame):
        print("\n⚠️  收到终止信号，正在清理...")
        if self.adapter:
            self.adapter.shutdown()

    def _get_tf_nodes(self):
        """获取当前运行的 static_transform_publisher 节点列表"""
        try:
            result = subprocess.run(
                ['rosnode', 'list'],
                capture_output=True, text=True, timeout=5
            )
            nodes = [n.strip() for n in result.stdout.split('\n') if n.strip()]
            return [n for n in nodes if 'static_transform_publisher' in n]
        except Exception:
            return []

    def run(self, reuse=False):
        print("=" * 60)
        print("  CameraAdapter TF 静态变换检查")
        print("=" * 60)
        print()

        try:
            self.adapter = CameraAdapter()
            config = {'enable_head': True, 'enable_wrist_camera': False}

            if reuse:
                import rospy
                if not rospy.core.is_initialized():
                    rospy.init_node('test_camera_tf', anonymous=True)
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

            # 查找 TF 节点
            tf_nodes = self._get_tf_nodes()
            print(f"   发现 {len(tf_nodes)} 个 static_transform_publisher 节点")

            if len(tf_nodes) >= 3:
                for node in tf_nodes:
                    print(f"   ✅ 节点: {node}")
                return True
            else:
                print(f"   ❌ 期望 ≥2 个 TF 节点，实际 {len(tf_nodes)} 个")
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

    test = CameraTFTest()
    success = test.run(reuse=args.reuse)
    sys.exit(0 if success else 1)
