#!/usr/bin/env python3
"""
CameraAdapter 初始化测试

验证 CameraAdapter.initialize() 返回 Result(success=True)，
相机 launch 成功启动。
"""

import sys
import os
import time
import signal
import argparse
from pathlib import Path
from typing import Optional

import rospy

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from adapters.hardware.leju_wheeled.camera_adapter import CameraAdapter
from adapters.hardware.leju_wheeled.mixins._logging_setup import (
    reconfigure_logging_after_rospy_init,
)
from core.common.config_loader import ConfigLoader


DEFAULT_CONFIG = Path(__file__).resolve().parents[2] / 'config' / 'camera_config.yaml'


class CameraInitTest:
    """CameraAdapter 初始化测试"""

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

    def run(self, config_path: str, keep_alive=False, rviz=False):
        print("=" * 60)
        print("  CameraAdapter 初始化测试")
        print("=" * 60)
        print()

        try:
            data = ConfigLoader().load(config_path)
            if not isinstance(data, dict) or not isinstance(data.get('camera'), dict):
                raise ValueError("配置文件必须包含 camera 映射")

            config = dict(data['camera'])
            if rviz:
                config['rviz'] = True

            print(f"   配置文件: {Path(config_path).resolve()}")
            print(f"   头部相机: {'启用' if config.get('enable_head', True) else '关闭'}")
            print(f"   腕部相机: {'启用' if config.get('enable_wrist_camera', False) else '关闭'}")

            # CameraAdapter 只负责订阅，不负责创建进程级 ROS 节点。
            # 未 init_node 时 Subscriber 对象虽能构造，却不会向 ROS master
            # 正常注册，按需启流的相机驱动因看不到订阅者而永远不发布首帧。
            if not rospy.core.is_initialized():
                rospy.init_node('test_camera_init', anonymous=True)
                reconfigure_logging_after_rospy_init()

            self.adapter = CameraAdapter()
            result = self.adapter.initialize(config)

            if result.success:
                msg_parts = [f"消息: {result.message}"]
                if config.get('rviz', False):
                    msg_parts.append("rviz: 已启动")
                print("   ✅ 初始化成功")
                for m in msg_parts:
                    print(f"   {m}")
                if keep_alive:
                    print()
                    print("   🔄 保持相机运行中，按 Ctrl+C 退出...")
                    try:
                        while not self._shutdown_flag:
                            time.sleep(1)
                    except KeyboardInterrupt:
                        print("\n   收到中断信号")
                return True
            else:
                print(f"   ❌ 初始化失败: {result.message}")
                return False

        except Exception as e:
            print(f"   ❌ 初始化异常: {e}")
            return False

        finally:
            if self.adapter and not keep_alive:
                self.adapter.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=str(DEFAULT_CONFIG), help='相机 YAML 配置文件')
    parser.add_argument('--keep-alive', action='store_true', help='保持相机运行不退出，供 --reuse 脚本使用')
    parser.add_argument('--rviz', action='store_true', help='覆盖配置并启动 rviz 可视化（使用 biped_s4_head.rviz 配置）')
    args = parser.parse_args()

    test = CameraInitTest()
    success = test.run(config_path=args.config, keep_alive=args.keep_alive, rviz=args.rviz)
    sys.exit(0 if success else 1)
