"""
底盘世界坐标系控制测试

使用的 Adapter 方法: hardware.send_base_velocity_timed(frame=WORLD)
底层路径: TimedCmd → _timed_cmd_manager.send_chassis_world → planner_index=0

世界坐标系转换机制:
- send_chassis_world 接收的是世界绝对坐标 (x, y, yaw)
- 测试通过 rel_to_abs() 将用户期望的相对位移转换为绝对目标:
  abs_target = 当前追踪位置 + 相对位移
- 内部维护 PositionTracker 追踪机器人的期望世界位姿

测试用例说明:
- test_forward:  前进 0.3m    (dx=+0.3)
- test_backward: 后退 0.3m    (dx=-0.3)
- test_lateral:  左移 0.2m    (dy=+0.2)
- test_rotation: 旋转 0.3rad  (dyaw=+0.3)
"""
import sys
import time
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent.parent.parent
sys.path.insert(0, str(project_root))

from core.common.logger import init_logging, get_logger
init_logging()
from adapters.hardware.factory import HardwareFactory
from core.domain.enums import FrameType

logger = get_logger(__name__)


class PositionTracker:
    """世界坐标系下的期望位姿追踪器。

    由于 send_chassis_world 接收的是世界绝对坐标，而用户输入的是相对位移
    （如"前进 0.3m"），本追踪器负责将相对位移转换为正确的绝对世界坐标。

    用法:
        tracker = PositionTracker(x=0.0, y=0.0, yaw=0.0)
        abs_x, abs_y, abs_yaw = tracker.rel_to_abs(dx=0.3, dy=0.0, dyaw=0.0)
        hardware.send_base_velocity_timed(vx=abs_x, vy=abs_y, vyaw=abs_yaw,
                                          frame=FrameType.WORLD)
    """

    def __init__(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0):
        """初始化追踪器，设定起始位姿（世界坐标系）。

        :param x: 初始世界 x 坐标（米）
        :param y: 初始世界 y 坐标（米）
        :param yaw: 初始世界偏航角（单位由 angle_unit 配置决定）
        """
        self.x = x
        self.y = y
        self.yaw = yaw

    def rel_to_abs(self, dx: float, dy: float, dyaw: float):
        """将相对位移转换为世界绝对坐标目标。

        在当前位置的基础上累加相对位移，返回新的绝对世界坐标。
        同时更新内部追踪状态。

        :param dx: X 方向相对位移（米），正值=前进
        :param dy: Y 方向相对位移（米），正值=左移
        :param dyaw: 偏航角相对变化（用户单位），正值=逆时针
        :return: (abs_x, abs_y, abs_yaw) 世界绝对坐标目标
        """
        self.x += dx
        self.y += dy
        self.yaw += dyaw
        logger.debug(
            f"相对→绝对: dx={dx:+}, dy={dy:+}, dyaw={dyaw:+} "
            f"→ abs=({self.x:.3f}, {self.y:.3f}, {self.yaw:.3f})"
        )
        return self.x, self.y, self.yaw

    def reset(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0):
        """重置追踪器到给定的世界位姿。"""
        self.x = x
        self.y = y
        self.yaw = yaw


def test_forward(hardware, tracker):
    """前进 0.3m"""
    logger.info("=== 测试：前进 0.3m (WORLD) ===")
    abs_x, abs_y, abs_yaw = tracker.rel_to_abs(dx=0.3, dy=0.0, dyaw=0.0)
    result = hardware.send_base_velocity_timed(
        vx=abs_x, vy=abs_y, vyaw=abs_yaw, frame=FrameType.WORLD
    )
    if result.success:
        logger.info(f"✅ 前进成功，实际时间: {result.data.get('actual_time', 'N/A')}s")
    else:
        logger.error(f"❌ 前进失败: {result.message}")
    time.sleep(2.0)


def test_backward(hardware, tracker):
    """后退 0.3m"""
    logger.info("=== 测试：后退 0.3m (WORLD) ===")
    abs_x, abs_y, abs_yaw = tracker.rel_to_abs(dx=-0.3, dy=0.0, dyaw=0.0)
    result = hardware.send_base_velocity_timed(
        vx=abs_x, vy=abs_y, vyaw=abs_yaw, frame=FrameType.WORLD
    )
    if result.success:
        logger.info(f"✅ 后退成功")
    else:
        logger.error(f"❌ 后退失败: {result.message}")
    time.sleep(2.0)


def test_lateral(hardware, tracker):
    """左移 0.2m"""
    logger.info("=== 测试：左移 0.2m (WORLD) ===")
    abs_x, abs_y, abs_yaw = tracker.rel_to_abs(dx=0.0, dy=0.2, dyaw=0.0)
    result = hardware.send_base_velocity_timed(
        vx=abs_x, vy=abs_y, vyaw=abs_yaw, frame=FrameType.WORLD
    )
    if result.success:
        logger.info(f"✅ 左移成功")
    else:
        logger.error(f"❌ 左移失败: {result.message}")
    time.sleep(2.0)


def test_rotation(hardware, tracker):
    """旋转 0.3rad"""
    logger.info("=== 测试：旋转 0.3rad (WORLD) ===")
    abs_x, abs_y, abs_yaw = tracker.rel_to_abs(dx=0.0, dy=0.0, dyaw=0.3)
    result = hardware.send_base_velocity_timed(
        vx=abs_x, vy=abs_y, vyaw=abs_yaw, frame=FrameType.WORLD
    )
    if result.success:
        logger.info(f"✅ 旋转成功")
    else:
        logger.error(f"❌ 旋转失败: {result.message}")
    time.sleep(2.0)


def main():
    hardware = HardwareFactory.create_hardware(
        config={
            'robot_type': 'leju_wheeled',
            'angle_unit': 'rad',
            'sdk_managers_whitelist': ['timed'],
            'skip_end_effector': True,
            'skip_camera': True,
            'skip_state_manager': True,
            'skip_force_publishers': True,
        }
    )
    # 世界坐标系位姿追踪器：假设机器人起始于原点
    tracker = PositionTracker(x=0.0, y=0.0, yaw=0.0)

    try:
        hardware.initialize()
        # === 脚手架: 前置设置 ===
        from apps.test_kuavo_5w_sdk_adapter._scaffold import factory_setup, factory_teardown
        factory_setup(hardware, need_arm=False)

        test_forward(hardware, tracker)
        test_backward(hardware, tracker)
        test_lateral(hardware, tracker)
        test_rotation(hardware, tracker)
        logger.info("🎉 底盘世界系测试完成")

        # === 脚手架: 后置复位 ===
        factory_teardown(hardware, need_arm=False)
    finally:
        hardware.shutdown()


if __name__ == "__main__":
    main()
