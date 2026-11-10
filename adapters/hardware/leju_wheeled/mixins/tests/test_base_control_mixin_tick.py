"""BaseControlMixin.send_base_velocity_tick 单元测试（非阻塞单帧速度发布）。"""

import sys
import time
from pathlib import Path

if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).resolve().parents[5]))

import pytest
from geometry_msgs.msg import Twist

import adapters.hardware.leju_wheeled.mixins.base_control_mixin as base_control_mixin
from adapters.hardware.leju_wheeled.mixins.base_control_mixin import BaseControlMixin


class _FakePublisher:
    """最小 Publisher 替身：只记录 publish 调用，供断言发布次数与内容。"""

    def __init__(self):
        self.published = []

    def publish(self, msg):
        self.published.append(msg)


class _TickHardware(BaseControlMixin):
    """最小 mixin 实例；本方法不读 config，实例无需其他状态。"""


@pytest.fixture
def fake_pub(monkeypatch):
    """把 rospy.Publisher 换成假实现，并返回同一个假 publisher 供断言。

    注意：被测实现每次调用都新建 Publisher，这里让工厂始终返回同一对象，
    这样「调用一次 → publish 一次」的计数才有意义。
    """
    pub = _FakePublisher()
    monkeypatch.setattr(base_control_mixin.rospy, "Publisher", lambda *a, **kw: pub)
    return pub


@pytest.mark.unit
def test_single_call_publishes_once(fake_pub):
    """调一次只发布一次，且发出去的是 Twist。"""
    hardware = _TickHardware()

    result = hardware.send_base_velocity_tick(0.1, 0.0, 0.0)

    assert result.success
    assert len(fake_pub.published) == 1
    assert isinstance(fake_pub.published[0], Twist)


@pytest.mark.unit
def test_tick_does_not_block(fake_pub, monkeypatch):
    """非阻塞：不 sleep、不建 rospy.Rate，调用耗时极小。

    循环版（publish_cmd_vel duration>0）靠 rospy.Rate + sleep 节流，
    本方法一旦碰到它们就说明实现跑偏了，因此这里让两者直接抛异常。
    """
    hardware = _TickHardware()

    def _boom(*args, **kwargs):
        raise AssertionError("非阻塞方法不应调用 sleep / rospy.Rate")

    monkeypatch.setattr(base_control_mixin.time, "sleep", _boom)
    monkeypatch.setattr(base_control_mixin.rospy, "Rate", _boom)

    start = time.monotonic()
    result = hardware.send_base_velocity_tick(0.1, 0.0, 0.5)
    elapsed = time.monotonic() - start

    assert result.success
    assert elapsed < 0.05


@pytest.mark.unit
def test_vyaw_rad_is_used_as_radians(fake_pub):
    """守门测试：vyaw_rad 是 rad/s，直接写入 Twist.angular.z，不经 _to_rad。"""
    hardware = _TickHardware()

    hardware.send_base_velocity_tick(0.0, 0.0, 0.5)

    assert fake_pub.published[0].angular.z == pytest.approx(0.5)
    # 若误当度/秒转换，这里会是 deg2rad(0.5) ≈ 0.00873（差 57 倍）
    assert fake_pub.published[0].angular.z != pytest.approx(0.008726646259971648)


@pytest.mark.unit
def test_zero_velocity_still_publishes(fake_pub):
    """零速也照常发布，作为停车兜底。"""
    hardware = _TickHardware()

    result = hardware.send_base_velocity_tick(0.0, 0.0, 0.0)

    assert result.success
    assert len(fake_pub.published) == 1
    msg = fake_pub.published[0]
    assert msg.linear.x == 0.0
    assert msg.linear.y == 0.0
    assert msg.angular.z == 0.0
