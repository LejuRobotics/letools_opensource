# -*- coding: utf-8 -*-
"""ArmControlMixin 关节设定点发布契约（无 ROS、无运动）。

守的是 DOCS-161 的两条接口约定：

1. ``publish_arm_joint_setpoint`` 是非阻塞的：Publisher 只建一次并跨帧复用，
   只发布当前时间戳的 JointState，不 sleep、不订阅到达时间；
2. ``send_arm_joint_trajectory`` 仍是原来的「下发-等到位」阻塞语义——
   码垛预备、失败/最终还原和 MPC 路径都还依赖它。

本机单元测试环境没有 ROS，因此按文件路径加载 mixin 模块（导入
``adapters.hardware.leju_wheeled.mixins`` 会连带导入整个适配器包，需要
geometry_msgs/kuavo_msgs）。rospy 与 sensor_msgs 用最小替身顶替。
"""

import importlib.util
import sys
import time
import types
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

MIXIN_PATH = REPO_ROOT / "adapters" / "hardware" / "leju_wheeled" / "mixins" / "arm_control_mixin.py"
MODULE_NAME = "arm_control_mixin_under_test"


class _FakeHeader:
    def __init__(self):
        self.stamp = None


class _FakeJointState:
    """sensor_msgs/JointState 的最小替身：只需 header / name / position。"""

    def __init__(self):
        self.header = _FakeHeader()
        self.name = []
        self.position = []


class _FakePublisher:
    def __init__(self, factory):
        self._factory = factory

    def publish(self, msg):
        if self._factory.fail:
            raise RuntimeError("publish 失败")
        self._factory.published.append(msg)


class _PublisherFactory:
    """记录 Publisher 创建次数与所有发布消息。"""

    def __init__(self, fail=False):
        self.created = []
        self.published = []
        self.fail = fail

    def __call__(self, *args, **kwargs):
        publisher = _FakePublisher(self)
        self.created.append(publisher)
        return publisher


class _SubscriberFactory:
    """记录订阅话题；可配置成订阅后立刻回调一帧。"""

    def __init__(self, immediate_data=None):
        self.subscriptions = []
        self.immediate_data = immediate_data

    def __call__(self, topic, msg_type, callback, *args, **kwargs):
        self.subscriptions.append((topic, callback))
        if self.immediate_data is not None:
            callback(types.SimpleNamespace(data=self.immediate_data))
        return object()


def _build_rospy_stub():
    rospy = types.ModuleType("rospy")
    rospy.Time = types.SimpleNamespace(now=lambda: 1234.5)
    rospy.Publisher = _PublisherFactory()
    rospy.Subscriber = _SubscriberFactory()
    rospy.sleep = lambda seconds: None
    rospy.init_node = lambda *args, **kwargs: None
    rospy.is_shutdown = lambda: False
    rospy.core = types.SimpleNamespace(is_initialized=lambda: True)
    return rospy


def _build_sensor_msgs_stub():
    msg_module = types.ModuleType("sensor_msgs.msg")
    msg_module.JointState = _FakeJointState
    package = types.ModuleType("sensor_msgs")
    package.msg = msg_module
    return package, msg_module


_ROSPY_STUB = _build_rospy_stub()
_SENSOR_MSGS_PACKAGE, _SENSOR_MSGS_MODULE = _build_sensor_msgs_stub()
_LOADED_MODULE = None


def _load_mixin_module():
    global _LOADED_MODULE
    if _LOADED_MODULE is not None:
        return _LOADED_MODULE
    spec = importlib.util.spec_from_file_location(MODULE_NAME, MIXIN_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[MODULE_NAME] = module
    spec.loader.exec_module(module)
    _LOADED_MODULE = module
    return module


@pytest.fixture
def mixin_module():
    """安装 ROS 替身并返回已加载的 mixin 模块；结束后恢复 sys.modules。"""
    names = ("rospy", "sensor_msgs", "sensor_msgs.msg")
    saved = {name: sys.modules.get(name) for name in names}
    sys.modules["rospy"] = _ROSPY_STUB
    sys.modules["sensor_msgs"] = _SENSOR_MSGS_PACKAGE
    sys.modules["sensor_msgs.msg"] = _SENSOR_MSGS_MODULE
    try:
        yield _load_mixin_module()
    finally:
        for name, previous in saved.items():
            if previous is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = previous


@pytest.fixture
def hardware(mixin_module):
    instance = mixin_module.ArmControlMixin()
    instance.state_manager = None
    return instance


@pytest.fixture
def publishers(monkeypatch):
    factory = _PublisherFactory()
    monkeypatch.setattr(_ROSPY_STUB, "Publisher", factory)
    return factory


POSITIONS = [float(index) * 10.0 for index in range(14)]


# ----------------------------------------------------------------------
# 非阻塞设定点发布
# ----------------------------------------------------------------------
@pytest.mark.unit
def test_publisher_is_created_once_and_reused_across_frames(hardware, publishers):
    hardware.publish_arm_joint_setpoint(POSITIONS)
    hardware.publish_arm_joint_setpoint(POSITIONS)

    assert len(publishers.created) == 1, "Publisher 必须跨帧复用，不能每帧新建"
    assert len(publishers.published) == 2


@pytest.mark.unit
def test_setpoint_does_not_sleep_or_subscribe(hardware, publishers, monkeypatch):
    """非阻塞：不 sleep、不订阅到达时间。"""
    subscribers = _SubscriberFactory()
    monkeypatch.setattr(_ROSPY_STUB, "Subscriber", subscribers)

    def _boom(*args, **kwargs):
        raise AssertionError("非阻塞接口不应 sleep")

    monkeypatch.setattr(time, "sleep", _boom)
    monkeypatch.setattr(_ROSPY_STUB, "sleep", _boom)

    start = time.monotonic()
    result = hardware.publish_arm_joint_setpoint(POSITIONS)
    elapsed = time.monotonic() - start

    assert result.success
    assert subscribers.subscriptions == []
    assert elapsed < 0.05


@pytest.mark.unit
def test_setpoint_publishes_14_joints_in_order_and_degrees(hardware, publishers):
    hardware.publish_arm_joint_setpoint(POSITIONS)

    message = publishers.published[0]
    assert len(message.position) == 14
    # 原值直发，不做弧度转换：位置接口的输入单位就是度。
    assert message.position == pytest.approx(POSITIONS)
    assert message.name[:7] == [
        "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw",
        "left_elbow_pitch", "left_elbow_yaw", "left_wrist_pitch", "left_wrist_roll"]
    assert message.name[7:] == [
        "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw",
        "right_elbow_pitch", "right_elbow_yaw", "right_wrist_pitch", "right_wrist_roll"]
    assert message.header.stamp == 1234.5


@pytest.mark.unit
def test_setpoint_rejects_wrong_joint_count(hardware, publishers):
    result = hardware.publish_arm_joint_setpoint([0.0] * 13)

    assert not result.success
    assert "14" in result.message
    assert publishers.published == []


@pytest.mark.unit
def test_setpoint_reports_publish_exception_as_failure(hardware, monkeypatch):
    monkeypatch.setattr(_ROSPY_STUB, "Publisher", _PublisherFactory(fail=True))

    result = hardware.publish_arm_joint_setpoint(POSITIONS)

    assert not result.success
    assert "设定点发布失败" in result.message


@pytest.mark.unit
def test_setpoint_success_only_claims_publication(hardware, publishers):
    """成功只代表 publish() 调用完成：没有到位反馈，也不写状态缓存。"""
    result = hardware.publish_arm_joint_setpoint(POSITIONS)

    assert result.success
    assert "已发布" in result.message
    assert publishers.published


# ----------------------------------------------------------------------
# 阻塞轨迹接口回归
# ----------------------------------------------------------------------
@pytest.mark.unit
def test_blocking_trajectory_still_waits_for_reach_time(hardware, publishers, monkeypatch):
    """send_arm_joint_trajectory 仍然是「下发-等到达时间-再等到位」。"""
    sleeps = []
    subscribers = _SubscriberFactory(immediate_data=1.0)

    def _fake_sleep(seconds):
        sleeps.append(seconds)

    monkeypatch.setattr(time, "sleep", _fake_sleep)
    monkeypatch.setattr(_ROSPY_STUB, "Subscriber", subscribers)

    result = hardware.send_arm_joint_trajectory(POSITIONS, time_sec=2.0)

    assert result.success
    assert [topic for topic, _ in subscribers.subscriptions] == ["/lb_arm_joint_reach_time/left"]
    assert len(publishers.created) == 1
    assert len(publishers.published) == 1
    # 0.5s 等 Publisher 建连 + reach_time(1.0s)+0.5s 等运动完成
    assert sleeps == pytest.approx([0.5, 1.5])
