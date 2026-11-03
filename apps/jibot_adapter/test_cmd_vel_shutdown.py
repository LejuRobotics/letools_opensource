#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""JibotChassisMixin 速度流的 rospy shutdown 回调单测（离线，假 rospy）。

DOCS-135 H-1：Ctrl+C / 进程退出时 100Hz 速度流必须停下来。这里用注入
`sys.modules` 的假 rospy 验证三件事：

1. `start_cmd_vel_stream()` 注册了 shutdown 回调，重复 start 不重复注册；
2. 回调执行后流已停（线程 join 完成、stop 事件已置位、末帧为零速）；
3. stop 之后再触发回调是无害 no-op（rospy 没有注销 API，回调靠幂等兜底）。

按文件路径加载 mixin，绕开 `adapters/__init__.py` 的顶层 rospy 依赖，
保证本容器（无 ROS）能跑。

运行：
    python3 -m pytest apps/jibot_adapter/test_cmd_vel_shutdown.py -q
"""
import contextlib
import importlib.util
import sys
import time
import types
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

MIXIN_PATH = (ROOT / "adapters/hardware/leju_wheeled/mixins/jibot"
              / "chassis_mixin.py")
MODULE_NAME = "_jibot_chassis_mixin_under_test"
_MISSING = object()


class _Vec3:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class _Twist:
    """geometry_msgs/Twist 的最小替身。"""

    def __init__(self):
        self.linear = _Vec3()
        self.angular = _Vec3()


class _FakePublisher:
    def __init__(self, topic, msg_type, queue_size=10):
        self.topic = topic
        self.msg_type = msg_type
        self.published = []

    def publish(self, msg):
        self.published.append(msg)


class _FakeSubscriber:
    def __init__(self, *args, **kwargs):
        self.unregistered = False

    def unregister(self):
        self.unregistered = True


class _FakeRate:
    def __init__(self, hz):
        self.period = 1.0 / hz if hz else 0.01

    def sleep(self):
        time.sleep(min(self.period, 0.02))


def _build_fake_ros():
    """构造假 rospy 及两条消息包，返回 (rospy 模块, 模块名->模块)。"""
    rospy = types.ModuleType("rospy")
    rospy.shutdown_callbacks = []
    rospy.publishers = []
    rospy.subscribers = []

    rospy.on_shutdown = lambda fn: rospy.shutdown_callbacks.append(fn)

    def _publisher(topic, msg_type, queue_size=10):
        rospy.publishers.append(_FakePublisher(topic, msg_type, queue_size))
        return rospy.publishers[-1]

    def _subscriber(*args, **kwargs):
        rospy.subscribers.append(_FakeSubscriber())
        return rospy.subscribers[-1]

    rospy.Publisher = _publisher
    rospy.Subscriber = _subscriber
    rospy.Rate = _FakeRate
    rospy.get_time = time.time

    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.Twist = _Twist

    leju_pkg = types.ModuleType("leju_mobile_base_msgs")
    leju_msg = types.ModuleType("leju_mobile_base_msgs.msg")
    leju_msg.BaseCmdVelStatus = type("BaseCmdVelStatus", (), {})

    return rospy, {
        "rospy": rospy,
        "geometry_msgs": geometry_msgs,
        "geometry_msgs.msg": geometry_msgs_msg,
        "leju_mobile_base_msgs": leju_pkg,
        "leju_mobile_base_msgs.msg": leju_msg,
    }


@contextlib.contextmanager
def _fake_ros_env():
    """临时注入假 ROS 模块并在退出时还原。"""
    rospy, modules = _build_fake_ros()
    saved = {name: sys.modules.get(name, _MISSING) for name in modules}
    try:
        for name, module in modules.items():
            sys.modules[name] = module
        sys.modules.pop(MODULE_NAME, None)
        spec = importlib.util.spec_from_file_location(MODULE_NAME, MIXIN_PATH)
        mixin_module = importlib.util.module_from_spec(spec)
        sys.modules[MODULE_NAME] = mixin_module
        spec.loader.exec_module(mixin_module)
        yield rospy, mixin_module
    finally:
        sys.modules.pop(MODULE_NAME, None)
        for name, module in saved.items():
            if module is _MISSING:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = module


def _started_hw(mixin_module, rospy):
    hw = mixin_module.JibotChassisMixin()
    result = hw.start_cmd_vel_stream(topic="/test/base_cmd_vel", rate_hz=20.0)
    assert result.success, result.message
    hw.update_cmd_vel_setpoint(0.03, 0.0, 0.05)
    return hw


def test_start_registers_shutdown_hook_once():
    """start 注册一次 shutdown 回调；重复 start 不重复注册。"""
    with _fake_ros_env() as (rospy, mixin_module):
        hw = _started_hw(mixin_module, rospy)
        assert len(rospy.shutdown_callbacks) == 1, \
            f"应注册 1 个 shutdown 回调，得到 {rospy.shutdown_callbacks}"
        hw.start_cmd_vel_stream(topic="/test/base_cmd_vel", rate_hz=20.0)
        assert len(rospy.shutdown_callbacks) == 1, "重复 start 不得重复注册"
        hw.stop_cmd_vel_stream(zero_hold_s=0.0)


def test_shutdown_hook_stops_stream_synchronously():
    """回调执行后线程已 join、stop 事件置位、末帧为零速、订阅已释放。"""
    with _fake_ros_env() as (rospy, mixin_module):
        hw = _started_hw(mixin_module, rospy)
        thread = hw._cmd_vel_stream_thread
        assert thread is not None and thread.is_alive(), "流线程应已启动"

        rospy.shutdown_callbacks[0]()

        assert not thread.is_alive(), "回调必须同步 join 掉流线程"
        assert hw._cmd_vel_stream_stop.is_set(), "stop 事件应已置位"
        assert hw._cmd_vel_stream_thread is None, "线程句柄应已清理"
        assert hw._cmd_vel_status_sub is None, "状态订阅应已释放"
        assert rospy.subscribers[-1].unregistered is True, "订阅应 unregister"

        last = rospy.publishers[-1].published[-1]
        assert (last.linear.x, last.linear.y, last.angular.z) == (0.0, 0.0, 0.0), \
            "流停止前的末帧必须是零速"


def test_shutdown_hook_after_stop_is_noop():
    """显式 stop 后再触发回调：幂等，不报错也不再发包。"""
    with _fake_ros_env() as (rospy, mixin_module):
        hw = _started_hw(mixin_module, rospy)
        hw.stop_cmd_vel_stream(zero_hold_s=0.0)
        published = len(rospy.publishers[-1].published)

        rospy.shutdown_callbacks[0]()

        assert len(rospy.publishers[-1].published) == published, \
            "流已停时回调不应再发布任何帧"
