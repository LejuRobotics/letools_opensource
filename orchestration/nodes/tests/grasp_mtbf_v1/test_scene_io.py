"""Scene-specific control transport tested against fake ROS, without SDK edits."""
import copy
import math
import sys
import threading
import time
from types import ModuleType, SimpleNamespace
from unittest.mock import MagicMock

import pytest

from core.domain.result import Result
from orchestration.nodes.grasp_mtbf_v1.scene_io import GraspSceneIO

pytestmark = pytest.mark.unit
TIMED = "/mobile_manipulator_timed_multi_cmd"
RUCKIG = "/mobile_manipulator_set_ruckig_planner_params"


def commands():
    return [
        {"planner_index": 2, "desire_time": 1.5, "cmd_vec": [0.2, 0.5, 90, -30]},
        {"planner_index": 6, "desire_time": 2.0, "cmd_vec": [0.4, 0.3, 0.2, 180, -90, 30]},
        {"planner_index": 7, "desire_time": 2.0, "cmd_vec": [0.4, -0.3, 0.2, -180, 90, -30]},
    ]


@pytest.fixture
def ros(monkeypatch):
    state = SimpleNamespace(calls=[], handlers={}, callbacks={}, subscriptions=[],
                            discovery=lambda name, timeout: None)
    rospy = ModuleType("rospy")
    rospy.core = SimpleNamespace(is_initialized=lambda: True)
    rospy.wait_for_service = lambda name, timeout: state.discovery(name, timeout)

    class Proxy:
        def __init__(self, name, *_args, **_kwargs):
            self.name = name

        def __call__(self, request):
            state.calls.append((self.name, request))
            handler = state.handlers.get(self.name)
            if handler:
                return handler(request)
            return SimpleNamespace(isSuccess=True, actualTime=2.7, result=True,
                                   success=True, message="accepted")

        def close(self):
            pass

    def subscribe(topic, kind, callback, queue_size):
        subscriber = SimpleNamespace(topic=topic, unregister=MagicMock())
        state.subscriptions.append(subscriber)
        state.callbacks[topic] = callback
        assert queue_size == 1
        return subscriber

    rospy.ServiceProxy = Proxy
    rospy.Subscriber = subscribe
    kuavo_srv = ModuleType("kuavo_msgs.srv")
    kuavo_srv.lbMultiTimedPosCmd = object()
    kuavo_srv.lbMultiTimedPosCmdRequest = lambda: SimpleNamespace(timedCmdVec=[], isSync=False)
    kuavo_srv.setRuckigPlannerParams = object()
    kuavo_srv.setRuckigPlannerParamsRequest = SimpleNamespace
    kuavo_msg = ModuleType("kuavo_msgs.msg")
    kuavo_msg.timedSingleCmd = SimpleNamespace
    std_srv = ModuleType("std_srvs.srv")
    std_srv.SetBool = object()
    std_srv.SetBoolRequest = lambda data: SimpleNamespace(data=data)
    std_msg = ModuleType("std_msgs.msg")
    std_msg.Float64MultiArray = SimpleNamespace
    geometry = ModuleType("geometry_msgs.msg")
    geometry.Twist = SimpleNamespace
    for name, module in {"rospy": rospy, "kuavo_msgs.srv": kuavo_srv,
                         "kuavo_msgs.msg": kuavo_msg, "std_srvs.srv": std_srv,
                         "std_msgs.msg": std_msg, "geometry_msgs.msg": geometry}.items():
        monkeypatch.setitem(sys.modules, name, module)
    state.rospy = rospy
    return state


def test_planner_vectors_convert_degrees_once_without_mutating_inputs(ros):
    source = commands()
    original = copy.deepcopy(source)
    result = GraspSceneIO(MagicMock()).send_timed_multi_commands(source, is_sync=True)
    assert result.success and result.data == {"actual_time": 2.7}
    assert source == original
    request = ros.calls[0][1]
    assert request.isSync is True
    assert [item.planner_index for item in request.timedCmdVec] == [2, 6, 7]
    for item, cmd in zip(request.timedCmdVec, source):
        start = 2 if item.planner_index == 2 else 3
        assert item.cmdVec[:start] == cmd["cmd_vec"][:start]
        assert item.cmdVec[start:] == pytest.approx([
            math.radians(x) for x in cmd["cmd_vec"][start:]])
        assert item.desireTime == cmd["desire_time"]


def test_torso_convenience_method_does_not_double_convert(ros):
    result = GraspSceneIO(MagicMock()).send_torso_pose_timed(0.2, 0.6, 90, -30, 2.5)
    assert result.success
    request = ros.calls[0][1]
    assert len(request.timedCmdVec) == 1
    cmd = request.timedCmdVec[0]
    assert cmd.planner_index == 2 and cmd.desireTime == 2.5
    assert cmd.cmdVec == pytest.approx([0.2, 0.6, math.pi / 2, -math.pi / 6])


def test_controller_rejection_does_not_turn_into_success(ros):
    ros.handlers[TIMED] = lambda request: SimpleNamespace(isSuccess=False, message="unreachable")
    io = GraspSceneIO(MagicMock())
    rejected = io.send_timed_multi_commands(commands())
    assert not rejected.success and "unreachable" in rejected.message
    ros.handlers.clear()
    # A definite rejection is known, unlike an unanswered RPC.
    assert io.send_timed_multi_commands(commands()).success


@pytest.mark.parametrize("change", [
    {"planner_index": 0}, {"cmd_vec": [1, 2, 3]},
    {"cmd_vec": [0, 0, math.nan, 0]}, {"desire_time": 0},
])
def test_invalid_scene_command_does_not_reach_ros(ros, change):
    cmd = commands()[0]
    cmd.update(change)
    assert not GraspSceneIO(MagicMock()).send_timed_multi_commands([cmd]).success
    assert ros.calls == []


def test_cancel_during_discovery_never_sends_late_command(ros):
    entered, release, cancelled = threading.Event(), threading.Event(), threading.Event()
    results = []

    def discovery(name, timeout):
        entered.set()
        release.wait(1)

    ros.discovery = discovery
    io = GraspSceneIO(MagicMock())
    worker = threading.Thread(target=lambda: results.append(io.send_timed_multi_commands(
        commands(), cancel_event=cancelled)))
    worker.start()
    try:
        assert entered.wait(0.3)
        cancelled.set()
        worker.join(0.3)
        assert not worker.is_alive() and not results[0].success
        release.set()
        time.sleep(0.03)
        assert ros.calls == []
    finally:
        release.set()


def test_unknown_motion_result_latches_but_stop_remains_available(ros):
    release = threading.Event()
    io = GraspSceneIO(MagicMock())
    io.SERVICE_TIMEOUT = 0.03

    def motion(request):
        release.wait(1)
        return SimpleNamespace(isSuccess=True, actualTime=1.0)

    ros.handlers[TIMED] = motion
    try:
        assert not io.send_timed_multi_commands(commands()).success
        assert io.set_motion_control_enabled(False).success
        release.set()
        time.sleep(0.03)
        assert not io.send_timed_multi_commands(commands()).success
        assert not io.set_ruckig_planner_params(0, True, [1]*3, [2]*3, [3]*3).success
        assert [name for name, _ in ros.calls] == [TIMED, "/enable_control"]
        assert ros.calls[1][1].data is False
    finally:
        release.set()


def test_fault_is_private_to_scene_io_instance(ros):
    release = threading.Event()
    count = [0]

    def motion(request):
        count[0] += 1
        if count[0] == 1:
            release.wait(1)
        return SimpleNamespace(isSuccess=True, actualTime=1.0)

    ros.handlers[TIMED] = motion
    first, second = GraspSceneIO(MagicMock()), GraspSceneIO(MagicMock())
    first.SERVICE_TIMEOUT = 0.03
    try:
        assert not first.send_timed_multi_commands(commands()).success
        assert second.send_timed_multi_commands(commands()).success
        assert not first.send_timed_multi_commands(commands()).success
        assert count[0] == 2
    finally:
        release.set()


def test_ruckig_passes_ros_units_and_optional_limits_and_handles_rejection(ros):
    io = GraspSceneIO(MagicMock())
    result = io.set_ruckig_planner_params(2, True, [1]*4, [2]*4, [3]*4,
                                         velocity_min=[-1]*4, acceleration_min=[-2]*4)
    assert result.success
    name, request = ros.calls[-1]
    assert name == RUCKIG
    assert request.planner_index == 2 and request.is_sync is True
    assert request.velocity_max == [1]*4 and request.velocity_min == [-1]*4
    assert request.acceleration_max == [2]*4 and request.acceleration_min == [-2]*4
    assert request.jerk_max == [3]*4
    ros.handlers[RUCKIG] = lambda request: SimpleNamespace(result=False, message="bad dimensions")
    assert not io.set_ruckig_planner_params(2, True, [1], [2], [3]).success


def test_feedback_subscriptions_are_lazy_unique_and_preserve_scene_shapes(ros):
    io = GraspSceneIO(MagicMock())
    assert ros.subscriptions == []
    assert io.get_motor_error_codes() is None
    assert len(ros.subscriptions) == 3
    assert io.get_base_cmd_vel() is None and io.get_torso_target_6d() is None
    assert len(ros.subscriptions) == 3
    ros.callbacks["/sensor_data_motor/motor_error_code"](SimpleNamespace(data=[0, 2]))
    ros.callbacks["/move_base/base_cmd_vel"](SimpleNamespace(
        linear=SimpleNamespace(x=0.1, y=-0.2), angular=SimpleNamespace(z=0.3)))
    ros.callbacks["/mobile_manipulator/torso_target_6D"](
        SimpleNamespace(data=[1, 2, 3, 4, 5, 6, 7]))
    assert io.get_motor_error_codes() == [0, 2]
    assert io.get_base_cmd_vel() == {"vx": 0.1, "vy": -0.2, "wz": 0.3}
    assert io.get_torso_target_6d() == {"values": [1, 2, 3, 4, 5, 6]}
    cached = io.get_torso_target_6d()
    cached["values"][0] = 999
    assert io.get_torso_target_6d()["values"][0] == 1
    ros.callbacks["/mobile_manipulator/torso_target_6D"](SimpleNamespace(data=[1, 2]))
    assert io.get_torso_target_6d() is None


def test_feedback_partial_subscription_failure_is_cleaned_up_and_retryable(ros):
    subscribe = ros.rospy.Subscriber
    calls = [0]

    def partial(topic, kind, callback, queue_size):
        calls[0] += 1
        if calls[0] == 2:
            raise RuntimeError("subscription failed")
        return subscribe(topic, kind, callback, queue_size)

    ros.rospy.Subscriber = partial
    io = GraspSceneIO(MagicMock())
    with pytest.raises(RuntimeError, match="subscription failed"):
        io.get_motor_error_codes()
    ros.subscriptions[0].unregister.assert_called_once()
    ros.rospy.Subscriber = subscribe
    assert io.get_motor_error_codes() is None
    assert len(io._subscribers) == 3


def test_existing_hardware_operations_forward_without_replacing_methods(ros):
    hardware = MagicMock()
    method_names = ("control_end_effector", "send_world_position", "send_base_position", "send_base_velocity")
    methods = {name: getattr(hardware, name) for name in method_names}
    for method in methods.values():
        method.return_value = Result.ok("public adapter unchanged")
    io = GraspSceneIO(hardware)
    for name, args in (
        ("control_end_effector", ("left", {"position": 1})),
        ("send_world_position", (1, 2, 3)),
        ("send_base_position", (4, 5, 6)),
        ("send_base_velocity", (0.1, 0.2, 0.3)),
    ):
        assert getattr(io, name)(*args) is methods[name].return_value
        assert getattr(hardware, name) is methods[name]
        methods[name].assert_called_once_with(*args)
    assert ros.calls == [] and ros.subscriptions == []
