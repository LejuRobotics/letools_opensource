"""Real private gates with fake ROS servers; no robot commands are emitted."""
import sys
import threading
import time
from types import ModuleType, SimpleNamespace

import pytest

from core.domain.chassis_options import MoveToTargetOptions
from orchestration.nodes.grasp_mtbf_v1.jibot_io import JiBotIO

pytestmark = pytest.mark.unit


class IO(JiBotIO):
    def __init__(self):
        self._init_jibot_io()


def _request_type(*fields):
    def request(*args, **kwargs):
        return SimpleNamespace(**dict(zip(fields, args)), **kwargs)
    return request


@pytest.fixture
def ros(monkeypatch):
    service_module = ModuleType("leju_mobile_base_msgs.srv")
    for name, fields in {
        "BaseMove": ("x", "y", "theta", "options"),
        "MoveToTarget": ("x", "y", "theta", "options"),
        "CheckArrived": ("task_id", "blocking", "timeout"),
    }.items():
        setattr(service_module, name, SimpleNamespace(_request_class=_request_type(*fields)))
    messages = ModuleType("leju_mobile_base_msgs.msg")
    messages.MoveToTargetOptions = SimpleNamespace
    standard = ModuleType("std_srvs.srv")
    standard.SetBool = object()
    standard.SetBoolRequest = _request_type("data")
    for name, module in {"leju_mobile_base_msgs.srv": service_module,
                         "leju_mobile_base_msgs.msg": messages,
                         "std_srvs.srv": standard}.items():
        monkeypatch.setitem(sys.modules, name, module)
    state = SimpleNamespace(calls=[], handlers={}, discovery=lambda name, timeout: None)
    fake = ModuleType("rospy")
    fake.wait_for_service = lambda name, timeout: state.discovery(name, timeout)

    class Proxy:
        def __init__(self, name, *_args, **_kwargs):
            self.name = name

        def __call__(self, request):
            state.calls.append((self.name, request))
            handler = state.handlers.get(self.name)
            if handler:
                return handler(request)
            return SimpleNamespace(success=True, message="accepted", task_id="task-1")

        def close(self):
            pass

    fake.ServiceProxy = Proxy
    monkeypatch.setitem(sys.modules, "rospy", fake)
    return state


def test_navigation_preserves_request_options_and_task_id(ros):
    io = IO()
    options = MoveToTargetOptions(avoid_enabled=True, linear_velocity=0.12)
    result = io.base_move_to_target_jibot(1.2, -0.3, 0.4, options)
    assert result.success and result.data == {"task_id": "task-1"}
    assert [name for name, _ in ros.calls] == [
        "/move_base/enable_costmaps", "/move_base/move_to_target"]
    request = ros.calls[-1][1]
    assert (request.x, request.y, request.theta) == (1.2, -0.3, 0.4)
    assert request.options.avoid_enabled is True
    assert request.options.linear_velocity == 0.12


def test_optional_costmap_failure_does_not_prevent_navigation(ros):
    ros.handlers["/move_base/enable_costmaps"] = lambda request: SimpleNamespace(
        success=False, message="legacy service unavailable")
    assert IO().base_move_to_target_jibot(1, 2, 0).success
    assert ros.calls[-1][0] == "/move_base/move_to_target"


def test_relative_uses_base_move_and_preserves_rejection(ros):
    ros.handlers["/move_base/base_move"] = lambda request: SimpleNamespace(
        success=False, message="obstacle", task_id="rejected-1")
    result = IO().base_move_relative_jibot(-0.3, 0, 0.2)
    assert not result.success and result.data == {"task_id": "rejected-1"}
    assert "obstacle" in result.message
    assert [name for name, _ in ros.calls] == ["/move_base/base_move"]


def test_query_preserves_server_task_status(ros):
    ros.handlers["/move_base/check_arrived"] = lambda request: SimpleNamespace(
        success=False, arrived=False, status=0, message="running")
    result = IO().check_arrived_jibot("test-task", blocking=False, timeout=0)
    assert result.success
    assert result.data == {"success": False, "arrived": False, "status": 0, "message": "running"}
    request = ros.calls[-1][1]
    assert (request.task_id, request.blocking, request.timeout) == ("test-task", False, 0)


def test_cancel_during_discovery_never_dispatches_navigation_later(ros):
    discovered = threading.Event()
    release = threading.Event()
    cancel = threading.Event()
    result = []

    def discovery(name, timeout):
        discovered.set()
        release.wait(1)

    ros.discovery = discovery
    worker = threading.Thread(target=lambda: result.append(IO().base_move_relative_jibot(
        1, 0, 0, cancel_event=cancel, timeout=0.5)))
    worker.start()
    assert discovered.wait(0.3)
    cancel.set()
    worker.join(0.3)
    assert not worker.is_alive() and not result[0].success
    release.set()
    time.sleep(0.03)
    assert ros.calls == []


def test_restore_has_independent_gate_after_release_timeout(ros):
    release = threading.Event()
    entered = threading.Event()

    def control(request):
        if not request.data:
            entered.set()
            release.wait(1)
        return SimpleNamespace(success=True, message="accepted")

    ros.handlers["/enable_vel_control"] = control
    io = IO()
    try:
        assert not io.enable_vel_control_jibot(False, timeout=0.03).success
        assert entered.is_set()
        assert io.enable_vel_control_jibot(True, timeout=0.2).success
        assert not io.enable_vel_control_jibot(False, timeout=0.1).success
        assert [request.data for _, request in ros.calls] == [False, True]
    finally:
        release.set()


def test_navigation_fault_does_not_leak_into_another_scene_io(ros):
    release = threading.Event()
    count = [0]

    def navigation(request):
        count[0] += 1
        if count[0] == 1:
            release.wait(1)
        return SimpleNamespace(success=True, message="accepted", task_id="task")

    ros.handlers["/move_base/base_move"] = navigation
    first, second = IO(), IO()
    try:
        assert not first.base_move_relative_jibot(1, 0, 0, timeout=0.03).success
        assert second.base_move_relative_jibot(1, 0, 0, timeout=0.2).success
        assert not first.base_move_relative_jibot(1, 0, 0).success
        assert count[0] == 2
    finally:
        release.set()
