"""MPC六服务离线测试：全部Mock，不连接ROS或SDK。"""

import importlib
import importlib.util
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from py_trees.blackboard import Blackboard
from py_trees.common import Status

from core.domain.result import Result
from core.domain.mpc import MPC_VALUES
from core.interfaces.i_hardware import IHardware
from adapters.hardware.leju_wheeled.services.ros_manager import mpc_ros_manager as ros

ROOT = Path(__file__).resolve().parents[1]
CASES = [(action, suffix, kind) for suffix, kind in (("control_mode", "mode"), ("control_flow", "flow"), ("frame", "frame")) for action in ("set", "get")]
ENDPOINTS = {
    "mode": ("/mobile_manipulator_mpc_control", "/mobile_manipulator_get_mpc_control_mode", "control_mode", "mode"),
    "flow": ("/enable_mm_wbc_arm_trajectory_control", "/get_mm_wbc_arm_trajectory_control", "control_mode", "mode"),
    "frame": ("/set_mm_ctrl_frame", "/get_mm_ctrl_frame", "frame", "currentFrame"),
}
pytestmark = pytest.mark.unit


@pytest.fixture
def backend(monkeypatch):
    monkeypatch.setattr(ros.rospy.core, "is_initialized", lambda: True)
    wait = Mock()
    client = Mock(return_value=SimpleNamespace(result=True, mode=1, currentFrame=1, message="accepted"))
    factory = Mock(return_value=client)
    monkeypatch.setattr(ros.rospy, "wait_for_service", wait)
    monkeypatch.setattr(ros.rospy, "ServiceProxy", factory)
    monkeypatch.setattr(ros.rospy, "init_node", Mock(side_effect=AssertionError("不能init ROS")))
    manager = ros.MpcRosManager()
    assert manager.initialize().success
    assert not wait.called and not factory.called
    return manager, wait, client


@pytest.mark.parametrize("action,suffix,kind", CASES)
def test_protocol_single_service_and_fields(backend, action, suffix, kind):
    manager, wait, client = backend
    name = f"{action}_mpc_{suffix}"
    result = getattr(manager, name)(1) if action == "set" else getattr(manager, name)()
    assert result.success and result.message == "accepted"
    assert result.data == (None if action == "set" else {kind: 1, "name": MPC_VALUES[kind][1]})
    set_url, get_url, request_field, _ = ENDPOINTS[kind]
    wait.assert_called_once_with(set_url if action == "set" else get_url, timeout=5.0)
    client.assert_called_once()
    assert getattr(client.call_args.args[0], request_field) == (1 if action == "set" else 0)


@pytest.mark.parametrize("suffix,kind", [("control_mode", "mode"), ("control_flow", "flow"), ("frame", "frame")])
@pytest.mark.parametrize("bad", [None, True, 1.0, "1", -1, 99])
def test_invalid_setter_rejected_before_service(backend, suffix, kind, bad):
    manager, wait, client = backend
    assert not getattr(manager, "set_mpc_" + suffix)(bad).success
    wait.assert_not_called()
    client.assert_not_called()


@pytest.mark.parametrize("action,suffix,kind", CASES)
@pytest.mark.parametrize("fault", ["refused", "exception", "timeout", "uninitialized"])
def test_failure_no_retry_or_cached_fallback(backend, action, suffix, kind, fault):
    manager, wait, client = backend
    if fault == "refused": client.return_value.result = False
    elif fault == "exception": client.side_effect = ros.rospy.ServiceException("lost")
    elif fault == "timeout": wait.side_effect = ros.rospy.ROSException("timeout")
    else: manager.shutdown()
    result = getattr(manager, f"{action}_mpc_{suffix}")(1) if action == "set" else getattr(manager, f"{action}_mpc_{suffix}")()
    assert not result.success
    assert client.call_count == (1 if fault in ("refused", "exception") else 0)


@pytest.mark.parametrize("suffix,kind", [("control_mode", "mode"), ("control_flow", "flow"), ("frame", "frame")])
@pytest.mark.parametrize("value", [-1, 99, True, 1.0, None])
def test_invalid_query_response_fails(backend, suffix, kind, value):
    manager, _, client = backend
    setattr(client.return_value, ENDPOINTS[kind][3], value)
    assert not getattr(manager, "get_mpc_" + suffix)().success


@pytest.mark.parametrize("action,suffix,kind", CASES)
@pytest.mark.parametrize("failure", [False, True])
def test_mixin_and_skill_cache_result(action, suffix, kind, failure):
    name = f"{action}_mpc_{suffix}"
    cls = "".join(s.title() for s in name.split("_"))
    spec = importlib.util.spec_from_file_location("mpc_mixin_test", ROOT / "adapters/hardware/leju_wheeled/mixins/mpc_mixin.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    hw = mod.MpcMixin()
    hw._mpc_ros_manager = Mock(spec=[name])
    expected = Result.fail("refused") if failure else Result.ok(data=None if action == "set" else {kind: 1, "name": MPC_VALUES[kind][1]})
    getattr(hw._mpc_ros_manager, name).return_value = expected
    module = importlib.import_module("skills.atomic.refactored_sdk." + name)
    skill = getattr(module, cls + "Skill")(hw)
    params = getattr(module, cls + "Params")(**{kind: 1} if action == "set" else {})
    assert skill.initialize(params).success
    assert skill.execute() is expected
    assert skill.execute() is expected
    assert skill.is_finished()
    getattr(hw._mpc_ros_manager, name).assert_called_once()
    assert skill.initialize(params).success
    assert not skill.is_finished()


@pytest.mark.parametrize("action,suffix,kind", CASES)
def test_node_blackboard_dryrun_and_failure(action, suffix, kind, monkeypatch):
    monkeypatch.setattr(Blackboard, "storage", {})
    monkeypatch.delenv("STUDIO_DRY_RUN", raising=False)
    name = f"{action}_mpc_{suffix}"
    cls = "".join(s.title() for s in name.split("_"))
    mod = importlib.import_module("orchestration.nodes." + name)
    hw = Mock(spec=[name])
    data = {kind: 1, "name": MPC_VALUES[kind][1]}
    getattr(hw, name).return_value = Result.ok(data=None if action == "set" else data)
    getter = Mock(return_value=hw)
    monkeypatch.setattr(mod, "get_shared_hardware", getter)
    params = {kind + "__board_key": "target"} if action == "set" else {"output_key": "out"}
    Blackboard.set("target", "1")
    Blackboard.set("out", "old")
    node = getattr(mod, cls)(name, name, "ns", params)
    node.initialise()
    if action == "get": assert Blackboard.get("out") is None
    assert node.update() == Status.SUCCESS
    assert node.update() == Status.SUCCESS
    getattr(hw, name).assert_called_once()
    if action == "get": assert Blackboard.get("out") == data
    getattr(hw, name).return_value = Result.fail("unavailable")
    node.initialise()
    assert node.update() == Status.FAILURE
    if action == "get": assert Blackboard.get("out") is None
    monkeypatch.setenv("STUDIO_DRY_RUN", "1")
    getter.reset_mock()
    node.initialise()
    assert node.update() == Status.SUCCESS
    getter.assert_not_called()
    if action == "get": assert Blackboard.get("out") is None
    else:
        Blackboard.set("target", True)
        node.initialise()
        assert node.update() == Status.FAILURE


@pytest.mark.parametrize("action,suffix,kind", CASES)
def test_contract_and_discovery(action, suffix, kind):
    name = f"{action}_mpc_{suffix}"
    assert name not in IHardware.__abstractmethods__
    result = getattr(IHardware, name)(object(), 1) if action == "set" else getattr(IHardware, name)(object())
    assert not result.success
    from orchestration.engine.behavior_tree_factory import BehaviorTreeFactory
    factory = object.__new__(BehaviorTreeFactory)
    factory.node_dir = str(ROOT / "orchestration/nodes")
    factory._build_node_index()
    cls = "".join(s.title() for s in name.split("_"))
    assert factory._node_index[cls] == "orchestration.nodes." + name
