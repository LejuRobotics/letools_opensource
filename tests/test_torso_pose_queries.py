"""躯干TF/初始目标/复位分层契约，仅Mock，不访问真机。"""

import importlib
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from py_trees.blackboard import Blackboard
from py_trees.common import Status

from core.domain.pose import Pose6D
from core.domain.result import Result
from core.interfaces.i_hardware import IHardware
from adapters.hardware.leju_wheeled.services.sdk_manager.low_level_sdk_manager import LowLevelSDKManager

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("torso_pose_query_mixin", ROOT / "adapters/hardware/leju_wheeled/mixins/torso_mixin.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
pytestmark = pytest.mark.unit
CASES = [("get_torso_pose", "GetTorsoPose", "torso_pose"),
         ("get_torso_pose_initial", "GetTorsoPoseInitial", "torso_pose_initial"),
         ("reset_torso", "ResetTorso", None)]


@pytest.fixture
def manager():
    value = LowLevelSDKManager()
    value._initialized = True
    value.robot_sdk = Mock()
    yield value
    value.shutdown()


def test_current_pose_reads_torso_relative_to_base_link(manager):
    q = Pose6D.from_euler(0, 0, 0, 30, -20, 10, degrees=True).to_quaternion()
    manager.robot_sdk.tools.get_link_pose.return_value = SimpleNamespace(position=[0.1, 0.2, 0.3], orientation=q)
    hw = MODULE.TorsoMixin()
    hw._low_level_sdk_manager = manager
    hw._angle_unit = "rad"
    result = hw.get_torso_pose()
    assert result.success
    assert result.data["pose"] == pytest.approx([0.1, 0.3, 30, -20])
    assert result.data["frame"] == "base_link"
    assert result.data["link"] == "waist_yaw_link"
    assert result.data["source"] == "tf"
    manager.robot_sdk.tools.get_link_pose.assert_called_once_with(link_name="waist_yaw_link", reference_frame="base_link")
    assert not manager.robot_sdk.control.mock_calls


@pytest.mark.parametrize("pose", [None, SimpleNamespace(position=[0, 0], orientation=[0, 0, 0, 1]),
    SimpleNamespace(position=[math.nan, 0, 0], orientation=[0, 0, 0, 1]),
    SimpleNamespace(position=[0, 0, 0], orientation=[0, 0, 0, 0])])
def test_tf_missing_or_malformed_never_falls_back_to_initial(manager, pose):
    manager.robot_sdk.tools.get_link_pose.return_value = pose
    assert not manager.get_torso_pose().success
    manager.robot_sdk.tools.get_link_pose.assert_called_once()
    assert not manager.robot_sdk.control.mock_calls


def test_initial_pose_mapping_does_not_claim_tf_frame():
    hw = MODULE.TorsoMixin()
    hw._torso_ros_manager = Mock()
    hw._torso_ros_manager.get_torso_pose_initial.return_value = Result.ok(data={
        "position": [0.1, 0.2, 0.3], "euler": [math.pi / 2, -math.pi / 4, 0.2]})
    result = hw.get_torso_pose_initial()
    assert result.success
    assert result.data == {"pose": [0.1, 0.3, 90.0, -45.0], "frame": None, "source": "initial_service"}
    hw._torso_ros_manager.get_torso_pose_initial.assert_called_once_with()
    reset = Result.ok("accepted", data={"accepted": True, "estimated_time": None})
    hw._torso_ros_manager.reset_torso.return_value = reset
    assert hw.reset_torso() is reset


@pytest.mark.parametrize("name,cls,output", CASES)
def test_mixin_missing_manager_and_failure(name, cls, output):
    hw = MODULE.TorsoMixin()
    hw._low_level_sdk_manager = None
    hw._torso_ros_manager = None
    assert not getattr(hw, name)().success
    backend = Mock()
    failure = Result.fail("unavailable")
    getattr(backend, name).return_value = failure
    hw._low_level_sdk_manager = backend
    hw._torso_ros_manager = backend
    assert getattr(hw, name)() is failure


@pytest.mark.parametrize("name,cls,output", CASES)
@pytest.mark.parametrize("fault", ["success", "failure", "exception"])
def test_skills_cache_result_and_exceptions(name, cls, output, fault):
    module = importlib.import_module("skills.atomic.refactored_sdk." + name)
    hw = Mock(spec=[name])
    method = getattr(hw, name)
    if fault == "exception":
        method.side_effect = RuntimeError("failed")
    else:
        method.return_value = Result.ok(data={"test": True}) if fault == "success" else Result.fail("refused")
    skill = getattr(module, cls + "Skill")(hw)
    params = getattr(module, cls + "Params")()
    assert skill.initialize(params).success
    result = skill.execute()
    assert result.success == (fault == "success")
    assert skill.is_finished()
    assert skill.execute() is result
    method.assert_called_once()
    assert skill.initialize(params).success
    assert not skill.is_finished()
    skill.execute()
    assert method.call_count == 2


@pytest.mark.parametrize("name,cls,output", CASES)
def test_node_caches_and_clears_outputs_and_dryrun(monkeypatch, name, cls, output):
    monkeypatch.setattr(Blackboard, "storage", {})
    monkeypatch.delenv("STUDIO_DRY_RUN", raising=False)
    module = importlib.import_module("orchestration.nodes." + name)
    hw = Mock(spec=[name])
    getattr(hw, name).return_value = Result.ok(data={"pose": [0.1, 0.3, 0, 0]})
    getter = Mock(return_value=hw)
    monkeypatch.setattr(module, "get_shared_hardware", getter)
    node = getattr(module, cls)(name, cls, "ns", {"output_key": "custom"} if output else {})
    if output: Blackboard.set("custom", "old")
    node.initialise()
    if output: assert Blackboard.get("custom") is None
    assert node.update() == Status.SUCCESS
    if output: assert Blackboard.get("custom") == {"pose": [0.1, 0.3, 0, 0]}
    assert node.update() == Status.SUCCESS
    getattr(hw, name).assert_called_once()
    getattr(hw, name).return_value = Result.fail("refused")
    node.initialise()
    assert node.update() == Status.FAILURE
    if output: assert Blackboard.get("custom") is None
    monkeypatch.setenv("STUDIO_DRY_RUN", "1")
    getter.reset_mock()
    node.initialise()
    assert node.update() == Status.SUCCESS
    if output: assert Blackboard.get("custom") is None
    getter.assert_not_called()


@pytest.mark.parametrize("name,cls,output", CASES)
def test_contract_default_and_factory(name, cls, output):
    assert name not in IHardware.__abstractmethods__
    assert not getattr(IHardware, name)(object()).success
    from orchestration.engine.behavior_tree_factory import BehaviorTreeFactory
    factory = object.__new__(BehaviorTreeFactory)
    factory.node_dir = str(ROOT / "orchestration/nodes")
    factory._build_node_index()
    assert factory._node_index[cls] == "orchestration.nodes." + name
