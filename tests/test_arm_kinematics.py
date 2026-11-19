"""新IK/FK计算接口和示例全层离线测试，无实际ROS/SDK调用。"""

import importlib
import importlib.util
from pathlib import Path
from types import SimpleNamespace, ModuleType
from unittest.mock import Mock
import sys

import pytest
from py_trees.common import Status

from core.domain.pose import Pose6D
from core.domain.result import Result
from skills.atomic.refactored_sdk.get_arm_ik_result import GetArmIkResultParams, GetArmIkResultSkill
from skills.atomic.refactored_sdk.get_arm_fk_result import GetArmFkResultParams, GetArmFkResultSkill

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("kinematics_arm_mixin", ROOT / "adapters/hardware/leju_wheeled/mixins/arm_mixin.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
# 仅作Mock输入，不是建议运动目标。
LEFT, RIGHT = [0.1, 0.2, 0.3, 10, 20, 30], [0.1, -0.2, 0.3, -10, -20, -30]
JOINT = [10.0] * 14
FK_DATA = {"left_pose": LEFT, "right_pose": RIGHT, "frame": "local", "source": "fk", "fk_frame": "test_root"}


def test_mixin_ik_converts_units_and_preserves_failure(monkeypatch):
    import math
    monkeypatch.setitem(sys.modules, "kuavo_humanoid_sdk.interfaces.data_types", SimpleNamespace(KuavoPose=SimpleNamespace))
    hw = MODULE.ArmMixin()
    hw._low_level_sdk_manager = Mock()
    hw._low_level_sdk_manager.get_arm_ik_result.return_value = Result.ok(data=[math.pi/6] * 14)
    result = hw.get_arm_ik_result(LEFT, RIGHT, ik_frame="test_root", joint=JOINT)
    assert result.success and result.data == pytest.approx([30.0] * 14)
    kwargs = hw._low_level_sdk_manager.get_arm_ik_result.call_args.kwargs
    assert kwargs["joint"] == pytest.approx([math.radians(10)] * 14)
    assert kwargs["left_pose"].orientation == pytest.approx(Pose6D.from_euler(*LEFT, degrees=True).to_quaternion())
    assert kwargs["frame"] == "local" and kwargs["ik_frame"] == "test_root"
    failure = Result.fail("no solution")
    hw._low_level_sdk_manager.get_arm_ik_result.return_value = failure
    assert hw.get_arm_ik_result(LEFT, RIGHT, ik_frame="test_root") is failure


def test_mixin_fk_converts_input_and_output():
    import math
    hw = MODULE.ArmMixin()
    hw._low_level_sdk_manager = Mock()
    hw._low_level_sdk_manager.get_arm_fk_result.return_value = Result.ok(data={
        "left_pose": LEFT[:3] + list(Pose6D.from_euler(*LEFT, degrees=True).to_quaternion()),
        "right_pose": RIGHT[:3] + list(Pose6D.from_euler(*RIGHT, degrees=True).to_quaternion()),
        "frame": "local", "source": "fk", "fk_frame": "test_root"})
    result = hw.get_arm_fk_result(JOINT, fk_frame="test_root")
    assert result.data["left_pose"] == pytest.approx(LEFT)
    assert result.data["right_pose"] == pytest.approx(RIGHT)
    hw._low_level_sdk_manager.get_arm_fk_result.assert_called_once_with(
        joint=[math.radians(10)] * 14, frame="local", fk_frame="test_root")


@pytest.mark.parametrize("cls,params,method", [
    (GetArmIkResultSkill, GetArmIkResultParams(left_pose=LEFT, right_pose=RIGHT, ik_frame="test_root"), "get_arm_ik_result"),
    (GetArmFkResultSkill, GetArmFkResultParams(joint=JOINT, fk_frame="test_root"), "get_arm_fk_result"),
])
@pytest.mark.parametrize("success", [True, False])
def test_skills_once(cls, params, method, success):
    hw = Mock()
    expected = Result.ok() if success else Result.fail("failed")
    getattr(hw, method).return_value = expected
    skill = cls(hw)
    assert skill.initialize(params).success
    assert skill.execute() is expected
    assert skill.execute() is expected
    assert skill.is_finished()
    getattr(hw, method).assert_called_once()
    assert skill.initialize(params).success and not skill.is_finished()


@pytest.mark.parametrize("kind", ["ik", "fk"])
def test_missing_model_frame_or_target_never_computes(kind):
    hw = Mock()
    if kind == "ik":
        skill = GetArmIkResultSkill(hw)
        params = GetArmIkResultParams(left_pose=LEFT, right_pose=RIGHT)
    else:
        skill = GetArmFkResultSkill(hw)
        params = GetArmFkResultParams(joint=JOINT)
    assert not skill.initialize(params).success
    assert not skill.execute().success
    assert not hw.mock_calls


@pytest.mark.parametrize("kind", ["ik", "fk"])
def test_node_success_and_dry_run(monkeypatch, kind):
    action = f"get_arm_{kind}_result"
    module = importlib.import_module(f"orchestration.nodes.{action}")
    cls = getattr(module, f"GetArm{kind.title()}Result")
    params = {"left_pose": LEFT, "right_pose": RIGHT, "ik_frame": "test_root"} if kind == "ik" else {"joint": JOINT, "fk_frame": "test_root"}
    result_data = JOINT if kind == "ik" else FK_DATA
    hw = Mock()
    getattr(hw, action).return_value = Result.ok(data=result_data)
    monkeypatch.delenv("STUDIO_DRY_RUN", raising=False)
    monkeypatch.setattr(module, "get_shared_hardware", lambda: hw)
    node = cls("kin", "kin", "ns", params)
    node.initialise()
    assert node.update() == Status.SUCCESS
    assert node.update() == Status.SUCCESS
    assert node.global_blackboard.get(f"arm_{kind}_result") == result_data
    getattr(hw, action).assert_called_once()
    monkeypatch.setenv("STUDIO_DRY_RUN", "1")
    unexpected = Mock(side_effect=AssertionError("no hardware in dry-run"))
    monkeypatch.setattr(module, "get_shared_hardware", unexpected)
    node.initialise()
    assert node.update() == Status.SUCCESS
    assert node.global_blackboard.get(f"arm_{kind}_result") is None
    invalid = cls("bad", "bad", "ns", {})
    invalid.initialise()
    assert invalid.update() == Status.FAILURE
    unexpected.assert_not_called()


@pytest.mark.parametrize("layer", ["adapters", "skills", "nodes"])
@pytest.mark.parametrize("kind", ["ik", "fk"])
def test_examples_only_compute_and_require_input(monkeypatch, layer, kind):
    action = f"get_arm_{kind}_result"
    module = importlib.import_module(f"apps.internal.{layer}.{action}")
    monkeypatch.setattr(module, "init_logging", lambda: None)
    # 无输入时不能初始化硬件，操作人员可能自行修改过示例，主动覆盖。
    if kind == "ik":
        monkeypatch.setattr(module, "LEFT_POSE", [])
        monkeypatch.setattr(module, "IK_FRAME", None)
    else:
        monkeypatch.setattr(module, "JOINT", [])
        monkeypatch.setattr(module, "FK_FRAME", None)
    assert module.main() == 1
    if kind == "ik":
        monkeypatch.setattr(module, "LEFT_POSE", LEFT)
        monkeypatch.setattr(module, "RIGHT_POSE", RIGHT)
        monkeypatch.setattr(module, "IK_FRAME", "test_root")
        monkeypatch.setattr(module, "JOINT", None)
    else:
        monkeypatch.setattr(module, "JOINT", JOINT)
        monkeypatch.setattr(module, "FK_FRAME", "test_root")
    hw = Mock()
    getattr(hw, action).return_value = Result.ok(data=JOINT if kind == "ik" else FK_DATA)
    monkeypatch.delenv("STUDIO_DRY_RUN", raising=False)
    if layer == "nodes":
        node_module = importlib.import_module(f"orchestration.nodes.{action}")
        monkeypatch.setattr(node_module, "get_shared_hardware", lambda: hw)
        assert getattr(module, f"test_{action}")()
    else:
        assert getattr(module, f"test_{action}")(hw)
    getattr(hw, action).assert_called_once()
    hw.set_arm_joint.assert_not_called()
    hw.set_arm_pose.assert_not_called()
    hw.set_arm_control_mode.assert_not_called()


@pytest.mark.parametrize("kind", ["ik", "fk"])
@pytest.mark.parametrize("layer", ["adapters", "skills"])
def test_example_main_initializes_and_releases_with_mocks(kind, layer, monkeypatch):
    action = f"get_arm_{kind}_result"
    module = importlib.import_module(f"apps.internal.{layer}.{action}")
    monkeypatch.setattr(module, "init_logging", lambda: None)
    if kind == "ik":
        for key, value in (("LEFT_POSE", LEFT), ("RIGHT_POSE", RIGHT), ("IK_FRAME", "test_root"), ("JOINT", None)):
            monkeypatch.setattr(module, key, value)
    else:
        monkeypatch.setattr(module, "JOINT", JOINT)
        monkeypatch.setattr(module, "FK_FRAME", "test_root")
    hw = Mock()
    hw.initialize.return_value = Result.ok()
    getattr(hw, action).return_value = Result.ok(data=JOINT if kind == "ik" else FK_DATA)
    fake = ModuleType("adapters.hardware.factory")
    fake.HardwareFactory = SimpleNamespace(create_hardware=Mock(return_value=hw))
    monkeypatch.setitem(sys.modules, "adapters.hardware.factory", fake)
    assert module.main() == 0
    hw.initialize.assert_called_once()
    hw.shutdown.assert_called_once()


def test_kinematics_nodes_are_discovered():
    from orchestration.engine.behavior_tree_factory import BehaviorTreeFactory
    factory = object.__new__(BehaviorTreeFactory)
    factory.node_dir = str(ROOT / "orchestration/nodes")
    factory._build_node_index()
    assert factory._node_index["GetArmIkResult"] == "orchestration.nodes.get_arm_ik_result"
    assert factory._node_index["GetArmFkResult"] == "orchestration.nodes.get_arm_fk_result"
