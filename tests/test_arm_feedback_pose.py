"""双臂关节反馈和末端设置/读取离线测试，全部Mock。"""

import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from py_trees.common import Status

from core.domain.result import Result
from core.domain.pose import Pose6D
from skills.atomic.refactored_sdk.get_arm_joint import GetArmJointParams, GetArmJointSkill
from skills.atomic.refactored_sdk.set_arm_pose import SetArmPoseParams, SetArmPoseSkill
from skills.atomic.refactored_sdk.get_arm_pose import GetArmPoseParams, GetArmPoseSkill
from orchestration.nodes import get_arm_joint, get_arm_pose, set_arm_pose

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("arm_feedback_mixin_test", ROOT / "adapters/hardware/leju_wheeled/mixins/arm_mixin.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
# 样本仅用于Mock，不是建议运动目标。
LEFT = [0.1, 0.2, 0.3, 30.0, -20.0, 10.0]
RIGHT = [0.1, -0.2, 0.3, -30.0, 20.0, -10.0]


class Hardware(MODULE.ArmMixin):
    def __init__(self):
        self._low_level_sdk_manager = Mock()
        self._low_level_sdk_manager.get_arm_joint.return_value = Result.ok(data=[math.pi / 6] * 14)
        self._low_level_sdk_manager.control_robot_end_effector_pose.return_value = Result.ok()
        self._low_level_sdk_manager.get_arm_pose.return_value = Result.ok(data={
            "left_pose": LEFT[:3] + list(Pose6D.from_euler(*LEFT, degrees=True).to_quaternion()),
            "right_pose": RIGHT[:3] + list(Pose6D.from_euler(*RIGHT, degrees=True).to_quaternion()),
            "frame": "local", "source": "tf"})


def test_joint_read_fixed_degrees():
    hardware = Hardware()
    hardware._angle_unit = "rad"
    assert hardware.get_arm_joint().data == pytest.approx([30.0] * 14)


def test_set_pose_requires_both_arms_and_converts_to_quaternion(monkeypatch):
    # 不导入SDK及ROS，只有数据类替身。
    import sys
    monkeypatch.setitem(sys.modules, "kuavo_humanoid_sdk.interfaces.data_types", SimpleNamespace(KuavoPose=SimpleNamespace))
    hardware = Hardware()
    assert hardware.set_arm_pose(LEFT, RIGHT).success
    kwargs = hardware._low_level_sdk_manager.control_robot_end_effector_pose.call_args.kwargs
    assert kwargs["frame"] == "local"
    assert kwargs["left_pose"].position == tuple(LEFT[:3])
    assert kwargs["left_pose"].orientation == pytest.approx(Pose6D.from_euler(*LEFT, degrees=True).to_quaternion())
    assert kwargs["right_pose"].orientation == pytest.approx(Pose6D.from_euler(*RIGHT, degrees=True).to_quaternion())


@pytest.mark.parametrize("left,right,frame", [(None, RIGHT, "local"), (LEFT, [], "local"),
                                                 ([math.nan] * 6, RIGHT, "local"), (LEFT, RIGHT, "typo")])
def test_invalid_pose_does_not_send(left, right, frame):
    hardware = Hardware()
    assert not hardware.set_arm_pose(left, right, frame).success
    hardware._low_level_sdk_manager.control_robot_end_effector_pose.assert_not_called()


def test_get_pose_euler_degrees_and_source_forwarding():
    hardware = Hardware()
    result = hardware.get_arm_pose()
    assert result.success
    assert result.data["left_pose"] == pytest.approx(LEFT)
    assert result.data["right_pose"] == pytest.approx(RIGHT)
    hardware._low_level_sdk_manager.get_arm_pose.assert_called_once_with(frame="local", source="tf", fk_frame=None)
    assert not hardware.get_arm_pose(source="fk").success
    assert not hardware.get_arm_pose(source="typo").success


@pytest.mark.parametrize("method,kwargs", [("get_arm_joint", {}), ("get_arm_pose", {}),
                                           ("set_arm_pose", {"left_pose": LEFT, "right_pose": RIGHT})])
def test_manager_unavailable_and_failure_propagation(method, kwargs, monkeypatch):
    import sys
    monkeypatch.setitem(sys.modules, "kuavo_humanoid_sdk.interfaces.data_types", SimpleNamespace(KuavoPose=SimpleNamespace))
    hardware = Hardware()
    sdk_method = "control_robot_end_effector_pose" if method == "set_arm_pose" else method
    failure = Result.fail("no data", error_code="SDK_FAILURE")
    getattr(hardware._low_level_sdk_manager, sdk_method).return_value = failure
    assert getattr(hardware, method)(**kwargs) is failure
    hardware._low_level_sdk_manager = None
    assert not getattr(hardware, method)(**kwargs).success


@pytest.mark.parametrize("cls,params,method", [(GetArmJointSkill, GetArmJointParams(), "get_arm_joint"),
                                              (SetArmPoseSkill, SetArmPoseParams(left_pose=LEFT, right_pose=RIGHT), "set_arm_pose"),
                                              (GetArmPoseSkill, GetArmPoseParams(), "get_arm_pose")])
@pytest.mark.parametrize("success", [True, False])
def test_skill_single_call_and_reinitialization(cls, params, method, success):
    hardware = Mock()
    expected = Result.ok() if success else Result.fail("failed")
    getattr(hardware, method).return_value = expected
    skill = cls(hardware)
    assert skill.initialize(params).success
    assert skill.execute() is expected
    assert skill.execute() is expected
    assert skill.is_finished()
    getattr(hardware, method).assert_called_once()
    assert skill.initialize(params).success
    assert not skill.is_finished()


def test_invalid_skill_params_have_no_default_motion():
    hardware = Mock()
    assert not SetArmPoseSkill(hardware).initialize(SetArmPoseParams()).success
    assert not GetArmPoseSkill(hardware).initialize(GetArmPoseParams(source="fk")).success
    assert not GetArmPoseSkill(hardware).initialize(GetArmPoseParams(frame="typo")).success
    hardware.set_arm_pose.assert_not_called()


@pytest.mark.parametrize("module,cls,params,key,method,data", [
    (get_arm_joint, get_arm_joint.GetArmJoint, {"output_key": "joint_feedback_test"}, "joint_feedback_test", "get_arm_joint", [30.0] * 14),
    (get_arm_pose, get_arm_pose.GetArmPose, {"output_key": "pose_feedback_test"}, "pose_feedback_test", "get_arm_pose", {"left_pose": LEFT, "right_pose": RIGHT, "frame": "local", "source": "tf"}),
])
@pytest.mark.parametrize("success", [True, False])
def test_getters_write_only_successful_result_and_read_once(monkeypatch, module, cls, params, key, method, data, success):
    monkeypatch.delenv("STUDIO_DRY_RUN", raising=False)
    hardware = Mock()
    getattr(hardware, method).return_value = Result.ok(data=data) if success else Result.fail("no feedback")
    monkeypatch.setattr(module, "get_shared_hardware", lambda: hardware)
    node = cls("get", "get", "ns", params)
    node.initialise()
    expected = Status.SUCCESS if success else Status.FAILURE
    assert node.update() == expected
    assert node.update() == expected
    assert node.global_blackboard.get(key) == (data if success else None)
    getattr(hardware, method).assert_called_once()


def test_getter_dry_run_does_not_fake_feedback(monkeypatch):
    monkeypatch.setenv("STUDIO_DRY_RUN", "1")
    for module, cls in ((get_arm_joint, get_arm_joint.GetArmJoint), (get_arm_pose, get_arm_pose.GetArmPose)):
        unexpected = Mock(side_effect=AssertionError("must not touch hardware"))
        monkeypatch.setattr(module, "get_shared_hardware", unexpected)
        node = cls("dry", "dry", "ns", {"output_key": "dry_feedback_test"})
        node.initialise()
        assert node.update() == Status.SUCCESS
        assert node.global_blackboard.get("dry_feedback_test") is None
        unexpected.assert_not_called()


def test_new_pose_nodes_are_discovered():
    from orchestration.engine.behavior_tree_factory import BehaviorTreeFactory
    factory = object.__new__(BehaviorTreeFactory)
    factory.node_dir = str(ROOT / "orchestration/nodes")
    factory._build_node_index()
    for name, path in (("GetArmJoint", "get_arm_joint"), ("SetArmPose", "set_arm_pose"), ("GetArmPose", "get_arm_pose")):
        assert factory._node_index[name] == f"orchestration.nodes.{path}"


def test_set_pose_node_success_and_dry_validation(monkeypatch):
    hardware = Mock()
    hardware.set_arm_pose.return_value = Result.ok()
    monkeypatch.delenv("STUDIO_DRY_RUN", raising=False)
    monkeypatch.setattr(set_arm_pose, "get_shared_hardware", lambda: hardware)
    node = set_arm_pose.SetArmPose("set", "set", "ns", {"left_pose": LEFT, "right_pose": RIGHT})
    node.initialise()
    assert node.update() == Status.SUCCESS
    assert node.update() == Status.SUCCESS
    hardware.set_arm_pose.assert_called_once_with(left_pose=LEFT, right_pose=RIGHT, frame="local")
    monkeypatch.setenv("STUDIO_DRY_RUN", "1")
    invalid = set_arm_pose.SetArmPose("invalid", "invalid", "ns", {"left_pose": LEFT})
    invalid.initialise()
    assert invalid.update() == Status.FAILURE
