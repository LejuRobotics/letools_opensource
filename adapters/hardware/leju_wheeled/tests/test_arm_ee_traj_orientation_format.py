# -*- coding: utf-8 -*-
"""末端轨迹节点 orientation_format 单元测试。

覆盖反馈 recvxqzQVFYXI1：arm_ee_traj_{local,world}_sdk_move 只接受
[x,y,z,qx,qy,qz,qw]，期望能直接给 XYZ + 欧拉角。

默认 orientation_format='quat' 必须与改动前完全一致；'euler' 时按
[x,y,z,yaw,pitch,roll]（米, 度）在节点内转四元数后仍下发 7 维。
"""

import math
from unittest.mock import MagicMock, patch

import pytest
from py_trees.common import Status

from orchestration.nodes.arm_ee_traj_local_sdk_move import ArmEeTrajLocalSdkMove
from orchestration.nodes.arm_ee_traj_world_sdk_move import ArmEeTrajWorldSdkMove
from orchestration.utils.pose_conversion import (
    ORIENTATION_EULER,
    ORIENTATION_QUAT,
    convert_traj_orientation,
)

pytestmark = pytest.mark.unit

_QUAT_POINT = [0.3, 0.25, 0.5, 0.0, 0.0, 0.0, 1.0]

# (节点类, 该节点模块路径) —— 两个节点各自 import get_shared_hardware，需分别打桩
_NODES = [
    (ArmEeTrajLocalSdkMove, "orchestration.nodes.arm_ee_traj_local_sdk_move"),
    (ArmEeTrajWorldSdkMove, "orchestration.nodes.arm_ee_traj_world_sdk_move"),
]
_NODE_IDS = ["local", "world"]


# ---------- 第一层：转换函数 ----------

class TestConvertTrajOrientation:
    """convert_traj_orientation 纯函数行为。"""

    def test_quat_format_passes_points_through(self):
        traj = [_QUAT_POINT, [0.5, 0.25, 0.5, 0.0, 0.0, 0.0, 1.0]]
        assert convert_traj_orientation(traj, ORIENTATION_QUAT) == traj

    def test_quat_format_does_not_alias_input(self):
        """返回新列表，避免调用方后续改动污染入参。"""
        traj = [[0.3, 0.25, 0.5, 0.0, 0.0, 0.0, 1.0]]
        converted = convert_traj_orientation(traj, ORIENTATION_QUAT)

        assert converted == traj
        assert converted is not traj
        assert converted[0] is not traj[0]

    def test_euler_identity_yields_unit_quaternion(self):
        converted = convert_traj_orientation(
            [[0.3, 0.25, 0.5, 0.0, 0.0, 0.0]], ORIENTATION_EULER
        )

        assert converted[0][:3] == [0.3, 0.25, 0.5]
        assert converted[0][3:] == pytest.approx([0.0, 0.0, 0.0, 1.0])

    def test_euler_uses_degrees_not_radians(self):
        """yaw=180 度 → (0,0,1,0)；若误当弧度会得到完全不同的四元数。"""
        converted = convert_traj_orientation(
            [[0.0, 0.0, 0.0, 180.0, 0.0, 0.0]], ORIENTATION_EULER
        )

        assert converted[0][3:] == pytest.approx([0.0, 0.0, 1.0, 0.0], abs=1e-12)

    def test_euler_order_is_yaw_pitch_roll(self):
        """列序为 [x,y,z,yaw,pitch,roll]，与 ArmEeTimedCmdMove 一致。

        单独绕 yaw / pitch / roll 转 90 度，各自的轴必须对上。
        """
        cases = {
            (90.0, 0.0, 0.0): [0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)],  # yaw → z
            (0.0, 90.0, 0.0): [0.0, math.sqrt(0.5), 0.0, math.sqrt(0.5)],  # pitch → y
            (0.0, 0.0, 90.0): [math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)],  # roll → x
        }
        for (yaw, pitch, roll), expected in cases.items():
            converted = convert_traj_orientation(
                [[0.0, 0.0, 0.0, yaw, pitch, roll]], ORIENTATION_EULER
            )
            assert converted[0][3:] == pytest.approx(expected, abs=1e-12)

    def test_euler_keeps_position_untouched(self):
        converted = convert_traj_orientation(
            [[1.1, -2.2, 3.3, 10.0, 20.0, 30.0]], ORIENTATION_EULER
        )
        assert converted[0][:3] == [1.1, -2.2, 3.3]

    def test_euler_result_is_seven_values(self):
        converted = convert_traj_orientation(
            [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]], ORIENTATION_EULER
        )
        assert len(converted[0]) == 7

    def test_unsupported_format_raises(self):
        with pytest.raises(ValueError, match="orientation_format"):
            convert_traj_orientation([_QUAT_POINT], "rpy")

    def test_euler_wrong_point_length_raises(self):
        with pytest.raises(ValueError, match="6 个值"):
            convert_traj_orientation([[0.3, 0.25, 0.5, 0.0, 0.0]], ORIENTATION_EULER)

    def test_flat_point_list_raises_instead_of_crashing(self):
        """误传单个点（未套一层列表）时报可读错误，而不是 TypeError 崩掉。"""
        with pytest.raises(ValueError, match="轨迹点列表"):
            convert_traj_orientation(_QUAT_POINT, ORIENTATION_QUAT)

    def test_non_finite_value_raises(self):
        with pytest.raises(ValueError, match="NaN|Inf"):
            convert_traj_orientation(
                [[0.0, 0.0, 0.0, 0.0, 0.0, float("inf")]], ORIENTATION_EULER
            )

    def test_non_numeric_value_raises(self):
        with pytest.raises(ValueError, match="float"):
            convert_traj_orientation(
                [[0.0, 0.0, 0.0, 0.0, 0.0, "a"]], ORIENTATION_EULER
            )


# ---------- 第二层：节点接线 ----------

def _initialise(node_cls, module, params):
    """构造节点并跑 initialise()，返回节点（硬件已打桩）。"""
    hw = MagicMock()
    with patch(f"{module}.get_shared_hardware", return_value=hw):
        node = node_cls("traj", "traj", "ns", params)
        node.initialise()
    return node


@pytest.mark.parametrize("node_cls,module", _NODES, ids=_NODE_IDS)
class TestNodeOrientationFormat:
    """两个节点的 orientation_format 参数接线。"""

    def test_default_is_quat_and_keeps_existing_behaviour(
        self, node_cls, module, monkeypatch
    ):
        """不配置该参数时轨迹原样进入技能 —— 既有场景 JSON 语义不变。"""
        monkeypatch.setattr(f"{module}._DRY_RUN", False)
        traj = [_QUAT_POINT, [0.5, 0.25, 0.5, 0.0, 0.0, 0.0, 1.0]]

        node = _initialise(node_cls, module, {"left_traj": traj, "right_traj": traj})

        assert node._skill.params.left_traj == traj
        assert node._skill.params.right_traj == traj

    def test_explicit_quat_matches_default(self, node_cls, module, monkeypatch):
        monkeypatch.setattr(f"{module}._DRY_RUN", False)
        traj = [_QUAT_POINT]

        node = _initialise(
            node_cls,
            module,
            {"left_traj": traj, "right_traj": traj, "orientation_format": "quat"},
        )

        assert node._skill.params.left_traj == traj

    def test_euler_input_is_converted_before_skill(self, node_cls, module, monkeypatch):
        """技能只认 7 维，6 维欧拉角必须在节点内先补成四元数。"""
        monkeypatch.setattr(f"{module}._DRY_RUN", False)
        euler = [[0.3, 0.25, 0.5, 180.0, 0.0, 0.0]]

        node = _initialise(
            node_cls,
            module,
            {
                "left_traj": euler,
                "right_traj": euler,
                "orientation_format": "euler",
            },
        )

        left = node._skill.params.left_traj[0]
        assert len(left) == 7
        assert left[:3] == [0.3, 0.25, 0.5]
        assert left[3:] == pytest.approx([0.0, 0.0, 1.0, 0.0], abs=1e-12)

    def test_euler_format_name_is_case_insensitive(self, node_cls, module, monkeypatch):
        monkeypatch.setattr(f"{module}._DRY_RUN", False)
        euler = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]

        node = _initialise(
            node_cls,
            module,
            {
                "left_traj": euler,
                "right_traj": euler,
                "orientation_format": "EULER",
            },
        )

        assert len(node._skill.params.left_traj[0]) == 7

    def test_json_string_traj_with_euler(self, node_cls, module, monkeypatch):
        monkeypatch.setattr(f"{module}._DRY_RUN", False)
        raw = "[[0.3, 0.25, 0.5, 0.0, 0.0, 0.0]]"

        node = _initialise(
            node_cls,
            module,
            {"left_traj": raw, "right_traj": raw, "orientation_format": "euler"},
        )

        assert node._skill.params.left_traj[0] == pytest.approx(_QUAT_POINT, abs=1e-12)

    def test_unsupported_format_fails_node(self, node_cls, module, monkeypatch):
        monkeypatch.setattr(f"{module}._DRY_RUN", False)

        node = _initialise(
            node_cls,
            module,
            {
                "left_traj": [_QUAT_POINT],
                "right_traj": [_QUAT_POINT],
                "orientation_format": "rpy",
            },
        )

        assert node.update() == Status.FAILURE
        assert "orientation_format" in node.feedback_message

    def test_euler_wrong_length_fails_node(self, node_cls, module, monkeypatch):
        monkeypatch.setattr(f"{module}._DRY_RUN", False)

        node = _initialise(
            node_cls,
            module,
            {
                "left_traj": [[0.3, 0.25, 0.5, 0.0, 0.0]],
                "right_traj": [[0.3, 0.25, 0.5, 0.0, 0.0]],
                "orientation_format": "euler",
            },
        )

        assert node.update() == Status.FAILURE
        assert "6 个值" in node.feedback_message

    def test_missing_traj_still_fails(self, node_cls, module, monkeypatch):
        monkeypatch.setattr(f"{module}._DRY_RUN", False)

        node = _initialise(node_cls, module, {})

        assert node.update() == Status.FAILURE
        assert "missing left_traj" in node.feedback_message

    def test_dry_run_succeeds_with_euler(self, node_cls, module, monkeypatch):
        # _DRY_RUN 是模块级常量（import 时求值），必须直接打桩模块属性
        monkeypatch.setattr(f"{module}._DRY_RUN", True)
        euler = [[0.3, 0.25, 0.5, 0.0, 0.0, 0.0]]

        node = _initialise(
            node_cls,
            module,
            {"left_traj": euler, "right_traj": euler, "orientation_format": "euler"},
        )

        assert node.update() == Status.SUCCESS
        assert "dry-run" in node.feedback_message
