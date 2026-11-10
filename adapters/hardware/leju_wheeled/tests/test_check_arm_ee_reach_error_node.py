# -*- coding: utf-8 -*-
"""手臂末端到位检查与误差反馈节点单元测试。

覆盖反馈 recvxBWfN9UuU7：手臂/折叠臂缺少到位检查与误差反馈。

节点只以硬件服务的 success 为准（控制器的"运动完成"判定），
误差向量仅作输出上报；到位后写黑板并写 feedback。
"""

from unittest.mock import MagicMock, patch

import pytest
from py_trees.common import Status

from core.domain.result import Result
from orchestration.nodes.check_arm_ee_reach_error_move import (
    CheckArmEeReachErrorMove,
)

pytestmark = pytest.mark.unit

_MODULE = "orchestration.nodes.check_arm_ee_reach_error_move"
_ERR_LEFT = [0.001, -0.002, 0.003, 0.01, -0.02, 0.03]
_ERR_RIGHT = [0.004, 0.005, -0.006, 0.04, 0.05, -0.06]


def _ok(err_vector):
    return Result.ok(data={"err_vector": list(err_vector)})


def _fail(message="末端未完成或不在末端模式"):
    return Result.fail(message)


def _node(params, hardware):
    """构造节点并跑 initialise()，hardware 已打桩。"""
    with patch(f"{_MODULE}.get_shared_hardware", return_value=hardware):
        node = CheckArmEeReachErrorMove("reach", "reach", "ns", params)
        node.initialise()
    return node


def _reach_node(params=None, side_errors=None):
    """按侧别给不同返回的服务桩；side_errors 为 None 表示该侧未到位。"""
    side_errors = side_errors or {}
    hardware = MagicMock()
    hardware.get_ee_pose_reach_error.side_effect = (
        lambda is_left: _ok(side_errors["left" if is_left else "right"])
        if ("left" if is_left else "right") in side_errors
        else _fail()
    )
    node = _node(params if params is not None else {"side": "left"}, hardware)
    return node, hardware


class TestSideAndParamValidation:
    """side / timeout / poll_interval 的参数校验。"""

    def test_invalid_side_fails_before_touching_hardware(self):
        hardware = MagicMock()

        node = _node({"side": "middle"}, hardware)

        assert node.update() == Status.FAILURE
        assert "left / right / both" in node.feedback_message
        hardware.get_ee_pose_reach_error.assert_not_called()

    def test_zero_timeout_is_rejected(self):
        node = _node({"side": "left", "timeout": 0}, MagicMock())

        assert node.update() == Status.FAILURE
        assert "timeout" in node.feedback_message

    def test_non_numeric_timeout_is_rejected(self):
        node = _node({"side": "left", "timeout": "abc"}, MagicMock())

        assert node.update() == Status.FAILURE
        assert "timeout" in node.feedback_message

    def test_side_is_case_insensitive(self):
        node, _ = _reach_node({"side": "LEFT"}, {"left": _ERR_LEFT})

        assert node.update() == Status.SUCCESS


class TestWaitForReach:

    def test_not_reached_keeps_running(self):
        node, hardware = _reach_node({"side": "left", "timeout": 5.0}, {})

        with patch(f"{_MODULE}.time.sleep"):
            assert node.update() == Status.RUNNING

        hardware.get_ee_pose_reach_error.assert_called_with(is_left=True)
        assert "等待到位" in node.feedback_message

    def test_reached_succeeds_and_reports_error(self):
        node, _ = _reach_node({"side": "left", "timeout": 5.0}, {"left": _ERR_LEFT})

        assert node.update() == Status.SUCCESS
        assert "已到位" in node.feedback_message
        assert "0.003000" in node.feedback_message

    def test_timeout_expiry_fails_with_pending_side(self):
        node, _ = _reach_node({"side": "left", "timeout": 1.0}, {})
        # initialise 已把 deadline 设为「真实 monotonic() + 1」，把时间推到它之后即超时
        node._deadline = 1_000.0

        with patch(f"{_MODULE}.time.monotonic", return_value=2_000.0):
            assert node.update() == Status.FAILURE

        assert "超时" in node.feedback_message
        assert "left" in node.feedback_message

    def test_both_sides_waits_until_second_one_arrives(self):
        node, hardware = _reach_node({"side": "both", "timeout": 5.0}, {"left": _ERR_LEFT})

        with patch(f"{_MODULE}.time.sleep"):
            assert node.update() == Status.RUNNING  # 仅左手到位

        # 右手随后到位，应立刻成功
        hardware.get_ee_pose_reach_error.side_effect = (
            lambda is_left: _ok(_ERR_LEFT if is_left else _ERR_RIGHT)
        )
        assert node.update() == Status.SUCCESS
        assert "left=" in node.feedback_message and "right=" in node.feedback_message

    def test_side_param_controls_queried_arm(self):
        node, hardware = _reach_node({"side": "right", "timeout": 5.0}, {"right": _ERR_RIGHT})

        assert node.update() == Status.SUCCESS
        hardware.get_ee_pose_reach_error.assert_called_once_with(is_left=False)

    def test_bad_error_vector_dimension_fails(self):
        hardware = MagicMock()
        hardware.get_ee_pose_reach_error.return_value = _ok([0.1, 0.2, 0.3])

        node = _node({"side": "left", "timeout": 5.0}, hardware)

        assert node.update() == Status.FAILURE
        assert "维度异常" in node.feedback_message

    def test_dry_run_succeeds_without_hardware(self):
        hardware = MagicMock()

        with patch(f"{_MODULE}._DRY_RUN", True):
            node = _node({"side": "left"}, hardware)
            status = node.update()

        assert status == Status.SUCCESS
        hardware.get_ee_pose_reach_error.assert_not_called()


class TestBlackboardResult:

    def test_writes_error_map_under_result_key(self):
        node, _ = _reach_node({"side": "left", "timeout": 5.0}, {"left": _ERR_LEFT})
        blackboard = MagicMock()
        node.global_blackboard = blackboard

        assert node.update() == Status.SUCCESS
        blackboard.set.assert_called_once_with("arm_ee_reach_error", {"left": _ERR_LEFT})

    def test_empty_result_key_skips_blackboard(self):
        node, _ = _reach_node(
            {"side": "left", "timeout": 5.0, "result_key": ""}, {"left": _ERR_LEFT}
        )
        blackboard = MagicMock()
        node.global_blackboard = blackboard

        assert node.update() == Status.SUCCESS
        blackboard.set.assert_not_called()

    def test_blackboard_failure_does_not_turn_reach_into_failure(self):
        """到位结论来自控制器，黑板写不进去不该把它翻成 FAILURE。"""
        node, _ = _reach_node({"side": "left", "timeout": 5.0}, {"left": _ERR_LEFT})
        blackboard = MagicMock()
        blackboard.set.side_effect = RuntimeError("no such key")
        node.global_blackboard = blackboard

        assert node.update() == Status.SUCCESS
