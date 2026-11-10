# -*- coding: utf-8 -*-
"""qiangnao 灵巧手控制通路单元测试（驱动 + mixin + 节点）。

覆盖反馈 recvxqzamBbL7B：控制节点中缺少 qiangnao 灵巧手控制节点。
通路三层：QiangnaoHandControl 节点 → EndEffectorMixin → LejuEndEffector 驱动。
"""

import sys
import types
from unittest.mock import MagicMock, patch

import pytest
from py_trees.common import Status

from core.domain.enums import ArmSide
from core.domain.end_effector import (
    HAND_JOINT_COUNT,
    HandFingerCommand,
    SG100HandCommand,
)
from core.domain.result import Result
from adapters.hardware.leju_wheeled.mixins.end_effector_mixin import EndEffectorMixin
from drivers.leju.end_effector import LejuEndEffector
from orchestration.nodes.qiangnao_hand_control import QiangnaoHandControl

pytestmark = pytest.mark.unit

_OPEN = [0.0] * HAND_JOINT_COUNT
_CLOSE = [100.0] * HAND_JOINT_COUNT


def _ok(msg="ok", data=None):
    return Result.ok(msg, data=data)


class _Mixin(EndEffectorMixin):
    """最小 mixin 实例，注入 mock 末端驱动。"""

    def __init__(self, end_effector):
        self._end_effector = end_effector


def _fake_robot_hand_position_module():
    """kuavo_msgs 由 submodule 提供，本机不一定可用，用桩替代。

    桩不校验字段类型：能记录到 float 说明驱动漏了 uint8 转换，正是要测的点。
    """

    class robotHandPosition:  # noqa: N801 - 与 ROS 生成的消息类同名
        def __init__(self):
            self.left_hand_position = []
            self.right_hand_position = []

    msg_module = types.ModuleType("kuavo_msgs.msg")
    msg_module.robotHandPosition = robotHandPosition
    pkg_module = types.ModuleType("kuavo_msgs")
    pkg_module.msg = msg_module
    return {"kuavo_msgs": pkg_module, "kuavo_msgs.msg": msg_module}


def _driver(pub):
    """构造只差一条已连接 qiangnao 通路的驱动实例（不跑 __init__ 的 ROS 连接）。"""
    driver = LejuEndEffector({"type": "qiangnao"})
    driver._connected = True
    driver._hand_pub = pub
    return driver


def _hand_pub(subscribers=1):
    pub = MagicMock()
    pub.get_num_connections.return_value = subscribers
    return pub


# ---------- 第一层：驱动 send_hand_command ----------

class TestSendHandCommand:
    """qiangnao 驱动的线格式与对端存活校验。"""

    def test_positions_are_published_as_uint8_compatible_int(self):
        """robotHandPosition 的左右手字段是 uint8[]，float 会在序列化时报错。"""
        pub = _hand_pub()
        driver = _driver(pub)

        with patch.dict(sys.modules, _fake_robot_hand_position_module()):
            result = driver.send_hand_command(
                HandFingerCommand(positions=_CLOSE),
                HandFingerCommand(positions=_OPEN),
            )

        assert result.success, result.message
        msg = pub.publish.call_args.args[0]
        assert all(isinstance(v, int) for v in msg.left_hand_position)
        assert all(isinstance(v, int) for v in msg.right_hand_position)
        assert msg.left_hand_position == [100] * HAND_JOINT_COUNT
        assert msg.right_hand_position == [0] * HAND_JOINT_COUNT

    def test_fractional_positions_are_rounded(self):
        pub = _hand_pub()
        driver = _driver(pub)

        with patch.dict(sys.modules, _fake_robot_hand_position_module()):
            result = driver.send_hand_command(
                HandFingerCommand(positions=[0.4, 0.6, 50.5, 99.6, 100.0, 0.0]),
                HandFingerCommand(positions=_OPEN),
            )

        assert result.success, result.message
        msg = pub.publish.call_args.args[0]
        assert msg.left_hand_position == [0, 1, 50, 100, 100, 0]

    def test_no_subscriber_fails_instead_of_silent_success(self):
        """话题无订阅者时 rospy 会直接丢弃，必须显式失败而不是假成功。"""
        pub = _hand_pub(subscribers=0)
        driver = _driver(pub)

        with patch.dict(sys.modules, _fake_robot_hand_position_module()):
            result = driver.send_hand_command(
                HandFingerCommand(positions=_CLOSE),
                HandFingerCommand(positions=_OPEN),
            )

        assert not result.success
        assert "订阅者" in result.message
        pub.publish.assert_not_called()

    def test_not_connected_fails(self):
        pub = _hand_pub()
        driver = _driver(pub)
        driver._connected = False

        result = driver.send_hand_command(
            HandFingerCommand(positions=_CLOSE),
            HandFingerCommand(positions=_OPEN),
        )

        assert not result.success
        pub.publish.assert_not_called()


# ---------- 第二层：mixin 侧别分发 ----------

class TestMixinHandDispatch:
    """EndEffectorMixin 对 HandFingerCommand 的左右手填充。"""

    def test_both_sides_receive_the_command(self):
        """side=BOTH 时两侧都应拿到实际指令，而不是被默认零位顶掉。"""
        ee = MagicMock()
        ee.send_hand_command.return_value = _ok()
        hw = _Mixin(ee)

        result = hw.control_end_effector(
            ArmSide.BOTH, HandFingerCommand(positions=_CLOSE)
        )

        assert result.success
        left_cmd, right_cmd = ee.send_hand_command.call_args.args
        assert left_cmd.positions == _CLOSE
        assert right_cmd.positions == _CLOSE

    def test_left_side_keeps_right_at_zero(self):
        """side=LEFT 时右侧仍是零位指令（保持既有语义）。"""
        ee = MagicMock()
        ee.send_hand_command.return_value = _ok()
        hw = _Mixin(ee)

        hw.control_end_effector(ArmSide.LEFT, HandFingerCommand(positions=_CLOSE))

        left_cmd, right_cmd = ee.send_hand_command.call_args.args
        assert left_cmd.positions == _CLOSE
        assert right_cmd.positions == _OPEN

    def test_right_side_keeps_left_at_zero(self):
        ee = MagicMock()
        ee.send_hand_command.return_value = _ok()
        hw = _Mixin(ee)

        hw.control_end_effector(ArmSide.RIGHT, HandFingerCommand(positions=_CLOSE))

        left_cmd, right_cmd = ee.send_hand_command.call_args.args
        assert left_cmd.positions == _OPEN
        assert right_cmd.positions == _CLOSE

    def test_sg100_still_routes_away_from_hand_path(self):
        """回归：SG100 指令不得落进 qiangnao 分支。"""
        ee = MagicMock()
        ee.send_sg100_command.return_value = _ok()
        hw = _Mixin(ee)

        hw.control_end_effector(ArmSide.LEFT, SG100HandCommand())

        ee.send_sg100_command.assert_called_once()
        ee.send_hand_command.assert_not_called()


# ---------- 第三层：节点解析与分发 ----------

class TestQiangnaoParseCommand:
    """QiangnaoHandControl._parse_command 解析逻辑。"""

    def test_compact_list(self):
        side, cmd = QiangnaoHandControl._parse_command([_CLOSE, "left"])
        assert side == "left"
        assert cmd.positions == _CLOSE

    def test_compact_list_both(self):
        side, cmd = QiangnaoHandControl._parse_command([_CLOSE, "both"])
        assert side == "both"
        assert cmd.positions == _CLOSE

    def test_dict_form(self):
        side, cmd = QiangnaoHandControl._parse_command(
            {"positions": _OPEN, "side": "right"}
        )
        assert side == "right"
        assert cmd.positions == _OPEN

    def test_json_string_form(self):
        side, cmd = QiangnaoHandControl._parse_command(
            '{"positions": [0,0,0,0,0,0], "side": "left"}'
        )
        assert side == "left"
        assert cmd.positions == _OPEN

    def test_upper_case_side_is_normalized(self):
        side, _ = QiangnaoHandControl._parse_command([_CLOSE, "LEFT"])
        assert side == "left"

    def test_invalid_side_raises(self):
        with pytest.raises(ValueError, match="left/right/both"):
            QiangnaoHandControl._parse_command([_CLOSE, "middle"])

    def test_wrong_position_count_raises(self):
        with pytest.raises(ValueError, match="positions"):
            QiangnaoHandControl._parse_command([[0.0] * 5, "left"])

    def test_position_above_100_raises(self):
        with pytest.raises(ValueError, match=r"\[0, 100\]"):
            QiangnaoHandControl._parse_command([[101.0] + [0.0] * 5, "left"])

    def test_negative_position_raises(self):
        with pytest.raises(ValueError, match=r"\[0, 100\]"):
            QiangnaoHandControl._parse_command([[-1.0] + [0.0] * 5, "left"])

    def test_non_finite_position_raises(self):
        with pytest.raises(ValueError, match=r"\[0, 100\]"):
            QiangnaoHandControl._parse_command(
                [[float("nan")] + [0.0] * 5, "left"]
            )

    def test_non_numeric_position_raises(self):
        with pytest.raises(ValueError, match="float"):
            QiangnaoHandControl._parse_command([["a"] * 6, "left"])

    def test_bad_json_raises(self):
        with pytest.raises(ValueError, match="JSON"):
            QiangnaoHandControl._parse_command("{not json")

    def test_missing_positions_key_raises(self):
        with pytest.raises(ValueError, match="positions"):
            QiangnaoHandControl._parse_command({"side": "left"})

    def test_select_command_key(self):
        stage = {"open": [_OPEN, "left"], "grasp": [_CLOSE, "right"]}
        raw = QiangnaoHandControl._select_command(stage, "grasp")
        side, cmd = QiangnaoHandControl._parse_command(raw)
        assert side == "right"
        assert cmd.positions == _CLOSE

    def test_select_command_key_absent_raises(self):
        with pytest.raises(ValueError, match="阶段键"):
            QiangnaoHandControl._select_command({"open": [_OPEN, "left"]}, "grasp")


def _make_node(params):
    node = QiangnaoHandControl("qiangnao", "qiangnao", "ns", params)
    node.initialise()
    return node


class TestQiangnaoNodeUpdate:
    """节点 update() 的状态映射与硬件分发。"""

    def test_dry_run_returns_success(self, monkeypatch):
        monkeypatch.setenv("STUDIO_DRY_RUN", "1")
        node = _make_node({"command": [_CLOSE, "left"]})

        assert node.update() == Status.SUCCESS
        assert "dry-run" in node.feedback_message

    def test_dispatch_left_forwards_hand_command(self, monkeypatch):
        monkeypatch.delenv("STUDIO_DRY_RUN", raising=False)
        mock_hw = MagicMock()
        mock_hw.control_end_effector.return_value = _ok()
        with patch(
            "orchestration.nodes.qiangnao_hand_control.get_shared_hardware",
            return_value=mock_hw,
        ):
            node = _make_node({"command": [_CLOSE, "left"]})
            assert node.update() == Status.SUCCESS

        side, cmd = mock_hw.control_end_effector.call_args.args
        assert side == ArmSide.LEFT
        assert isinstance(cmd, HandFingerCommand)
        assert cmd.positions == _CLOSE

    def test_dispatch_both_maps_to_both(self, monkeypatch):
        monkeypatch.delenv("STUDIO_DRY_RUN", raising=False)
        mock_hw = MagicMock()
        mock_hw.control_end_effector.return_value = _ok()
        with patch(
            "orchestration.nodes.qiangnao_hand_control.get_shared_hardware",
            return_value=mock_hw,
        ):
            node = _make_node({"command": [_CLOSE, "both"]})
            assert node.update() == Status.SUCCESS

        side, _ = mock_hw.control_end_effector.call_args.args
        assert side == ArmSide.BOTH

    def test_dispatch_reads_command_key(self, monkeypatch):
        monkeypatch.delenv("STUDIO_DRY_RUN", raising=False)
        mock_hw = MagicMock()
        mock_hw.control_end_effector.return_value = _ok()
        with patch(
            "orchestration.nodes.qiangnao_hand_control.get_shared_hardware",
            return_value=mock_hw,
        ):
            node = _make_node(
                {
                    "command": {"open": [_OPEN, "left"], "close": [_CLOSE, "right"]},
                    "command_key": "close",
                }
            )
            assert node.update() == Status.SUCCESS

        side, cmd = mock_hw.control_end_effector.call_args.args
        assert side == ArmSide.RIGHT
        assert cmd.positions == _CLOSE

    def test_hardware_failure_returns_failure(self, monkeypatch):
        monkeypatch.delenv("STUDIO_DRY_RUN", raising=False)
        mock_hw = MagicMock()
        mock_hw.control_end_effector.return_value = Result.fail("driver not ready")
        with patch(
            "orchestration.nodes.qiangnao_hand_control.get_shared_hardware",
            return_value=mock_hw,
        ):
            node = _make_node({"command": [_CLOSE, "left"]})
            assert node.update() == Status.FAILURE

        assert "driver not ready" in node.feedback_message

    def test_invalid_command_returns_failure(self, monkeypatch):
        monkeypatch.delenv("STUDIO_DRY_RUN", raising=False)
        node = _make_node({"command": [[0.0] * 5, "left"]})

        assert node.update() == Status.FAILURE
        assert "invalid qiangnao command" in node.feedback_message
