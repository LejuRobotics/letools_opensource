# -*- coding: utf-8 -*-
"""下肢关节位置读取的离线单元测试（不依赖 ROS）。"""

import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

# 直接加载纯 Mixin，避免 adapters 包初始化连带导入 ROS；本测试只验证状态读取逻辑。
_MODULE_PATH = (Path(__file__).resolve().parents[1] / "adapters" / "hardware" /
                "leju_wheeled" / "mixins" / "sdk_control_mixin.py")
_SPEC = importlib.util.spec_from_file_location("sdk_control_mixin_under_test", _MODULE_PATH)
_MODULE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MODULE)
SDKControlMixin = _MODULE.SDKControlMixin


class _FakeHardware(SDKControlMixin):
    def __init__(self, position, angle_unit="deg"):
        self._angle_unit = angle_unit
        state = SimpleNamespace(
            joint_state=SimpleNamespace(position=position) if position is not None else None)
        self._low_level_sdk_manager = SimpleNamespace(robot_sdk=SimpleNamespace(state=state))

    def _from_rad(self, values):
        values = np.asarray(values)
        return values if self._angle_unit == "rad" else np.degrees(values)


def test_get_leg_joint_positions_returns_four_values_in_user_unit():
    values_rad = [math.pi / 6, math.pi / 4, math.pi / 3, math.pi / 2]
    hardware = _FakeHardware(values_rad)

    result = hardware.get_leg_joint_positions()

    assert result.success
    assert result.data == pytest.approx([30.0, 45.0, 60.0, 90.0])


def test_get_leg_joint_positions_supports_radian_user_unit():
    values_rad = [0.1, 0.2, 0.3, 0.4]
    hardware = _FakeHardware(values_rad, angle_unit="rad")

    result = hardware.get_leg_joint_positions()

    assert result.success
    assert result.data == pytest.approx(values_rad)


def test_get_leg_joint_positions_requires_low_level_manager():
    hardware = _FakeHardware([0.0] * 4)
    hardware._low_level_sdk_manager = None

    result = hardware.get_leg_joint_positions()

    assert not result.success
    assert "未初始化" in result.message


@pytest.mark.parametrize("position", [None, [0.1, 0.2, 0.3]])
def test_get_leg_joint_positions_rejects_missing_or_short_state(position):
    hardware = _FakeHardware(position)

    result = hardware.get_leg_joint_positions()

    assert not result.success


def test_get_leg_joint_positions_rejects_non_finite_value():
    hardware = _FakeHardware([0.1, math.nan, 0.3, math.inf])

    result = hardware.get_leg_joint_positions()

    assert not result.success
    assert "非有限值" in result.message
