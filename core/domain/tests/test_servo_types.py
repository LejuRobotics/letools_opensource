# -*- coding: utf-8 -*-
"""core/domain/servo_types 单元测试。

覆盖 ServoCommand 的位置/速度双模字段：
1. 默认 mode 为 "position"，速度字段全为 0 —— 保证旧构造点零改动
2. 显式构造 velocity 模式时，vx/vy/wz 可原样读回
3. frozen dataclass 默认值齐全，可无参构造
"""

from dataclasses import FrozenInstanceError

import pytest

from core.domain.servo_types import ServoCommand


def test_default_mode_is_position():
    """不传任何参数时，mode 默认 position，速度字段归零。"""
    cmd = ServoCommand()

    assert cmd.mode == "position"
    assert cmd.vx_mps == 0.0
    assert cmd.vy_mps == 0.0
    assert cmd.wz_radps == 0.0


def test_position_mode_fields_read_back():
    """位置模式字段与旧构造口径一致（仅关键字传位移）。"""
    cmd = ServoCommand(dx_m=0.005, dy_m=0.003, dtheta_rad=0.01, iter_index=2)

    assert cmd.mode == "position"
    assert cmd.dx_m == 0.005
    assert cmd.dy_m == 0.003
    assert cmd.dtheta_rad == 0.01
    assert cmd.iter_index == 2


def test_velocity_mode_fields_read_back():
    """显式构造 velocity 模式：vx/vy/wz 原样读回，单位 SI（m/s、rad/s）。"""
    cmd = ServoCommand(mode="velocity", vx_mps=0.12, vy_mps=-0.03, wz_radps=0.4)

    assert cmd.mode == "velocity"
    assert cmd.vx_mps == 0.12
    assert cmd.vy_mps == -0.03
    assert cmd.wz_radps == 0.4
    # 未指定的位置字段仍为其默认值
    assert cmd.dx_m == 0.0
    assert cmd.dy_m == 0.0
    assert cmd.dtheta_rad == 0.0


def test_is_frozen():
    """ServoCommand 仍是 frozen dataclass，不可变。"""
    cmd = ServoCommand()

    with pytest.raises(FrozenInstanceError):
        cmd.mode = "velocity"
