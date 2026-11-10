# -*- coding: utf-8 -*-
"""终端日志噪音回归测试（无需真机）。

被测对象:
1. skills.atomic.refactored_sdk 下三个技能的 on_execute 成功路径 —— 断言成功时
   INFO 级只剩一行 [Perf] 汇总，且原业务行的独有字段（角度 / actual_time /
   arrived 等）不因删行而丢失。
2. adapters.hardware.leju_wheeled.mixins.arm_control_mixin —— 断言裸 print 已清空，
   且原有的位姿/时序细节改由 logger.debug 承载而非直接丢弃。
"""

import logging
import time
from pathlib import Path

import pytest
from unittest.mock import MagicMock

from core.domain.result import Result
from skills.atomic.refactored_sdk.check_arrived_jibot import (
    CheckArrivedJibotParams,
    CheckArrivedJibotSkill,
)
from skills.atomic.refactored_sdk.leg_joint_sdk import (
    LegJointSdkParams,
    LegJointSdkSkill,
)
from skills.atomic.refactored_sdk.leg_joint_timed import (
    LegJointTimedParams,
    LegJointTimedSkill,
)

_ANGLES = [29.00, -41.11, 12.21, 0.0]  # [knee, leg, waist_pitch, waist_yaw]，度

_MIXIN_PATH = (
    Path(__file__).resolve().parents[1] / "mixins" / "arm_control_mixin.py"
)


def _info_lines(caplog, logger_name):
    """只取被测技能自己 logger 的 INFO 记录，忽略装饰器/框架的旁路输出。"""
    return [
        r.getMessage()
        for r in caplog.records
        if r.name == logger_name and r.levelno == logging.INFO
    ]


@pytest.mark.unit
class TestLegJointSdkLogNoise:
    def test_success_emits_single_info_line_with_angles(self, monkeypatch, caplog):
        """成功时 INFO 只留 hardware_call_done 一行，且角度仍在其中"""
        monkeypatch.setattr(time, "sleep", lambda *_: None)
        hw = MagicMock()
        hw.send_leg_joint_sdk.return_value = Result.ok("下发成功")

        skill = LegJointSdkSkill(hardware=hw)
        assert skill.initialize(
            LegJointSdkParams(joint_angles=_ANGLES, total_time=2.0)
        ).success

        with caplog.at_level(logging.INFO):
            assert skill.execute().success

        lines = _info_lines(caplog, "skills.atomic.refactored_sdk.leg_joint_sdk")
        assert len(lines) == 1, lines
        assert "hardware_call_done" in lines[0]
        assert "29.000" in lines[0]  # 原业务行的 joint_angles 已并入，未丢失

    def test_start_line_is_debug_only(self, monkeypatch, caplog):
        """hardware_call_start 降为 DEBUG：INFO 级看不到，DEBUG 级仍可诊断"""
        monkeypatch.setattr(time, "sleep", lambda *_: None)
        hw = MagicMock()
        hw.send_leg_joint_sdk.return_value = Result.ok("下发成功")

        def _new_skill():
            skill = LegJointSdkSkill(hardware=hw)
            skill.initialize(LegJointSdkParams(joint_angles=_ANGLES, total_time=2.0))
            return skill

        with caplog.at_level(logging.INFO):
            _new_skill().execute()
        assert not any(
            "hardware_call_start" in line
            for line in _info_lines(caplog, "skills.atomic.refactored_sdk.leg_joint_sdk")
        ), caplog.text

        caplog.clear()
        with caplog.at_level(logging.DEBUG):
            _new_skill().execute()
        assert "hardware_call_start" in caplog.text


@pytest.mark.unit
class TestLegJointTimedLogNoise:
    def test_success_emits_single_info_line_keeping_actual_time(
        self, monkeypatch, caplog
    ):
        """成功时 INFO 只留一行，且 actual_time 不丢"""
        monkeypatch.setattr(time, "sleep", lambda *_: None)
        hw = MagicMock()
        hw.send_leg_joint_timed.return_value = Result.ok(
            "下发成功", data={"actual_time": 2.5}
        )

        skill = LegJointTimedSkill(hardware=hw)
        assert skill.initialize(
            LegJointTimedParams(joint_angles=_ANGLES, desire_time=2.0)
        ).success

        with caplog.at_level(logging.INFO):
            assert skill.execute().success

        lines = _info_lines(caplog, "skills.atomic.refactored_sdk.leg_joint_timed")
        assert len(lines) == 1, lines
        assert "hardware_call_done" in lines[0]
        assert "2.500" in lines[0]  # actual_time 原只在业务行，现并入汇总行

    def test_start_line_is_debug_only(self, monkeypatch, caplog):
        monkeypatch.setattr(time, "sleep", lambda *_: None)
        hw = MagicMock()
        hw.send_leg_joint_timed.return_value = Result.ok(
            "下发成功", data={"actual_time": 2.5}
        )

        skill = LegJointTimedSkill(hardware=hw)
        skill.initialize(LegJointTimedParams(joint_angles=_ANGLES, desire_time=2.0))

        with caplog.at_level(logging.INFO):
            skill.execute()
        assert not any(
            "hardware_call_start" in line
            for line in _info_lines(caplog, "skills.atomic.refactored_sdk.leg_joint_timed")
        ), caplog.text


@pytest.mark.unit
class TestCheckArrivedJibotLogNoise:
    def test_success_emits_single_info_line_keeping_result_fields(
        self, monkeypatch, caplog
    ):
        """成功时 INFO 只留一行，且 arrived/status/message 不丢"""
        hw = MagicMock()
        hw.check_arrived_jibot.return_value = Result.ok(
            "到达", data={"arrived": True, "status": 0, "message": "ok"}
        )

        skill = CheckArrivedJibotSkill(hardware=hw)
        assert skill.initialize(
            CheckArrivedJibotParams(task_id="t-1", blocking=True, timeout=20.0)
        ).success

        with caplog.at_level(logging.INFO):
            assert skill.execute().success

        lines = _info_lines(caplog, "skills.atomic.refactored_sdk.check_arrived_jibot")
        assert len(lines) == 1, lines
        assert "hardware_call_done" in lines[0]
        assert "arrived=True" in lines[0]
        assert "ok" in lines[0]

    def test_start_line_is_debug_only(self, caplog):
        hw = MagicMock()
        hw.check_arrived_jibot.return_value = Result.ok(
            "到达", data={"arrived": True, "status": 0, "message": "ok"}
        )

        skill = CheckArrivedJibotSkill(hardware=hw)
        skill.initialize(
            CheckArrivedJibotParams(task_id="t-1", blocking=True, timeout=20.0)
        )

        with caplog.at_level(logging.INFO):
            skill.execute()
        assert not any(
            "hardware_call_start" in line
            for line in _info_lines(
                caplog, "skills.atomic.refactored_sdk.check_arrived_jibot"
            )
        ), caplog.text


@pytest.mark.unit
class TestArmControlMixinNoBarePrint:
    def test_source_has_no_bare_print(self):
        """裸 print 绕过 logging 系统，终端无法统一收敛，源码里应已清空"""
        source = _MIXIN_PATH.read_text(encoding="utf-8")
        assert "print(" not in source, [
            f"{i}: {line}"
            for i, line in enumerate(source.splitlines(), 1)
            if "print(" in line
        ]

    def test_unique_details_moved_to_debug(self):
        """原 print 里独有的位姿/时序细节改由 logger.debug 承载，未直接丢弃"""
        source = _MIXIN_PATH.read_text(encoding="utf-8")
        for fragment in (
            'logger.debug(f"[send_both_ee_poses] frame=',
            'logger.debug(f"收到 joint reach_time:',
            'logger.debug(f"reach_time=',
        ):
            assert fragment in source, fragment
