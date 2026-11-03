# -*- coding: utf-8 -*-
"""MultiTimedCommandMove: configurable multi-planner TimedCmd node."""

import json
import os

import py_trees
from py_trees.common import Status

from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.multi_timed_command import (
    MultiTimedCommandParams,
    MultiTimedCommandSkill,
)

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


@define_manifest(
    label="多规划器组合控制（TimedCmd）",
    category=["motion", "timed_cmd"],
    tree_type="studio_smoke",
    description="按配置将任意一组 planner 通过一次 TimedCmd 请求下发",
    params=[
        {
            "name": "commands",
            "type": "json",
            "default": "[]",
            "description": "指令列表，每项包含 planner_index 和 cmd_vec，可选 desire_time",
        },
        {"name": "desire_time", "type": "float", "default": "3.0", "description": "默认执行时间（秒）"},
        {"name": "is_sync", "type": "bool", "default": "true", "description": "是否让各 planner 同时完成"},
        {"name": "focus_ee", "type": "bool", "default": "false",
         "description": "true=专注手臂末端解算，躯干可能随动；false=尽可能不动躯干，在手臂工作区间内解算，精度可能降低。focus_z 内部跟随该值"},
        {"name": "prepare_arm_control", "type": "bool", "default": "true",
         "description": "包含躯干或手臂 planner 2–9 时，发令前切换到手臂外部控制模式 2；不复位、不自动切回 0。关闭则需外部保证模式"},
        {"name": "settle_time", "type": "float", "default": "0.5", "description": "运动后的额外等待（秒）"},
    ],
    inputs=[],
    outputs=[],
)
class MultiTimedCommandMove(BaseAction):
    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._skill = None
        self._dry_done = False
        self._error = ""

    def initialise(self):
        self._skill = None
        self._dry_done = False
        self._error = ""

        self._commands_board_key = self.params.get("commands__board_key", "")
        if self._commands_board_key:
            self.global_blackboard.register_key(
                key=self._commands_board_key, access=py_trees.common.Access.READ
            )
        try:
            commands = self.params.get("commands", [])
            if self._commands_board_key and self.global_blackboard.exists(self._commands_board_key):
                commands = self.global_blackboard.get(self._commands_board_key)
            skill_params = MultiTimedCommandParams(
                commands=self._parse_commands(commands),
                desire_time=float(self.params.get("desire_time", 3.0)),
                is_sync=self._to_bool(self.params.get("is_sync", True)),
                focus_ee=self._to_bool(self.params.get("focus_ee", False)),
                prepare_arm_control=self._to_bool(self.params.get("prepare_arm_control", True)),
                settle_time=float(self.params.get("settle_time", 0.5)),
            )
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            self._fail(str(exc))
            return

        skill = MultiTimedCommandSkill(hardware=None if _DRY_RUN else get_shared_hardware())
        result = skill.initialize(skill_params)
        if not result.success:
            self._fail(result.message or "multi_timed_command init failed")
            return

        if _DRY_RUN:
            planners = [command["planner_index"] for command in skill.commands]
            self.feedback_message = f"dry-run multi_timed_command planners={planners}"
            self._dry_done = True
            return
        self._skill = skill

    def update(self):
        if self._error:
            return Status.FAILURE
        if _DRY_RUN:
            return Status.SUCCESS if self._dry_done else Status.FAILURE
        if self._skill is None:
            return Status.FAILURE
        if self._skill.is_finished():
            return Status.SUCCESS
        result = self._skill.execute()
        if not result.success:
            self._fail(result.message or "multi_timed_command failed")
            return Status.FAILURE
        return Status.RUNNING

    def _fail(self, message):
        self._error = message
        self.feedback_message = message

    @staticmethod
    def _parse_commands(raw):
        if isinstance(raw, str):
            raw = json.loads(raw)
        if not isinstance(raw, list):
            raise TypeError("commands must be a list")
        return raw

    @staticmethod
    def _to_bool(value):
        if isinstance(value, bool):
            return value
        return str(value).lower() in ("1", "true", "yes", "on")
