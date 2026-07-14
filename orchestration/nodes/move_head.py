# -*- coding: utf-8 -*-
"""
MoveHead：头部控制。
mode=smoke 时薄节点 → head_control Skill；其它 mode 保留 embodied 逻辑。
"""

import os

import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status

from orchestration.utils.manifest_decorators import define_manifest

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


@define_manifest(
    label="控制头部移动",
    category=["head", "control"],
    tree_type="studio_smoke",
    description="控制头部；smoke 模式使用锁定角度参数",
    params=[
        {"name": "mode", "type": "string", "default": "manual", "description": "manual / from_board / smoke"},
        {"name": "yaw_deg", "type": "float", "default": "11.5", "description": "smoke 模式 yaw（度）"},
        {"name": "pitch_deg", "type": "float", "default": "5.7", "description": "smoke 模式 pitch（度）"},
        {"name": "count", "type": "int", "default": "0", "description": "重复计数"},
        {"name": "head_event_timeout", "type": "int", "default": "20000", "description": "ms"},
        {"name": "head_search_yaws", "type": "string", "default": "", "description": "manual 模式"},
        {"name": "head_search_pitchs", "type": "string", "default": "", "description": "manual 模式"},
    ],
    inputs=[
        {
            "name": "head_move_values",
            "type": "list",
            "required": False,
            "default_key": "HeadMoveValues",
            "description": "from_board 模式",
        }
    ],
    outputs=[],
)
class MoveHead(Behaviour):

    def __init__(self, name: str, label: str, namespace: str, params):
        super(MoveHead, self).__init__(name)
        self.params = params
        self.label = label.split("/", -1)[-1]
        self.mode = str(self.params.get("mode", "manual"))
        self.global_blackboard = self.attach_blackboard_client(name=name)
        self._skill = None
        self._smoke_dry_done = False
        self._legacy_ready = False

    def initialise(self):
        self._smoke_dry_done = False
        self._skill = None
        if self.mode == "smoke":
            if _DRY_RUN:
                return
            from orchestration.shared_hardware import get_shared_hardware
            from skills.atomic.manipulation.head_control.params import HeadControlParams
            from skills.atomic.manipulation.head_control.skill import HeadControlSkill

            head_params = HeadControlParams(
                yaw_deg=float(self.params.get("yaw_deg", 11.5)),
                pitch_deg=float(self.params.get("pitch_deg", 5.7)),
            )
            self._skill = HeadControlSkill(hardware=get_shared_hardware())
            result = self._skill.initialize(head_params)
            if not result.success:
                self.feedback_message = result.message or "head_control init failed"
            return
        if _DRY_RUN:
            self._legacy_ready = True
            return
        self._init_legacy_head_event()

    def _init_legacy_head_event(self):
        """沿用 embodied RobotSDK 路径（非 smoke / 非 dry-run）。"""
        from shared_robot_sdk import get_shared_robot_sdk
        from kuavo_humanoid_sdk.kuavo_strategy_v2.common.events.mobile_manipulate import (
            EventHeadMoveKeyPoint,
        )

        robot_sdk = get_shared_robot_sdk()
        timeout = int(self.params.get("head_event_timeout", 20000))
        self.head_event = EventHeadMoveKeyPoint(robot_sdk=robot_sdk, timeout=timeout)
        self.head_search_yaw_pitch = []
        if self.mode == "manual":
            yaws_str = self.params.get("head_search_yaws", "")
            head_search_yaws = [float(y) for y in yaws_str.split(",") if y.strip()]
            pitchs_str = self.params.get("head_search_pitchs", "")
            head_search_pitchs = [float(p) for p in pitchs_str.split(",") if p.strip()]
            for yaw_deg, pitch_deg in zip(head_search_yaws, head_search_pitchs):
                self.head_search_yaw_pitch.append(
                    (float(yaw_deg), float(pitch_deg))
                )
        self.head_event.close()
        self.head_event.open()
        self.head_event.cur_head_target_index = 0
        self.head_event.set_target(self.head_search_yaw_pitch)
        self._legacy_ready = True

    def update(self):
        if self.mode == "smoke":
            if _DRY_RUN:
                if not self._smoke_dry_done:
                    yaw = float(self.params.get("yaw_deg", 11.5))
                    pitch = float(self.params.get("pitch_deg", 5.7))
                    self.feedback_message = f"dry-run head yaw={yaw} pitch={pitch}"
                    self._smoke_dry_done = True
                return Status.SUCCESS if self._smoke_dry_done else Status.RUNNING

            if self._skill is None:
                return Status.FAILURE
            if self._skill.is_finished():
                return Status.SUCCESS
            result = self._skill.execute()
            if not result.success:
                self.feedback_message = result.message or "head_control failed"
                return Status.FAILURE
            return Status.RUNNING

        if _DRY_RUN:
            return Status.SUCCESS
        if not self._legacy_ready:
            return Status.FAILURE
        head_status = self.head_event.step()
        if self.head_event.cur_head_target_index >= len(self.head_search_yaw_pitch):
            return Status.SUCCESS
        return Status.RUNNING
