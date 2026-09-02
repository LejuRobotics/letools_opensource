# -*- coding: utf-8 -*-
"""LegJointTimedMove：下肢关节 TimedCmd 薄节点 → leg_joint_timed 原子技能。

与 LegJointSdkMove 参数完全兼容（j0-j3/total_time），仅底层路径不同：
send_leg_joint_timed（服务端 Ruckig 时间同步规划）替代 send_leg_joint_sdk
（Python 100Hz 插值循环下发，折叠臂卡顿根因）。
"""

import os

from py_trees.common import Status

from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.leg_joint_timed import (
    LegJointTimedParams,
    LegJointTimedSkill,
)

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


@define_manifest(
    label="下肢关节控制（TimedCmd）",
    category=["motion", "leg"],
    tree_type="studio_smoke",
    description="调用 hardware.send_leg_joint_timed(joint_angles, desire_time)，服务端 Ruckig 时间同步规划",
    params=[
        {"name": "j0", "type": "float", "default": "14.90", "description": "knee_joint °"},
        {"name": "j1", "type": "float", "default": "-32.01", "description": "leg_joint °"},
        {"name": "j2", "type": "float", "default": "18.03", "description": "waist_pitch_joint °"},
        {"name": "j3", "type": "float", "default": "-90.0", "description": "waist_yaw_joint °"},
        {"name": "total_time", "type": "float", "default": "3.0", "description": "期望执行时间（秒），映射到 desire_time"},
        {"name": "settle_time", "type": "float", "default": "0.5", "description": "运动完成后的额外等待（秒）"},
    ],
    inputs=[],
    outputs=[],
)
class LegJointTimedMove(BaseAction):
    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._skill = None
        self._dry_done = False

    def initialise(self):
        self._dry_done = False
        self._skill = None
        if _DRY_RUN:
            return
        skill_params = LegJointTimedParams(
            joint_angles=[
                float(self.params.get("j0", 14.90)),
                float(self.params.get("j1", -32.01)),
                float(self.params.get("j2", 18.03)),
                float(self.params.get("j3", -90.0)),
            ],
            desire_time=float(self.params.get("total_time", 3.0)),
            settle_time=float(self.params.get("settle_time", 0.5)),
        )
        self._skill = LegJointTimedSkill(hardware=get_shared_hardware())
        result = self._skill.initialize(skill_params)
        if not result.success:
            self.feedback_message = result.message or "leg_joint_timed init failed"

    def update(self):
        if _DRY_RUN:
            if not self._dry_done:
                self.feedback_message = "dry-run leg_joint_timed"
                self._dry_done = True
            return Status.SUCCESS if self._dry_done else Status.RUNNING

        if self._skill is None:
            return Status.FAILURE
        if self._skill.is_finished():
            return Status.SUCCESS
        result = self._skill.execute()
        if not result.success:
            self.feedback_message = result.message or "leg_joint_timed failed"
            return Status.FAILURE
        return Status.RUNNING
