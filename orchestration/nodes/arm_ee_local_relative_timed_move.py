# -*- coding: utf-8 -*-
"""ArmEeLocalRelativeTimedMove：双臂末端 local 相对补偿薄节点。

只从黑板读取 6 个参数：offset_x / offset_y / offset_z / fixed_yaw / fixed_pitch / fixed_roll。
其余运行参数（desire_time、tf、topic、frame、release_after 等）固定在 skill 模块常量中，
与 apps/test_kuavo_5w_sdk_adapter/timed/04_arm/test_arm_ee_local_relative_servo.py 一致。
"""

import os

from py_trees.common import Status

from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.arm_ee_local_relative_timed import (
    ArmEELocalRelativeTimedParams,
    ArmEELocalRelativeTimedSkill,
)

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


@define_manifest(
    label="双臂末端 local 相对补偿（TimedCmd）",
    category=["motion", "arm"],
    tree_type="studio_smoke",
    description=(
        "读取当前 /humanoid_wheel/eePoses，经 TF 转为 base_link/local，"
        "叠加 x/y/z offset，姿态使用固定 yaw/pitch/roll，然后下发 send_arm_ee_local_timed。"
        "其余运行参数固定与示例脚本一致。"
    ),
    params=[
        {"name": "offset_x", "type": "float", "default": "0.05", "description": "local x 偏移，正值向前，单位 m"},
        {"name": "offset_y", "type": "float", "default": "0.0", "description": "local y 偏移，正值向左，单位 m"},
        {"name": "offset_z", "type": "float", "default": "-0.05", "description": "local z 偏移，正值向上，单位 m"},
        {"name": "fixed_yaw", "type": "float", "default": "0.0", "description": "目标 yaw，单位 deg"},
        {"name": "fixed_pitch", "type": "float", "default": "-90.0", "description": "目标 pitch，单位 deg"},
        {"name": "fixed_roll", "type": "float", "default": "0.0", "description": "目标 roll，单位 deg"},
    ],
    inputs=[],
    outputs=[],
)
class ArmEeLocalRelativeTimedMove(BaseAction):
    """双臂末端 local/base_link 相对补偿节点。

    用法示例 (py_tree_child.json)，只需在黑板中配置 6 个参数：
    {
      "name": "ArmEeLocalRelativeTimedMove",
      "label": "ee_local_relative_1",
      "params": {
        "offset_x":   { "source": "READ_BOARD", "board_key": "ee_local_relative_1_offset_x" },
        "offset_y":   { "source": "READ_BOARD", "board_key": "ee_local_relative_1_offset_y" },
        "offset_z":   { "source": "READ_BOARD", "board_key": "ee_local_relative_1_offset_z" },
        "fixed_yaw":  { "source": "READ_BOARD", "board_key": "ee_local_relative_1_fixed_yaw" },
        "fixed_pitch":{ "source": "READ_BOARD", "board_key": "ee_local_relative_1_fixed_pitch" },
        "fixed_roll": { "source": "READ_BOARD", "board_key": "ee_local_relative_1_fixed_roll" }
      }
    }
    """

    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._skill = None
        self._dry_done = False

    def initialise(self):
        self._skill = None
        self._dry_done = False

        skill_params = ArmEELocalRelativeTimedParams(
            offset_x=float(self.params.get("offset_x", 0.05)),
            offset_y=float(self.params.get("offset_y", 0.0)),
            offset_z=float(self.params.get("offset_z", -0.05)),
            fixed_yaw=float(self.params.get("fixed_yaw", 0.0)),
            fixed_pitch=float(self.params.get("fixed_pitch", -90.0)),
            fixed_roll=float(self.params.get("fixed_roll", 0.0)),
            timeout=float(self.params.get("timeout", 30.0)),
        )

        if _DRY_RUN:
            self.feedback_message = (
                "dry-run arm_ee_local_relative_timed "
                f"offset=({skill_params.offset_x},{skill_params.offset_y},{skill_params.offset_z}) "
                f"fixed=({skill_params.fixed_yaw},{skill_params.fixed_pitch},{skill_params.fixed_roll})"
            )
            self._dry_done = True
            return

        self._skill = ArmEELocalRelativeTimedSkill(hardware=get_shared_hardware())
        result = self._skill.initialize(skill_params)
        if not result.success:
            self.feedback_message = result.message or "arm_ee_local_relative_timed init failed"

    def update(self):
        if _DRY_RUN:
            return Status.SUCCESS if self._dry_done else Status.FAILURE
        if self._skill is None:
            return Status.FAILURE
        if self._skill.is_finished():
            return Status.SUCCESS
        result = self._skill.execute()
        if not result.success:
            self.feedback_message = result.message or "arm_ee_local_relative_timed failed"
            return Status.FAILURE
        return Status.RUNNING
