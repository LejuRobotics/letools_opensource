"""SetHeadPoseSkill：设置头部姿态。"""

import math
from dataclasses import dataclass
from typing import Optional

from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.interfaces.i_hardware import IHardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.base.skill_base import SkillBase


@dataclass
class SetHeadPoseParams(SkillParams):
    """yaw/pitch 均必填，单位固定为度，不受 angle_unit 配置影响。"""

    skill_name: str = "set_head_pose"
    yaw: Optional[float] = None
    pitch: Optional[float] = None


@define_manifest(
    label="设置头部姿态",
    category=["motion", "head"],
    tree_type="studio_smoke",
    description="设置头部姿态 yaw/pitch（deg）。",
    params=[
        {"name": "yaw", "type": "float", "required": True, "description": "yaw（deg）。"},
        {"name": "pitch", "type": "float", "required": True, "description": "pitch（deg）。"},
    ],
    inputs=[],
    outputs=[],
)
class SetHeadPoseSkill(SkillBase):
    """Skill → IHardware.set_head_pose → Mixin → LowLevelSDKManager → SDK。"""

    def __init__(self, hardware: IHardware) -> None:
        super().__init__(name="set_head_pose")
        self.hardware = hardware
        self.params: Optional[SetHeadPoseParams] = None
        self._result: Optional[Result] = None

    def on_initialize(self, params: SetHeadPoseParams) -> Result:
        self.params = None
        self._result = None
        if not isinstance(params, SetHeadPoseParams):
            return Result.fail("Invalid parameters for SetHeadPoseSkill")
        try:
            yaw, pitch = float(params.yaw), float(params.pitch)
        except (TypeError, ValueError, OverflowError):
            return Result.fail("yaw/pitch 均必填，且必须是有限数字（度）")
        if not math.isfinite(yaw) or not math.isfinite(pitch):
            return Result.fail("yaw/pitch 必须是有限数字（度）")
        self.params = params
        return Result.ok()

    def on_execute(self) -> Result:
        if self._result is not None:
            return self._result
        if self.params is None:
            return Result.fail("SetHeadPoseSkill 未成功初始化")
        self._result = self.hardware.set_head_pose(
            yaw=float(self.params.yaw), pitch=float(self.params.pitch)
        )
        return self._result

    def on_is_finished(self) -> bool:
        return self._result is not None