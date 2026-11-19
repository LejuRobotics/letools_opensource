"""GetHeadPoseSkill：获取头部姿态。"""

from dataclasses import dataclass
from typing import Optional

from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.interfaces.i_hardware import IHardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.base.skill_base import SkillBase


@dataclass
class GetHeadPoseParams(SkillParams):
    """无业务输入；继承技能生命周期通用参数。"""

    skill_name: str = "get_head_pose"


@define_manifest(
    label="获取头部姿态",
    category=["motion", "head"],
    tree_type="studio_smoke",
    description="获取头部姿态 yaw/pitch（deg）。",
    params=[],
    inputs=[],
    outputs=[{"name": "head_pose", "type": "json", "description": "获取到 yaw/pitch（deg）。"}],
)
class GetHeadPoseSkill(SkillBase):
    """Skill → IHardware.get_head_pose → Mixin → LowLevelSDKManager → SDK。"""

    def __init__(self, hardware: IHardware) -> None:
        super().__init__(name="get_head_pose")
        self.hardware = hardware
        self.params: Optional[GetHeadPoseParams] = None
        self._result: Optional[Result] = None

    def on_initialize(self, params: GetHeadPoseParams) -> Result:
        self.params = None
        self._result = None
        if not isinstance(params, GetHeadPoseParams):
            return Result.fail("Invalid parameters for GetHeadPoseSkill")
        self.params = params
        return Result.ok()

    def on_execute(self) -> Result:
        if self._result is not None:
            return self._result
        if self.params is None:
            return Result.fail("GetHeadPoseSkill 未成功初始化")
        self._result = self.hardware.get_head_pose()
        return self._result

    def on_is_finished(self) -> bool:
        return self._result is not None