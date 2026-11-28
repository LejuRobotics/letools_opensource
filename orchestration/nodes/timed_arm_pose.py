"""单/双臂MultiCmd定时末端目标，不调用SDK ArmAPI轨迹循环。"""

import json
import os
from typing import Optional
from py_trees.common import Access, Status

from core.domain.result import Result
from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.timed_arm_pose import TimedArmPoseParams, TimedArmPoseSkill


@define_manifest(
    label="定时单/双臂末端目标（MultiCmd）", category=["motion", "arm"], tree_type="studio_smoke",
    description="6维m/deg，local左6右7、world左4右5；服务返回不等于到位。",
    params=[
        {"name": "left_pose", "type": "floatArr", "description": "左臂[x,y,z,yaw,pitch,roll]，m/deg，省略不提交"},
        {"name": "right_pose", "type": "floatArr", "description": "右臂6维m/deg，至少提供一侧"},
        {"name": "desire_time", "type": "float", "required": True, "description": "期望秒数，大于0"},
        {"name": "frame", "type": "string", "default": "local", "description": "local/base_link或world/odom，不接受其他frame"},
        {"name": "is_sync", "type": "bool", "default": True, "description": "规划器执行时长同步，不是到位等待"},
        {"name": "prepare_arm_control", "type": "bool", "default": True,
         "description": "发令前请求模式2，失败不发令；切2可能激活旧目标，须先确认控制器状态"},
        {"name": "focus_ee", "type": "bool", "description": "可选：省略不修改；true末端优先、false躯干优先；全局状态"},
        {"name": "focus_z", "type": "bool", "description": "可选：省略不修改；Z焦点跟随开关；设置失败不发目标，不自动恢复"},
    ], inputs=[], outputs=[],
)
class TimedArmPose(BaseAction):
    def __init__(self, name: str, label: str, namespace: str, params: dict) -> None:
        super().__init__(name, label, namespace, params)
        self._skill: Optional[TimedArmPoseSkill] = None
        self._result: Optional[Result] = None
        self._dry_run = False

    def initialise(self) -> None:
        self._skill = None
        self._result = None
        self._dry_run = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")
        try:
            values = {}
            for key, default in (("left_pose", None), ("right_pose", None),
                                 ("desire_time", None), ("frame", "local"), ("is_sync", True),
                                 ("focus_ee", None), ("focus_z", None), ("prepare_arm_control", True)):
                board_key = self.params.get(f"{key}__board_key")
                if board_key:
                    self.global_blackboard.register_key(key=board_key, access=Access.READ)
                    value = self.global_blackboard.get(board_key)
                else:
                    value = self.params.get(key, default)
                # frame本身就是普通字符串，不应按JSON解码。
                values[key] = json.loads(value) if isinstance(value, str) and key != "frame" else value
            self._skill = TimedArmPoseSkill(hardware=None)
            result = self._skill.initialize(TimedArmPoseParams(**values))
            if not result.success:
                self._result = result
            elif not self._dry_run:
                self._skill.hardware = get_shared_hardware()
        except Exception as exc:
            self._result = Result.fail(f"TimedArmPose 初始化失败: {exc}")

    def update(self) -> Status:
        if self._result is None:
            if self._skill is None:
                self._result = Result.fail("TimedArmPose 未初始化")
            elif self._dry_run:
                self._result = Result.ok("dry-run timed_arm_pose")
            else:
                self._result = self._skill.execute()
        self.feedback_message = self._result.message
        if self._result.success and isinstance(self._result.data, dict):
            self.feedback_message += f"; actual_time={self._result.data.get('actual_time')}s（不是到位确认）"
        return Status.SUCCESS if self._result.success else Status.FAILURE
