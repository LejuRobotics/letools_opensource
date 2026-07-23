# -*- coding: utf-8 -*-
"""Atomic skill: chassis_stop — 底盘停止（切换速度控制权限）。

包装 `hardware.enable_vel_control_jibot(enable)` 接口，调用 /enable_vel_control 服务
（std_srvs/SetBool）切换底盘控制权限。

enable=True  → 速度控制模式开启，导航模块释放底盘（用于手动接管 / 紧急停止导航运动）
enable=False → 导航模块接管底盘

对齐 kuavo 原始掉落处理中的 `set_enable_vel_control(True, force=True)`：
检测到掉落时调用 enable=True 停止导航运动。
"""

from dataclasses import dataclass
from typing import Optional

from core.common.logger import get_logger
from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.interfaces.i_hardware import IHardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.base.skill_base import SkillBase

logger = get_logger(__name__)


@dataclass
class ChassisStopParams(SkillParams):
    """底盘停止参数。

    enable: True=停止导航（速度控制模式开启），False=交还导航控制
    """

    skill_name: str = "chassis_stop"
    enable: bool = True
    timeout: float = 5.0


@define_manifest(
    label="底盘停止（速度控制切换）",
    category=["motion", "chassis", "safety"],
    tree_type="studio_smoke",
    description="调用 /enable_vel_control 切换底盘控制权限，enable=True 停止导航运动",
    params=[
        {"name": "enable", "type": "bool", "default": True,
         "description": "True=停止导航(速度控制开启), False=交还导航控制"},
    ],
    inputs=[],
    outputs=[],
)
class ChassisStopSkill(SkillBase):
    """底盘停止技能 —— 一次性操作，调用 enable_vel_control_jibot 切换控制权限。"""

    def __init__(self, hardware: IHardware):
        super().__init__(name="chassis_stop")
        self.hardware = hardware
        self.params: Optional[ChassisStopParams] = None
        self._done = False
        self._result: Optional[Result] = None

    def on_initialize(self, params: ChassisStopParams) -> Result:
        if not isinstance(params, ChassisStopParams):
            return Result.fail("Invalid parameters for ChassisStopSkill")
        self.params = params
        self._done = False
        self._result = None
        return Result.ok()

    def on_execute(self) -> Result:
        if self._done:
            return self._result or Result.ok("ChassisStopSkill already finished")

        fn = getattr(self.hardware, "enable_vel_control_jibot", None)
        if fn is None:
            self._done = True
            self._result = Result.fail("Hardware does not implement enable_vel_control_jibot()")
            return self._result

        enable = self.params.enable
        result = fn(enable)
        self._done = True

        if result.success:
            action_desc = "停止导航" if enable else "交还导航控制"
            logger.info("[chassis_stop] %s: %s", action_desc, result.message or "成功")
        else:
            logger.error("[chassis_stop] enable_vel_control(%s) 失败: %s", enable, result.message)

        self._result = result
        return self._result

    def on_is_finished(self) -> bool:
        return self._done
