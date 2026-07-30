# -*- coding: utf-8 -*-
"""Atomic skill: arm_ee_single_timed.

轮臂末端独立控制 - 单次末端位姿（TimedCmd 路径，planner 4/5/6/7）。
躯干/底盘保持不动（初始化由脚手架/编排层负责，本技能只下发末端指令）。

对齐源脚本 cmd_arm_ee_only_test.py 的核心指令下发逻辑。
"""

from dataclasses import dataclass, field
from typing import List, Optional

from core.common.logger import get_logger
from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.interfaces.i_hardware import IHardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.base.skill_base import SkillBase

logger = get_logger(__name__)

# 默认位姿: [x, y, z, yaw, pitch, roll]（位置米，姿态用户单位/默认度）
_DEFAULT_POSE = [0.3, 0.25, 0.5, 0.0, 0.0, 0.0]


@dataclass
class ArmEESingleTimedParams(SkillParams):
    """单次末端位姿参数。

    约定: pose [x, y, z, yaw, pitch, roll]，位置米；姿态单位由适配器 angle_unit 配置决定（默认度）。
    """

    skill_name: str = "arm_ee_single_timed"
    side: str = "left"          # 'left' / 'right'
    frame: str = "world"        # 'world' / 'local'
    pose: List[float] = field(default_factory=lambda: list(_DEFAULT_POSE))
    desire_time: float = 3.0
    timeout: float = 30.0


@define_manifest(
    label="单次末端位姿（TimedCmd）",
    category=["motion", "arm"],
    tree_type="studio_smoke",
    description="单次末端位姿指令（planner 4/5/6/7），躯干/底盘不动，格式 [x,y,z,yaw,pitch,roll]",
    params=[
        {"name": "side", "type": "string", "default": "left", "description": "手臂侧: 'left' / 'right'"},
        {"name": "frame", "type": "string", "default": "world", "description": "坐标系: 'world' / 'local'"},
        {"name": "pose", "type": "json", "default": "[0.3,0.25,0.5,0,0,0]",
         "description": "末端位姿 [x,y,z,yaw,pitch,roll]（米, 度）"},
        {"name": "desire_time", "type": "float", "default": "3.0", "description": "期望执行时间（秒）"},
    ],
    inputs=[],
    outputs=[],
)
class ArmEESingleTimedSkill(SkillBase):
    """单次末端位姿控制（TimedCmd 路径，planner 4/5/6/7）。

    不负责初始化（focus_ee=False、set_control_mode、复位等），由上层编排/脚手架负责。
    """

    def __init__(self, hardware: IHardware):
        super().__init__(name="arm_ee_single_timed")
        self.hardware = hardware
        self.params: Optional[ArmEESingleTimedParams] = None
        self._done = False

    def on_initialize(self, params: ArmEESingleTimedParams) -> Result:
        if not isinstance(params, ArmEESingleTimedParams):
            return Result.fail("Invalid parameters for ArmEESingleTimedSkill")

        if params.side not in ("left", "right"):
            return Result.fail(f"side 必须是 'left' 或 'right'，收到: {params.side}")
        if params.frame not in ("world", "local"):
            return Result.fail(f"frame 必须是 'world' 或 'local'，收到: {params.frame}")
        if not params.pose or len(params.pose) != 6:
            return Result.fail(
                f"pose 需要 6 个值 [x,y,z,yaw,pitch,roll]，收到 {len(params.pose) if params.pose else 0} 个")

        self.params = params
        self._done = False
        return Result.ok()

    def on_execute(self) -> Result:
        if self._done:
            return Result.ok("ArmEESingleTimedSkill already finished")

        fn_name = f"send_timed_{self.params.side}_arm_ee_{self.params.frame}"
        fn = getattr(self.hardware, fn_name, None)
        if fn is None:
            self._done = True
            return Result.fail(f"Hardware does not implement {fn_name}()")

        result = fn(pose=list(self.params.pose), desire_time=float(self.params.desire_time))
        self._done = True
        if result.success:
            logger.info(
                "arm_ee_single_timed: %s臂/%s系 pos=[%.2f,%.2f,%.2f] t=%.2fs",
                self.params.side, self.params.frame,
                self.params.pose[0], self.params.pose[1], self.params.pose[2],
                float(self.params.desire_time),
            )
        return result

    def on_is_finished(self) -> bool:
        return self._done
