# -*- coding: utf-8 -*-
"""Atomic skill: leg_joint_timed.

下肢（折叠臂）关节控制 TimedCmd 路径：hardware.send_leg_joint_timed(joint_angles, desire_time)。

与 leg_joint_sdk 的区别（卡顿修复）:
- SDK 路径: move_wheel_lower_joint_auto 在 Python 侧 100Hz 插值循环下发，
  每条消息触发服务端 desiredTime=0 最短时间 Ruckig 重规划 → 走停卡顿。
- TimedCmd 路径: 一次下发目标关节角 + desire_time，服务端（planner_index=3）
  按 desiredTime 时间同步规划完整轨迹后连续执行。
- 服务端规划完成即返回 actualTime（非阻塞），本技能发送成功后
  sleep(actual_time + settle_time) 等待运动完成，保持节点在运动期间 RUNNING。

对齐 `timed/03_leg/test_leg_joint.py`：关节顺序 [knee, leg, waist_pitch, waist_yaw]，单位度。
"""

import time
from dataclasses import dataclass, field
from typing import List, Optional

from core.common.logger import get_logger
from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.interfaces.i_hardware import IHardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.base.skill_base import SkillBase

logger = get_logger(__name__)


@dataclass
class LegJointTimedParams(SkillParams):
    """对齐 timed/03_leg 测试：hardware.send_leg_joint_timed(joint_angles, desire_time)。"""

    skill_name: str = "leg_joint_timed"
    joint_angles: List[float] = field(default_factory=lambda: [14.90, -32.01, 18.03, -90.0])
    desire_time: float = 3.0
    settle_time: float = 0.5   # actual_time 之外的额外等待（秒）
    timeout: float = 60.0


@define_manifest(
    label="下肢关节控制（TimedCmd）",
    category=["motion", "leg"],
    tree_type="studio_smoke",
    description="调用 hardware.send_leg_joint_timed(joint_angles, desire_time)，服务端 Ruckig 时间同步规划",
    params=[
        {
            "name": "joint_angles",
            "type": "floatArr",
            "default": "14.90,-32.01,18.03,-90.0",
            "description": "4 个关节角 [knee, leg, waist_pitch, waist_yaw]（deg）",
        },
        {"name": "desire_time", "type": "float", "default": "3.0", "description": "期望执行时间（秒）"},
        {"name": "settle_time", "type": "float", "default": "0.5", "description": "运动完成后的额外等待（秒）"},
    ],
    inputs=[],
    outputs=[],
)
class LegJointTimedSkill(SkillBase):
    """下肢关节控制（TimedCmd）：一次下发目标关节角，服务端规划完整轨迹。"""

    def __init__(self, hardware: IHardware):
        super().__init__(name="leg_joint_timed")
        self.hardware = hardware
        self.params: Optional[LegJointTimedParams] = None
        self._done = False

    def on_initialize(self, params: LegJointTimedParams) -> Result:
        if not isinstance(params, LegJointTimedParams):
            return Result.fail("Invalid parameters for LegJointTimedSkill")
        if len(params.joint_angles) != 4:
            return Result.fail(
                f"leg_joint_timed expects 4 joint angles, got {len(params.joint_angles)}"
            )
        self.params = params
        self._done = False
        return Result.ok()

    def on_execute(self) -> Result:
        if self._done:
            return Result.ok("LegJointTimedSkill already finished")

        fn = getattr(self.hardware, "send_leg_joint_timed", None)
        if fn is None:
            self._done = True
            return Result.fail("Hardware does not implement send_leg_joint_timed()")

        p = self.params
        start_ts = time.monotonic()
        logger.debug(
            "[Perf][leg_joint_timed] hardware_call_start angles=%s desire_time=%.3fs",
            list(p.joint_angles),
            float(p.desire_time),
        )
        result = fn(
            joint_angles=list(p.joint_angles),
            desire_time=float(p.desire_time),
        )

        if not result.success:
            self._done = True
            elapsed = time.monotonic() - start_ts
            logger.info(
                "[Perf][leg_joint_timed] hardware_call_done success=%s elapsed=%.3fs expected=%.3fs",
                result.success,
                elapsed,
                float(p.desire_time),
            )
            return result

        # 服务规划完即返回（非阻塞）：等待 actual_time + settle_time 让运动执行完成
        actual_time = float((result.data or {}).get("actual_time", p.desire_time))
        time.sleep(actual_time + float(p.settle_time))

        elapsed = time.monotonic() - start_ts
        logger.info(
            "[Perf][leg_joint_timed] hardware_call_done success=%s elapsed=%.3fs expected=%.3fs angles=%s desire_time=%.3fs actual_time=%.3fs",
            result.success,
            elapsed,
            float(p.desire_time),
            list(p.joint_angles),
            float(p.desire_time),
            actual_time,
        )
        self._done = True
        return result

    def on_is_finished(self) -> bool:
        return self._done
