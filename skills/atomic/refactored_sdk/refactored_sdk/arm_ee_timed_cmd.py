# -*- coding: utf-8 -*-
"""Atomic skill: arm_ee_timed_cmd.

通过 TimedCmd API (`/mobile_manipulator_timed_single_cmd` 服务) 控制手臂末端位姿。
与 `send_both_ee_poses` (话题直发) 不同，TimedCmd 的 `desireTime` 字段让 C++ 规划器
能在指定时间内平滑规划路径，而不是"马上到"。

对齐 `test_arm_ee_local.py`

支持多 waypoint 时自动拆分: 每个 waypoint 作为一个独立 timed cmd 依次执行。

输入格式: [x, y, z, yaw, pitch, roll] (6D, rpy 度)
"""

from dataclasses import dataclass, field
from typing import List, Optional
import time

from core.common.logger import get_logger
from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.interfaces.i_hardware import IHardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.base.skill_base import SkillBase

logger = get_logger(__name__)


@dataclass
class ArmEETimedCmdParams(SkillParams):
    """TimedCmd 末端轨迹参数。

    约定格式: Pose6D [x, y, z, yaw, pitch, roll] (位置: 米, 姿态: 度)
    """

    skill_name: str = "arm_ee_timed_cmd"
    left_waypoints: List[List[float]] = field(default_factory=list)
    right_waypoints: List[List[float]] = field(default_factory=list)
    desire_time: float = 3.0  # 每段期望执行时间(秒) — 越大越平滑
    frame: str = "local"       # 'world' 或 'local'
    timeout: float = 120.0


@define_manifest(
    label="TimedCmd 末端位姿 (desireTime 平滑规划)",
    category=["motion", "arm"],
    tree_type="studio_smoke",
    description=(
        "通过 TimedCmd 服务控制手臂末端位姿。"
        "desireTime 字段让 C++ Ruckig 规划器在指定时间内平滑计算轨迹。"
        "格式: [x, y, z, yaw, pitch, roll] (米, 度)"
    ),
    params=[
        {"name": "left_waypoints", "type": "json", "default": "",
         "description": "左手关键点 [[x,y,z,yaw,pitch,roll], ...]"},
        {"name": "right_waypoints", "type": "json", "default": "",
         "description": "右手关键点 [[x,y,z,yaw,pitch,roll], ...]"},
        {"name": "desire_time", "type": "float", "default": "3.0",
         "description": "每段期望时间(秒), 越大越平滑"},
        {"name": "frame", "type": "string", "default": "local",
         "description": "坐标系: 'world' / 'local'"},
    ],
    inputs=[],
    outputs=[],
)
class ArmEETimedCmdSkill(SkillBase):
    """TimedCmd 末端位姿控制"""

    def __init__(self, hardware: IHardware):
        super().__init__(name="arm_ee_timed_cmd")
        self.hardware = hardware
        self.params: Optional[ArmEETimedCmdParams] = None
        self._done = False

    def on_initialize(self, params: ArmEETimedCmdParams) -> Result:
        if not isinstance(params, ArmEETimedCmdParams):
            return Result.fail("Invalid parameters for ArmEETimedCmdSkill")
        if not params.left_waypoints or not params.right_waypoints:
            return Result.fail("left_waypoints and right_waypoints required")
        if len(params.left_waypoints) != len(params.right_waypoints):
            return Result.fail(
                f"left/right waypoints count mismatch: "
                f"{len(params.left_waypoints)} vs {len(params.right_waypoints)}")
        for side, wps in [("left", params.left_waypoints), ("right", params.right_waypoints)]:
            for i, wp in enumerate(wps):
                if len(wp) != 6:
                    return Result.fail(
                        f"{side}_waypoints[{i}]: expected [x,y,z,yaw,pitch,roll], got {len(wp)} values")
        self.params = params
        self._done = False
        return Result.ok()

    def on_execute(self) -> Result:
        if self._done:
            return Result.ok("ArmEETimedCmdSkill already finished")

        left_wps = self.params.left_waypoints
        right_wps = self.params.right_waypoints
        desire_time = float(self.params.desire_time)
        frame = self.params.frame

        # 选择 TimedCmd 方法
        fn_name = {
            "local": "send_arm_ee_local_timed",
            "world": "send_arm_ee_world_timed",
        }.get(frame, "send_arm_ee_local_timed")

        fn = getattr(self.hardware, fn_name, None)
        if fn is None:
            self._done = True
            return Result.fail(f"Hardware missing {fn_name}()")

        try:
            for i, (lwp, rwp) in enumerate(zip(left_wps, right_wps)):
                logger.info(
                    "[timed_cmd] waypoint %d/%d: left=%s right=%s time=%.1fs",
                    i + 1, len(left_wps),
                    [f"{v:.2f}" for v in lwp],
                    [f"{v:.2f}" for v in rwp],
                    desire_time,
                )
                result = fn(
                    left_pose=list(lwp),
                    right_pose=list(rwp),
                    desire_time=desire_time,
                )
                if not result.success:
                    self._done = True
                    return Result.fail(
                        f"TimedCmd waypoint {i} failed: {result.message}")

                # 等待执行完成 + 缓冲
                time.sleep(desire_time + 1.0)

            # ── 后置: 释放 arm_control_mode=0 ──
            logger.info("[timed_cmd] 后置: set_arm_control_mode(0) — 保持当前位姿")
            self.hardware.set_arm_control_mode(0)
            time.sleep(0.5)

        except Exception as e:
            self._done = True
            logger.error(f"[timed_cmd] ❌ 异常: {e}", exc_info=True)
            return Result.fail(f"arm_ee_timed_cmd error: {e}")

        self._done = True
        logger.info("[timed_cmd] ✅ 全部 waypoint 执行完成")
        return Result.ok(f"{len(left_wps)} waypoints via TimedCmd done")

    def on_is_finished(self) -> bool:
        return self._done
