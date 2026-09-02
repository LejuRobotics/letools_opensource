# -*- coding: utf-8 -*-
"""Atomic skill: arm_ee_dual_timed.

轮臂末端独立控制 - 双臂单次末端位姿（TimedCmd 路径，planner 4+5/6+7）。

安全约束：
- 运行前设置 focus_ee=False、focus_z=False，避免躯干参与末端结算；
- 只切换手臂外部控制模式，不做手臂物理复位；
- 执行完成后保持手臂当前位置，不复位手臂；
- 不调用 reset_torso_to_initial，躯干不复位。

对齐 apps/test_kuavo_5w_sdk_adapter/timed/04_arm/test_arm_ee_dual_timed.py 的双臂末端指令下发逻辑。
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

_DEFAULT_LEFT_POSE = [0.5, 0.25, 0.7, 0.0, -90.0, 0.0]
_DEFAULT_RIGHT_POSE = [0.5, -0.25, 0.7, 0.0, -90.0, 0.0]


@dataclass
class ArmEEDualTimedParams(SkillParams):
    """双臂单次末端位姿参数。"""

    skill_name: str = "arm_ee_dual_timed"
    frame: str = "world"        # 'world' / 'local'
    left_pose: List[float] = field(default_factory=lambda: list(_DEFAULT_LEFT_POSE))
    right_pose: List[float] = field(default_factory=lambda: list(_DEFAULT_RIGHT_POSE))
    desire_time: float = 2.0
    focus_ee: bool = False
    focus_z: bool = False
    prepare_arm_control: bool = True
    release_arm_control: bool = True
    timeout: float = 30.0


@define_manifest(
    label="双臂单次末端位姿（TimedCmd/躯干不参与）",
    category=["motion", "arm"],
    tree_type="studio_smoke",
    description=(
        "双臂单次末端位姿指令（planner 4+5/6+7）。"
        "默认 focus_ee=False/focus_z=False，躯干不参与结算；执行后不复位手臂、不复位躯干。"
    ),
    params=[
        {"name": "frame", "type": "string", "default": "world", "description": "坐标系: 'world' / 'local'"},
        {"name": "left_pose", "type": "json", "default": "[0.5,0.25,0.7,0,-90,0]",
         "description": "左臂末端位姿 [x,y,z,yaw,pitch,roll]（米, 度）"},
        {"name": "right_pose", "type": "json", "default": "[0.5,-0.25,0.7,0,-90,0]",
         "description": "右臂末端位姿 [x,y,z,yaw,pitch,roll]（米, 度）"},
        {"name": "desire_time", "type": "float", "default": "2.0", "description": "期望执行时间（秒）"},
        {"name": "focus_ee", "type": "bool", "default": "False",
         "description": "False=躯干优先/躯干不被末端带动；默认 False"},
        {"name": "focus_z", "type": "bool", "default": "False",
         "description": "False=禁用 Z 轴焦点跟随；默认 False"},
        {"name": "prepare_arm_control", "type": "bool", "default": "True",
         "description": "执行前切换到手臂外部控制模式 set_arm_control_mode(2)，不做手臂复位"},
        {"name": "release_arm_control", "type": "bool", "default": "True",
         "description": "执行完成后释放外部末端保持 set_arm_control_mode(0)，不复位手臂"},
    ],
    inputs=[],
    outputs=[],
)
class ArmEEDualTimedSkill(SkillBase):
    """双臂单次末端位姿控制（TimedCmd 路径，planner 4+5/6+7）。"""

    def __init__(self, hardware: IHardware):
        super().__init__(name="arm_ee_dual_timed")
        self.hardware = hardware
        self.params: Optional[ArmEEDualTimedParams] = None
        self._done = False

    def on_initialize(self, params: ArmEEDualTimedParams) -> Result:
        if not isinstance(params, ArmEEDualTimedParams):
            return Result.fail("Invalid parameters for ArmEEDualTimedSkill")
        if params.frame not in ("world", "local"):
            return Result.fail(f"frame 必须是 'world' 或 'local'，收到: {params.frame}")
        if not params.left_pose or len(params.left_pose) != 6:
            return Result.fail(
                f"left_pose 需要 6 个值 [x,y,z,yaw,pitch,roll]，收到 {len(params.left_pose) if params.left_pose else 0} 个")
        if not params.right_pose or len(params.right_pose) != 6:
            return Result.fail(
                f"right_pose 需要 6 个值 [x,y,z,yaw,pitch,roll]，收到 {len(params.right_pose) if params.right_pose else 0} 个")

        self.params = params
        self._done = False
        return Result.ok()

    def on_execute(self) -> Result:
        if self._done:
            return Result.ok("ArmEEDualTimedSkill already finished")

        total_start_ts = time.monotonic()
        setup_result = self._prepare_without_reset()
        if not setup_result.success:
            self._done = True
            return setup_result

        fn_name = f"send_arm_ee_{self.params.frame}_timed"
        fn = getattr(self.hardware, fn_name, None)
        if fn is None:
            self._done = True
            return Result.fail(f"Hardware does not implement {fn_name}()")

        send_start_ts = time.monotonic()
        logger.info(
            "[Perf][arm_ee_dual_timed] timed_cmd_start frame=%s desire_time=%.3fs prepare=%s release=%s",
            self.params.frame,
            float(self.params.desire_time),
            self.params.prepare_arm_control,
            self.params.release_arm_control,
        )
        result = fn(
            left_pose=list(self.params.left_pose),
            right_pose=list(self.params.right_pose),
            desire_time=float(self.params.desire_time),
        )
        send_elapsed = time.monotonic() - send_start_ts
        logger.info(
            "[Perf][arm_ee_dual_timed] timed_cmd_done success=%s elapsed=%.3fs desire_time=%.3fs",
            result.success,
            send_elapsed,
            float(self.params.desire_time),
        )
        if result.success:
            sleep_start_ts = time.monotonic()
            time.sleep(float(self.params.desire_time) + 0.2)
            logger.info(
                "[Perf][arm_ee_dual_timed] post_sleep_done elapsed=%.3fs planned=%.3fs",
                time.monotonic() - sleep_start_ts,
                float(self.params.desire_time) + 0.2,
            )
            release_result = self._release_external_hold()
            if not release_result.success:
                self._done = True
                return release_result

        self._done = True
        logger.info("[Perf][arm_ee_dual_timed] total_elapsed=%.3fs", time.monotonic() - total_start_ts)
        if result.success:
            logger.info(
                "arm_ee_dual_timed: %s系 left=[%.2f,%.2f,%.2f] right=[%.2f,%.2f,%.2f] t=%.2fs；执行后释放末端保持，不复位手臂/躯干",
                self.params.frame,
                self.params.left_pose[0], self.params.left_pose[1], self.params.left_pose[2],
                self.params.right_pose[0], self.params.right_pose[1], self.params.right_pose[2],
                float(self.params.desire_time),
            )
        return result

    def _prepare_without_reset(self) -> Result:
        """准备末端控制状态，不做手臂复位和躯干复位。"""
        if hasattr(self.hardware, "set_focus_ee"):
            focus_ee_start_ts = time.monotonic()
            result = self.hardware.set_focus_ee(bool(self.params.focus_ee))
            logger.info(
                "[Perf][arm_ee_dual_timed] set_focus_ee elapsed=%.3fs success=%s value=%s",
                time.monotonic() - focus_ee_start_ts,
                result.success,
                self.params.focus_ee,
            )
            if not result.success:
                return Result.fail(f"set_focus_ee({self.params.focus_ee}) failed: {result.message}")

        if hasattr(self.hardware, "set_focus_z"):
            focus_z_start_ts = time.monotonic()
            result = self.hardware.set_focus_z(bool(self.params.focus_z))
            logger.info(
                "[Perf][arm_ee_dual_timed] set_focus_z elapsed=%.3fs success=%s value=%s",
                time.monotonic() - focus_z_start_ts,
                result.success,
                self.params.focus_z,
            )
            if not result.success:
                return Result.fail(f"set_focus_z({self.params.focus_z}) failed: {result.message}")

        if self.params.prepare_arm_control:
            fn = getattr(self.hardware, "set_arm_control_mode", None)
            if fn is None:
                return Result.fail("Hardware does not implement set_arm_control_mode()")
            mode_start_ts = time.monotonic()
            result = fn(2)
            logger.info(
                "[Perf][arm_ee_dual_timed] set_arm_control_mode mode=2 elapsed=%.3fs success=%s",
                time.monotonic() - mode_start_ts,
                result.success,
            )
            if not result.success:
                return Result.fail(f"set_arm_control_mode(2) failed: {result.message}")

        logger.info(
            "arm_ee_dual_timed 前置完成: focus_ee=%s, focus_z=%s, prepare_arm_control=%s；不复位手臂/躯干",
            self.params.focus_ee, self.params.focus_z, self.params.prepare_arm_control,
        )
        return Result.ok()

    def _release_external_hold(self) -> Result:
        """释放外部末端保持，让后续躯干动作不再触发手臂末端补偿。"""
        if not self.params.release_arm_control:
            return Result.ok()

        fn = getattr(self.hardware, "set_arm_control_mode", None)
        if fn is None:
            return Result.fail("Hardware does not implement set_arm_control_mode()")

        mode_start_ts = time.monotonic()
        result = fn(0)
        logger.info(
            "[Perf][arm_ee_dual_timed] set_arm_control_mode mode=0 elapsed=%.3fs success=%s",
            time.monotonic() - mode_start_ts,
            result.success,
        )
        if not result.success:
            return Result.fail(f"set_arm_control_mode(0) failed: {result.message}")

        logger.info("arm_ee_dual_timed 后置完成: set_arm_control_mode(0)，释放末端保持，不复位手臂")
        time.sleep(0.2)
        return Result.ok()

    def on_is_finished(self) -> bool:
        return self._done
