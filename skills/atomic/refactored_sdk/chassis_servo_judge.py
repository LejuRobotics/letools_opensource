# -*- coding: utf-8 -*-
"""Atomic skill: chassis_servo_judge —— 底盘伺服判断（观测量 → 收敛/继续/超限失败）。

飞书文档 §3.3 / §4.4 的「判断」模块：拿当前观测量判断伺服该收敛、继续还是失败。

收敛判据（§4.4）：
    连续 servo_converge_frames（默认 2）帧满足
        max(|e_bottom_px|, |e_right_px|) < servo_stop_threshold_px
        且 |theta_rad| < servo_stop_theta_rad
    → CONVERGED

超限失败（§4.4）：
    iter_index >= servo_max_iters（8）仍未收敛      → FAILED_ITER
    cumulative_xy_m > servo_cumulative_limit_m（0.10） → FAILED_CUM

本技能为纯计算无 hardware（遵循 PalletServoSkill 先例 + 飞书附录三「无 get_shared_hardware() 依赖」）。
跨帧的「连续收敛帧计数」由调用方（节点层）维护并作为 prev_converge_count 传入，
技能本身无状态，可直接构造单测。
"""

from dataclasses import dataclass
from typing import Optional

from core.common.logger import get_logger
from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.domain.servo_types import AlignObservation, observation_field
from orchestration.utils.manifest_decorators import define_manifest
from skills.base.skill_base import SkillBase

logger = get_logger(__name__)

# 判决结果常量（写入 Result.data["verdict"]）
VERDICT_CONVERGED = "converged"
VERDICT_CONTINUE = "continue"
VERDICT_FAILED_ITER = "failed_iter"
VERDICT_FAILED_CUM = "failed_cum"


@dataclass
class ChassisServoJudgeParams(SkillParams):
    """一次伺服判断的输入（飞书 §4.4）。

    prev_converge_count：截至上一帧已连续满足收敛条件的帧数（由节点层跨帧维护）。
    其余阈值参数沿用 §4.6 board.json（全部 READ_BOARD，不硬编码）。
    """

    skill_name: str = "chassis_servo_judge"
    observation: Optional[AlignObservation] = None
    iter_index: int = 0
    cumulative_xy_m: float = 0.0
    prev_converge_count: int = 0
    # 收敛判据（§4.4）
    servo_stop_threshold_px: float = 10.0   # 现场标定，board.json 覆盖
    servo_stop_theta_rad: float = 0.02
    servo_converge_frames: int = 2
    servo_max_iters: int = 8
    servo_cumulative_limit_m: float = 0.10
    timeout: float = 5.0


@define_manifest(
    label="底盘伺服判断",
    category=["skill", "chassis", "servo"],
    tree_type="studio_smoke",
    description="观测量 → 收敛/继续/超限失败：连续帧判据 + 迭代超限 + 累积量安全保护（纯计算）",
    params=[
        {"name": "servo_stop_threshold_px", "type": "float", "default": "10.0",
         "description": "收敛像素阈值（现场标定）"},
        {"name": "servo_stop_theta_rad", "type": "float", "default": "0.02",
         "description": "角度收敛阈值(rad)"},
        {"name": "servo_converge_frames", "type": "int", "default": "2",
         "description": "连续收敛帧数"},
        {"name": "servo_max_iters", "type": "int", "default": "8", "description": "最大迭代次数"},
        {"name": "servo_cumulative_limit_m", "type": "float", "default": "0.10",
         "description": "累积位移安全保护上限(m)"},
    ],
    inputs=[{"name": "latest_servo_error", "type": "object",
             "description": "AlignObservation：两路像素平移偏差 + 偏差角 + valid"}],
    outputs=[{"name": "latest_servo_verdict", "type": "object",
              "description": "判决：verdict + converge_count + reason"}],
)
class ChassisServoJudgeSkill(SkillBase):
    """底盘伺服判断：观测量 → 收敛/继续/超限失败，纯计算无硬件依赖。"""

    def __init__(self):
        super().__init__(name="chassis_servo_judge")
        self.params: Optional[ChassisServoJudgeParams] = None
        self._done = False
        self._result: Optional[Result] = None

    def on_initialize(self, params: ChassisServoJudgeParams) -> Result:
        if not isinstance(params, ChassisServoJudgeParams):
            return Result.fail("Invalid parameters for ChassisServoJudgeSkill")
        self.params = params
        self._done = False
        self._result = None
        return Result.ok()

    def on_execute(self) -> Result:
        if self._done:
            return self._result or Result.ok("ChassisServoJudgeSkill already finished")

        p = self.params
        obs = p.observation
        if obs is None:
            self._done = True
            self._result = Result.fail("没有观测量（AlignObservation 为空）")
            return self._result
        if not observation_field(obs, "valid", True):
            self._done = True
            self._result = Result.fail(
                f"观测无效：{observation_field(obs, 'reject', '') or 'unknown'}",
                error_code="invalid_observation",
            )
            return self._result

        # 安全保护优先（§4.4）：累积量超限立即终止
        if p.cumulative_xy_m > p.servo_cumulative_limit_m:
            self._done = True
            self._result = self._verdict(VERDICT_FAILED_CUM, 0, "累积位移超限")
            return self._result

        # 迭代超限（§4.4）：达 max_iters 仍未收敛
        if p.iter_index >= p.servo_max_iters:
            self._done = True
            self._result = self._verdict(VERDICT_FAILED_ITER, 0, "迭代超限未收敛")
            return self._result

        # 收敛判据（§4.4）：四路像素偏差 + 角度均低于阈值
        max_err_px = max(
            abs(obs.e_bottom_px), abs(obs.e_right_px),
        )
        if max_err_px < p.servo_stop_threshold_px and abs(obs.theta_rad) < p.servo_stop_theta_rad:
            new_count = p.prev_converge_count + 1
        else:
            new_count = 0

        if new_count >= p.servo_converge_frames:
            verdict = VERDICT_CONVERGED
            reason = f"连续 {new_count} 帧满足收敛阈值"
        else:
            verdict = VERDICT_CONTINUE
            reason = f"未收敛（连续 {new_count}/{p.servo_converge_frames} 帧）"

        self._done = True
        self._result = self._verdict(verdict, new_count, reason)
        logger.info(
            "chassis_servo_judge: iter=%d verdict=%s converge_count=%d "
            "max_err_px=%.2f theta=%.4f",
            p.iter_index, verdict, new_count, max_err_px, obs.theta_rad,
        )
        return self._result

    def _verdict(self, verdict: str, converge_count: int, reason: str) -> Result:
        return Result.ok(
            f"{verdict}: {reason}",
            data={"verdict": verdict, "converge_count": int(converge_count), "reason": reason},
        )

    def on_is_finished(self) -> bool:
        return self._done
