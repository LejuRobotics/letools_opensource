#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""底盘伺服判断：技能层单测（无硬件、无 ROS）。

飞书 §4.4 收敛判据 + 超限失败的纯计算验证：
    收敛 = 连续 servo_converge_frames(2)帧满足
        max(|e_long|,|e_short|,|e_x|,|e_y|) < servo_stop_threshold_px 且 |dtheta| < servo_stop_theta_rad
    超限失败 = iter_index >= servo_max_iters(8) 或 cumulative_xy_m > servo_cumulative_limit_m(0.10)

运行：
    python3 apps/test_kuavo_5w_skills/test_chassis_servo_judge.py
退出码 0 通过、1 失败。
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from core.domain.servo_types import AlignObservation                  # noqa: E402
from skills.atomic.refactored_sdk.chassis_servo_judge import (        # noqa: E402
    ChassisServoJudgeParams,
    ChassisServoJudgeSkill,
    VERDICT_CONVERGED,
    VERDICT_CONTINUE,
    VERDICT_FAILED_ITER,
    VERDICT_FAILED_CUM,
)

# 判据参数
THRESH_PX = 10.0
THETA_RAD = 0.02
CONVERGE_FRAMES = 2
MAX_ITERS = 8
CUM_LIMIT = 0.10


def _obs(e_bottom=0.0, e_right=0.0, theta_rad=0.0,  valid=True, reason=""):
    return AlignObservation(
        e_bottom_px=e_bottom, e_right_px=e_right, theta_rad=theta_rad,
        valid=valid, reject=reason,
    )


def _params(**over):
    base = dict(
        observation=_obs(),
        iter_index=0,
        cumulative_xy_m=0.0,
        prev_converge_count=0,
        servo_stop_threshold_px=THRESH_PX,
        servo_stop_theta_rad=THETA_RAD,
        servo_converge_frames=CONVERGE_FRAMES,
        servo_max_iters=MAX_ITERS,
        servo_cumulative_limit_m=CUM_LIMIT,
    )
    base.update(over)
    return ChassisServoJudgeParams(**base)


def _run(params):
    skill = ChassisServoJudgeSkill()
    skill.initialize(params)
    res = skill.execute()
    return res, (res.data if res.success else None)


def test_continue_when_error_above_threshold():
    """偏差超阈值 → continue，converge_count=0。"""
    res, data = _run(_params(observation=_obs(e_bottom=15.0)))
    assert data["verdict"] == VERDICT_CONTINUE
    assert data["converge_count"] == 0
    print("    偏差超阈值 → continue")


def test_theta_above_threshold_continues():
    """像素 OK 但角度超阈值 → continue，count=0。"""
    res, data = _run(_params(observation=_obs(theta_rad=0.05)))  # > 0.02
    assert data["verdict"] == VERDICT_CONTINUE
    assert data["converge_count"] == 0
    print("    角度超阈值 → continue")


def test_one_converge_frame_not_enough():
    """单帧满足但 prev=0 → continue，count=1。"""
    res, data = _run(_params(observation=_obs(), prev_converge_count=0))
    assert data["verdict"] == VERDICT_CONTINUE
    assert data["converge_count"] == 1
    print("    单帧收敛不足 → continue (count=1)")


def test_converged_after_two_consecutive_frames():
    """prev=1 且本帧满足 → converged。"""
    res, data = _run(_params(observation=_obs(), prev_converge_count=1))
    assert data["verdict"] == VERDICT_CONVERGED
    assert data["converge_count"] == 2
    print("    连续 2 帧满足 → converged")


def test_converge_count_resets_on_break():
    """上一帧收敛但本帧不满足 → count 清零。"""
    res, data = _run(_params(observation=_obs(e_bottom=15.0), prev_converge_count=1))
    assert data["verdict"] == VERDICT_CONTINUE
    assert data["converge_count"] == 0
    print("    连续帧被打断 → count 清零")


def test_max_err_uses_both_pixel_errors():
    """e_bottom 或 e_right 超阈值判 continue（两路偏差都要过）。"""
    res, data = _run(_params(observation=_obs(e_bottom=15.0)))  # 其余为 0
    assert data["verdict"] == VERDICT_CONTINUE
    print("    两路像素偏差任一超阈值 → continue")
    res, data = _run(_params(observation=_obs(e_right=15.0)))
    assert data["verdict"] == VERDICT_CONTINUE
    print("    e_right 超阈值同样判 continue")


def test_iter_limit_when_not_converged():
    """iter >= max_iters 且未收敛 → failed_iter。"""
    res, data = _run(_params(observation=_obs(e_bottom=15.0), iter_index=8))
    assert data["verdict"] == VERDICT_FAILED_ITER
    print("    迭代超限未收敛 → failed_iter")


def test_cum_limit_takes_priority():
    """累积量超限优先于迭代超限（§4.4 安全保护立即终止）。"""
    res, data = _run(_params(
        observation=_obs(e_bottom=15.0), iter_index=8, cumulative_xy_m=0.11))
    assert data["verdict"] == VERDICT_FAILED_CUM
    print("    累积量超限优先于迭代超限")


def test_cum_limit_even_if_converging():
    """即便本帧满足收敛，累积量超限仍判 failed_cum。"""
    res, data = _run(_params(
        observation=_obs(), prev_converge_count=1, cumulative_xy_m=0.11))
    assert data["verdict"] == VERDICT_FAILED_CUM
    print("    累积量超限压过收敛")


def test_invalid_observation_rejected():
    """观测无效 → Result.fail。"""
    skill = ChassisServoJudgeSkill()
    skill.initialize(_params(observation=_obs(valid=False, reason="no_edge")))
    res = skill.execute()
    assert not res.success
    print("    无效观测被拒")


def test_no_observation_rejected():
    """observation=None → Result.fail。"""
    skill = ChassisServoJudgeSkill()
    skill.initialize(_params(observation=None))
    res = skill.execute()
    assert not res.success
    print("    空观测被拒")


def test_is_finished_and_idempotent():
    """execute 后 is_finished，重复 execute 幂等。"""
    skill = ChassisServoJudgeSkill()
    assert not skill.is_finished()
    skill.initialize(_params())
    r1 = skill.execute()
    assert skill.is_finished()
    r2 = skill.execute()
    assert r1 is r2
    print("    is_finished 置真 + execute 幂等")


def main() -> int:
    cases = [
        ("偏差超阈值 continue", test_continue_when_error_above_threshold),
        ("角度超阈值 continue", test_theta_above_threshold_continues),
        ("单帧不足 continue", test_one_converge_frame_not_enough),
        ("连续两帧 converged", test_converged_after_two_consecutive_frames),
        ("打断清零", test_converge_count_resets_on_break),
        ("两路像素偏差", test_max_err_uses_both_pixel_errors),
        ("迭代超限 failed_iter", test_iter_limit_when_not_converged),
        ("累积量优先", test_cum_limit_takes_priority),
        ("累积量压过收敛", test_cum_limit_even_if_converging),
        ("无效观测被拒", test_invalid_observation_rejected),
        ("空观测被拒", test_no_observation_rejected),
        ("幂等与完成", test_is_finished_and_idempotent),
    ]
    failed = 0
    for title, fn in cases:
        try:
            fn()
        except AssertionError as exc:
            failed += 1
            current_case = title  # 追踪失败的用例
            print(f"  FAIL  {current_case}: {exc}")
        except Exception as exc:                       # noqa: BLE001
            failed += 1
            print(f"  ERROR {title}: {type(exc).__name__}: {exc}")

    print()
    if failed:
        print(f"FAILED: {failed}/{len(cases)}")
        return 1
    print(f"OK: {len(cases)}/{len(cases)} 全部通过")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
