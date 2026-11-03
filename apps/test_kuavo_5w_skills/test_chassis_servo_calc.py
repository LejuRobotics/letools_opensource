#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""底盘伺服计算：技能层单测（无硬件、无 ROS）。

飞书 §4.3 控制律 + §4.4 安全保护的纯计算验证：
    dx_m = clamp(gain * e_bottom_px * mm_per_px / 1000, ±max_step_m)
    dtheta_rad = clamp(servo_gain_theta * dtheta_rad, ±servo_max_dtheta_rad)
    累积量超 servo_cumulative_limit_m → stop_reason="cum_limit"
    iter_index >= servo_max_iters → stop_reason="iter_limit"

合成约定：mm_per_px = 10（10mm/px），gain=0.6，max_step_m=0.015：
    e_x_px = 1.0 px → dx = 0.6 * 1.0 * 10 / 1000 = 0.006 m（未触 max_step）
    e_x_px = 5.0 px → dx = 0.6 * 5.0 * 10 / 1000 = 0.030 m → clamp 到 0.015

运行：
    python3 apps/test_kuavo_5w_skills/test_chassis_servo_calc.py
退出码 0 通过、1 失败。
"""
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from core.domain.servo_types import AlignObservation, ServoCommand  # noqa: E402
from core.domain.result import Result                                # noqa: E402
from skills.atomic.refactored_sdk.chassis_servo_calc import (        # noqa: E402
    NOMINAL_DT_S,
    ChassisServoCalcParams,
    ChassisServoCalcSkill,
    _clamp,
)

# 合成场景常数
SCALE = 10.0  # 10 mm/px
GAIN = 0.6
MAX_STEP = 0.015
GAIN_THETA = 0.6
MAX_DTHETA = 0.05
CUM_LIMIT = 0.10
MAX_ITERS = 8
# 速度模式（§4.2）合成常数
GAIN_VEL = 0.6
GAIN_VEL_THETA = 0.6
V_MAX = 0.05
W_MAX = 0.15
V_MIN = 0.02


def _obs(e_bottom=0.0, e_right=0.0, theta_rad=0.0, valid=True, reason=""):
    """构造一个 AlignObservation（目标=当前=0，偏差由参数给定）。"""
    return AlignObservation(
        e_bottom_px=e_bottom, e_right_px=e_right, theta_rad=theta_rad,
        valid=valid, reject=reason,
    )


def _params(**over):
    base = dict(
        observation=_obs(),
        iter_index=0,
        cumulative_xy_m=0.0,
        gain=GAIN, mm_per_px=SCALE, max_step_m=MAX_STEP,
        servo_gain_theta=GAIN_THETA, servo_max_dtheta_rad=MAX_DTHETA,
        servo_cumulative_limit_m=CUM_LIMIT, servo_max_iters=MAX_ITERS,
    )
    base.update(over)
    return ChassisServoCalcParams(**base)


def _run(params):
    """initialize → execute，返回 (result, command)。"""
    skill = ChassisServoCalcSkill()
    init = skill.initialize(params)
    assert init.success, f"init 失败: {init.message}"
    res = skill.execute()
    cmd = res.data["command"] if (res.success and res.data) else None
    return res, cmd


def test_clamp():
    assert _clamp(3.0, 5.0) == 3.0
    assert _clamp(-3.0, 5.0) == -3.0
    assert _clamp(7.0, 5.0) == 5.0
    assert _clamp(-7.0, 5.0) == -5.0
    assert _clamp(1.0, 0.0) == 0.0
    print("    _clamp 正确夹到 [-limit, +limit]")


def test_control_law_small_error():
    """小误差未触 max_step：dx = gain * e_bottom * mm_per_px / 1000。"""
    _, cmd = _run(_params(observation=_obs(e_bottom=1.0, e_right=2.0, theta_rad=0.01)))
    assert isinstance(cmd, ServoCommand)
    # dx = 0.6 * 1.0 * 10 / 1000 = 0.006
    assert abs(cmd.dx_m - 0.006) < 1e-9, f"dx={cmd.dx_m}"
    # dy = 0.6 * 2.0 * 10 / 1000 = 0.012
    assert abs(cmd.dy_m - 0.012) < 1e-9, f"dy={cmd.dy_m}"
    # dtheta = 0.6 * 0.01 = 0.006
    assert abs(cmd.dtheta_rad - 0.006) < 1e-9, f"dtheta={cmd.dtheta_rad}"
    assert cmd.stop_reason == ""
    print("    小误差控制律精确（未触上限）")


def test_control_law_clamps_to_max_step():
    """大误差触 max_step：dx 被夹到 ±0.015。"""
    _, cmd = _run(_params(observation=_obs(e_bottom=5.0)))  # 0.030 → 夹到 0.015
    assert abs(cmd.dx_m - 0.015) < 1e-9, f"dx={cmd.dx_m}"
    _, cmd = _run(_params(observation=_obs(e_bottom=-5.0)))
    assert abs(cmd.dx_m - (-0.015)) < 1e-9, f"dx={cmd.dx_m}"
    print("    大误差被 clamp 到 ±max_step_m")


def test_dtheta_clamped():
    """角度偏差大时被夹到 ±servo_max_dtheta_rad。"""
    _, cmd = _run(_params(observation=_obs(theta_rad=1.0)))  # 0.6 → 夹到 0.05
    assert abs(cmd.dtheta_rad - 0.05) < 1e-9, f"dtheta={cmd.dtheta_rad}"
    _, cmd = _run(_params(observation=_obs(theta_rad=-1.0)))
    assert abs(cmd.dtheta_rad - (-0.05)) < 1e-9, f"dtheta={cmd.dtheta_rad}"
    print("    旋转增量被 clamp 到 ±servo_max_dtheta_rad")


def test_cumulative_accumulates():
    """累积量 = 上轮 + 本步位移模。"""
    _, cmd = _run(_params(
        observation=_obs(e_bottom=1.0, e_right=2.0), cumulative_xy_m=0.03))
    # 本步 = hypot(0.006, 0.012) = 0.013416...
    expected = 0.03 + math.hypot(0.006, 0.012)
    assert abs(cmd.cumulative_xy_m - expected) < 1e-9, f"cum={cmd.cumulative_xy_m}"
    assert cmd.stop_reason == ""
    print("    累积量正确累加（未触上限）")


def test_cumulative_limit_triggers_stop():
    """累积量超限 → stop_reason='cum_limit'。"""
    _, cmd = _run(_params(
        observation=_obs(e_bottom=1.0, e_right=2.0), cumulative_xy_m=0.09))
    # 本步 ≈ 0.0134，累计 ≈ 0.1034 > 0.10
    assert cmd.cumulative_xy_m > CUM_LIMIT
    assert cmd.stop_reason == "cum_limit", f"stop={cmd.stop_reason}"
    print("    累积量超限触发 cum_limit")


def test_iter_limit_triggers_stop():
    """迭代超限 → stop_reason='iter_limit'（累积量未超时）。"""
    _, cmd = _run(_params(observation=_obs(e_bottom=0.1), iter_index=8))
    # dx 很小，cum 不会超限；iter_index=8 >= max_iters=8
    assert cmd.stop_reason == "iter_limit", f"stop={cmd.stop_reason}"
    print("    迭代超限触发 iter_limit")


def test_cumulative_takes_priority_over_iter():
    """累积量超限优先于迭代超限（§4.4 安全保护立即终止）。"""
    _, cmd = _run(_params(
        observation=_obs(e_bottom=5.0), iter_index=8, cumulative_xy_m=0.09))
    # dx 被夹到 0.015，cum ≈ 0.105 > 0.10；iter 也超
    assert cmd.stop_reason == "cum_limit", f"stop={cmd.stop_reason}"
    print("    累积量超限优先于迭代超限")


def test_invalid_observation_rejected():
    """观测无效 → Result.fail，不输出指令。"""
    skill = ChassisServoCalcSkill()
    init = skill.initialize(_params(observation=_obs(valid=False, reason="no_edge")))
    assert init.success
    res = skill.execute()
    assert not res.success
    assert res.data is None or "command" not in (res.data or {})
    print("    无效观测被拒")


def test_no_observation_rejected():
    """observation=None → Result.fail。"""
    skill = ChassisServoCalcSkill()
    skill.initialize(_params(observation=None))
    res = skill.execute()
    assert not res.success
    print("    空观测被拒")


def test_invalid_mode_rejected():
    """M2：非法 mode（拼错）→ initialize 失败，不静默落进位置分支。"""
    skill = ChassisServoCalcSkill()
    init = skill.initialize(_params(mode="velocty"))
    assert not init.success, "非法 mode 应在 initialize 被拒"
    assert "mode" in init.message
    print("    非法 mode 被拒")


def test_nan_observation_rejected():
    """L2：观测含 NaN（json 裸字面量来源）→ Result.fail，不穿透到 /cmd_vel。"""
    skill = ChassisServoCalcSkill()
    skill.initialize(_params(observation=_obs(e_bottom=float("nan"))))
    res = skill.execute()
    assert not res.success, "NaN 观测应被拒"
    assert res.error_code == "invalid_observation"
    print("    NaN 观测被拒")


def test_inf_observation_rejected():
    """L2：观测含 Inf → Result.fail。"""
    skill = ChassisServoCalcSkill()
    skill.initialize(_params(observation=_obs(theta_rad=float("inf"))))
    res = skill.execute()
    assert not res.success, "Inf 观测应被拒"
    assert res.error_code == "invalid_observation"
    print("    Inf 观测被拒")


def test_zero_error_yields_zero_command():
    """零偏差 → dx=dy=theta_rad=0，stop_reason=''。"""
    _, cmd = _run(_params(observation=_obs()))
    assert cmd.dx_m == 0.0 and cmd.dy_m == 0.0 and cmd.dtheta_rad == 0.0
    assert cmd.stop_reason == ""
    print("    零偏差输出零指令")


# ---------------------------------------------------------------------------
# 速度模式（§4.2，mode="velocity"）
# ---------------------------------------------------------------------------

def _vel_params(**over):
    """速度模式参数：mode=velocity + 速度环增益/限幅/死区。"""
    over.setdefault("mode", "velocity")
    over.setdefault("servo_gain_vel", GAIN_VEL)
    over.setdefault("servo_gain_vel_theta", GAIN_VEL_THETA)
    over.setdefault("v_max_mps", V_MAX)
    over.setdefault("w_max_radps", W_MAX)
    over.setdefault("v_min_mps", V_MIN)
    return _params(**over)


def test_velocity_control_law_small_error():
    """速度模式：wz 精确按公式计算，位置字段归零。"""
    _, cmd = _run(_vel_params(
        observation=_obs(e_bottom=1.0, e_right=2.0, theta_rad=0.02)))
    assert cmd.mode == "velocity"
    # vx = 0.6 * 1.0 * 10 / 1000 = 0.006 → 落死区 (< 0.02) → 抬到 0.02
    assert abs(abs(cmd.vx_mps) - V_MIN) < 1e-9, f"vx={cmd.vx_mps}"
    # wz = 0.6 * 0.02 = 0.012（不做死区补偿）
    assert abs(cmd.wz_radps - 0.012) < 1e-9, f"wz={cmd.wz_radps}"
    # 位置字段在速度模式下保持 0
    assert cmd.dx_m == 0.0 and cmd.dy_m == 0.0 and cmd.dtheta_rad == 0.0
    print("    速度模式：wz 精确、位置字段归零")


def test_velocity_control_law_above_deadzone():
    """速度模式：值大于 v_min 时不被死区补偿，保留公式原值。"""
    # vx = 0.6 * 5.0 * 10 / 1000 = 0.030（> v_min=0.02，未触上限）
    _, cmd = _run(_vel_params(observation=_obs(e_bottom=5.0, e_right=-5.0)))
    assert abs(cmd.vx_mps - 0.030) < 1e-9, f"vx={cmd.vx_mps}"
    assert abs(cmd.vy_mps - (-0.030)) < 1e-9, f"vy={cmd.vy_mps}"
    print("    速度模式：超死区值不被补偿")


def test_velocity_clamps():
    """速度模式限幅：大误差被 v_max_mps / w_max_radps 夹住。"""
    _, cmd = _run(_vel_params(
        observation=_obs(e_bottom=100.0, e_right=-100.0, theta_rad=10.0)))
    assert abs(cmd.vx_mps - V_MAX) < 1e-9, f"vx={cmd.vx_mps}"
    assert abs(cmd.vy_mps - (-V_MAX)) < 1e-9, f"vy={cmd.vy_mps}"
    assert abs(cmd.wz_radps - W_MAX) < 1e-9, f"wz={cmd.wz_radps}"
    print("    速度模式：vx/vy/wz 被上限夹住")


def test_velocity_deadzone_lifts_small_nonzero():
    """死区补偿：0 < |v| < v_min 被抬到 ±v_min（符号保持）。"""
    # vx = 0.6 * 1.0 * 10/1000 = 0.006 ∈ (0, 0.02) → +0.02
    _, cmd = _run(_vel_params(observation=_obs(e_bottom=1.0, e_right=-1.0)))
    assert abs(cmd.vx_mps - V_MIN) < 1e-9, f"vx={cmd.vx_mps}"
    assert abs(cmd.vy_mps - (-V_MIN)) < 1e-9, f"vy={cmd.vy_mps}"
    print("    死区补偿：小非零抬到下限（符号保持）")


def test_velocity_zero_not_lifted():
    """v == 0 不被死区补偿抬（零就是零，停车指令）。"""
    _, cmd = _run(_vel_params(observation=_obs()))
    assert cmd.vx_mps == 0.0, f"vx={cmd.vx_mps}（零不得被抬）"
    assert cmd.vy_mps == 0.0, f"vy={cmd.vy_mps}（零不得被抬）"
    assert cmd.wz_radps == 0.0, f"wz={cmd.wz_radps}"
    assert cmd.stop_reason == ""
    print("    零速度保持零（不被死区补偿）")


def test_velocity_cumulative_uses_actual_dt():
    """速度模式累积量 ≈ 上轮 + hypot(vx, vy) * 节点注入的实际 dt_s。"""
    # 用超死区值避免补偿干扰：vx = vy = 0.030；dt=0.14 ≈ 7 个 50Hz tick
    _, cmd = _run(_vel_params(
        observation=_obs(e_bottom=5.0, e_right=5.0), cumulative_xy_m=0.01,
        dt_s=0.14))
    expected = 0.01 + math.hypot(0.030, 0.030) * 0.14
    assert abs(cmd.cumulative_xy_m - expected) < 1e-9, f"cum={cmd.cumulative_xy_m}"
    assert cmd.stop_reason == ""
    print("    速度模式累积量按实际 dt 累计")


def test_velocity_cumulative_falls_back_to_nominal_dt():
    """不传 dt_s（直接调技能/单测）→ 回退 NOMINAL_DT_S，保持旧行为。"""
    _, cmd = _run(_vel_params(
        observation=_obs(e_bottom=5.0, e_right=5.0), cumulative_xy_m=0.01))
    expected = 0.01 + math.hypot(0.030, 0.030) * NOMINAL_DT_S
    assert abs(cmd.cumulative_xy_m - expected) < 1e-9, f"cum={cmd.cumulative_xy_m}"
    print("    缺省 dt_s 回退名义周期")


def test_velocity_cumulative_limit():
    """速度模式累积量超限 → stop_reason='cum_limit'（与位置模式同语义）。"""
    # 每 tick ≈ hypot(0.030,0.030)*NOMINAL_DT_S ≈ 0.006364，给一个已接近上限的值
    _, cmd = _run(_vel_params(
        observation=_obs(e_bottom=5.0, e_right=5.0), cumulative_xy_m=0.0999))
    assert cmd.cumulative_xy_m > CUM_LIMIT
    assert cmd.stop_reason == "cum_limit", f"stop={cmd.stop_reason}"
    print("    速度模式累积量超限触发 cum_limit")


def test_default_mode_is_position_regression():
    """mode 默认时行为与改动前完全一致（关键回归断言）。"""
    # 不传 mode，走默认 position
    _, cmd = _run(_params(observation=_obs(e_bottom=1.0, e_right=2.0, theta_rad=0.01)))
    assert cmd.mode == "position"
    assert abs(cmd.dx_m - 0.006) < 1e-9, f"dx={cmd.dx_m}"
    assert abs(cmd.dy_m - 0.012) < 1e-9, f"dy={cmd.dy_m}"
    assert abs(cmd.dtheta_rad - 0.006) < 1e-9, f"dtheta={cmd.dtheta_rad}"
    # 速度字段在位置模式下保持 0
    assert cmd.vx_mps == 0.0 and cmd.vy_mps == 0.0 and cmd.wz_radps == 0.0
    print("    默认 position 模式行为回归一致")


# ---------------------------------------------------------------------------
# 阶段门（先转后平，phase=rotate|translate）
# ---------------------------------------------------------------------------

def test_invalid_phase_rejected():
    """非法 phase（拼错）→ initialize 失败，不静默落进 both 分支。"""
    skill = ChassisServoCalcSkill()
    init = skill.initialize(_vel_params(phase="rotat"))
    assert not init.success, "非法 phase 应在 initialize 被拒"
    assert "phase" in init.message, f"错误信息应提到 phase：{init.message}"
    print("    非法 phase 被拒")


def test_phase_rotate_zeroes_translation():
    """阶段 rotate：vx/vy 严格置零，wz 照常发 —— 阶段 1 只转不平。"""
    _, cmd = _run(_vel_params(phase="rotate",
                              observation=_obs(e_bottom=30.0, e_right=-20.0,
                                               theta_rad=0.10)))
    assert cmd.vx_mps == 0.0, f"rotate 阶段 vx 应置零，得到 {cmd.vx_mps}"
    assert cmd.vy_mps == 0.0, f"rotate 阶段 vy 应置零，得到 {cmd.vy_mps}"
    assert abs(cmd.wz_radps - 0.06) < 1e-9, f"wz=0.6*0.10=0.06，得到 {cmd.wz_radps}"
    print("    rotate 阶段 vx/vy 置零、wz 保留")


def test_phase_translate_zeroes_rotation():
    """阶段 translate：wz 严格置零，vx/vy 照常发 —— 阶段 2 只平不转。"""
    _, cmd = _run(_vel_params(phase="translate",
                              observation=_obs(e_bottom=30.0, e_right=-20.0,
                                               theta_rad=0.10)))
    assert cmd.wz_radps == 0.0, f"translate 阶段 wz 应置零，得到 {cmd.wz_radps}"
    assert abs(cmd.vx_mps - V_MAX) < 1e-9, f"vx 应夹到 {V_MAX}，得到 {cmd.vx_mps}"
    assert abs(cmd.vy_mps + V_MAX) < 1e-9, f"vy 应夹到 -{V_MAX}，得到 {cmd.vy_mps}"
    print("    translate 阶段 wz 置零、vx/vy 保留")


def test_phase_gate_not_lifted_by_deadzone():
    """阶段置零的通道**不许**被死区补偿抬起来（0 是停车指令，必须保持 0）。

    这是阶段门最容易失效的地方：若先做死区补偿再置零、或者拿 ±v_min 去「表示 0」，
    阶段 1 就会一边转一边以 v_min 平移，阶段门形同虚设。
    """
    _, cmd = _run(_vel_params(phase="rotate",
                              observation=_obs(e_bottom=0.001, e_right=0.001,
                                               theta_rad=0.10)))
    assert cmd.vx_mps == 0.0 and cmd.vy_mps == 0.0, \
        f"极小平移误差在 rotate 阶段应仍为 0，得到 vx={cmd.vx_mps} vy={cmd.vy_mps}"
    assert abs(cmd.wz_radps - 0.06) < 1e-9, f"wz 不受影响，得到 {cmd.wz_radps}"
    print("    rotate 阶段置零的 vx/vy 未被死区补偿抬起")


def test_w_min_lifts_small_nonzero_wz():
    """w_min_radps：小非零 wz 抬到下限（角速度死区补偿）。"""
    _, cmd = _run(_vel_params(phase="rotate", w_min_radps=0.03,
                              observation=_obs(theta_rad=0.01)))
    assert abs(cmd.wz_radps - 0.03) < 1e-9, \
        f"0.6*0.01=0.006 < 0.03 应抬到 0.03，得到 {cmd.wz_radps}"
    print("    小非零 wz 被抬到 w_min_radps")


def test_w_min_does_not_lift_zero_wz():
    """w_min_radps：wz 恰为 0（停车指令）不被抬起来。"""
    _, cmd = _run(_vel_params(phase="rotate", w_min_radps=0.03,
                              observation=_obs(theta_rad=0.0)))
    assert cmd.wz_radps == 0.0, f"零速是停车指令，不该被抬，得到 {cmd.wz_radps}"
    print("    wz=0 不被 w_min 抬起")


def test_axis_deadband_zeroes_converged_axis():
    """逐轴死区：已到位那轴置零，另一轴照常输出（分轴收敛，互不拖累）。"""
    # e_bottom=5px 已在 30px 死区内 → vx=0；e_right=100px → vy 满速
    _, cmd = _run(_vel_params(axis_deadband_px=30.0,
                              observation=_obs(e_bottom=5.0, e_right=100.0)))
    assert cmd.vx_mps == 0.0, f"x 轴已到位应置零，得到 {cmd.vx_mps}"
    assert abs(cmd.vy_mps - V_MAX) < 1e-9, f"y 轴应照常输出，得到 {cmd.vy_mps}"
    print("    逐轴死区：到位轴置零、未到位轴照常走")


def test_axis_deadband_zero_not_lifted_by_deadzone():
    """逐轴死区置的 0 不许被 v_min 抬起来（否则已到位轴永远停不下来）。"""
    # vx 原始值 = 0.6*5*10/1000 = 0.03 > 0，若在补偿之后置零不成立；这里验证最终为 0
    _, cmd = _run(_vel_params(axis_deadband_px=30.0, v_min_mps=0.06,
                              observation=_obs(e_bottom=5.0, e_right=0.0)))
    assert cmd.vx_mps == 0.0, f"死区内的轴被死区补偿抬起，得到 {cmd.vx_mps}"
    assert cmd.vy_mps == 0.0, f"e_right=0 也应为 0，得到 {cmd.vy_mps}"
    print("    逐轴死区置零未被 v_min 抬起")


def test_axis_deadband_boundary_is_strict():
    """边界：|e| 恰好等于阈值不置零（与 judge 的严格小于判据一致）。"""
    _, cmd = _run(_vel_params(axis_deadband_px=30.0,
                              observation=_obs(e_bottom=30.0, e_right=0.0)))
    assert abs(cmd.vx_mps - V_MAX) < 1e-9, f"等于阈值应继续走，得到 {cmd.vx_mps}"
    _, cmd2 = _run(_vel_params(axis_deadband_px=30.0,
                               observation=_obs(e_bottom=29.9, e_right=0.0)))
    assert cmd2.vx_mps == 0.0, f"小于阈值应置零，得到 {cmd2.vx_mps}"
    print("    逐轴死区边界为严格小于")


def test_axis_deadband_disabled_by_default():
    """axis_deadband_px 默认 0：行为与改动前完全一致（关键回归断言）。"""
    _, cmd = _run(_vel_params(observation=_obs(e_bottom=1.0, e_right=1.0)))
    assert abs(cmd.vx_mps - V_MIN) < 1e-9, f"默认应被死区补偿抬起，得到 {cmd.vx_mps}"
    assert abs(cmd.vy_mps - V_MIN) < 1e-9, f"默认应被死区补偿抬起，得到 {cmd.vy_mps}"
    print("    逐轴死区默认关闭（回归一致）")


def test_default_phase_is_both_regression():
    """phase 默认 both 时行为与改动前完全一致（关键回归断言）。

    未接阶段机的场景（v1 / real_v1 / dual_v1）读不到 servo_phase，走默认 both，
    两通道必须同时生效 —— 否则那些场景会被静默改成单通道。
    """
    _, cmd = _run(_vel_params(observation=_obs(e_bottom=5.0, e_right=0.0,
                                               theta_rad=0.10)))
    assert abs(cmd.vx_mps - 0.03) < 1e-9, f"vx 应正常发出，得到 {cmd.vx_mps}"
    assert abs(cmd.wz_radps - 0.06) < 1e-9, f"wz 应正常发出，得到 {cmd.wz_radps}"
    print("    默认 both 阶段两通道同时生效（回归一致）")


def test_is_finished_after_execute():
    """execute 后 is_finished() 为 True。"""
    skill = ChassisServoCalcSkill()
    assert not skill.is_finished()
    skill.initialize(_params())
    assert not skill.is_finished()
    skill.execute()
    assert skill.is_finished()
    print("    is_finished 在 execute 后置真")


def test_idempotent_execute():
    """重复 execute 返回同一 Result（_done 短路）。"""
    skill = ChassisServoCalcSkill()
    skill.initialize(_params(observation=_obs(e_bottom=1.0)))
    r1 = skill.execute()
    r2 = skill.execute()
    assert r1 is r2
    print("    重复 execute 幂等")


def main() -> int:
    cases = [
        ("_clamp 夹限", test_clamp),
        ("控制律小误差", test_control_law_small_error),
        ("控制律大误差夹限", test_control_law_clamps_to_max_step),
        ("旋转增量夹限", test_dtheta_clamped),
        ("累积量累加", test_cumulative_accumulates),
        ("累积量超限", test_cumulative_limit_triggers_stop),
        ("迭代超限", test_iter_limit_triggers_stop),
        ("累积量优先于迭代", test_cumulative_takes_priority_over_iter),
        ("无效观测被拒", test_invalid_observation_rejected),
        ("空观测被拒", test_no_observation_rejected),
        ("非法 mode 被拒", test_invalid_mode_rejected),
        ("NaN 观测被拒", test_nan_observation_rejected),
        ("Inf 观测被拒", test_inf_observation_rejected),
        ("零偏差零指令", test_zero_error_yields_zero_command),
        ("is_finished 置真", test_is_finished_after_execute),
        ("execute 幂等", test_idempotent_execute),
        ("速度模式 wz 精确", test_velocity_control_law_small_error),
        ("速度模式超死区值", test_velocity_control_law_above_deadzone),
        ("速度模式限幅", test_velocity_clamps),
        ("死区补偿抬小非零", test_velocity_deadzone_lifts_small_nonzero),
        ("零速度不补偿", test_velocity_zero_not_lifted),
        ("速度模式累积量", test_velocity_cumulative_uses_actual_dt),
        ("速度模式累积量缺省回退", test_velocity_cumulative_falls_back_to_nominal_dt),
        ("速度模式累积量超限", test_velocity_cumulative_limit),
        ("默认 position 回归", test_default_mode_is_position_regression),
        ("非法 phase 被拒", test_invalid_phase_rejected),
        ("rotate 阶段 vx/vy 置零", test_phase_rotate_zeroes_translation),
        ("translate 阶段 wz 置零", test_phase_translate_zeroes_rotation),
        ("阶段置零不被死区抬起", test_phase_gate_not_lifted_by_deadzone),
        ("w_min 抬小非零 wz", test_w_min_lifts_small_nonzero_wz),
        ("wz=0 不被 w_min 抬起", test_w_min_does_not_lift_zero_wz),
        ("逐轴死区置零已到位轴", test_axis_deadband_zeroes_converged_axis),
        ("逐轴死区零不被抬起", test_axis_deadband_zero_not_lifted_by_deadzone),
        ("逐轴死区边界严格小于", test_axis_deadband_boundary_is_strict),
        ("逐轴死区默认关闭", test_axis_deadband_disabled_by_default),
        ("默认 both 阶段回归", test_default_phase_is_both_regression),
    ]
    failed = 0
    for title, fn in cases:
        try:
            fn()
        except AssertionError as exc:
            failed += 1
            print(f"  FAIL  {title}: {exc}")
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
