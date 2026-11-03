# -*- coding: utf-8 -*-
"""Atomic skill: chassis_servo_calc —— 底盘伺服计算（控制器，纯计算无硬件依赖）。

飞书文档 §3.3 / §4.3 / §5.2 的「控制器」模块：拿到 AlignObservation 之后做纯计算，
输出 ServoCommand（底盘 xy 位移增量 + 旋转增量），不碰底盘、不做视觉。

观测只带三被控量（e_bottom_px / e_right_px / theta_rad，与 dev 视觉链路的
ServoError 对齐）；像素→米的换算系数 mm_per_px 是 calc 自己的参数（真机标定），
不从观测里读。

控制律（§4.3）：
    位置模式（mode="position"，默认）：
        dx_m       = clamp(gain * e_bottom_px * mm_per_px / 1000, ±max_step_m)
        dy_m       = clamp(gain * e_right_px  * mm_per_px / 1000, ±max_step_m)
        dtheta_rad = clamp(servo_gain_theta * theta_rad, ±servo_max_dtheta_rad)
        gain=0.6 ⇒ 每步残差 ×0.4，几何收敛。
    速度模式（mode="velocity"，双模方案 §4.2）：
        vx = clamp(servo_gain_vel * e_bottom_px * mm_per_px / 1000, ±v_max_mps)
        vy = clamp(servo_gain_vel * e_right_px  * mm_per_px / 1000, ±v_max_mps)
        wz = clamp(servo_gain_vel_theta * theta_rad, ±w_max_radps)
        纯 P 速度环，无积分项（伺服本身每帧闭环迭代，不需积分消残差）。
        逐轴死区（axis_deadband_px>0 时生效）：某一轴偏差绝对值已小于该阈值时，
        该轴速度**严格置零**（在死区补偿之前），使两轴收敛互不拖累 ——
        否则已到位的轴会被死区补偿抬到 v_min，只能靠来回穿越零点停不下来。
        死区补偿：非零但小于 v_min_mps 时抬到 v_min_mps（底盘静摩擦，否则可能不启动）；
        v == 0 是停车指令，必须保持零、不抬（零就是零）。wz 不做补偿。

安全保护（§4.4）：
    - 累积量：new_cum = cumulative_xy_m + 本步位移模；超 servo_cumulative_limit_m → stop_reason="cum_limit"
      位置模式本步位移 = hypot(dx, dy)；速度模式本步位移 ≈ hypot(vx, vy) * dt_s
      （dt_s 由节点按两帧新观测的实际间隔注入；缺省回退 NOMINAL_DT_S）。
    - 迭代超限：iter_index >= servo_max_iters → stop_reason="iter_limit"

本技能不接 hardware 参数（遵循 PalletServoSkill 先例 + 飞书附录三「无 get_shared_hardware() 依赖」），
可直接构造单测，无需 ROS、无需硬件。

运行与测试
----------
    python3 apps/test_kuavo_5w_skills/test_chassis_servo_calc.py
"""
import math
from dataclasses import dataclass
from typing import Optional

from core.common.logger import get_logger
from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.domain.servo_types import (
    AlignObservation, ServoCommand, observation_field)
from orchestration.utils.manifest_decorators import define_manifest
from skills.base.skill_base import SkillBase

logger = get_logger(__name__)


def _clamp(value: float, limit: float) -> float:
    """把 value 夹到 [-limit, +limit]。limit 非正时返回 0。"""
    if limit <= 0:
        return 0.0
    if value > limit:
        return float(limit)
    if value < -limit:
        return float(-limit)
    return float(value)


# 名义控制周期（s）：速度模式累积量的**缺省回退值**（直接调技能、无节点注入 dt_s 时用）。
# 真实运行时节点会按「上一帧新观测 → 本帧」的实际耗时注入 dt_s：版本门禁只在新观测
# 时重算一次，而执行节点在中间每个 tick 都按 50Hz 重发同一速度，用 0.02s 会把真实
# 位移低估数倍（实测约 7 倍），累积量保护因此形同虚设。
NOMINAL_DT_S = 0.15


def _apply_velocity_deadzone(v: float, v_min: float) -> float:
    """速度死区补偿（§4.2）：非零但小于 v_min_mps 时抬到 ±v_min_mps。

    底盘静摩擦导致过小速度可能不启动，故给小非零速度补到下限以驱动底盘。
    **v == 0 是停车指令，必须保持零、不被抬**（零就是零）；v_min 非正时也不补偿。
    """
    if v == 0.0 or v_min <= 0.0:
        return v
    if abs(v) < v_min:
        return float(v_min) if v > 0 else -float(v_min)
    return v


@dataclass
class ChassisServoCalcParams(SkillParams):
    """一次伺服计算的输入（飞书 §4.3 控制律 + §4.4 安全保护）。

    mm_per_px 是像素→米换算系数（真机标定，如 10.0 表示 1px=10mm），
    属于 calc 自己的参数，不在观测里。
    """

    skill_name: str = "chassis_servo_calc"
    observation: Optional[AlignObservation] = None
    iter_index: int = 0                 # 当前迭代序号（从 0 起）
    cumulative_xy_m: float = 0.0        # 截至本步前的 xy 位移累积量
    # 速度模式本步实际耗时(s)：节点按「上一帧新观测 → 本帧」的 monotonic 间隔注入；
    # 直接调技能/单测不传时回退 NOMINAL_DT_S。位置模式不使用该字段。
    dt_s: float = NOMINAL_DT_S
    # 模式选择（§4.2）：默认 "position"，现有场景零改动
    mode: str = "position"              # "position" | "velocity"
    # 阶段选择（先转后平）：默认 "both"，两通道同时生效，现有场景零改动
    phase: str = "both"                 # "rotate" | "translate" | "both"
    # 控制律（§4.3）
    gain: float = 0.6
    mm_per_px: float = 10.0
    max_step_m: float = 0.015
    servo_gain_theta: float = 0.6
    servo_max_dtheta_rad: float = 0.05
    # 速度模式控制律（§4.2，mode="velocity" 时生效）
    servo_gain_vel: float = 0.6         # 速度环 P 增益 (1/s)
    servo_gain_vel_theta: float = 0.6   # 角速度环 P 增益 (1/s)
    v_max_mps: float = 0.05             # 线速度上限 (m/s)
    w_max_radps: float = 0.15           # 角速度上限 (rad/s)
    v_min_mps: float = 0.02             # 线速度下限（死区补偿，需现场标定）
    w_min_radps: float = 0.0            # 角速度下限（死区补偿，0=不补偿，需现场标定）
    axis_deadband_px: float = 0.0       # 逐轴死区(px)：该轴偏差小于它就置零，0=关闭
    # 安全保护（§4.4）
    servo_cumulative_limit_m: float = 0.50
    servo_max_iters: int = 8
    timeout: float = 5.0


@define_manifest(
    label="底盘伺服计算",
    category=["skill", "chassis", "servo"],
    tree_type="studio_smoke",
    description="AlignObservation → ServoCommand：控制律 + 累积量安全保护 + 迭代超限（纯计算，不碰底盘）",
    params=[
        {"name": "mode", "type": "string", "default": "position",
         "description": "控制模式：position=位置增量(dx/dy/dtheta)，velocity=速度指令(vx/vy/wz)"},
        {"name": "phase", "type": "string", "default": "both",
         "description": "阶段：rotate=只发角度通道(vx/vy 置零)，translate=只发平移通道(wz 置零)，"
                        "both=两通道同时生效（默认，现有场景行为不变）"},
        {"name": "gain", "type": "float", "default": "0.6", "description": "xy 通道 P 增益"},
        {"name": "mm_per_px", "type": "float", "default": "10.0",
         "description": "像素→毫米换算系数（真机标定），dx = gain*e_px*mm_per_px/1000"},
        {"name": "max_step_m", "type": "float", "default": "0.015",
         "description": "单步 xy 位移上限(m)，须明显大于执行节点的 position_threshold，否则底盘一步都不走"},
        {"name": "servo_gain_theta", "type": "float", "default": "0.6", "description": "角度通道 P 增益"},
        {"name": "servo_max_dtheta_rad", "type": "float", "default": "0.05",
         "description": "单步旋转增量上限(rad)，须明显大于执行节点的 angle_threshold，否则转身不会执行"},
        {"name": "servo_gain_vel", "type": "float", "default": "0.6",
         "description": "速度模式线速度环 P 增益(1/s)"},
        {"name": "servo_gain_vel_theta", "type": "float", "default": "0.6",
         "description": "速度模式角速度环 P 增益(1/s)"},
        {"name": "v_max_mps", "type": "float", "default": "0.05",
         "description": "速度模式线速度上限(m/s)"},
        {"name": "w_max_radps", "type": "float", "default": "0.15",
         "description": "速度模式角速度上限(rad/s)"},
        {"name": "v_min_mps", "type": "float", "default": "0.02",
         "description": "速度模式线速度下限(m/s)：非零速度低于此值抬到下限（死区补偿，需现场标定）"},
        {"name": "w_min_radps", "type": "float", "default": "0.0",
         "description": "速度模式角速度下限(rad/s)：非零 wz 低于此值抬到下限（死区补偿，"
                        "0=不补偿；需 rostopic pub 实测底盘角速度死区后现场标定）"},
        {"name": "axis_deadband_px", "type": "float", "default": "0.0",
         "description": "逐轴死区(px)：某轴像素偏差小于此值时该轴速度置零（在死区补偿之前），"
                        "使两轴收敛互不拖累；0=关闭（默认，行为与改动前一致）。"
                        "建议与 judge 的 servo_stop_threshold_px 同值"},
        {"name": "servo_cumulative_limit_m", "type": "float", "default": "0.10", "description": "累积位移安全保护上限(m)"},
        {"name": "servo_max_iters", "type": "int", "default": "8", "description": "最大迭代次数"},
    ],
    inputs=[{"name": "latest_servo_error", "type": "object",
             "description": "AlignObservation：两路像素平移偏差 + 偏差角 + valid"}],
    outputs=[{"name": "latest_servo_command", "type": "object",
               "description": "ServoCommand：dx/dy/dtheta + 累积量 + stop_reason"}],
)
class ChassisServoCalcSkill(SkillBase):
    """底盘伺服计算（控制器）：AlignObservation → ServoCommand，纯计算无硬件依赖。"""

    def __init__(self):
        super().__init__(name="chassis_servo_calc")
        self.params: Optional[ChassisServoCalcParams] = None
        self._done = False
        self._result: Optional[Result] = None

    def on_initialize(self, params: ChassisServoCalcParams) -> Result:
        if not isinstance(params, ChassisServoCalcParams):
            return Result.fail("Invalid parameters for ChassisServoCalcSkill")
        # 静态 mode 白名单校验：拼错（如 "velocty"）会静默落进位置分支、速度通路永不
        # 启用且无告警，此处显式拒绝，避免配置错误被吞掉。
        if params.mode not in ("position", "velocity"):
            return Result.fail(
                f"非法 mode={params.mode!r}，只支持 position|velocity")
        # 非法 phase 同样显式拒绝：拼错会静默落进 both 分支（两通道同时发），
        # 而调用方以为自己在跑单通道阶段机。
        if params.phase not in ("rotate", "translate", "both"):
            return Result.fail(
                f"非法 phase={params.phase!r}，只支持 rotate|translate|both")
        self.params = params
        self._done = False
        self._result = None
        return Result.ok()

    def on_execute(self) -> Result:
        if self._done:
            return self._result or Result.ok("ChassisServoCalcSkill already finished")

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

        # 非有限值防御（§4.3）：`json.loads` 接受裸 `NaN`/`Infinity` 字面量，上游感知
        # 异常也可能产出 NaN；而 `_clamp` / `_apply_velocity_deadzone` 的比较对 NaN 全为
        # False，会原样穿透到 /cmd_vel。此处显式拒绝无效输入（不把数据静默改成 0）。
        for _name in ("e_bottom_px", "e_right_px", "theta_rad"):
            if not math.isfinite(float(observation_field(obs, _name, 0.0))):
                self._done = True
                self._result = Result.fail(
                    "观测含非有限值（NaN/Inf）",
                    error_code="invalid_observation",
                )
                return self._result

        # 速度模式（§4.2）：走独立分支，位置模式路径逐字保持不变
        if p.mode == "velocity":
            return self._on_execute_velocity(obs)

        # 控制律（§4.3）位置模式：像素偏差 × 标定系数 → 米，再 clamp
        dx_m = _clamp(p.gain * obs.e_bottom_px * p.mm_per_px / 1000.0, p.max_step_m)
        dy_m = _clamp(p.gain * obs.e_right_px * p.mm_per_px / 1000.0, p.max_step_m)
        dtheta_rad = _clamp(p.servo_gain_theta * obs.theta_rad, p.servo_max_dtheta_rad)

        # 累积量安全保护（§4.4）：本步位移模加入累计，超限立即终止
        new_cum = p.cumulative_xy_m + math.hypot(dx_m, dy_m)
        stop_reason = ""
        if new_cum > p.servo_cumulative_limit_m:
            stop_reason = "cum_limit"
        elif p.iter_index >= p.servo_max_iters:
            stop_reason = "iter_limit"

        command = ServoCommand(
            dx_m=dx_m,
            dy_m=dy_m,
            dtheta_rad=dtheta_rad,
            iter_index=int(p.iter_index),
            cumulative_xy_m=new_cum,
            stop_reason=stop_reason,
        )

        self._done = True
        logger.info(
            "chassis_servo_calc: mode=%s iter=%d phase=%s e=(%.1f,%.1f)px theta=%.4f "
            "dx=%.4f dy=%.4f dtheta=%.4f cum=%.4f stop=%r",
            p.mode, command.iter_index, p.phase, obs.e_bottom_px, obs.e_right_px, obs.theta_rad,
            dx_m, dy_m, dtheta_rad, new_cum, stop_reason or "正常步",
        )
        self._result = Result.ok(
            f"ServoCommand iter={command.iter_index} stop={stop_reason or '正常步'}",
            data={"command": command},
        )
        return self._result

    def _on_execute_velocity(self, obs) -> Result:
        """速度模式（§4.2）：误差 → 速度指令 vx/vy/wz + 死区补偿 + 累积量保护。

        与位置模式共用累积量/迭代保护语义：新版 ServoCommand 只填速度字段，
        位置字段保持 0（两模式互斥）。
        """
        p = self.params

        # P 速度环（§4.2）：像素偏差 × 标定系数 → m/s，再按上限 clamp
        vx = _clamp(p.servo_gain_vel * obs.e_bottom_px * p.mm_per_px / 1000.0, p.v_max_mps)
        vy = _clamp(p.servo_gain_vel * obs.e_right_px * p.mm_per_px / 1000.0, p.v_max_mps)
        wz = _clamp(p.servo_gain_vel_theta * obs.theta_rad, p.w_max_radps)

        # 阶段门（先转后平）：把不属于本阶段的通道**严格置零**（不是「算完再忽略」）。
        # 必须在下游死区补偿之前做，且置的是 0.0 本身 —— 0 是停车指令，死区补偿
        # 对 v==0 不抬（见 _apply_velocity_deadzone），否则会被 v_min/w_min 抬起来，
        # 阶段门就形同虚设。
        if p.phase == "rotate":
            vx = 0.0
            vy = 0.0
        elif p.phase == "translate":
            wz = 0.0

        # 逐轴死区（分轴收敛）：某轴偏差已小于阈值 → 该轴**严格置零**，让另一轴
        # 独自收敛，互不拖累。必须在下游死区补偿之前做，且置的是 0.0 本身 ——
        # 否则小误差会被 v_min 抬到下限，已到位的轴反而停不下来、来回穿越零点
        # （还会被 cum = hypot(vx,vy) 反复计费）。阈值 <=0 表示关闭。
        if p.axis_deadband_px > 0.0:
            if abs(obs.e_bottom_px) < p.axis_deadband_px:
                vx = 0.0
            if abs(obs.e_right_px) < p.axis_deadband_px:
                vy = 0.0

        # 死区补偿（§4.2）：小非零速度抬到下限；零速不补偿（停车指令）
        vx = _apply_velocity_deadzone(vx, p.v_min_mps)
        vy = _apply_velocity_deadzone(vy, p.v_min_mps)
        wz = _apply_velocity_deadzone(wz, p.w_min_radps)

        # 累积量保护（§4.4）：速度模式下本步位移 ≈ v · 实际帧间隔。
        # dt_s 由节点按两帧新观测的 monotonic 间隔注入（版本门禁期间执行节点
        # 每个 tick 都会重发同一速度，故必须用真实帧间隔而非名义 50Hz 周期，
        # 否则累积量会被低估数倍）；首帧 dt_s=0。缺省回退 NOMINAL_DT_S。
        dt_s = max(0.0, float(p.dt_s))
        new_cum = p.cumulative_xy_m + math.hypot(vx, vy) * dt_s
        stop_reason = ""
        if new_cum > p.servo_cumulative_limit_m:
            stop_reason = "cum_limit"
        elif p.iter_index >= p.servo_max_iters:
            stop_reason = "iter_limit"

        command = ServoCommand(
            mode="velocity",
            vx_mps=vx,
            vy_mps=vy,
            wz_radps=wz,
            iter_index=int(p.iter_index),
            cumulative_xy_m=new_cum,
            stop_reason=stop_reason,
        )

        self._done = True
        logger.info(
            "chassis_servo_calc: mode=velocity iter=%d phase=%s e=(%.1f,%.1f)px theta=%.4f "
            "vx=%.4f vy=%.4f wz=%.4f dt=%.3f cum=%.4f stop=%r",
            command.iter_index, p.phase, obs.e_bottom_px, obs.e_right_px, obs.theta_rad,
            vx, vy, wz, dt_s, new_cum, stop_reason or "正常步",
        )
        self._result = Result.ok(
            f"ServoCommand(mode=velocity) iter={command.iter_index} stop={stop_reason or '正常步'}",
            data={"command": command},
        )
        return self._result

    def on_is_finished(self) -> bool:
        return self._done
