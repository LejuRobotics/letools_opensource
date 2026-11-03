# -*- coding: utf-8 -*-
"""NodeChassisServoCalc：读黑板观测量 → 伺服计算 → 写回 ServoCommand（纯计算节点）。

飞书 §3.3 / §5.2 的「控制器」薄封装：上游视觉感知模块把 AlignObservation 写进
黑板（latest_servo_error + _version），本节点读出来调 ChassisServoCalcSkill 算出
ServoCommand，写回黑板（latest_servo_command + _version）供执行节点消费。

本节点不访问硬件（控制器是纯计算），故无 get_shared_hardware() 调用，
可脱离 ROS / 硬件单测（见 apps/test_kuavo_5w_orchestration/test_chassis_servo_calc_node.py）。

黑板契约
--------
读 latest_servo_error          AlignObservation 或 dev 的 ServoError（视觉感知模块写入，鸭子类型判别）
读 latest_servo_error_version int，判断观测是否更新过
读 servo_exec_idle             bool，False = 底盘有动作在途，此时不递推下一步
读 servo_mode                  运行模式（模式选择节点写入）：position | velocity；
                              每个 tick 都读，键不存在 / None / 非法值回退到静态 mode 参数
读 servo_phase                 运行阶段（阶段选择节点写入）：rotate | translate | both；
                              同 mode 口径，回退到静态 phase 参数（默认 both = 不分阶段）
读 latest_servo_command      上一轮 ServoCommand（取 iter_index / cumulative_xy_m；
                              从未写过则 iter=0 / cum=0）
写 latest_servo_command      本轮 ServoCommand
写 latest_servo_command_version int，每算一次自增

节点持续返回 RUNNING（伺服循环由上层 Parallel / Sequence 收口），
观测版本未更新时不重复计算。

静态 mode 参数是**初值**，也是黑板不可用（键不存在 / None / 非法值）时的**回退值**；
运行模式由黑板 mode_key（默认 servo_mode）实时决定，见黑板契约。
"""

import os
import time

import py_trees
from py_trees.common import Status

from core.common.logger import get_logger
from core.domain.servo_types import (
    ServoCommand, is_observation_like, observation_field)
from orchestration.nodes.base_node import BaseAction
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.chassis_servo_calc import (
    ChassisServoCalcParams, ChassisServoCalcSkill)

logger = get_logger(__name__)

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")

# 两种合法运行模式；黑板上的其他值一律视为非法并回退静态 mode 参数
_MODE_POSITION = "position"
_MODE_VELOCITY = "velocity"
# 哨兵：区分「引用不存在/未注册」与「引用了但值是 None」
_MODE_MISSING = object()

# 三种合法阶段；both = 两通道同时生效（未接阶段机的场景行为逐字不变）
_PHASE_ROTATE = "rotate"
_PHASE_TRANSLATE = "translate"
_PHASE_BOTH = "both"
_PHASE_VALUES = (_PHASE_ROTATE, _PHASE_TRANSLATE, _PHASE_BOTH)
_PHASE_MISSING = object()


def _read_blackboard(client, key, default=None):
    """读黑板键；py_trees 的 Client.__getattr__ 抛 KeyError（非 AttributeError），
    getattr(client, key, None) 接不住，须显式 try。故意只接 KeyError：
    AttributeError 意味着键未注册，应响亮抛出。"""
    try:
        return getattr(client, key)
    except KeyError:
        return default


def _read_version(client, key) -> int:
    """读 `key_version`；从未写过 / 不是数字都当 0。"""
    try:
        return int(_read_blackboard(client, key, 0))
    except (TypeError, ValueError):
        return 0


def _read_idle(client, key) -> bool:
    """读节拍门禁键；从未写过（执行节点还没构造）当作空闲，别把树卡死。"""
    return bool(_read_blackboard(client, key, True))


def _read_mode(client, key, fallback: str) -> str:
    """读运行模式并收敛到 {position, velocity}；不可用（KeyError/None/非法值）回退 fallback。

    模式选择节点会在自己 __init__ 里给 mode_key 置初值，但两节点初始化顺序在
    py_trees 里不保证，故本节点必须自己兜底，不能假设键已存在。
    键不存在或值为 None（还没判决）属正常启动态，静默回退；
    值非法（既非 position 也非 velocity）才告警。
    """
    raw = _read_blackboard(client, key, _MODE_MISSING)
    if raw is _MODE_MISSING or raw is None:
        return fallback
    if raw in (_MODE_POSITION, _MODE_VELOCITY):
        return raw
    logger.warning(
        "[chassis_servo_calc] 黑板 %s 值非法（%r，期望 position|velocity），"
        "回退静态 mode=%s", key, raw, fallback)
    return fallback


def _read_phase(client, key, fallback: str) -> str:
    """读阶段并收敛到 {rotate, translate, both}；不可用回退 fallback。

    与 _read_mode 同口径：阶段节点会先置初值，但初始化顺序不保证，故自己兜底。
    键不存在或值为 None 属正常启动态，静默回退（默认 both = 两通道同时生效，
    即未接阶段机的场景行为逐字不变）；值非法才告警。
    """
    raw = _read_blackboard(client, key, _PHASE_MISSING)
    if raw is _PHASE_MISSING or raw is None:
        return fallback
    if raw in _PHASE_VALUES:
        return raw
    logger.warning(
        "[chassis_servo_calc] 黑板 %s 值非法（%r，期望 rotate|translate|both），"
        "回退静态 phase=%s", key, raw, fallback)
    return fallback


@define_manifest(
    label="底盘伺服计算",
    category=["skill", "chassis", "servo"],
    tree_type="studio_smoke",
    description="读黑板 AlignObservation → 控制律算 ServoCommand → 写回黑板（纯计算，不碰底盘）",
    params=[
        {"name": "obs_key", "type": "string", "default": "latest_servo_error",
         "description": "读观测量的黑板键名"},
        {"name": "command_key", "type": "string", "default": "latest_servo_command",
         "description": "写 ServoCommand 的黑板键名"},
        {"name": "idle_key", "type": "string", "default": "servo_exec_idle",
         "description": "读节拍门禁的黑板键名（False=底盘在途，本节点原地等待）"},
        {"name": "mode", "type": "string", "default": "position",
         "description": "控制模式：position=位置增量(dx/dy/dtheta)，velocity=速度指令(vx/vy/wz)。"
                        "这是初值，也是黑板 mode_key 不可用（键不存在/None/非法值）时的回退值"},
        {"name": "phase_key", "type": "string", "default": "servo_phase",
         "description": "读运行阶段的黑板键名（阶段选择节点写；缺省键不存在时回退静态 phase）"},
        {"name": "phase", "type": "string", "default": "both",
         "description": "阶段：rotate=只发角度通道(vx/vy 置零)，translate=只发平移通道(wz 置零)，"
                        "both=两通道同时生效。这是初值，也是黑板 phase_key 不可用时的回退值"},
        {"name": "mode_key", "type": "string", "default": "servo_mode",
         "description": "读运行模式的黑板键名；每个 tick 实时读取，缺省时回退到静态 mode 参数"},
        {"name": "gain", "type": "float", "default": "0.6", "description": "xy 通道 P 增益"},
        {"name": "mm_per_px", "type": "float", "default": "10.0",
         "description": "像素→毫米换算系数（真机标定），dx = gain*e_px*mm_per_px/1000"},
        {"name": "max_step_m", "type": "float", "default": "0.015",
         "description": "单步 xy 位移上限(m)，须明显大于执行节点的 position_threshold，否则底盘一步都不走"},
        {"name": "servo_gain_theta", "type": "float", "default": "0.6",
         "description": "角度通道 P 增益"},
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
        {"name": "axis_deadband_px", "type": "float", "default": "0.0",
         "description": "逐轴死区(px)：某轴像素偏差小于此值时该轴速度置零（在死区补偿之前），"
                        "使两轴收敛互不拖累；0=关闭。建议与 judge 的 servo_stop_threshold_px 同值"},
        {"name": "servo_cumulative_limit_m", "type": "float", "default": "0.10",
         "description": "累积位移安全保护上限(m)"},
        {"name": "servo_max_iters", "type": "int", "default": "8",
         "description": "最大迭代次数"},
    ],
    inputs=[{"name": "latest_servo_error", "type": "object",
             "description": "AlignObservation/ServoError：两路像素偏差 + 偏差角 + valid"}],
    outputs=[{"name": "latest_servo_command", "type": "object",
              "description": "ServoCommand：按 mode 填 dx/dy/dtheta 或 vx/vy/wz + 累积量 + stop_reason"}],
)
class NodeChassisServoCalc(BaseAction):
    """底盘伺服计算节点（纯计算）：AlignObservation → ServoCommand。"""

    # "这个键从未被写过"的哨兵（不能拿 None/0 代替，理由见 node_pallet_pose）
    _MISSING = object()

    def __init__(self, name, label, namespace, params):
        super(NodeChassisServoCalc, self).__init__(name, label, namespace, params)
        self._obs_key = str(self.params.get("obs_key", "latest_servo_error"))
        self._cmd_key = str(self.params.get("command_key", "latest_servo_command"))
        self._idle_key = str(self.params.get("idle_key", "servo_exec_idle"))
        # 运行模式每 tick 从黑板读（模式选择节点写）；静态 mode 参数是初值 / 回退值
        self._mode_key = str(self.params.get("mode_key", "servo_mode"))
        self._mode_fallback = str(self.params.get("mode", "position"))
        # 运行阶段同理（阶段选择节点写 servo_phase）
        self._phase_key = str(self.params.get("phase_key", "servo_phase"))
        self._phase_fallback = str(self.params.get("phase", "both"))

        self._skill = None
        self._version_seen = None
        # 上一帧新观测被消费的 monotonic 时刻，用于速度模式累积量的真实 dt
        self._last_calc_ts = None

        # 注册黑板权限
        for key in (self._obs_key, f"{self._obs_key}_version", self._idle_key,
                    self._mode_key, self._phase_key):
            self.global_blackboard.register_key(
                key=key, access=py_trees.common.Access.READ)
        for key in (self._cmd_key, f"{self._cmd_key}_version"):
            self.global_blackboard.register_key(
                key=key, access=py_trees.common.Access.WRITE)
        # 先置初值，免得其他节点 getattr 时 KeyError
        self.global_blackboard.set(self._cmd_key, None)
        setattr(self.global_blackboard, f"{self._cmd_key}_version", 0)

    def initialise(self):
        self._skill = None
        self._version_seen = None
        self._last_calc_ts = None

    def update(self):
        if _DRY_RUN:
            self.feedback_message = "dry-run chassis_servo_calc"
            return Status.SUCCESS

        # 节拍门禁：上一步还在途中就不递推。否则 iter/cum 会虚涨，而算出来的新指令
        # 又会被执行节点（在途期间不接新指令）直接丢掉。
        if not _read_idle(self.global_blackboard, self._idle_key):
            self.feedback_message = "底盘在途，等到达后再算下一步"
            return Status.RUNNING

        # 读观测量（哨兵区分"从未写过" vs "写过 None"）
        obs = _read_blackboard(self.global_blackboard, self._obs_key, self._MISSING)
        if obs is self._MISSING:
            self.feedback_message = f"等观测：{self._obs_key} 从未被写过"
            return Status.RUNNING
        if obs is None:
            self.feedback_message = "等观测（黑板上还没有 AlignObservation）"
            return Status.RUNNING
        if not is_observation_like(obs):
            self.feedback_message = (
                f"{self._obs_key} 不是观测量（缺 e_bottom_px/e_right_px/theta_rad）")
            return Status.FAILURE
        if not observation_field(obs, "valid", True):
            self.feedback_message = (
                f"观测无效：{observation_field(obs, 'reject', '') or 'unknown'}")
            return Status.FAILURE

        # 版本门禁：观测没更新就不重算
        newest_version = _read_version(self.global_blackboard, f"{self._obs_key}_version")
        if newest_version == self._version_seen:
            return Status.RUNNING

        # 真实帧间隔：版本门禁期间执行节点每个 tick 都在重发同一速度，
        # 所以累积量必须按「上一帧新观测 → 本帧」的实际耗时累计，不能用名义 50Hz。
        # 首帧（含 initialise 后的 settle 时间）记 0，不把等待时间算成位移。
        now = time.monotonic()
        dt_s = 0.0 if self._last_calc_ts is None else max(0.0, now - self._last_calc_ts)
        if dt_s > 0.0:
            logger.info(
                "chassis_servo_calc: 观测更新 dt=%.3fs rate=%.2fHz obs_ver=%s",
                dt_s, 1.0 / dt_s, newest_version,
            )

        # 读上一轮 ServoCommand，取 iter_index / cumulative_xy_m 做递推
        prev_cmd = _read_blackboard(self.global_blackboard, self._cmd_key, self._MISSING)
        if prev_cmd is self._MISSING or prev_cmd is None:
            iter_index = 0
            cumulative_xy_m = 0.0
        else:
            iter_index = int(getattr(prev_cmd, "iter_index", 0)) + 1
            cumulative_xy_m = float(getattr(prev_cmd, "cumulative_xy_m", 0.0))

        # 运行模式每 tick 从黑板取（模式选择节点写 servo_mode）；读不到 / 非法回退静态 mode
        mode = _read_mode(self.global_blackboard, self._mode_key, self._mode_fallback)
        # 运行阶段同理（阶段选择节点写 servo_phase）；缺省键不存在→回退 both（行为不变）
        phase = _read_phase(self.global_blackboard, self._phase_key, self._phase_fallback)

        skill_params = ChassisServoCalcParams(
            observation=obs,
            iter_index=iter_index,
            cumulative_xy_m=cumulative_xy_m,
            dt_s=dt_s,
            mode=mode,
            phase=phase,
            gain=float(self.params.get("gain", 0.6)),
            mm_per_px=float(self.params.get("mm_per_px", 10.0)),
            max_step_m=float(self.params.get("max_step_m", 0.015)),
            servo_gain_theta=float(self.params.get("servo_gain_theta", 0.6)),
            servo_max_dtheta_rad=float(self.params.get("servo_max_dtheta_rad", 0.05)),
            servo_gain_vel=float(self.params.get("servo_gain_vel", 0.6)),
            servo_gain_vel_theta=float(self.params.get("servo_gain_vel_theta", 0.6)),
            v_max_mps=float(self.params.get("v_max_mps", 0.05)),
            w_max_radps=float(self.params.get("w_max_radps", 0.15)),
            v_min_mps=float(self.params.get("v_min_mps", 0.02)),
            w_min_radps=float(self.params.get("w_min_radps", 0.0)),
            axis_deadband_px=float(self.params.get("axis_deadband_px", 0.0)),
            servo_cumulative_limit_m=float(self.params.get("servo_cumulative_limit_m", 0.10)),
            servo_max_iters=int(self.params.get("servo_max_iters", 8)),
        )

        if self._skill is None:
            self._skill = ChassisServoCalcSkill()
        init = self._skill.initialize(skill_params)
        if not init.success:
            self.feedback_message = init.message
            return Status.FAILURE

        result = self._skill.execute()
        if not result.success or not result.data:
            self.feedback_message = result.message
            return Status.FAILURE

        command = result.data["command"]
        self.global_blackboard.set(self._cmd_key, command)
        cur_ver = getattr(self.global_blackboard, f"{self._cmd_key}_version", 0)
        setattr(self.global_blackboard, f"{self._cmd_key}_version", cur_ver + 1)
        self._version_seen = newest_version
        self._last_calc_ts = now
        self.feedback_message = result.message
        return Status.RUNNING
