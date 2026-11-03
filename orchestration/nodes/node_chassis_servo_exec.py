# -*- coding: utf-8 -*-
"""NodeChassisServoExec：读黑板 ServoCommand → 下发底盘 → 等到达 → 写回 task_id（硬件节点）。

飞书 §3.3 / §4.3 的「执行」薄封装：读控制器写下的 ServoCommand，复用
ChassisServoExecSkill（内部走 base_move_relative_jibot）下发底盘移动，把返回的
task_id 写回黑板，并轮询 check_arrived_jibot 直到这一步真正走完。

为什么要等到达
--------------
`base_move_relative_jibot` 是**异步**的：拿到 task_id 就返回，动作在底盘侧自己跑。
不等到达会连环出事：

* 下一条指令在同一秒内下发 —— task_id 是秒级时间戳（`move_task_HH:MM:SS`），两条
  撞车，前一条被底盘判成 `interrupted`；
* 树在动作还在途中就收口 —— `NodeEnableVelControl.terminate()` 把
  `/enable_vel_control` 还原成 True，导航当场失去轮子，在途的 base_move 被掐断，
  现象就是「权限切成功了、底盘却没动」。

等到达之后，收口那一刻必然没有在途动作，还原权限才是安全的。
轮询用 `blocking=False`：`blocking=True` 会把整棵树的主循环整个卡住几十秒。

节拍门禁
--------
在途期间往黑板写 `servo_exec_idle=False`，NodeInjectAlignObs / NodeChassisServoCalc /
NodeChassisServoJudge 读到 False 就原地等待。否则主循环会在底盘还没走完这 10cm
时，把后面几帧观测、几步指令、几次判决全烧掉。

速度模式的差异（DESIGN §4.4）
-----------------------------
位置模式的 `servo_exec_idle` 是「有没有异步动作在途」的产物；速度**没有在途概念**
（发完即返回），需要的是每 tick 持续重发，故速度模式下 `servo_exec_idle` **恒 True**，
且不做指令去重（同一 ServoCommand 每帧都要重发，否则 /cmd_vel 断流、底盘 200 ms 归零）。
这样 calc / judge / inject 侧读 `servo_exec_idle` 的逻辑一行都不用改。

速度模式安全兜底（DESIGN §4.5，跨 tick 的两项在这层）
-----------------------------------------------------
* 观测看门狗 `obs_timeout_s=0.5`：超过这么久没收到新观测（版本号不变）→ 立即发零速；
* 单模式时长上限 `velocity_timeout_s=10`：速度模式连续超过它 → 停车退出；
* 退出/终止（含树 Ctrl-C 的 `terminate()`）一律先发零速。

⚠️ 权限默认值是 True（DESIGN §4.5.1），下位机门槛本就敞开，零速兜底是唯一防线。

速度模式的执行通道（`command_channel`，父议题 Q7/Q8）
----------------------------------------------------
* `humanoid`（默认）：每 tick 由技能发一帧 `/cmd_vel`（`send_base_velocity_tick`），
  行为与引入该参数前逐字一致；停车路径也会调 `_stop_direct_channel()`，未起流时
  是幂等 no-op，不会启动或改写流状态；
* `chassis_direct`：`initialise()` 起一条 100Hz 后台流（`start_cmd_vel_stream`），
  每 tick 只 `update_cmd_vel_setpoint(vx, vy, wz)`，由适配器线程按频率重发，
  **不再逐帧单发**（Q8）。`cmd_vel_topic` / `publish_rate_hz` 只在该通道生效。

单门控（Q6/Q10/Q23A）
--------------------
只在 `chassis_direct` 下评估，每 tick 调 `get_motion_gate()`：

* 从未收到 `BaseCmdVelStatus`（`effective_age_s is None`）→ 视为有效，不阻断；
* 收到消息则按 `cmd_vel_effective` 判，连续 `_GATE_INVALID_FRAMES` 帧为 false →
  置零停流 → FAILURE，且**不自动恢复**（Q11，需重新 initialise）；
* `effective_age_s` 只写日志，不参与阻断；`RobotStatus.mode` 不做门控（Q18）。

停车时序（Q12，不可调换）与权限分工
----------------------------------
`chassis_direct` 的任何退出路径统一走 `_stop_direct_channel()`：
`setpoint 置零并保持 _DIRECT_ZERO_HOLD_S` → `stop_cmd_vel_stream()` → 返回；
`terminate()` 也调它，靠 `_direct_streaming` / `_stream_stop_pending` 保证幂等
与「停不掉就留着重试」。

`enable_vel_control` 的还原**不在本节点**：由树里的 `NodeEnableVelControl`
负责（Q5）。因此本节点 terminate 只负责让底盘先静止，不能提前交还控制权——
先停流、后由权限节点还原，顺序依据是「交还控制权时不允许还有本节点的非零速度
在流里」。

黑板契约
--------
读 latest_servo_command           ServoCommand（控制器节点写入）
读 latest_servo_error_version     观测版本号，速度模式看门狗用
写 current_task_id                base_move_relative_jibot 返回的任务ID（速度模式恒空）
写 servo_exec_idle                bool，True = 底盘空闲、可接下一步

状态语义
--------
* 正常步 / 等到达 / 速度模式持续发布 → RUNNING；
* `stop_reason=converged` → SUCCESS（正常收敛收口）；
* `stop_reason=cum_limit/iter_limit/guard` → FAILURE（未对准，明确失败）。
"""

import os
import time

import py_trees
from py_trees.common import Status

from core.common.logger import get_logger
from core.domain.servo_types import ServoCommand
from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.chassis_servo_exec import (
    ChassisServoExecParams, ChassisServoExecSkill)

logger = get_logger(__name__)

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")

# stop_reason 里哪些算「没对准」：超累积量 / 超迭代 / 守护终止 → 节点返回 FAILURE。
# converged 是正常收敛，仍按 SUCCESS 收口。
_FAILURE_STOP_REASONS = frozenset({"cum_limit", "iter_limit", "guard"})

# 真机确认项（Q10）：连续多少帧 `cmd_vel_effective=false` 判失败。
# 只改本常量，不要改判定结构；改前需真机复测。
_GATE_INVALID_FRAMES = 5

# 停车时序（Q12）：setpoint 置零后保持多久再停流，保证底盘收到零。
_DIRECT_ZERO_HOLD_S = 0.2

# 速度模式的合法执行通道（L-2）：未知取值是配置错误，发布任何帧之前就拒绝。
_VALID_COMMAND_CHANNELS = frozenset({"humanoid", "chassis_direct"})


def _read_blackboard(client, key, default=None):
    """读黑板键；py_trees Client.__getattr__ 抛 KeyError，须显式 try（同 node_pallet_pose）。"""
    try:
        return getattr(client, key)
    except KeyError:
        return default


def _read_version(client, key) -> int:
    """读版本号；缺失/非数字按 0（同 node_chassis_servo_calc/judge）。"""
    try:
        return int(_read_blackboard(client, key, 0))
    except (TypeError, ValueError):
        return 0


def _parse_bool(value, default=True) -> bool:
    """JSON 里的 bool 到这儿可能是字符串 'false'，而 bool('false') 是 True。"""
    if isinstance(value, bool):
        return value
    if value is None:
        return default
    text = str(value).strip().lower()
    if text in ("true", "1", "yes", "y", "on"):
        return True
    if text in ("false", "0", "no", "n", "off"):
        return False
    return default


@define_manifest(
    label="底盘伺服执行",
    category=["skill", "chassis", "servo"],
    tree_type="studio_smoke",
    description="读黑板 ServoCommand → base_move_relative_jibot 下发 → 轮询等到达 → 写回 task_id",
    params=[
        {"name": "command_key", "type": "string", "default": "latest_servo_command",
         "description": "读 ServoCommand 的黑板键名"},
        {"name": "task_id_key", "type": "string", "default": "current_task_id",
         "description": "写 task_id 的黑板键名"},
        {"name": "idle_key", "type": "string", "default": "servo_exec_idle",
         "description": "写节拍门禁的黑板键名（True=底盘空闲，下游才往下走一步）"},
        {"name": "obs_key", "type": "string", "default": "latest_servo_error",
         "description": "读观测版本号的黑板键名（速度模式看门狗据此判断观测是否更新）"},
        {"name": "obs_timeout_s", "type": "float", "default": "0.5",
         "description": "观测看门狗(s)：速度模式下超过这么久没收到新观测即发零速（超时未更新=视觉断了，不能盲跑）"},
        {"name": "velocity_timeout_s", "type": "float", "default": "10.0",
         "description": "单模式时长上限(s)：速度模式连续跑超过它即停车退出（失控保险丝）"},
        {"name": "position_threshold", "type": "float", "default": "0.08",
         "description": "位置到达阈值(m)：剩余距离小于它就判到达、一步不走，故单步位移必须明显大于它"},
        {"name": "angle_threshold", "type": "float", "default": "0.1",
         "description": "角度到达阈值(rad)：同理，单步 dtheta 必须明显大于它，否则转身不会执行"},
        {"name": "linear_velocity", "type": "float", "default": "0.15",
         "description": "线速度(m/s)"},
        {"name": "angular_velocity", "type": "float", "default": "0.25",
         "description": "角速度(rad/s)"},
        {"name": "allow_rotation", "type": "bool", "default": "True",
         "description": "是否允许旋转"},
        {"name": "wait_arrival", "type": "bool", "default": "True",
         "description": "是否轮询 check_arrived 等这一步走完（关掉即恢复发完就走的旧行为）"},
        {"name": "arrival_timeout", "type": "float", "default": "20.0",
         "description": "单步等到达上限(s)，超时放行并让下游继续"},
        {"name": "arrival_poll_interval", "type": "float", "default": "0.2",
         "description": "轮询 check_arrived 的最小间隔(s)，避免每 tick 都打一次服务"},
        {"name": "command_channel", "type": "string", "default": "humanoid",
         "description": "速度模式执行通道：'humanoid' = 现有 /cmd_vel 逐帧通路；'chassis_direct' = 底盘原生速度流"},
        {"name": "cmd_vel_topic", "type": "string", "default": "/move_base/base_cmd_vel",
         "description": "底盘原生速度话题，仅 command_channel='chassis_direct' 生效"},
        {"name": "publish_rate_hz", "type": "float", "default": "100.0",
         "description": "底盘原生速度流频率(Hz)，仅 command_channel='chassis_direct' 生效"},
    ],
    inputs=[{"name": "latest_servo_command", "type": "object",
             "description": "ServoCommand：dx/dy/dtheta + 累积量 + stop_reason"}],
    outputs=[{"name": "current_task_id", "type": "string",
              "description": "base_move_relative_jibot 返回的任务ID"},
             {"name": "servo_exec_idle", "type": "bool",
              "description": "节拍门禁：True=底盘空闲可接下一步，False=有动作在途"}],
)
class NodeChassisServoExec(BaseAction):
    """底盘伺服执行节点（硬件）：ServoCommand → base_move_relative_jibot → 等到达。"""

    _MISSING = object()

    def __init__(self, name, label, namespace, params):
        super(NodeChassisServoExec, self).__init__(name, label, namespace, params)
        self._cmd_key = str(self.params.get("command_key", "latest_servo_command"))
        self._task_key = str(self.params.get("task_id_key", "current_task_id"))
        self._idle_key = str(self.params.get("idle_key", "servo_exec_idle"))
        self._obs_key = str(self.params.get("obs_key", "latest_servo_error"))
        self._wait_arrival = _parse_bool(self.params.get("wait_arrival", True), True)
        self._arrival_timeout = float(self.params.get("arrival_timeout", 20.0))
        self._poll_interval = float(self.params.get("arrival_poll_interval", 0.2))
        # 速度模式安全兜底（DESIGN §4.5）
        self._obs_timeout = float(self.params.get("obs_timeout_s", 0.5))
        self._velocity_timeout = float(self.params.get("velocity_timeout_s", 10.0))
        # 速度模式执行通道（父议题 Q7/Q8）
        self._channel = str(self.params.get("command_channel", "humanoid"))
        # L-2：未知通道是配置错误，在 update() 最前面就判失败，一帧都不发
        self._channel_invalid = self._channel not in _VALID_COMMAND_CHANNELS
        self._cmd_vel_topic = str(
            self.params.get("cmd_vel_topic", "/move_base/base_cmd_vel"))
        self._publish_rate = float(self.params.get("publish_rate_hz", 100.0))

        self._skill = None
        self._dry_done = False
        # 记录上一次执行的 ServoCommand 的 id()，避免同一指令重复下发
        self._last_cmd_id = None
        # 在途动作：task_id / 下发时刻 / 上次轮询时刻
        self._pending_task_id = None
        self._pending_since = 0.0
        self._last_poll = 0.0
        # 速度模式看门狗状态：进入时刻 / 上次观测版本 / 上次观测更新时刻
        self._vel_since = None
        self._vel_obs_version = None
        self._vel_last_obs_time = None
        # chassis_direct 通道：流是否由本节点启动 / 是否有未完成的停车 / 门控连续无效帧数 / 门控失败闩锁
        self._direct_streaming = False
        self._stream_stop_pending = False
        self._gate_false_frames = 0
        self._gate_tripped = False

        self.global_blackboard.register_key(
            key=self._cmd_key, access=py_trees.common.Access.READ)
        self.global_blackboard.register_key(
            key=f"{self._obs_key}_version", access=py_trees.common.Access.READ)
        for key in (self._task_key, self._idle_key):
            self.global_blackboard.register_key(
                key=key, access=py_trees.common.Access.WRITE)
        self.global_blackboard.set(self._task_key, "")
        self.global_blackboard.set(self._idle_key, True)

    def initialise(self):
        self._skill = None
        self._dry_done = False
        self._last_cmd_id = None
        self._pending_task_id = None
        self._reset_velocity_watch()
        # 上一轮若异常退出、流仍在跑，先停掉再重入（幂等；平时是 no-op）
        self._stop_direct_channel()
        self._gate_false_frames = 0
        self._gate_tripped = False
        # chassis_direct：流生命周期绑定节点生命周期（Q8，start 在 initialise）。
        # humanoid 通道不碰流：`_channel` 判断保证 start 永不被调用；dry-run 同样不起流。
        if self._channel == "chassis_direct" and not _DRY_RUN:
            self._ensure_direct_stream()
        self._set_idle(True)

    def update(self):
        # L-2：未知 command_channel 是配置错误，任何发布（含零速兜底）之前就失败
        if self._channel_invalid:
            self.feedback_message = (
                f"未知 command_channel={self._channel!r}，"
                f"只支持 humanoid / chassis_direct")
            return Status.FAILURE

        if _DRY_RUN:
            if not self._dry_done:
                self.feedback_message = "dry-run chassis_servo_exec"
                self._dry_done = True
            return Status.SUCCESS if self._dry_done else Status.RUNNING

        # 有动作在途：先把它等完，期间一律不接新指令（否则 task_id 撞车、前一条被打断）
        if self._pending_task_id:
            return self._poll_arrival()

        cmd = _read_blackboard(self.global_blackboard, self._cmd_key, self._MISSING)
        if cmd is self._MISSING or cmd is None:
            self.feedback_message = "等伺服指令（黑板上还没有 ServoCommand）"
            return Status.RUNNING
        if not isinstance(cmd, ServoCommand):
            self.feedback_message = f"{self._cmd_key} 不是 ServoCommand"
            return Status.FAILURE

        # 双模分派（DESIGN §4.3/§4.4）：速度模式每 tick 重发，不走去重/等到达
        if cmd.mode == "velocity":
            return self._update_velocity(cmd)

        # 从速度模式切回位置模式：清速度看门狗状态。位置模式路径本身不碰速度，
        # 但失败路径**故意**保留了 `_vel_since`（见 `_reset_velocity_watch` 注释），
        # 若在这里不清，下一次再切回速度模式时会带着旧的 `elapsed` 立即撞上时长上限、
        # 被误判为「刚进速度模式就超时」。只在 `_vel_since` 非空（确实跑过速度模式）
        # 时触发，`chassis_servo_v1` 等纯位置场景 `_vel_since` 恒为 None，零影响。
        if self._vel_since is not None:
            self._reset_velocity_watch()
        # chassis_direct 起过流时，位置模式不得带着流跑（未起流时是 no-op）
        self._stop_direct_channel()

        # 同一指令对象不重复下发（控制器每个 tick 重算后才换新对象）
        cmd_id = id(cmd)
        if cmd_id == self._last_cmd_id:
            return Status.RUNNING
        self._last_cmd_id = cmd_id

        skill_params = ChassisServoExecParams(
            command=cmd,
            position_threshold=float(self.params.get("position_threshold", 0.08)),
            angle_threshold=float(self.params.get("angle_threshold", 0.1)),
            linear_velocity=float(self.params.get("linear_velocity", 0.15)),
            angular_velocity=float(self.params.get("angular_velocity", 0.25)),
            allow_rotation=_parse_bool(self.params.get("allow_rotation", True), True),
            command_channel=self._channel,
            cmd_vel_topic=self._cmd_vel_topic,
            publish_rate_hz=self._publish_rate,
        )

        self._skill = ChassisServoExecSkill(hardware=get_shared_hardware())
        init = self._skill.initialize(skill_params)
        if not init.success:
            self.feedback_message = init.message
            return Status.FAILURE

        result = self._skill.execute()
        if not result.success:
            self.feedback_message = result.message or "chassis_servo_exec failed"
            return Status.FAILURE

        task_id = str(result.data.get("task_id", "")) if result.data else ""
        self.global_blackboard.set(self._task_key, task_id)

        # 跳过移动（本来就没有在途动作）：可收口
        if result.data and result.data.get("skipped") and not result.data.get("stop_reason"):
            self.feedback_message = result.message
            self._set_idle(True)
            return Status.SUCCESS

        # 终止指令：converged = 正常收敛；cum_limit/iter_limit/guard = 未对准失败
        stop_reason = result.data.get("stop_reason", "") if result.data else ""
        if stop_reason:
            self.feedback_message = result.message
            self._set_idle(True)
            if stop_reason in _FAILURE_STOP_REASONS:
                logger.warning(
                    "chassis_servo_exec: 伺服未对准终止 stop_reason=%s → FAILURE",
                    stop_reason)
                return Status.FAILURE
            return Status.SUCCESS

        if not self._wait_arrival or not task_id:
            # 不等到达（或压根没拿到 task_id，无从查起）：沿用旧口径
            self.feedback_message = result.message
            self._set_idle(True)
            return Status.RUNNING

        now = time.monotonic()
        self._pending_task_id = task_id
        self._pending_since = now
        self._last_poll = now
        self._set_idle(False)
        self.feedback_message = f"{result.message}（在途，等到达）"
        logger.info("chassis_servo_exec: 下发 task_id=%s，开始等到达", task_id)
        return Status.RUNNING

    def _poll_arrival(self) -> Status:
        """非阻塞轮询 check_arrived，直到到达 / 被中断 / 超时。一律返回 RUNNING。

        返回 RUNNING 而不是 SUCCESS：本节点是伺服支路，收口由 judge 或 skip 分支决定；
        在这儿 SUCCESS 会让 Parallel(success_on_one) 提前把整棵树收掉。
        """
        now = time.monotonic()
        elapsed = now - self._pending_since

        if elapsed > self._arrival_timeout:
            self._clear_pending(
                f"等到达超时（{elapsed:.1f}s > {self._arrival_timeout}s），放行")
            return Status.RUNNING

        # 限频：主循环每 tick 都打一次 /move_base/check_arrived 太重
        if now - self._last_poll < self._poll_interval:
            self.feedback_message = f"在途 {elapsed:.1f}s（等到达）"
            return Status.RUNNING
        self._last_poll = now

        hw = get_shared_hardware()
        if not hasattr(hw, "check_arrived_jibot"):
            self._clear_pending("硬件无 check_arrived_jibot，不等到达")
            return Status.RUNNING

        res = hw.check_arrived_jibot(self._pending_task_id, blocking=False, timeout=0.0)
        if not res.success:
            # 查不到不等于没到，留给超时兜底
            self.feedback_message = f"在途 {elapsed:.1f}s，查到达失败：{res.message}"
            return Status.RUNNING

        data = res.data or {}
        message = str(data.get("message", ""))
        if data.get("arrived"):
            self._clear_pending(f"已到达（{message or 'arrived'}，用时 {elapsed:.1f}s）")
        elif message == "interrupted":
            self._clear_pending(f"任务被中断（interrupted，用时 {elapsed:.1f}s）")
        else:
            self.feedback_message = f"在途 {elapsed:.1f}s：{message or 'moving'}"
        return Status.RUNNING

    def _clear_pending(self, message):
        """结束在途状态：清 task_id、放开节拍门禁。"""
        self._pending_task_id = None
        self._set_idle(True)
        self.feedback_message = message
        logger.info("chassis_servo_exec: %s", message)

    def _set_idle(self, idle: bool):
        """写节拍门禁键：True = 底盘空闲，下游可以往下走一步。"""
        self.global_blackboard.set(self._idle_key, bool(idle))

    # --- 速度模式（DESIGN §4.3/§4.4/§4.5） ---

    def _reset_velocity_watch(self):
        """清速度模式看门狗状态（正常收敛 / 切回位置模式时调用）。

        **失败返回路径故意不调用本方法**（DESIGN §4.5）：若在超时/看门狗触发的
        FAILURE 上清状态，而本节点随后仍被继续 tick（如被包在 `FailureIsRunning`
        下、或 `test_bt_loop.py` 这类连续 tick 驱动），下一 tick 会以「首次进入速度
        模式」的姿态重新发布**同一条非零速度**，看门狗与时长上限双重防线自相抵消
        （观测停更时该速度会一轮一轮重复下发）。保留 `_vel_since` / `_vel_last_obs_time`
        后，`elapsed` / `stale_s` 在后续 tick 只增不减，保护持续生效（等价于闩锁）。
        切回位置模式时由 `update()` 的清理 guard 负责复位，勿在此处画蛇添足。
        """
        self._vel_since = None
        self._vel_obs_version = None
        self._vel_last_obs_time = None

    def _update_velocity(self, cmd) -> Status:
        """速度模式每 tick 的处理：持续重发 + 观测看门狗 + 时长上限。

        与位置模式的根本差异（§4.4）：速度没有「在途」概念，本方法**不走去重**、
        **不做等到达**，每 tick 都发一帧；`servo_exec_idle` 恒 True，让 calc/judge
        每帧都能推进。

        两条跨 tick 防线（时长上限 / 观测看门狗）在通道分派**之前**统一评估，
        按通道停车（`_stop_velocity_channel`），两条通路共用同一套判定；
        之后 `chassis_direct` 走 `_emit_direct`，`humanoid` 走原有技能通路（逐字不变）。

        四条失败返回路径**一律不清**看门狗状态（见 `_reset_velocity_watch` 注释）：
        失败后若本节点仍被 tick，`elapsed` / `stale_s` 只增不减，看门狗与时长上限持续
        生效，避免「以首次进入姿态重发同一条非零速度」把双重防线自相抵消。
        """
        now = time.monotonic()

        # 门控失败闩锁（Q11 不自动恢复）：已判失败后不得再发任何非零速度
        if self._gate_tripped:
            self.feedback_message = (
                f"运动门控连续 {_GATE_INVALID_FRAMES} 帧无效，已判失败（不自动恢复）")
            return Status.FAILURE

        # 首次进入速度模式：记录起点与当前观测版本
        if self._vel_since is None:
            self._vel_since = now
            self._vel_last_obs_time = now
            self._vel_obs_version = _read_version(
                self.global_blackboard, f"{self._obs_key}_version")

        # 单模式时长上限（§4.5）：连续跑太久没收敛 → 停车退出
        elapsed = now - self._vel_since
        if elapsed > self._velocity_timeout:
            self._stop_velocity_channel()
            # 故意不清看门狗状态：若之后仍被 tick，elapsed 继续只增不减，保护持续生效
            self._set_idle(True)
            self.feedback_message = (
                f"速度模式超时（{elapsed:.1f}s > {self._velocity_timeout}s），停车退出")
            logger.error("chassis_servo_exec: %s", self.feedback_message)
            return Status.FAILURE

        # 观测看门狗（§4.5）：观测版本变了才刷新时刻；超时未更新即发零速
        obs_version = _read_version(self.global_blackboard, f"{self._obs_key}_version")
        if obs_version != self._vel_obs_version:
            self._vel_obs_version = obs_version
            self._vel_last_obs_time = now
        else:
            stale_s = now - self._vel_last_obs_time
            if stale_s > self._obs_timeout:
                self._stop_velocity_channel()
                # 故意不清看门狗状态：stale_s 只增不减，后续 tick 继续发零速而非非零速
                self._set_idle(True)
                self.feedback_message = (
                    f"观测看门狗超时（{stale_s:.2f}s > "
                    f"{self._obs_timeout}s 无新观测），已发零速")
                logger.error("chassis_servo_exec: %s", self.feedback_message)
                return Status.FAILURE

        # 通道分派（Q7/Q8）：chassis_direct 只更新流的目标速度，不再逐帧单发
        if self._channel == "chassis_direct":
            return self._emit_direct(cmd)

        skill_params = ChassisServoExecParams(
            command=cmd,
            position_threshold=float(self.params.get("position_threshold", 0.08)),
            angle_threshold=float(self.params.get("angle_threshold", 0.1)),
            linear_velocity=float(self.params.get("linear_velocity", 0.15)),
            angular_velocity=float(self.params.get("angular_velocity", 0.25)),
            allow_rotation=_parse_bool(self.params.get("allow_rotation", True), True),
            command_channel=self._channel,
            cmd_vel_topic=self._cmd_vel_topic,
            publish_rate_hz=self._publish_rate,
        )

        self._skill = ChassisServoExecSkill(hardware=get_shared_hardware())
        init = self._skill.initialize(skill_params)
        if not init.success:
            self._stop_velocity()
            # 故意不清看门狗状态：失败后若继续被 tick，时长/观测保护仍生效，防非零速重复下发
            self.feedback_message = init.message
            return Status.FAILURE

        result = self._skill.execute()
        if not result.success:
            # 技能内部失败路径已发零速（§4.5）；但若失败来自**异常**（硬件抛异常被
            # SkillBase.execute 兜成 fail），零速还没发，这里再兜一次。零速幂等，多发无害。
            self._stop_velocity()
            # 故意不清看门狗状态（同上）：失败路径复位会让保护自相抵消
            self._set_idle(True)
            self.feedback_message = result.message or "chassis_servo_exec velocity failed"
            return Status.FAILURE

        # 速度模式下 task_id 恒空；节拍门禁恒 True
        self._set_idle(True)

        # 终止指令（stop_reason 非空）：技能已发零速。
        # converged = 正常收敛 → SUCCESS；cum_limit/iter_limit/guard = 未对准 → FAILURE。
        stop_reason = result.data.get("stop_reason", "") if result.data else ""
        if stop_reason:
            self.feedback_message = result.message
            if stop_reason in _FAILURE_STOP_REASONS:
                # 故意不清看门狗状态：失败后若继续被 tick，保护仍生效
                logger.warning(
                    "chassis_servo_exec: 伺服未对准终止 stop_reason=%s → FAILURE",
                    stop_reason)
                return Status.FAILURE
            self._reset_velocity_watch()
            return Status.SUCCESS

        self.feedback_message = result.message
        return Status.RUNNING

    # --- chassis_direct 通道（Q7/Q8/Q10/Q12） ---

    def _emit_direct(self, cmd) -> Status:
        """速度模式 · 底盘原生通道：每 tick 只更新流的目标速度。

        与 humanoid 的分工（Q8）：单帧发布不再由本节点逐帧做，后台流按
        `publish_rate_hz` 重发 `update_cmd_vel_setpoint` 设下的目标值，节点只负责
        门控、终止语义与停车时序。

        失败路径一律「先停流再返回」：`_stop_direct_channel()` 幂等且已含零速保持。
        """
        hw = get_shared_hardware()

        # 终止指令（收敛/超限）：先停车，再按语义收口。放在起流之前——终止帧
        # 不需要流，也就不必为了停车再把流拉起来。
        if cmd.stop_reason:
            self._stop_direct_channel()
            self._set_idle(True)
            self.feedback_message = (
                f"velocity(chassis_direct) stop_reason={cmd.stop_reason}")
            if cmd.stop_reason in _FAILURE_STOP_REASONS:
                # 故意不清看门狗状态：失败后若继续被 tick，保护仍生效
                logger.warning(
                    "chassis_servo_exec: 伺服未对准终止 stop_reason=%s → FAILURE",
                    cmd.stop_reason)
                return Status.FAILURE
            self._reset_velocity_watch()
            return Status.SUCCESS

        if not self._ensure_direct_stream():
            return Status.FAILURE

        # 单门控（Q6/Q10）：每 tick 评估，连续 _GATE_INVALID_FRAMES 帧无效才判失败
        if not self._evaluate_motion_gate(hw):
            self._stop_direct_channel()
            self._gate_tripped = True       # Q11：不自动恢复，需重新 initialise
            self._set_idle(True)
            self.feedback_message = (
                f"运动门控连续 {_GATE_INVALID_FRAMES} 帧无效"
                f"（cmd_vel_effective=false），已停车")
            logger.error("chassis_servo_exec: %s", self.feedback_message)
            return Status.FAILURE

        result = hw.update_cmd_vel_setpoint(
            float(cmd.vx_mps), float(cmd.vy_mps), float(cmd.wz_radps))
        if not result.success:
            self._stop_direct_channel()
            self._set_idle(True)
            self.feedback_message = result.message or "chassis_direct 更新目标速度失败"
            logger.error("chassis_servo_exec: %s", self.feedback_message)
            return Status.FAILURE

        # 速度模式下 task_id 恒空；节拍门禁恒 True
        self._set_idle(True)
        self.feedback_message = (
            f"velocity(chassis_direct) vx={cmd.vx_mps:.3f} "
            f"vy={cmd.vy_mps:.3f} wz={cmd.wz_radps:.3f}")
        return Status.RUNNING

    def _ensure_direct_stream(self) -> bool:
        """确保底盘原生速度流已启动；只起一次，失败返回 False。

        先置 `_direct_streaming=True` 再调 `start_cmd_vel_stream`：起流若在
        「线程已建、返回前抛异常」这种半完成状态中断，`_stop_direct_channel()`
        才有机会把线程收掉——杜绝「起了流却没有停止路径」。
        """
        if self._direct_streaming:
            return True

        hw = get_shared_hardware()
        for name in ("start_cmd_vel_stream", "update_cmd_vel_setpoint",
                     "stop_cmd_vel_stream"):
            if not hasattr(hw, name):
                self.feedback_message = f"硬件无 {name}，chassis_direct 通道不可用"
                logger.error("chassis_servo_exec: %s", self.feedback_message)
                return False

        self._direct_streaming = True
        try:
            result = hw.start_cmd_vel_stream(
                topic=self._cmd_vel_topic, rate_hz=self._publish_rate)
        except Exception as exc:  # noqa: BLE001
            logger.error(
                "chassis_servo_exec: 启动底盘速度流异常：%s", exc, exc_info=True)
            self._stop_direct_channel()
            self.feedback_message = f"启动底盘速度流异常：{exc}"
            return False

        if not result.success:
            logger.error(
                "chassis_servo_exec: 启动底盘速度流失败：%s", result.message)
            self._stop_direct_channel()
            self.feedback_message = result.message or "启动底盘速度流失败"
            return False

        # 新流已起，之前的「待停」已被本次启动覆盖
        self._stream_stop_pending = False
        logger.info(
            "chassis_servo_exec: 底盘原生速度流已启动 topic=%s rate_hz=%.1f",
            self._cmd_vel_topic, self._publish_rate)
        return True

    def _evaluate_motion_gate(self, hw) -> bool:
        """单门控（Q6/Q10/Q23A）：返回 True = 本帧允许继续运动。

        * 从未收到 `BaseCmdVelStatus`（`effective_age_s is None`）→ 视为有效，
          门控待命，不阻断（该话题真机可能没有发布者）；
        * 收到消息则按最后一条的 `cmd_vel_effective` 判；为 false 时连续
          `_GATE_INVALID_FRAMES` 帧才返回 False（第 4 帧仍放行）；
        * 中途变回 true → 计数清零（「连续」语义）；
        * `effective_age_s` 只写日志，不参与判定；`RobotStatus.mode` 不参与（Q18）。
        """
        if not hasattr(hw, "get_motion_gate"):
            # 硬件没有门控能力（旧适配器 / 伪硬件）：按「从未收到状态」处理
            self._gate_false_frames = 0
            return True

        gate = hw.get_motion_gate() or {}
        age = gate.get("effective_age_s")
        if age is None:
            self._gate_false_frames = 0
            logger.debug("chassis_servo_exec: 未收到 base_cmd_vel_status，门控待命（按有效处理）")
            return True

        if gate.get("cmd_vel_effective", True):
            if self._gate_false_frames:
                logger.info(
                    "chassis_servo_exec: 运动门控恢复有效（此前连续 %d 帧无效）",
                    self._gate_false_frames)
                self._gate_false_frames = 0
            logger.debug("chassis_servo_exec: 运动门控有效，状态消息 age=%.2fs", age)
            return True

        self._gate_false_frames += 1
        logger.warning(
            "chassis_servo_exec: 运动门控无效 reason=%s description=%s "
            "age=%.2fs（连续 %d/%d 帧）",
            gate.get("reason"), gate.get("description"), age,
            self._gate_false_frames, _GATE_INVALID_FRAMES)
        return self._gate_false_frames < _GATE_INVALID_FRAMES

    def _stop_direct_channel(self):
        """chassis_direct 停车时序（Q12，不可调换）：setpoint 置零并保持 → 停流。

        幂等：两个标志都为假时直接返回，重复调用不会重复睡眠/停流。

        `_stream_stop_pending` 在真正停流成功前一直保持，`_direct_streaming` 也
        只在停流成功后才清：中途取硬件/停流抛异常时状态不被抹掉，`terminate()`
        再调一次就能重试 —— 只要流可能还活着，就一定留着再停一次的路径。
        零速置零失败时**不睡**，直接停流——适配器 `stop_cmd_vel_stream()` 内部
        还有一次零速保持，是这条路径的最后兜底。
        """
        if not self._direct_streaming and not self._stream_stop_pending:
            return
        # 先挂上「停车未完成」，任何异常都不会让重试路径消失
        self._stream_stop_pending = True

        hw = None
        zero_ok = False
        try:
            hw = get_shared_hardware()
            hw.update_cmd_vel_setpoint(0.0, 0.0, 0.0)
            zero_ok = True
        except Exception:  # noqa: BLE001
            logger.error(
                "chassis_servo_exec: 停车前置零失败，直接停流", exc_info=True)
        if zero_ok:
            time.sleep(_DIRECT_ZERO_HOLD_S)

        stopped = False
        if hw is None:
            logger.error(
                "chassis_servo_exec: 停车时取不到硬件，保留待停标志供 terminate() 重试")
        else:
            try:
                result = hw.stop_cmd_vel_stream()
                stopped = result.success
                if not stopped:
                    logger.error(
                        "chassis_servo_exec: 停流失败：%s", result.message)
            except Exception:  # noqa: BLE001
                logger.error("chassis_servo_exec: 停流失败", exc_info=True)

        if stopped:
            self._direct_streaming = False
            self._stream_stop_pending = False

    def _stop_velocity_channel(self):
        """按当前通道停车：chassis_direct → 停流；其余 → 单帧零速（原行为）。"""
        if self._channel == "chassis_direct":
            self._stop_direct_channel()
        else:
            self._stop_velocity()

    def _stop_velocity(self):
        """零速兜底：best-effort，硬件不支持或抛异常都不阻断退出路径。"""
        try:
            hw = get_shared_hardware()
            if hasattr(hw, "send_base_velocity_tick"):
                hw.send_base_velocity_tick(0.0, 0.0, 0.0)
        except Exception:  # noqa: BLE001
            logger.error("chassis_servo_exec: 零速兜底发送失败", exc_info=True)

    def terminate(self, new_status):
        """树终止钩子（DESIGN §4.5）：速度模式在跑时，终止即停车。

        行为树被 Ctrl-C 或异常终止时这是**唯一**的兜底机会——最后一条非零速度还在
        下位机 200 ms 窗口内，而权限默认 True（§4.5.1），不兜底车会一直跑。
        权限还原由 `NodeEnableVelControl` / `NodeChassisServoMode.terminate()` 负责
        （Q5/§4.6）：本节点只把底盘停住，交还控制权交给权限节点，顺序不可反
        ——交还时不允许还有本节点的非零速度在流里。

        只在**本节点发过速度指令**（`_vel_since` 非空）时才停车：位置模式从不
        经 `/cmd_vel` 驱动底盘，多余地打一帧零速会平白改变 `chassis_servo_v1` /
        `real_v1` 的现有行为（位置模式逐字不动的约束）。
        `chassis_direct` 的停流本身幂等，重复 terminate 安全。
        """
        if _DRY_RUN:
            return
        if self._channel == "chassis_direct":
            # 流是 initialise 起的、绑定节点生命周期，与「本节点是否走过速度模式」无关，
            # 因此这里无条件停（幂等 no-op 于未起流时）
            self._stop_direct_channel()
        elif self._vel_since is not None:
            self._stop_velocity()
