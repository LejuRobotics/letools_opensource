# -*- coding: utf-8 -*-
"""Atomic skill: chassis_servo_exec —— 底盘伺服执行（ServoCommand → 底盘移动）。

飞书文档 §3.3 / §4.3 的「执行」模块：拿到控制器算出的 ServoCommand，复用
`base_move_relative_jibot` 把 dx/dy/dtheta 发给底盘。

阈值是「到达容差」不是「单步上限」
----------------------------------
`position_threshold` / `angle_threshold` 传给底盘后是**到达判据**：剩余距离小于它
就判已到达、一步都不走。所以约束方向是「单步位移必须明显**大于**阈值」，
而不是早先文档写的「单步必须 < 8cm」——那条说法把语义弄反了，会把控制量
一路压进死区（底盘定位分辨率 = position_threshold，阈值给大了等于让底盘不动）。
chassis_servo_v1 场景取 position_threshold=0.02 / angle_threshold=0.05，
配 calc 的 max_step_m=0.1、servo_max_dtheta_rad=0.2。

本技能只负责「发」，不等到达：`base_move_relative_jibot` 是异步的，拿到 task_id
就返回。轮询 `check_arrived_jibot` 等这一步走完，由上层节点
`orchestration/nodes/node_chassis_servo_exec.py` 做。

终止语义（§4.4）：ServoCommand.stop_reason 非空 = 伺服已终止（收敛/累积量超限/迭代超限/
守护），执行模块不再下发移动，直接返回带 skipped + stop_reason 标记的成功结果。
「成功」只是指「本帧已正确停车」，**不代表对准成功**：上层
`node_chassis_servo_exec` 会把 `cum_limit/iter_limit/guard` 翻译成 FAILURE，
只有 `converged` 才按 SUCCESS 收口。

位置 / 速度双模分派（DESIGN §4.3）
---------------------------------
按 `cmd.mode` 分派两条通路：

* `"position"`（默认）：走现有 `base_move_relative_jibot`，行为逐字不变；
* `"velocity"`：走 `send_base_velocity_tick`（**非阻塞单帧发布**），由行为树 50 Hz
  tick 每帧调用以维持 `/cmd_vel`（下位机 200 ms 超时清零，见 DESIGN §2.3 C1）。

速度模式不做单位换算：calc 输出的 vx/vy(m/s)、wz(rad/s) 与 `send_base_velocity_tick`
入参口径一致（DESIGN §2.3.1），exec 层原样透传。

安全兜底（DESIGN §4.5）：速度是「持续的」，失控不会自己停，故本技能内**任何**
失败/异常路径都先发零速再返回（`_send_zero_velocity`）。观测看门狗与单模式时长上限
属于跨 tick 的状态，由上层 `node_chassis_servo_exec` 负责（见该模块）。

本技能接 hardware 参数（遵循 base_move_relative_jibot 硬件技能范式）。
"""

from dataclasses import dataclass
from typing import Optional

from core.common.logger import get_logger
from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.domain.chassis_options import MoveToTargetOptions
from core.domain.servo_types import ServoCommand
from core.interfaces.i_hardware import IHardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.base.skill_base import SkillBase

logger = get_logger(__name__)

# 伺服终止原因：出现这些 stop_reason 时不再下发移动（§4.4 立即终止）
_TERMINATE_REASONS = frozenset({"cum_limit", "iter_limit", "guard", "converged"})

# 速度模式合法通道（L-2）：未知取值在 on_initialize 就拒绝，一帧都不发
_VALID_COMMAND_CHANNELS = frozenset({"humanoid", "chassis_direct"})


@dataclass
class ChassisServoExecParams(SkillParams):
    """一次伺服执行的输入：ServoCommand + base_move_relative_jibot 参数。

    position_threshold / angle_threshold 是到达容差（沿用 base_move_relative_jibot
    的默认值）。伺服微调的单步位移往往只有几厘米，用导航尺度的 0.08 会整个落进
    死区 —— 场景里应显式调小（chassis_servo_v1 取 0.02 / 0.05）。
    """

    skill_name: str = "chassis_servo_exec"
    command: Optional[ServoCommand] = None
    # base_move_relative_jibot 参数（沿用导航链路默认值，伺服场景需按单步量级调小阈值）
    avoid_enabled: bool = False
    avoid_distance: float = 0.5
    linear_velocity: float = 0.15
    angular_velocity: float = 0.25
    position_threshold: float = 0.08  # 到达容差(m)：单步位移须明显大于它，否则一步不走
    angle_threshold: float = 0.1      # 到达容差(rad)：同理，单步 dtheta 须明显大于它
    allow_rotation: bool = True
    timeout: float = 60.0
    # 速度模式通道分派：humanoid = 现有 /cmd_vel 通路；chassis_direct = 底盘原生话题
    command_channel: str = "humanoid"
    cmd_vel_topic: str = "/move_base/base_cmd_vel"  # 仅 chassis_direct 生效
    publish_rate_hz: float = 100.0  # 仅 chassis_direct 生效；本技能不启动流，仅供节点透传


@define_manifest(
    label="底盘伺服执行",
    category=["skill", "chassis", "servo"],
    tree_type="studio_smoke",
    description="ServoCommand → 底盘移动：复用 base_move_relative_jibot（异步下发，等到达由上层节点做）",
    params=[
        {"name": "position_threshold", "type": "float", "default": "0.08",
         "description": "位置到达容差(m)：剩余距离小于它就判到达、一步不走，单步位移须明显大于它"},
        {"name": "angle_threshold", "type": "float", "default": "0.1",
         "description": "角度到达容差(rad)：同理，单步 dtheta 须明显大于它，否则转身不执行"},
        {"name": "linear_velocity", "type": "float", "default": "0.15", "description": "线速度(m/s)"},
        {"name": "angular_velocity", "type": "float", "default": "0.25", "description": "角速度(rad/s)"},
        {"name": "allow_rotation", "type": "bool", "default": "True", "description": "是否允许旋转"},
        {"name": "command_channel", "type": "string", "default": "humanoid",
         "description": "速度模式通道：'humanoid' = 现有 /cmd_vel 通路；'chassis_direct' = 底盘原生速度话题"},
        {"name": "cmd_vel_topic", "type": "string", "default": "/move_base/base_cmd_vel",
         "description": "底盘原生速度话题，仅 command_channel='chassis_direct' 生效"},
        {"name": "publish_rate_hz", "type": "float", "default": "100.0",
         "description": "底盘原生速度流频率，仅 command_channel='chassis_direct' 生效（本技能不启动流，仅供节点透传）"},
    ],
    inputs=[{"name": "latest_servo_command", "type": "object",
             "description": "ServoCommand：dx/dy/dtheta + 累积量 + stop_reason"}],
    outputs=[{"name": "current_task_id", "type": "string",
              "description": "base_move_relative_jibot 返回的任务ID"}],
)
class ChassisServoExecSkill(SkillBase):
    """底盘伺服执行（Adapter）：ServoCommand → hardware.base_move_relative_jibot()。"""

    def __init__(self, hardware: IHardware):
        super().__init__(name="chassis_servo_exec")
        self.hardware = hardware
        self.params: Optional[ChassisServoExecParams] = None
        self._done = False
        self._result: Optional[Result] = None

    def on_initialize(self, params: ChassisServoExecParams) -> Result:
        if not isinstance(params, ChassisServoExecParams):
            return Result.fail("Invalid parameters for ChassisServoExecSkill")
        # L-2：未知通道是配置错误，在任何发布之前拒绝；先清掉旧 params，
        # 防止调用方忽略 init 失败后 execute() 复用上一轮的合法配置。
        if params.command_channel not in _VALID_COMMAND_CHANNELS:
            self.params = None
            return Result.fail(
                f"未知 command_channel={params.command_channel!r}，"
                f"只支持 humanoid / chassis_direct",
                error_code="INVALID_COMMAND_CHANNEL",
            )
        self.params = params
        self._done = False
        self._result = None
        return Result.ok()

    def on_execute(self) -> Result:
        if self._done:
            return self._result or Result.ok("ChassisServoExecSkill already finished")

        p = self.params
        cmd = p.command
        if cmd is None:
            self._done = True
            self._result = Result.fail("没有伺服指令（ServoCommand 为空）")
            return self._result

        # 双模分派（§4.3）：速度模式走独立分支，位置模式路径逐字保持不变
        if cmd.mode == "velocity":
            return self._dispatch_velocity(cmd)

        # 终止语义（§4.4）：stop_reason 非空 = 伺服已终止，不再下发移动
        if cmd.stop_reason in _TERMINATE_REASONS:
            self._done = True
            logger.info(
                "chassis_servo_exec: stop_reason=%s，跳过移动（伺服已终止）", cmd.stop_reason
            )
            self._result = Result.ok(
                f"skipped: stop_reason={cmd.stop_reason}",
                data={"task_id": "", "skipped": True, "stop_reason": cmd.stop_reason},
            )
            return self._result

        options = MoveToTargetOptions(
            avoid_enabled=bool(p.avoid_enabled),
            avoid_distance=float(p.avoid_distance),
            linear_velocity=float(p.linear_velocity),
            angular_velocity=float(p.angular_velocity),
            position_threshold=float(p.position_threshold),
            angle_threshold=float(p.angle_threshold),
            allow_rotation=bool(p.allow_rotation),
        )

        result = self.hardware.base_move_relative_jibot(
            x=float(cmd.dx_m),
            y=float(cmd.dy_m),
            theta=float(cmd.dtheta_rad),
            options=options,
        )
        self._done = True
        self._result = result
        if result.success:
            logger.info(
                "chassis_servo_exec: iter=%d dx=%.4f dy=%.4f dtheta=%.4f task_id=%s",
                cmd.iter_index, cmd.dx_m, cmd.dy_m, cmd.dtheta_rad,
                result.data.get("task_id") if result.data else "N/A",
            )
        return result

    def _dispatch_velocity(self, cmd: ServoCommand) -> Result:
        """按 command_channel 分派速度模式：humanoid（默认）/ chassis_direct。

        未知取值已在 `on_initialize()` 拒绝，这里只会收到两个合法值。
        """
        channel = self.params.command_channel
        if channel == "chassis_direct":
            return self._on_execute_velocity_chassis_direct(cmd)
        return self._on_execute_velocity(cmd)

    def _on_execute_velocity_chassis_direct(self, cmd: ServoCommand) -> Result:
        """速度模式 · 底盘原生通道：单帧发 base_cmd_vel，语义与 humanoid 分支一致。"""
        if cmd.stop_reason:
            # 终止指令（收敛/超限）：最后一步必须停车，不能把旧速度留着
            self._done = True
            self._send_zero_velocity()
            logger.info(
                "chassis_servo_exec: velocity(chassis_direct) stop_reason=%s，发零速后退出",
                cmd.stop_reason)
            self._result = Result.ok(
                f"velocity stopped: stop_reason={cmd.stop_reason}",
                data={"velocity": True, "skipped": True,
                      "stop_reason": cmd.stop_reason, "task_id": "",
                      "channel": "chassis_direct"},
            )
            return self._result

        hw = self.hardware
        if not hasattr(hw, "send_base_cmd_vel_tick"):
            self._done = True
            logger.error(
                "chassis_servo_exec: 硬件无 send_base_cmd_vel_tick，chassis_direct 通道不可用")
            self._result = Result.fail(
                "硬件无 send_base_cmd_vel_tick，chassis_direct 通道不可用")
            return self._result

        result = hw.send_base_cmd_vel_tick(
            float(cmd.vx_mps), float(cmd.vy_mps), float(cmd.wz_radps),
            topic=self.params.cmd_vel_topic)

        if result.success:
            self._done = True
            self._result = result
            data = dict(result.data) if result.data else {}
            data.update({"velocity": True, "skipped": False, "task_id": "",
                         "channel": "chassis_direct"})
            logger.info(
                "chassis_servo_exec: iter=%d velocity(chassis_direct) "
                "vx=%.4f vy=%.4f wz=%.4f topic=%s",
                cmd.iter_index, cmd.vx_mps, cmd.vy_mps, cmd.wz_radps,
                self.params.cmd_vel_topic)
            return Result.ok(result.message, data=data)

        # 发速度失败：立即零速兜底，避免底盘停留在上一条速度指令上
        self._done = True
        self._send_zero_velocity()
        logger.error(
            "chassis_servo_exec: velocity(chassis_direct) 下发失败：%s（已发零速）",
            result.message)
        self._result = Result.fail(result.message)
        return self._result

    def _on_execute_velocity(self, cmd: ServoCommand) -> Result:
        """速度模式（§4.3）：单帧非阻塞发布 /cmd_vel，不做单位换算、不发 task_id。

        与位置模式的根本区别：速度是**持续的**，本方法不 sleep、不轮询到达，发一帧
        就返回；持续发布由行为树每 tick 调用本技能保证（下位机 200 ms 超时清零）。
        因此 `skipped`/`task_id` 两个位置模式的概念在这里都不适用，返回带
        `velocity=True` 标记的数据供上层节点区分。

        安全（§4.5）：任何失败路径先发零速再返回，绝不让最后一条非零速度留在
        下位机的 200 ms 窗口里（权限默认 True，零速兜底是唯一防线）。
        """
        if cmd.stop_reason:
            # 终止指令（收敛/超限）：最后一步必须停车，不能把旧速度留着
            self._done = True
            self._send_zero_velocity()
            logger.info(
                "chassis_servo_exec: velocity 模式 stop_reason=%s，发零速后退出",
                cmd.stop_reason)
            self._result = Result.ok(
                f"velocity stopped: stop_reason={cmd.stop_reason}",
                data={"velocity": True, "skipped": True,
                      "stop_reason": cmd.stop_reason, "task_id": ""},
            )
            return self._result

        hw = self.hardware
        if not hasattr(hw, "send_base_velocity_tick"):
            # 硬件不支持非阻塞速度接口：视为失败，且零速兜底也无从发送
            self._done = True
            logger.error("chassis_servo_exec: 硬件无 send_base_velocity_tick，速度模式不可用")
            self._result = Result.fail("硬件无 send_base_velocity_tick，速度模式不可用")
            return self._result

        result = hw.send_base_velocity_tick(
            float(cmd.vx_mps), float(cmd.vy_mps), float(cmd.wz_radps))

        if result.success:
            self._done = True
            self._result = result
            data = dict(result.data) if result.data else {}
            data.update({"velocity": True, "skipped": False, "task_id": ""})
            logger.info(
                "chassis_servo_exec: iter=%d velocity vx=%.4f vy=%.4f wz=%.4f",
                cmd.iter_index, cmd.vx_mps, cmd.vy_mps, cmd.wz_radps)
            return Result.ok(result.message, data=data)

        # 发速度失败：立即零速兜底，避免底盘停留在上一条速度指令上
        self._done = True
        self._send_zero_velocity()
        logger.error("chassis_servo_exec: velocity 下发失败：%s（已发零速）", result.message)
        self._result = Result.fail(result.message)
        return self._result

    def _send_zero_velocity(self) -> None:
        """零速兜底：best-effort，硬件不支持或抛异常都不阻断退出路径（同 mode 节点）。

        走当前 command_channel：humanoid → send_base_velocity_tick；
        chassis_direct → send_base_cmd_vel_tick(topic=...)。
        """
        try:
            hw = self.hardware
            if self.params is not None and self.params.command_channel == "chassis_direct":
                if hasattr(hw, "send_base_cmd_vel_tick"):
                    hw.send_base_cmd_vel_tick(
                        0.0, 0.0, 0.0, topic=self.params.cmd_vel_topic)
                else:
                    logger.error(
                        "chassis_servo_exec: 硬件无 send_base_cmd_vel_tick，零速兜底无法发送")
                return
            if hasattr(hw, "send_base_velocity_tick"):
                hw.send_base_velocity_tick(0.0, 0.0, 0.0)
        except Exception:  # noqa: BLE001
            # 零速是兜底，发不出去不该让执行技能崩掉
            logger.error("chassis_servo_exec: 零速兜底发送失败", exc_info=True)

    def on_is_finished(self) -> bool:
        return self._done
