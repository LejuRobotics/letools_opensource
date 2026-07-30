# -*- coding: utf-8 -*-
"""Atomic skill: arm_ee_burst_timed.

轮臂末端独立控制 - 在线连发多个末端航点（循环 send_timed_single_command）。
躯干/底盘保持不动（初始化由脚手架/编排层负责，本技能只下发末端指令）。

对齐源脚本 cmd_arm_ee_traj_stream_test.py：每段独立规划，MPC 在线平滑插值
（二阶连续、三阶可导，有失真但满足运动学限制）。
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

# 默认航点: [[x, y, z, yaw, pitch, roll], ...]（位置米，姿态用户单位/默认度）
_DEFAULT_WAYPOINTS = [
    [0.3, 0.25, 0.5, 0.0, 0.0, 0.0],
    [0.5, 0.25, 0.5, 0.0, 0.0, 0.0],
    [0.3, 0.25, 0.7, 0.0, 0.0, 0.0],
]


@dataclass
class ArmEEBurstTimedParams(SkillParams):
    """在线连发航点参数。

    约定: waypoints [[x,y,z,yaw,pitch,roll], ...]，位置米；姿态单位由适配器 angle_unit 配置决定（默认度）。
    """

    skill_name: str = "arm_ee_burst_timed"
    side: str = "left"          # 'left' / 'right'
    frame: str = "world"        # 'world' / 'local'
    waypoints: List[List[float]] = field(default_factory=lambda: [list(p) for p in _DEFAULT_WAYPOINTS])
    desire_time: float = 3.0        # 每段期望执行时间（秒）
    settle_time: float = 1.0        # 每段后额外等待（秒）
    check_reach: bool = False       # 是否每段后查询静差
    reach_linear_tol: float = 0.01  # 静差位置容差（米）
    reach_angular_tol: float = 0.05  # 静差姿态容差（弧度）
    timeout: float = 300.0


@define_manifest(
    label="连发末端航点（TimedCmd）",
    category=["motion", "arm"],
    tree_type="studio_smoke",
    description="循环 send_timed_single_command 执行多航点，躯干/底盘不动，可选静差检查",
    params=[
        {"name": "side", "type": "string", "default": "left", "description": "手臂侧: 'left' / 'right'"},
        {"name": "frame", "type": "string", "default": "world", "description": "坐标系: 'world' / 'local'"},
        {"name": "waypoints", "type": "json", "default": "",
         "description": "航点 [[x,y,z,yaw,pitch,roll], ...]（米, 度）"},
        {"name": "desire_time", "type": "float", "default": "3.0", "description": "每段期望时间（秒）"},
        {"name": "settle_time", "type": "float", "default": "1.0", "description": "每段后额外等待（秒）"},
        {"name": "check_reach", "type": "bool", "default": "false", "description": "是否每段后查询静差"},
        {"name": "reach_linear_tol", "type": "float", "default": "0.01", "description": "静差位置容差（米）"},
        {"name": "reach_angular_tol", "type": "float", "default": "0.05", "description": "静差姿态容差（弧度）"},
    ],
    inputs=[],
    outputs=[],
)
class ArmEEBurstTimedSkill(SkillBase):
    """在线连发航点（循环 send_timed_single_command，单臂独立）。

    每段：发送 → sleep(desire_time + settle_time) → 可选 get_ee_pose_reach_error。
    不负责初始化（focus_ee=False、set_control_mode、复位等），由上层编排/脚手架负责。
    """

    def __init__(self, hardware: IHardware):
        super().__init__(name="arm_ee_burst_timed")
        self.hardware = hardware
        self.params: Optional[ArmEEBurstTimedParams] = None
        self._done = False

    def _check_reach_error(self, is_left: bool, linear_tol: float, angular_tol: float):
        """查询末端静差。

        :return: (status, err_vec, message)
            status: 'ok' 已到位 / 'out_of_tol' 超差 / 'unavailable' 服务不可用或查询失败
            err_vec: 6 维误差向量（status='ok'/'out_of_tol' 时有效），否则 None
        """
        fn = getattr(self.hardware, "get_ee_pose_reach_error", None)
        if fn is None:
            return 'unavailable', None, "Hardware does not implement get_ee_pose_reach_error()"
        res = fn(is_left=is_left)
        if not res.success:
            return 'unavailable', None, res.message
        err = (res.data or {}).get("err_vector", [])
        if len(err) != 6:
            return 'unavailable', None, f"静差向量维度异常: {err}"
        linear_err = (err[0] ** 2 + err[1] ** 2 + err[2] ** 2) ** 0.5
        angular_err = (err[3] ** 2 + err[4] ** 2 + err[5] ** 2) ** 0.5
        if linear_err > linear_tol or angular_err > angular_tol:
            return 'out_of_tol', err, (
                f"末端静差超差: 线={linear_err:.4f}m(容差{linear_tol}), "
                f"角={angular_err:.4f}rad(容差{angular_tol})")
        return 'ok', err, f"末端到位: 线={linear_err:.4f}m, 角={angular_err:.4f}rad"

    def on_initialize(self, params: ArmEEBurstTimedParams) -> Result:
        if not isinstance(params, ArmEEBurstTimedParams):
            return Result.fail("Invalid parameters for ArmEEBurstTimedSkill")

        if params.side not in ("left", "right"):
            return Result.fail(f"side 必须是 'left' 或 'right'，收到: {params.side}")
        if params.frame not in ("world", "local"):
            return Result.fail(f"frame 必须是 'world' 或 'local'，收到: {params.frame}")
        if not params.waypoints:
            return Result.fail("waypoints 不能为空")
        for idx, wp in enumerate(params.waypoints):
            if len(wp) != 6:
                return Result.fail(
                    f"航点需要 6 个值 [x,y,z,yaw,pitch,roll]，第 {idx} 个点有 {len(wp)} 个值")

        self.params = params
        self._done = False
        return Result.ok()

    def on_execute(self) -> Result:
        if self._done:
            return Result.ok("ArmEEBurstTimedSkill already finished")

        p = self.params
        fn_name = f"send_timed_{p.side}_arm_ee_{p.frame}"
        fn = getattr(self.hardware, fn_name, None)
        if fn is None:
            self._done = True
            return Result.fail(f"Hardware does not implement {fn_name}()")

        is_left = (p.side == "left")
        total = len(p.waypoints)
        logger.info("arm_ee_burst_timed: 开始连发 %d 个航点 (%s臂/%s系)", total, p.side, p.frame)

        for i, wp in enumerate(p.waypoints, 1):
            result = fn(pose=list(wp), desire_time=float(p.desire_time))
            if not result.success:
                self._done = True
                return Result.fail(f"第 {i}/{total} 航点下发失败: {result.message}")

            time.sleep(float(p.desire_time) + float(p.settle_time))
            logger.info("arm_ee_burst_timed: [%d/%d] 完成", i, total)

            if p.check_reach:
                status, err_vec, msg = self._check_reach_error(
                    is_left=is_left,
                    linear_tol=float(p.reach_linear_tol),
                    angular_tol=float(p.reach_angular_tol),
                )
                if status == 'out_of_tol':
                    # 已稳态但超差：中止连发
                    self._done = True
                    return Result.fail(f"第 {i}/{total} 航点 {msg}")
                elif status == 'unavailable':
                    # 期望位姿仍在更新/服务不可用：仅告警，继续连发
                    logger.warning("arm_ee_burst_timed: [%d/%d] 静差查询不可用: %s（继续）", i, total, msg)
                else:
                    logger.info("arm_ee_burst_timed: [%d/%d] %s", i, total, msg)

        self._done = True
        return Result.ok(f"连发完成, {total} 个航点")

    def on_is_finished(self) -> bool:
        return self._done
