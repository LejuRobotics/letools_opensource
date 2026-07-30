# -*- coding: utf-8 -*-
"""Atomic skill: arm_ee_offline_traj.

轮臂末端独立控制 - 离线整体时间最优轨迹（一次性提交整条带时间戳轨迹）。
躯干/底盘保持不动（初始化由脚手架/编排层负责）。

对齐源脚本 cmd_arm_ee_offline_traj_test.py：
  enable(True) → set_offline_trajectory → sleep(total_time) → enable(False)

与在线连发（arm_ee_burst_timed）的区别：
  - 连发：逐点发在线服务，每点独立规划，MPC 在线平滑（有失真）；
  - 离线：一次提交整条带时间戳轨迹，底层 Ruckig 预规划整体时间最优，
          二阶连续三阶可导，适合密集航点 / 示教回放 / 涂胶。

离线专用约定（与 TimedCmd 路径不同！）：
  - planner_index: 0=左臂, 1=右臂（不支持 2=躯干，本需求末端独立控制）
  - frame: 0=世界系, 1=局部系
  - times 为绝对时间（秒），第一帧必须为 0，严格递增
  - cmd_vec 姿态字段直接收弧度（离线服务不经 TimedCmd 的 _to_rad 转换）
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

# 默认轨迹: [[x, y, z, yaw, pitch, roll], ...]（位置米，姿态弧度！）
_DEFAULT_TRAJ = [
    [0.3, 0.25, 0.5, 0.0, 0.0, 0.0],
    [0.5, 0.25, 0.5, 0.0, 0.0, 0.0],
    [0.5, 0.15, 0.6, 0.0, 0.0, 0.0],
    [0.3, 0.15, 0.6, 0.0, 0.0, 0.0],
]
_DEFAULT_TIMES = [0.0, 1.0, 2.0, 3.0]


@dataclass
class ArmEEOfflineTrajParams(SkillParams):
    """离线时间最优轨迹参数。

    约定: traj [[x,y,z,yaw,pitch,roll], ...]，位置米；姿态弧度（离线服务直接收弧度）。
    times 为绝对时间（秒），第一帧必须 0，严格递增，长度与 traj 一致。
    """

    skill_name: str = "arm_ee_offline_traj"
    side: str = "left"          # 'left' / 'right' → planner_index 0/1
    frame: str = "world"        # 'world' / 'local' → frame 0/1
    traj: List[List[float]] = field(default_factory=lambda: [list(p) for p in _DEFAULT_TRAJ])
    times: List[float] = field(default_factory=lambda: list(_DEFAULT_TIMES))
    total_time: float = 0.0     # 若 0 则取 times[-1]
    post_settle: float = 0.5    # enable(False) 前的额外等待（秒）
    timeout: float = 600.0


@define_manifest(
    label="离线时间最优末端轨迹",
    category=["motion", "arm"],
    tree_type="studio_smoke",
    description="enable→set→sleep→enable(False) 全流程，planner 0左/1右，姿态弧度",
    params=[
        {"name": "side", "type": "string", "default": "left", "description": "手臂侧: 'left' / 'right'"},
        {"name": "frame", "type": "string", "default": "world", "description": "坐标系: 'world' / 'local'"},
        {"name": "traj", "type": "json", "default": "",
         "description": "轨迹 [[x,y,z,yaw,pitch,roll], ...]（米, 弧度！）"},
        {"name": "times", "type": "json", "default": "",
         "description": "绝对时间 [0,1,2,...]，第一帧必须 0，严格递增"},
        {"name": "total_time", "type": "float", "default": "0", "description": "总时间（秒），0=取 times[-1]"},
        {"name": "post_settle", "type": "float", "default": "0.5", "description": "关闭使能前额外等待（秒）"},
    ],
    inputs=[],
    outputs=[],
)
class ArmEEOfflineTrajSkill(SkillBase):
    """离线整体时间最优轨迹（enable→set→sleep→enable(False)）。

    planner_index 映射（离线专用，与 TimedCmd 不同！）:
      0=左臂, 1=右臂；frame: 0=世界系, 1=局部系。
    不负责初始化（focus_ee=False、set_control_mode、复位等），由上层编排/脚手架负责。
    """

    def __init__(self, hardware: IHardware):
        super().__init__(name="arm_ee_offline_traj")
        self.hardware = hardware
        self.params: Optional[ArmEEOfflineTrajParams] = None
        self._done = False

    def on_initialize(self, params: ArmEEOfflineTrajParams) -> Result:
        if not isinstance(params, ArmEEOfflineTrajParams):
            return Result.fail("Invalid parameters for ArmEEOfflineTrajSkill")

        if params.side not in ("left", "right"):
            return Result.fail(f"side 必须是 'left' 或 'right'，收到: {params.side}")
        if params.frame not in ("world", "local"):
            return Result.fail(f"frame 必须是 'world' 或 'local'，收到: {params.frame}")
        if not params.traj:
            return Result.fail("traj 不能为空")
        if len(params.traj) != len(params.times):
            return Result.fail(
                f"traj 长度({len(params.traj)}) 与 times 长度({len(params.times)}) 不一致")
        for idx, wp in enumerate(params.traj):
            if len(wp) != 6:
                return Result.fail(
                    f"轨迹点需要 6 个值 [x,y,z,yaw,pitch,roll]，第 {idx} 个点有 {len(wp)} 个值")
        # 离线时间校验：第一帧必须为 0、严格递增
        if abs(params.times[0]) > 1e-6:
            return Result.fail(f"第一帧时间必须为 0，收到 {params.times[0]}")
        for j in range(1, len(params.times)):
            if params.times[j] <= params.times[j - 1]:
                return Result.fail(
                    f"时间未严格递增：第{j-1}点 t={params.times[j-1]}，第{j}点 t={params.times[j]}")

        self.params = params
        self._done = False
        return Result.ok()

    def on_execute(self) -> Result:
        if self._done:
            return Result.ok("ArmEEOfflineTrajSkill already finished")

        p = self.params
        set_fn = getattr(self.hardware, "set_offline_trajectory", None)
        enable_fn = getattr(self.hardware, "enable_offline_trajectory", None)
        if set_fn is None or enable_fn is None:
            self._done = True
            return Result.fail(
                "Hardware does not implement set_offline_trajectory()/enable_offline_trajectory()")

        planner_index = 0 if p.side == "left" else 1
        frame_int = 0 if p.frame == "world" else 1
        trajectories = [{
            'planner_index': planner_index,
            'frame': frame_int,
            'timed_traj': [
                {'desire_time': float(t), 'cmd_vec': list(wp)}
                for t, wp in zip(p.times, p.traj)
            ],
        }]
        total = float(p.total_time) if p.total_time else float(p.times[-1])

        logger.info(
            "arm_ee_offline_traj: %s臂/%s系 planner=%d frame=%d %d点 总时长 %.2fs",
            p.side, p.frame, planner_index, frame_int, len(p.traj), total)

        # 1. 先使能离线轨迹
        r1 = enable_fn(True)
        if not r1.success:
            self._done = True
            return Result.fail(f"离线轨迹使能失败: {r1.message}")

        # 2-4. 提交 → 等待 → 关闭使能（try/finally 确保任何异常路径都恢复在线控制）
        try:
            # 2. 一次性提交整条轨迹
            r2 = set_fn(trajectories)
            if not r2.success:
                self._done = True
                return Result.fail(f"离线轨迹提交失败: {r2.message}")

            # 3. 等待整条轨迹执行完成
            time.sleep(total + float(p.post_settle))
        finally:
            # 4. 无论成功/失败/中断都关闭离线轨迹使能，恢复正常在线控制
            r4 = enable_fn(False)

        self._done = True
        if not r4.success:
            return Result.fail(f"离线轨迹关闭使能失败: {r4.message}")
        return Result.ok(f"离线轨迹执行完成, 总时长 {total:.2f}s")

    def on_is_finished(self) -> bool:
        return self._done
