# -*- coding: utf-8 -*-
"""双臂关节轨迹执行技能 (Refactored SDK)."""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, List, Optional

from adapters.hardware.leju_wheeled.source_sdk_compat import ImmediateHandle
from core.common.logger import get_logger
from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.interfaces.i_hardware import IHardware
from orchestration.scenarios.wheel_arm_single_tag_pick_v1.utils.async_execution import (
    submit_arm_joint_trajectory,
)
from orchestration.scenarios.wheel_arm_single_tag_pick_v1.utils.trajectory import (
    merge_bimanual_trajectory,
    validate_bimanual_14d_trajectory,
)
from orchestration.utils.manifest_decorators import define_manifest
from skills.base.skill_base import SkillBase

logger = get_logger(__name__)


@dataclass
class BimanualJointTrajectoryParams(SkillParams):
    """双臂关节轨迹执行参数"""

    skill_name: str = "bimanual_joint_trajectory"
    left_arm_joint_traj: Optional[List[List[float]]] = None
    right_arm_joint_traj: Optional[List[List[float]]] = None
    bimanual_traj_14d: Optional[List[List[float]]] = None
    total_time: float = 3.0
    timeout: float = 30.0


@define_manifest(
    label="双臂14关节轨迹执行",
    category=["motion", "arm"],
    tree_type="studio_smoke",
    description="合并左右臂7-DOF轨迹并异步下发底层硬件14关节同步执行",
    params=[
        {"name": "total_time", "type": "float", "default": "3.0", "description": "执行总时间（秒）"},
        {"name": "timeout", "type": "float", "default": "30.0", "description": "超时时间（秒）"},
    ],
    inputs=[],
    outputs=[],
)
class BimanualJointTrajectorySkill(SkillBase):
    """
    原子技能：合并并执行双臂 14 关节同步轨迹。
    支持传入 (left_traj, right_traj) 或预先合并好的 14 维轨迹。
    """

    def __init__(self, hardware: IHardware):
        super().__init__(name="bimanual_joint_trajectory")
        self.hardware = hardware
        self.params: Optional[BimanualJointTrajectoryParams] = None
        self._trajectory_14d: Optional[List[List[float]]] = None
        self._handle = None
        self._started = False
        self._is_finished = False

    def on_initialize(self, params: BimanualJointTrajectoryParams) -> Result:
        if not isinstance(params, BimanualJointTrajectoryParams):
            return Result.fail("Invalid parameters for BimanualJointTrajectorySkill")

        self.params = params
        self._handle = None
        self._started = False
        self._is_finished = False

        if params.bimanual_traj_14d is not None:
            try:
                validate_bimanual_14d_trajectory(params.bimanual_traj_14d)
                self._trajectory_14d = params.bimanual_traj_14d
            except Exception as exc:
                return Result.fail(f"Invalid 14D trajectory: {exc}")
        elif params.left_arm_joint_traj is not None or params.right_arm_joint_traj is not None:
            try:
                self._trajectory_14d = merge_bimanual_trajectory(
                    params.left_arm_joint_traj, params.right_arm_joint_traj
                )
            except Exception as exc:
                return Result.fail(f"Trajectory merge failed: {exc}")
        else:
            return Result.fail("No trajectory data provided")

        logger.info(
            f"BimanualJointTrajectorySkill initialized with {len(self._trajectory_14d)} frames, "
            f"total_time={params.total_time}s, timeout={params.timeout}s"
        )
        return Result.ok()

    def on_execute(self) -> Result:
        # 1. 首次触发执行，提交硬件异步任务
        if not self._started:
            try:
                res = submit_arm_joint_trajectory(
                    self.hardware,
                    self._trajectory_14d,
                    total_time=self.params.total_time,
                )
            except Exception as exc:
                logger.exception(f"Failed to submit bimanual trajectory: {exc}")
                return Result.fail(f"Submission failed: {exc}")

            self._handle = res if self._is_handle(res) else ImmediateHandle(res)
            self._started = True

        # 2. 轮询句柄状态
        if not self._handle.done():
            return Result.ok("Executing")

        # 3. 句柄已结束，检查结果
        try:
            error = self._handle.exception()
            if error is not None:
                logger.error(f"Trajectory execution encountered exception: {error}")
                return Result.fail(str(error))
            res = self._handle.result()
            if not getattr(res, "success", False):
                msg = getattr(res, "message", "trajectory execution failed")
                logger.error(f"Trajectory execution failed: {msg}")
                return Result.fail(msg)
        except Exception as exc:
            logger.exception(f"Trajectory handle evaluation failed: {exc}")
            return Result.fail(str(exc))

        self._is_finished = True
        logger.info("Bimanual joint trajectory execution succeeded")
        return Result.ok("Trajectory executed successfully")

    def on_cancel(self) -> Result:
        if self._handle is not None and callable(getattr(self._handle, "cancel", None)):
            try:
                self._handle.cancel()
                logger.info("Trajectory execution cancelled")
            except Exception as exc:
                logger.warning(f"Error cancelling trajectory execution: {exc}")
        self._is_finished = True
        return Result.ok("Cancelled")

    def on_is_finished(self) -> bool:
        return self._is_finished

    @staticmethod
    def _is_handle(value: Any) -> bool:
        return all(callable(getattr(value, name, None)) for name in ("done", "result", "exception"))
