# -*- coding: utf-8 -*-
"""场景专用的非阻塞轨迹提交包装。"""

from __future__ import annotations

from concurrent.futures import Future, ThreadPoolExecutor
from typing import Sequence


_EXECUTOR = ThreadPoolExecutor(
    max_workers=1,
    thread_name_prefix="wheel-arm-single-tag",
)




def _get_source_sdk_compat(hardware):
    from adapters.hardware.leju_wheeled.source_sdk_compat import (
        get_source_sdk_compat,
    )

    return get_source_sdk_compat(hardware)


def _send_source_joint_trajectory(hardware, trajectory, total_time):
    """通过 Adapter 的源 SDK facade 调用 ThroughFullBodyMpc。"""
    return _get_source_sdk_compat(hardware).move_joint_traj_auto(
        joint_traj=trajectory,
        total_time=total_time,
        back_default=False,
        direct_to_wbc=False,
    )

def submit_arm_joint_trajectory(
    hardware,
    trajectory: Sequence[Sequence[float]],
    total_time: float,
) -> Future:
    """在后台按源脚本 MPC flow 提交轨迹，返回可轮询的 Future。"""
    return _EXECUTOR.submit(
        _send_source_joint_trajectory,
        hardware,
        trajectory,
        total_time,
    )
