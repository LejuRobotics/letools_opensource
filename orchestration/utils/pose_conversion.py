# -*- coding: utf-8 -*-
"""末端轨迹点姿态表示转换。

轨迹点两种写法：
    quat  : [x, y, z, qx, qy, qz, qw]          位置米、姿态四元数（底层契约）
    euler : [x, y, z, yaw, pitch, roll]        位置米、姿态度
           列序与单位对齐 ArmEeTimedCmdMove / arm_ee_timed_cmd；
           转换方式与 arm_control_mixin.send_both_ee_poses 的
           [x,y,z,yaw,pitch,roll] 路径一致（同一 tf 调用）。
"""

import math
from typing import List

import tf.transformations as tf_trans

ORIENTATION_QUAT = "quat"
ORIENTATION_EULER = "euler"

_EULER_POINT_LEN = 6


def convert_traj_orientation(
    traj: List[List[float]], orientation_format: str
) -> List[List[float]]:
    """把轨迹点姿态统一成四元数，返回新列表（不修改入参）。

    :param traj: 轨迹点列表，每点姿态按 ``orientation_format`` 解释
    :param orientation_format: ``quat`` 原样返回；``euler`` 按
        [x,y,z,yaw,pitch,roll]（米, 度）转换
    :raises ValueError: 格式不支持，或轨迹结构、点数、数值非法
    """
    if orientation_format not in (ORIENTATION_QUAT, ORIENTATION_EULER):
        raise ValueError(
            f"orientation_format 必须为 '{ORIENTATION_QUAT}' 或 "
            f"'{ORIENTATION_EULER}'，实际为 {orientation_format!r}"
        )

    if not isinstance(traj, (list, tuple)) or not all(
        isinstance(point, (list, tuple)) for point in traj
    ):
        raise ValueError("轨迹点列表格式非法：需为 [[...], [...]] 形式")

    if orientation_format == ORIENTATION_QUAT:
        return [list(point) for point in traj]
    return [_euler_point_to_quat(point) for point in traj]


def _euler_point_to_quat(point) -> List[float]:
    """[x, y, z, yaw, pitch, roll]（米, 度）→ [x, y, z, qx, qy, qz, qw]。"""
    if len(point) != _EULER_POINT_LEN:
        raise ValueError(
            f"orientation_format='euler' 时每点需要 {_EULER_POINT_LEN} 个值 "
            f"[x,y,z,yaw,pitch,roll]，实际有 {len(point)} 个值"
        )

    try:
        x, y, z, yaw, pitch, roll = (float(v) for v in point)
    except (TypeError, ValueError):
        raise ValueError(f"轨迹点含无法转为 float 的值: {point}")

    values = (x, y, z, yaw, pitch, roll)
    if not all(math.isfinite(v) for v in values):
        raise ValueError(f"轨迹点含 NaN/Inf: {point}")

    # quaternion_from_euler 的默认轴序 sxyz 等价于 R = Rz(yaw)·Ry(pitch)·Rx(roll)
    qx, qy, qz, qw = tf_trans.quaternion_from_euler(
        math.radians(roll), math.radians(pitch), math.radians(yaw)
    )
    return [x, y, z, float(qx), float(qy), float(qz), float(qw)]
