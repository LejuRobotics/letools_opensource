#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""机器人运动中使用的轻量几何工具。

本模块只依赖 Python 标准库，导入时不需要 ROS。四元数统一采用
``[x, y, z, w]`` 顺序，角度统一使用弧度。
"""

import math


_TWO_PI = 2.0 * math.pi


def _finite_float(value, name):
    """将输入转换为有限浮点数，并给出便于定位的错误信息。"""

    number = float(value)
    if not math.isfinite(number):
        raise ValueError("%s 必须是有限数值" % name)
    return number


def _vector(values, size, name):
    """读取定长数值序列；同时兼容 list、tuple、numpy 数组等对象。"""

    try:
        result = [_finite_float(values[index], "%s[%d]" % (name, index)) for index in range(size)]
    except (IndexError, KeyError, TypeError, AttributeError) as exc:
        raise ValueError("%s 必须至少包含 %d 个数值" % (name, size)) from exc
    return result


def normalize_angle(angle):
    """把弧度角归一化到 ``[-pi, pi]``。

    与旧实现保持边界语义：正方向的奇数倍 ``pi`` 返回 ``pi``，负方向
    的奇数倍 ``pi`` 返回 ``-pi``。使用取模避免超大角度导致长循环。
    """

    angle = _finite_float(angle, "angle")
    if -math.pi <= angle <= math.pi:
        return angle
    wrapped = (angle + math.pi) % _TWO_PI - math.pi
    if wrapped == -math.pi and angle > 0.0:
        return math.pi
    return wrapped


def quat_xyzw(quaternion):
    """把四元数对象或序列转换为 ``[x, y, z, w]`` 浮点列表。"""

    if all(hasattr(quaternion, attr) for attr in ("x", "y", "z", "w")):
        return [
            _finite_float(quaternion.x, "quaternion.x"),
            _finite_float(quaternion.y, "quaternion.y"),
            _finite_float(quaternion.z, "quaternion.z"),
            _finite_float(quaternion.w, "quaternion.w"),
        ]
    return _vector(quaternion, 4, "quaternion")


def yaw_from_quat(quaternion):
    """从 ``[x, y, z, w]`` 四元数提取绕 Z 轴的 yaw。"""

    qx, qy, qz, qw = quat_xyzw(quaternion)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def rotate_vector_by_quat(vector, quaternion):
    """使用四元数旋转三维向量。

    调用方应传入单位四元数；此处不自动归一化，以保留底层 TF 数据的原始
    语义，并避免在每个闭环周期引入额外缩放判断。
    """

    qx, qy, qz, qw = quat_xyzw(quaternion)
    vx, vy, vz = _vector(vector, 3, "vector")
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return [
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    ]


def quaternion_multiply(first, second):
    """计算两个 ``[x, y, z, w]`` 四元数的 Hamilton 乘积。"""

    x1, y1, z1, w1 = quat_xyzw(first)
    x2, y2, z2, w2 = quat_xyzw(second)
    return [
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ]


def scan_yaws(scan_range, scan_step):
    """生成 ``0,+step,-step,...`` 顺序的对称扫描角列表。"""

    scan_range = abs(_finite_float(scan_range, "scan_range"))
    scan_step = abs(_finite_float(scan_step, "scan_step"))
    if scan_step == 0.0:
        scan_step = scan_range or 1.0

    values = [0.0]
    step = scan_step
    while step <= scan_range + 1e-6:
        values.extend([step, -step])
        step += scan_step
    return values


def scan_head_points(yaw_range, yaw_step, pitch_center, pitch_range, pitch_step):
    """按俯仰层生成头部扫描点 ``(yaw, pitch)``。"""

    yaws = scan_yaws(yaw_range, yaw_step)
    pitch_offsets = scan_yaws(pitch_range, pitch_step)
    center = _finite_float(pitch_center, "pitch_center")
    return [
        (yaw, center + pitch_offset)
        for pitch_offset in pitch_offsets
        for yaw in yaws
    ]


__all__ = [
    "normalize_angle",
    "quat_xyzw",
    "quaternion_multiply",
    "rotate_vector_by_quat",
    "scan_head_points",
    "scan_yaws",
    "yaw_from_quat",
]
