# -*- coding: utf-8 -*-
"""单 Tag 场景的 IK 返回值规范化。"""

from __future__ import annotations

import math
from typing import Any, List, Mapping


FULL_BODY_DOF = 18
ARM_DOF = 7
LEFT_ARM_SLICE = slice(4, 11)
RIGHT_ARM_SLICE = slice(11, 18)


def normalize_ik_result(
    result: Any,
    is_left: bool,
    *,
    allow_legacy_seven: bool = False,
) -> List[float] | None:
    """严格提取 IK 精确解，返回指定侧的 7 个弧度关节角。

    服务结果必须同时满足外层成功、数据为映射、数据中的 ``success`` 为真，
    并提供 ``q_best``。真实服务的 18 维结果按腿部 4 维、左臂 7 维、右臂
    7 维排列；7 维结果仅在调用方显式开启兼容开关时接受。
    """
    if not getattr(result, "success", False):
        return None
    data = getattr(result, "data", None)
    if not isinstance(data, Mapping) or data.get("success") is not True:
        return None

    joints = data.get("q_best")
    if not isinstance(joints, (list, tuple)):
        return None

    if len(joints) == FULL_BODY_DOF:
        arm_joints = joints[LEFT_ARM_SLICE if is_left else RIGHT_ARM_SLICE]
    elif len(joints) == ARM_DOF and allow_legacy_seven:
        arm_joints = joints
    else:
        return None

    try:
        values = [float(value) for value in arm_joints]
    except (TypeError, ValueError):
        return None
    if len(values) != ARM_DOF or not all(math.isfinite(value) for value in values):
        return None
    return values
