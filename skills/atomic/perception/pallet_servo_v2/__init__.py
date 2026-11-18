# -*- coding: utf-8 -*-
"""托盘伺服误差：由托盘位姿与箱子观测算图像空间的三个误差量。

`algorithm.py` 是纯函数库（零框架依赖），`skill.py` 是接进技能层的封装。
几何定义、单位约定、四个坑都写在 `algorithm.py` 顶部的模块 docstring 里。

这里只转出纯算法部分：`skill.py` 会 import `orchestration.utils`，把它一起
转出会让 `import pallet_servo` 顺带拉起编排层，测试时没必要。
"""
from .algorithm import (  # noqa: F401
    MIN_EDGE_PX,
    REF_EDGE_SPECS,
    THETA_WARN_RAD,
    BoxObservation,
    EdgeOffset,
    Reject,
    ServoError,
    box_corners,
    box_edges,
    edge_offset,
    fold_line_angle,
    inward_normal,
    line_angle,
    parse_edge_spec,
    parse_ref_edges,
    project_pallet_points,
    ref_edge_points_mm,
    ref_edge_px,
    servo_error,
)

__all__ = [
    "MIN_EDGE_PX",
    "REF_EDGE_SPECS",
    "THETA_WARN_RAD",
    "BoxObservation",
    "EdgeOffset",
    "Reject",
    "ServoError",
    "box_corners",
    "box_edges",
    "edge_offset",
    "fold_line_angle",
    "inward_normal",
    "line_angle",
    "parse_edge_spec",
    "parse_ref_edges",
    "project_pallet_points",
    "ref_edge_points_mm",
    "ref_edge_px",
    "servo_error",
]
