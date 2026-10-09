# -*- coding: utf-8 -*-
"""箱子顶面四角：从 YOLO 轴对齐框恢复被丢掉的旋转角。

`algorithm.py` 是纯函数库（零框架依赖、零 ROS），`window.py` 是包在它外面的
**有状态**滑动时间窗（也不 import ROS）。ROS 那一层在
`infrastructure/ros_packages/src/ros_vision/detection_industrial_yolo/box_detection/`。

方案说明、九个必须知道的坑、以及运行/测试方式都写在 `algorithm.py` 顶部的
模块 docstring 与同目录 `README.md` 里。

这里只转出纯算法部分 —— 与 `pallet_pose` / `pallet_servo` 的约定一致。
"""
from .algorithm import (  # noqa: F401
    DEFAULT_TARGET_MM,
    CameraIntrinsics,
    fit_box_frame,
    frame_long_axis,
    load_frame,
    order_corners_uv,
    roi_from_box,
    segment_top_plane,
)
from .window import BoxFrameWindow, build_payload, hint_from_instances  # noqa: F401

__all__ = [
    "DEFAULT_TARGET_MM",
    "BoxFrameWindow",
    "CameraIntrinsics",
    "build_payload",
    "fit_box_frame",
    "frame_long_axis",
    "hint_from_instances",
    "load_frame",
    "order_corners_uv",
    "roi_from_box",
    "segment_top_plane",
]
