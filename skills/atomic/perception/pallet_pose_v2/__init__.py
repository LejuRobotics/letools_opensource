# -*- coding: utf-8 -*-
"""托盘位姿：AprilTag → 木托盘的标定与反识别。

`algorithm.py` 是纯函数库（零框架依赖），`skill.py` 是接进技能层的封装。
方案说明、三个必须知道的坑、以及运行/测试方式都写在 `algorithm.py` 顶部的
模块 docstring 里。

这里只转出纯算法部分：`skill.py` 会 import `orchestration.utils`，把它一起
转出会让 `import pallet_pose` 顺带拉起编排层，测试时没必要。
"""
from .algorithm import (  # noqa: F401
    CalibrationResult,
    PalletFrame,
    TagObs,
    calibrate_pallet_tag,
    fuse_pallet_poses,
    order_quad,
    pallet_frame_from_clicks,
    pallet_pose_from_tag,
    pallet_pose_from_tags,
    smooth_series,
)

__all__ = [
    "CalibrationResult",
    "PalletFrame",
    "TagObs",
    "calibrate_pallet_tag",
    "fuse_pallet_poses",
    "order_quad",
    "pallet_frame_from_clicks",
    "pallet_pose_from_tag",
    "pallet_pose_from_tags",
    "smooth_series",
]
