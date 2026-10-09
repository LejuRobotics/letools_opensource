# -*- coding: utf-8 -*-
"""无初值检测木托盘台面坐标系：`detect`（给初值）+ `refine`（细化）。

`algorithm.py` 与 `refine.py` 都是**纯函数库**（零状态、零 ROS、不 import 框架），
ROS 那一层在
`infrastructure/ros_packages/src/ros_vision/pallet_detection/`。
`payload.py` 是两者共用的纯函数搬运层（`build_payload` / `format_diag` / `format_rejects`）。

⚠️ 从 TF 链查法向的 `normal_from_tf` **不在这里** —— 它要 `import rospy`，按分层
约束只能待在 `infrastructure/`（`pallet_detection/scripts/tf_normal.py`）。

链路：

    pallet_detect.algorithm.detect_pallet_frame   法向先验 + 颜色/深度掩码 -> 旋转矩形搜索
    pallet_detect.refine.refine_pallet_frame      已知尺寸 + 可见边 -> 细化位姿
    pallet_detect.payload.build_payload           位姿 + 诊断 -> PalletDetection 字段

⚠️ 两个模块各自内联了一份 `_backproject` / `_project_px` / `_corners_mm` /
`_fit_plane`（同一份代码的两份拷贝）—— 这样它们能各自单独 import、互不依赖。
**改一处要改两处**，`tests/` 里有逐位比对钉住。

方案说明、踩过的坑、跑法都写在同目录 `README.md` 与两个算法文件顶部的 docstring 里。
"""
from .algorithm import (  # noqa: F401
    TARGET_MM,
    CameraIntrinsics,
    detect_pallet_frame,
    fit_floor_normal,
)
from .payload import (  # noqa: F401
    build_payload,
    format_diag,
    format_rejects,
)
from .refine import refine_pallet_frame  # noqa: F401

__all__ = [
    "CameraIntrinsics",
    "TARGET_MM",
    "build_payload",
    "detect_pallet_frame",
    "fit_floor_normal",
    "format_diag",
    "format_rejects",
    "refine_pallet_frame",
]
