# -*- coding: utf-8 -*-
"""托盘检测：OBB 初值（`detect`）+ 用户规则开关与深度细化（`refine`）。

`detect.py` / `refine_*.py` / `solve_obb.py` / `geom.py` 都是**纯函数库**
（零状态、零 ROS、不 import 框架），ROS 那一层在
`infrastructure/ros_packages/src/ros_vision/pallet_detection/`；
`payload.py` 是共用纯函数搬运层（`build_payload` / `format_diag` / `format_rejects`）。

⚠️ 从 TF 链查法向的 `normal_from_tf` **不在这里** —— 它要 `import rospy`，按分层
约束只能待在 `infrastructure/`（`pallet_detection/scripts/tf_normal.py`）。

链路：

    pallet_detect.normals.fit_floor_normal    TF 先验 + 当前帧深度自检 -> 实际法向
    pallet_detect.detect.detect_pallet_frame  YOLO-OBB 推理 -> frame_from_obb -> frame
    pallet_detect.detect_pose.detect_pallet_frame_pose  YOLO-pose -> 四边 -> solve_edges -> frame
    pallet_detect.refine_depth.refine_pallet_frame  用户规则开关 -> 脊线/角点/深度三级细化
    pallet_detect.payload.build_payload       位姿 + 诊断 -> PalletDetection 字段

方案说明、踩过的坑、跑法都写在同目录 `README.md` 与各算法文件顶部的 docstring 里。
"""
from .detect import (  # noqa: F401
    DEFAULT_ENGINE_PATH,
    DEFAULT_MODEL_PATH,
    LONG_SIDE_PARALLEL,
    TARGET_MM,
    detect_pallet_frame,
)
from .detect_pose import (  # noqa: F401
    DEFAULT_POSE_ENGINE_PATH,
    DEFAULT_POSE_MODEL_PATH,
    detect_pallet_frame_pose,
)
from .dsearch import D_dev, D_from_frame  # noqa: F401
from .geom import CameraIntrinsics
from .normals import (  # noqa: F401
    fit_floor_normal,
    fit_floor_normal_checked,
    normal_is_plausible,
)
from .payload import (  # noqa: F401
    build_payload,
    format_diag,
    format_rejects,
)
from .refine_depth import refine_pallet_frame  # noqa: F401

__all__ = [
    "CameraIntrinsics",
    "D_dev",
    "D_from_frame",
    "DEFAULT_ENGINE_PATH",
    "DEFAULT_MODEL_PATH",
    "DEFAULT_POSE_ENGINE_PATH",
    "DEFAULT_POSE_MODEL_PATH",
    "LONG_SIDE_PARALLEL",
    "TARGET_MM",
    "build_payload",
    "detect_pallet_frame",
    "detect_pallet_frame_pose",
    "fit_floor_normal",
    "fit_floor_normal_checked",
    "normal_is_plausible",
    "format_diag",
    "format_rejects",
    "refine_pallet_frame",
]
