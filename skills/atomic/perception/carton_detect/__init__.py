# -*- coding: utf-8 -*-
"""纸箱顶面四角（OBB）：图像 → ONNX(OBB) → 四角 + 选箱。

`obb_decoder.py` 是**纯函数库**（零状态、零 ROS、不 import ultralytics / torch）；
ROS 那一层在 `infrastructure/ros_packages/src/ros_vision/carton_obb/`
（收图像 → 调这里 → 发 `/box/detection`）。

⚠️ 它**不 import rospy / sensor_msgs**，也**不读模型文件** —— 推理在 ROS 壳里做
（onnxruntime 跑包内的 `.onnx`）。这是分层硬约束：`skills/` 下的东西要能脱离 ROS
单独单测，见 `pallet_detect/__init__.py` 里同一条说明。

⚠️ 托盘那条链（`pallet_detect/detect.py`）的 OBB 前/后处理是**照抄**这里的
（那边文件里写着「照抄 carton_obb/obb_decoder.py」）。两边现在靠**手工**保持一致；
把两份并成一份是**单独一轮**的事 —— 它会给已经验到逐位一致的托盘链引入数值风险，
必须单独用两边的回归网验证。**2026-10-15 这一轮只做搬迁，不碰数值。**

链路：

    carton_detect.obb_decoder.to_input_tensor   letterbox 到 imgsz
    carton_detect.obb_decoder.decode_obb        ONNX 输出 -> 旋转框（xc,yc,w,h,θ,conf）
    carton_detect.obb_decoder.nms_rotated       旋转框去重（IoU + IoMin，导出图**没有** NMS）
    carton_detect.obb_decoder.score_candidates  选箱打分（离下中最近 + 面积 + 深度）
    carton_detect.obb_decoder.pick_best         取分最高的那个

方案说明与实测依据在各文件顶部 docstring 与同目录 `README.md` 里。
"""
from .obb_decoder import (  # noqa: F401
    DEFAULT_D_FAR_PX,
    DEFAULT_D_NEAR_PX,
    DEFAULT_DEPTH_PATCH_PX,
    DEFAULT_MODEL_PATH,
    DEFAULT_NMS_IOMIN,
    DEFAULT_NMS_IOU,
    DEFAULT_REFINE_BOTTOM_BAND_PX,
    DEFAULT_REFINE_BOTTOM_JUMP_MM,
    DEFAULT_REFINE_BOTTOM_MIN_SHRINK_PX,
    DEFAULT_REFINE_BOTTOM_START_PX,
    DEFAULT_REFINE_BOTTOM_STEP,
    DEFAULT_SCORE_W_AREA,
    DEFAULT_SCORE_W_DCENTER,
    DEFAULT_SCORE_W_DEPTH,
    DEFAULT_Z_FAR_MM,
    DEFAULT_Z_NEAR_MM,
    angle_deg_from_corners,
    candidate_score,
    d_center_px,
    d_center_score,
    decode_obb,
    depth_score,
    dets_from_rotated_rects,
    letterbox,
    nms_rotated,
    order_corners_uv,
    pick_best,
    quad_from_xywhr,
    refine_bottom_by_depth,
    sample_depth_mm,
    score_candidates,
    to_input_tensor,
)

__all__ = [
    "DEFAULT_D_FAR_PX",
    "DEFAULT_D_NEAR_PX",
    "DEFAULT_DEPTH_PATCH_PX",
    "DEFAULT_MODEL_PATH",
    "DEFAULT_NMS_IOMIN",
    "DEFAULT_NMS_IOU",
    "DEFAULT_REFINE_BOTTOM_BAND_PX",
    "DEFAULT_REFINE_BOTTOM_JUMP_MM",
    "DEFAULT_REFINE_BOTTOM_MIN_SHRINK_PX",
    "DEFAULT_REFINE_BOTTOM_START_PX",
    "DEFAULT_REFINE_BOTTOM_STEP",
    "DEFAULT_SCORE_W_AREA",
    "DEFAULT_SCORE_W_DCENTER",
    "DEFAULT_SCORE_W_DEPTH",
    "DEFAULT_Z_FAR_MM",
    "DEFAULT_Z_NEAR_MM",
    "angle_deg_from_corners",
    "candidate_score",
    "d_center_px",
    "d_center_score",
    "decode_obb",
    "depth_score",
    "dets_from_rotated_rects",
    "letterbox",
    "nms_rotated",
    "order_corners_uv",
    "pick_best",
    "quad_from_xywhr",
    "refine_bottom_by_depth",
    "sample_depth_mm",
    "score_candidates",
    "to_input_tensor",
]
