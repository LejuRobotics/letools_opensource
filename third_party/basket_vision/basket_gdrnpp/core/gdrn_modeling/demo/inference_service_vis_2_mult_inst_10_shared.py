#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Shared ROS service node for one-shot YOLO + GDRN++ 6D pose inference.

This node loads one shared YOLO model and one shared GDRN++ predictor,
and exposes two services at the same time:
  - /infer_basket_pose
  - /infer_top_basket_ids
"""

import json
import math
import os
import os.path as osp
import shutil
import threading
import time
import traceback
from datetime import datetime

import cv2
import numpy as np

import rospy
import torch
import tf2_ros
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, TransformStamped
from PIL import Image as PILImage
from sensor_msgs.msg import CameraInfo, Image
from tf.transformations import quaternion_from_matrix, quaternion_matrix
from ultralytics import YOLO
from scipy.spatial.transform import Rotation as R

from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Header as StdHeader

# basket_5 uses free-form placement — no stack rules needed

from basket_vision_msgs.srv import InferBasketPose, InferBasketPoseResponse
from predictor_gdrn import GdrnPredictor

COLOR_RESET = "\033[0m"
COLOR_CYAN = "\033[96m"
COLOR_GREEN = "\033[92m"
COLOR_YELLOW = "\033[93m"

BASE_LINK_FRAME = "base_link"


def wrap_to_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def fix_yaw_with_offset_and_symmetry(yaw_rad, offset_deg=90.0, max_abs_deg=90.0):
    offset_rad = math.radians(offset_deg)
    max_abs_rad = math.radians(max_abs_deg)

    yaw = yaw_rad + offset_rad
    yaw = wrap_to_pi(yaw)

    if yaw > max_abs_rad:
        yaw -= math.pi
    elif yaw < -max_abs_rad:
        yaw += math.pi

    return wrap_to_pi(yaw)


def ensure_numpy(arr, want_dtype=None, want_channels=None, name="arr"):
    if arr is None:
        raise ValueError(f"{name} is None")
    if not isinstance(arr, np.ndarray):
        arr = np.asarray(arr)
    if want_dtype is not None and arr.dtype != want_dtype:
        arr = arr.astype(want_dtype, copy=False)
    if want_channels is not None:
        if arr.ndim == 2 and want_channels == 3:
            arr = cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
        elif arr.ndim == 3 and arr.shape[2] == 4 and want_channels == 3:
            arr = cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
    if hasattr(arr, "flags") and not arr.flags["C_CONTIGUOUS"]:
        arr = np.ascontiguousarray(arr)
    return arr


def colorize_depth(depth):
    if depth.dtype == np.uint16:
        depth_m = depth.astype(np.float32) / 1000.0
    else:
        depth_m = depth.astype(np.float32)
    dmin, dmax = 0.2, 3.0
    depth_m = np.clip(depth_m, dmin, dmax)
    norm = (depth_m - dmin) / (dmax - dmin)
    vis = (norm * 255).astype(np.uint8)
    vis = cv2.applyColorMap(vis, cv2.COLORMAP_JET)
    vis[depth_m <= dmin + 1e-6] = 0
    return vis


def draw_axes_on_image(img_bgr, T_cam_obj, K, D, axis_len_m=0.20):
    img = img_bgr.copy()
    h, w = img.shape[:2]

    if T_cam_obj is None or np.asarray(T_cam_obj).shape != (4, 4):
        rospy.logwarn('[VIS] skip axis drawing: invalid T_cam_obj shape')
        return img
    if not np.all(np.isfinite(T_cam_obj)):
        rospy.logwarn('[VIS] skip axis drawing: T_cam_obj contains non-finite values')
        return img
    if float(T_cam_obj[2, 3]) <= 0.05:
        rospy.logwarn(f'[VIS] skip axis drawing: object z in camera is too small ({float(T_cam_obj[2, 3]):.4f} m)')
        return img

    axis_3d = np.float32([
        [0, 0, 0],
        [axis_len_m, 0, 0],
        [0, axis_len_m, 0],
        [0, 0, axis_len_m],
    ])
    R = T_cam_obj[:3, :3].astype(np.float32)
    t = T_cam_obj[:3, 3].astype(np.float32).reshape(3, 1)
    rvec, _ = cv2.Rodrigues(R)
    pts2d, _ = cv2.projectPoints(axis_3d, rvec, t, K, D)
    pts2d = pts2d.reshape(-1, 2)

    def _cv_point(pt):
        if not np.all(np.isfinite(pt)):
            return None
        x = float(pt[0])
        y = float(pt[1])
        if abs(x) > 100000 or abs(y) > 100000:
            return None
        # Allow a modest off-image margin so partially visible axes can still be drawn.
        margin = max(w, h) * 2
        if x < -margin or x > w + margin or y < -margin or y > h + margin:
            return None
        return (int(round(x)), int(round(y)))

    origin = _cv_point(pts2d[0])
    x_axis = _cv_point(pts2d[1])
    y_axis = _cv_point(pts2d[2])
    z_axis = _cv_point(pts2d[3])
    if origin is None:
        rospy.logwarn(f'[VIS] skip axis drawing: projected origin invalid, pts2d={pts2d.tolist()}')
        return img

    def _draw_arrow(img, tip, tail, color, thickness=3, tip_length=0.35):
        """Draw a line with an arrow head at the tip, AprilTag/QR-code style."""
        cv2.line(img, tail, tip, color, thickness, cv2.LINE_AA)
        # Arrow head: compute direction and orthogonal
        dx = float(tip[0] - tail[0])
        dy = float(tip[1] - tail[1])
        length = math.hypot(dx, dy)
        if length < 1.0:
            return
        ux, uy = dx / length, dy / length
        head_len = min(length * tip_length, 18.0)
        head_w = head_len * 0.45
        # Base of arrow head
        bx, by = tip[0] - ux * head_len, tip[1] - uy * head_len
        # Two wing points
        wx = -uy * head_w
        wy = ux * head_w
        pts = np.array([
            [tip[0], tip[1]],
            [bx + wx, by + wy],
            [bx - wx, by - wy],
        ], dtype=np.int32)
        cv2.fillConvexPoly(img, pts, color, cv2.LINE_AA)

    if x_axis is not None:
        _draw_arrow(img, x_axis, origin, (0, 0, 255))
    if y_axis is not None:
        _draw_arrow(img, y_axis, origin, (0, 255, 0))
    if z_axis is not None:
        _draw_arrow(img, z_axis, origin, (255, 0, 0))
    cv2.circle(img, origin, 5, (255, 255, 255), -1)
    return img


def draw_origin_point(img_bgr, T_cam_obj, K, D, radius=5, color=(0, 255, 255)):
    img = img_bgr.copy()
    h, w = img.shape[:2]
    if T_cam_obj is None or np.asarray(T_cam_obj).shape != (4, 4):
        return img
    if not np.all(np.isfinite(T_cam_obj)) or float(T_cam_obj[2, 3]) <= 0.05:
        return img
    origin_3d = np.float32([[0, 0, 0]])
    R = T_cam_obj[:3, :3].astype(np.float32)
    t = T_cam_obj[:3, 3].astype(np.float32).reshape(3, 1)
    rvec, _ = cv2.Rodrigues(R)
    pts2d, _ = cv2.projectPoints(origin_3d, rvec, t, K, D)
    pt = pts2d.reshape(-1, 2)[0]
    if not np.all(np.isfinite(pt)):
        return img
    u = float(pt[0])
    v = float(pt[1])
    margin = max(w, h) * 2
    if abs(u) > 100000 or abs(v) > 100000:
        return img
    if u < -margin or u > w + margin or v < -margin or v > h + margin:
        return img
    cv2.circle(img, (int(round(u)), int(round(v))), int(radius), tuple(int(c) for c in color), -1)
    return img


def rot_x(rad):
    c, s = math.cos(rad), math.sin(rad)
    return np.array([
        [1, 0, 0],
        [0, c, -s],
        [0, s, c],
    ], dtype=np.float32)


def rot_z(rad):
    c, s = math.cos(rad), math.sin(rad)
    return np.array([
        [c, -s, 0],
        [s, c, 0],
        [0, 0, 1],
    ], dtype=np.float32)


SYM_MATS_OBJ = []
for flip in [0.0, math.pi]:
    rx_mat = rot_x(flip)
    for k in [0, 1, 2, 3]:
        rz_mat = rot_z(k * math.pi / 2.0)
        SYM_MATS_OBJ.append(rz_mat @ rx_mat)


def choose_best_symmetric_orientation(R_base_est, yaw_offset_deg=90.0, max_abs_deg=90.0):
    def decompose_rpy(R):
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.asin(np.clip(-R[2, 0], -1.0, 1.0))
        yaw = math.atan2(R[1, 0], R[0, 0])
        return roll, pitch, yaw

    best_R = R_base_est
    best_yaw_before = math.atan2(R_base_est[1, 0], R_base_est[0, 0])
    best_yaw_after = fix_yaw_with_offset_and_symmetry(
        best_yaw_before, offset_deg=yaw_offset_deg, max_abs_deg=max_abs_deg
    )
    best_sym_mat = np.eye(3, dtype=np.float32)

    roll0, pitch0, _ = decompose_rpy(R_base_est)
    best_cost = roll0 ** 2 + pitch0 ** 2 + best_yaw_after ** 2

    for sym_mat in SYM_MATS_OBJ:
        R_cand = R_base_est @ sym_mat
        roll, pitch, yaw = decompose_rpy(R_cand)
        yaw_fix = fix_yaw_with_offset_and_symmetry(
            yaw, offset_deg=yaw_offset_deg, max_abs_deg=max_abs_deg
        )
        cost = roll ** 2 + pitch ** 2 + yaw_fix ** 2
        if cost < best_cost:
            best_cost = cost
            best_R = R_cand
            best_yaw_before = yaw
            best_yaw_after = yaw_fix
            best_sym_mat = sym_mat

    return best_R, best_yaw_before, best_yaw_after, best_sym_mat


# ---------------------------------------------------------------------------
#  AprilTag-compatible pose transformation utilities
# ---------------------------------------------------------------------------

def transform_to_pose(transform_matrix):
    """
    Convert 4x4 transform matrix to [x, y, z, qx, qy, qz, qw] format.
    """
    import numpy as _np
    rotation_matrix = transform_matrix[0:3, 0:3]
    position = transform_matrix[0:3, 3]
    rotation = R.from_matrix(rotation_matrix)
    quaternion = rotation.as_quat()
    return _np.concatenate((position, quaternion))


def correct_box_pose(pos, quat):
    """
    Use dot-product to determine whether the box is facing toward or away
    from the camera, and flip 180° around local Y if facing away.

    Args:
        pos: [x, y, z] translation in camera frame
        quat: [x, y, z, w] quaternion in camera frame
    Returns:
        corrected_quat: [x, y, z, w]
    """
    r = R.from_quat(quat)
    rot_matrix = r.as_matrix()
    box_x_in_cam = rot_matrix[:, 0]

    if np.dot(box_x_in_cam, pos) < 0:
        rospy.logdebug("[APRILTAG] box facing away, applying 180° flip")
        q_flip = R.from_euler('y', 180, degrees=True)
        new_r = r * q_flip
        return new_r.as_quat()
    else:
        rospy.logdebug("[APRILTAG] box facing toward camera, no flip needed")
        return quat


def apply_vision_to_tag_offset(quat, angle_deg=-90):
    """
    Rotate around the object's local Y axis by `angle_deg` (default -90°)
    to convert the model coordinate frame into a pseudo-AprilTag frame.
    Right-multiplication = rotation around local axis.

    After rotation: original X → right, original Z → backward.
    """
    r = R.from_quat(quat)
    q_offset = R.from_euler('y', angle_deg, degrees=True)
    new_r = r * q_offset
    return new_r.as_quat()


def combine_id_with_status(data_id, status_flag, total_digits=5, id_digits=3, status_digits=2):
    """
    Combine data_id and status_flag into a fixed-width integer ID.

    Example: data_id=601, status_flag=2 → combined_id=60102 (id_digits=3, status_digits=2)

    Args:
        data_id: instance index starting from 601
        status_flag: basket class (obj_id - 1)
        total_digits: total digits in combined ID
        id_digits: digits reserved for data_id
        status_digits: digits reserved for status_flag
    Returns:
        combined_id: integer
    """
    status_flag_abs = abs(status_flag) if status_flag < 0 else status_flag

    data_id_str = str(data_id)
    if len(data_id_str) > id_digits:
        data_id_str = data_id_str[-id_digits:]
    data_id_str = data_id_str.zfill(id_digits)

    status_flag_str = str(status_flag_abs)
    if len(status_flag_str) > status_digits:
        status_flag_str = status_flag_str[-status_digits:]
    status_flag_str = status_flag_str.zfill(status_digits)

    combined_id_str = data_id_str + status_flag_str
    combined_id_str = combined_id_str.zfill(total_digits)
    if len(combined_id_str) > total_digits:
        combined_id_str = combined_id_str[-total_digits:]
    return int(combined_id_str)


def publish_april_tag_detections(publisher, poses_list, stamp, obj_ids_list, obj_names_list,
                                data_ids_list, status_flags_list, camera_frame,
                                total_digits=5, id_digits=3, status_digits=2):
    """
    Publish a batch of poses as AprilTagDetectionArray on /tag_detections.

    Args:
        publisher: rospy.Publisher for AprilTagDetectionArray
        poses_list: list of [x, y, z, qx, qy, qz, qw] arrays
        stamp: rospy.Time
        obj_ids_list: list of int obj_ids
        obj_names_list: list of str obj_names
        data_ids_list: list of int data_ids (starting from 601)
        status_flags_list: list of int status_flags (obj_id - 1)
        camera_frame: str, frame_id for header
    """
    if publisher is None or len(poses_list) == 0:
        return

    tag_detection_array = AprilTagDetectionArray()
    header = StdHeader()
    header.stamp = stamp
    header.frame_id = camera_frame
    tag_detection_array.header = header

    for i, pose in enumerate(poses_list):
        detection = AprilTagDetection()
        obj_id = obj_ids_list[i] if i < len(obj_ids_list) else None
        obj_name = obj_names_list[i] if i < len(obj_names_list) else "unknown"
        data_id = data_ids_list[i] if i < len(data_ids_list) else None
        status_flag = status_flags_list[i] if i < len(status_flags_list) else 0

        if data_id is None:
            data_id = 601 + i

        combined_id = combine_id_with_status(data_id, status_flag,
                                             total_digits=total_digits,
                                             id_digits=id_digits,
                                             status_digits=status_digits)
        detection.id = [combined_id]
        detection.size = [10]

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = camera_frame

        pose_relative = Pose()
        pose_relative.position.x = pose[0]
        pose_relative.position.y = pose[1]
        pose_relative.position.z = pose[2]
        pose_relative.orientation.x = pose[3]
        pose_relative.orientation.y = pose[4]
        pose_relative.orientation.z = pose[5]
        pose_relative.orientation.w = pose[6]

        pose_msg.pose.pose = pose_relative
        pose_msg.pose.covariance = [0] * 36
        detection.pose = pose_msg
        tag_detection_array.detections.append(detection)

        rospy.logdebug(
            f"[APRILTAG] published: obj_id={obj_id}, obj_name={obj_name}, "
            f"data_id={data_id}, status_flag={status_flag}, combined_id={combined_id}"
        )

    publisher.publish(tag_detection_array)
    rospy.loginfo(f"[APRILTAG] published {len(poses_list)} detections to /tag_detections")


def filter_center_column(
    xyxy,
    image_width,
    column_margin_ratio=0.25,
    min_bbox_area_abs=0.0,
    max_bbox_area_abs=0.0,
    max_keep=2,
    debug=False,
    logger=None,
):
    if xyxy is None or len(xyxy) == 0:
        return []

    log_fn = None
    if debug and logger:
        if callable(logger):
            log_fn = logger
        elif hasattr(logger, "info") and callable(logger.info):
            log_fn = logger.info

    xyxy = np.asarray(xyxy, dtype=np.float32)
    n_boxes = len(xyxy)

    centers_x = []
    centers_y = []
    widths = []
    areas = []

    for box in xyxy:
        x1, y1, x2, y2 = box
        x1i, y1i = max(int(x1), 0), max(int(y1), 0)
        x2i, y2i = int(x2), int(y2)
        centers_x.append(float((x1 + x2) * 0.5))
        centers_y.append(float((y1 + y2) * 0.5))
        widths.append(float(x2 - x1))
        areas.append(float(max(x2i - x1i, 0) * max(y2i - y1i, 0)))

    centers_x = np.array(centers_x, dtype=np.float32)
    centers_y = np.array(centers_y, dtype=np.float32)
    widths = np.array(widths, dtype=np.float32)
    areas = np.array(areas, dtype=np.float32)

    area_mask = areas >= float(min_bbox_area_abs)
    if float(max_bbox_area_abs) > 0.0:
        area_mask = area_mask & (areas <= float(max_bbox_area_abs))
    if not np.any(area_mask):
        area_mask = np.ones(n_boxes, dtype=bool)

    candidate_idx = np.where(area_mask)[0]
    if len(candidate_idx) == 0:
        return []

    img_center_x = float(image_width) * 0.5
    dist_to_center = np.abs(centers_x - img_center_x)
    min_dist_x = float(np.min(dist_to_center[candidate_idx]))
    avg_width = float(np.mean(widths[candidate_idx]))
    column_margin = float(column_margin_ratio) * avg_width

    center_column_mask = dist_to_center <= (min_dist_x + column_margin)
    final_mask = center_column_mask & area_mask
    final_idx = np.where(final_mask)[0]

    if len(final_idx) > 0:
        sorted_order = np.argsort(-centers_y[final_idx])
        final_idx = final_idx[sorted_order]

    kept_idx = final_idx[:max_keep] if max_keep > 0 else final_idx
    trimmed_idx = final_idx[max_keep:] if max_keep > 0 and len(final_idx) > max_keep else np.array([], dtype=int)

    if log_fn:
        log_fn(
            f"{COLOR_CYAN}[FILTER]{COLOR_RESET} img_center_x={img_center_x:.1f}, "
            f"min_dist_x={min_dist_x:.1f}, column_margin={column_margin:.1f}"
        )
        log_fn(f"{COLOR_CYAN}[FILTER]{COLOR_RESET} ========== pre-filter result ==========")
        kept_set = set(kept_idx.tolist())
        trimmed_set = set(trimmed_idx.tolist())
        for i, box in enumerate(xyxy):
            if i in kept_set:
                reason = f"pass(center+area, center_y={centers_y[i]:.1f})"
            elif i in trimmed_set:
                reason = f"trimmed_by_max_keep(center_y={centers_y[i]:.1f})"
            elif areas[i] < float(min_bbox_area_abs):
                reason = f"small_area({areas[i]:.0f} < {min_bbox_area_abs:.0f})"
            elif float(max_bbox_area_abs) > 0.0 and areas[i] > float(max_bbox_area_abs):
                reason = f"large_area({areas[i]:.0f} > {max_bbox_area_abs:.0f})"
            elif not center_column_mask[i]:
                reason = f"off_center({dist_to_center[i]:.1f} > {min_dist_x + column_margin:.1f})"
            else:
                reason = "filtered"
            log_fn(
                f"{COLOR_CYAN}[FILTER]{COLOR_RESET}[box#{i}] xyxy=({box[0]:.1f},{box[1]:.1f},{box[2]:.1f},{box[3]:.1f}) "
                f"center_x={centers_x[i]:.1f} center_y={centers_y[i]:.1f} area={areas[i]:.0f} | {reason}"
            )
        log_fn(f"{COLOR_CYAN}[FILTER]{COLOR_RESET} kept: {kept_idx.tolist()} (max_keep={max_keep})")

    return [int(i) for i in kept_idx.tolist()]


def filter_by_area_only(
    xyxy,
    image_width,
    column_margin_ratio=0.25,
    min_bbox_area_abs=0.0,
    max_bbox_area_abs=0.0,
    max_keep=0,
    debug=False,
    logger=None,
):
    del column_margin_ratio

    if xyxy is None or len(xyxy) == 0:
        return []

    log_fn = None
    if debug and logger:
        if callable(logger):
            log_fn = logger
        elif hasattr(logger, "info") and callable(logger.info):
            log_fn = logger.info

    xyxy = np.asarray(xyxy, dtype=np.float32)
    centers_x = []
    centers_y = []
    areas = []

    for box in xyxy:
        x1, y1, x2, y2 = box
        x1i, y1i = max(int(x1), 0), max(int(y1), 0)
        x2i, y2i = int(x2), int(y2)
        centers_x.append(float((x1 + x2) * 0.5))
        centers_y.append(float((y1 + y2) * 0.5))
        areas.append(float(max(x2i - x1i, 0) * max(y2i - y1i, 0)))

    centers_x = np.array(centers_x, dtype=np.float32)
    centers_y = np.array(centers_y, dtype=np.float32)
    areas = np.array(areas, dtype=np.float32)

    area_mask = areas >= float(min_bbox_area_abs)
    if float(max_bbox_area_abs) > 0.0:
        area_mask = area_mask & (areas <= float(max_bbox_area_abs))

    candidate_idx = np.where(area_mask)[0]
    if len(candidate_idx) == 0:
        final_idx = np.array([], dtype=int)
    else:
        img_center_x = float(image_width) * 0.5
        dist_to_center = np.abs(centers_x - img_center_x)
        # Prefer boxes closer to the image center; use lower image position as tie-breaker.
        order = np.lexsort((-centers_y[candidate_idx], dist_to_center[candidate_idx]))
        final_idx = candidate_idx[order]
        if max_keep > 0:
            final_idx = final_idx[:max_keep]

    if log_fn:
        log_fn(f"{COLOR_CYAN}[FILTER]{COLOR_RESET} ========== pre-filter result ==========")
        kept_set = set(final_idx.tolist())
        img_center_x = float(image_width) * 0.5
        dist_to_center = np.abs(centers_x - img_center_x)
        for i, box in enumerate(xyxy):
            if i in kept_set:
                reason = f"pass(area+center, center_y={centers_y[i]:.1f})"
            elif areas[i] < float(min_bbox_area_abs):
                reason = f"small_area({areas[i]:.0f} < {min_bbox_area_abs:.0f})"
            elif float(max_bbox_area_abs) > 0.0 and areas[i] > float(max_bbox_area_abs):
                reason = f"large_area({areas[i]:.0f} > {max_bbox_area_abs:.0f})"
            elif max_keep > 0:
                reason = f"trimmed_by_max_keep(dist_x={dist_to_center[i]:.1f})"
            else:
                reason = "filtered"
            log_fn(
                f"{COLOR_CYAN}[FILTER]{COLOR_RESET}[box#{i}] xyxy=({box[0]:.1f},{box[1]:.1f},{box[2]:.1f},{box[3]:.1f}) "
                f"center_x={centers_x[i]:.1f} center_y={centers_y[i]:.1f} area={areas[i]:.0f} | {reason}"
            )
        log_fn(f"{COLOR_CYAN}[FILTER]{COLOR_RESET} kept: {final_idx.tolist()} (max_keep={max_keep})")

    return [int(i) for i in final_idx.tolist()]


def estimate_total_basket_count(
    top_layer_results,
    box_height_m,
    pallet_height_m,
    logger=None,
    max_layers=6,
    full_layer_box_count=6,
):
    if top_layer_results is None or len(top_layer_results) == 0:
        return {
            "top_count": 0,
            "layer_count": 0,
            "total_count": 0,
            "z_top_mean": None,
        }

    if box_height_m <= 1e-6:
        raise ValueError(f"box_height_m must be positive, got {box_height_m}")

    top_z_values = np.array([r["T_base_obj"][2, 3] for r in top_layer_results], dtype=np.float32)
    z_top_mean = float(np.mean(top_z_values))
    top_count = int(len(top_layer_results))
    # BOP/GDRN pose uses the model origin near the 3D bbox center, so the returned z
    # is expected to be close to the box center height rather than the top face.
    raw_center_layers = (z_top_mean - float(pallet_height_m)) / float(box_height_m) + 0.5
    layer_count = int(np.rint(raw_center_layers))
    layer_count = max(1, min(int(max_layers), layer_count))
    total_count = int(full_layer_box_count) * (layer_count - 1) + top_count

    if logger is not None:
        logger(
            f"{COLOR_CYAN}[COUNT]{COLOR_RESET} top_z_mean={z_top_mean:.3f}m, "
            f"pallet_h={pallet_height_m:.3f}m, box_h={box_height_m:.3f}m, "
            f"raw_center_layers={raw_center_layers:.3f}, "
            f"max_layers={int(max_layers)}, full_layer_box_count={int(full_layer_box_count)}, "
            f"layers={layer_count}, top_count={top_count}, total={total_count}"
        )

    return {
        "top_count": top_count,
        "layer_count": layer_count,
        "total_count": total_count,
        "z_top_mean": z_top_mean,
    }




class SharedBasketPoseServiceNode:
    def __init__(self):
        rospy.init_node("basket_pose_shared_service_node", anonymous=True)
        self.bridge = CvBridge()
        self.inference_lock = threading.Lock()
        self.image_lock = threading.Lock()
        self.pose_lock = threading.Lock()

        self.camera_frame = rospy.get_param("~camera_frame", "camera_color_optical_frame")
        self.save_outputs = bool(rospy.get_param("~save_outputs", True))

        # Target inference resolution (both 0 = use native resolution)
        self.inference_width = int(rospy.get_param("~inference_image_width", 640))
        self.inference_height = int(rospy.get_param("~inference_image_height", 480))

        proj_root = osp.abspath(rospy.get_param("~proj_root", "/media/data/basket_gdrnpp/"))
        self.proj_root = proj_root
        default_box_cfg = "core/gdrn_modeling/demo/box_configs/basket_5.yaml"
        box_config_yaml = rospy.get_param("~box_config_yaml", default_box_cfg)
        self.box_config_path = self.resolve_local_path(box_config_yaml)
        self.box_config = self.load_box_config(self.box_config_path)
        self.box_name = str(self.box_config.get("box_name", osp.splitext(osp.basename(self.box_config_path))[0]))
        self.validate_box_config()

        runtime_model_paths = self.load_runtime_model_paths()
        gdrn_cfg = runtime_model_paths["gdrn_cfg"]
        gdrn_ckpt = runtime_model_paths["gdrn_ckpt"]
        obj_models = runtime_model_paths["obj_models"]
        yolo_weights = runtime_model_paths["yolo_weights"]

        camera_info_topic = rospy.get_param("~camera_info_topic", "/camera/color/camera_info")
        depth_scale_param = rospy.get_param("~depth_scale", 0.001)
        self.depth_scale = float(depth_scale_param)

        rospy.loginfo(f"Waiting for CameraInfo on {camera_info_topic} ...")
        cam_info_msg = rospy.wait_for_message(camera_info_topic, CameraInfo)
        rospy.loginfo("Got CameraInfo.")

        K = np.array(cam_info_msg.K, dtype=np.float32).reshape(3, 3)
        D = np.array(cam_info_msg.D, dtype=np.float32)
        self.native_width = int(cam_info_msg.width)
        self.native_height = int(cam_info_msg.height)

        # Compute image scale for resolution-adaptive inference
        self.pad_left = 0
        self.pad_top = 0
        self.pad_right = 0
        self.pad_bottom = 0
        if self.inference_width > 0 and self.inference_height > 0 \
                and (self.inference_width != self.native_width or self.inference_height != self.native_height):
            # Letterbox: scale to fit within target, maintain aspect ratio, then pad
            scale_w = float(self.inference_width) / float(self.native_width)
            scale_h = float(self.inference_height) / float(self.native_height)
            self.image_scale = min(scale_w, scale_h)
            scaled_w = int(round(self.native_width * self.image_scale))
            scaled_h = int(round(self.native_height * self.image_scale))
            self.scaled_w = scaled_w
            self.scaled_h = scaled_h
            self.pad_left = int((self.inference_width - scaled_w) / 2)
            self.pad_top = int((self.inference_height - scaled_h) / 2)
            self.pad_right = self.inference_width - scaled_w - self.pad_left
            self.pad_bottom = self.inference_height - scaled_h - self.pad_top
            # Scale intrinsics: fx/fy proportional, cx/cy adjusted for padding offset
            K[0, 0] *= self.image_scale  # fx
            K[1, 1] *= self.image_scale  # fy
            K[0, 2] = K[0, 2] * self.image_scale + self.pad_left   # cx
            K[1, 2] = K[1, 2] * self.image_scale + self.pad_top    # cy
            rospy.loginfo(
                f"[RESOLUTION] letterbox {self.native_width}x{self.native_height} "
                f"→ {self.inference_width}x{self.inference_height} "
                f"(content={scaled_w}x{scaled_h}, scale={self.image_scale:.4f}, "
                f"pad=[{self.pad_top},{self.pad_bottom},{self.pad_left},{self.pad_right}])"
            )
            rospy.loginfo(
                f"[RESOLUTION] scaled intrinsics: fx={K[0,0]:.2f}, fy={K[1,1]:.2f}, "
                f"cx={K[0,2]:.2f}, cy={K[1,2]:.2f}"
            )
        else:
            self.image_scale = 1.0
            self.inference_width = self.native_width
            self.inference_height = self.native_height
            rospy.loginfo(
                f"[RESOLUTION] using native resolution {self.native_width}x{self.native_height}"
            )

        self.cam_info = {
            "intrinsics": K,
            "distortion": D,
        }

        camera_json_path = rospy.get_param("~camera_json", osp.join(proj_root, "camera.json"))
        cam_json_dict = {
            "fx": float(K[0, 0]),
            "fy": float(K[1, 1]),
            "cx": float(K[0, 2]),
            "cy": float(K[1, 2]),
            "width": self.inference_width,
            "height": self.inference_height,
            "depth_scale": float(depth_scale_param),
        }
        os.makedirs(osp.dirname(camera_json_path), exist_ok=True)
        with open(camera_json_path, "w") as f:
            json.dump(cam_json_dict, f, indent=2)
        rospy.loginfo(f"camera.json written to: {camera_json_path}")

        self.gdrn_predictor = GdrnPredictor(
            config_file_path=gdrn_cfg,
            ckpt_file_path=gdrn_ckpt,
            camera_json_path=camera_json_path,
            path_to_obj_models=obj_models,
        )
        if not osp.isabs(self.gdrn_predictor.cfg.OUTPUT_DIR):
            self.gdrn_predictor.cfg.OUTPUT_DIR = osp.join(proj_root, self.gdrn_predictor.cfg.OUTPUT_DIR)
        os.makedirs(self.gdrn_predictor.cfg.OUTPUT_DIR, exist_ok=True)
        rospy.loginfo(f"GDRN output dir: {self.gdrn_predictor.cfg.OUTPUT_DIR}")
        # obj_id is set per-detection from YOLO class in the inference loop
        rospy.loginfo(f"GDRN predictor obj_ids={self.gdrn_predictor.obj_ids}, objs={self.gdrn_predictor.objs}")
        self.detector = YOLO(yolo_weights)

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.latest_pose_groups = {"single": {}, "top": {}}
        self.tf_timer = rospy.Timer(rospy.Duration(0.1), self.publish_latest_tf)

        self.rgb_image = None
        self.depth_image = None

        rospy.Subscriber("/camera/color/image_raw", Image, self.rgb_callback, queue_size=1)
        rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback, queue_size=1)

        self.single_cfg = self.load_single_cfg()
        self.top_cfg = self.load_top_cfg()

        self.service_single = rospy.Service(
            "/infer_basket_pose", InferBasketPose, self.handle_single_service
        )
        self.service_top = rospy.Service(
            "/infer_top_basket_ids", InferBasketPose, self.handle_top_service
        )

        # Publish pseudo-AprilTag detections for compatibility with QR/AprilTag workflow
        self.tag_publisher = rospy.Publisher(
            "/tag_detections", AprilTagDetectionArray, queue_size=10
        )

        # Publish visualization image for downstream consumers
        self.viz_publisher = rospy.Publisher(
            "/basket_vision/viz_image", Image, queue_size=1
        )

        rospy.loginfo("SharedBasketPoseServiceNode ready.")
        rospy.loginfo("  service: /infer_basket_pose")
        rospy.loginfo("  service: /infer_top_basket_ids")
        rospy.loginfo("  publisher: /tag_detections (AprilTagDetectionArray)")
        rospy.loginfo("  publisher: /basket_vision/viz_image")

    def _get_single_param(self, name, default):
        scoped_name = f"~single_{name}"
        legacy_name = f"~{name}"
        if rospy.has_param(scoped_name):
            return rospy.get_param(scoped_name)
        return rospy.get_param(legacy_name, default)

    def _get_top_param(self, name, default):
        scoped_name = f"~top_{name}"
        return rospy.get_param(scoped_name, default)

    def resolve_local_path(self, path_value):
        if path_value is None:
            return None
        if osp.isabs(path_value):
            return path_value
        return osp.abspath(osp.join(self.proj_root, path_value))

    def resolve_cfg_dir_path(self, path_value):
        resolved = self.resolve_local_path(path_value)
        if resolved is None:
            return None
        return resolved

    def load_box_config(self, config_path):
        if not config_path:
            raise ValueError("box config yaml path is required")
        if not osp.exists(config_path):
            raise FileNotFoundError(f"box config yaml not found: {config_path}")
        with open(config_path, "r") as f:
            data = yaml.safe_load(f) or {}
        if not isinstance(data, dict):
            raise ValueError(f"box config yaml must be a dict: {config_path}")
        rospy.loginfo(
            f"[BOX_CFG] loaded box config: {config_path}"
            f" (box_name={data.get('box_name', osp.splitext(osp.basename(config_path))[0])})"
        )
        return data

    def load_runtime_model_paths(self):
        runtime_cfg = self.box_config.get("runtime_models") or {}
        if not isinstance(runtime_cfg, dict):
            raise ValueError(
                f"box config runtime_models must be a dict: {self.box_config_path}"
            )

        resolved = {}
        for key in ("gdrn_cfg", "gdrn_ckpt", "obj_models", "yolo_weights"):
            ros_param_name = f"~{key}"
            if rospy.has_param(ros_param_name):
                raw_value = rospy.get_param(ros_param_name)
                source = ros_param_name
            elif key in runtime_cfg:
                raw_value = runtime_cfg[key]
                source = f"runtime_models.{key}"
            else:
                raise ValueError(
                    f"[BOX_CFG] missing required runtime model path `runtime_models.{key}` in "
                    f"{self.box_config_path}"
                )

            if raw_value in (None, ""):
                raise ValueError(
                    f"[BOX_CFG] empty required runtime model path `{key}` in "
                    f"{self.box_config_path}"
                )

            resolved[key] = self.resolve_local_path(raw_value)
            rospy.loginfo(f"[BOX_CFG] {key} <- {source}: {resolved[key]}")

        return resolved

    def _get_box_cfg_value(self, mode_name, key, default):
        common_cfg = self.box_config.get("common") or {}
        mode_cfg = self.box_config.get(mode_name) or {}
        if not isinstance(common_cfg, dict):
            common_cfg = {}
        if not isinstance(mode_cfg, dict):
            mode_cfg = {}
        if key in mode_cfg:
            return mode_cfg[key]
        if key in common_cfg:
            return common_cfg[key]
        return default

    def validate_box_config(self):
        rospy.loginfo(f"[BOX_CFG] {self.box_name} free-form placement config validated")

    def get_basket_type(self, box_name=None):
        box_name = str(box_name or self.box_name)
        return box_name

    def load_single_cfg(self):
        save_dir_default = self._get_box_cfg_value("single", "save_dir", "/media/data/basket_gdrnpp/vis_logs")
        save_dir = self.resolve_cfg_dir_path(self._get_single_param("save_dir", save_dir_default))
        os.makedirs(save_dir, exist_ok=True)
        return {
            "mode": "single",
            "service_name": "/infer_basket_pose",
            "pose_group": "single",
            "save_dir": save_dir,
            "base_frame": self._get_single_param(
                "base_frame", self._get_box_cfg_value("single", "base_frame", BASE_LINK_FRAME)
            ),
            "conf_thr": float(
                self._get_single_param("conf_thr", self._get_box_cfg_value("single", "conf_thr", 0.10))
            ),
            "column_margin_ratio": float(
                self._get_single_param(
                    "column_margin_ratio",
                    self._get_box_cfg_value("single", "column_margin_ratio", 0.25),
                )
            ),
            "min_bbox_area_abs": float(
                self._get_single_param(
                    "min_bbox_area_abs", self._get_box_cfg_value("single", "min_bbox_area_abs", 7000.0)
                )
            ),
            "max_keep_instances": int(
                self._get_single_param(
                    "max_keep_instances", self._get_box_cfg_value("single", "max_keep_instances", 2)
                )
            ),
            "max_bbox_area_abs": float(
                self._get_single_param(
                    "max_bbox_area_abs", self._get_box_cfg_value("single", "max_bbox_area_abs", 0.0)
                )
            ),
            "filter_debug": bool(
                self._get_single_param("filter_debug", self._get_box_cfg_value("single", "filter_debug", True))
            ),
            "use_depth_translation": bool(
                self._get_single_param(
                    "use_depth_translation", self._get_box_cfg_value("single", "use_depth_translation", False)
                )
            ),
            "depth_translation_roi_ratio": float(
                self._get_single_param(
                    "depth_translation_roi_ratio",
                    self._get_box_cfg_value("single", "depth_translation_roi_ratio", 0.50),
                )
            ),
            "box_height_m": float(
                self._get_single_param("box_height_m", self._get_box_cfg_value("single", "box_height_m", 0.180))
            ),
            "layer_z_threshold": float(
                self._get_single_param(
                    "layer_z_threshold", self._get_box_cfg_value("single", "layer_z_threshold", 0.10)
                )
            ),
            "pose_rp_max_deg": float(
                self._get_single_param(
                    "pose_rp_max_deg", self._get_box_cfg_value("single", "pose_rp_max_deg", 20.0)
                )
            ),
            "canonicalize_axes_to_base": bool(
                self._get_single_param("canonicalize_axes_to_base", True)
            ),
            "base_x_offset_m": float(
                self._get_single_param("base_x_offset_m", self._get_box_cfg_value("single", "base_x_offset_m", 0.0))
            ),
            "base_y_offset_m": float(
                self._get_single_param("base_y_offset_m", self._get_box_cfg_value("single", "base_y_offset_m", 0.0))
            ),
            "base_z_offset_m": float(
                self._get_single_param("base_z_offset_m", self._get_box_cfg_value("single", "base_z_offset_m", 0.0))
            ),
            "base_x_min": float(
                self._get_single_param("base_x_min", self._get_box_cfg_value("single", "base_x_min", 0.55))
            ),
            "base_x_max": float(
                self._get_single_param("base_x_max", self._get_box_cfg_value("single", "base_x_max", 0.87))
            ),
            "max_saved_runs": int(
                self._get_single_param("max_saved_runs", self._get_box_cfg_value("single", "max_saved_runs", 35))
            ),
            "filter_fn": filter_center_column,
            "position_filter": "x_only",
            "select_single_best": True,
            "publish_only_top": True,
        }

    def load_top_cfg(self):
        top_box_cfg = self.box_config.get("top") or {}
        full_layer_box_count_default = 5
        max_layers_default = int(self._get_box_cfg_value("top", "max_layers", 4))

        save_dir_default = self._get_box_cfg_value("top", "save_dir", "/media/data/basket_gdrnpp/top_vis_logs")
        save_dir = self.resolve_cfg_dir_path(self._get_top_param("save_dir", save_dir_default))
        os.makedirs(save_dir, exist_ok=True)
        return {
            "mode": "top",
            "service_name": "/infer_top_basket_ids",
            "pose_group": "top",
            "save_dir": save_dir,
            "base_frame": self._get_top_param(
                "base_frame", self._get_box_cfg_value("top", "base_frame", BASE_LINK_FRAME)
            ),
            "conf_thr": float(self._get_top_param("conf_thr", self._get_box_cfg_value("top", "conf_thr", 0.10))),
            "column_margin_ratio": float(
                self._get_top_param(
                    "column_margin_ratio",
                    self._get_box_cfg_value("top", "column_margin_ratio", 0.25),
                )
            ),
            "min_bbox_area_abs": float(
                self._get_top_param(
                    "min_bbox_area_abs", self._get_box_cfg_value("top", "min_bbox_area_abs", 2000.0)
                )
            ),
            "max_keep_instances": int(
                self._get_top_param(
                    "max_keep_instances", self._get_box_cfg_value("top", "max_keep_instances", 6)
                )
            ),
            "max_bbox_area_abs": float(
                self._get_top_param("max_bbox_area_abs", self._get_box_cfg_value("top", "max_bbox_area_abs", 0.0))
            ),
            "filter_debug": bool(
                self._get_top_param("filter_debug", self._get_box_cfg_value("top", "filter_debug", True))
            ),
            "use_depth_translation": bool(
                self._get_top_param("use_depth_translation", self._get_box_cfg_value("top", "use_depth_translation", False))
            ),
            "depth_translation_roi_ratio": float(
                self._get_top_param(
                    "depth_translation_roi_ratio",
                    self._get_box_cfg_value("top", "depth_translation_roi_ratio", 0.50),
                )
            ),
            "camera_prefilter_enabled": bool(
                self._get_top_param(
                    "camera_prefilter_enabled",
                    self._get_box_cfg_value("top", "camera_prefilter_enabled", True),
                )
            ),
            "camera_x_min": float(
                self._get_top_param("camera_x_min", self._get_box_cfg_value("top", "camera_x_min", -0.8))
            ),
            "camera_x_max": float(
                self._get_top_param("camera_x_max", self._get_box_cfg_value("top", "camera_x_max", 0.8))
            ),
            "camera_y_min": float(
                self._get_top_param("camera_y_min", self._get_box_cfg_value("top", "camera_y_min", -0.2))
            ),
            "camera_y_max": float(
                self._get_top_param("camera_y_max", self._get_box_cfg_value("top", "camera_y_max", 1.2))
            ),
            "camera_z_min": float(
                self._get_top_param("camera_z_min", self._get_box_cfg_value("top", "camera_z_min", 0.4))
            ),
            "camera_z_max": float(
                self._get_top_param("camera_z_max", self._get_box_cfg_value("top", "camera_z_max", 1.6))
            ),
            "box_height_m": float(
                self._get_top_param("box_height_m", self._get_box_cfg_value("top", "box_height_m", 0.180))
            ),
            "pallet_height_m": float(
                self._get_top_param(
                    "pallet_height_m", self._get_box_cfg_value("top", "pallet_height_m", 0.170)
                )
            ),
            "layer_z_threshold": float(
                self._get_top_param(
                    "layer_z_threshold", self._get_box_cfg_value("top", "layer_z_threshold", 0.09)
                )
            ),
            "pose_rp_max_deg": float(
                self._get_top_param(
                    "pose_rp_max_deg", self._get_box_cfg_value("top", "pose_rp_max_deg", 20.0)
                )
            ),
            "canonicalize_axes_to_base": bool(
                self._get_top_param("canonicalize_axes_to_base", True)
            ),
            "base_x_offset_m": float(
                self._get_top_param("base_x_offset_m", self._get_box_cfg_value("top", "base_x_offset_m", 0.0))
            ),
            "base_y_offset_m": float(
                self._get_top_param("base_y_offset_m", self._get_box_cfg_value("top", "base_y_offset_m", 0.0))
            ),
            "base_z_offset_m": float(
                self._get_top_param("base_z_offset_m", self._get_box_cfg_value("top", "base_z_offset_m", 0.0))
            ),
            "return_single_target": bool(
                self._get_top_param(
                    "return_single_target",
                    self._get_box_cfg_value("top", "return_single_target", True),
                )
            ),
            "base_x_min": float(
                self._get_top_param("base_x_min", self._get_box_cfg_value("top", "base_x_min", 0.50))
            ),
            "base_x_max": float(
                self._get_top_param("base_x_max", self._get_box_cfg_value("top", "base_x_max", 1.9))
            ),
            "base_y_min": float(
                self._get_top_param("base_y_min", self._get_box_cfg_value("top", "base_y_min", -0.4))
            ),
            "base_y_max": float(
                self._get_top_param("base_y_max", self._get_box_cfg_value("top", "base_y_max", 0.4))
            ),
            "top_slot_match_max_dist_m": float(
                self._get_top_param(
                    "top_slot_match_max_dist_m",
                    self._get_box_cfg_value("top", "top_slot_match_max_dist_m", 0.30),
                )
            ),
            "max_layers": int(
                self._get_top_param("max_layers", self._get_box_cfg_value("top", "max_layers", max_layers_default))
            ),
            "full_layer_box_count": int(
                self._get_top_param(
                    "full_layer_box_count",
                    self._get_box_cfg_value("top", "full_layer_box_count", full_layer_box_count_default),
                )
            ),
            "max_saved_runs": int(
                self._get_top_param("max_saved_runs", self._get_box_cfg_value("top", "max_saved_runs", 35))
            ),
            "box_name": self.box_name,
            "filter_fn": filter_by_area_only,
            "position_filter": "xy",
            "disable_position_filter": bool(
                self._get_top_param(
                    "disable_position_filter",
                    self._get_box_cfg_value("top", "disable_position_filter", False),
                )
            ),
            "select_single_best": False,
            "publish_only_top": True,
        }

    def rgb_callback(self, msg):
        try:
            if not msg.data or msg.width == 0 or msg.height == 0:
                return
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            enc = (msg.encoding or "").lower()
            if enc == "rgb8":
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            elif enc == "rgba8":
                img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
            elif enc == "bgra8":
                img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
            elif img.ndim == 2:
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            img = ensure_numpy(img, want_dtype=np.uint8, want_channels=3, name="RGB_cb")
            if self.image_scale != 1.0:
                img = cv2.resize(img, (self.scaled_w, self.scaled_h),
                                 interpolation=cv2.INTER_LINEAR)
                img = cv2.copyMakeBorder(img, self.pad_top, self.pad_bottom,
                                         self.pad_left, self.pad_right,
                                         cv2.BORDER_CONSTANT, value=(0, 0, 0))
            with self.image_lock:
                self.rgb_image = img
        except Exception as e:
            rospy.logerr(f"[RGB] callback error: {e}")

    def depth_callback(self, msg):
        try:
            if not msg.data or msg.width == 0 or msg.height == 0:
                return
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            if isinstance(depth, np.ndarray) and depth.dtype in (np.uint16, np.float32):
                pass
            elif isinstance(depth, np.ndarray) and depth.dtype == np.float64:
                depth = depth.astype(np.float32, copy=False)
            else:
                depth = depth.astype(np.uint16, copy=False)
            depth = ensure_numpy(depth, name="DEPTH_cb")
            if self.image_scale != 1.0:
                depth = cv2.resize(depth, (self.scaled_w, self.scaled_h),
                                   interpolation=cv2.INTER_NEAREST)
                depth = cv2.copyMakeBorder(depth, self.pad_top, self.pad_bottom,
                                           self.pad_left, self.pad_right,
                                           cv2.BORDER_CONSTANT, value=0)
            with self.image_lock:
                self.depth_image = depth
        except Exception as e:
            rospy.logerr(f"[DEPTH] callback error: {e}")

    def publish_latest_tf(self, _event):
        with self.pose_lock:
            pose_groups = [dict(group) for group in self.latest_pose_groups.values()]
        for pose_group in pose_groups:
            for tf_name, (T_matrix, parent_frame) in pose_group.items():
                self.publish_pose_and_tf(T_matrix, parent_frame, tf_name)

    def get_latest_images(self):
        with self.image_lock:
            rgb = None if self.rgb_image is None else self.rgb_image.copy()
            depth = None if self.depth_image is None else self.depth_image.copy()
        return rgb, depth

    def clear_pose_group(self, group_name):
        with self.pose_lock:
            self.latest_pose_groups[group_name] = {}

    def set_pose_group(self, group_name, pose_map):
        with self.pose_lock:
            self.latest_pose_groups[group_name] = dict(pose_map)

    def lookup_transform_matrix(self, target_frame, source_frame):
        tf_msg = self.tf_buffer.lookup_transform(
            target_frame=target_frame,
            source_frame=source_frame,
            time=rospy.Time(0),
            timeout=rospy.Duration(1.0),
        )
        trans = tf_msg.transform.translation
        quat = tf_msg.transform.rotation
        T_target_source = quaternion_matrix([quat.x, quat.y, quat.z, quat.w])
        T_target_source[0, 3] = trans.x
        T_target_source[1, 3] = trans.y
        T_target_source[2, 3] = trans.z
        return T_target_source

    def transform_pose_between_frames(self, T_source_obj, source_frame, target_frame):
        if source_frame == target_frame:
            return T_source_obj.copy()
        T_target_source = self.lookup_transform_matrix(target_frame, source_frame)
        return T_target_source.dot(T_source_obj)

    def estimate_depth_translation_from_bbox(self, depth, bbox_xyxy, roi_ratio=0.50):
        if depth is None:
            return None
        h_img, w_img = depth.shape[:2]
        x1, y1, x2, y2 = [float(v) for v in bbox_xyxy]
        cx = 0.5 * (x1 + x2)
        cy = 0.5 * (y1 + y2)
        bw = max(x2 - x1, 1.0)
        bh = max(y2 - y1, 1.0)
        rw = max(3, int(bw * float(roi_ratio) * 0.5))
        rh = max(3, int(bh * float(roi_ratio) * 0.5))
        u0 = max(0, int(round(cx)) - rw)
        u1 = min(w_img, int(round(cx)) + rw + 1)
        v0 = max(0, int(round(cy)) - rh)
        v1 = min(h_img, int(round(cy)) + rh + 1)
        roi = depth[v0:v1, u0:u1]
        if roi.size == 0:
            return None
        vals = roi.astype(np.float32).reshape(-1)
        vals = vals[np.isfinite(vals)]
        vals = vals[vals > 0]
        if vals.size < 20:
            return None
        z_raw = float(np.median(vals))
        z = z_raw * self.depth_scale if z_raw > 20.0 else z_raw
        if not np.isfinite(z) or z <= 0.05 or z > 5.0:
            return None
        K = self.cam_info["intrinsics"]
        fx, fy = float(K[0, 0]), float(K[1, 1])
        ox, oy = float(K[0, 2]), float(K[1, 2])
        if abs(fx) < 1e-6 or abs(fy) < 1e-6:
            return None
        x = (cx - ox) * z / fx
        y = (cy - oy) * z / fy
        return np.array([x, y, z], dtype=np.float32), (cx, cy), z_raw, vals.size

    def filter_candidates_by_camera_depth(self, keep, xyxy, depth, cfg):
        if cfg.get("mode") != "top" or not cfg.get("camera_prefilter_enabled", False):
            return keep

        filtered = []
        x_min = float(cfg.get("camera_x_min", -0.8))
        x_max = float(cfg.get("camera_x_max", 0.8))
        y_min = float(cfg.get("camera_y_min", -0.2))
        y_max = float(cfg.get("camera_y_max", 1.2))
        z_min = float(cfg.get("camera_z_min", 0.4))
        z_max = float(cfg.get("camera_z_max", 1.6))
        rospy.loginfo(
            f"{COLOR_CYAN}[CAM_PREFILTER]{COLOR_RESET} "
            f"camera x in [{x_min:.3f}, {x_max:.3f}], "
            f"y in [{y_min:.3f}, {y_max:.3f}], z in [{z_min:.3f}, {z_max:.3f}]m"
        )

        for idx in keep:
            depth_pose = self.estimate_depth_translation_from_bbox(
                depth,
                xyxy[int(idx)],
                roi_ratio=cfg.get("depth_translation_roi_ratio", 0.50),
            )
            if depth_pose is None:
                rospy.logwarn(
                    f"{COLOR_CYAN}[CAM_PREFILTER]{COLOR_RESET} inst#{int(idx)}: no valid depth -> reject"
                )
                continue
            t_cam, uv, depth_raw, depth_count = depth_pose
            cx, cy, cz = [float(v) for v in t_cam]
            ok = x_min <= cx <= x_max and y_min <= cy <= y_max and z_min <= cz <= z_max
            if ok:
                filtered.append(int(idx))
                rospy.loginfo(
                    f"{COLOR_CYAN}[CAM_PREFILTER]{COLOR_RESET} inst#{int(idx)}: "
                    f"t_cam=({cx:.3f},{cy:.3f},{cz:.3f}) uv=({uv[0]:.1f},{uv[1]:.1f}) "
                    f"raw_median={depth_raw:.1f} n={depth_count} -> pass"
                )
            else:
                rospy.logwarn(
                    f"{COLOR_CYAN}[CAM_PREFILTER]{COLOR_RESET} inst#{int(idx)}: "
                    f"t_cam=({cx:.3f},{cy:.3f},{cz:.3f}) uv=({uv[0]:.1f},{uv[1]:.1f}) "
                    f"raw_median={depth_raw:.1f} n={depth_count} -> reject"
                )

        if len(filtered) == 0:
            rospy.logwarn(f"{COLOR_CYAN}[CAM_PREFILTER]{COLOR_RESET} all candidates rejected")
            return np.array([], dtype=np.int64)
        return np.array(filtered, dtype=np.int64)

    def apply_base_translation_offset(self, T_cam_obj, T_base_obj, T_base_link_obj, cfg):
        offset = np.array(
            [
                float(cfg.get("base_x_offset_m", 0.0)),
                float(cfg.get("base_y_offset_m", 0.0)),
                float(cfg.get("base_z_offset_m", 0.0)),
            ],
            dtype=np.float32,
        )
        if float(np.linalg.norm(offset)) <= 1e-9:
            return T_cam_obj, T_base_obj, T_base_link_obj

        corrected_base_link = T_base_link_obj.copy()
        corrected_base_link[:3, 3] = corrected_base_link[:3, 3] + offset
        corrected_cam = self.transform_pose_between_frames(
            corrected_base_link, BASE_LINK_FRAME, self.camera_frame
        )
        if cfg["base_frame"] == BASE_LINK_FRAME:
            corrected_base = corrected_base_link.copy()
        else:
            corrected_base = self.transform_pose_between_frames(
                corrected_base_link, BASE_LINK_FRAME, cfg["base_frame"]
            )
        rospy.loginfo(
            f"{COLOR_CYAN}[POSE_OFFSET]{COLOR_RESET} {cfg.get('mode', 'unknown')} base offset applied: "
            f"dx={offset[0]:.3f}, dy={offset[1]:.3f}, dz={offset[2]:.3f}m"
        )
        return corrected_cam, corrected_base, corrected_base_link

    def handle_single_service(self, req):
        del req
        return self.handle_service(self.single_cfg)

    def handle_top_service(self, req):
        del req
        return self.handle_service(self.top_cfg)

    def handle_service(self, cfg):
        t_service_start = time.perf_counter()
        res = InferBasketPoseResponse()
        try:
            rgb, depth = self.get_latest_images()
            if rgb is None or depth is None:
                res.success = False
                res.message = "No RGB/Depth yet. Ensure camera topics are publishing."
                res.num_instances = 0
                return res

            with self.inference_lock:
                pipeline_data = self.run_shared_pipeline(rgb, depth, cfg)

            if pipeline_data is None:
                self.clear_pose_group(cfg["pose_group"])
                res.success = False
                res.message = "Inference pipeline did not produce results."
                res.num_instances = 0
                return res

            return self.finish_service_response(
                cfg=cfg,
                res=res,
                pipeline_data=pipeline_data,
                t_service_start=t_service_start,
            )
        except Exception as e:
            rospy.logerr(f"[Service {cfg['service_name']}] error: {e}")
            if rospy.get_param("~debug", True):
                traceback.print_exc()
            self.clear_pose_group(cfg["pose_group"])
            res.success = False
            res.message = str(e)
            res.num_instances = 0
            return res

    def run_shared_pipeline(self, rgb, depth, cfg):
        conf_thr = cfg["conf_thr"]
        t_det_start = time.perf_counter()
        rgb_for_yolo = PILImage.fromarray(cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB))
        yolo_res = self.detector.predict(rgb_for_yolo, imgsz=640, conf=conf_thr, verbose=False)
        if len(yolo_res) == 0 or len(yolo_res[0].boxes) == 0:
            return {
                "status": "no_detection",
                "message": f"No detection from YOLO (conf >= {conf_thr}).",
            }

        boxes = yolo_res[0].boxes
        xyxy = boxes.xyxy.cpu().numpy()
        conf = boxes.conf.cpu().numpy().reshape(-1)
        cls = boxes.cls.cpu().numpy().astype(int)  # YOLO class ids (0-indexed)
        keep = np.where(conf >= conf_thr)[0]
        if keep.size == 0:
            return {
                "status": "no_detection",
                "message": f"No detection passed conf >= {conf_thr}.",
            }

        bbox_w = xyxy[:, 2] - xyxy[:, 0]
        bbox_h = xyxy[:, 3] - xyxy[:, 1]
        min_bbox_w_abs = float(cfg.get("min_bbox_width_abs", 20.0))
        min_bbox_h_abs = float(cfg.get("min_bbox_height_abs", 20.0))
        size_mask = (bbox_w >= min_bbox_w_abs) & (bbox_h >= min_bbox_h_abs)
        keep = np.array([int(j) for j in keep if bool(size_mask[int(j)])], dtype=np.int64)
        if keep.size == 0:
            return {
                "status": "filtered_empty",
                "message": (
                    f"No valid boxes after bbox size filtering "
                    f"(min_w={min_bbox_w_abs:.1f}px, min_h={min_bbox_h_abs:.1f}px)."
                ),
            }

        filtered_rel_idx = cfg["filter_fn"](
            xyxy[keep],
            image_width=rgb.shape[1],
            column_margin_ratio=cfg["column_margin_ratio"],
            min_bbox_area_abs=cfg["min_bbox_area_abs"],
            max_bbox_area_abs=cfg.get("max_bbox_area_abs", 0.0),
            max_keep=cfg["max_keep_instances"],
            debug=cfg["filter_debug"],
            logger=rospy.loginfo,
        )
        if len(filtered_rel_idx) == 0:
            filter_name = "center-column/area" if cfg["mode"] == "single" else "area"
            return {
                "status": "filtered_empty",
                "message": f"No valid boxes after {filter_name} filtering.",
            }

        keep = keep[filtered_rel_idx]
        keep = self.filter_candidates_by_camera_depth(keep, xyxy, depth, cfg)
        if keep.size == 0:
            return {
                "status": "filtered_empty",
                "message": "No valid boxes after camera-depth prefiltering.",
            }
        t_filter_end = time.perf_counter()
        filter_elapsed = t_filter_end - t_det_start

        out_dir = self.prepare_output_dir(cfg)
        if out_dir is not None:
            self.save_common_outputs(out_dir, rgb, depth, xyxy, conf, keep, cfg)

        viz = rgb.copy()
        K = self.cam_info["intrinsics"]
        D = self.cam_info["distortion"]
        gdrn_time_sum = 0.0
        all_results = []

        rospy.loginfo(
            f"{COLOR_CYAN}[PIPELINE]{COLOR_RESET} {cfg['service_name']} pre-filter kept {len(keep)} boxes, run shared GDRN++ ..."
        )

        for idx in keep:
            x1, y1, x2, y2 = xyxy[idx]
            w, h = (x2 - x1), (y2 - y1)

            # --- multi-class: map YOLO class id → GDRN category_idx / obj_id ---
            yolo_cls_id = int(cls[idx]) if idx < len(cls) else 0
            # YOLO classes are 0-indexed; GDRN obj_ids are [1,2,3,4,5]
            # category_idx = index into self.gdrn_predictor.obj_ids
            try:
                category_idx = list(self.gdrn_predictor.obj_ids).index(yolo_cls_id + 1)
            except ValueError:
                rospy.logwarn(
                    f"[PIPELINE] inst#{idx}: YOLO cls_id={yolo_cls_id} "
                    f"not in predictor.obj_ids={self.gdrn_predictor.obj_ids}, skip"
                )
                continue
            obj_id = self.gdrn_predictor.obj_ids[category_idx]
            obj_name = self.gdrn_predictor.objs[obj_id]
            self.gdrn_predictor.obj_id = obj_id
            rospy.loginfo(
                f"[PIPELINE] inst#{idx}: YOLO cls={yolo_cls_id} → "
                f"category_idx={category_idx}, obj_id={obj_id}, obj_name={obj_name}"
            )

            arr = np.array(
                [[x1, y1, x1 + w + 5, y1 + h + 5, 1.0, 1.0, float(category_idx)]],
                dtype=np.float32,
            )
            arr = np.ascontiguousarray(arr)

            t1 = time.perf_counter()
            data_dict = self.gdrn_predictor.preprocessing(
                outputs=[torch.from_numpy(arr)],
                image=rgb,
                depth_img=depth,
            )
            out_dict = self.gdrn_predictor.inference(data_dict)
            est_pose = self.gdrn_predictor.postprocessing(data_dict, out_dict)
            if torch.cuda.is_available():
                torch.cuda.synchronize()
            t2 = time.perf_counter()
            gdrn_time_sum += t2 - t1
            rospy.loginfo(f"[TIME] GDRNPP est (inst#{idx}, {cfg['mode']}, {obj_name}): {t2 - t1:.3f}s")

            # Use dynamic obj_name key from multi-class predictor, fallback to first key for compat
            if obj_name in est_pose and est_pose[obj_name] is not None:
                T_cam_obj = np.array(est_pose[obj_name])
            else:
                rospy.logwarn(
                    f"[PIPELINE] inst#{idx}: obj_name='{obj_name}' not in est_pose keys={list(est_pose.keys())}, "
                    f"trying first available key"
                )
                first_key = next((k for k in est_pose if est_pose[k] is not None), None)
                if first_key is None:
                    rospy.logwarn(f"[PIPELINE] inst#{idx}: no valid est_pose entry, skip")
                    continue
                T_cam_obj = np.array(est_pose[first_key])
                obj_name = first_key  # update for downstream logging
            raw_t_cam = T_cam_obj[:3, 3].astype(float).copy()
            rospy.loginfo(
                f"{COLOR_CYAN}[CAM_POSE_NET_RAW]{COLOR_RESET} inst#{idx} "
                f"camera_frame={self.camera_frame} "
                f"t_cam=({raw_t_cam[0]:.3f}, {raw_t_cam[1]:.3f}, {raw_t_cam[2]:.3f})"
            )
            if np.linalg.norm(raw_t_cam) > 10.0:
                T_cam_obj[:3, 3] = raw_t_cam / 1000.0
                rospy.loginfo(
                    f"{COLOR_CYAN}[CAM_POSE_UNIT]{COLOR_RESET} inst#{idx} "
                    "translation looks like millimeters, converted to meters"
                )
            if cfg.get("use_depth_translation", False):
                depth_pose = self.estimate_depth_translation_from_bbox(
                    depth,
                    (x1, y1, x2, y2),
                    roi_ratio=cfg.get("depth_translation_roi_ratio", 0.50),
                )
                if depth_pose is not None:
                    depth_t_cam, depth_uv, depth_raw, depth_count = depth_pose
                    net_t_cam_m = T_cam_obj[:3, 3].astype(float).copy()
                    T_cam_obj[:3, 3] = depth_t_cam
                    rospy.loginfo(
                        f"{COLOR_CYAN}[CAM_POSE_DEPTH]{COLOR_RESET} inst#{idx} "
                        f"replace net_t=({net_t_cam_m[0]:.3f},{net_t_cam_m[1]:.3f},{net_t_cam_m[2]:.3f})m "
                        f"with depth_t=({depth_t_cam[0]:.3f},{depth_t_cam[1]:.3f},{depth_t_cam[2]:.3f})m "
                        f"uv=({depth_uv[0]:.1f},{depth_uv[1]:.1f}) raw_median={depth_raw:.1f} n={depth_count}"
                    )
                else:
                    rospy.logwarn(
                        f"{COLOR_CYAN}[CAM_POSE_DEPTH]{COLOR_RESET} inst#{idx}: no valid depth translation; keep network translation"
                    )
            checked_t_cam = T_cam_obj[:3, 3].astype(float)
            if (
                not np.all(np.isfinite(checked_t_cam))
                or np.linalg.norm(checked_t_cam) > 5.0
                or checked_t_cam[2] <= 0.05
                or checked_t_cam[2] > 5.0
            ):
                rospy.logwarn(
                    f"{COLOR_CYAN}[CAM_POSE_SANITY]{COLOR_RESET} inst#{idx}: "
                    f"reject implausible camera translation "
                    f"t=({checked_t_cam[0]:.3f}, {checked_t_cam[1]:.3f}, {checked_t_cam[2]:.3f})m"
                )
                continue
            rospy.loginfo(
                f"{COLOR_CYAN}[CAM_POSE_RAW]{COLOR_RESET} inst#{idx} "
                f"camera_frame={self.camera_frame} "
                f"t_cam=({T_cam_obj[0, 3]:.3f}, {T_cam_obj[1, 3]:.3f}, {T_cam_obj[2, 3]:.3f})"
            )
            qx, qy, qz, qw = quaternion_from_matrix(T_cam_obj)
            rospy.loginfo(
                f"{COLOR_GREEN}[CAM_POSE_6D]{COLOR_RESET} inst#{idx} "
                f"obj_id={obj_id} obj_name={obj_name} confidence={float(conf[idx]):.4f} "
                f"bbox_xyxy=({x1:.1f}, {y1:.1f}, {x2:.1f}, {y2:.1f}) "
                f"frame={self.camera_frame} "
                f"position_m=({T_cam_obj[0, 3]:.6f}, {T_cam_obj[1, 3]:.6f}, {T_cam_obj[2, 3]:.6f}) "
                f"quaternion_xyzw=({qx:.6f}, {qy:.6f}, {qz:.6f}, {qw:.6f})"
            )
            T_pose_frame_obj_est = self.transform_cam_to_frame(
                T_cam_obj, self.camera_frame, cfg["base_frame"]
            )
            R_pose_frame_est = T_pose_frame_obj_est[:3, :3]
            _, yaw_before, yaw_after, best_sym_mat = choose_best_symmetric_orientation(
                R_pose_frame_est, yaw_offset_deg=0.0, max_abs_deg=90.0
            )
            T_cam_obj_aligned = T_cam_obj.copy()
            T_cam_obj_aligned[:3, :3] = T_cam_obj_aligned[:3, :3].dot(best_sym_mat)

            T_base_link_obj = self.transform_cam_to_frame(
                T_cam_obj_aligned, self.camera_frame, BASE_LINK_FRAME
            )
            # Use base_link as the output frame directly
            T_base_obj = T_base_link_obj.copy()
            if cfg["base_frame"] != BASE_LINK_FRAME:
                T_base_obj = self.transform_cam_to_frame(
                    T_cam_obj_aligned, self.camera_frame, cfg["base_frame"]
                )

            T_cam_obj_aligned, T_base_obj, T_base_link_obj = self.apply_base_translation_offset(
                T_cam_obj_aligned, T_base_obj, T_base_link_obj, cfg
            )

            all_results.append(
                {
                    "idx": int(idx),
                    "T_cam_obj": T_cam_obj_aligned,
                    "T_base_obj": T_base_obj,
                    "T_base_link_obj": T_base_link_obj,
                    "yaw_before": yaw_before,
                    "yaw_after": yaw_after,
                    "data_dict": data_dict,
                    "out_dict": out_dict,
                    "obj_id": obj_id,
                    "obj_name": obj_name,
                }
            )

            viz = draw_axes_on_image(viz, T_cam_obj_aligned, K, D, axis_len_m=0.20)
            viz = draw_origin_point(viz, T_cam_obj_aligned, K, D, radius=6, color=(0, 255, 255))

            if out_dir is not None:
                try:
                    self.gdrn_predictor.gdrn_visualization(
                        batch=data_dict,
                        out_dict=out_dict,
                        image=rgb,
                        image_name=f"alignment_vis_{idx}",
                        gt_pose=None,
                        results_dir=out_dir,
                    )
                except Exception as e_vis:
                    rospy.logwarn(f"[VIS] alignment inst#{idx} failed: {e_vis}")
                    rospy.logwarn(traceback.format_exc())

            rospy.loginfo(
                f"{COLOR_GREEN}[CANDIDATE]{COLOR_RESET}[inst#{idx}] base t=("
                f"{T_base_obj[0,3]:.3f}, {T_base_obj[1,3]:.3f}, {T_base_obj[2,3]:.3f}) "
                f"yaw={math.degrees(yaw_after):.2f}deg"
            )

        if len(all_results) == 0:
            return {
                "status": "gdrn_empty",
                "message": "GDRNPP inference failed for all candidates.",
            }

        valid_results = self.apply_pose_reasonableness_check(all_results, cfg)
        valid_results = self.apply_position_filter(valid_results, cfg)
        if len(valid_results) == 0:
            return {
                "status": "filtered_empty",
                "message": "No valid pose after position sanity filtering.",
            }

        return {
            "status": "ok",
            "message": "ok",
            "xyxy": xyxy,
            "conf": conf,
            "keep": keep,
            "viz": viz,
            "out_dir": out_dir,
            "filter_elapsed": filter_elapsed,
            "gdrn_time_sum": gdrn_time_sum,
            "valid_results": valid_results,
        }

    def prepare_output_dir(self, cfg):
        if not self.save_outputs:
            return None
        self.prune_old_output_dirs(cfg["save_dir"], cfg.get("max_saved_runs", 35) - 1)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        out_dir = os.path.join(cfg["save_dir"], stamp)
        os.makedirs(out_dir, exist_ok=True)
        self.prune_old_output_dirs(cfg["save_dir"], cfg.get("max_saved_runs", 35))
        return out_dir

    def prune_old_output_dirs(self, parent_dir, max_dirs):
        if max_dirs is None:
            return
        max_dirs = int(max_dirs)
        if max_dirs < 0:
            return
        if not osp.isdir(parent_dir):
            return

        child_dirs = []
        for name in os.listdir(parent_dir):
            child_path = osp.join(parent_dir, name)
            if osp.isdir(child_path):
                child_dirs.append(child_path)

        if len(child_dirs) <= max_dirs:
            return

        child_dirs.sort(key=lambda path: osp.getmtime(path))
        to_remove = child_dirs[: len(child_dirs) - max_dirs]
        for old_dir in to_remove:
            try:
                shutil.rmtree(old_dir)
                rospy.loginfo(
                    f"{COLOR_CYAN}[LOG_CLEANUP]{COLOR_RESET} removed old output dir: {old_dir}"
                )
            except Exception as e:
                rospy.logwarn(
                    f"{COLOR_CYAN}[LOG_CLEANUP]{COLOR_RESET} failed to remove {old_dir}: {e}"
                )

    def save_common_outputs(self, out_dir, rgb, depth, xyxy, conf, keep, cfg):
        cv2.imwrite(osp.join(out_dir, "rgb_input.png"), rgb)
        if depth.dtype != np.uint16:
            cv2.imwrite(osp.join(out_dir, "depth_input.png"), depth.astype(np.uint16))
        else:
            cv2.imwrite(osp.join(out_dir, "depth_input.png"), depth)
        cv2.imwrite(osp.join(out_dir, "depth_colormap.png"), colorize_depth(depth))

        if cfg["min_bbox_area_abs"] > 0:
            ref_viz = rgb.copy()
            side = max(1, int(math.sqrt(cfg["min_bbox_area_abs"])))
            cx, cy = ref_viz.shape[1] // 2, ref_viz.shape[0] // 2
            x1 = max(cx - side // 2, 0)
            y1 = max(cy - side // 2, 0)
            x2 = min(x1 + side, ref_viz.shape[1] - 1)
            y2 = min(y1 + side, ref_viz.shape[0] - 1)
            cv2.rectangle(ref_viz, (x1, y1), (x2, y2), (255, 0, 255), 2)
            cv2.putText(
                ref_viz,
                f"min_bbox_area_abs={int(cfg['min_bbox_area_abs'])} px^2",
                (x1, max(y1 - 5, 0)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 0, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.imwrite(osp.join(out_dir, "ref_min_bbox_area_abs.jpg"), ref_viz)

        try:
            det_viz = rgb.copy()
            for idx, (x1, y1, x2, y2) in enumerate(xyxy):
                cv2.rectangle(det_viz, (int(x1), int(y1)), (int(x2), int(y2)), (0, 128, 255), 2)
                label = f"id:{idx} conf:{conf[idx]:.3f}"
                cv2.putText(
                    det_viz,
                    label,
                    (int(x1), max(int(y1) - 5, 0)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 128, 255),
                    2,
                    cv2.LINE_AA,
                )
            cv2.imwrite(osp.join(out_dir, "detection_result.jpg"), det_viz)
        except Exception:
            pass

        try:
            filtered_viz = rgb.copy()
            for idx in keep:
                x1, y1, x2, y2 = xyxy[idx]
                cv2.rectangle(filtered_viz, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 255), 2)
                cv2.putText(
                    filtered_viz,
                    f"id:{idx}",
                    (int(x1), max(int(y1) - 5, 0)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
            cv2.imwrite(osp.join(out_dir, "detection_filtered.jpg"), filtered_viz)
        except Exception:
            pass

    def apply_pose_reasonableness_check(self, all_results, cfg):
        rp_max_rad = math.radians(cfg["pose_rp_max_deg"])
        pose_check_frame = "base_link"
        T_baselink_cam = None
        try:
            tf_bl = self.tf_buffer.lookup_transform(
                target_frame=pose_check_frame,
                source_frame=self.camera_frame,
                time=rospy.Time(0),
                timeout=rospy.Duration(1.0),
            )
            t_bl = tf_bl.transform.translation
            q_bl = tf_bl.transform.rotation
            T_baselink_cam = quaternion_matrix([q_bl.x, q_bl.y, q_bl.z, q_bl.w])
            T_baselink_cam[0, 3] = t_bl.x
            T_baselink_cam[1, 3] = t_bl.y
            T_baselink_cam[2, 3] = t_bl.z
        except Exception as e_tf:
            rospy.logwarn(
                f"[POSE_CHECK] TF lookup {pose_check_frame}<-{self.camera_frame} failed: {e_tf}, fallback to {cfg['base_frame']}"
            )

        valid_results = []
        rospy.loginfo(f"{COLOR_CYAN}[POSE_CHECK]{COLOR_RESET} ========== pose check ({cfg['mode']}) ==========")
        rospy.loginfo(
            f"{COLOR_CYAN}[POSE_CHECK]{COLOR_RESET} max roll/pitch deviation: {cfg['pose_rp_max_deg']:.1f} deg"
        )

        for result in all_results:
            if T_baselink_cam is not None:
                T_bl_obj = T_baselink_cam.dot(result["T_cam_obj"])
            else:
                T_bl_obj = result["T_base_obj"]

            R_bl = T_bl_obj[:3, :3]
            roll = math.atan2(R_bl[2, 1], R_bl[2, 2])
            pitch = math.asin(np.clip(-R_bl[2, 0], -1.0, 1.0))
            roll_deg = math.degrees(roll)
            pitch_deg = math.degrees(pitch)

            if abs(roll) <= rp_max_rad and abs(pitch) <= rp_max_rad:
                valid_results.append(result)
                rospy.loginfo(
                    f"{COLOR_CYAN}[POSE_CHECK]{COLOR_RESET} inst#{result['idx']}: "
                    f"roll={roll_deg:.1f} pitch={pitch_deg:.1f} -> pass"
                )
            else:
                rospy.logwarn(
                    f"{COLOR_CYAN}[POSE_CHECK]{COLOR_RESET} inst#{result['idx']}: "
                    f"roll={roll_deg:.1f} pitch={pitch_deg:.1f} -> reject"
                )

        if len(valid_results) == 0:
            rospy.logwarn(f"{COLOR_CYAN}[POSE_CHECK]{COLOR_RESET} all candidates rejected, fallback to all results")
            return list(all_results)

        rospy.loginfo(
            f"{COLOR_CYAN}[POSE_CHECK]{COLOR_RESET} passed: {len(valid_results)}/{len(all_results)}"
        )
        return valid_results

    def apply_position_filter(self, results, cfg):
        if cfg.get("disable_position_filter", False):
            rospy.loginfo(
                f"{COLOR_CYAN}[POSITION_CHECK]{COLOR_RESET} disabled; keep {len(results)} candidates"
            )
            return list(results)

        if cfg["position_filter"] == "x_only":
            x_min = cfg["base_x_min"]
            x_max = cfg["base_x_max"]
            rospy.loginfo(f"{COLOR_CYAN}[X_CHECK]{COLOR_RESET} x in [{x_min:.3f}, {x_max:.3f}]m")
            filtered = []
            for result in results:
                bx = result["T_base_obj"][0, 3]
                if x_min <= bx <= x_max:
                    filtered.append(result)
                    rospy.loginfo(f"{COLOR_CYAN}[X_CHECK]{COLOR_RESET} inst#{result['idx']}: x={bx:.3f} -> pass")
                else:
                    rospy.logwarn(f"{COLOR_CYAN}[X_CHECK]{COLOR_RESET} inst#{result['idx']}: x={bx:.3f} -> reject")
            if len(filtered) == 0:
                rospy.logwarn(f"{COLOR_CYAN}[X_CHECK]{COLOR_RESET} all candidates out of range")
                return []
            return filtered

        x_min = cfg["base_x_min"]
        x_max = cfg["base_x_max"]
        y_min = cfg["base_y_min"]
        y_max = cfg["base_y_max"]
        rospy.loginfo(f"{COLOR_CYAN}[XY_CHECK]{COLOR_RESET} x in [{x_min:.3f}, {x_max:.3f}]m")
        rospy.loginfo(f"{COLOR_CYAN}[XY_CHECK]{COLOR_RESET} y in ({y_min:.3f}, {y_max:.3f})m")
        filtered = []
        for result in results:
            bx = result["T_base_obj"][0, 3]
            by = result["T_base_obj"][1, 3]
            if x_min <= bx <= x_max and y_min < by < y_max:
                filtered.append(result)
                rospy.loginfo(
                    f"{COLOR_CYAN}[XY_CHECK]{COLOR_RESET} inst#{result['idx']}: x={bx:.3f} y={by:.3f} -> pass"
                )
            else:
                rospy.logwarn(
                    f"{COLOR_CYAN}[XY_CHECK]{COLOR_RESET} inst#{result['idx']}: x={bx:.3f} y={by:.3f} -> reject"
                )
        if len(filtered) == 0:
            rospy.logwarn(f"{COLOR_CYAN}[XY_CHECK]{COLOR_RESET} all candidates out of range")
            return []
        return filtered

    def select_top_layer(self, results, layer_z_threshold):
        base_z_values = np.array([r["T_base_obj"][2, 3] for r in results], dtype=np.float32)
        max_z = float(np.max(base_z_values))
        rospy.loginfo(f"{COLOR_CYAN}[SELECT]{COLOR_RESET} base_z values: {[f'{z:.3f}' for z in base_z_values]}")
        rospy.loginfo(
            f"{COLOR_CYAN}[SELECT]{COLOR_RESET} max_z={max_z:.3f}m, layer_z_threshold={layer_z_threshold:.3f}m"
        )

        top_layer_results = []
        for result in results:
            z_val = result["T_base_obj"][2, 3]
            if z_val >= max_z - layer_z_threshold:
                top_layer_results.append(result)
                rospy.loginfo(f"{COLOR_CYAN}[SELECT]{COLOR_RESET} inst#{result['idx']}: z={z_val:.3f} -> top")
            else:
                rospy.loginfo(
                    f"{COLOR_CYAN}[SELECT]{COLOR_RESET} inst#{result['idx']}: z={z_val:.3f} -> non-top"
                )

        if len(top_layer_results) == 0:
            return list(results), max_z, base_z_values
        return top_layer_results, max_z, base_z_values

    def finish_service_response(self, cfg, res, pipeline_data, t_service_start):
        status = pipeline_data["status"]
        if status != "ok":
            self.clear_pose_group(cfg["pose_group"])
            res.success = False
            res.message = pipeline_data["message"]
            res.num_instances = 0
            return res

        if cfg["mode"] == "single":
            return self.finish_single_response(cfg, res, pipeline_data, t_service_start)
        return self.finish_top_response(cfg, res, pipeline_data, t_service_start)

    def finish_single_response(self, cfg, res, pipeline_data, t_service_start):
        valid_results = pipeline_data["valid_results"]
        xyxy = pipeline_data["xyxy"]
        viz = pipeline_data["viz"]
        out_dir = pipeline_data["out_dir"]

        top_layer_results, _, _ = self.select_top_layer(valid_results, cfg["layer_z_threshold"])
        best_result = None
        best_dist = float("inf")
        for result in top_layer_results:
            bx = result["T_base_obj"][0, 3]
            by = result["T_base_obj"][1, 3]
            dist_xy = math.sqrt(bx ** 2 + by ** 2)
            rospy.loginfo(
                f"{COLOR_CYAN}[SELECT]{COLOR_RESET} inst#{result['idx']}: base_xy_dist={dist_xy:.3f}m (x={bx:.3f}, y={by:.3f})"
            )
            if dist_xy < best_dist:
                best_dist = dist_xy
                best_result = result

        chosen = best_result
        chosen_idx = chosen["idx"]
        T_cfg_frame_obj = chosen["T_base_obj"]
        T_base_link_obj = chosen["T_base_link_obj"]
        yaw_before = chosen["yaw_before"]
        yaw_after = chosen["yaw_after"]

        self.set_pose_group(
            cfg["pose_group"],
            {f"basket_{chosen_idx}_base": (T_cfg_frame_obj, cfg["base_frame"])}
        )

        res.poses_base_link = []
        res.poses_camera_link = []
        res.bbox_xyxy = []
        res.yaw = []

        x1, y1, x2, y2 = xyxy[chosen_idx]
        res.bbox_xyxy.extend([float(x1), float(y1), float(x2), float(y2)])

        pose_base_link = self.matrix_to_pose(T_base_link_obj)
        pose_camera_link = self.matrix_to_pose(chosen["T_cam_obj"])
        res.poses_base_link.append(pose_base_link)
        res.poses_camera_link.append(pose_camera_link)
        res.yaw.append(float(yaw_after))

        rospy.loginfo(
            f"{COLOR_GREEN}[RESULT]{COLOR_RESET}[CHOSEN inst#{chosen_idx}] "
            f"{self.camera_frame} t=({pose_camera_link.position.x:.3f},{pose_camera_link.position.y:.3f},{pose_camera_link.position.z:.3f})"
        )
        rospy.loginfo(
            f"{COLOR_GREEN}[RESULT]{COLOR_RESET}[CHOSEN inst#{chosen_idx}] "
            f"base_link t=({pose_base_link.position.x:.3f},{pose_base_link.position.y:.3f},{pose_base_link.position.z:.3f})"
        )
        rospy.loginfo(
            f"{COLOR_GREEN}[RESULT]{COLOR_RESET}[CHOSEN inst#{chosen_idx}] "
            f"yaw_before={math.degrees(yaw_before):.2f}deg, yaw_after={math.degrees(yaw_after):.2f}deg"
        )

        # --- publish as pseudo-AprilTag for QR/AprilTag workflow compatibility ---
        self._publish_april_tag_result(chosen, pose_camera_link)

        cv2.rectangle(viz, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 3)
        cv2.putText(
            viz,
            f"CHOSEN #{chosen_idx}",
            (int(x1), max(int(y1) - 10, 0)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

        if out_dir is not None:
            cv2.imwrite(osp.join(out_dir, "viz_multi.png"), viz)

        # Publish visualization image on /basket_vision/viz_image
        self._publish_viz_image(viz)

        t_pipeline_end = time.perf_counter()
        rospy.loginfo(
            f"{COLOR_YELLOW}[TIME]{COLOR_RESET} mode=single "
            f"yolo+filter={pipeline_data['filter_elapsed']:.3f}s "
            f"gdrn_total={pipeline_data['gdrn_time_sum']:.3f}s "
            f"service_total={t_pipeline_end - t_service_start:.3f}s "
            f"(candidates={len(pipeline_data['keep'])}, chosen=inst#{chosen_idx})"
        )

        res.num_instances = 1
        res.success = True
        res.message = f"Chosen inst#{chosen_idx} from {len(pipeline_data['keep'])} candidates."
        return res

    def finish_top_response(self, cfg, res, pipeline_data, t_service_start):
        returned_results = pipeline_data["valid_results"]
        xyxy = pipeline_data["xyxy"]
        viz = pipeline_data["viz"]
        out_dir = pipeline_data["out_dir"]

        top_layer_results, _, _ = self.select_top_layer(returned_results, cfg["layer_z_threshold"])

        # Free-form placement: no slot matching, use instance indices directly
        rospy.loginfo(f"{COLOR_CYAN}[FREE_FORM]{COLOR_RESET} ========== free-form top-layer ({len(top_layer_results)} instances) ==========")
        for r in top_layer_results:
            rospy.loginfo(
                f"{COLOR_CYAN}[FREE_FORM]{COLOR_RESET} inst#{r['idx']}: "
                f"base_xy=({r['T_base_obj'][0,3]:.3f},{r['T_base_obj'][1,3]:.3f})"
            )
        top_slot_ids = [int(r["idx"]) for r in top_layer_results]
        count_info = estimate_total_basket_count(
            top_layer_results=top_layer_results,
            box_height_m=cfg["box_height_m"],
            pallet_height_m=cfg["pallet_height_m"],
            logger=rospy.loginfo,
            max_layers=cfg["max_layers"],
            full_layer_box_count=cfg["full_layer_box_count"],
        )

        if cfg.get("return_single_target", True) and returned_results:
            selectable_results = top_layer_results if top_layer_results else returned_results
            chosen_single = min(
                selectable_results,
                key=lambda item: abs(float(item["T_base_link_obj"][1, 3])),
            )
            returned_results = [chosen_single]
            top_layer_results = [chosen_single]
            top_slot_ids = [int(chosen_single["idx"])]
            rospy.loginfo(
                f"{COLOR_GREEN}[SELECT]{COLOR_RESET} return_single_target enabled: "
                f"chosen inst#{chosen_single['idx']} by min |base_y|="
                f"{abs(float(chosen_single['T_base_link_obj'][1, 3])):.3f}m"
            )

        new_poses = {}
        for chosen in top_layer_results:
            chosen_idx = chosen["idx"]
            T_cfg_frame_obj = chosen["T_base_obj"]
            tf_name = f"basket_{chosen_idx}_base"
            new_poses[tf_name] = (T_cfg_frame_obj, cfg["base_frame"])
        self.set_pose_group(cfg["pose_group"], new_poses)

        res.poses_base_link = []
        res.poses_camera_link = []
        res.bbox_xyxy = []
        res.yaw = []

        for chosen in returned_results:
            chosen_idx = chosen["idx"]
            T_base_link_obj = chosen["T_base_link_obj"]
            yaw_before = chosen["yaw_before"]
            yaw_after = chosen["yaw_after"]
            x1, y1, x2, y2 = xyxy[chosen_idx]

            res.bbox_xyxy.extend([float(x1), float(y1), float(x2), float(y2)])
            pose_base_link = self.matrix_to_pose(T_base_link_obj)
            pose_camera_link = self.matrix_to_pose(chosen["T_cam_obj"])
            res.poses_base_link.append(pose_base_link)
            res.poses_camera_link.append(pose_camera_link)
            res.yaw.append(float(yaw_after))

            rospy.loginfo(
                f"{COLOR_GREEN}[RESULT]{COLOR_RESET}[RETURN inst#{chosen_idx}] "
                f"{self.camera_frame} t=({pose_camera_link.position.x:.3f},{pose_camera_link.position.y:.3f},{pose_camera_link.position.z:.3f})"
            )
            rospy.loginfo(
                f"{COLOR_GREEN}[RESULT]{COLOR_RESET}[RETURN inst#{chosen_idx}] "
                f"base_link t=({pose_base_link.position.x:.3f},{pose_base_link.position.y:.3f},{pose_base_link.position.z:.3f})"
            )
            rospy.loginfo(
                f"{COLOR_GREEN}[RESULT]{COLOR_RESET}[RETURN inst#{chosen_idx}] "
                f"yaw_before={math.degrees(yaw_before):.2f}deg, yaw_after={math.degrees(yaw_after):.2f}deg"
            )

            cv2.rectangle(viz, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 255), 2)
            cv2.putText(
                viz,
                f"RETURN #{chosen_idx}",
                (int(x1), max(int(y1) - 10, 0)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )

        # --- publish returned_results as pseudo-AprilTag batch ---
        self._publish_april_tag_batch(returned_results, res.poses_camera_link)

        for chosen in top_layer_results:
            chosen_idx = chosen["idx"]
            x1, y1, x2, y2 = xyxy[chosen_idx]
            cv2.rectangle(viz, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 3)
            top_text = f"FREE #{chosen_idx}"
            cv2.putText(
                viz,
                top_text,
                (int(x1), min(int(y2) + 25, viz.shape[0] - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

        count_text = (
            f"TOTAL={count_info['total_count']}  "
            f"LAYERS={count_info['layer_count']}  "
            f"TOP={count_info['top_count']}"
        )
        cv2.putText(
            viz,
            count_text,
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        if out_dir is not None:
            cv2.imwrite(osp.join(out_dir, "viz_multi.png"), viz)

        # Publish visualization image on /basket_vision/viz_image
        self._publish_viz_image(viz)

        t_pipeline_end = time.perf_counter()
        rospy.loginfo(
            f"{COLOR_YELLOW}[TIME]{COLOR_RESET} mode=top "
            f"yolo+filter={pipeline_data['filter_elapsed']:.3f}s "
            f"gdrn_total={pipeline_data['gdrn_time_sum']:.3f}s "
            f"service_total={t_pipeline_end - t_service_start:.3f}s "
            f"(candidates={len(pipeline_data['keep'])}, returned={len(returned_results)}, top={top_slot_ids})"
        )

        res.num_instances = len(returned_results)
        res.success = True
        basket_type = self.get_basket_type(cfg.get("box_name"))
        res.message = (
            f"basket_type={basket_type}; "
            f"Returned {len(returned_results)} instances from {len(pipeline_data['keep'])} candidates; "
            f"top ids: {top_slot_ids}; "
            f"estimated total baskets={count_info['total_count']} "
            f"(layers={count_info['layer_count']}, top={count_info['top_count']})."
        )
        return res

    def _apply_april_tag_transform(self, result, cam_pose_msg):
        """
        Apply the AprilTag-compatible pose transformation pipeline to a single result.
        Pipeline: extract pos/quat → correct_box_pose → apply_vision_to_tag_offset

        Args:
            result: result dict with T_cam_obj, obj_id, obj_name
            cam_pose_msg: Pose message (before transform, for fallback)
        Returns:
            [x, y, z, qx, qy, qz, qw] in camera frame (AprilTag frame)
        """
        try:
            T_cam_obj = result["T_cam_obj"]
            pose_arr = transform_to_pose(T_cam_obj)
            pos = pose_arr[:3]
            quat = pose_arr[3:]

            corrected_quat = correct_box_pose(pos, quat)
            final_quat = apply_vision_to_tag_offset(corrected_quat, angle_deg=-90)

            return np.concatenate((pos, final_quat))
        except Exception as e:
            rospy.logwarn(f"[APRILTAG] transform failed for inst#{result.get('idx', '?')}: {e}")
            # fallback: use the raw camera pose
            return np.array([
                cam_pose_msg.position.x, cam_pose_msg.position.y, cam_pose_msg.position.z,
                cam_pose_msg.orientation.x, cam_pose_msg.orientation.y,
                cam_pose_msg.orientation.z, cam_pose_msg.orientation.w,
            ])

    def _publish_april_tag_result(self, result, cam_pose_msg):
        """Apply AprilTag transform to a single chosen result and publish."""
        try:
            april_pose = self._apply_april_tag_transform(result, cam_pose_msg)
            obj_id = result.get("obj_id", 1)
            obj_name = result.get("obj_name", "unknown")
            data_id = 601
            status_flag = int(obj_id) - 1

            publish_april_tag_detections(
                publisher=self.tag_publisher,
                poses_list=[april_pose],
                stamp=rospy.Time.now(),
                obj_ids_list=[int(obj_id)],
                obj_names_list=[str(obj_name)],
                data_ids_list=[data_id],
                status_flags_list=[status_flag],
                camera_frame=self.camera_frame,
            )
        except Exception as e:
            rospy.logwarn(f"[APRILTAG] publish_single failed: {e}")

    def _publish_april_tag_batch(self, returned_results, cam_poses):
        """Apply AprilTag transform to all returned results and publish as batch."""
        if not returned_results:
            return
        try:
            poses_list = []
            obj_ids_list = []
            obj_names_list = []
            data_ids_list = []
            status_flags_list = []

            for i, result in enumerate(returned_results):
                obj_id = result.get("obj_id", 1)
                obj_name = result.get("obj_name", "unknown")
                data_id = 601 + i
                status_flag = int(obj_id) - 1

                # Use camera pose from the response list if available, else from result
                if i < len(cam_poses):
                    cam_pose_msg = cam_poses[i]
                    april_pose = self._apply_april_tag_transform(result, cam_pose_msg)
                else:
                    april_pose = transform_to_pose(result["T_cam_obj"])

                poses_list.append(april_pose)
                obj_ids_list.append(int(obj_id))
                obj_names_list.append(str(obj_name))
                data_ids_list.append(data_id)
                status_flags_list.append(status_flag)

            publish_april_tag_detections(
                publisher=self.tag_publisher,
                poses_list=poses_list,
                stamp=rospy.Time.now(),
                obj_ids_list=obj_ids_list,
                obj_names_list=obj_names_list,
                data_ids_list=data_ids_list,
                status_flags_list=status_flags_list,
                camera_frame=self.camera_frame,
            )
        except Exception as e:
            rospy.logwarn(f"[APRILTAG] publish_batch failed: {e}")

    def matrix_to_pose(self, T):
        pose = Pose()
        pose.position.x = float(T[0, 3])
        pose.position.y = float(T[1, 3])
        pose.position.z = float(T[2, 3])
        qx, qy, qz, qw = quaternion_from_matrix(T)
        pose.orientation.x = float(qx)
        pose.orientation.y = float(qy)
        pose.orientation.z = float(qz)
        pose.orientation.w = float(qw)
        return pose

    def _publish_viz_image(self, viz_bgr):
        """Publish a BGR visualization image on /basket_vision/viz_image."""
        try:
            if viz_bgr is None:
                return
            viz_msg = self.bridge.cv2_to_imgmsg(viz_bgr, encoding="bgr8")
            viz_msg.header.stamp = rospy.Time.now()
            viz_msg.header.frame_id = self.camera_frame
            self.viz_publisher.publish(viz_msg)
        except Exception as e:
            rospy.logwarn(f"[VIZ_PUB] failed to publish viz image: {e}")

    def publish_pose_and_tf(self, T_parent_child, frame, tf_child):
        pose = self.matrix_to_pose(T_parent_child)
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = frame
        transform.child_frame_id = tf_child
        transform.transform.translation.x = pose.position.x
        transform.transform.translation.y = pose.position.y
        transform.transform.translation.z = pose.position.z
        transform.transform.rotation.x = pose.orientation.x
        transform.transform.rotation.y = pose.orientation.y
        transform.transform.rotation.z = pose.orientation.z
        transform.transform.rotation.w = pose.orientation.w
        self.tf_broadcaster.sendTransform(transform)

    def transform_cam_to_frame(self, T_cam_obj, camera_frame, target_frame):
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                target_frame=target_frame,
                source_frame=camera_frame,
                time=rospy.Time(0),
                timeout=rospy.Duration(1.0),
            )
            trans = tf_msg.transform.translation
            quat = tf_msg.transform.rotation
            T_base_cam = quaternion_matrix([quat.x, quat.y, quat.z, quat.w])
            T_base_cam[0, 3] = trans.x
            T_base_cam[1, 3] = trans.y
            T_base_cam[2, 3] = trans.z
            return T_base_cam.dot(T_cam_obj)
        except Exception as e:
            msg = "[TF] lookup {}<-{} failed: {}".format(target_frame, camera_frame, e)
            rospy.logerr(msg)
            raise RuntimeError(msg)


if __name__ == "__main__":
    try:
        node = SharedBasketPoseServiceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
