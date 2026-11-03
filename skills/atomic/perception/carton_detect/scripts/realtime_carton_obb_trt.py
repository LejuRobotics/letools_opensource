#!/usr/bin/env python3
# --enable-base-servo --servo-kp 0.3 --servo-max-yaw-rate 4 --servo-max-runtime 15
"""Visualize shared ROS-service OBB inference on the head-camera stream."""

import argparse
import json
import math
import threading
import time
import urllib.request
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image


DEFAULT_TOPIC = "/camera/color/image_raw"
DEFAULT_DEPTH_TOPIC = "/camera/depth/image_raw"
DEFAULT_INFERENCE_SERVICE = "/carton_obb/infer"
NAVIGATION_CMD_VEL_TOPIC = "/move_base/base_cmd_vel"
WINDOW_NAME = "Carton OBB - TensorRT FP16"


class LatestFrameSubscriber:
    def __init__(self, topic):
        self.lock = threading.Lock()
        self.condition = threading.Condition(self.lock)
        self.ready = threading.Event()
        self.frame = None
        self.stamp = 0.0
        self.stamp_ns = 0
        self.sequence = 0
        self.error = None
        self.history = deque(maxlen=30)
        self.subscriber = rospy.Subscriber(
            topic,
            Image,
            self._callback,
            queue_size=1,
            buff_size=16 * 1024 * 1024,
        )

    def _callback(self, message):
        try:
            frame = ros_image_to_bgr(message)
        except Exception as error:
            with self.lock:
                self.error = error
            self.ready.set()
            return

        stamp = message.header.stamp
        if stamp == rospy.Time():
            stamp = rospy.Time.now()
        with self.condition:
            self.frame = frame
            self.stamp = stamp.to_sec()
            self.stamp_ns = stamp.to_nsec()
            self.sequence += 1
            self.error = None
            self.history.append((self.stamp_ns, frame))
            self.condition.notify_all()
        self.ready.set()

    def wait_for_first_frame(self, timeout):
        if not self.ready.wait(timeout):
            raise TimeoutError(
                f"No image received within {timeout:.1f} seconds"
            )
        with self.lock:
            if self.error is not None:
                raise RuntimeError(f"Failed to decode camera image: {self.error}")

    def latest(self):
        with self.lock:
            if self.error is not None:
                raise RuntimeError(f"Failed to decode camera image: {self.error}")
            if self.frame is None:
                return None, self.sequence, self.stamp
            return self.frame.copy(), self.sequence, self.stamp

    def frame_for_stamp(self, stamp_ns, timeout=0.1):
        deadline = time.monotonic() + timeout
        with self.condition:
            while True:
                for candidate_stamp_ns, candidate_frame in reversed(
                    self.history
                ):
                    if candidate_stamp_ns == stamp_ns:
                        return candidate_frame.copy()
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self.condition.wait(remaining)

    def close(self):
        self.subscriber.unregister()


class LatestDepthSubscriber:
    def __init__(self, topic):
        self.lock = threading.Lock()
        self.ready = threading.Event()
        self.depth_m = None
        self.stamp = 0.0
        self.error = None
        self.subscriber = rospy.Subscriber(
            topic,
            Image,
            self._callback,
            queue_size=1,
            buff_size=8 * 1024 * 1024,
        )

    def _callback(self, message):
        try:
            depth_m = ros_depth_to_meters(message)
        except Exception as error:
            with self.lock:
                self.error = error
            self.ready.set()
            return

        with self.lock:
            self.depth_m = depth_m
            self.stamp = message.header.stamp.to_sec() or rospy.get_time()
            self.error = None
        self.ready.set()

    def wait_for_first_frame(self, timeout):
        if not self.ready.wait(timeout):
            raise TimeoutError(
                f"No depth image received within {timeout:.1f} seconds"
            )
        with self.lock:
            if self.error is not None:
                raise RuntimeError(f"Failed to decode depth image: {self.error}")

    def latest(self):
        with self.lock:
            if self.error is not None:
                raise RuntimeError(f"Failed to decode depth image: {self.error}")
            if self.depth_m is None:
                return None, self.stamp
            return self.depth_m.copy(), self.stamp

    def close(self):
        self.subscriber.unregister()


def ros_image_to_bgr(message):
    encodings = {
        "rgb8": (3, cv2.COLOR_RGB2BGR),
        "bgr8": (3, None),
        "rgba8": (4, cv2.COLOR_RGBA2BGR),
        "bgra8": (4, cv2.COLOR_BGRA2BGR),
        "mono8": (1, cv2.COLOR_GRAY2BGR),
    }
    if message.encoding not in encodings:
        raise ValueError(f"Unsupported ROS image encoding: {message.encoding}")

    channels, conversion = encodings[message.encoding]
    rows = np.frombuffer(message.data, dtype=np.uint8).reshape(
        message.height,
        message.step,
    )
    image = rows[:, : message.width * channels]
    if channels == 1:
        image = image.reshape(message.height, message.width)
    else:
        image = image.reshape(message.height, message.width, channels)
    if conversion is not None:
        image = cv2.cvtColor(image, conversion)
    return np.ascontiguousarray(image)


def ros_depth_to_meters(message):
    encodings = {
        "16UC1": (np.uint16, 0.001),
        "mono16": (np.uint16, 0.001),
        "32FC1": (np.float32, 1.0),
    }
    if message.encoding not in encodings:
        raise ValueError(
            f"Unsupported depth encoding: {message.encoding}; "
            "expected 16UC1 or 32FC1"
        )

    dtype, scale = encodings[message.encoding]
    dtype = np.dtype(dtype).newbyteorder(">" if message.is_bigendian else "<")
    item_size = dtype.itemsize
    values_per_row = message.step // item_size
    rows = np.frombuffer(message.data, dtype=dtype).reshape(
        message.height,
        values_per_row,
    )
    depth = rows[:, : message.width].astype(np.float32)
    depth *= scale
    depth[~np.isfinite(depth)] = 0.0
    return np.ascontiguousarray(depth)


class RosObbInferenceClient:
    """Request inference on the service's latest head-camera frame."""

    def __init__(self, service_name, timeout):
        from box_detection_msgs.srv import InferCartonObb

        rospy.wait_for_service(service_name, timeout=timeout)
        self.service_name = service_name
        self.proxy = rospy.ServiceProxy(
            service_name,
            InferCartonObb,
            persistent=True,
        )

    def infer(self, confidence, nms_iou):
        try:
            response = self.proxy(
                confidence,
                nms_iou,
            )
        except rospy.ServiceException:
            self.proxy.close()
            raise
        if not response.success:
            raise RuntimeError(
                f"OBB inference service failed: {response.message}"
            )

        fields = (
            response.center_x,
            response.center_y,
            response.width,
            response.height,
            response.angle_deg,
            response.scores,
        )
        lengths = {len(field) for field in fields}
        if len(lengths) != 1:
            raise RuntimeError(
                "OBB inference service returned mismatched array lengths"
            )
        if response.source_header.stamp == rospy.Time():
            raise RuntimeError(
                "OBB inference service returned an empty source timestamp"
            )
        if response.image_width <= 0 or response.image_height <= 0:
            raise RuntimeError(
                "OBB inference service returned an invalid image size"
            )

        detections = []
        for values in zip(*fields):
            center_x, center_y, width, height, angle_deg, score = values
            rect = (
                (float(center_x), float(center_y)),
                (float(width), float(height)),
                float(angle_deg),
            )
            detections.append((rect, float(score)))
        return (
            detections,
            float(response.inference_ms),
            response.source_header.stamp.to_sec(),
            response.source_header.stamp.to_nsec(),
            (int(response.image_height), int(response.image_width)),
        )

    def close(self):
        self.proxy.close()


def sample_obb_depth(
    depth_m,
    rect,
    color_shape,
    shrink_ratio,
    min_depth_m,
    max_depth_m,
    mad_scale,
    outlier_floor_m,
    min_samples,
):
    color_height, color_width = color_shape[:2]
    depth_height, depth_width = depth_m.shape
    box_points = cv2.boxPoints(rect).astype(np.float32)
    center = np.asarray(rect[0], dtype=np.float32)
    inner_points = center + (box_points - center) * shrink_ratio
    inner_points[:, 0] *= depth_width / color_width
    inner_points[:, 1] *= depth_height / color_height
    inner_points = np.rint(inner_points).astype(np.int32)

    x_min = max(0, int(inner_points[:, 0].min()))
    x_max = min(depth_width - 1, int(inner_points[:, 0].max()))
    y_min = max(0, int(inner_points[:, 1].min()))
    y_max = min(depth_height - 1, int(inner_points[:, 1].max()))
    if x_min > x_max or y_min > y_max:
        return None

    local_points = inner_points - np.array([x_min, y_min], dtype=np.int32)
    mask = np.zeros((y_max - y_min + 1, x_max - x_min + 1), dtype=np.uint8)
    cv2.fillConvexPoly(mask, local_points, 255)
    depth_roi = depth_m[y_min : y_max + 1, x_min : x_max + 1]
    samples = depth_roi[mask != 0]
    samples = samples[
        np.isfinite(samples)
        & (samples >= min_depth_m)
        & (samples <= max_depth_m)
    ]
    if samples.size < min_samples:
        return None

    initial_median = float(np.median(samples))
    absolute_deviation = np.abs(samples - initial_median)
    mad = float(np.median(absolute_deviation))
    robust_sigma = 1.4826 * mad
    outlier_threshold = max(outlier_floor_m, mad_scale * robust_sigma)
    inliers = samples[absolute_deviation <= outlier_threshold]
    if inliers.size < min_samples:
        return None
    return float(np.median(inliers))


def normalize_scores(values, higher_is_better):
    values = np.asarray(values, dtype=np.float32)
    value_min = float(values.min())
    value_max = float(values.max())
    if value_max - value_min < 1e-6:
        return np.ones_like(values)
    normalized = (values - value_min) / (value_max - value_min)
    return normalized if higher_is_better else 1.0 - normalized


def select_best_obb(
    detections,
    depth_m,
    color_shape,
    shrink_ratio,
    min_depth_m,
    max_depth_m,
    mad_scale,
    outlier_floor_m,
    min_samples,
    depth_weight,
    bottom_center_weight,
    area_weight,
):
    depth_readings = []
    valid_detections = []
    score_details = [None] * len(detections)
    color_height, color_width = color_shape[:2]
    bottom_center = np.array(
        [color_width / 2.0, float(color_height)],
        dtype=np.float32,
    )

    for index, (rect, confidence) in enumerate(detections):
        distance_m = sample_obb_depth(
            depth_m,
            rect,
            color_shape,
            shrink_ratio,
            min_depth_m,
            max_depth_m,
            mad_scale,
            outlier_floor_m,
            min_samples,
        )
        depth_readings.append(distance_m)
        if distance_m is not None:
            center = np.asarray(rect[0], dtype=np.float32)
            bottom_center_distance = float(np.linalg.norm(center - bottom_center))
            area = float(rect[1][0] * rect[1][1])
            valid_detections.append(
                {
                    "index": index,
                    "rect": rect,
                    "confidence": confidence,
                    "distance_m": distance_m,
                    "bottom_center_distance": bottom_center_distance,
                    "area": area,
                }
            )

    if not valid_detections:
        return [], depth_readings, score_details, []

    depth_scores = normalize_scores(
        [item["distance_m"] for item in valid_detections],
        higher_is_better=False,
    )
    bottom_center_scores = normalize_scores(
        [item["bottom_center_distance"] for item in valid_detections],
        higher_is_better=False,
    )
    area_scores = normalize_scores(
        [item["area"] for item in valid_detections],
        higher_is_better=True,
    )
    weight_sum = depth_weight + bottom_center_weight + area_weight
    normalized_weights = (
        depth_weight / weight_sum,
        bottom_center_weight / weight_sum,
        area_weight / weight_sum,
    )

    scored_detections = []
    for item, depth_score, bottom_score, area_score in zip(
        valid_detections,
        depth_scores,
        bottom_center_scores,
        area_scores,
    ):
        total_score = (
            normalized_weights[0] * float(depth_score)
            + normalized_weights[1] * float(bottom_score)
            + normalized_weights[2] * float(area_score)
        )
        scored_detection = (
            item["rect"],
            item["confidence"],
            item["distance_m"],
            total_score,
            float(depth_score),
            float(bottom_score),
            float(area_score),
        )
        scored_detections.append(scored_detection)
        score_details[item["index"]] = scored_detection[3:]

    best = max(scored_detections, key=lambda detection: detection[3])
    return [best], depth_readings, score_details, scored_detections


def horizontal_rotation_deg(rect):
    angle = (float(rect[2]) + 90.0) % 180.0 - 90.0
    if angle > 45.0:
        angle -= 90.0
    elif angle <= -45.0:
        angle += 90.0
    return angle


def classify_carton_orientation(rect, aspect_threshold=1.10):
    """Classify the selected OBB as horizontal/vertical in image space."""
    width = float(rect[1][0])
    height = float(rect[1][1])
    if not (
        math.isfinite(width)
        and math.isfinite(height)
        and width > 0.0
        and height > 0.0
    ):
        return "unknown"

    aspect_ratio = max(width, height) / max(min(width, height), 1e-6)
    if aspect_ratio < float(aspect_threshold):
        return "unknown"

    long_axis_angle = float(rect[2]) if width >= height else float(rect[2]) + 90.0
    long_axis_angle = (long_axis_angle + 90.0) % 180.0 - 90.0
    return "horizontal" if abs(long_axis_angle) <= 45.0 else "vertical"


def compute_servo_yaw_deg_s(
    angle_error_deg,
    gain,
    deadband_deg,
    max_yaw_rate_deg_s,
):
    """Map the observed OBB angle to the empirically verified chassis sign."""
    if abs(angle_error_deg) <= deadband_deg:
        return 0.0
    return float(
        np.clip(
            gain * angle_error_deg,
            -max_yaw_rate_deg_s,
            max_yaw_rate_deg_s,
        )
    )


class CameraPointProjector:
    """Project a color pixel and aligned depth into the robot base frame."""

    def __init__(self, camera_info_topic, base_frame, timeout):
        import tf2_ros
        from sensor_msgs.msg import CameraInfo

        camera_info = rospy.wait_for_message(
            camera_info_topic,
            CameraInfo,
            timeout=timeout,
        )
        self.fx = float(camera_info.K[0])
        self.fy = float(camera_info.K[4])
        self.cx = float(camera_info.K[2])
        self.cy = float(camera_info.K[5])
        self.camera_frame = camera_info.header.frame_id
        self.base_frame = base_frame
        if self.fx <= 0.0 or self.fy <= 0.0 or not self.camera_frame:
            raise RuntimeError("Invalid color-camera calibration")

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_buffer.lookup_transform(
            self.base_frame,
            self.camera_frame,
            rospy.Time(0),
            rospy.Duration(timeout),
        )

    @staticmethod
    def _rotation_matrix(quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w
        return np.array(
            [
                [
                    1.0 - 2.0 * (y * y + z * z),
                    2.0 * (x * y - z * w),
                    2.0 * (x * z + y * w),
                ],
                [
                    2.0 * (x * y + z * w),
                    1.0 - 2.0 * (x * x + z * z),
                    2.0 * (y * z - x * w),
                ],
                [
                    2.0 * (x * z - y * w),
                    2.0 * (y * z + x * w),
                    1.0 - 2.0 * (x * x + y * y),
                ],
            ],
            dtype=np.float64,
        )

    def project(self, center, depth_m, stamp):
        u, v = center
        point_camera = np.array(
            [
                (float(u) - self.cx) * depth_m / self.fx,
                (float(v) - self.cy) * depth_m / self.fy,
                depth_m,
            ],
            dtype=np.float64,
        )
        query_time = rospy.Time.from_sec(stamp) if stamp > 0.0 else rospy.Time(0)
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.camera_frame,
                query_time,
                rospy.Duration(0.08),
            )
        except Exception:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.camera_frame,
                rospy.Time(0),
                rospy.Duration(0.08),
            )

        translation = transform.transform.translation
        offset = np.array(
            [translation.x, translation.y, translation.z],
            dtype=np.float64,
        )
        rotation = self._rotation_matrix(transform.transform.rotation)
        return rotation.dot(point_camera) + offset


class LockedObbTracker:
    """Keep following the initially selected carton instead of rescoring targets."""

    def __init__(self, max_center_jump_px, max_depth_jump_m):
        self.max_center_jump_px = max_center_jump_px
        self.max_depth_jump_m = max_depth_jump_m
        self.last_rect = None
        self.last_depth_m = None

    def reset(self):
        self.last_rect = None
        self.last_depth_m = None

    def select(self, candidates):
        if not candidates:
            return None
        if self.last_rect is None:
            selected = max(candidates, key=lambda item: item[3])
        else:
            previous_center = np.asarray(self.last_rect[0], dtype=np.float32)
            matches = []
            for candidate in candidates:
                center_distance = float(
                    np.linalg.norm(
                        np.asarray(candidate[0][0], dtype=np.float32)
                        - previous_center
                    )
                )
                depth_distance = abs(candidate[2] - self.last_depth_m)
                if (
                    center_distance <= self.max_center_jump_px
                    and depth_distance <= self.max_depth_jump_m
                ):
                    tracking_cost = (
                        center_distance / self.max_center_jump_px
                        + depth_distance / self.max_depth_jump_m
                    )
                    matches.append((tracking_cost, candidate))
            if not matches:
                return None
            selected = min(matches, key=lambda item: item[0])[1]

        self.last_rect = selected[0]
        self.last_depth_m = selected[2]
        return selected


class BasePoseVisualServo:
    """Move the carton to a fixed base-frame XY target while aligning yaw."""

    def __init__(
        self,
        gain,
        deadband_deg,
        max_yaw_rate_deg_s,
        angle_filter_alpha,
        aligned_frames,
        command_interval,
        command_duration,
        max_runtime,
        target_x_m,
        target_y_m,
        position_gain,
        position_deadband_m,
        max_linear_speed_m_s,
        acquire_frames,
        yaw_sign,
        position_sign,
        min_yaw_rate_deg_s,
        angle_offset_deg,
        cmd_vel_topic,
        vel_control_service,
        control_timeout,
    ):
        from geometry_msgs.msg import Twist
        from std_srvs.srv import SetBool

        self.gain = gain
        self.deadband_deg = deadband_deg
        self.max_yaw_rate_deg_s = max_yaw_rate_deg_s
        self.angle_filter_alpha = angle_filter_alpha
        self.required_aligned_frames = aligned_frames
        self.command_interval = command_interval
        self.command_duration = command_duration
        self.max_runtime = max_runtime
        self.target_position_xy = np.array(
            [target_x_m, target_y_m],
            dtype=np.float64,
        )
        self.position_gain = position_gain
        self.position_deadband_m = position_deadband_m
        self.max_linear_speed_m_s = max_linear_speed_m_s
        self.required_acquire_frames = acquire_frames
        self.yaw_sign = yaw_sign
        self.position_sign = position_sign
        self.min_yaw_rate_deg_s = min_yaw_rate_deg_s
        self.angle_offset_deg = angle_offset_deg
        self.filtered_angle_deg = None
        self.filtered_position = None
        self.reference_position = None
        self.acquire_positions = []
        self.acquire_angles = []
        self.last_linear_x = 0.0
        self.last_linear_y = 0.0
        self.last_command_deg_s = 0.0
        self.last_publish_time = 0.0
        self.aligned_frame_count = 0
        self.started_at = time.monotonic()
        self.active = True
        self.status = "waiting_target"
        self.cmd_vel_publisher = None
        self.vel_control_proxy = None
        self.restore_vel_control_on_close = False
        self.twist_type = Twist
        self.uses_navigation_cmd_vel = (
            rospy.resolve_name(cmd_vel_topic)
            == NAVIGATION_CMD_VEL_TOPIC
        )

        rospy.wait_for_service(vel_control_service, timeout=control_timeout)
        self.vel_control_proxy = rospy.ServiceProxy(
            vel_control_service,
            SetBool,
        )
        requested_vel_control = not self.uses_navigation_cmd_vel
        try:
            mode_response = self.vel_control_proxy(requested_vel_control)
        except Exception:
            self.vel_control_proxy.close()
            self.vel_control_proxy = None
            raise
        if not mode_response.success:
            self.vel_control_proxy.close()
            self.vel_control_proxy = None
            raise RuntimeError(
                "Failed to switch chassis velocity control to "
                f"{requested_vel_control}: "
                f"{mode_response.message}"
            )
        self.restore_vel_control_on_close = self.uses_navigation_cmd_vel

        self.cmd_vel_publisher = rospy.Publisher(
            cmd_vel_topic,
            Twist,
            queue_size=10,
        )
        connection_deadline = time.monotonic() + control_timeout
        while (
            self.cmd_vel_publisher.get_num_connections() == 0
            and time.monotonic() < connection_deadline
            and not rospy.is_shutdown()
        ):
            rospy.sleep(0.02)
        if self.cmd_vel_publisher.get_num_connections() == 0:
            self.cmd_vel_publisher.unregister()
            self.cmd_vel_publisher = None
            restore_error = None
            try:
                self._restore_vel_control()
            except Exception as error:
                restore_error = error
            finally:
                self.vel_control_proxy.close()
                self.vel_control_proxy = None
            restore_detail = (
                f"; additionally failed to restore velocity control: "
                f"{restore_error}"
                if restore_error is not None
                else ""
            )
            raise RuntimeError(
                f"No subscriber connected to {cmd_vel_topic}"
                f"{restore_detail}"
            )

    def set_target_x_m(self, target_x_m):
        target_x_m = float(target_x_m)
        if not math.isfinite(target_x_m) or target_x_m <= 0.0:
            raise ValueError(f"Invalid servo target_x_m: {target_x_m}")
        self.target_position_xy[0] = target_x_m

    def _enter_servo_control(self):
        if (
            not self.uses_navigation_cmd_vel
            or self.restore_vel_control_on_close
            or self.vel_control_proxy is None
        ):
            return
        response = self.vel_control_proxy(False)
        if not response.success:
            raise RuntimeError(
                "Failed to set /enable_vel_control=False: "
                f"{response.message}"
            )
        self.restore_vel_control_on_close = True

    def _restore_vel_control(self):
        if (
            not self.restore_vel_control_on_close
            or self.vel_control_proxy is None
        ):
            return
        response = self.vel_control_proxy(True)
        if not response.success:
            raise RuntimeError(
                "Failed to restore /enable_vel_control=True: "
                f"{response.message}"
            )
        self.restore_vel_control_on_close = False

    def _publish(self, linear_x, linear_y, yaw_rate_deg_s, duration):
        message = self.twist_type()
        message.linear.x = linear_x
        message.linear.y = linear_y
        message.angular.z = math.radians(yaw_rate_deg_s)
        publish_count = max(1, int(round(duration * 50.0)))
        rate = rospy.Rate(50)
        for _ in range(publish_count):
            self.cmd_vel_publisher.publish(message)
            rate.sleep()
        self.last_linear_x = linear_x
        self.last_linear_y = linear_y
        self.last_command_deg_s = yaw_rate_deg_s
        self.last_publish_time = time.monotonic()

    def stop(self, status, force=False):
        if self.cmd_vel_publisher is None:
            self.status = status
            return
        moving = (
            abs(self.last_linear_x) > 1e-6
            or abs(self.last_linear_y) > 1e-6
            or abs(self.last_command_deg_s) > 1e-6
        )
        if force or moving:
            self._publish(0.0, 0.0, 0.0, self.command_duration)
        self.last_linear_x = 0.0
        self.last_linear_y = 0.0
        self.last_command_deg_s = 0.0
        self.status = status

    def reset_reference(self):
        self._enter_servo_control()
        self.stop("reference_reset", force=True)
        self.filtered_angle_deg = None
        self.filtered_position = None
        self.reference_position = None
        self.acquire_positions.clear()
        self.acquire_angles.clear()
        self.aligned_frame_count = 0
        self.started_at = time.monotonic()
        self.active = True

    def update(self, angle_error_deg, position):
        now = time.monotonic()
        if (
            self.active
            and self.max_runtime > 0.0
            and now - self.started_at >= self.max_runtime
        ):
            self.active = False
            self.stop("timeout")
            self._restore_vel_control()

        if not self.active:
            return self._state()

        if (
            angle_error_deg is None
            or not math.isfinite(angle_error_deg)
            or position is None
            or not np.all(np.isfinite(position))
        ):
            self.filtered_angle_deg = None
            self.filtered_position = None
            self.aligned_frame_count = 0
            self.stop("target_lost")
            return self._state()

        if self.filtered_angle_deg is None:
            self.filtered_angle_deg = float(angle_error_deg)
            self.filtered_position = np.asarray(position, dtype=np.float64)
        else:
            alpha = self.angle_filter_alpha
            self.filtered_angle_deg = (
                alpha * float(angle_error_deg)
                + (1.0 - alpha) * self.filtered_angle_deg
            )
            self.filtered_position = (
                alpha * np.asarray(position, dtype=np.float64)
                + (1.0 - alpha) * self.filtered_position
            )

        if self.reference_position is None:
            self.acquire_positions.append(self.filtered_position.copy())
            self.acquire_angles.append(self.filtered_angle_deg)
            if len(self.acquire_positions) < self.required_acquire_frames:
                self.stop(
                    f"acquiring {len(self.acquire_positions)}/"
                    f"{self.required_acquire_frames}"
                )
                return self._state()
            stable_position = np.median(
                np.asarray(self.acquire_positions),
                axis=0,
            )
            self.reference_position = stable_position
            self.reference_position[:2] = self.target_position_xy
            self.status = "target_locked"
            self.stop(self.status)
            return self._state()

        angle_error_deg = (
            (self.filtered_angle_deg - self.angle_offset_deg + 90.0) % 180.0
        ) - 90.0
        command_deg_s = self.yaw_sign * compute_servo_yaw_deg_s(
            angle_error_deg,
            self.gain,
            self.deadband_deg,
            self.max_yaw_rate_deg_s,
        )
        if (
            command_deg_s != 0.0
            and abs(command_deg_s) < self.min_yaw_rate_deg_s
        ):
            command_deg_s = math.copysign(
                self.min_yaw_rate_deg_s,
                command_deg_s,
            )
        omega_rad_s = math.radians(command_deg_s)
        position_error = (
            self.filtered_position[:2] - self.reference_position[:2]
        )
        feedback = (
            self.position_sign * self.position_gain * position_error
        )
        position_error_norm = float(np.linalg.norm(position_error))
        if position_error_norm <= self.position_deadband_m:
            feedback[:] = 0.0

        x_current, y_current = self.filtered_position[:2]
        linear_x = float(feedback[0] + omega_rad_s * y_current)
        linear_y = float(feedback[1] - omega_rad_s * x_current)
        speed = math.hypot(linear_x, linear_y)
        if speed > self.max_linear_speed_m_s:
            scale = self.max_linear_speed_m_s / speed
            linear_x *= scale
            linear_y *= scale

        position_aligned = position_error_norm <= self.position_deadband_m
        if command_deg_s == 0.0 and position_aligned:
            self.aligned_frame_count += 1
            if self.aligned_frame_count >= self.required_aligned_frames:
                status = "aligned"
            else:
                status = (
                    f"settling {self.aligned_frame_count}/"
                    f"{self.required_aligned_frames}"
                )
            self.stop(status)
            if status == "aligned":
                self.active = False
                self._restore_vel_control()
        else:
            self.aligned_frame_count = 0
            self.status = "tracking"
            if now - self.last_publish_time >= self.command_interval:
                self._publish(
                    linear_x,
                    linear_y,
                    command_deg_s,
                    self.command_duration,
                )

        return self._state()

    def _state(self):
        position_error = None
        if (
            self.filtered_position is not None
            and self.reference_position is not None
        ):
            position_error = (
                self.filtered_position[:2] - self.reference_position[:2]
            )
        return {
            "status": self.status,
            "angle_deg": self.filtered_angle_deg,
            "position": self.filtered_position,
            "reference": self.reference_position,
            "position_error": position_error,
            "linear_x": self.last_linear_x,
            "linear_y": self.last_linear_y,
            "yaw_deg_s": self.last_command_deg_s,
        }

    def close(self):
        if (
            self.cmd_vel_publisher is None
            and self.vel_control_proxy is None
        ):
            return
        try:
            self.stop("stopped", force=True)
        except Exception as error:
            print(f"Failed to publish final base stop command: {error}")
        finally:
            if self.cmd_vel_publisher is not None:
                self.cmd_vel_publisher.unregister()
                self.cmd_vel_publisher = None
            if self.vel_control_proxy is not None:
                try:
                    self._restore_vel_control()
                except Exception as error:
                    print(f"Failed to restore velocity control: {error}")
                finally:
                    self.vel_control_proxy.close()
                    self.vel_control_proxy = None


class CartonObbServoService:
    """Run one blocking visual-servo cycle for each ROS service request."""

    def __init__(self, args):
        from box_detection_msgs.srv import ServoCartonObb
        from box_detection_msgs.srv import ServoCartonObbResponse
        from geometry_msgs.msg import PointStamped

        self.args = args
        self.response_type = ServoCartonObbResponse
        self.point_stamped_type = PointStamped
        self.request_lock = threading.Lock()
        self.inference_client = RosObbInferenceClient(
            args.inference_service,
            args.service_timeout,
        )
        self.color_subscriber = LatestFrameSubscriber(args.topic)
        self.color_subscriber.wait_for_first_frame(args.camera_timeout)
        self.depth_subscriber = LatestDepthSubscriber(args.depth_topic)
        self.depth_subscriber.wait_for_first_frame(args.camera_timeout)
        self.capture_root = (
            Path(args.servo_capture_dir).expanduser().resolve()
            if args.servo_capture_dir
            else None
        )
        if self.capture_root is not None:
            self.capture_root.mkdir(parents=True, exist_ok=True)
        self.position_projector = CameraPointProjector(
            args.camera_info_topic,
            args.servo_base_frame,
            args.camera_timeout,
        )
        self.result_projector = CameraPointProjector(
            args.camera_info_topic,
            args.servo_result_frame,
            args.camera_timeout,
        )
        self.result_publisher = rospy.Publisher(
            args.servo_result_topic,
            PointStamped,
            queue_size=1,
            latch=True,
        )
        self.service = rospy.Service(
            args.servo_service_name,
            ServoCartonObb,
            self._handle_request,
        )

    def _response(self, success, message, carton_pose=None):
        response = self.response_type()
        response.success = success
        response.message = message
        if carton_pose is not None:
            response.carton_pose = carton_pose
        return response

    @staticmethod
    def _validate_request(request):
        values = (
            request.max_yaw_rate_deg_s,
            request.max_linear_speed_m_s,
            request.target_x_m,
        )
        if not all(math.isfinite(value) for value in values):
            return "All servo request parameters must be finite"
        if request.max_yaw_rate_deg_s <= 0.0:
            return "max_yaw_rate_deg_s must be positive"
        if request.max_linear_speed_m_s <= 0.0:
            return "max_linear_speed_m_s must be positive"
        if request.target_x_m <= 0.0:
            return "target_x_m must be positive"
        if bool(getattr(request, "auto_target_x", False)):
            auto_values = (
                request.horizontal_target_x_m,
                request.vertical_target_x_m,
            )
            if not all(
                math.isfinite(value) and value > 0.0
                for value in auto_values
            ):
                return (
                    "horizontal_target_x_m and vertical_target_x_m "
                    "must be positive when auto_target_x is true"
                )
        return None

    def _auto_target_x_for_selection(self, request, selected):
        orientation = classify_carton_orientation(
            selected[0],
            self.args.servo_orientation_aspect_threshold,
        )
        if orientation == "horizontal":
            target_x = float(request.horizontal_target_x_m)
        elif orientation == "vertical":
            target_x = float(request.vertical_target_x_m)
        else:
            target_x = float(request.target_x_m)
        return orientation, target_x

    def _make_servo(self, request):
        args = self.args
        return BasePoseVisualServo(
            gain=args.servo_kp,
            deadband_deg=args.servo_deadband,
            max_yaw_rate_deg_s=request.max_yaw_rate_deg_s,
            angle_filter_alpha=args.servo_angle_alpha,
            aligned_frames=args.servo_aligned_frames,
            command_interval=args.servo_command_interval,
            command_duration=args.servo_command_duration,
            max_runtime=args.servo_max_runtime,
            target_x_m=request.target_x_m,
            target_y_m=args.servo_target_y,
            position_gain=args.servo_position_kp,
            position_deadband_m=args.servo_position_deadband,
            max_linear_speed_m_s=request.max_linear_speed_m_s,
            acquire_frames=args.servo_acquire_frames,
            yaw_sign=args.servo_yaw_sign,
            position_sign=args.servo_position_sign,
            min_yaw_rate_deg_s=args.servo_min_yaw_rate,
            angle_offset_deg=args.servo_angle_offset,
            cmd_vel_topic=args.cmd_vel_topic,
            vel_control_service=args.vel_control_service,
            control_timeout=args.servo_control_timeout,
        )

    def _select_target(self, detections, source_shape, depth_m):
        args = self.args
        _, _, _, candidates = select_best_obb(
            detections,
            depth_m,
            source_shape,
            args.depth_shrink,
            args.min_depth,
            args.max_depth,
            args.depth_mad_scale,
            args.depth_outlier_floor,
            args.min_depth_samples,
            args.depth_weight,
            args.bottom_center_weight,
            args.area_weight,
        )
        return candidates

    def _build_result(self, selected, color_stamp):
        result_position = self.result_projector.project(
            selected[0][0],
            selected[2],
            color_stamp,
        )
        message = self.point_stamped_type()
        message.header.stamp = rospy.Time.from_sec(color_stamp)
        message.header.frame_id = self.result_projector.base_frame
        message.point.x = float(result_position[0])
        message.point.y = float(result_position[1])
        message.point.z = float(result_position[2])
        return message

    def _create_capture_session(self):
        if self.capture_root is None:
            return None
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        milliseconds = int((time.time() % 1.0) * 1000)
        session_dir = self.capture_root / (
            f"request_{timestamp}_{milliseconds:03d}"
        )
        session_dir.mkdir(parents=True, exist_ok=False)
        return session_dir

    @staticmethod
    def _save_capture(session_dir, phase, frame_index, rendered):
        if session_dir is None or rendered is None:
            return None
        timestamp_ns = time.time_ns()
        save_path = session_dir / (
            f"{phase}_{frame_index:06d}_{timestamp_ns}.jpg"
        )
        if not cv2.imwrite(
            str(save_path),
            rendered,
            [cv2.IMWRITE_JPEG_QUALITY, 95],
        ):
            rospy.logwarn(f"Failed to save OBB servo capture: {save_path}")
            return None
        return save_path

    def _run_servo(self, request):
        args = self.args
        servo = None
        capture_session = self._create_capture_session()
        capture_last_saved_at = float("-inf")
        capture_last_rendered = None
        capture_last_frame_index = 0
        # #region debug-point B:servo-runtime-state
        debug_url = "http://127.0.0.1:7777/event"
        debug_started_at = time.monotonic()
        debug_last_report_at = 0.0
        debug_previous_position = None
        debug_frame_count = 0
        debug_selected_count = 0
        auto_target_orientation = "fixed"
        active_target_x_m = float(request.target_x_m)
        auto_target_reported = False

        def debug_report(hypothesis_id, message, data):
            try:
                payload = json.dumps(
                    {
                        "sessionId": "carton-servo-timeout",
                        "runId": "post-fix",
                        "hypothesisId": hypothesis_id,
                        "location": "realtime_carton_obb_trt.py:_run_servo",
                        "msg": f"[DEBUG] {message}",
                        "data": data,
                        "ts": int(time.time() * 1000),
                    }
                ).encode()
                urllib.request.urlopen(
                    urllib.request.Request(
                        debug_url,
                        data=payload,
                        headers={"Content-Type": "application/json"},
                    ),
                    timeout=0.05,
                ).read()
            except Exception:
                pass
        # #endregion
        try:
            servo = self._make_servo(request)
            auto_target_x_enabled = bool(getattr(request, "auto_target_x", False))
            # #region debug-point D:request-budget
            debug_report(
                "D",
                "servo request configuration",
                {
                    "max_runtime_s": args.servo_max_runtime,
                    "target_x_m": request.target_x_m,
                    "auto_target_x": auto_target_x_enabled,
                    "horizontal_target_x_m": getattr(
                        request,
                        "horizontal_target_x_m",
                        None,
                    ),
                    "vertical_target_x_m": getattr(
                        request,
                        "vertical_target_x_m",
                        None,
                    ),
                    "max_linear_speed_m_s": request.max_linear_speed_m_s,
                    "max_yaw_rate_deg_s": request.max_yaw_rate_deg_s,
                    "position_deadband_m": args.servo_position_deadband,
                    "angle_deadband_deg": args.servo_deadband,
                    "required_aligned_frames": args.servo_aligned_frames,
                },
            )
            # #endregion
            tracker = LockedObbTracker(
                args.servo_max_center_jump,
                args.servo_max_depth_jump,
            )
            last_source_stamp_ns = 0
            while not rospy.is_shutdown():
                (
                    detections,
                    inference_ms,
                    color_stamp,
                    source_stamp_ns,
                    source_shape,
                ) = self.inference_client.infer(
                    args.conf,
                    args.iou,
                )
                if source_stamp_ns == last_source_stamp_ns:
                    rospy.sleep(0.001)
                    continue
                last_source_stamp_ns = source_stamp_ns
                frame = self.color_subscriber.frame_for_stamp(source_stamp_ns)
                if frame is None:
                    rospy.logwarn_throttle(
                        2.0,
                        "OBB servo capture skipped: source frame %d is not "
                        "in the local color-frame history",
                        source_stamp_ns,
                    )

                depth_m, depth_stamp = self.depth_subscriber.latest()
                candidates = []
                if (
                    depth_m is not None
                    and abs(color_stamp - depth_stamp) <= args.max_depth_age
                ):
                    candidates = self._select_target(
                        detections,
                        source_shape,
                        depth_m,
                    )
                selected = tracker.select(candidates)
                debug_frame_count += 1
                if selected is not None:
                    debug_selected_count += 1
                angle_error_deg = None
                position = None
                if selected is not None:
                    angle_error_deg = horizontal_rotation_deg(selected[0])
                    if (
                        auto_target_x_enabled
                        and servo.reference_position is None
                    ):
                        (
                            auto_target_orientation,
                            active_target_x_m,
                        ) = self._auto_target_x_for_selection(
                            request,
                            selected,
                        )
                        servo.set_target_x_m(active_target_x_m)
                        if not auto_target_reported:
                            print(
                                "Servo auto target: "
                                f"orientation={auto_target_orientation}, "
                                f"target_x={active_target_x_m:.3f}m",
                                flush=True,
                            )
                            auto_target_reported = True
                    try:
                        position = self.position_projector.project(
                            selected[0][0],
                            selected[2],
                            color_stamp,
                        )
                    except Exception as error:
                        rospy.logwarn_throttle(
                            2.0,
                            f"Carton position projection failed: {error}",
                        )

                state = servo.update(angle_error_deg, position)
                capture_last_frame_index = debug_frame_count
                if frame is not None:
                    capture_last_rendered = draw_servo_capture(
                        frame=frame,
                        detections=detections,
                        selected=selected,
                        state=state,
                        frame_index=debug_frame_count,
                        phase="PROCESS",
                        inference_ms=inference_ms,
                    )
                    capture_now = time.monotonic()
                    if (
                        capture_session is not None
                        and capture_now - capture_last_saved_at
                        >= args.servo_capture_interval
                    ):
                        saved_path = self._save_capture(
                            capture_session,
                            "process",
                            debug_frame_count,
                            capture_last_rendered,
                        )
                        if saved_path is not None:
                            capture_last_saved_at = capture_now
                # #region debug-point B,C,E:periodic-control-state
                debug_now = time.monotonic()
                if debug_now - debug_last_report_at >= 1.0:
                    state_position = state["position"]
                    position_delta_m = None
                    if (
                        state_position is not None
                        and debug_previous_position is not None
                    ):
                        position_delta_m = float(
                            np.linalg.norm(
                                state_position[:2] - debug_previous_position[:2]
                            )
                        )
                    debug_report(
                        "B",
                        "target tracking state",
                        {
                            "elapsed_s": debug_now - debug_started_at,
                            "detections": len(detections),
                            "candidates": len(candidates),
                            "selected": selected is not None,
                            "selected_ratio": (
                                debug_selected_count / debug_frame_count
                            ),
                            "auto_target_orientation": auto_target_orientation,
                            "active_target_x_m": active_target_x_m,
                            "depth_age_s": (
                                None
                                if depth_m is None
                                else abs(color_stamp - depth_stamp)
                            ),
                            "status": state["status"],
                        },
                    )
                    debug_report(
                        "C",
                        "command and observed position",
                        {
                            "elapsed_s": debug_now - debug_started_at,
                            "position": (
                                None
                                if state_position is None
                                else state_position.tolist()
                            ),
                            "position_delta_m": position_delta_m,
                            "linear_x": state["linear_x"],
                            "linear_y": state["linear_y"],
                            "yaw_deg_s": state["yaw_deg_s"],
                            "cmd_vel_connections": (
                                0
                                if servo.cmd_vel_publisher is None
                                else servo.cmd_vel_publisher.get_num_connections()
                            ),
                            "vel_control_switched": (
                                servo.restore_vel_control_on_close
                            ),
                        },
                    )
                    debug_report(
                        "E",
                        "alignment error state",
                        {
                            "elapsed_s": debug_now - debug_started_at,
                            "angle_deg": state["angle_deg"],
                            "position_error": (
                                None
                                if state["position_error"] is None
                                else state["position_error"].tolist()
                            ),
                            "status": state["status"],
                            "aligned_frames": servo.aligned_frame_count,
                        },
                    )
                    debug_last_report_at = debug_now
                    if state_position is not None:
                        debug_previous_position = state_position.copy()
                # #endregion
                if state["status"] == "aligned" and selected is not None:
                    if frame is not None:
                        capture_last_rendered = draw_servo_capture(
                            frame=frame,
                            detections=detections,
                            selected=selected,
                            state=state,
                            frame_index=debug_frame_count,
                            phase="FINAL_ALIGNED",
                            inference_ms=inference_ms,
                        )
                    final_path = self._save_capture(
                        capture_session,
                        "final_aligned",
                        debug_frame_count,
                        capture_last_rendered,
                    )
                    result = self._build_result(selected, color_stamp)
                    self.result_publisher.publish(result)
                    message = (
                        "Servo aligned; "
                        f"orientation={auto_target_orientation}, "
                        f"target_x={active_target_x_m:.3f}m, "
                        f"{result.header.frame_id} xyz=("
                        f"{result.point.x:.4f}, "
                        f"{result.point.y:.4f}, "
                        f"{result.point.z:.4f})m"
                    )
                    if final_path is not None:
                        message += f"; capture={final_path}"
                    print(message, flush=True)
                    return self._response(True, message, result)
                if state["status"] == "timeout":
                    self._save_capture(
                        capture_session,
                        "final_timeout",
                        capture_last_frame_index,
                        capture_last_rendered,
                    )
                    return self._response(
                        False,
                        f"Servo timed out after {args.servo_max_runtime:.1f}s",
                    )
            return self._response(False, "ROS shutdown during servo")
        except Exception as error:
            rospy.logerr(f"Carton OBB servo request failed: {error}")
            return self._response(False, str(error))
        finally:
            if servo is not None:
                servo.close()

    def _handle_request(self, request):
        validation_error = self._validate_request(request)
        if validation_error is not None:
            print(
                f"Servo response   : success=False, message={validation_error}",
                flush=True,
            )
            return self._response(False, validation_error)
        if not self.request_lock.acquire(blocking=False):
            message = "Another servo request is running"
            print(
                f"Servo response   : success=False, message={message}",
                flush=True,
            )
            return self._response(False, message)
        try:
            print(
                "Servo request    : "
                f"yaw_limit={request.max_yaw_rate_deg_s:.2f}deg/s, "
                f"xy_limit={request.max_linear_speed_m_s:.3f}m/s, "
                f"target_x={request.target_x_m:.3f}m, "
                f"auto_target_x={bool(getattr(request, 'auto_target_x', False))}, "
                f"horizontal={float(getattr(request, 'horizontal_target_x_m', 0.0)):.3f}m, "
                f"vertical={float(getattr(request, 'vertical_target_x_m', 0.0)):.3f}m",
                flush=True,
            )
            response = self._run_servo(request)
            print(
                f"Servo response   : success={response.success}, "
                f"message={response.message}",
                flush=True,
            )
            return response
        finally:
            self.request_lock.release()

    def close(self):
        if self.service is not None:
            self.service.shutdown("carton OBB servo service stopped")
            self.service = None
        if self.result_publisher is not None:
            self.result_publisher.unregister()
            self.result_publisher = None
        if self.depth_subscriber is not None:
            self.depth_subscriber.close()
            self.depth_subscriber = None
        if self.color_subscriber is not None:
            self.color_subscriber.close()
            self.color_subscriber = None
        if self.inference_client is not None:
            self.inference_client.close()
            self.inference_client = None


def draw_fitted_text(
    image,
    text,
    origin,
    font_scale,
    color,
    thickness,
):
    font = cv2.FONT_HERSHEY_SIMPLEX
    available_width = max(1, image.shape[1] - origin[0] - 10)
    (text_width, _), _ = cv2.getTextSize(
        text,
        font,
        font_scale,
        thickness,
    )
    if text_width > available_width:
        font_scale *= available_width / text_width
    cv2.putText(
        image,
        text,
        origin,
        font,
        max(font_scale, 0.30),
        color,
        thickness,
        cv2.LINE_AA,
    )


def draw_rotation_reference(image, center, rotation_deg):
    center_x, center_y = center
    axis_length = 68
    arc_radius = 27
    image_height, image_width = image.shape[:2]

    x_end = (center_x + axis_length, center_y)
    y_end = (center_x, center_y + axis_length)
    angle_rad = math.radians(rotation_deg)
    obb_end = (
        int(round(center_x + axis_length * math.cos(angle_rad))),
        int(round(center_y + axis_length * math.sin(angle_rad))),
    )

    cv2.arrowedLine(
        image,
        center,
        x_end,
        (255, 128, 0),
        2,
        cv2.LINE_AA,
        tipLength=0.14,
    )
    cv2.arrowedLine(
        image,
        center,
        y_end,
        (255, 255, 0),
        2,
        cv2.LINE_AA,
        tipLength=0.14,
    )
    cv2.arrowedLine(
        image,
        center,
        obb_end,
        (255, 0, 255),
        3,
        cv2.LINE_AA,
        tipLength=0.16,
    )

    arc_angles = np.linspace(0.0, angle_rad, 24)
    arc_points = np.column_stack(
        (
            center_x + arc_radius * np.cos(arc_angles),
            center_y + arc_radius * np.sin(arc_angles),
        )
    )
    cv2.polylines(
        image,
        [np.rint(arc_points).astype(np.int32)],
        False,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )

    x_label = (
        min(image_width - 28, center_x + axis_length + 3),
        max(14, center_y - 5),
    )
    y_label = (
        min(image_width - 28, center_x + 5),
        min(image_height - 5, center_y + axis_length),
    )
    cv2.putText(
        image,
        "+X",
        x_label,
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        (255, 128, 0),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        image,
        "+Y",
        y_label,
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        (255, 255, 0),
        2,
        cv2.LINE_AA,
    )
    angle_label_position = (
        int(np.clip(center_x + 30, 4, image_width - 92)),
        int(np.clip(center_y - 12, 14, image_height - 5)),
    )
    cv2.putText(
        image,
        f"{rotation_deg:+.1f} deg",
        angle_label_position,
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )


def draw_servo_capture(
    frame,
    detections,
    selected,
    state,
    frame_index,
    phase,
    inference_ms,
):
    """Render the exact inference frame and target used by visual servo."""
    rendered = frame.copy()
    selected_rect = selected[0] if selected is not None else None
    for rect, confidence in detections:
        points = cv2.boxPoints(rect).astype(np.int32)
        is_selected = rect is selected_rect or (
            selected_rect is not None and rect == selected_rect
        )
        color = (0, 255, 0) if is_selected else (180, 180, 180)
        thickness = 3 if is_selected else 1
        cv2.polylines(
            rendered,
            [points],
            True,
            color,
            thickness,
            cv2.LINE_AA,
        )
        center = tuple(np.round(rect[0]).astype(int))
        label = (
            f"{'SELECTED ' if is_selected else ''}"
            f"conf={confidence:.3f} angle={horizontal_rotation_deg(rect):+.1f}"
        )
        cv2.putText(
            rendered,
            label,
            (max(4, center[0] - 90), max(118, center[1] - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            color,
            2 if is_selected else 1,
            cv2.LINE_AA,
        )

    cv2.rectangle(rendered, (0, 0), (rendered.shape[1], 105), (0, 0, 0), -1)
    selected_depth = selected[2] if selected is not None else None
    selected_score = selected[3] if selected is not None else None
    angle = (
        horizontal_rotation_deg(selected_rect)
        if selected_rect is not None
        else None
    )
    position_error = state.get("position_error")
    error_text = (
        "N/A"
        if position_error is None
        else f"({position_error[0]:+.3f},{position_error[1]:+.3f})m"
    )
    draw_fitted_text(
        rendered,
        f"{phase} | frame {frame_index} | servo={state.get('status', 'N/A')}",
        (10, 29),
        0.63,
        (0, 255, 0) if phase == "FINAL_ALIGNED" else (0, 200, 255),
        2,
    )
    draw_fitted_text(
        rendered,
        (
            f"OBB total={len(detections)} | selected="
            f"{'yes' if selected is not None else 'no'} | "
            f"conf={selected[1]:.3f} depth={selected_depth:.3f}m "
            f"score={selected_score:.3f}"
            if selected is not None
            else f"OBB total={len(detections)} | selected=no"
        ),
        (10, 59),
        0.52,
        (255, 255, 255),
        1,
    )
    draw_fitted_text(
        rendered,
        (
            f"angle={angle:+.1f}deg | eXY={error_text} | "
            f"v=({state.get('linear_x', 0.0):+.3f},"
            f"{state.get('linear_y', 0.0):+.3f})m/s | "
            f"w={state.get('yaw_deg_s', 0.0):+.1f}deg/s | "
            f"infer={inference_ms:.2f}ms"
            if angle is not None
            else f"angle=N/A | eXY={error_text}"
        ),
        (10, 89),
        0.50,
        (0, 255, 255),
        1,
    )
    return rendered


def draw_detections(
    frame,
    detections,
    inference_ms,
    display_fps,
    confidence,
    candidate_count,
    valid_depth_count,
    depth_age_ms,
    servo_enabled,
    servo_status,
    servo_raw_angle_deg,
    servo_filtered_angle_deg,
    servo_position_error,
    servo_linear_x,
    servo_linear_y,
    servo_command_deg_s,
):
    rendered = frame.copy()
    for (
        rect,
        _confidence,
        distance_m,
        total_score,
        _depth_score,
        _bottom_score,
        _area_score,
    ) in detections:
        points = cv2.boxPoints(rect).astype(np.int32)
        cv2.polylines(
            rendered,
            [points],
            True,
            (0, 255, 0),
            3,
            cv2.LINE_AA,
        )
        center = tuple(np.round(rect[0]).astype(int))
        cv2.circle(rendered, center, 4, (0, 0, 255), -1, cv2.LINE_AA)

        rotation_deg = horizontal_rotation_deg(rect)
        draw_rotation_reference(rendered, center, rotation_deg)
        label = (
            f"BEST score {total_score:.3f} | {distance_m:.3f} m "
            f"| angle {rotation_deg:+.1f} deg"
        )
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.65
        text_thickness = 2
        (text_width, text_height), baseline = cv2.getTextSize(
            label,
            font,
            font_scale,
            text_thickness,
        )
        margin = 6
        label_x = int(points[:, 0].min())
        label_x = int(
            np.clip(
                label_x,
                margin,
                max(margin, rendered.shape[1] - text_width - margin),
            )
        )
        label_y = int(points[:, 1].min()) - 8
        if label_y - text_height < 106:
            label_y = int(points[:, 1].max()) + text_height + 8
        label_y = int(
            np.clip(
                label_y,
                106 + text_height,
                rendered.shape[0] - baseline - margin,
            )
        )
        position = (label_x, label_y)
        cv2.putText(
            rendered,
            label,
            position,
            font,
            font_scale,
            (0, 0, 0),
            text_thickness + 2,
            cv2.LINE_AA,
        )
        cv2.putText(
            rendered,
            label,
            position,
            font,
            font_scale,
            (0, 255, 0),
            text_thickness,
            cv2.LINE_AA,
        )

    cv2.rectangle(
        rendered,
        (0, 0),
        (rendered.shape[1], 102),
        (0, 0, 0),
        -1,
    )
    status = (
        f"TRT FP16 | boxes {len(detections)} | infer {inference_ms:.2f} ms "
        f"| FPS {display_fps:.1f} | conf {confidence:.2f}"
    )
    draw_fitted_text(
        rendered,
        status,
        (10, 33),
        0.63,
        (255, 255, 255),
        2,
    )
    selected_text = (
        f"candidates {candidate_count} | valid depth {valid_depth_count} "
        f"| depth age {depth_age_ms:.1f} ms"
    )
    if detections:
        selected_text += (
            f" | best score {detections[0][3]:.3f} "
            f"| depth {detections[0][2]:.3f} m "
            f"| angle {horizontal_rotation_deg(detections[0][0]):+.1f} deg"
        )
    else:
        selected_text += " | best N/A"
    draw_fitted_text(
        rendered,
        selected_text,
        (10, 63),
        0.52,
        (0, 255, 255),
        1,
    )
    if servo_enabled:
        raw_text = (
            "N/A"
            if servo_raw_angle_deg is None
            else f"{servo_raw_angle_deg:+.1f}"
        )
        filtered_text = (
            "N/A"
            if servo_filtered_angle_deg is None
            else f"{servo_filtered_angle_deg:+.1f}"
        )
        error_text = (
            "N/A"
            if servo_position_error is None
            else (
                f"{servo_position_error[0]:+.3f},"
                f"{servo_position_error[1]:+.3f}"
            )
        )
        servo_text = (
            f"SERVO {servo_status} | angle {raw_text}/{filtered_text} deg "
            f"| eXY {error_text} m | v {servo_linear_x:+.3f},"
            f"{servo_linear_y:+.3f} m/s | w {servo_command_deg_s:+.1f} deg/s"
        )
        servo_color = (
            (0, 255, 0) if servo_status == "aligned" else (0, 165, 255)
        )
    else:
        servo_text = "SERVO disabled (use --enable-base-servo)"
        servo_color = (160, 160, 160)
    draw_fitted_text(
        rendered,
        servo_text,
        (10, 91),
        0.52,
        servo_color,
        1,
    )
    return rendered


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Visualize and servo from the shared carton OBB inference service."
        )
    )
    parser.add_argument(
        "--inference-service",
        default=DEFAULT_INFERENCE_SERVICE,
        help="Shared carton OBB TensorRT ROS service",
    )
    parser.add_argument(
        "--service-timeout",
        type=float,
        default=15.0,
        help="Seconds to wait for the shared inference service",
    )
    parser.add_argument(
        "--topic",
        default=DEFAULT_TOPIC,
        help="ROS sensor_msgs/Image topic",
    )
    parser.add_argument(
        "--depth-topic",
        default=DEFAULT_DEPTH_TOPIC,
        help="Aligned ROS depth image topic",
    )
    parser.add_argument(
        "--camera-info-topic",
        default="/camera/color/camera_info",
        help="Color camera calibration topic",
    )
    parser.add_argument(
        "--conf",
        type=float,
        default=0.25,
        help="Confidence threshold (default: 0.25)",
    )
    parser.add_argument(
        "--iou",
        type=float,
        default=0.45,
        help="Rotated NMS IoU threshold (default: 0.45)",
    )
    parser.add_argument(
        "--camera-timeout",
        type=float,
        default=10.0,
        help="Seconds to wait for the first color and depth frames",
    )
    parser.add_argument(
        "--depth-shrink",
        type=float,
        default=0.6,
        help="OBB side-length ratio used for depth sampling (default: 0.6)",
    )
    parser.add_argument(
        "--depth-mad-scale",
        type=float,
        default=3.5,
        help="MAD outlier threshold in robust standard deviations",
    )
    parser.add_argument(
        "--depth-outlier-floor",
        type=float,
        default=0.02,
        help="Minimum depth outlier threshold in meters",
    )
    parser.add_argument(
        "--min-depth-samples",
        type=int,
        default=30,
        help="Minimum inlier pixels required for an OBB depth",
    )
    parser.add_argument(
        "--max-depth-age",
        type=float,
        default=0.2,
        help="Maximum color/depth timestamp difference in seconds",
    )
    parser.add_argument(
        "--min-depth",
        type=float,
        default=0.1,
        help="Minimum valid center depth in meters",
    )
    parser.add_argument(
        "--max-depth",
        type=float,
        default=10.0,
        help="Maximum valid center depth in meters",
    )
    parser.add_argument(
        "--depth-weight",
        type=float,
        default=0.3,
        help="Weight for nearer depth (default: 0.3)",
    )
    parser.add_argument(
        "--bottom-center-weight",
        type=float,
        default=0.5,
        help="Weight for proximity to image bottom-center (default: 0.5)",
    )
    parser.add_argument(
        "--area-weight",
        type=float,
        default=0.2,
        help="Weight for larger OBB area (default: 0.2)",
    )
    parser.add_argument(
        "--no-display",
        action="store_true",
        help="Disable the OpenCV window",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="Stop after N inferred frames; 0 means unlimited",
    )
    parser.add_argument(
        "--save-dir",
        default=str(Path(__file__).resolve().parent),
        help="Directory used when the S key saves a result",
    )
    parser.add_argument(
        "--output",
        default="",
        help="Save the final rendered frame to this path on exit",
    )
    parser.add_argument(
        "--enable-base-servo",
        action="store_true",
        help="Enable closed-loop chassis XY and yaw control from the selected OBB",
    )
    parser.add_argument(
        "--serve-servo",
        action="store_true",
        help="Run a blocking ROS service that starts one servo cycle per call",
    )
    parser.add_argument(
        "--servo-service-name",
        default="/carton_obb/servo",
        help="Blocking visual-servo ROS service name",
    )
    parser.add_argument(
        "--cmd-vel-topic",
        default=NAVIGATION_CMD_VEL_TOPIC,
        help=(
            "Chassis velocity command topic; using "
            "/move_base/base_cmd_vel switches /enable_vel_control to False "
            "during servo and restores True on exit"
        ),
    )
    parser.add_argument(
        "--vel-control-service",
        default="/enable_vel_control",
        help="Service used to enable direct chassis velocity control",
    )
    parser.add_argument(
        "--servo-control-timeout",
        type=float,
        default=5.0,
        help="Seconds to wait for chassis control ROS interfaces",
    )
    parser.add_argument(
        "--servo-kp",
        type=float,
        default=0.5,
        help="Yaw-rate gain in (deg/s)/deg (default: 0.5)",
    )
    parser.add_argument(
        "--servo-deadband",
        type=float,
        default=1.0,
        help="Stop when the filtered OBB angle is within this range in degrees",
    )
    parser.add_argument(
        "--servo-max-yaw-rate",
        type=float,
        default=8.0,
        help="Maximum absolute chassis yaw rate in deg/s (default: 8)",
    )
    parser.add_argument(
        "--servo-min-yaw-rate",
        type=float,
        default=0.0,
        help="Minimum nonzero chassis yaw rate in deg/s; 0 disables it",
    )
    parser.add_argument(
        "--servo-angle-offset",
        type=float,
        default=0.0,
        help="Desired OBB angle used as the calibrated zero in degrees",
    )
    parser.add_argument(
        "--servo-angle-alpha",
        type=float,
        default=0.35,
        help="OBB-angle EMA factor in the range (0, 1] (default: 0.35)",
    )
    parser.add_argument(
        "--servo-aligned-frames",
        type=int,
        default=5,
        help="Consecutive in-deadband frames required for aligned status",
    )
    parser.add_argument(
        "--servo-command-interval",
        type=float,
        default=0.10,
        help="Minimum seconds between chassis command bursts",
    )
    parser.add_argument(
        "--servo-command-duration",
        type=float,
        default=0.08,
        help="Duration of each 100 Hz command burst in seconds",
    )
    parser.add_argument(
        "--servo-max-runtime",
        type=float,
        default=30.0,
        help="Maximum active servo time in seconds; 0 disables the timeout",
    )
    parser.add_argument(
        "--servo-base-frame",
        default="base_link",
        help="Robot frame used for carton position control",
    )
    parser.add_argument(
        "--servo-result-frame",
        default="waist_link",
        help="Frame used for the carton position published after alignment",
    )
    parser.add_argument(
        "--servo-result-topic",
        default="/carton_obb/servo_result",
        help="Latched PointStamped topic published after servo alignment",
    )
    parser.add_argument(
        "--servo-target-x",
        type=float,
        default=1.10,
        help="Target carton X coordinate in the servo base frame, in meters",
    )
    parser.add_argument(
        "--servo-target-y",
        type=float,
        default=0.0,
        help="Target carton Y coordinate in the servo base frame, in meters",
    )
    parser.add_argument(
        "--servo-position-kp",
        type=float,
        default=0.8,
        help="Carton XY position gain in (m/s)/m",
    )
    parser.add_argument(
        "--servo-yaw-sign",
        type=float,
        choices=(-1.0, 1.0),
        default=-1.0,
        help="Yaw correction direction multiplier (default: -1)",
    )
    parser.add_argument(
        "--servo-position-sign",
        type=float,
        choices=(-1.0, 1.0),
        default=1.0,
        help="XY position correction direction multiplier (default: +1)",
    )
    parser.add_argument(
        "--servo-position-deadband",
        type=float,
        default=0.01,
        help="Carton XY position deadband in meters",
    )
    parser.add_argument(
        "--servo-max-linear-speed",
        type=float,
        default=0.04,
        help="Maximum combined chassis XY speed in m/s",
    )
    parser.add_argument(
        "--servo-acquire-frames",
        type=int,
        default=5,
        help="Stable target frames required before moving to the fixed XY target",
    )
    parser.add_argument(
        "--servo-max-center-jump",
        type=float,
        default=90.0,
        help="Maximum per-frame locked-target center jump in pixels",
    )
    parser.add_argument(
        "--servo-max-depth-jump",
        type=float,
        default=0.15,
        help="Maximum per-frame locked-target depth jump in meters",
    )
    parser.add_argument(
        "--servo-orientation-aspect-threshold",
        type=float,
        default=1.10,
        help=(
            "Minimum OBB long/short side ratio used to classify "
            "horizontal vs vertical carton orientation"
        ),
    )
    parser.add_argument(
        "--servo-capture-dir",
        default="",
        help=(
            "Save annotated process and final servo frames under this "
            "directory; empty disables capture"
        ),
    )
    parser.add_argument(
        "--servo-capture-interval",
        type=float,
        default=1.0,
        help="Minimum seconds between saved process frames (default: 1.0)",
    )
    args = parser.parse_args()
    if not 0.0 <= args.conf <= 1.0:
        parser.error("--conf must be between 0 and 1")
    if not 0.0 <= args.iou <= 1.0:
        parser.error("--iou must be between 0 and 1")
    if args.service_timeout <= 0.0:
        parser.error("--service-timeout must be positive")
    if args.servo_control_timeout <= 0.0:
        parser.error("--servo-control-timeout must be positive")
    if args.max_frames < 0:
        parser.error("--max-frames must be non-negative")
    if not 0.0 < args.depth_shrink <= 1.0:
        parser.error("--depth-shrink must be in the range (0, 1]")
    if args.depth_mad_scale <= 0 or args.depth_outlier_floor < 0:
        parser.error(
            "--depth-mad-scale must be positive and "
            "--depth-outlier-floor must be non-negative"
        )
    if args.min_depth_samples <= 0:
        parser.error("--min-depth-samples must be positive")
    if args.max_depth_age < 0:
        parser.error("--max-depth-age must be non-negative")
    if args.min_depth < 0 or args.max_depth <= args.min_depth:
        parser.error("--max-depth must be greater than --min-depth")
    weights = (
        args.depth_weight,
        args.bottom_center_weight,
        args.area_weight,
    )
    if any(weight < 0 for weight in weights) or sum(weights) <= 0:
        parser.error("Scoring weights must be non-negative with a positive sum")
    if args.servo_kp <= 0.0:
        parser.error("--servo-kp must be positive")
    if args.servo_deadband < 0.0 or args.servo_max_yaw_rate <= 0.0:
        parser.error(
            "--servo-deadband must be non-negative and "
            "--servo-max-yaw-rate must be positive"
        )
    if (
        args.servo_min_yaw_rate < 0.0
        or args.servo_min_yaw_rate > args.servo_max_yaw_rate
    ):
        parser.error(
            "--servo-min-yaw-rate must be between 0 and "
            "--servo-max-yaw-rate"
        )
    if not 0.0 < args.servo_angle_alpha <= 1.0:
        parser.error("--servo-angle-alpha must be in the range (0, 1]")
    if args.servo_aligned_frames <= 0:
        parser.error("--servo-aligned-frames must be positive")
    if (
        args.servo_command_interval <= 0.0
        or args.servo_command_duration <= 0.0
    ):
        parser.error(
            "--servo-command-interval and --servo-command-duration "
            "must be positive"
        )
    if args.servo_max_runtime < 0.0:
        parser.error("--servo-max-runtime must be non-negative")
    if not (
        math.isfinite(args.servo_target_x)
        and math.isfinite(args.servo_target_y)
    ):
        parser.error("--servo-target-x and --servo-target-y must be finite")
    if args.servo_position_kp <= 0.0:
        parser.error("--servo-position-kp must be positive")
    if (
        args.servo_position_deadband < 0.0
        or args.servo_max_linear_speed <= 0.0
    ):
        parser.error(
            "--servo-position-deadband must be non-negative and "
            "--servo-max-linear-speed must be positive"
        )
    if args.servo_acquire_frames <= 0:
        parser.error("--servo-acquire-frames must be positive")
    if (
        args.servo_max_center_jump <= 0.0
        or args.servo_max_depth_jump <= 0.0
    ):
        parser.error(
            "--servo-max-center-jump and --servo-max-depth-jump "
            "must be positive"
        )
    if args.servo_orientation_aspect_threshold < 1.0:
        parser.error("--servo-orientation-aspect-threshold must be >= 1.0")
    if args.servo_capture_interval <= 0.0:
        parser.error("--servo-capture-interval must be positive")
    return args


def main():
    args = parse_args()
    save_dir = Path(args.save_dir).expanduser().resolve()
    output_path = (
        Path(args.output).expanduser().resolve() if args.output else None
    )
    save_dir.mkdir(parents=True, exist_ok=True)
    if output_path is not None:
        output_path.parent.mkdir(parents=True, exist_ok=True)

    rospy.init_node(
        (
            "carton_obb_servo_service"
            if args.serve_servo
            else "carton_obb_trt_realtime"
        ),
        anonymous=not args.serve_servo,
        disable_signals=True,
    )
    if args.serve_servo:
        server = None
        try:
            server = CartonObbServoService(args)
            print(f"Servo service    : {args.servo_service_name}")
            print(f"Inference service: {args.inference_service}")
            print(
                "Response frame   : "
                f"{args.servo_result_frame} "
                f"(also latched on {args.servo_result_topic})"
            )
            if server.capture_root is not None:
                print(
                    "Servo captures   : "
                    f"{server.capture_root} "
                    f"(every {args.servo_capture_interval:.2f}s + final)"
                )
            print("Waiting for servo requests; press Ctrl-C to stop.")
            rospy.spin()
        except KeyboardInterrupt:
            print("\nInterrupted by user.")
        finally:
            if server is not None:
                server.close()
            rospy.signal_shutdown("carton OBB servo service stopped")
        return

    subscriber = None
    depth_subscriber = None
    inference_client = None
    base_servo = None
    position_projector = None
    result_projector = None
    result_publisher = None
    result_published = False
    target_tracker = None
    processed_frames = 0
    last_sequence = -1
    last_source_stamp_ns = 0
    smoothed_fps = 0.0
    last_rendered = None

    try:
        print(f"Inference service: {args.inference_service}")
        print(f"Head camera topic: {args.topic}")
        print(f"Depth topic      : {args.depth_topic}")
        print(
            "Score weights    : "
            f"depth={args.depth_weight:.2f}, "
            f"bottom-center={args.bottom_center_weight:.2f}, "
            f"area={args.area_weight:.2f}"
        )
        print("Waiting for the shared TensorRT inference service...")
        inference_client = RosObbInferenceClient(
            args.inference_service,
            args.service_timeout,
        )
        print("Inference service: connected")

        subscriber = LatestFrameSubscriber(args.topic)
        depth_subscriber = LatestDepthSubscriber(args.depth_topic)
        print("Waiting for the first color and depth frames...")
        subscriber.wait_for_first_frame(args.camera_timeout)
        depth_subscriber.wait_for_first_frame(args.camera_timeout)
        if args.enable_base_servo:
            from geometry_msgs.msg import PointStamped

            print(
                "Base pose servo  : ENABLED "
                f"(kp={args.servo_kp:.2f}, "
                f"deadband={args.servo_deadband:.1f}deg, "
                f"yaw_min={args.servo_min_yaw_rate:.1f}deg/s, "
                f"yaw_limit={args.servo_max_yaw_rate:.1f}deg/s, "
                f"angle_offset={args.servo_angle_offset:+.1f}deg, "
                f"target_xy=({args.servo_target_x:.3f},"
                f"{args.servo_target_y:.3f})m, "
                f"xy_limit={args.servo_max_linear_speed:.3f}m/s, "
                f"signs=yaw:{args.servo_yaw_sign:+.0f},"
                f"xy:{args.servo_position_sign:+.0f}, "
                f"timeout={args.servo_max_runtime:.1f}s)"
            )
            position_projector = CameraPointProjector(
                args.camera_info_topic,
                args.servo_base_frame,
                args.camera_timeout,
            )
            print(
                "Position frame   : "
                f"{position_projector.camera_frame} -> "
                f"{position_projector.base_frame}"
            )
            result_projector = CameraPointProjector(
                args.camera_info_topic,
                args.servo_result_frame,
                args.camera_timeout,
            )
            result_publisher = rospy.Publisher(
                args.servo_result_topic,
                PointStamped,
                queue_size=1,
                latch=True,
            )
            print(
                "Result interface : "
                f"{args.servo_result_topic} "
                f"[geometry_msgs/PointStamped, "
                f"frame={result_projector.base_frame}]"
            )
            target_tracker = LockedObbTracker(
                args.servo_max_center_jump,
                args.servo_max_depth_jump,
            )
            base_servo = BasePoseVisualServo(
                gain=args.servo_kp,
                deadband_deg=args.servo_deadband,
                max_yaw_rate_deg_s=args.servo_max_yaw_rate,
                angle_filter_alpha=args.servo_angle_alpha,
                aligned_frames=args.servo_aligned_frames,
                command_interval=args.servo_command_interval,
                command_duration=args.servo_command_duration,
                max_runtime=args.servo_max_runtime,
                target_x_m=args.servo_target_x,
                target_y_m=args.servo_target_y,
                position_gain=args.servo_position_kp,
                position_deadband_m=args.servo_position_deadband,
                max_linear_speed_m_s=args.servo_max_linear_speed,
                acquire_frames=args.servo_acquire_frames,
                yaw_sign=args.servo_yaw_sign,
                position_sign=args.servo_position_sign,
                min_yaw_rate_deg_s=args.servo_min_yaw_rate,
                angle_offset_deg=args.servo_angle_offset,
                cmd_vel_topic=args.cmd_vel_topic,
                vel_control_service=args.vel_control_service,
                control_timeout=args.servo_control_timeout,
            )
        else:
            print(
                "Base pose servo  : disabled; inference will not move the chassis"
            )
        if args.no_display:
            print("Running without display; press Ctrl-C to stop.")
        else:
            print("Controls: Q/ESC=quit, S=save, R=recapture pose reference")
            cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

        while not rospy.is_shutdown():
            trigger_frame, sequence, _ = subscriber.latest()
            if trigger_frame is None or sequence == last_sequence:
                rospy.sleep(0.001)
                continue
            last_sequence = sequence
            loop_start = time.perf_counter()

            (
                detections,
                inference_ms,
                color_stamp,
                source_stamp_ns,
                source_shape,
            ) = inference_client.infer(
                args.conf,
                args.iou,
            )
            if source_stamp_ns == last_source_stamp_ns:
                continue
            last_source_stamp_ns = source_stamp_ns
            frame = subscriber.frame_for_stamp(source_stamp_ns)
            if frame is None:
                rospy.logwarn_throttle(
                    2.0,
                    "Skipped OBB result: source frame %d is not in the "
                    "local color-frame history",
                    source_stamp_ns,
                )
                continue
            if frame.shape[:2] != source_shape:
                raise RuntimeError(
                    "OBB source image size does not match the local frame: "
                    f"service={source_shape}, local={frame.shape[:2]}"
                )

            depth_m, depth_stamp = depth_subscriber.latest()
            depth_age = abs(color_stamp - depth_stamp)
            if depth_m is not None and depth_age <= args.max_depth_age:
                (
                    selected_detections,
                    depth_readings,
                    score_details,
                    scored_detections,
                ) = select_best_obb(
                    detections,
                    depth_m,
                    frame.shape,
                    args.depth_shrink,
                    args.min_depth,
                    args.max_depth,
                    args.depth_mad_scale,
                    args.depth_outlier_floor,
                    args.min_depth_samples,
                    args.depth_weight,
                    args.bottom_center_weight,
                    args.area_weight,
                )
            else:
                selected_detections = []
                depth_readings = [None] * len(detections)
                score_details = [None] * len(detections)
                scored_detections = []
            if target_tracker is not None:
                tracked_detection = target_tracker.select(scored_detections)
                selected_detections = (
                    [tracked_detection] if tracked_detection is not None else []
                )
            valid_depth_count = sum(
                distance is not None for distance in depth_readings
            )
            servo_raw_angle_deg = (
                horizontal_rotation_deg(selected_detections[0][0])
                if selected_detections
                else None
            )
            if base_servo is not None:
                servo_position = None
                if selected_detections:
                    selected = selected_detections[0]
                    try:
                        servo_position = position_projector.project(
                            selected[0][0],
                            selected[2],
                            color_stamp,
                        )
                    except Exception as error:
                        rospy.logwarn_throttle(
                            2.0,
                            f"Carton position projection failed: {error}",
                        )
                servo_state = base_servo.update(
                    servo_raw_angle_deg,
                    servo_position,
                )
            else:
                servo_state = {
                    "status": "disabled",
                    "angle_deg": None,
                    "position": None,
                    "reference": None,
                    "position_error": None,
                    "linear_x": 0.0,
                    "linear_y": 0.0,
                    "yaw_deg_s": 0.0,
                }
            servo_status = servo_state["status"]
            servo_filtered_angle_deg = servo_state["angle_deg"]
            servo_current_position = servo_state["position"]
            servo_target_position = servo_state["reference"]
            servo_position_error = servo_state["position_error"]
            servo_linear_x = servo_state["linear_x"]
            servo_linear_y = servo_state["linear_y"]
            servo_command_deg_s = servo_state["yaw_deg_s"]
            if (
                servo_status == "aligned"
                and not result_published
                and selected_detections
                and result_projector is not None
                and result_publisher is not None
            ):
                selected = selected_detections[0]
                try:
                    result_position = result_projector.project(
                        selected[0][0],
                        selected[2],
                        color_stamp,
                    )
                    result_message = PointStamped()
                    result_message.header.stamp = rospy.Time.from_sec(
                        color_stamp
                    )
                    result_message.header.frame_id = (
                        result_projector.base_frame
                    )
                    result_message.point.x = float(result_position[0])
                    result_message.point.y = float(result_position[1])
                    result_message.point.z = float(result_position[2])
                    result_publisher.publish(result_message)
                    result_published = True
                    print(
                        "Servo result     : "
                        f"frame={result_message.header.frame_id}, "
                        f"xyz=({result_message.point.x:.4f}, "
                        f"{result_message.point.y:.4f}, "
                        f"{result_message.point.z:.4f})m, "
                        f"topic={args.servo_result_topic}"
                    )
                except Exception as error:
                    rospy.logwarn_throttle(
                        2.0,
                        f"Failed to publish servo result: {error}",
                    )

            elapsed = max(time.perf_counter() - loop_start, 1e-6)
            instant_fps = 1.0 / elapsed
            if smoothed_fps == 0.0:
                smoothed_fps = instant_fps
            else:
                smoothed_fps = 0.9 * smoothed_fps + 0.1 * instant_fps
            rendered = draw_detections(
                frame,
                selected_detections,
                inference_ms,
                smoothed_fps,
                args.conf,
                len(detections),
                valid_depth_count,
                depth_age * 1000.0,
                base_servo is not None,
                servo_status,
                servo_raw_angle_deg,
                servo_filtered_angle_deg,
                servo_position_error,
                servo_linear_x,
                servo_linear_y,
                servo_command_deg_s,
            )
            last_rendered = rendered

            processed_frames += 1
            if processed_frames == 1 or processed_frames % 30 == 0:
                depths_text = ", ".join(
                    "invalid" if distance is None else f"{distance:.3f}m"
                    for distance in depth_readings
                )
                scores_text = ", ".join(
                    "invalid"
                    if details is None
                    else (
                        f"T={details[0]:.3f}/D={details[1]:.3f}/"
                        f"P={details[2]:.3f}/A={details[3]:.3f}"
                    )
                    for details in score_details
                )
                best_text = (
                    f"score={selected_detections[0][3]:.3f},"
                    f"depth={selected_detections[0][2]:.3f}m"
                    if selected_detections
                    else "N/A"
                )
                angle_text = (
                    f"{horizontal_rotation_deg(selected_detections[0][0]):+.1f}deg"
                    if selected_detections
                    else "N/A"
                )
                if (
                    servo_current_position is not None
                    and servo_target_position is not None
                    and servo_position_error is not None
                ):
                    position_text = (
                        "xy=("
                        f"{servo_current_position[0]:.3f},"
                        f"{servo_current_position[1]:.3f})->("
                        f"{servo_target_position[0]:.3f},"
                        f"{servo_target_position[1]:.3f})m "
                        "dxy=("
                        f"{servo_position_error[0]:+.3f},"
                        f"{servo_position_error[1]:+.3f})m"
                    )
                else:
                    position_text = "xy=N/A"
                print(
                    f"frame={processed_frames} candidates={len(detections)} "
                    f"depths=[{depths_text}] scores=[{scores_text}] "
                    f"best={best_text} "
                    f"angle={angle_text} "
                    f"servo={servo_status} "
                    f"{position_text} "
                    f"vx={servo_linear_x:+.3f}m/s "
                    f"vy={servo_linear_y:+.3f}m/s "
                    f"vyaw={servo_command_deg_s:+.1f}deg/s "
                    f"depth_age={depth_age * 1000.0:.1f}ms "
                    f"infer={inference_ms:.2f}ms fps={smoothed_fps:.1f}"
                )

            if not args.no_display:
                cv2.imshow(WINDOW_NAME, rendered)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")):
                    break
                if key == ord("s"):
                    timestamp = time.strftime("%Y%m%d_%H%M%S")
                    save_path = save_dir / f"carton_obb_trt_{timestamp}.jpg"
                    if cv2.imwrite(
                        str(save_path),
                        rendered,
                        [cv2.IMWRITE_JPEG_QUALITY, 95],
                    ):
                        print(f"Saved result: {save_path}")
                    else:
                        print(f"Failed to save result: {save_path}")
                if key == ord("r") and base_servo is not None:
                    target_tracker.reset()
                    base_servo.reset_reference()
                    result_published = False
                    print("Visual-servo tracking reset; fixed XY target retained.")

            if args.max_frames and processed_frames >= args.max_frames:
                break
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        if output_path is not None and last_rendered is not None:
            if cv2.imwrite(
                str(output_path),
                last_rendered,
                [cv2.IMWRITE_JPEG_QUALITY, 95],
            ):
                print(f"Saved final result: {output_path}")
            else:
                print(f"Failed to save final result: {output_path}")
        if base_servo is not None:
            base_servo.close()
        if subscriber is not None:
            subscriber.close()
        if depth_subscriber is not None:
            depth_subscriber.close()
        if inference_client is not None:
            inference_client.close()
        if result_publisher is not None:
            result_publisher.unregister()
        cv2.destroyAllWindows()
        rospy.signal_shutdown("carton OBB real-time inference stopped")
        print(f"Stopped cleanly after {processed_frames} inferred frames.")


if __name__ == "__main__":
    main()
