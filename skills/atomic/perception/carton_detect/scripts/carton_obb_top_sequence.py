#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""OBB 顶层箱序服务。

调用 ``/carton_obb/infer``，忽略手里箱/邻箱类别，对全部框采深度并投影到
``base_link``，再按高度和平面位置生成原先的 ``InferCartonPose.message``。
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path

import rospy
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image

_SCRIPT_DIR = Path(__file__).resolve().parent
_PACKAGE_DIR = _SCRIPT_DIR.parent
sys.path.insert(0, str(_SCRIPT_DIR))
sys.path.insert(0, str(_PACKAGE_DIR))

from realtime_carton_obb_trt import (  # noqa: E402
    CameraPointProjector,
    RosObbInferenceClient,
    ros_depth_to_meters,
    sample_obb_depth,
)
from top_sequence import infer_top_sequence, load_config  # noqa: E402


class DepthCache:
    def __init__(self, topic: str):
        self.lock = threading.Lock()
        self.ready = threading.Event()
        self.error = None
        self.depth_m = None
        self.stamp = 0.0
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
        stamp = message.header.stamp
        if stamp == rospy.Time():
            stamp = rospy.Time.now()
        with self.lock:
            self.depth_m = depth_m
            self.stamp = stamp.to_sec()
            self.error = None
        self.ready.set()

    def wait_for_first_frame(self, timeout: float) -> None:
        if not self.ready.wait(timeout):
            raise TimeoutError(f"{timeout:.1f}s 内没有收到深度图")
        with self.lock:
            if self.error is not None:
                raise RuntimeError(f"深度图解码失败：{self.error}")

    def latest(self):
        with self.lock:
            if self.error is not None:
                raise RuntimeError(f"深度图解码失败：{self.error}")
            if self.depth_m is None:
                return None, 0.0
            return self.depth_m.copy(), self.stamp

    def close(self) -> None:
        self.subscriber.unregister()


class TopSequenceService:
    def __init__(self, args, config):
        from dynamic_biped.srv import InferCartonPose

        self.args = args
        self.config = config
        self.depth = DepthCache(args.depth_topic)
        self.depth.wait_for_first_frame(args.camera_timeout)
        self.projector = CameraPointProjector(
            args.camera_info_topic,
            args.base_frame,
            args.camera_timeout,
        )
        self.inference = RosObbInferenceClient(
            args.infer_service,
            args.camera_timeout,
        )
        self.service = rospy.Service(
            args.service_name,
            InferCartonPose,
            self._handle,
        )

    def _handle(self, _request):
        from dynamic_biped.srv import InferCartonPoseResponse

        response = InferCartonPoseResponse()
        response.success = False
        response.num_instances = 0
        try:
            result = self._infer()
        except Exception as error:
            response.message = f"箱序服务失败：{error}"
            rospy.logwarn(response.message)
            return response
        response.success = bool(result["success"])
        response.message = result["message"]
        for point in result["matched"]:
            pose = Pose()
            pose.position.x = float(point["x"])
            pose.position.y = float(point["y"])
            pose.position.z = float(point["z"])
            pose.orientation.w = 1.0
            response.poses_base_link.append(pose)
            response.bbox_xyxy.extend(
                [
                    float(point["x1"]),
                    float(point["y1"]),
                    float(point["x2"]),
                    float(point["y2"]),
                ]
            )
            response.yaw.append(0.0)
            response.tape_orientation_deg.append(-1)
        response.num_instances = len(result["matched"])
        log = rospy.loginfo if response.success else rospy.logwarn
        log(response.message)
        return response

    def _infer(self):
        if not self.config["slots_calibrated"]:
            return infer_top_sequence([], self.config, 0, 0)

        detections, _inference_ms, color_stamp, _stamp_ns, color_shape = (
            self.inference.infer(
                self.config["confidence"],
                self.config["nms_iou"],
            )
        )
        depth_m, _depth_stamp = self._wait_depth(color_stamp)
        points = []
        for index, (rect, _score) in enumerate(detections):
            distance_m = sample_obb_depth(
                depth_m,
                rect,
                color_shape,
                self.config["depth_shrink"],
                self.config["min_depth_m"],
                self.config["max_depth_m"],
                self.config["depth_mad_scale"],
                self.config["depth_outlier_floor_m"],
                self.config["min_depth_samples"],
            )
            if distance_m is None:
                continue
            position = self.projector.project(rect[0], distance_m, color_stamp)
            x1, y1, x2, y2 = _rect_bounds(rect)
            points.append(
                {
                    "index": index,
                    "x": float(position[0]),
                    "y": float(position[1]),
                    "z": float(position[2]),
                    "x1": x1,
                    "y1": y1,
                    "x2": x2,
                    "y2": y2,
                }
            )
        return infer_top_sequence(
            points,
            self.config,
            returned_count=len(points),
            candidate_count=len(detections),
        )

    def _wait_depth(self, color_stamp: float):
        deadline = time.monotonic() + max(self.config["max_depth_age_s"], 0.05)
        while True:
            depth_m, depth_stamp = self.depth.latest()
            if (
                depth_m is not None
                and abs(color_stamp - depth_stamp) <= self.config["max_depth_age_s"]
            ):
                return depth_m, depth_stamp
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    "彩色帧与深度帧时间差超过 "
                    f"{self.config['max_depth_age_s']:.3f}s"
                )
            rospy.sleep(0.01)

    def close(self) -> None:
        self.service.shutdown()
        self.inference.close()
        self.depth.close()


def _rect_bounds(rect):
    import cv2
    import numpy as np

    corners = cv2.boxPoints(rect).astype(np.float32)
    return (
        float(corners[:, 0].min()),
        float(corners[:, 1].min()),
        float(corners[:, 0].max()),
        float(corners[:, 1].max()),
    )


def _parse_args():
    parser = argparse.ArgumentParser(description="OBB 顶层箱序服务")
    parser.add_argument("--infer-service", default="/carton_obb/infer")
    parser.add_argument("--service-name", default="/carton_obb/top_sequence")
    parser.add_argument("--config", required=True)
    parser.add_argument("--depth-topic", default="/camera/depth/image_raw")
    parser.add_argument("--camera-info-topic", default="/camera/color/camera_info")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--camera-timeout", type=float, default=20.0)
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    config = load_config(args.config)
    rospy.init_node("carton_obb_top_sequence", disable_signals=True)
    server = TopSequenceService(args, config)
    rospy.loginfo(
        "箱序服务已注册：%s，投影坐标系 %s，槽位标定=%s",
        args.service_name,
        args.base_frame,
        config["slots_calibrated"],
    )
    try:
        rospy.spin()
    finally:
        server.close()


if __name__ == "__main__":
    main()
