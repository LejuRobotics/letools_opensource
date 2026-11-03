#!/usr/bin/env python3
"""ROS1 service that keeps the carton OBB TensorRT engine resident."""

from __future__ import annotations

import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rospy
from box_detection_msgs.srv import (
    InferCartonObb,
    InferCartonObbResponse,
)
from sensor_msgs.msg import Image
from std_msgs.msg import Header


def _repo_root():
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "skills").is_dir() and (
            parent / "infrastructure"
        ).is_dir():
            return parent
    return here.parents[-1]


REPO_ROOT = _repo_root()
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from skills.atomic.perception.carton_detect.trt_backend import (  # noqa: E402
    CartonObbTensorRT,
    DEFAULT_TRT_ENGINE_PATH,
)


def _image_to_bgr(message):
    encodings = {
        "rgb8": (3, cv2.COLOR_RGB2BGR),
        "bgr8": (3, None),
        "rgba8": (4, cv2.COLOR_RGBA2BGR),
        "bgra8": (4, cv2.COLOR_BGRA2BGR),
        "mono8": (1, cv2.COLOR_GRAY2BGR),
        "8UC3": (3, None),
        "8UC1": (1, cv2.COLOR_GRAY2BGR),
    }
    if message.encoding not in encodings:
        raise ValueError(
            f"Unsupported image encoding: {message.encoding}; "
            "expected bgr8/rgb8/mono8/rgba8/bgra8"
        )

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


class CartonObbTrtService:
    def __init__(self):
        configured_engine = str(rospy.get_param("~engine_path", "")).strip()
        engine_path = (
            Path(configured_engine).expanduser()
            if configured_engine
            else DEFAULT_TRT_ENGINE_PATH
        )
        warmup_count = int(rospy.get_param("~warmup", 5))
        self.service_name = rospy.get_param(
            "~service_name",
            "/carton_obb/infer",
        )
        self.image_topic = rospy.get_param(
            "~image_topic",
            "/camera/color/image_raw",
        )
        camera_timeout = float(rospy.get_param("~camera_timeout", 15.0))
        self.max_frame_age = float(rospy.get_param("~max_frame_age", 0.5))
        if camera_timeout <= 0.0:
            raise ValueError("~camera_timeout must be greater than zero")
        if self.max_frame_age <= 0.0:
            raise ValueError("~max_frame_age must be greater than zero")

        self._frame_lock = threading.Lock()
        self._frame_ready = threading.Event()
        self._inference_lock = threading.Lock()
        self._latest_frame = None
        self._latest_header = None
        self._latest_received_at = 0.0
        self.backend = None
        self.service = None
        self.image_subscriber = rospy.Subscriber(
            self.image_topic,
            Image,
            self._image_callback,
            queue_size=1,
            buff_size=16 * 1024 * 1024,
        )
        rospy.loginfo(
            "[carton_obb_trt_service] waiting for head camera: %s",
            self.image_topic,
        )
        if not self._frame_ready.wait(camera_timeout):
            self.image_subscriber.unregister()
            raise RuntimeError(
                f"No head-camera image received from {self.image_topic} "
                f"within {camera_timeout:.1f}s"
            )

        self.backend = CartonObbTensorRT(engine_path)
        rospy.loginfo(
            "[carton_obb_trt_service] TensorRT %s loaded %s, input=%s",
            self.backend.tensorrt_version,
            self.backend.engine_path,
            self.backend.input_shape,
        )
        self.backend.warmup(warmup_count)
        rospy.loginfo(
            "[carton_obb_trt_service] warmup completed: %d runs",
            warmup_count,
        )
        self.service = rospy.Service(
            self.service_name,
            InferCartonObb,
            self._handle,
        )
        rospy.on_shutdown(self.close)
        rospy.loginfo(
            "[carton_obb_trt_service] ready: %s, latest frames from %s",
            self.service_name,
            self.image_topic,
        )

    def _image_callback(self, message):
        try:
            image = _image_to_bgr(message)
            header = Header(
                seq=message.header.seq,
                stamp=message.header.stamp,
                frame_id=message.header.frame_id,
            )
            if header.stamp == rospy.Time():
                header.stamp = rospy.Time.now()
        except Exception as error:
            rospy.logerr_throttle(
                2.0,
                "[carton_obb_trt_service] head-camera decode failed: %s",
                error,
            )
            return

        with self._frame_lock:
            self._latest_frame = image
            self._latest_header = header
            self._latest_received_at = time.monotonic()
        self._frame_ready.set()

    def _latest_frame_snapshot(self):
        with self._frame_lock:
            if self._latest_frame is None or self._latest_header is None:
                raise RuntimeError(
                    f"No head-camera frame available from {self.image_topic}"
                )
            frame_age = time.monotonic() - self._latest_received_at
            if frame_age > self.max_frame_age:
                raise RuntimeError(
                    f"Latest head-camera frame is stale: {frame_age:.3f}s "
                    f"> {self.max_frame_age:.3f}s"
                )
            return (
                self._latest_frame.copy(),
                Header(
                    seq=self._latest_header.seq,
                    stamp=self._latest_header.stamp,
                    frame_id=self._latest_header.frame_id,
                ),
            )

    def _handle(self, request):
        response = InferCartonObbResponse()
        try:
            # Snapshot only when this request reaches the execution slot.
            with self._inference_lock:
                image, source_header = self._latest_frame_snapshot()
                detections, inference_ms = self.backend.infer_bgr(
                    image,
                    confidence=float(request.confidence),
                    nms_iou=float(request.nms_iou),
                )
            response.source_header = source_header
            response.image_width = image.shape[1]
            response.image_height = image.shape[0]
            for rect, score in detections:
                response.center_x.append(float(rect[0][0]))
                response.center_y.append(float(rect[0][1]))
                response.width.append(float(rect[1][0]))
                response.height.append(float(rect[1][1]))
                response.angle_deg.append(float(rect[2]))
                response.scores.append(float(score))
            response.success = True
            response.message = ""
            response.inference_ms = float(inference_ms)
        except Exception as error:
            response.success = False
            response.message = str(error)
            response.inference_ms = 0.0
            rospy.logerr_throttle(
                2.0,
                "[carton_obb_trt_service] inference failed: %s",
                error,
            )
        return response

    def close(self):
        if self.service is not None:
            self.service.shutdown("carton OBB TensorRT service is stopping")
            self.service = None
        if self.image_subscriber is not None:
            self.image_subscriber.unregister()
            self.image_subscriber = None
        if self.backend is not None:
            self.backend.close()
            self.backend = None
            rospy.loginfo("[carton_obb_trt_service] TensorRT resources released")


def main():
    rospy.init_node("carton_obb_trt_service")
    CartonObbTrtService()
    rospy.spin()


if __name__ == "__main__":
    main()
