import json
import math
import os
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional

import rospy
from tf.transformations import euler_from_quaternion

from core.common.logger import get_logger
from core.domain.pose import Pose6D
from core.domain.result import Result

logger = get_logger(__name__)


class BasketVisionClient:
    """Client for basket vision ROS services."""

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        self.config = config or {}
        self.basket_pose_service = self.config.get("basket_pose_service", "/infer_basket_pose")
        self.top_basket_service = self.config.get("top_basket_service", "/infer_top_basket_ids")
        self.timeout = float(self.config.get("timeout", 10.0))
        self.auto_init_node = bool(self.config.get("auto_init_node", True))
        self.node_name = self.config.get("node_name", "kuavo_basket_vision_hardware")
        self.save_images = bool(self.config.get("save_images", True))
        self.image_topic = self.config.get("image_topic", "/camera/color/image_raw")
        self.image_timeout = float(self.config.get("image_timeout", 2.0))
        self._initialized = False

    def initialize(self) -> Result:
        if self._initialized:
            return Result.ok("BasketVisionClient already initialized")

        try:
            if self.auto_init_node and not rospy.core.is_initialized():
                rospy.init_node(self.node_name, anonymous=True)
            self._initialized = True
            return Result.ok("BasketVisionClient initialized")
        except Exception as e:
            logger.error(f"BasketVisionClient initialize failed: {e}", exc_info=True)
            return Result.fail(f"BasketVisionClient initialize failed: {e}")

    def is_ready(self) -> Result:
        init_result = self._ensure_initialized()
        if not init_result.success:
            return init_result

        ready = self._service_ready(self.basket_pose_service) and self._service_ready(self.top_basket_service)
        return Result.ok(
            "Basket vision ready" if ready else "Basket vision not ready",
            data={
                "ready": ready,
                "basket_pose_service": self.basket_pose_service,
                "top_basket_service": self.top_basket_service,
            },
        )

    def wait_until_ready(self) -> Result:
        init_result = self._ensure_initialized()
        if not init_result.success:
            return init_result

        try:
            rospy.wait_for_service(self.basket_pose_service, timeout=self.timeout)
            rospy.wait_for_service(self.top_basket_service, timeout=self.timeout)
            return Result.ok("Basket vision services are ready")
        except Exception as e:
            logger.error(f"Waiting for basket vision failed: {e}", exc_info=True)
            return Result.fail(f"Waiting for basket vision failed: {e}")

    def infer_basket_pose(self) -> Result:
        return self._infer(self.basket_pose_service, include_target=False)

    def infer_top_basket(self) -> Result:
        return self._infer(self.top_basket_service, include_target=True)

    def _infer(self, service_name: str, include_target: bool) -> Result:
        init_result = self._ensure_initialized()
        if not init_result.success:
            return init_result

        try:
            rospy.wait_for_service(service_name, timeout=self.timeout)
            response = rospy.ServiceProxy(service_name, _get_infer_basket_pose_srv())()
            data = self._response_to_data(response)
            if include_target:
                data["target"] = data["baskets"][0] if data["baskets"] else None
            self._save_inference_artifacts(service_name, data)
            if not data["success"]:
                return Result.fail(data.get("message", "Basket vision inference failed"), data=data)
            return Result.ok("Basket vision inference succeeded", data=data)
        except Exception as e:
            logger.error(f"Basket vision inference failed: {e}", exc_info=True)
            return Result.fail(f"Basket vision inference failed: {e}")

    def _response_to_data(self, response) -> Dict[str, Any]:
        poses = list(response.poses_base_link)
        frame_id = "base_link"
        poses_camera = list(getattr(response, "poses_camera_link", []))

        bboxes = _split_bbox(response.bbox_xyxy)
        yaws = list(response.yaw)

        baskets = []
        for index, pose in enumerate(poses):
            pose6d = _pose_to_pose6d(pose)
            camera_pose = poses_camera[index] if index < len(poses_camera) else None
            camera_pose6d = _pose_to_pose6d(camera_pose) if camera_pose is not None else None
            if index < len(yaws) and math.isfinite(yaws[index]):
                pose6d.yaw = yaws[index]
            baskets.append(
                {
                    "index": index,
                    "pose6d": pose6d,
                    "pose6d_list": pose6d.to_list(),
                    "bbox": bboxes[index] if index < len(bboxes) else None,
                    "frame_id": frame_id,
                    "pose_camera_link": _pose_to_dict(camera_pose) if camera_pose is not None else None,
                    "pose6d_camera_link": camera_pose6d,
                    "pose6d_camera_link_list": camera_pose6d.to_list() if camera_pose6d is not None else None,
                    "camera_frame_id": "camera_color_optical_frame" if camera_pose is not None else None,
                }
            )

        ros_response = {
            "poses_camera_link": [_pose_to_dict(pose) for pose in poses_camera],
            "poses_base_link": [_pose_to_dict(pose) for pose in response.poses_base_link],
            "bbox_xyxy": list(response.bbox_xyxy),
            "num_instances": int(response.num_instances),
            "yaw": list(response.yaw),
        }

        return {
            "success": bool(response.success),
            "message": response.message,
            "num_instances": int(response.num_instances),
            "poses_camera_link": ros_response["poses_camera_link"],
            "poses_base_link": ros_response["poses_base_link"],
            "bbox_xyxy": ros_response["bbox_xyxy"],
            "yaw": ros_response["yaw"],
            "baskets": baskets,
            "embodied_compat": _response_to_embodied_compat(response),
            "ros_response": ros_response,
        }

    def _service_ready(self, service_name: str) -> bool:
        try:
            rospy.wait_for_service(service_name, timeout=0.2)
            return True
        except Exception:
            return False

    def _ensure_initialized(self) -> Result:
        if self._initialized:
            return Result.ok("BasketVisionClient initialized")
        return self.initialize()

    def _save_inference_artifacts(self, service_name: str, data: Dict[str, Any]) -> None:
        if not self.save_images:
            return

        try:
            cv2, CvBridge, Image = _get_image_modules()
            run_dir = _resolve_run_dir(self.config)
            image_dir = run_dir / "images"
            internal_dir = run_dir / "internal"
            image_dir.mkdir(parents=True, exist_ok=True)
            internal_dir.mkdir(parents=True, exist_ok=True)

            stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            service_tag = service_name.strip("/").replace("/", "_") or "basket_vision"
            base_name = f"{service_tag}_{stamp}"

            bridge = CvBridge()
            msg = rospy.wait_for_message(self.image_topic, Image, timeout=self.image_timeout)
            image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            input_path = image_dir / f"{base_name}_input.jpg"
            bbox_path = image_dir / f"{base_name}_bbox.jpg"
            pose_path = image_dir / f"{base_name}_pose6d.jpg"
            response_path = internal_dir / f"{base_name}_response.json"

            cv2.imwrite(str(input_path), image)
            bbox_image = image.copy()
            _draw_bboxes(cv2, bbox_image, data.get("bbox_xyxy", []))
            cv2.imwrite(str(bbox_path), bbox_image)

            pose_image = image.copy()
            _draw_pose_overlay(cv2, pose_image, data.get("baskets", []))
            cv2.imwrite(str(pose_path), pose_image)

            crop_paths = _save_bbox_crops(cv2, image, data.get("baskets", []), image_dir, base_name)

            log_data = _json_safe(
                {
                    "service": service_name,
                    "image_topic": self.image_topic,
                    "input_image": str(input_path),
                    "bbox_image": str(bbox_path),
                    "pose6d_image": str(pose_path),
                    "crop_images": [str(path) for path in crop_paths],
                    "response": data,
                }
            )
            response_path.write_text(json.dumps(log_data, ensure_ascii=False, indent=2), encoding="utf-8")

            data["artifacts"] = {
                "run_dir": str(run_dir),
                "input_image": str(input_path),
                "bbox_image": str(bbox_path),
                "pose6d_image": str(pose_path),
                "crop_images": [str(path) for path in crop_paths],
                "response_json": str(response_path),
            }
        except Exception as e:
            logger.warning(f"Saving basket vision artifacts failed: {e}", exc_info=True)
            data["artifacts"] = {
                "error": str(e),
            }


def _pose_to_pose6d(pose) -> Pose6D:
    quat = [
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ]
    roll, pitch, yaw = euler_from_quaternion(quat)
    return Pose6D(
        x=pose.position.x,
        y=pose.position.y,
        z=pose.position.z,
        yaw=yaw,
        pitch=pitch,
        roll=roll,
    )


def _pose_to_dict(pose) -> Dict[str, Dict[str, float]]:
    return {
        "position": {
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "z": float(pose.position.z),
        },
        "orientation": {
            "x": float(pose.orientation.x),
            "y": float(pose.orientation.y),
            "z": float(pose.orientation.z),
            "w": float(pose.orientation.w),
        },
    }


def _pose_to_translation(pose) -> list:
    return [
        float(pose.position.x),
        float(pose.position.y),
        float(pose.position.z),
    ]


def _get_infer_basket_pose_srv():
    try:
        from basket_vision_msgs.srv import InferBasketPose

        return InferBasketPose
    except ModuleNotFoundError as e:
        raise RuntimeError(
            "basket_vision_msgs is not available. Run: "
            "source ~/kuavo-studio/third_party/basket_vision/basket_vision_ws/devel/setup.bash"
        ) from e


def _pose_to_quaternion(pose) -> list:
    return [
        float(pose.orientation.x),
        float(pose.orientation.y),
        float(pose.orientation.z),
        float(pose.orientation.w),
    ]


def _response_to_embodied_compat(response) -> Dict[str, Any]:
    """Return the basket pose shape consumed by embodied basket behaviors."""
    poses_base = list(response.poses_base_link)
    poses_camera = list(getattr(response, "poses_camera_link", []))
    yaws = list(response.yaw)
    num_instances = int(response.num_instances)

    base_x_min = 0.0
    base_x_max = 2.5
    base_y_min = -0.3
    base_y_max = 0.3
    base_z_min = -1.5
    base_z_max = 2.5

    valid_instances = []
    for index in range(num_instances):
        if index >= len(poses_base):
            continue
        base_pose = poses_base[index]
        if (
            base_x_min <= base_pose.position.x <= base_x_max
            and base_y_min <= base_pose.position.y <= base_y_max
            and base_z_min <= base_pose.position.z <= base_z_max
        ):
            camera_pose = poses_camera[index] if index < len(poses_camera) else None
            valid_instances.append(
                {
                    "index": index,
                    "base_pose": _pose_to_dict(base_pose),
                    "camera_pose": _pose_to_dict(camera_pose) if camera_pose else None,
                    "yaw_rad": float(yaws[index]) if index < len(yaws) else 0.0,
                    "t_base": _pose_to_translation(base_pose),
                    "q_base": _pose_to_quaternion(base_pose),
                    "t_camera": _pose_to_translation(camera_pose) if camera_pose else None,
                    "q_camera": _pose_to_quaternion(camera_pose) if camera_pose else None,
                }
            )

    selected = valid_instances[0] if valid_instances else None
    return {
        "source": "service_selected_pose" if selected else "no_valid_instance",
        "t_base": selected["t_base"] if selected else None,
        "q_base": selected["q_base"] if selected else None,
        "message": (
            f"{response.message} | selected first embodied-compatible instance"
            if selected
            else f"{response.message} | no embodied-compatible instance"
        ),
        "num_instances": num_instances,
        "valid_instances": len(valid_instances),
        "base_pose": selected["base_pose"] if selected else None,
        "camera_pose": selected["camera_pose"] if selected else None,
        "yaw_rad": selected["yaw_rad"] if selected else 0.0,
        "t_camera": selected["t_camera"] if selected else None,
        "q_camera": selected["q_camera"] if selected else None,
        "filter": {
            "frame": "base_link",
            "base_x_min": base_x_min,
            "base_x_max": base_x_max,
            "base_y_min": base_y_min,
            "base_y_max": base_y_max,
            "base_z_min": base_z_min,
            "base_z_max": base_z_max,
        },
    }


def _split_bbox(flat_bbox):
    values = list(flat_bbox)
    return [
        values[i : i + 4]
        for i in range(0, len(values), 4)
        if len(values[i : i + 4]) == 4
    ]


def _get_image_modules():
    import cv2
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image

    return cv2, CvBridge, Image


def _resolve_run_dir(config: Dict[str, Any]) -> Path:
    configured = config.get("log_run_dir") or os.environ.get("BASKET_VISION_RUN_DIR")
    if configured:
        return Path(configured)

    studio_dir = Path(
        config.get("kuavo_studio_dir")
        or os.environ.get("KUAVO_STUDIO_DIR")
        or Path(__file__).resolve().parents[3]
    )
    log_root = Path(
        config.get("log_root")
        or os.environ.get("BASKET_VISION_LOG_ROOT")
        or studio_dir / "third_party/basket_vision/logs"
    )
    service_name = config.get("log_service_name") or os.environ.get("BASKET_VISION_SERVICE_NAME", "gdrn_inference")
    service_dir = log_root / service_name
    if service_dir.exists():
        run_dirs = [p for p in service_dir.iterdir() if p.is_dir()]
        if run_dirs:
            return sorted(run_dirs)[-1]

    return service_dir / datetime.now().strftime("%Y%m%d_%H%M%S")


def _draw_bboxes(cv2, image, flat_bbox) -> None:
    for index, bbox in enumerate(_split_bbox(flat_bbox)):
        x1, y1, x2, y2 = [int(round(float(v))) for v in bbox]
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(
            image,
            f"basket#{index}",
            (x1, max(0, y1 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )


def _draw_pose_overlay(cv2, image, baskets) -> None:
    for basket in baskets:
        bbox = basket.get("bbox")
        if not bbox or len(bbox) != 4:
            continue

        x1, y1, x2, y2 = [int(round(float(v))) for v in bbox]
        center_x = int(round((x1 + x2) / 2.0))
        center_y = int(round((y1 + y2) / 2.0))
        index = basket.get("index", 0)
        pose6d = basket.get("pose6d_list") or []

        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.circle(image, (center_x, center_y), 6, (0, 0, 255), -1)
        cv2.drawMarker(
            image,
            (center_x, center_y),
            (0, 0, 255),
            markerType=cv2.MARKER_CROSS,
            markerSize=18,
            thickness=2,
        )

        labels = [
            f"basket#{index}",
            f"center=({center_x},{center_y})",
        ]
        if len(pose6d) == 6:
            labels.extend(
                [
                    f"x={pose6d[0]:.3f} y={pose6d[1]:.3f} z={pose6d[2]:.3f}",
                    f"yaw={pose6d[3]:.3f} pitch={pose6d[4]:.3f} roll={pose6d[5]:.3f}",
                ]
            )

        text_x = max(0, min(x1, image.shape[1] - 1))
        text_y = max(18, y1 - 60)
        for line_index, label in enumerate(labels):
            y = text_y + line_index * 20
            cv2.putText(
                image,
                label,
                (text_x, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )


def _save_bbox_crops(cv2, image, baskets, image_dir: Path, base_name: str) -> list:
    crop_paths = []
    height, width = image.shape[:2]
    for basket in baskets:
        bbox = basket.get("bbox")
        if not bbox or len(bbox) != 4:
            continue

        x1, y1, x2, y2 = [int(round(float(v))) for v in bbox]
        x1 = max(0, min(width - 1, x1))
        x2 = max(0, min(width, x2))
        y1 = max(0, min(height - 1, y1))
        y2 = max(0, min(height, y2))
        if x2 <= x1 or y2 <= y1:
            continue

        index = basket.get("index", len(crop_paths))
        crop_path = image_dir / f"{base_name}_basket_{index}_crop.jpg"
        cv2.imwrite(str(crop_path), image[y1:y2, x1:x2])
        crop_paths.append(crop_path)
    return crop_paths


def _json_safe(value):
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_json_safe(item) for item in value]
    if isinstance(value, tuple):
        return [_json_safe(item) for item in value]
    if isinstance(value, Pose6D):
        return value.to_list()
    if hasattr(value, "to_list"):
        return value.to_list()
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    return repr(value)