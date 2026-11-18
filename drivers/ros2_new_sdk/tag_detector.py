#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 2 AprilTag 检测节点（对应 ROS 1 的 apriltag_ros + ar_control）。

检测后端：
  1. 优先使用官方 AprilTag 3 库（libapriltag.so，MIT 检测器 + 位姿估计）。
     这是 ROS 1 apriltag_ros 使用的同一套算法，能稳定识别 tag36h11；
     OpenCV 4.6 的 DICT_APRILTAG_36H11 对真实标签有已知缺陷，不能作为主路径。
  2. 找不到官方库时回退 OpenCV aruco（仅 OpenCV >= 4.7 可用）。

链路：相机彩色图 + camera_info → (id, R, t)（相机光学系）
      → tf2 转到 base_link → MarkerArray（marker.id = tag id）。
"""
import ctypes
import math
import os

import cv2
import numpy as np
import rclpy
from builtin_interfaces.msg import Duration as MsgDuration
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


# --------------------------------------------------------------------------- 数学
def _quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _quat_rotate(q, v):
    x, y, z, w = q
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def _euler_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _mat_to_quat(r):
    m00, m01, m02 = float(r[0][0]), float(r[0][1]), float(r[0][2])
    m10, m11, m12 = float(r[1][0]), float(r[1][1]), float(r[1][2])
    m20, m21, m22 = float(r[2][0]), float(r[2][1]), float(r[2][2])
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w, x, y, z = 0.25 * s, (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        w, x, y, z = (m21 - m12) / s, 0.25 * s, (m01 + m10) / s, (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        w, x, y, z = (m02 - m20) / s, (m01 + m10) / s, 0.25 * s, (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        w, x, y, z = (m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, 0.25 * s
    return (x, y, z, w)


def _decode_gray(msg: Image):
    h, w = int(msg.height), int(msg.width)
    if h <= 0 or w <= 0:
        return None
    enc = (msg.encoding or "").lower()
    try:
        if enc == "rgb8":
            return cv2.cvtColor(np.frombuffer(msg.data, np.uint8).reshape(h, w, 3),
                                cv2.COLOR_RGB2GRAY)
        if enc == "bgr8":
            return cv2.cvtColor(np.frombuffer(msg.data, np.uint8).reshape(h, w, 3),
                                cv2.COLOR_BGR2GRAY)
        if enc in ("mono8", "8uc1"):
            return np.frombuffer(msg.data, np.uint8).reshape(h, w)
    except (TypeError, ValueError):
        return None
    return None


# --------------------------------------------------------------------------- 官方库
class _ImageU8(ctypes.Structure):
    _fields_ = [("width", ctypes.c_int32), ("height", ctypes.c_int32),
                ("stride", ctypes.c_int32), ("buf", ctypes.POINTER(ctypes.c_uint8))]


class _ZArray(ctypes.Structure):
    _fields_ = [("el_sz", ctypes.c_size_t), ("size", ctypes.c_int),
                ("alloc", ctypes.c_int), ("data", ctypes.POINTER(ctypes.c_char))]


class _Detection(ctypes.Structure):
    _fields_ = [("family", ctypes.c_void_p), ("id", ctypes.c_int),
                ("hamming", ctypes.c_int), ("decision_margin", ctypes.c_float),
                ("H", ctypes.c_void_p), ("c", ctypes.c_double * 2),
                ("p", (ctypes.c_double * 2) * 4)]


class _Matd(ctypes.Structure):
    _fields_ = [("nrows", ctypes.c_uint), ("ncols", ctypes.c_uint),
                ("data", ctypes.POINTER(ctypes.c_double))]


class _DetectionInfo(ctypes.Structure):
    _fields_ = [("det", ctypes.POINTER(_Detection)), ("tagsize", ctypes.c_double),
                ("fx", ctypes.c_double), ("fy", ctypes.c_double),
                ("cx", ctypes.c_double), ("cy", ctypes.c_double)]


class _Pose(ctypes.Structure):
    _fields_ = [("R", ctypes.POINTER(_Matd)), ("t", ctypes.POINTER(_Matd))]


_LIB_CANDIDATES = (
    "/opt/ros/jazzy/lib/aarch64-linux-gnu/libapriltag.so",
    "/opt/ros/jazzy/lib/libapriltag.so",
    "/usr/lib/libapriltag.so",
    "/usr/local/lib/libapriltag.so",
)


def _find_library():
    explicit = os.environ.get("LEJU_APRILTAG_LIB", "").strip()
    candidates = ([explicit] if explicit else []) + list(_LIB_CANDIDATES)
    for path in candidates:
        if path and os.path.isfile(path):
            return path
    raise FileNotFoundError(
        "libapriltag.so 未找到；安装 ros-jazzy-apriltag，或设置 LEJU_APRILTAG_LIB")


class NativeApriltag:
    """官方 AprilTag 3 的最小 ctypes 封装（检测 + 位姿）。"""

    def __init__(self, family: str = "tag36h11"):
        self.library_path = _find_library()
        self._lib = ctypes.CDLL(self.library_path)
        create_family = getattr(self._lib, f"{family}_create", None)
        if create_family is None:
            raise RuntimeError(f"libapriltag 不支持家族 {family}")
        self._declare(self._lib)
        self._family_ptr = create_family()
        if not self._family_ptr:
            raise RuntimeError(f"{family}_create() 失败")
        self._detector = self._lib.apriltag_detector_create()
        if not self._detector:
            raise RuntimeError("apriltag_detector_create() 失败")
        self._lib.apriltag_detector_add_family_bits(
            self._detector, self._family_ptr, 2)
        self._family = family
        self._family_destroy = getattr(self._lib, f"{family}_destroy", None)

    @staticmethod
    def _declare(lib):
        lib.image_u8_create.restype = ctypes.POINTER(_ImageU8)
        lib.image_u8_create.argtypes = [ctypes.c_uint, ctypes.c_uint]
        lib.image_u8_destroy.argtypes = [ctypes.POINTER(_ImageU8)]
        lib.apriltag_detector_create.restype = ctypes.c_void_p
        lib.apriltag_detector_destroy.argtypes = [ctypes.c_void_p]
        lib.apriltag_detector_add_family_bits.argtypes = [
            ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int]
        lib.apriltag_detector_detect.restype = ctypes.POINTER(_ZArray)
        lib.apriltag_detector_detect.argtypes = [
            ctypes.c_void_p, ctypes.POINTER(_ImageU8)]
        lib.apriltag_detections_destroy.argtypes = [ctypes.POINTER(_ZArray)]
        lib.estimate_pose_for_tag_homography.argtypes = [
            ctypes.POINTER(_DetectionInfo), ctypes.POINTER(_Pose)]
        lib.matd_destroy.argtypes = [ctypes.POINTER(_Matd)]

    def detect(self, gray, intrinsics, tag_size):
        """返回 [(tag_id, R(3x3), t(3,))]，位姿在相机光学系。"""
        h, w = gray.shape[:2]
        image = self._lib.image_u8_create(w, h)
        if not image:
            return []
        try:
            stride = image.contents.stride
            buf = np.ctypeslib.as_array(
                image.contents.buf, shape=(h * stride,))
            buf[:] = 0
            buf.reshape(h, stride)[:, :w] = gray
            zarray = self._lib.apriltag_detector_detect(self._detector, image)
            if not zarray:
                return []
            results = []
            try:
                za = zarray.contents
                for index in range(za.size):
                    elem = ctypes.cast(
                        ctypes.addressof(za.data.contents) + index * za.el_sz,
                        ctypes.POINTER(ctypes.c_void_p)).contents
                    det = ctypes.cast(elem, ctypes.POINTER(_Detection))
                    info = _DetectionInfo(
                        det=det, tagsize=float(tag_size),
                        fx=float(intrinsics[0][0]), fy=float(intrinsics[1][1]),
                        cx=float(intrinsics[0][2]), cy=float(intrinsics[1][2]))
                    pose = _Pose()
                    self._lib.estimate_pose_for_tag_homography(
                        ctypes.byref(info), ctypes.byref(pose))
                    if not pose.R or not pose.t:
                        continue
                    try:
                        R = np.ctypeslib.as_array(
                            pose.R.contents.data, shape=(9,)).reshape(3, 3).copy()
                        t = np.ctypeslib.as_array(
                            pose.t.contents.data, shape=(3,)).copy()
                    finally:
                        self._lib.matd_destroy(pose.R)
                        self._lib.matd_destroy(pose.t)
                    results.append((int(det.contents.id), R, t))
            finally:
                self._lib.apriltag_detections_destroy(zarray)
            return results
        finally:
            self._lib.image_u8_destroy(image)

    def close(self):
        if getattr(self, "_detector", None):
            self._lib.apriltag_detector_destroy(self._detector)
            self._detector = None
        if getattr(self, "_family_ptr", None):
            if self._family_destroy is not None:
                self._family_destroy(self._family_ptr)
            self._family_ptr = None


class TagDetector(Node):
    def __init__(self):
        super().__init__("tag_detector")
        self.declare_parameter("image_topic", "/sensor/head_camera/color_image_raw")
        self.declare_parameter("camera_info_topic", "/sensor/head_camera/color_info")
        self.declare_parameter("tag_ids_csv", "1,2")
        self.declare_parameter("tag_family", "tag36h11")
        self.declare_parameter("tag_size", 0.10)
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("source_frame", "head_camera_optical_link")
        self.declare_parameter("publish_topic", "/perception/tags")
        self.declare_parameter("process_rate_hz", 10.0)
        self.declare_parameter("publish_static_tf", True)
        self.declare_parameter("camera_base_frame", "head_camera_base")
        self.declare_parameter("camera_optical_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("camera_optical_rpy", [-1.5707963, 0.0, -1.5707963])

        self._tag_ids = self._parse_ids(self.get_parameter("tag_ids_csv").value)
        self._family = str(self.get_parameter("tag_family").value)
        self._tag_size = float(self.get_parameter("tag_size").value)
        self._target = str(self.get_parameter("target_frame").value)
        self._source = str(self.get_parameter("source_frame").value)
        self._rate = max(1.0, float(self.get_parameter("process_rate_hz").value))
        self._publish_static = bool(self.get_parameter("publish_static_tf").value)
        self._camera_base = str(self.get_parameter("camera_base_frame").value)
        self._optical_xyz = list(self.get_parameter("camera_optical_xyz").value)
        self._optical_rpy = list(self.get_parameter("camera_optical_rpy").value)

        self._native = None
        self._aruco_dict = None
        self._aruco_params = None
        try:
            self._native = NativeApriltag(self._family)
            self.get_logger().info(
                f"检测后端: libapriltag ({self._native.library_path}), family={self._family}")
        except Exception as exc:
            self.get_logger().warning(
                f"官方 apriltag 库不可用（{exc}），回退 OpenCV aruco；"
                "OpenCV 4.6 对 tag36h11 有缺陷，建议安装 ros-jazzy-apriltag")
            self._aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36H11)
            self._aruco_params = cv2.aruco.DetectorParameters_create()

        self._K = None
        self._D = np.zeros(5, np.float64)
        self._K_matrix = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
        self._last_process = 0.0
        self._tf_failures = 0

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._static_broadcaster = StaticTransformBroadcaster(self)
        self._static_done = False
        self._static_attempts = 0
        self._static_timer = self.create_timer(1.0, self._maybe_publish_static_tf)

        self._pub = self.create_publisher(
            MarkerArray, str(self.get_parameter("publish_topic").value), 10)
        self.create_subscription(
            CameraInfo, str(self.get_parameter("camera_info_topic").value),
            self._on_info, qos_profile_sensor_data)
        self.create_subscription(
            Image, str(self.get_parameter("image_topic").value),
            self._on_image, qos_profile_sensor_data)
        self.get_logger().info(
            "tag_detector: image=%s info=%s tags=%s size=%.3f %s->%s pub=%s" % (
                self.get_parameter("image_topic").value,
                self.get_parameter("camera_info_topic").value,
                self._tag_ids, self._tag_size, self._source, self._target,
                self.get_parameter("publish_topic").value))

    @staticmethod
    def _parse_ids(text):
        ids = []
        for part in str(text or "").replace(";", ",").split(","):
            part = part.strip()
            if part:
                ids.append(int(part))
        return ids

    def _on_info(self, msg: CameraInfo):
        k = list(msg.k)
        self._K_matrix = np.array([
            [k[0], 0.0, k[2]],
            [0.0, k[4], k[5]],
            [0.0, 0.0, 1.0],
        ], np.float64)
        self._D = np.array(msg.d if msg.d else [0.0] * 5, np.float64)
        self._K = self._K_matrix

    def _maybe_publish_static_tf(self):
        if self._static_done:
            return
        if not self._publish_static:
            self._static_done = True
            return
        self._static_attempts += 1
        try:
            exists = self._tf_buffer.can_transform(
                self._camera_base, self._source, Time())
        except Exception:
            exists = False
        if exists:
            self._static_done = True
            self.get_logger().info(
                f"静态 TF 已存在，跳过发布: {self._camera_base} -> {self._source}")
            return
        if self._static_attempts < 2:
            return
        self._static_done = True
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self._camera_base
        t.child_frame_id = self._source
        t.transform.translation.x = float(self._optical_xyz[0])
        t.transform.translation.y = float(self._optical_xyz[1])
        t.transform.translation.z = float(self._optical_xyz[2])
        q = _euler_to_quat(float(self._optical_rpy[0]),
                           float(self._optical_rpy[1]),
                           float(self._optical_rpy[2]))
        t.transform.rotation.x, t.transform.rotation.y = q[0], q[1]
        t.transform.rotation.z, t.transform.rotation.w = q[2], q[3]
        self._static_broadcaster.sendTransform(t)
        self.get_logger().warning(
            f"已发布静态 TF {self._camera_base} -> {self._source}"
            f"（rpy={self._optical_rpy}）；安装角度变化后需要重新标定")

    def _detect(self, gray):
        """统一返回 [(id, R, t)]，位姿在相机光学系。"""
        if self._native is not None:
            return self._native.detect(gray, self._K_matrix, self._tag_size)
        # OpenCV 回退（仅 cv2 >= 4.7 对 AprilTag 36h11 可靠）
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self._aruco_dict, parameters=self._aruco_params)
        if ids is None or len(ids) == 0:
            return []
        half = self._tag_size / 2.0
        obj = np.array([[-half, half, 0.0], [half, half, 0.0],
                        [half, -half, 0.0], [-half, -half, 0.0]], np.float32)
        results = []
        for index, tag_id in enumerate(ids.flatten()):
            image_pts = corners[index][0].astype(np.float64)
            try:
                ok, rvec, tvec = cv2.solvePnP(
                    obj, image_pts, self._K_matrix, self._D,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE)
            except cv2.error:
                ok, rvec, tvec = False, None, None
            if not ok:
                ok, rvec, tvec = cv2.solvePnP(
                    obj, image_pts, self._K_matrix, self._D)
            if not ok:
                continue
            R = cv2.Rodrigues(rvec)[0]
            t = tvec.reshape(3)
            results.append((int(tag_id), R, t))
        return results

    def _on_image(self, msg: Image):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_process < 1.0 / self._rate:
            return
        if self._K is None:
            return
        gray = _decode_gray(msg)
        if gray is None:
            return
        self._last_process = now
        detections = self._detect(gray)
        if not detections:
            return

        markers = MarkerArray()
        for tag_id, R_cam, t_cam in detections:
            if self._tag_ids and tag_id not in self._tag_ids:
                continue
            tf = self._lookup_tf()
            if tf is None:
                continue
            tx, ty, tz, rot = tf
            q_tf = (rot.x, rot.y, rot.z, rot.w)
            p = _quat_rotate(q_tf, (float(t_cam[0]), float(t_cam[1]), float(t_cam[2])))
            q_cam = _mat_to_quat(R_cam)
            q = _quat_mul(q_tf, q_cam)

            marker = Marker()
            # 相机 header 是设备时基（Orbbec 972 s），Marker 必须用 ROS 时间。
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = self._target
            marker.ns = "apriltag"
            marker.id = tag_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = p[0] + tx
            marker.pose.position.y = p[1] + ty
            marker.pose.position.z = p[2] + tz
            marker.pose.orientation.x, marker.pose.orientation.y = q[0], q[1]
            marker.pose.orientation.z, marker.pose.orientation.w = q[2], q[3]
            marker.scale.x = self._tag_size
            marker.scale.y = self._tag_size
            marker.scale.z = 0.01
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.2, 0.2, 0.6
            marker.lifetime = MsgDuration(sec=0, nanosec=500000000)
            markers.markers.append(marker)

        if markers.markers:
            self._pub.publish(markers)

    def _lookup_tf(self):
        # 相机时间戳是设备时基，不能拿去查 TF；取最新 TF 足够。
        try:
            tf = self._tf_buffer.lookup_transform(
                self._target, self._source, Time(), timeout=Duration(seconds=0.05))
        except Exception:
            tf = None
        if tf is not None:
            tr = tf.transform.translation
            return (tr.x, tr.y, tr.z, tf.transform.rotation)
        self._tf_failures += 1
        if self._tf_failures % 50 == 1:
            self.get_logger().warning(
                f"TF 查询失败 {self._target} -> {self._source}"
                f"（累计 {self._tf_failures} 次）")
        return None

    def destroy_node(self):
        if self._native is not None:
            try:
                self._native.close()
            except Exception:
                pass
            self._native = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TagDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        # SIGTERM 时 rclpy 已自行 shutdown，这里再调会抛 RCLError；忽略即可。
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
