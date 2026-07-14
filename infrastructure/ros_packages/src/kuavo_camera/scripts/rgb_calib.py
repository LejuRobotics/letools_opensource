#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RGB 标定相关节点：~mode=camera_info | tar_watch | coordinate

Launch：rgb_calib.launch（相机+腕部内参覆盖；可选辨识协调器或 GUI）。

协调器（~mode=coordinate）在 ~event_topic（默认 /rgb_calib/phase）仅发布进度，类型 std_msgs/String：
  CAPTURING / CALIB_RUNNING / CALIB_DONE / CALIB_FAILED / WAITING_SESSION_DONE / SESSION_FINISHED

  话题模式（~use_service_flow:=false）：默认先等 ~image_topic 首帧（~wait_for_image，超时
  ~wait_image_timeout_sec），再延时 ~pre_capture_delay_sec 后采图；避免腕部相机未就绪就标定。
  成功写 yaml 后可配置 ~calibration_done_service（对端 std_srvs/Trigger）。

  服务流程（~use_service_flow:=true）：~joint_motion_service 循环至 success → 对端调用
  ~start_calibration → 写 yaml → 可选 ~homing_service → 对端 ~notify_session_done。
"""
import copy
import os
import tarfile
import threading
import time

import rospy
import yaml
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

_MODES = ("camera_info", "tar_watch", "coordinate")

PH_CAPTURING = "CAPTURING"
PH_CALIB_RUNNING = "CALIB_RUNNING"
PH_CALIB_DONE = "CALIB_DONE"
PH_CALIB_FAILED = "CALIB_FAILED"
PH_WAITING_SESSION_DONE = "WAITING_SESSION_DONE"
PH_SESSION_FINISHED = "SESSION_FINISHED"


def _flat(block):
    if block is None:
        return []
    if isinstance(block, dict) and "data" in block:
        block = block["data"]
    return [float(x) for x in block]


def _caminfo_from_yaml(calib, header, width, height):
    m = CameraInfo()
    m.header = header
    m.width = width
    m.height = height
    m.distortion_model = str(calib.get("distortion_model") or "plumb_bob")
    m.K = _flat(calib.get("camera_matrix"))
    m.D = _flat(calib.get("distortion_coefficients"))
    m.R = _flat(calib.get("rectification_matrix"))
    m.P = _flat(calib.get("projection_matrix"))
    if len(m.K) != 9 or len(m.R) != 9 or len(m.P) != 12:
        raise ValueError("bad K/R/P")
    return m


class _CameraInfoOverlay(object):
    def __init__(self):
        v = rospy.get_param("~vendor_topic")
        o = rospy.get_param("~output_topic")
        self._path = rospy.get_param("~calibration_file", "")
        self._cache = self._mt = None
        self._pub = rospy.Publisher(o, CameraInfo, queue_size=2, latch=True)
        rospy.Subscriber(v, CameraInfo, self._cb, queue_size=20)
        rospy.Timer(rospy.Duration(2.0), lambda _e: self._load())

    def _load(self):
        if not self._path or not os.path.isfile(self._path):
            self._cache = self._mt = None
            return None
        try:
            mt = os.path.getmtime(self._path)
            if self._cache is not None and mt == self._mt:
                return self._cache
            with open(self._path, "r") as f:
                self._cache = yaml.safe_load(f)
            self._mt = mt
            rospy.loginfo("calibration: %s", self._path)
        except Exception as exc:
            rospy.logwarn("calibration load: %s", exc)
            self._cache = self._mt = None
        return self._cache

    def _cb(self, msg):
        c = self._load()
        if c is None:
            self._pub.publish(copy.deepcopy(msg))
            return
        iw, ih = c.get("image_width"), c.get("image_height")
        if iw and ih and (int(iw) != msg.width or int(ih) != msg.height):
            rospy.logwarn_throttle(
                30.0, "yaml %sx%s != stream %sx%s", iw, ih, msg.width, msg.height
            )
        try:
            self._pub.publish(_caminfo_from_yaml(c, msg.header, msg.width, msg.height))
        except Exception as exc:
            rospy.logwarn_throttle(10.0, "bad yaml, pass-through: %s", exc)
            self._pub.publish(copy.deepcopy(msg))


def _tar_yaml(path):
    with tarfile.open(path, "r:*") as tar:
        ys = [
            m
            for m in tar.getmembers()
            if m.isfile() and m.name.lower().endswith((".yaml", ".yml"))
        ]
        if not ys:
            return None
        ys.sort(key=lambda m: (0 if "ost.yaml" in m.name.lower() else 1, m.name))
        f = tar.extractfile(ys[0])
        if f is None:
            return None
        return yaml.safe_load(f.read())


class _TarWatch(object):
    def __init__(self):
        self._tar = rospy.get_param("~tar_path", "/tmp/calibrationdata.tar.gz")
        self._out = rospy.get_param("~output_yaml")
        self._name = rospy.get_param("~camera_name", "")
        self._last = 0.0
        rospy.Timer(rospy.Duration(1.0), self._tick)

    def _tick(self, _e):
        try:
            mt = os.stat(self._tar).st_mtime
        except OSError:
            return
        if mt <= self._last:
            return
        self._last = mt
        rospy.sleep(0.6)
        try:
            data = _tar_yaml(self._tar)
        except Exception as exc:
            rospy.logwarn("tar: %s", exc)
            return
        if not data or not isinstance(data, dict):
            rospy.logwarn("tar: no yaml")
            return
        if self._name:
            data["camera_name"] = self._name
        d = os.path.dirname(self._out)
        if d:
            os.makedirs(d, exist_ok=True)
        with open(self._out, "w") as f:
            yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)
        rospy.loginfo("wrote %s", self._out)


class _Coordinator(object):
    def __init__(self):
        import cv2
        import numpy as np
        from cv_bridge import CvBridge

        self._cv2 = cv2
        self._np = np
        self._bridge = CvBridge()
        self._event = rospy.get_param("~event_topic", "/rgb_calib/phase")
        self._control_topic = rospy.get_param("~control_topic", "").strip()
        self._img_top = rospy.get_param("~image_topic", "")
        self._out = rospy.get_param("~output_yaml", "")
        self._cam = rospy.get_param("~camera_name", "").strip()
        # Default to parallel capture for multi-camera (targets+control_topic) sessions.
        self._parallel_capture = bool(rospy.get_param("~parallel_capture", True))
        cols = int(rospy.get_param("~board_cols", 11))
        rows = int(rospy.get_param("~board_rows", 8))
        sq = float(rospy.get_param("~square_size", 0.03))
        self._nmin = int(rospy.get_param("~min_samples", 20))
        self._tmax = float(rospy.get_param("~max_capture_sec", 120.0))
        self._mov = float(rospy.get_param("~min_corner_motion_px", 3.0))
        self._use_service_flow = rospy.get_param("~use_service_flow", False)
        self._joint_motion_service = rospy.get_param("~joint_motion_service", "").strip()
        self._homing_service = rospy.get_param("~homing_service", "").strip()
        self._pre_capture_delay_sec = float(
            max(0.0, rospy.get_param("~pre_capture_delay_sec", 1.0))
        )
        self._wait_for_image = rospy.get_param("~wait_for_image", True)
        self._wait_image_timeout_sec = float(
            max(1.0, rospy.get_param("~wait_image_timeout_sec", 60.0))
        )
        self._calibration_done_service = rospy.get_param(
            "~calibration_done_service", ""
        ).strip()
        self._shutdown_on_session_done = rospy.get_param(
            "~shutdown_on_session_done", True
        )
        self._shutdown_delay_sec = float(
            max(0.0, rospy.get_param("~shutdown_delay_sec", 0.5))
        )

        self._pat = (cols, rows)
        self._obj = np.zeros((cols * rows, 3), np.float32)
        self._obj[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
        self._obj *= sq
        self._lk = threading.Lock()
        self._so, self._si = [], []
        self._last = None
        self._t0 = None
        self._twall = 0.0
        self._sub = None
        self._results = {}  # cam -> {"ok": bool, "out": str, "rms": float|None, "why": str|None}
        self._session_done_at = None
        self._shutdown_requested = False
        self._pending_select = None  # camera name to switch to after calibrating
        # Deferred calibration: collect samples per camera during session,
        # run calibrateCamera for all cameras when SESSION_DONE arrives.
        self._deferred = rospy.get_param("~deferred_calibration", True)
        self._samples = {}  # cam -> {"so": list, "si": list, "w": int, "h": int, "out": str}
        self._caps = {}  # cam -> capture state (parallel mode)
        self._subs = {}  # cam -> rospy.Subscriber (parallel mode)

        # Multi-camera, topic-driven calibration:
        # Lower machine publishes std_msgs/String on ~control_topic.
        #
        # Commands:
        #   SELECT|<camera_name>       begin capture for that camera
        #   MOTION_DONE|<camera_name>  stop capture and calibrate that camera
        #   ABORT|<camera_name>        stop capture without calibrating
        #   SESSION_DONE               publish SESSION_FINISHED
        #
        # Target config is provided via ~targets param:
        #   targets:
        #     head_camera: {image_topic: "...", output_yaml: "..."}
        #     left_wrist_camera: {...}
        # If ~targets is empty, falls back to single-camera params (~image_topic/~output_yaml).
        self._targets = rospy.get_param("~targets", {}) or {}

        if self._use_service_flow:
            self._st = (
                "requesting_motion" if self._joint_motion_service else "waiting_robot"
            )
            self._pub = rospy.Publisher(self._event, String, queue_size=2, latch=True)
            if self._img_top:
                self._sub = rospy.Subscriber(self._img_top, Image, self._on_im, queue_size=1)
            rospy.Service("~start_calibration", Trigger, self._srv_start_calibration)
            rospy.Service("~notify_session_done", Trigger, self._srv_notify_session_done)
            if self._joint_motion_service:
                threading.Thread(target=self._thread_joint_motion_loop, daemon=True).start()
            rospy.loginfo(
                "rgb_calib coordinator (service flow): joint_motion=%r homing=%r",
                self._joint_motion_service or None,
                self._homing_service or None,
            )
        else:
            # topic flow: either auto-start capture (legacy) or wait for ~control_topic
            self._st = "waiting_robot"
            self._pub = rospy.Publisher(self._event, String, queue_size=2, latch=True)
            if self._control_topic:
                rospy.Subscriber(self._control_topic, String, self._on_control, queue_size=20)
                rospy.loginfo("rgb_calib: waiting control on %s", self._control_topic)
                self._pubp("WAITING_CONTROL", "")
                if self._parallel_capture and isinstance(self._targets, dict) and self._targets:
                    self._init_parallel_subscribers()
            else:
                if not self._img_top:
                    raise ValueError("~image_topic is required when ~control_topic is empty")
                self._sub = rospy.Subscriber(self._img_top, Image, self._on_im, queue_size=1)
                threading.Thread(
                    target=self._thread_wait_image_then_capture, daemon=True
                ).start()

    def _pubp(self, phase, detail="", cam_override=None):
        # Include camera name for downstream disambiguation when doing multi-camera sessions.
        # Format:
        #   <PHASE>|<camera>|<detail>
        # Backwards compatibility:
        #   when camera is empty, keep legacy "<PHASE>|<detail>".
        cam = (cam_override if cam_override is not None else (self._cam or "")).strip()
        if cam:
            s = phase + "|" + cam if not detail else phase + "|" + cam + "|" + detail
        else:
            s = phase if not detail else phase + "|" + detail
        self._pub.publish(String(data=s))

    def _init_parallel_subscribers(self):
        # Subscribe to all target image topics up-front so we can capture concurrently.
        for cam, cfg in (self._targets or {}).items():
            cam = (cam or "").strip()
            if not cam:
                continue
            img_top = (cfg.get("image_topic") or "").strip()
            out = (cfg.get("output_yaml") or "").strip()
            if not img_top:
                continue
            self._caps[cam] = {
                "capturing": False,
                "so": [],
                "si": [],
                "last": None,
                "t0": None,
                "twall": 0.0,
                "img_top": img_top,
                "out": out,
            }
            self._subs[cam] = rospy.Subscriber(
                img_top, Image, lambda m, c=cam: self._on_im_parallel(c, m), queue_size=1
            )
        rospy.loginfo("rgb_calib: parallel_capture enabled (%d cameras)", len(self._subs))

    def _parse_control(self, s):
        s = (s or "").strip()
        if not s:
            return None, None
        if "|" in s:
            cmd, arg = s.split("|", 1)
            return cmd.strip().upper(), arg.strip()
        # Convenience: a bare camera name acts as SELECT
        if s.upper() in ("SESSION_DONE", "FINISH", "FINISHED"):
            return "SESSION_DONE", ""
        return "SELECT", s

    def _set_active_target(self, cam):
        cam = (cam or "").strip()
        if not cam:
            return False, "empty camera"
        cfg = None
        if isinstance(self._targets, dict) and cam in self._targets:
            cfg = self._targets.get(cam) or {}
        # Fallback to legacy single-camera config
        if cfg is None:
            cfg = {}
        img_top = (cfg.get("image_topic") or self._img_top or "").strip()
        out = (cfg.get("output_yaml") or self._out or "").strip()
        if not img_top:
            return False, "missing image_topic for %s" % cam
        if not out:
            return False, "missing output_yaml for %s" % cam
        with self._lk:
            # Only allow switching when idle (or switching to same target while capturing).
            # Treat terminal states as idle so the session can continue.
            if self._st in ("failed", "done", "finished"):
                self._st = "waiting_robot"
            if self._st not in ("waiting_robot", "capturing"):
                return False, "busy state %s" % self._st
            self._cam = cam
            self._img_top = img_top
            self._out = out
        # Re-subscribe to the active image topic.
        if self._sub is not None:
            try:
                self._sub.unregister()
            except Exception:
                pass
        self._sub = rospy.Subscriber(self._img_top, Image, self._on_im, queue_size=1)
        return True, "ok"

    def _stop_capture(self):
        with self._lk:
            if self._st != "capturing":
                return False
            self._st = "waiting_robot"
        return True

    def _on_control(self, msg):
        cmd, arg = self._parse_control(getattr(msg, "data", ""))
        if not cmd:
            return
        if cmd == "SESSION_DONE":
            with self._lk:
                # Mark session as ending. If we're calibrating, delay shutdown until
                # the calibration finishes writing its yaml to avoid truncating output.
                self._shutdown_requested = True
                self._st = "finished"
                self._session_done_at = time.time()
            self._pubp(PH_SESSION_FINISHED, "")
            rospy.loginfo("rgb_calib: session finished by control")
            self._note_inflight_as_incomplete()
            if self._deferred:
                self._run_deferred_calibrations()
            self._print_session_summary()
            self._maybe_shutdown_now()
            return

        if cmd in ("SELECT", "START", "CAM", "CAMERA"):
            cam = (arg or "").strip()
            if self._parallel_capture and cam in (self._targets or {}):
                ok, why = self._begin_capture_cam(cam)
                if not ok:
                    self._pubp(PH_CALIB_FAILED, "select_failed:%s" % why, cam_override=cam)
                    rospy.logwarn("rgb_calib: select %r failed: %s", cam, why)
                    self._results[cam] = {
                        "ok": False,
                        "out": "",
                        "rms": None,
                        "why": "select_failed:%s" % why,
                    }
                    return
                rospy.loginfo(
                    "rgb_calib: selected %s (%s) [parallel]",
                    cam,
                    self._caps.get(cam, {}).get("img_top") or "",
                )
                return

            ok, why = self._set_active_target(cam)
            if not ok:
                self._pubp(PH_CALIB_FAILED, "select_failed:%s" % why)
                rospy.logwarn("rgb_calib: select %r failed: %s", cam, why)
                if cam:
                    self._results[cam] = {
                        "ok": False,
                        "out": "",
                        "rms": None,
                        "why": "select_failed:%s" % why,
                    }
                return
            rospy.loginfo("rgb_calib: selected %s (%s)", self._cam, self._img_top)
            # Wait for first frame of the selected topic, then begin capture.
            threading.Thread(
                target=self._thread_wait_image_then_capture, daemon=True
            ).start()
            return

        if cmd in ("MOTION_DONE", "STOP", "END"):
            if self._parallel_capture:
                cam = (arg or "").strip()
                ok, why = self._freeze_samples_cam(cam)
                if not ok:
                    self._pubp(PH_CALIB_FAILED, why, cam_override=cam)
                return

            # Stop capture and calibrate if we have enough samples.
            with self._lk:
                st = self._st
                n = len(self._si)
                need = self._nmin
                cam = self._cam
            if st not in ("capturing", "calibrating"):
                self._pubp(PH_CALIB_FAILED, "motion_done_in_state:%s" % st)
                return
            if st == "capturing" and n < need:
                self._stop_capture()
                self._pubp(PH_CALIB_FAILED, "not_enough_samples:%d/%d" % (n, need))
                rospy.logwarn("rgb_calib: %s not enough samples %d/%d", cam, n, need)
                self._results[cam] = {
                    "ok": False,
                    "out": self._out or "",
                    "rms": None,
                    "why": "not_enough_samples:%d/%d" % (n, need),
                }
                return
            # If already calibrating, just ignore duplicate motion_done.
            if st == "calibrating":
                return
            # Trigger calibration with the captured samples (copy outside lock).
            with self._lk:
                oc = [o.copy() for o in self._so]
                ic = [c.copy() for c in self._si]
                out = self._out
                cam = self._cam
                # Immediately go back to idle so the next SELECT can proceed.
                self._st = "waiting_robot"
            # Derive size from last successful gray conversion is not stored;
            # instead use the current ROS image size as a proxy by waiting for one message.
            try:
                im = rospy.wait_for_message(self._img_top, Image, timeout=2.0)
                img = self._bridge.imgmsg_to_cv2(im, desired_encoding="bgr8")
                h, w = img.shape[0], img.shape[1]
            except Exception:
                # Fallback: assume 848x480 commonly used; calibration will still run but yaml dims may be off.
                w, h = 848, 480
            if self._deferred:
                cam = (cam or "").strip() or "camera"
                self._samples[cam] = {"so": oc, "si": ic, "w": int(w), "h": int(h), "out": out}
                rospy.loginfo(
                    "rgb_calib: %s samples frozen (%d/%d) — will calibrate on SESSION_DONE",
                    cam,
                    len(ic),
                    self._nmin,
                )
            else:
                # Legacy: compute immediately in the foreground.
                self._cal(oc, ic, w, h)
            return

        if cmd == "ABORT":
            self._stop_capture()
            self._pubp(PH_CALIB_FAILED, "aborted")
            return

    def _begin_capture(self):
        with self._lk:
            if self._st != "waiting_robot":
                return False
            self._st = "capturing"
            self._so, self._si = [], []
            self._last = None
            self._t0 = time.monotonic()
            self._twall = 0.0
        self._pubp(PH_CAPTURING, "need %d frames" % self._nmin)
        return True

    def _srv_start_calibration(self, _req):
        if not self._begin_capture():
            return TriggerResponse(False, "state is not waiting_robot")
        return TriggerResponse(True, "capturing")

    def _srv_notify_session_done(self, _req):
        with self._lk:
            if self._st != "waiting_session_done":
                return TriggerResponse(
                    False, "expected waiting_session_done, got %s" % self._st
                )
            self._st = "finished"
        self._pubp(PH_SESSION_FINISHED, "")
        rospy.loginfo("rgb_calib coordinator: session finished")
        return TriggerResponse(True, "ok")

    def _thread_joint_motion_loop(self):
        rate = rospy.Rate(1.0)
        proxy = rospy.ServiceProxy(self._joint_motion_service, Trigger)
        while not rospy.is_shutdown():
            try:
                rospy.wait_for_service(self._joint_motion_service, timeout=2.0)
                resp = proxy()
                if resp.success:
                    rospy.loginfo(
                        "joint_motion_service %s succeeded",
                        self._joint_motion_service,
                    )
                    with self._lk:
                        self._st = "waiting_robot"
                    return
            except rospy.ServiceException as exc:
                rospy.logwarn_throttle(5.0, "joint_motion_service: %s", exc)
            except rospy.ROSException:
                pass
            rate.sleep()

    def _thread_homing_then_wait_notify(self):
        if not self._homing_service:
            return
        rate = rospy.Rate(1.0)
        proxy = rospy.ServiceProxy(self._homing_service, Trigger)
        while not rospy.is_shutdown():
            try:
                rospy.wait_for_service(self._homing_service, timeout=2.0)
                resp = proxy()
                if resp.success:
                    rospy.loginfo(
                        "homing_service %s succeeded; waiting notify_session_done",
                        self._homing_service,
                    )
                    with self._lk:
                        self._st = "waiting_session_done"
                    self._pubp(
                        PH_WAITING_SESSION_DONE,
                        "robot call ~notify_session_done when homed",
                    )
                    return
            except rospy.ServiceException as exc:
                rospy.logwarn_throttle(5.0, "homing_service: %s", exc)
            except rospy.ROSException:
                pass
            rate.sleep()

    def _thread_wait_image_then_capture(self):
        if self._wait_for_image:
            rospy.loginfo(
                "rgb_calib: waiting for first image on %s (timeout %.1fs)",
                self._img_top,
                self._wait_image_timeout_sec,
            )
            try:
                rospy.wait_for_message(
                    self._img_top, Image, timeout=self._wait_image_timeout_sec
                )
            except rospy.ROSException:
                rospy.logerr(
                    "rgb_calib: timeout — no image on %s; check RealSense / USB",
                    self._img_top,
                )
                return
            rospy.loginfo("rgb_calib: first image ok")
        else:
            rospy.loginfo(
                "rgb_calib: wait_for_image off — sleeping %.2fs then capture",
                self._pre_capture_delay_sec,
            )
            rospy.sleep(self._pre_capture_delay_sec)
            self._timer_auto_start_capture()
            return
        if self._pre_capture_delay_sec > 0:
            rospy.loginfo(
                "rgb_calib: extra delay %.2fs before capture", self._pre_capture_delay_sec
            )
            rospy.sleep(self._pre_capture_delay_sec)
        self._timer_auto_start_capture()

    def _timer_auto_start_capture(self, _e=None):
        if rospy.is_shutdown():
            return
        if self._begin_capture():
            rospy.loginfo("rgb_calib: capturing")
        else:
            with self._lk:
                st = self._st
            rospy.logwarn("rgb_calib: start capture skipped (state %s)", st)

    def _begin_capture_cam(self, cam):
        cam = (cam or "").strip()
        c = self._caps.get(cam)
        if not c:
            return False, "unknown camera"
        img_top = c.get("img_top") or ""
        if self._wait_for_image:
            rospy.loginfo(
                "rgb_calib: waiting for first image on %s (timeout %.1fs)",
                img_top,
                self._wait_image_timeout_sec,
            )
            try:
                rospy.wait_for_message(img_top, Image, timeout=self._wait_image_timeout_sec)
            except rospy.ROSException:
                rospy.logerr(
                    "rgb_calib: timeout — no image on %s; check driver / USB",
                    img_top,
                )
                return False, "timeout_no_image"
            rospy.loginfo("rgb_calib: first image ok (%s)", cam)
        if self._pre_capture_delay_sec > 0:
            rospy.loginfo(
                "rgb_calib: extra delay %.2fs before capture (%s)",
                self._pre_capture_delay_sec,
                cam,
            )
            rospy.sleep(self._pre_capture_delay_sec)
        with self._lk:
            c = self._caps.get(cam)
            if not c:
                return False, "unknown camera"
            c["capturing"] = True
            c["so"], c["si"] = [], []
            c["last"] = None
            c["t0"] = time.monotonic()
            c["twall"] = 0.0
        self._pubp(PH_CAPTURING, "need %d frames" % self._nmin, cam_override=cam)
        rospy.loginfo("rgb_calib: capturing (%s)", cam)
        return True, "ok"

    def _freeze_samples_cam(self, cam):
        cam = (cam or "").strip()
        c = self._caps.get(cam)
        if not c:
            return False, "unknown camera"
        with self._lk:
            c = self._caps.get(cam)
            if not c:
                return False, "unknown camera"
            c["capturing"] = False
            n = len(c["si"])
            need = self._nmin
            so = [o.copy() for o in c["so"]]
            si = [p.copy() for p in c["si"]]
            img_top = c.get("img_top") or ""
            out = c.get("out") or (self._targets.get(cam) or {}).get("output_yaml") or ""
        if n < need:
            self._results[cam] = {
                "ok": False,
                "out": out,
                "rms": None,
                "why": "not_enough_samples:%d/%d" % (n, need),
            }
            rospy.logwarn("rgb_calib: %s not enough samples %d/%d", cam, n, need)
            return False, "not_enough_samples:%d/%d" % (n, need)
        w, h = self._read_wh(img_top)
        if self._deferred:
            self._samples[cam] = {"so": so, "si": si, "w": int(w), "h": int(h), "out": out}
            rospy.loginfo(
                "rgb_calib: %s samples frozen (%d/%d) — will calibrate on SESSION_DONE",
                cam,
                len(si),
                self._nmin,
            )
            return True, "ok"
        self._cam = cam
        self._out = out
        self._cal(so, si, int(w), int(h))
        return True, "ok"

    def _read_wh(self, img_top):
        try:
            im = rospy.wait_for_message(img_top, Image, timeout=2.0)
            img = self._bridge.imgmsg_to_cv2(im, desired_encoding="bgr8")
            return img.shape[1], img.shape[0]
        except Exception:
            return 848, 480

    def _thread_calibration_done_service(self):
        name = self._calibration_done_service
        rate = rospy.Rate(2.0)
        proxy = rospy.ServiceProxy(name, Trigger)
        while not rospy.is_shutdown():
            try:
                rospy.wait_for_service(name, timeout=3.0)
                resp = proxy()
                rospy.loginfo(
                    "calibration_done_service %s success=%s %s",
                    name,
                    resp.success,
                    resp.message,
                )
                if resp.success:
                    return
            except rospy.ServiceException as exc:
                rospy.logwarn_throttle(3.0, "calibration_done_service: %s", exc)
            except rospy.ROSException as exc:
                rospy.logwarn_throttle(3.0, "calibration_done_service: %s", exc)
            rate.sleep()

    def _dshift(self, a, b):
        np = self._np
        a, b = a.reshape(-1, 2), b.reshape(-1, 2)
        return float(np.mean(np.linalg.norm(a - b, axis=1)))

    def _on_im(self, msg):
        cv2 = self._cv2
        with self._lk:
            if self._st != "capturing":
                return
            if time.monotonic() - self._t0 > self._tmax:
                self._st = "failed"
                self._pubp(
                    PH_CALIB_FAILED,
                    "timeout %d/%d" % (len(self._si), self._nmin),
                )
                # In topic-driven sessions, allow continuing with the next SELECT.
                if self._control_topic:
                    self._st = "waiting_robot"
                return
        try:
            img = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            rospy.logwarn("cv_bridge: %s", exc)
            return
        g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        fl = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        ok, corners = cv2.findChessboardCorners(g, self._pat, fl)
        if not ok:
            return
        cr = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 60, 0.001)
        corners = cv2.cornerSubPix(g, corners, (11, 11), (-1, -1), cr)
        with self._lk:
            if self._st != "capturing":
                return
            now = time.monotonic()
            if now - self._twall < 0.25:
                return
            if self._last is not None and self._dshift(corners, self._last) < self._mov:
                return
            self._last = corners.copy()
            self._twall = now
            self._so.append(self._obj.copy())
            self._si.append(corners.copy())
            n = len(self._si)
            self._pubp(PH_CAPTURING, "%d/%d" % (n, self._nmin))
            # In topic-driven multi-camera mode, we defer calibration until MOTION_DONE.
            if self._control_topic:
                return
            if n < self._nmin:
                return
            oc = [o.copy() for o in self._so]
            ic = [c.copy() for c in self._si]
            self._st = "calibrating"
        w, h = g.shape[1], g.shape[0]
        self._cal(oc, ic, w, h)

    def _on_im_parallel(self, cam, msg):
        cv2 = self._cv2
        with self._lk:
            c = self._caps.get(cam)
            if not c or not c.get("capturing"):
                return
            t0 = c.get("t0")
            if t0 is not None and time.monotonic() - float(t0) > self._tmax:
                c["capturing"] = False
                self._results[cam] = {
                    "ok": False,
                    "out": c.get("out") or "",
                    "rms": None,
                    "why": "timeout %d/%d" % (len(c["si"]), self._nmin),
                }
                self._pubp(PH_CALIB_FAILED, "timeout", cam_override=cam)
                return
        try:
            img = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            return
        g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        fl = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        ok, corners = cv2.findChessboardCorners(g, self._pat, fl)
        if not ok:
            return
        cr = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 60, 0.001)
        corners = cv2.cornerSubPix(g, corners, (11, 11), (-1, -1), cr)
        with self._lk:
            c = self._caps.get(cam)
            if not c or not c.get("capturing"):
                return
            now = time.monotonic()
            if now - float(c.get("twall") or 0.0) < 0.25:
                return
            last = c.get("last")
            if last is not None and self._dshift(corners, last) < self._mov:
                return
            c["last"] = corners.copy()
            c["twall"] = now
            c["so"].append(self._obj.copy())
            c["si"].append(corners.copy())
            n = len(c["si"])
        self._pubp(PH_CAPTURING, "%d/%d" % (n, self._nmin), cam_override=cam)

    def _run_deferred_calibrations(self):
        if not isinstance(self._targets, dict) or not self._targets:
            return
        cams = list(self._targets.keys())
        cams.sort()
        for cam in cams:
            if rospy.is_shutdown():
                return
            # If already has an explicit result (e.g., select_failed), don't overwrite.
            if cam in self._results and self._results[cam].get("ok") is False:
                continue
            s = self._samples.get(cam)
            if not s:
                continue
            try:
                self._cam = cam
                self._out = s.get("out") or (self._targets.get(cam) or {}).get("output_yaml") or ""
                self._cal(s["so"], s["si"], int(s["w"]), int(s["h"]))
            except Exception as exc:
                self._results[cam] = {
                    "ok": False,
                    "out": s.get("out") or "",
                    "rms": None,
                    "why": "deferred_calibrate_failed:%s" % exc,
                }

    def _cal(self, so, si, w, h):
        cv2, np = self._cv2, self._np
        self._pubp(PH_CALIB_RUNNING, "calibrateCamera")
        try:
            rms, mtx, dist, _, _ = cv2.calibrateCamera(so, si, (w, h), None, None)
        except Exception as exc:
            with self._lk:
                self._st = "failed"
            self._pubp(PH_CALIB_FAILED, str(exc))
            cam = (self._cam or "").strip() or "camera"
            self._results[cam] = {
                "ok": False,
                "out": self._out or "",
                "rms": None,
                "why": "calibrate_failed:%s" % exc,
            }
            # In topic-driven sessions, allow continuing with the next SELECT.
            if self._control_topic:
                with self._lk:
                    self._st = "waiting_robot"
            return
        d5 = dist.flatten()[:5].tolist()
        if len(d5) < 5:
            d5.extend([0.0] * (5 - len(d5)))
        K = mtx.flatten().tolist()
        Rid = np.eye(3).flatten().tolist()
        P = np.zeros((3, 4), dtype=float)
        P[0:3, 0:3] = mtx
        P = P.flatten().tolist()
        data = {
            "image_width": w,
            "image_height": h,
            "camera_name": self._cam or "camera",
            "camera_matrix": {"rows": 3, "cols": 3, "data": K},
            "distortion_model": "plumb_bob",
            "distortion_coefficients": {"rows": 1, "cols": len(d5), "data": d5},
            "rectification_matrix": {"rows": 3, "cols": 3, "data": Rid},
            "projection_matrix": {"rows": 3, "cols": 4, "data": P},
            "reprojection_error_rms": float(rms),
        }
        dname = os.path.dirname(self._out)
        if dname:
            os.makedirs(dname, exist_ok=True)
        with open(self._out, "w") as f:
            yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)
        rospy.loginfo("wrote %s rms=%.4f", self._out, rms)
        cam = (self._cam or "").strip() or "camera"
        self._results[cam] = {
            "ok": True,
            "out": self._out or "",
            "rms": float(rms),
            "why": None,
        }
        with self._lk:
            self._st = "done"
        # Calibration completion is signaled by the yaml being written.
        # Do not publish a "done" phase to reduce coupling with downstream controllers.
        if self._calibration_done_service:
            threading.Thread(
                target=self._thread_calibration_done_service, daemon=True
            ).start()
        if self._use_service_flow and self._homing_service:
            threading.Thread(
                target=self._thread_homing_then_wait_notify, daemon=True
            ).start()
        # If SESSION_DONE was received mid-calibration, finish gracefully now.
        self._finish_session_if_shutdown_requested()
        # If a SELECT was queued during calibration, switch now.
        self._drain_pending_select()

    def _drain_pending_select(self):
        with self._lk:
            nxt = (self._pending_select or "").strip()
            self._pending_select = None
            st = self._st
            finished = st == "finished"
        if not nxt or finished or rospy.is_shutdown():
            return
        ok, why = self._set_active_target(nxt)
        if not ok:
            # If still busy for some other reason, record and stop.
            self._results[nxt] = {
                "ok": False,
                "out": "",
                "rms": None,
                "why": "select_failed:%s" % why,
            }
            rospy.logwarn("rgb_calib: drain queued select %r failed: %s", nxt, why)
            return
        rospy.loginfo("rgb_calib: selected %s (%s) [drained queue]", self._cam, self._img_top)
        threading.Thread(target=self._thread_wait_image_then_capture, daemon=True).start()

    def _print_session_summary(self):
        # Best-effort summary for operators watching the console.
        if not isinstance(self._targets, dict) or not self._targets:
            rospy.loginfo("rgb_calib: session summary: (no targets configured)")
            return
        cams = list(self._targets.keys())
        cams.sort()
        rospy.loginfo("rgb_calib: ===== session summary =====")
        for cam in cams:
            r = self._results.get(cam)
            if not r:
                # Try to read any existing yaml (may be from previous run). We still print it,
                # but tag it as "from_file" so operators know it's not guaranteed to be this session.
                cfg = self._targets.get(cam) or {}
                out = (cfg.get("output_yaml") or "").strip()
                fr = self._read_result_from_yaml(out)
                if fr:
                    rospy.loginfo(
                        "rgb_calib: %s -> OK(from_file) rms=%.4f yaml=%s",
                        cam,
                        float(fr.get("rms") or 0.0),
                        out,
                    )
                else:
                    rospy.loginfo("rgb_calib: %s -> no result", cam)
                continue
            if r.get("ok"):
                rospy.loginfo(
                    "rgb_calib: %s -> OK rms=%.4f yaml=%s",
                    cam,
                    float(r.get("rms") or 0.0),
                    r.get("out") or "",
                )
            else:
                rospy.loginfo(
                    "rgb_calib: %s -> FAIL reason=%s",
                    cam,
                    r.get("why") or "unknown",
                )
        rospy.loginfo("rgb_calib: =========================")

    def _note_inflight_as_incomplete(self):
        # When the operator ends the session early, make that explicit in the summary.
        with self._lk:
            cam = (self._cam or "").strip()
            st = self._st
            n = len(self._si)
            need = self._nmin
            out = self._out or ""
        if not cam:
            return
        if cam in self._results:
            return
        if st in ("capturing", "calibrating"):
            self._results[cam] = {
                "ok": False,
                "out": out,
                "rms": None,
                "why": "session_done_in_state:%s samples:%d/%d" % (st, n, need),
            }

    def _read_result_from_yaml(self, path):
        path = (path or "").strip()
        if not path or not os.path.isfile(path):
            return None
        try:
            with open(path, "r") as f:
                d = yaml.safe_load(f)
            if not isinstance(d, dict):
                return None
            rms = d.get("reprojection_error_rms")
            if rms is None:
                return None
            return {"rms": float(rms)}
        except Exception:
            return None

    def _finish_session_if_shutdown_requested(self):
        with self._lk:
            req = bool(self._shutdown_requested)
            st = self._st
            if req and st != "finished":
                self._st = "finished"
        if not req:
            return
        self._print_session_summary()
        self._maybe_shutdown_now()

    def _maybe_shutdown_now(self):
        if not self._shutdown_on_session_done:
            return
        if self._shutdown_delay_sec > 0:
            rospy.sleep(self._shutdown_delay_sec)
        rospy.loginfo("rgb_calib: shutting down (session finished)")
        rospy.signal_shutdown("session finished")


def main():
    rospy.init_node("rgb_calib", anonymous=False)
    mode = rospy.get_param("~mode", "camera_info")
    if mode not in _MODES:
        rospy.logfatal("~mode must be one of %s", _MODES)
        return
    if mode == "camera_info":
        _CameraInfoOverlay()
    elif mode == "tar_watch":
        _TarWatch()
    else:
        _Coordinator()
    rospy.spin()


if __name__ == "__main__":
    main()
