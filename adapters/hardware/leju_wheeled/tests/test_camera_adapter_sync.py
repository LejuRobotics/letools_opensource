"""CameraAdapter RGBD synchronization tests without a camera or ROS master."""

import threading
import unittest
from unittest.mock import Mock, patch

import message_filters
import numpy as np
import rospy
from sensor_msgs.msg import Image

from adapters.hardware.leju_wheeled.camera_adapter import CameraAdapter
from core.domain.camera import CameraInfo, CameraIntrinsics


class _FakeBridge:
    def __init__(self):
        self._images = {}
        self.raise_for = set()
        self.encodings = []

    def add(self, msg, image):
        self._images[id(msg)] = image

    def imgmsg_to_cv2(self, msg, desired_encoding):
        self.encodings.append(desired_encoding)
        if id(msg) in self.raise_for:
            raise ValueError("conversion failed")
        return self._images[id(msg)]


def _image_msg(stamp_sec, frame_id):
    msg = Image()
    msg.header.stamp = rospy.Time.from_sec(stamp_sec)
    msg.header.frame_id = frame_id
    return msg


class CameraAdapterSyncTest(unittest.TestCase):
    CAMERA = "camera"
    SLOP = 0.015

    def setUp(self):
        self.adapter = CameraAdapter()
        self.bridge = _FakeBridge()
        self.adapter._bridge = self.bridge
        self.adapter._sync_enabled[self.CAMERA] = True
        self.adapter._sync_sequences[self.CAMERA] = 0
        self.adapter._frame_counts[self.CAMERA] = 0
        self.adapter._last_frame_time[self.CAMERA] = 0.0
        self.adapter._error_counts[self.CAMERA] = {
            "color": 0,
            "depth": 0,
            "rgbd": 0,
            "pointcloud": 0,
        }
        self.adapter._processing_latencies[self.CAMERA] = []
        self.adapter._running_cameras[self.CAMERA] = True
        self.adapter._initialized = True
        self.adapter._is_connected = True

    def _pair(
        self,
        color_stamp,
        depth_stamp,
        color_shape=(4, 6, 3),
        depth_shape=(4, 6),
    ):
        color_msg = _image_msg(color_stamp, "camera_color_optical_frame")
        depth_msg = _image_msg(depth_stamp, "camera_color_optical_frame")
        color = np.zeros(color_shape, dtype=np.uint8)
        depth = np.full(depth_shape, 1000, dtype=np.uint16)
        self.bridge.add(color_msg, color)
        self.bridge.add(depth_msg, depth)
        return color_msg, depth_msg, color, depth

    def _publish_pair(self, color_msg, depth_msg, require_same_resolution=True):
        self.adapter._rgbd_callback(
            color_msg,
            depth_msg,
            self.CAMERA,
            self.SLOP,
            require_same_resolution,
        )

    def test_rgbd_callback_caches_pair_atomically(self):
        color_msg, depth_msg, color, depth = self._pair(10.0, 10.008)

        self._publish_pair(color_msg, depth_msg)

        frame = self.adapter.get_synchronized_camera_frame(self.CAMERA)
        self.assertIsNotNone(frame)
        self.assertIs(self.adapter.get_camera_frame(self.CAMERA), frame)
        self.assertIs(frame.color_image, color)
        self.assertIs(frame.depth_image, depth)
        self.assertAlmostEqual(frame.timestamp, 10.0)
        self.assertAlmostEqual(frame.depth_timestamp, 10.008)
        self.assertAlmostEqual(frame.sync_delta_sec, 0.008)
        self.assertEqual(frame.color_frame_id, "camera_color_optical_frame")
        self.assertEqual(frame.depth_frame_id, "camera_color_optical_frame")
        self.assertEqual(frame.sequence, 1)
        self.assertEqual(self.adapter._frame_counts[self.CAMERA], 1)
        self.assertEqual(self.bridge.encodings, ["rgb8", "passthrough"])

    def test_real_approximate_synchronizer_invokes_adapter_callback(self):
        color_msg, depth_msg, _, _ = self._pair(10.0, 10.008)
        color_filter = message_filters.SimpleFilter()
        depth_filter = message_filters.SimpleFilter()
        synchronizer = message_filters.ApproximateTimeSynchronizer(
            [color_filter, depth_filter],
            queue_size=5,
            slop=self.SLOP,
            allow_headerless=False,
        )
        synchronizer.registerCallback(
            self.adapter._rgbd_callback,
            self.CAMERA,
            self.SLOP,
            True,
        )

        with patch(
            "message_filters.rospy.Time.now",
            return_value=rospy.Time.from_sec(10.01),
        ):
            color_filter.signalMessage(color_msg)
            depth_filter.signalMessage(depth_msg)

        frame = self.adapter.get_synchronized_camera_frame(self.CAMERA)
        self.assertIsNotNone(frame)
        self.assertEqual(frame.sequence, 1)
        self.assertAlmostEqual(frame.sync_delta_sec, 0.008)

    def test_pair_outside_slop_is_rejected(self):
        color_msg, depth_msg, _, _ = self._pair(10.0, 10.02)

        self._publish_pair(color_msg, depth_msg)

        self.assertIsNone(self.adapter.get_synchronized_camera_frame(self.CAMERA))
        self.assertEqual(self.adapter._sync_sequences[self.CAMERA], 0)
        self.assertEqual(self.adapter._error_counts[self.CAMERA]["rgbd"], 1)

    def test_zero_timestamp_is_rejected(self):
        color_msg, depth_msg, _, _ = self._pair(0.0, 0.0)

        self._publish_pair(color_msg, depth_msg)

        self.assertIsNone(self.adapter.get_synchronized_camera_frame(self.CAMERA))
        self.assertEqual(self.adapter._sync_sequences[self.CAMERA], 0)
        self.assertEqual(self.adapter._error_counts[self.CAMERA]["rgbd"], 1)

    def test_resolution_mismatch_is_rejected(self):
        color_msg, depth_msg, _, _ = self._pair(
            10.0,
            10.005,
            color_shape=(4, 6, 3),
            depth_shape=(2, 3),
        )

        self._publish_pair(color_msg, depth_msg)

        self.assertIsNone(self.adapter.get_synchronized_camera_frame(self.CAMERA))
        self.assertEqual(self.adapter._sync_sequences[self.CAMERA], 0)
        self.assertEqual(self.adapter._error_counts[self.CAMERA]["rgbd"], 1)

    def test_conversion_failure_does_not_publish_partial_frame(self):
        color_msg, depth_msg, _, _ = self._pair(10.0, 10.005)
        self.bridge.raise_for.add(id(depth_msg))

        self._publish_pair(color_msg, depth_msg)

        self.assertIsNone(self.adapter.get_synchronized_camera_frame(self.CAMERA))
        self.assertEqual(self.adapter._sync_sequences[self.CAMERA], 0)

    def test_wait_for_next_does_not_return_existing_frame(self):
        color_msg, depth_msg, _, _ = self._pair(10.0, 10.005)
        self._publish_pair(color_msg, depth_msg)

        result = self.adapter.wait_for_next_synchronized_camera_frame(
            self.CAMERA,
            timeout_sec=0.02,
        )

        self.assertFalse(result.success)
        self.assertEqual(result.error_code, "RGBD_SYNC_TIMEOUT")

    def test_wait_rejects_non_finite_timeout(self):
        result = self.adapter.wait_for_next_synchronized_camera_frame(
            self.CAMERA,
            timeout_sec=float("nan"),
        )

        self.assertFalse(result.success)
        self.assertEqual(result.error_code, "INVALID_ARGUMENT")

    def test_wait_for_next_returns_new_sequence(self):
        color_msg_1, depth_msg_1, _, _ = self._pair(10.0, 10.005)
        self._publish_pair(color_msg_1, depth_msg_1)
        color_msg_2, depth_msg_2, _, _ = self._pair(11.0, 11.004)
        timer = threading.Timer(
            0.03,
            self._publish_pair,
            args=(color_msg_2, depth_msg_2),
        )
        timer.start()
        try:
            result = self.adapter.wait_for_next_synchronized_camera_frame(
                self.CAMERA,
                timeout_sec=0.5,
            )
        finally:
            timer.cancel()
            timer.join()

        self.assertTrue(result.success, result.message)
        self.assertEqual(result.data.sequence, 2)
        self.assertAlmostEqual(result.data.timestamp, 11.0)

    def test_shutdown_wakes_waiter(self):
        timer = threading.Timer(0.03, self.adapter.shutdown)
        timer.start()
        try:
            result = self.adapter.wait_for_next_synchronized_camera_frame(
                self.CAMERA,
                timeout_sec=0.5,
            )
        finally:
            timer.cancel()
            timer.join()

        self.assertFalse(result.success)
        self.assertEqual(result.error_code, "CAMERA_SHUTTING_DOWN")

    def test_callbacks_do_not_repopulate_caches_after_shutdown(self):
        color_msg, depth_msg, _, _ = self._pair(10.0, 10.005)
        self.adapter.shutdown()

        self.adapter._color_callback(color_msg, self.CAMERA)
        self.adapter._depth_callback(depth_msg, self.CAMERA)
        self._publish_pair(color_msg, depth_msg)

        self.assertEqual(self.adapter._frames_cache, {})
        self.assertEqual(self.adapter._synchronized_frames_cache, {})
        self.assertEqual(self.adapter._depth_cache, {})
        self.assertEqual(self.adapter._error_counts, {})

    def test_get_depth_data_uses_original_ros_timestamp(self):
        depth_msg = _image_msg(42.125, "camera_color_optical_frame")
        depth = np.full((4, 6), 1000, dtype=np.uint16)
        self.bridge.add(depth_msg, depth)
        self.adapter._depth_camera_infos[self.CAMERA] = CameraInfo(
            camera_type="orbbec",
            resolution=(6, 4),
            frame_rate=30.0,
            intrinsics=CameraIntrinsics(
                fx=100.0,
                fy=100.0,
                cx=3.0,
                cy=2.0,
                width=6,
                height=4,
            ),
        )

        self.adapter._depth_callback(depth_msg, self.CAMERA)
        data = self.adapter.get_depth_data(self.CAMERA)

        self.assertIsNotNone(data)
        self.assertAlmostEqual(data.timestamp, 42.125)
        self.assertIs(data.depth_image, depth)

    def test_setup_subscribers_uses_approximate_sync_when_enabled(self):
        fake_sync = Mock()
        with patch(
            "adapters.hardware.leju_wheeled.camera_adapter.message_filters.Subscriber"
        ) as sync_subscriber, patch(
            "adapters.hardware.leju_wheeled.camera_adapter."
            "message_filters.ApproximateTimeSynchronizer",
            return_value=fake_sync,
        ) as synchronizer_cls, patch(
            "adapters.hardware.leju_wheeled.camera_adapter.rospy.Subscriber"
        ) as rospy_subscriber:
            self.adapter._setup_subscribers({
                "enable_head": True,
                "enable_wrist_camera": False,
                "head_rgbd_sync": True,
                "rgbd_sync_queue_size": 5,
                "rgbd_sync_slop_sec": self.SLOP,
            })

        self.assertEqual(sync_subscriber.call_count, 2)
        synchronizer_cls.assert_called_once()
        _, kwargs = synchronizer_cls.call_args
        self.assertEqual(kwargs["queue_size"], 5)
        self.assertEqual(kwargs["slop"], self.SLOP)
        self.assertFalse(kwargs["allow_headerless"])
        fake_sync.registerCallback.assert_called_once()
        # CameraInfo x2；点云关闭，RGB/depth 不应再建立普通 subscriber。
        self.assertEqual(rospy_subscriber.call_count, 2)
        self.assertIs(self.adapter._synchronizers[self.CAMERA], fake_sync)

    def test_invalid_sync_config_fails_before_starting_roslaunch(self):
        with patch(
            "adapters.hardware.leju_wheeled.camera_adapter.subprocess.Popen"
        ) as popen:
            result = CameraAdapter().initialize({
                "head_rgbd_sync": True,
                "rgbd_sync_queue_size": 0,
            })

        self.assertFalse(result.success)
        self.assertIn("rgbd_sync_queue_size", result.message)
        popen.assert_not_called()

    def test_no_enabled_camera_fails_before_starting_roslaunch(self):
        with patch(
            "adapters.hardware.leju_wheeled.camera_adapter.subprocess.Popen"
        ) as popen:
            result = CameraAdapter().initialize({
                "enable_head": False,
                "enable_wrist_camera": True,
                "has_left_wrist": False,
                "has_right_wrist": False,
            })

        self.assertFalse(result.success)
        self.assertIn("No camera is enabled", result.message)
        popen.assert_not_called()

    def test_legacy_has_head_key_is_rejected(self):
        with patch(
            "adapters.hardware.leju_wheeled.camera_adapter.subprocess.Popen"
        ) as popen:
            result = CameraAdapter().initialize({"has_head": False})

        self.assertFalse(result.success)
        self.assertIn("renamed to 'enable_head'", result.message)
        popen.assert_not_called()

    def test_wrist_master_switch_respects_installed_side_flags(self):
        adapter = CameraAdapter()
        launch_process = Mock(pid=101)
        launch_process.poll.return_value = None

        with patch(
            "adapters.hardware.leju_wheeled.camera_adapter.time.sleep"
        ), patch(
            "adapters.hardware.leju_wheeled.camera_adapter.subprocess.Popen",
            return_value=launch_process,
        ) as popen, patch.object(
            adapter,
            "_setup_subscribers",
        ), patch.object(
            adapter,
            "_wait_for_cameras_ready",
        ) as wait_ready:
            result = adapter.initialize({
                "enable_head": False,
                "enable_wrist_camera": True,
                "has_left_wrist": True,
                "has_right_wrist": False,
                "left_wrist_camera_sn": "left-serial",
            })

        self.assertTrue(result.success, result.message)
        launch_cmd = popen.call_args_list[0].args[0]
        self.assertIn("enable_head:=false", launch_cmd)
        self.assertIn("enable_left_wrist:=true", launch_cmd)
        self.assertIn("enable_right_wrist:=false", launch_cmd)
        wait_ready.assert_called_once_with(
            ["left_wrist_camera"],
            launch_process,
            10.0,
        )
        adapter.shutdown()

    def test_driver_only_starts_driver_without_image_subscribers(self):
        adapter = CameraAdapter()
        launch_process = Mock(pid=101)
        launch_process.poll.return_value = None
        tf_processes = [Mock(pid=102), Mock(pid=103)]

        with patch(
            "adapters.hardware.leju_wheeled.camera_adapter.time.sleep"
        ), patch(
            "adapters.hardware.leju_wheeled.camera_adapter.subprocess.Popen",
            side_effect=[launch_process, *tf_processes],
        ), patch.object(
            adapter,
            "_setup_subscribers",
        ) as setup_subscribers, patch.object(
            adapter,
            "_wait_for_cameras_ready",
        ) as wait_ready:
            result = adapter.initialize({
                "enable_head": True,
                "enable_wrist_camera": False,
                "driver_only": True,
                # 这些均为完整 Adapter 的数据面配置。driver-only 不应读取、
                # 转换或校验它们；需要时由外部数据消费者自行配置。
                "camera_startup_timeout_sec": "unused-invalid-timeout",
                "head_rgbd_sync": True,
                "rgbd_sync_queue_size": "unused-invalid-queue",
                "rgbd_sync_slop_sec": "unused-invalid-slop",
                "rgbd_require_same_resolution": "unused-invalid-flag",
            })

        self.assertTrue(result.success, result.message)
        self.assertTrue(adapter._driver_only)
        setup_subscribers.assert_not_called()
        wait_ready.assert_not_called()
        self.assertEqual(adapter._subscribers, {})
        adapter.shutdown()

    def test_driver_only_requires_boolean_config(self):
        with patch(
            "adapters.hardware.leju_wheeled.camera_adapter.subprocess.Popen"
        ) as popen:
            result = CameraAdapter().initialize({"driver_only": "true"})

        self.assertFalse(result.success)
        self.assertIn("driver_only must be a bool", result.message)
        popen.assert_not_called()

    def test_camera_readiness_times_out_when_launch_lives_without_frames(self):
        adapter = CameraAdapter()
        launch_process = Mock()
        launch_process.poll.return_value = None

        with patch(
            "adapters.hardware.leju_wheeled.camera_adapter.time.monotonic",
            side_effect=[0.0, 2.0],
        ):
            with self.assertRaisesRegex(RuntimeError, "camera"):
                adapter._wait_for_cameras_ready(
                    ["camera"],
                    launch_process,
                    timeout_sec=1.0,
                )

    def test_camera_readiness_requires_color_and_depth(self):
        adapter = CameraAdapter()
        adapter._frames_cache["camera"] = Mock()
        adapter._depth_cache["camera"] = Mock()
        launch_process = Mock()
        launch_process.poll.return_value = None

        adapter._wait_for_cameras_ready(
            ["camera"],
            launch_process,
            timeout_sec=1.0,
        )

    def test_initialize_failure_cleans_up_started_processes(self):
        adapter = CameraAdapter()
        launch_process = Mock(pid=101)
        launch_process.poll.return_value = None
        tf_processes = [Mock(pid=102), Mock(pid=103)]

        with patch(
            "adapters.hardware.leju_wheeled.camera_adapter.time.sleep"
        ), patch(
            "adapters.hardware.leju_wheeled.camera_adapter.subprocess.Popen",
            side_effect=[launch_process, *tf_processes],
        ), patch.object(
            adapter,
            "_setup_subscribers",
            side_effect=RuntimeError("subscriber setup failed"),
        ):
            result = adapter.initialize({"enable_head": True})

        self.assertFalse(result.success)
        self.assertIn("subscriber setup failed", result.message)
        launch_process.terminate.assert_called_once()
        launch_process.wait.assert_called_once()
        for process in tf_processes:
            process.terminate.assert_called_once()
            process.wait.assert_called_once()
        self.assertIsNone(adapter._launch_process)
        self.assertEqual(adapter._tf_processes, [])
        self.assertFalse(adapter._initialized)

    def test_popen_failure_cleans_up_temporary_stderr_file(self):
        adapter = CameraAdapter()
        with patch(
            "adapters.hardware.leju_wheeled.camera_adapter.subprocess.Popen",
            side_effect=FileNotFoundError("roslaunch not found"),
        ):
            result = adapter.initialize({"enable_head": True})

        self.assertFalse(result.success)
        self.assertIn("roslaunch not found", result.message)
        self.assertIsNone(adapter._launch_stderr_file)


if __name__ == "__main__":
    unittest.main()
