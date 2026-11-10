"""Offline tests of the hardware start/stop script; never launch cameras or ROS nodes."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from apps.test_camera_adapter import test_perception_start_stop as script
from core.domain.result import Result

pytestmark = pytest.mark.unit


@pytest.fixture
def args(tmp_path):
    path = tmp_path / "camera.yaml"
    path.write_text("camera:\n  enable_head: true\n  driver_only: true\n", encoding="utf-8")
    return SimpleNamespace(config=path, head_only=True, cycles=2, duration=2.0,
                           idle_duration=1.0, frame_timeout=2.0, exit_timeout=2.0,
                           tag_id=[], require_tag=False)


def test_config_override_never_changes_file(args):
    args.config.write_text("camera:\n  enable_head: true\n  enable_wrist_camera: true\n"
                           "  has_left_wrist: true\n  left_wrist_camera_sn: ''\n")
    original = args.config.read_text()
    config = script.load_camera_config(args.config, head_only=True)
    assert config["enable_wrist_camera"] is False
    assert config["launch_apriltag"] is True
    assert args.config.read_text() == original
    with pytest.raises(script.TestFailure, match="left_wrist_camera_sn"):
        script.load_camera_config(args.config)


@pytest.mark.parametrize("text", ["", "camera: []", "other: {}", "camera:\n  enable_head: false\n"])
def test_invalid_camera_config_fails_before_start(args, text):
    args.config.write_text(text)
    with pytest.raises(script.TestFailure):
        script.load_camera_config(args.config)


@pytest.mark.parametrize("value", ["0", "-1", "nan", "inf"])
def test_invalid_durations_rejected(value):
    with pytest.raises(script.argparse.ArgumentTypeError):
        script.positive_float(value)


def test_probe_tracks_frames_and_ids_independently():
    probe = script.MessageProbe()
    message = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda: 12.0)))
    probe.callback(message)
    message.detections = [SimpleNamespace(id=[1, 2])]
    probe.callback(message)
    assert probe.snapshot() == (2, 12.0, {1, 2})
    assert probe.latest_snapshot()[0] == {1, 2}
    probe.clear_ids()
    assert probe.snapshot() == (2, 12.0, set())
    assert probe.latest_snapshot() == (set(), None)


def test_empty_detection_clears_latest_but_preserves_seen_ids():
    probe = script.MessageProbe()
    message = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda: 12.0)),
        detections=[SimpleNamespace(id=[1])],
    )
    probe.callback(message)
    assert probe.latest_snapshot()[0] == {1}
    message.detections = []  # 遮挡后收到空检测帧。
    probe.callback(message)
    assert probe.latest_snapshot()[0] == set()
    assert probe.snapshot()[2] == {1}


def test_latest_ids_replace_previous_frame_and_message_age_is_reported(monkeypatch):
    now = [10.0]
    monkeypatch.setattr(script.time, "monotonic", lambda: now[0])
    probe = script.MessageProbe()
    message = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda: 12.0)),
        detections=[SimpleNamespace(id=[1])],
    )
    probe.callback(message)
    now[0] = 11.0
    message.detections = [SimpleNamespace(id=[2])]
    probe.callback(message)
    now[0] = 11.5
    assert probe.latest_snapshot() == ({2}, 0.5)
    assert probe.snapshot()[2] == {1, 2}


def test_preflight_conflict_does_not_launch_camera(args, monkeypatch):
    monkeypatch.setattr(script.rosnode, "get_node_names", lambda: ["/camera/camera", "/ar_control_node"])
    camera = Mock()
    monkeypatch.setattr(script, "CameraAdapter", camera)
    tester = script.PerceptionStartStopTest(args)
    with pytest.raises(script.TestFailure, match="冲突"):
        tester.run_checks()
    camera.assert_not_called()


def test_successful_sequence_exercises_repeat_start_stop_and_shutdown(args, monkeypatch):
    camera = Mock()
    camera.initialize.return_value = Result.ok()
    perception = Mock()
    perception.initialize.return_value = True
    perception._ros_process = SimpleNamespace(pid=101)
    perception._ar_control_process = SimpleNamespace(pid=102)
    perception.start.side_effect = [Result.ok()] * 5 + [Result.fail("not initialized")]
    perception.stop.return_value = Result.ok()
    perception.check_health.side_effect = [{"initialized": True}, {"initialized": True},
                                          {"initialized": False}]
    monkeypatch.setattr(script.rosnode, "get_node_names", lambda: [])
    monkeypatch.setattr(script.rospy, "init_node", Mock())
    monkeypatch.setattr(script.rospy, "Subscriber", Mock())
    monkeypatch.setattr(script, "CameraAdapter", lambda: camera)
    monkeypatch.setattr(script, "PerceptionAdapter", lambda: perception)
    tester = script.PerceptionStartStopTest(args)
    monkeypatch.setattr(tester, "assert_fresh_images", Mock())
    monkeypatch.setattr(tester, "assert_idle", Mock())
    monkeypatch.setattr(tester, "observe_running", Mock())
    monkeypatch.setattr(tester, "remember_processes", Mock())
    monkeypatch.setattr(tester, "wait_until", lambda predicate, timeout: True)
    tester.run_checks()
    assert perception.start.call_count == 6
    assert perception.stop.call_count == 4
    perception.shutdown.assert_called_once()
    assert tester.observe_running.call_count == 2
    assert tester.assert_idle.call_count == 3
    camera.shutdown.assert_not_called()


@pytest.mark.parametrize("exception,expected", [(RuntimeError("failed"), 1), (KeyboardInterrupt(), 130)])
def test_failure_or_interrupt_always_cleans_up(args, monkeypatch, exception, expected):
    tester = script.PerceptionStartStopTest(args)
    monkeypatch.setattr(tester, "run_checks", Mock(side_effect=exception))
    cleanup = Mock(return_value=True)
    monkeypatch.setattr(tester, "cleanup", cleanup)
    assert tester.run() == expected
    cleanup.assert_called_once()


def test_cleanup_failure_cannot_report_success(args, monkeypatch):
    tester = script.PerceptionStartStopTest(args)
    monkeypatch.setattr(tester, "run_checks", Mock())
    monkeypatch.setattr(tester, "cleanup", Mock(return_value=False))
    assert tester.run() == 1


def test_cleanup_continues_after_perception_failure(args):
    tester = script.PerceptionStartStopTest(args)
    tester.perception = Mock()
    tester.perception.stop.side_effect = RuntimeError("stop failed")
    tester.camera = Mock()
    subscriber = Mock()
    tester.subscribers.append(subscriber)
    assert not tester.cleanup()
    tester.camera.shutdown.assert_called_once()
    subscriber.unregister.assert_called_once()
