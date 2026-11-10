"""Common AprilTag lifecycle tests without a ROS master or camera."""

import os
import signal
import subprocess
import threading
from types import SimpleNamespace
from unittest.mock import Mock, call

import pytest

from adapters.hardware.leju_wheeled import perception_adapter as module
from core.domain.result import Result

pytestmark = pytest.mark.unit


@pytest.fixture
def adapter(monkeypatch):
    monkeypatch.setattr(module.rospy.core, "is_initialized", lambda: True)
    # start() 会检查 config/ 下的启动文件；测试不依赖现场的 apriltag_config.yaml，只校验存在性。
    real_isfile = module.os.path.isfile
    monkeypatch.setattr(
        module.os.path, "isfile",
        lambda path: str(path).endswith(("apriltag_continuous.launch", "apriltag_config.yaml")) or real_isfile(path),
    )
    instance = module.PerceptionAdapter()
    assert instance.initialize(Mock(), {"launch_apriltag": True})
    return instance


@pytest.fixture
def runtime(adapter, monkeypatch):
    processes = []
    callbacks = []
    subscribers = []

    def subscribe(topic, msg_type, callback, **kwargs):
        callbacks.append(callback)
        subscriber = Mock()
        subscribers.append(subscriber)
        return subscriber

    def popen(*args, **kwargs):
        process = Mock(pid=9000 + len(processes))
        process.poll.return_value = None
        processes.append(process)
        if len(processes) % 2 == 0:
            callbacks[-1](SimpleNamespace(detections=[]))
        return process

    monkeypatch.setattr(module.rospy, "Subscriber", subscribe)
    monkeypatch.setattr(module.subprocess, "Popen", Mock(side_effect=popen))
    monkeypatch.setattr(adapter, "_stop_process_group", Mock())
    return SimpleNamespace(processes=processes, callbacks=callbacks, subscribers=subscribers)


def _message():
    pose = SimpleNamespace(
        position=SimpleNamespace(x=1.0, y=2.0, z=3.0),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    detection = SimpleNamespace(id=[1], pose=SimpleNamespace(pose=SimpleNamespace(pose=pose)))
    return SimpleNamespace(
        detections=[detection],
        header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda: 100.0), frame_id="base_link"),
    )


def test_initialize_and_getters_never_spawn(adapter, monkeypatch):
    spawn = Mock()
    monkeypatch.setattr(module.subprocess, "Popen", spawn)
    assert adapter.get_tag_detections() == []
    assert not adapter.get_latest_result().success
    assert adapter.check_health()["healthy"]
    spawn.assert_not_called()


def test_start_requires_initialization():
    assert not module.PerceptionAdapter().start().success


def test_start_requires_camera(adapter):
    adapter._camera = None
    assert not adapter.start().success


def test_disabled_start_does_not_spawn(adapter, monkeypatch):
    adapter._config["launch_apriltag"] = False
    spawn = Mock()
    monkeypatch.setattr(module.subprocess, "Popen", spawn)
    assert not adapter.start().success
    spawn.assert_not_called()


def test_start_is_idempotent_and_uses_owned_sessions(adapter, runtime):
    assert adapter.start().success
    assert adapter.start().success
    assert len(runtime.processes) == 2
    for call in module.subprocess.Popen.call_args_list:
        assert call.kwargs["start_new_session"] is True
    assert adapter.check_health()["apriltag_started"]


def test_stop_clears_data_and_keeps_camera_initialized(adapter, runtime):
    camera = adapter._camera
    assert adapter.start().success
    runtime.callbacks[-1](_message())
    assert adapter.get_tag_detections()[0].tag_id == 1
    assert adapter.stop().success
    assert adapter.stop().success
    assert adapter.get_tag_detections() == []
    assert adapter._is_initialized
    assert adapter.check_health()["healthy"]
    assert adapter._camera is camera
    camera.shutdown.assert_not_called()
    assert adapter.get_performance_metrics()["total_detections"] == 0
    assert adapter._ros_process is None
    assert adapter._ar_control_process is None
    runtime.subscribers[0].unregister.assert_called_once()
    assert adapter._stop_process_group.call_count == 2


def test_empty_detection_clears_adapter_cache_while_running(adapter, runtime):
    assert adapter.start().success
    runtime.callbacks[-1](_message())
    assert adapter.get_tag_detections()[0].tag_id == 1
    runtime.callbacks[-1](SimpleNamespace(detections=[]))
    assert adapter.get_tag_detections() == []
    assert not adapter.get_latest_result().success
    assert adapter.check_health()["apriltag_started"]


def test_restart_rejects_old_session_callbacks(adapter, runtime):
    assert adapter.start().success
    old_callback = runtime.callbacks[-1]
    assert adapter.stop().success
    old_callback(_message())
    assert adapter.get_tag_detections() == []
    assert adapter.start().success
    old_callback(_message())
    assert adapter.get_tag_detections() == []
    runtime.callbacks[-1](_message())
    assert len(adapter.get_tag_detections()) == 1
    assert len(runtime.processes) == 4


def test_inflight_callback_cannot_repopulate_stopped_cache(adapter, runtime, monkeypatch):
    assert adapter.start().success
    entered = threading.Event()
    resume = threading.Event()

    def convert(*args):
        entered.set()
        assert resume.wait(2)
        return 0.0, 0.0, 0.0

    monkeypatch.setattr(module, "quaternion_to_euler", convert)
    worker = threading.Thread(target=runtime.callbacks[-1], args=(_message(),))
    worker.start()
    assert entered.wait(2)
    try:
        assert adapter.stop().success
    finally:
        resume.set()
        worker.join(2)
    assert not worker.is_alive()
    assert adapter._latest_detections == []


def test_partial_start_failure_cleans_owned_resources(adapter, runtime, monkeypatch):
    process = Mock(pid=9000)
    process.poll.return_value = None
    monkeypatch.setattr(module.subprocess, "Popen", Mock(side_effect=[process, OSError("missing ar_control")]))
    result = adapter.start()
    assert not result.success
    assert "missing ar_control" in result.message
    adapter._stop_process_group.assert_called_once_with(process)
    runtime.subscribers[0].unregister.assert_called_once()
    assert adapter._ros_process is None
    assert not adapter.check_health()["healthy"]


def test_start_timeout_is_failure_and_cleans_both_processes(adapter, runtime, monkeypatch):
    monkeypatch.setattr(adapter._startup_event, "wait", lambda **kwargs: False)
    monkeypatch.setattr(module.time, "monotonic", Mock(side_effect=[0.0, 0.0, 6.0]))
    assert not adapter.start().success
    assert adapter._stop_process_group.call_count == 2
    assert not adapter._apriltag_started


def test_dead_process_is_not_reported_as_running(adapter, runtime):
    assert adapter.start().success
    runtime.callbacks[-1](_message())
    runtime.processes[0].poll.return_value = 1
    assert adapter.get_tag_detections() == []
    assert not adapter.check_health()["healthy"]


def test_cleanup_failure_keeps_handle_for_retry(adapter, runtime):
    assert adapter.start().success
    adapter._stop_process_group.side_effect = [OSError("denied"), None]
    result = adapter.stop()
    assert not result.success
    assert adapter._ar_control_process is runtime.processes[1]
    assert adapter._ros_process is None
    adapter._stop_process_group.side_effect = None
    assert adapter.stop().success
    assert adapter._ar_control_process is None


def test_shutdown_stops_detection_and_disallows_restart(adapter, runtime):
    assert adapter.start().success
    adapter.shutdown()
    assert not adapter._is_initialized
    assert not adapter.start().success
    adapter._camera.shutdown.assert_not_called()


def test_custom_topic_is_remapped_at_publisher(adapter, runtime):
    adapter._config["apriltag_topic"] = "/custom_tag_info"
    assert adapter.start().success
    cmd = module.subprocess.Popen.call_args_list[1].args[0]
    assert "/robot_tag_info:=/custom_tag_info" in cmd


def test_head_namespace_is_used_by_common_apriltag_launcher(adapter, runtime):
    adapter._config['head_camera_namespace'] = '/robot/cam_h'
    assert adapter.start().success
    cmd = module.subprocess.Popen.call_args_list[0].args[0]
    assert 'camera_name:=/robot/cam_h/color' in cmd
    assert 'image_topic:=image_raw' in cmd
    # 只改图像话题；ar_control 仍使用原头部 optical frame。
    control_cmd = module.subprocess.Popen.call_args_list[1].args[0]
    assert control_cmd[control_cmd.index('--source_frame') + 1] == 'camera_color_optical_frame'


@pytest.mark.parametrize('config', [{}, {'head_camera_namespace': ''}, {'head_camera_namespace': None}])
def test_common_apriltag_launcher_empty_namespace_uses_default(adapter, runtime, config):
    adapter._config.update(config)
    assert adapter.start().success
    cmd = module.subprocess.Popen.call_args_list[0].args[0]
    assert 'camera_name:=/camera/color' in cmd


@pytest.mark.parametrize('value', ['/bad/', 'camera', False])
def test_common_apriltag_launcher_rejects_invalid_namespace(adapter, runtime, value):
    adapter._config['head_camera_namespace'] = value
    result = adapter.start()
    assert not result.success
    assert 'head_camera_namespace' in result.message
    module.subprocess.Popen.assert_not_called()


def test_process_cleanup_kills_independent_roslaunch_children(monkeypatch):
    process = Mock(pid=12345)
    child_pid = 23456
    states = {child_pid: "S"}

    def fake_state(pid):
        return states.get(pid)

    def fake_kill(pid, sig):
        # 子进程收到 SIGTERM 后不退出，只有 SIGKILL 才会结束。
        if sig == signal.SIGKILL:
            states[pid] = "Z"

    monkeypatch.setattr(module.PerceptionAdapter, "_descendant_pids", staticmethod(lambda pid: [child_pid]))
    monkeypatch.setattr(module.PerceptionAdapter, "_pid_state", staticmethod(fake_state))
    killpg = Mock()
    kill = Mock(side_effect=fake_kill)
    monkeypatch.setattr(module.os, "killpg", killpg)
    monkeypatch.setattr(module.os, "kill", kill)
    module.PerceptionAdapter._stop_process_group(process)
    killpg.assert_called_once_with(12345, signal.SIGTERM)
    assert call(child_pid, signal.SIGTERM) in kill.call_args_list
    assert call(child_pid, signal.SIGKILL) in kill.call_args_list
    assert states[child_pid] == "Z"


def test_process_cleanup_escalates_parent_timeout(monkeypatch):
    process = Mock(pid=12345)
    process.wait.side_effect = [subprocess.TimeoutExpired("roslaunch", 5), 0]
    monkeypatch.setattr(module.PerceptionAdapter, "_descendant_pids", staticmethod(lambda pid: []))
    monkeypatch.setattr(module.PerceptionAdapter, "_pid_state", staticmethod(lambda pid: None))
    killpg = Mock()
    monkeypatch.setattr(module.os, "killpg", killpg)
    module.PerceptionAdapter._stop_process_group(process)
    assert killpg.call_args_list[1].args == (12345, signal.SIGKILL)


def test_descendant_discovery_parses_proc_with_tricky_comm(tmp_path, monkeypatch):
    """comm 字段可含空格和括号；ppid 必须从最后一个 ')' 之后解析。"""
    proc = tmp_path / "proc"
    layout = {
        100: "100 (roslaunch) S 1 100 100 0",
        200: "200 (weird name (x)) S 100 200 200 0",   # 子进程，comm 含括号和空格
        300: "300 (grandchild) S 200 300 300 0",       # 孙进程
        400: "400 (unrelated) S 1 400 400 0",          # 无关进程
    }
    for pid, stat in layout.items():
        (proc / str(pid)).mkdir(parents=True)
        (proc / str(pid) / "stat").write_text(stat)
    (proc / "self").mkdir()  # 非数字目录应被忽略
    real_open = open

    def fake_open(path, *args, **kwargs):
        if str(path).startswith("/proc/"):
            path = str(proc / str(path)[len("/proc/"):])
        return real_open(path, *args, **kwargs)

    real_listdir = os.listdir

    def fake_listdir(path):
        return real_listdir(proc) if path == "/proc" else real_listdir(path)

    monkeypatch.setattr(module.os, "listdir", fake_listdir)
    monkeypatch.setattr("builtins.open", fake_open)
    found = module.PerceptionAdapter._descendant_pids(100)
    assert sorted(found) == [200, 300]


def test_pid_state_returns_none_for_missing_process(tmp_path, monkeypatch):
    real_open = open

    def fake_open(path, *args, **kwargs):
        if str(path).startswith("/proc/"):
            raise FileNotFoundError(path)
        return real_open(path, *args, **kwargs)

    monkeypatch.setattr("builtins.open", fake_open)
    assert module.PerceptionAdapter._pid_state(999999) is None
