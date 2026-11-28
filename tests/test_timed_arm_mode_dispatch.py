"""用真实人工测试函数复现客户端发令序列；绝不模拟成物理复现或连接ROS。

只证明模式1下客户端仍提交MultiCmd，脚本shutdown后仍可单独切模式2。
服务端是否保留目标、计时及切换后跳变，不能由这里的Mock结论推出。
"""

import importlib.util
import sys
from pathlib import Path
from types import ModuleType
from unittest.mock import Mock

import pytest

from core.domain.result import Result

ROOT = Path(__file__).resolve().parents[1]
pytestmark = pytest.mark.unit


def load(relative, name):
    spec = importlib.util.spec_from_file_location(name, ROOT / relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def isolated_scripts(monkeypatch):
    def no_network(*args, **kwargs):
        pytest.fail("offline reproduction must not connect to any service")

    monkeypatch.setattr("socket.socket.connect", no_network)
    mode_script = load("apps/test_internal/adapters/set_arm_control_mode.py", "mode_script_repro")
    pose_script = load("apps/test_internal/adapters/timed_arm_pose.py", "pose_script_repro")
    arm = load("adapters/hardware/leju_wheeled/mixins/arm_mixin.py", "arm_mixin_repro")
    timed = load("adapters/hardware/leju_wheeled/mixins/timedcmd_mixin.py", "timed_mixin_repro")
    events = []
    controller_mode = [None]

    class Hardware(arm.ArmMixin, timed.TimedCmdMixin):
        def __init__(self):
            self._low_level_sdk_manager = Mock()
            self._low_level_sdk_manager.set_arm_control_mode.side_effect = self.set_mode
            self._timed_ros_manager = Mock()
            self._timed_ros_manager.send_multi_commands.side_effect = self.submit

        def set_mode(self, control_mode):
            controller_mode[0] = control_mode
            events.append(("mode", control_mode))
            return Result.ok()

        def submit(self, commands, is_sync):
            events.append(("submit", controller_mode[0], [c["planner_index"] for c in commands]))
            # 仅模拟服务接受，不执行或模拟任何目标轨迹。
            return Result.ok(data={"actual_time": 3.0})

        def set_focus_ee(self, value):
            events.append(("focus_ee", value))
            return Result.ok()

        def set_focus_z(self, value):
            events.append(("focus_z", value))
            return Result.ok()

        def initialize(self):
            events.append(("initialize",))
            return Result.ok()

        def shutdown(self):
            events.append(("shutdown",))
            return Result.ok()

    # 全部数据只用于Mock，取现有脚本人工配置；空配置时使用测试样本而非真机目标。
    if pose_script.LEFT_POSE is None and pose_script.RIGHT_POSE is None:
        monkeypatch.setattr(pose_script, "LEFT_POSE", [0.0] * 6)
    if pose_script.DESIRE_TIME is None:
        monkeypatch.setattr(pose_script, "DESIRE_TIME", 3.0)
    return mode_script, pose_script, Hardware, controller_mode, events


def test_real_script_test_functions_submit_in_mode_one_then_set_two(isolated_scripts, monkeypatch):
    mode_script, pose_script, Hardware, current_mode, events = isolated_scripts
    hw = Hardware()
    monkeypatch.setattr(mode_script, "CONTROL_MODE", 1)
    assert mode_script.test_set_arm_control_mode(hw)
    assert current_mode[0] == 1
    assert pose_script.test_timed_arm_pose(hw)
    assert current_mode[0] == 1
    submits = [e for e in events if e[0] == "submit"]
    assert len(submits) == 1 and submits[0][1] == 1
    assert set(submits[0][2]) <= {6, 7} if pose_script.FRAME in ("local", "base_link") else set(submits[0][2]) <= {4, 5}
    monkeypatch.setattr(mode_script, "CONTROL_MODE", 2)
    assert mode_script.test_set_arm_control_mode(hw)
    assert events[-1] == ("mode", 2)
    assert hw._timed_ros_manager.send_multi_commands.call_count == 1
    print("CLIENT ONLY: mode1 -> submit accepted while mode1 -> mode2; no client resend at switch")


def test_real_script_mains_use_separate_clients_and_shutdown(isolated_scripts, monkeypatch):
    mode_script, pose_script, Hardware, current_mode, events = isolated_scripts
    factory = ModuleType("adapters.hardware.factory")
    created = []

    def create_hardware(config):
        hw = Hardware()
        created.append(hw)
        return hw

    factory.HardwareFactory = Mock()
    factory.HardwareFactory.create_hardware.side_effect = create_hardware
    monkeypatch.setitem(sys.modules, "adapters.hardware.factory", factory)
    monkeypatch.setattr(mode_script, "init_logging", lambda: None)
    monkeypatch.setattr(pose_script, "init_logging", lambda: None)
    monkeypatch.setattr(mode_script, "CONTROL_MODE", 1)
    assert mode_script.main() == 0
    assert pose_script.main() == 0
    monkeypatch.setattr(mode_script, "CONTROL_MODE", 2)
    assert mode_script.main() == 0
    assert len(created) == 3
    assert [e[0] for e in events].count("shutdown") == 3
    assert [e for e in events if e[0] == "mode"] == [("mode", 1), ("mode", 2)]
    assert len([e for e in events if e[0] == "submit" and e[1] == 1]) == 1
    print("CLIENT ONLY: three script mains returned success, each shut down its own local client")
