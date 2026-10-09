"""Service response deadlines and cancellation without ROS or robot hardware."""
import importlib
import sys
import threading
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from orchestration.nodes.grasp_mtbf_v1.ros_service_call import RosServiceGate

pytestmark = pytest.mark.unit


@pytest.fixture
def ros(monkeypatch):
    proxy = MagicMock(return_value=SimpleNamespace(success=True))
    fake = SimpleNamespace(wait_for_service=MagicMock(), ServiceProxy=MagicMock(return_value=proxy))
    monkeypatch.setitem(sys.modules, "rospy", fake)
    return fake, proxy


def _call_in_thread(gate, **kwargs):
    result = {"done": threading.Event()}
    def call():
        try:
            result["value"] = gate.call("/control", object, object(), **kwargs)
        except BaseException as exc:
            result["error"] = exc
        finally:
            result["done"].set()
    thread = threading.Thread(target=call, daemon=True)
    thread.start()
    return result


def test_close_cannot_block_callers_completion(ros):
    fake, proxy = ros
    closing, release = threading.Event(), threading.Event()
    def close():
        closing.set()
        release.wait(2.0)
    proxy.close.side_effect = close
    try:
        result = _call_in_thread(RosServiceGate(), timeout=0.2)
        assert closing.wait(1.0)
        assert result["done"].wait(0.1)
        assert result["value"].success
    finally:
        release.set()


def test_inflight_call_rejects_concurrent_call_without_sending(ros):
    fake, proxy = ros
    entered, release = threading.Event(), threading.Event()
    def blocked(_request):
        entered.set()
        assert release.wait(2.0)
        return SimpleNamespace(success=True)
    proxy.side_effect = blocked
    gate = RosServiceGate()
    try:
        result = _call_in_thread(gate, timeout=1.0)
        assert entered.wait(1.0)
        with pytest.raises(RuntimeError, match="in flight"):
            gate.call("/other", object, object())
        assert proxy.call_count == 1
    finally:
        release.set()
    assert result["done"].wait(1.0)
    assert result["value"].success


def test_timeout_latches_fault_even_after_late_success(ros):
    fake, proxy = ros
    entered, release, closed = threading.Event(), threading.Event(), threading.Event()
    def blocked(_request):
        entered.set()
        assert release.wait(2.0)
        return SimpleNamespace(success=True)
    proxy.side_effect = blocked
    proxy.close.side_effect = closed.set
    gate = RosServiceGate()
    try:
        result = _call_in_thread(gate, timeout=0.1)
        assert entered.wait(1.0)
        assert result["done"].wait(0.5)
        assert isinstance(result["error"], TimeoutError)
        with pytest.raises(RuntimeError, match="outcome is unknown"):
            gate.call("/other", object, object())
    finally:
        release.set()
    assert closed.wait(1.0)
    with pytest.raises(RuntimeError, match="outcome is unknown"):
        gate.call("/other", object, object())
    assert proxy.call_count == 1


def test_discovery_timeout_cannot_send_when_service_later_appears(ros):
    fake, proxy = ros
    entered, release, closed = threading.Event(), threading.Event(), threading.Event()
    def discovery(*args, **kwargs):
        entered.set()
        assert release.wait(2.0)
    fake.wait_for_service.side_effect = discovery
    proxy.close.side_effect = closed.set
    gate = RosServiceGate()
    try:
        result = _call_in_thread(gate, timeout=0.1)
        assert entered.wait(1.0)
        assert result["done"].wait(0.5)
        assert isinstance(result["error"], TimeoutError)
    finally:
        release.set()
    assert closed.wait(1.0)
    proxy.assert_not_called()
    # No request was sent, so there is no unknown controller outcome.
    fake.wait_for_service.side_effect = None
    assert gate.call("/control", object, object(), timeout=0.5).success
    proxy.assert_called_once()


def test_transport_exception_blocks_further_control_calls(ros):
    fake, proxy = ros
    proxy.side_effect = ConnectionError("link lost")
    gate = RosServiceGate()
    with pytest.raises(ConnectionError, match="link lost"):
        gate.call("/control", object, object(), timeout=0.5)
    with pytest.raises(RuntimeError, match="outcome is unknown"):
        gate.call("/other", object, object())
    proxy.assert_called_once()


def test_worker_latches_late_response_before_waiter_handles_timeout(ros, monkeypatch):
    fake, proxy = ros
    module = importlib.import_module("orchestration.nodes.grasp_mtbf_v1.ros_service_call")
    clock = SimpleNamespace(now=10.0)
    monkeypatch.setattr(module, "time", SimpleNamespace(monotonic=lambda: clock.now))
    def late(_request):
        clock.now = 20.0
        return SimpleNamespace(success=True)
    proxy.side_effect = late
    gate = RosServiceGate()
    with pytest.raises(TimeoutError):
        gate.call("/control", object, object(), timeout=1.0)
    with pytest.raises(RuntimeError, match="outcome is unknown"):
        gate.call("/other", object, object())
    proxy.assert_called_once()


def test_cancel_after_send_latches_fault_and_never_retries(ros):
    fake, proxy = ros
    entered, release, closed = threading.Event(), threading.Event(), threading.Event()
    cancel = threading.Event()
    def blocked(_request):
        entered.set()
        assert release.wait(2.0)
        return SimpleNamespace(success=True)
    proxy.side_effect = blocked
    proxy.close.side_effect = closed.set
    gate = RosServiceGate()
    try:
        result = _call_in_thread(gate, timeout=1.0, cancel_event=cancel)
        assert entered.wait(1.0)
        cancel.set()
        assert result["done"].wait(0.5)
        assert "cancelled" in str(result["error"])
    finally:
        release.set()
    assert closed.wait(1.0)
    with pytest.raises(RuntimeError, match="outcome is unknown"):
        gate.call("/control", object, object())
    proxy.assert_called_once()
