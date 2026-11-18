# -*- coding: utf-8 -*-
"""托盘伺服 bag 记录：启动/停止、500MB 分卷、按天清理。"""
import logging
import subprocess
from datetime import date
from pathlib import Path
from types import SimpleNamespace

import pytest

from orchestration.nodes.pallet_servo_bag import (
    DEFAULT_TOPICS,
    BagRecorder,
)

pytestmark = pytest.mark.unit


class FakeProc:
    def __init__(self, pid=123):
        self.pid = pid
        self.returncode = None
        self.terminated = False
        self.killed = False
        self.waited = 0

    def poll(self):
        return self.returncode

    def terminate(self):
        self.terminated = True
        self.returncode = 0

    def wait(self, timeout=None):
        self.waited += 1
        if self.returncode is None:
            self.returncode = 0
        return self.returncode

    def kill(self):
        self.killed = True
        self.returncode = -9


class TimeoutProc(FakeProc):
    def wait(self, timeout=None):
        self.waited += 1
        if self.waited == 1:
            raise subprocess.TimeoutExpired(cmd="rosbag", timeout=timeout)
        return self.returncode


def _recorder(tmp_path, *, popen=None, which=None, today=None, **over):
    captured = {}

    def _popen(cmd, **kwargs):
        captured["cmd"] = cmd
        captured["kwargs"] = kwargs
        return (popen or FakeProc)()

    if which is None:
        which = lambda name: "/usr/bin/rosbag"
    if today is None:
        today = lambda: date(2026, 11, 13)
    kwargs = dict(
        enabled=True,
        root_dir=tmp_path / "bags",
        topics=["/a", "/b"],
        split_mb=500,
        keep_days=1,
        stop_timeout_s=1.0,
        logger=logging.getLogger("test-bag"),
        popen=_popen,
        which=which,
        today=today,
    )
    kwargs.update(over)
    return BagRecorder(**kwargs), captured


def test_disabled_recorder_does_nothing(tmp_path):
    rec, captured = _recorder(tmp_path, enabled=False)
    assert rec.start(box_type=0, slot=1, stage=0, stage_name="final") is False
    assert captured == {}
    rec.stop("test")


def test_start_builds_rosbag_command(tmp_path):
    rec, captured = _recorder(tmp_path)
    assert rec.start(box_type=1, slot=2, stage=1, stage_name="initial") is True
    cmd = captured["cmd"]
    assert cmd[0] == "/usr/bin/rosbag"
    assert cmd[1] == "record"
    assert cmd[2] == "-o"
    prefix = Path(cmd[3])
    assert prefix.parent == tmp_path / "bags" / "20261113"
    assert "box1_slot2_stage1_initial" in prefix.name
    assert "--split" in cmd
    assert "--size=500" in cmd
    assert cmd[-2:] == ["/a", "/b"]
    assert captured["kwargs"]["stdout"] is subprocess.DEVNULL
    assert captured["kwargs"]["stderr"] is subprocess.DEVNULL
    assert captured["kwargs"]["start_new_session"] is True


def test_start_stops_the_previous_recording(tmp_path):
    rec, _ = _recorder(tmp_path)
    rec.start(box_type=0, slot=1, stage=1, stage_name="initial")
    first = rec._proc
    rec.start(box_type=0, slot=1, stage=0, stage_name="final")
    assert first.terminated is True
    assert rec._proc is not first


def test_stop_terminates_process(tmp_path):
    rec, _ = _recorder(tmp_path)
    rec.start(box_type=0, slot=1, stage=0, stage_name="final")
    proc = rec._proc
    rec.stop("slot_-1")
    assert proc.terminated is True
    assert rec._proc is None


def test_stop_kills_after_timeout(tmp_path):
    rec, _ = _recorder(tmp_path, popen=TimeoutProc)
    rec.start(box_type=0, slot=1, stage=0, stage_name="final")
    proc = rec._proc
    rec.stop("slot_-1")
    assert proc.killed is True


def test_missing_rosbag_is_a_warning_not_a_crash(tmp_path):
    rec, captured = _recorder(tmp_path, which=lambda name: None)
    assert rec.start(box_type=0, slot=1, stage=0, stage_name="final") is False
    assert captured == {}


def test_cleanup_removes_previous_days_only(tmp_path):
    root = tmp_path / "bags"
    (root / "20261112").mkdir(parents=True)
    (root / "20261112" / "old.bag").write_bytes(b"x")
    (root / "20261113").mkdir()
    (root / "20261113" / "today.bag").write_bytes(b"x")
    (root / "keep.txt").write_text("not a bag dir")
    rec, _ = _recorder(tmp_path)
    assert not (root / "20261112").exists()
    assert (root / "20261113" / "today.bag").is_file()
    assert (root / "keep.txt").is_file()


def test_keep_days_two_keeps_yesterday(tmp_path):
    root = tmp_path / "bags"
    (root / "20261112").mkdir(parents=True)
    (root / "20261111").mkdir()
    rec, _ = _recorder(tmp_path, keep_days=2)
    assert (root / "20261112").is_dir()
    assert not (root / "20261111").exists()


def test_close_is_idempotent(tmp_path):
    rec, _ = _recorder(tmp_path)
    rec.start(box_type=0, slot=1, stage=0, stage_name="final")
    rec.close()
    rec.close()
    assert rec._proc is None


def test_default_topics_cover_the_v2_chain():
    assert "/camera/color/image_raw/compressed" in DEFAULT_TOPICS
    assert "/pallet_v2/detection" in DEFAULT_TOPICS
    assert "/box_v2/detection" in DEFAULT_TOPICS
    assert "/pallet_servo_v2/dis" in DEFAULT_TOPICS
    assert "/tf" in DEFAULT_TOPICS and "/tf_static" in DEFAULT_TOPICS
