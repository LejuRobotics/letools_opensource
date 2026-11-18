# -*- coding: utf-8 -*-
"""NodePalletServoV2：箱型 + 初/终阶段的激活层测试。"""
import json
import logging
import time

import pytest

from orchestration.nodes.node_pallet_servo_v2 import NodePalletServoV2
from orchestration.nodes.pallet_servo_plan import STAGE_FINAL, STAGE_INITIAL
from orchestration.nodes.tests.test_node_pallet_servo_v2 import (
    IMAGE_SIZE,
    K_PARAM,
    PALLET_H_MM,
    PALLET_W_MM,
    _fake_ros_modules,
    _fake_servo_msgs,
    make_node,
)

pytestmark = pytest.mark.unit


def _box_types():
    return [
        {
            "box_type": 0,
            "name": "小箱",
            "slots": [
                {
                    "slot": 1,
                    "mode": "pallet",
                    "ref_edges": ["y=50", "x=950"],
                    "initial": {"ref_edges": ["y=10", "x=900"]},
                },
                {
                    "slot": 2,
                    "mode": "pallet",
                    "ref_edges": [f"y={PALLET_H_MM}", "x=0"],
                },
            ],
        },
        {
            "box_type": 1,
            "name": "大箱",
            "slots": [
                {
                    "slot": 1,
                    "mode": "pallet",
                    "ref_edges": ["y=0", "x=0"],
                }
            ],
        },
    ]


def make_box_type_node(tmp_path, **over):
    params = {
        "box_types": _box_types(),
        "pallet_size_mm": [PALLET_W_MM, PALLET_H_MM],
        "K": K_PARAM,
        "image_size": IMAGE_SIZE,
        "dump_dir": str(tmp_path / "dump"),
        "diag_log_path": "off",
        "pallet_frame": "base_link",
    }
    params.update(over)
    node = NodePalletServoV2("servo", "servo", "ns", params)
    node.initialise()
    return node


def test_activate_with_box_type_and_initial_stage(tmp_path, monkeypatch):
    _fake_ros_modules(monkeypatch)
    _fake_servo_msgs(monkeypatch)
    node = make_box_type_node(tmp_path)

    ok, message = node._activate_slot(1, box_type=0, stage=STAGE_INITIAL)
    assert ok is True, message
    assert node._active_slot == 1
    assert node._active_plan.box_type == 0
    assert node._active_plan.box_type_name == "小箱"
    assert node._active_plan.stage == STAGE_INITIAL
    assert node._active_plan.ref_edges == ("y=10", "x=900")
    assert "小箱" in message and "initial" in message, message


def test_activate_with_box_type_one_uses_its_own_slot_table(tmp_path, monkeypatch):
    _fake_ros_modules(monkeypatch)
    _fake_servo_msgs(monkeypatch)
    node = make_box_type_node(tmp_path)

    ok, message = node._activate_slot(1, box_type=1, stage=STAGE_FINAL)
    assert ok is True, message
    assert node._active_plan.box_type == 1
    assert node._active_plan.ref_edges == ("y=0", "x=0")

    ok, message = node._activate_slot(2, box_type=1, stage=STAGE_FINAL)
    assert ok is False, message
    assert node._active_slot == 1, "越界调用不许改状态"


def test_activate_unknown_box_type_keeps_state(tmp_path, monkeypatch):
    _fake_ros_modules(monkeypatch)
    _fake_servo_msgs(monkeypatch)
    node = make_box_type_node(tmp_path)
    node._activate_slot(1, box_type=0, stage=STAGE_FINAL)
    before = node._active_plan

    ok, message = node._activate_slot(1, box_type=9, stage=STAGE_FINAL)
    assert ok is False, message
    assert "9" in message
    assert node._active_plan is before
    assert node._active_slot == 1


def test_activate_illegal_stage_keeps_state(tmp_path, monkeypatch):
    _fake_ros_modules(monkeypatch)
    _fake_servo_msgs(monkeypatch)
    node = make_box_type_node(tmp_path)
    node._activate_slot(1, box_type=0, stage=STAGE_FINAL)
    before = node._active_plan

    ok, message = node._activate_slot(1, box_type=0, stage=2)
    assert ok is False, message
    assert "stage" in message
    assert node._active_plan is before


def test_legacy_config_activates_as_box_type_zero_final(tmp_path, monkeypatch):
    _fake_ros_modules(monkeypatch)
    _fake_servo_msgs(monkeypatch)
    node = make_node(tmp_path)
    assert node._active_plan.box_type == 0
    assert node._active_plan.stage == STAGE_FINAL
    assert node._active_plan.ref_edges == ("y=0", "x=W")


def test_box_types_and_legacy_keys_are_mutually_exclusive(tmp_path):
    node = make_box_type_node(
        tmp_path,
        ref_edges=[["y=0", "x=W"]],
        servo_slots=[{"slot": 1, "mode": "pallet"}],
    )
    assert node._config_err is not None
    assert "box_types" in node._config_err
    assert "ref_edges" in node._config_err or "servo_slots" in node._config_err


def test_box_types_config_error_rejects_activation(tmp_path):
    node = make_box_type_node(
        tmp_path,
        box_types=[{"box_type": 0, "slots": [
            {"slot": 2, "mode": "pallet", "ref_edges": ["y=0", "x=W"]},
        ]}],
    )
    assert node._config_err is not None, "槽位不连续该在 initialise 就报出来"
    ok, message = node._activate_slot(1)
    assert ok is False
    assert "配置错误" in message or "box_types" in message


def _write_record(path):
    path.write_text(json.dumps({
        "version": 1, "D_mm": 900.0, "normal": [0.0, 0.0, 1.0],
        "base_frame": "base_link", "pallet_size_mm": [1000.0, 1200.0],
    }), encoding="utf-8")


def test_initial_stage_does_not_mark_placement(tmp_path, monkeypatch):
    _fake_ros_modules(monkeypatch)
    _fake_servo_msgs(monkeypatch)
    rec = tmp_path / "pallet_record.json"
    _write_record(rec)
    node = make_box_type_node(tmp_path, pallet_record_path=str(rec))

    ok, message = node._activate_slot(1, box_type=0, stage=STAGE_INITIAL)
    assert ok is True, message
    record = json.loads(rec.read_text(encoding="utf-8"))
    assert "placements_by_box_type" not in record, record

    ok, message = node._activate_slot(1, box_type=0, stage=STAGE_FINAL)
    assert ok is True, message
    record = json.loads(rec.read_text(encoding="utf-8"))
    bucket = record["placements_by_box_type"]["0"]
    assert bucket["placed_slots"] == [1], record
    assert bucket["slots"]["1"]["stage_name"] == "final", record


def test_box_type_placements_are_separate(tmp_path, monkeypatch):
    _fake_ros_modules(monkeypatch)
    _fake_servo_msgs(monkeypatch)
    rec = tmp_path / "pallet_record.json"
    _write_record(rec)
    node = make_box_type_node(tmp_path, pallet_record_path=str(rec))

    assert node._activate_slot(1, box_type=0, stage=STAGE_FINAL)[0] is True
    assert node._activate_slot(1, box_type=1, stage=STAGE_FINAL)[0] is True
    record = json.loads(rec.read_text(encoding="utf-8"))
    assert record["placements_by_box_type"]["0"]["placed_slots"] == [1]
    assert record["placements_by_box_type"]["1"]["placed_slots"] == [1]
    assert (record["placements_by_box_type"]["0"]["slots"]["1"]["box_type_name"]
            == "小箱")
    assert (record["placements_by_box_type"]["1"]["slots"]["1"]["box_type_name"]
            == "大箱")


class _FakeBag:
    def __init__(self):
        self.starts = []
        self.stops = []

    def start(self, **kwargs):
        self.starts.append(dict(kwargs))
        return True

    def stop(self, reason=""):
        self.stops.append(str(reason))

    def close(self):
        self.stops.append("close")


def test_bag_recording_follows_servo_lifecycle(tmp_path, monkeypatch):
    _fake_ros_modules(monkeypatch)
    _fake_servo_msgs(monkeypatch)
    node = make_box_type_node(tmp_path, pallet_record_timeout_s=0.01)
    bag = _FakeBag()
    node._bag = bag

    ok, message = node._activate_slot(1, box_type=0, stage=STAGE_INITIAL)
    assert ok is True, message
    assert bag.starts[-1] == {
        "box_type": 0, "slot": 1, "stage": 1, "stage_name": "initial"}
    assert bag.stops, "切到初伺服时旧记录应先停"

    ok, message = node._activate_slot(1, box_type=0, stage=STAGE_FINAL)
    assert ok is True, message
    assert bag.starts[-1]["stage_name"] == "final"

    # 失败的激活不改状态，也不该把正在录的 bag 停掉。
    before = len(bag.stops)
    ok, _ = node._activate_slot(1, box_type=9, stage=STAGE_FINAL)
    assert ok is False
    assert len(bag.stops) == before

    ok, message = node._activate_slot(-1)
    assert ok is True, message
    assert bag.stops[-1] == "slot_-1"


def test_bag_recording_stops_on_call_zero(tmp_path, monkeypatch):
    _fake_ros_modules(monkeypatch)
    _fake_servo_msgs(monkeypatch)
    node = make_box_type_node(
        tmp_path, pallet_record_timeout_s=0.01,
        pallet_record_samples=5, pallet_record_min_samples=5)
    bag = _FakeBag()
    node._bag = bag
    node._activate_slot(1, box_type=0, stage=STAGE_FINAL)
    node._activate_slot(0)
    assert bag.stops[-1] == "slot_0"


def test_reject_warning_is_deduped_until_reason_changes(tmp_path, caplog):
    node = make_box_type_node(tmp_path)
    with caplog.at_level(logging.WARNING):
        node._warn_reject("托盘伺服", "same reason")
        node._warn_reject("托盘伺服", "same reason")
    assert caplog.text.count("托盘伺服第") == 1, caplog.text
    with caplog.at_level(logging.WARNING):
        node._warn_reject("托盘伺服", "new reason")
    assert caplog.text.count("托盘伺服第") == 2, caplog.text


def test_diag_heartbeat_info_is_gated_on_state_change(tmp_path, caplog):
    node = make_box_type_node(tmp_path)
    node._diag_period_s = 0.001
    with caplog.at_level(logging.INFO):
        node._diag_heartbeat(1, "pallet")
        time.sleep(0.002)
        node._diag_heartbeat(1, "pallet")
    infos = [r for r in caplog.records
             if r.levelno == logging.INFO and "诊断：" in r.getMessage()]
    assert len(infos) == 1, [r.getMessage() for r in infos]

    node._active_stage = STAGE_INITIAL
    with caplog.at_level(logging.INFO):
        time.sleep(0.002)
        node._diag_heartbeat(1, "pallet")
    infos = [r for r in caplog.records
             if r.levelno == logging.INFO and "诊断：" in r.getMessage()]
    assert len(infos) == 2, [r.getMessage() for r in infos]
