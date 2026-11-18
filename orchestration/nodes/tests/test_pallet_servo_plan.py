# -*- coding: utf-8 -*-
"""托盘伺服 SlotPlan：箱型 / 槽位 / 初终阶段 的配置解析。

这一层只做**纯配置**：不碰 ROS、不碰位姿。节点在 call 服务时调用一次
`resolve_slot_plan` 得到这一档真正生效的全部参数，运行路径不再自己拼
`ref_edges` / 偏置 / 箱高。
"""
import pytest

from orchestration.nodes.pallet_servo_plan import (
    STAGE_FINAL,
    STAGE_INITIAL,
    legacy_box_type,
    parse_box_types,
    resolve_slot_plan,
)

pytestmark = pytest.mark.unit


def _box_types():
    return [
        {
            "box_type": 0,
            "name": "小箱",
            "box_plane_height_mm": 300,
            "slots": [
                {
                    "slot": 1,
                    "mode": "pallet",
                    "ref_edges": ["y=50", "x=950"],
                    "initial": {"ref_edges": ["y=10", "x=900"]},
                },
                {
                    "slot": 2,
                    "mode": "neighbor",
                    "side": "right",
                    "ref_edges": ["y=0", "x=W"],
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


def test_parse_box_types_keeps_per_type_slots():
    profiles, err = parse_box_types(_box_types())
    assert err is None, err
    assert [p.box_type for p in profiles] == [0, 1]
    assert [p.name for p in profiles] == ["小箱", "大箱"]
    assert [len(p.slots) for p in profiles] == [2, 1]


def test_initial_stage_uses_its_own_ref_edges():
    profiles, err = parse_box_types(_box_types())
    assert err is None, err

    plan, err = resolve_slot_plan(
        profiles, box_type=0, slot=1, stage=STAGE_INITIAL,
        global_height_mm=250.0)
    assert err is None, err
    assert plan.ref_edges == ("y=10", "x=900")
    assert plan.stage == STAGE_INITIAL
    assert plan.stage_name == "initial"


def test_final_stage_uses_slot_ref_edges():
    profiles, _ = parse_box_types(_box_types())
    plan, err = resolve_slot_plan(
        profiles, box_type=0, slot=1, stage=STAGE_FINAL,
        global_height_mm=250.0)
    assert err is None, err
    assert plan.ref_edges == ("y=50", "x=950")
    assert plan.stage_name == "final"


def test_stage_override_can_set_height_and_bias():
    raw = _box_types()
    raw[0]["slots"][0]["box_plane_height_mm"] = 250
    raw[0]["slots"][0]["e_right_bias_px"] = -3
    raw[0]["slots"][0]["initial"] = {
        "ref_edges": ["y=10", "x=900"],
        "box_plane_height_mm": 120,
        "e_right_bias_px": -8,
        "e_bottom_bias_px": 4,
    }
    profiles, err = parse_box_types(raw)
    assert err is None, err

    plan, err = resolve_slot_plan(
        profiles, box_type=0, slot=1, stage=STAGE_INITIAL,
        global_height_mm=250.0)
    assert err is None, err
    assert plan.box_plane_height_mm == 120.0
    assert plan.e_right_bias_px == -8.0
    assert plan.e_bottom_bias_px == 4.0

    plan, err = resolve_slot_plan(
        profiles, box_type=0, slot=1, stage=STAGE_FINAL,
        global_height_mm=250.0)
    assert err is None, err
    assert plan.box_plane_height_mm == 250.0
    assert plan.e_right_bias_px == -3.0
    assert plan.e_bottom_bias_px == 0.0


def test_height_falls_back_to_box_type_then_global():
    profiles, err = parse_box_types(_box_types())
    assert err is None, err

    plan, err = resolve_slot_plan(
        profiles, box_type=0, slot=1, stage=STAGE_FINAL,
        global_height_mm=999.0)
    assert err is None, err
    assert plan.box_plane_height_mm == 300.0

    raw = _box_types()
    raw[0].pop("box_plane_height_mm")
    profiles, err = parse_box_types(raw)
    assert err is None, err
    plan, err = resolve_slot_plan(
        profiles, box_type=0, slot=1, stage=STAGE_FINAL,
        global_height_mm=999.0)
    assert err is None, err
    assert plan.box_plane_height_mm == 999.0


def test_neighbor_slot_may_omit_ref_edges():
    raw = [_box_types()[1]]
    raw[0]["slots"][0] = {"slot": 1, "mode": "neighbor", "side": "left"}
    profiles, err = parse_box_types(raw)
    assert err is None, err

    plan, err = resolve_slot_plan(
        profiles, box_type=1, slot=1, stage=STAGE_FINAL,
        global_height_mm=250.0)
    assert err is None, err
    assert plan.mode == "neighbor"
    assert plan.side == "left"
    assert plan.ref_edges is None


def test_pallet_slot_requires_ref_edges():
    raw = [_box_types()[1]]
    raw[0]["slots"][0] = {"slot": 1, "mode": "pallet"}
    profiles, err = parse_box_types(raw)
    assert err is not None
    assert "ref_edges" in err


def test_unknown_box_type_rejected():
    profiles, _ = parse_box_types(_box_types())
    plan, err = resolve_slot_plan(
        profiles, box_type=9, slot=1, stage=STAGE_FINAL,
        global_height_mm=250.0)
    assert plan is None
    assert "9" in err


def test_slot_out_of_range_rejected():
    profiles, _ = parse_box_types(_box_types())
    plan, err = resolve_slot_plan(
        profiles, box_type=0, slot=3, stage=STAGE_FINAL,
        global_height_mm=250.0)
    assert plan is None
    assert "3" in err


def test_illegal_stage_rejected():
    profiles, _ = parse_box_types(_box_types())
    plan, err = resolve_slot_plan(
        profiles, box_type=0, slot=1, stage=2,
        global_height_mm=250.0)
    assert plan is None
    assert "stage" in err


def test_slots_must_be_contiguous_from_one():
    raw = _box_types()
    raw[0]["slots"] = [raw[0]["slots"][0], {**raw[0]["slots"][1], "slot": 3}]
    profiles, err = parse_box_types(raw)
    assert profiles == ()
    assert "1..2" in err or "连续" in err


def test_duplicate_slot_rejected():
    raw = _box_types()
    raw[0]["slots"][1]["slot"] = 1
    profiles, err = parse_box_types(raw)
    assert profiles == ()
    assert "重复" in err or "1.." in err


def test_stage_override_rejects_route_keys():
    raw = _box_types()
    raw[0]["slots"][0]["initial"] = {
        "ref_edges": ["y=10", "x=900"],
        "mode": "neighbor",
    }
    profiles, err = parse_box_types(raw)
    assert profiles == ()
    assert "mode" in err


def test_invalid_ref_edge_syntax_rejected():
    raw = _box_types()
    raw[0]["slots"][0]["ref_edges"] = ["q=5", "x=900"]
    profiles, err = parse_box_types(raw)
    assert profiles == ()
    assert "q=5" in err


def test_legacy_config_builds_box_type_zero():
    profile = legacy_box_type(
        ref_edge_slots=[["y=50", "x=950"], ["y=0", "x=W"]],
        slot_modes=[
            ("pallet", "none", 0.0, 0.0, 250.0, (0.0, 0.0, 0.0)),
            ("neighbor", "right", -1.0, 2.0, None, (0.0, 0.0, 180.0)),
        ],
        name="默认箱型",
    )
    assert profile.box_type == 0
    assert [s.slot for s in profile.slots] == [1, 2]
    assert profile.slots[0].ref_edges == ("y=50", "x=950")
    assert profile.slots[1].mode == "neighbor"
    assert profile.slots[1].side == "right"
    assert profile.slots[1].normal_rot_deg == (0.0, 0.0, 180.0)

    plan, err = resolve_slot_plan(
        [profile], box_type=0, slot=1, stage=STAGE_INITIAL,
        global_height_mm=250.0)
    assert err is None, err
    assert plan.ref_edges == ("y=50", "x=950")
    assert plan.stage == STAGE_INITIAL


def test_legacy_default_box_type_has_zero_biases_and_global_height():
    profile = legacy_box_type(
        ref_edge_slots=[["y=0", "x=W"]],
        slot_modes=[("pallet", "none", 0.0, 0.0, None, (0.0, 0.0, 0.0))],
        name="默认箱型",
    )
    plan, err = resolve_slot_plan(
        [profile], box_type=0, slot=1, stage=STAGE_FINAL,
        global_height_mm=250.0)
    assert err is None, err
    assert plan.box_plane_height_mm == 250.0
    assert plan.e_right_bias_px == 0.0
    assert plan.e_bottom_bias_px == 0.0


def test_pallet_slot_can_put_ref_edges_only_in_both_stages():
    raw = [{
        "box_type": 0,
        "slots": [{
            "slot": 1,
            "mode": "pallet",
            "initial": {"ref_edges": ["y=10", "x=900"]},
            "final": {"ref_edges": ["y=50", "x=950"]},
        }],
    }]
    profiles, err = parse_box_types(raw)
    assert err is None, err

    initial, err = resolve_slot_plan(
        profiles, box_type=0, slot=1, stage=STAGE_INITIAL,
        global_height_mm=250.0)
    assert err is None, err
    assert initial.ref_edges == ("y=10", "x=900")

    final, err = resolve_slot_plan(
        profiles, box_type=0, slot=1, stage=STAGE_FINAL,
        global_height_mm=250.0)
    assert err is None, err
    assert final.ref_edges == ("y=50", "x=950")
