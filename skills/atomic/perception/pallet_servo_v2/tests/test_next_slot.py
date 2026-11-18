# -*- coding: utf-8 -*-
"""下一个码放槽位推算的单元测试（纯函数 + CLI，不需要 ROS）。"""
import json

import pytest

from ..next_slot import (
    PLACE_ORDERS,
    RecordError,
    SlotFullError,
    load_placed_slots,
    main,
    next_slot,
    normalize_orientation,
    remaining_slots,
)

pytestmark = pytest.mark.unit


def _write_record(tmp_path, payload):
    path = tmp_path / "pallet_record.json"
    path.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")
    return path


def test_orders_are_exactly_the_operator_given_sequences():
    assert PLACE_ORDERS["horizontal"] == (3, 2, 5, 8, 7, 10)
    assert PLACE_ORDERS["vertical"] == (1, 4, 6, 9)


@pytest.mark.parametrize("alias, want", [
    ("横", "horizontal"), ("横向", "horizontal"), ("h", "horizontal"),
    ("H", "horizontal"), ("horizontal", "horizontal"),
    ("竖", "vertical"), ("竖向", "vertical"), ("v", "vertical"),
    ("V", "vertical"), ("vertical", "vertical"),
])
def test_orientation_aliases(alias, want):
    assert normalize_orientation(alias) == want


def test_unknown_orientation_is_rejected():
    with pytest.raises(ValueError):
        normalize_orientation("斜")


@pytest.mark.parametrize("placed, want", [
    (set(), 3), ({3}, 2), ({3, 2}, 5), ({2}, 3),
    ({3, 2, 5, 8, 7}, 10), ({1, 4, 6, 9}, 3),   # 竖的占位不影响横
])
def test_horizontal_sequence(placed, want):
    assert next_slot("横", placed) == want


@pytest.mark.parametrize("placed, want", [
    (set(), 1), ({1}, 4), ({1, 4}, 6), ({1, 4, 6}, 9),
    ({4}, 1), ({3, 2, 5, 8, 7, 10}, 1),         # 横满不影响竖
])
def test_vertical_sequence(placed, want):
    assert next_slot("竖", placed) == want


def test_full_direction_raises_with_clear_message():
    with pytest.raises(SlotFullError) as exc:
        next_slot("横", {3, 2, 5, 8, 7, 10})
    assert "横向槽位已全部码放" in str(exc.value)
    assert "3, 2, 5, 8, 7, 10" in str(exc.value)

    with pytest.raises(SlotFullError):
        next_slot("竖", {1, 4, 6, 9})


def test_remaining_slots_follow_the_order():
    placed = {3, 5, 1}
    assert remaining_slots("横", placed) == [2, 8, 7, 10]
    assert remaining_slots("竖", placed) == [4, 6, 9]


def test_loads_placed_slots_from_record(tmp_path):
    path = _write_record(tmp_path, {"version": 1, "placed_slots": [1, 3]})
    assert load_placed_slots(path) == {1, 3}


def test_falls_back_to_placements_when_placed_slots_is_absent(tmp_path):
    path = _write_record(tmp_path, {
        "version": 1,
        "placements": {"2": {"slot": 2}, "5": {"slot": 5}},
    })
    assert load_placed_slots(path) == {2, 5}


def test_old_record_without_position_fields_is_empty(tmp_path):
    path = _write_record(tmp_path, {"version": 1, "D_mm": 890.0})
    assert load_placed_slots(path) == set()


@pytest.mark.parametrize("payload", [
    {"placed_slots": "1,2"},
    {"placed_slots": [1, "x"]},
    {"placements": [1, 2]},
    {"placements": {"x": {}}},
])
def test_bad_position_fields_are_record_errors(tmp_path, payload):
    path = _write_record(tmp_path, payload)
    with pytest.raises(RecordError):
        load_placed_slots(path)


def test_missing_file_is_a_record_error(tmp_path):
    with pytest.raises(RecordError):
        load_placed_slots(tmp_path / "none.json")


def test_cli_prints_only_the_slot_number(tmp_path, capsys):
    path = _write_record(tmp_path, {"placed_slots": [1, 3]})
    assert main(["--orientation", "横", "--record", str(path)]) == 0
    out = capsys.readouterr()
    assert out.out.strip() == "2", out.out
    assert "下一槽位 2" in out.err, out.err


def test_cli_json_mode(tmp_path, capsys):
    path = _write_record(tmp_path, {"placed_slots": [1]})
    assert main(["--orientation", "竖", "--record", str(path), "--json"]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["ok"] is True
    assert payload["next_slot"] == 4
    assert payload["remaining"] == [4, 6, 9]
    assert payload["placed_slots"] == [1]


def test_cli_full_direction_exits_1(tmp_path, capsys):
    path = _write_record(tmp_path, {"placed_slots": [3, 2, 5, 8, 7, 10]})
    assert main(["--orientation", "横", "--record", str(path)]) == 1
    err = capsys.readouterr().err
    assert "横向槽位已全部码放" in err, err


def test_cli_missing_record_exits_2(tmp_path, capsys):
    assert main(["--orientation", "横", "--record", str(tmp_path / "none.json")]) == 2
    assert "不存在" in capsys.readouterr().err


def test_cli_unknown_orientation_exits_3(tmp_path):
    path = _write_record(tmp_path, {"placed_slots": []})
    assert main(["--orientation", "斜", "--record", str(path)]) == 3


def test_reads_per_box_type_placements(tmp_path):
    path = _write_record(tmp_path, {
        "version": 1,
        "placements_by_box_type": {
            "0": {"placed_slots": [1, 3]},
            "1": {"placed_slots": [2]},
        },
    })
    assert load_placed_slots(path, box_type=0) == {1, 3}
    assert load_placed_slots(path, box_type=1) == {2}


def test_missing_box_type_bucket_is_empty(tmp_path):
    path = _write_record(tmp_path, {
        "version": 1,
        "placements_by_box_type": {"0": {"placed_slots": [1]}},
    })
    assert load_placed_slots(path, box_type=9) == set()


def test_legacy_flat_record_only_applies_to_box_type_zero(tmp_path):
    path = _write_record(tmp_path, {"version": 1, "placed_slots": [1, 3]})
    assert load_placed_slots(path, box_type=0) == {1, 3}
    assert load_placed_slots(path, box_type=1) == set()


def test_cli_accepts_box_type(tmp_path, capsys):
    path = _write_record(tmp_path, {
        "placements_by_box_type": {"1": {"placed_slots": [1, 3]}},
    })
    assert main(["--orientation", "横", "--record", str(path),
                 "--box-type", "1"]) == 0
    out = capsys.readouterr()
    assert out.out.strip() == "2", out.out


def test_board_limits_slots_to_the_box_type_table(tmp_path, capsys):
    board_path = tmp_path / "board.json"
    board_path.write_text(json.dumps({
        "pallet": [{
            "key": "box_types",
            "value": [{
                "box_type": 1,
                "slots": [{"slot": 1}, {"slot": 4}],
            }],
        }],
    }), encoding="utf-8")
    record = _write_record(tmp_path, {
        "placements_by_box_type": {"1": {"placed_slots": [1]}},
    })
    assert main(["--orientation", "竖", "--record", str(record),
                 "--box-type", "1", "--board", str(board_path)]) == 0
    out = capsys.readouterr()
    assert out.out.strip() == "4", out.out


def test_board_missing_box_type_is_an_error(tmp_path, capsys):
    board_path = tmp_path / "board.json"
    board_path.write_text(json.dumps({
        "pallet": [{"key": "box_types", "value": [{"box_type": 0, "slots": [{"slot": 1}]}]}],
    }), encoding="utf-8")
    record = _write_record(tmp_path, {})
    assert main(["--orientation", "竖", "--record", str(record),
                 "--box-type", "9", "--board", str(board_path)]) == 2
    assert "9" in capsys.readouterr().err
