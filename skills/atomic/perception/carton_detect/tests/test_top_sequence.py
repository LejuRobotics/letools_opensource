#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""顶层箱序几何的纯函数自检，不需要 ROS、相机或模型。"""

from __future__ import annotations

import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from top_sequence import infer_top_sequence, load_config  # noqa: E402


_FAILED = []
_TOP_IDS = re.compile(r"top ids:\s*\[([0-9,\s]+)\]")
_TOTAL = re.compile(r"estimated total cartons\s*=\s*(\d+)")
_TYPE = re.compile(r"carton_type\s*=\s*([0-9A-Za-z_]+)")
_ORIENTATION = re.compile(r"top_orientation\s*=\s*([0-9A-Za-z_]+)")


def check(name, cond, detail=""):
    if cond:
        print(f"  PASS  {name}")
    else:
        print(f"  FAIL  {name}  {detail}")
        _FAILED.append(name)


def _config():
    return {
        "carton_type": "type5",
        "top_orientation": "pose_1",
        "boxes_per_layer": 5,
        "max_layers": 6,
        "box_height_m": 0.18,
        "pallet_height_m": 0.17,
        "layer_z_threshold_m": 0.09,
        "surface_to_center_z_m": 0.0,
        "slot_match_max_dist_m": 0.30,
        "base_x_min": 0.3,
        "base_x_max": 2.2,
        "base_y_min": -0.8,
        "base_y_max": 0.8,
        "slots_calibrated": True,
        "slot_xy_map": {
            1: (1.00, -0.20),
            2: (1.00, 0.00),
            3: (1.00, 0.20),
            4: (1.40, -0.10),
            5: (1.40, 0.10),
        },
    }


def test_gapped_top_slots():
    # 顶层只命中槽位 1、3、4，下层一箱被高度阈值排除。
    z_top = 0.17 + 1.5 * 0.18
    points = [
        {"index": 0, "x": 1.02, "y": -0.18, "z": z_top},
        {"index": 1, "x": 1.01, "y": 0.19, "z": z_top - 0.01},
        {"index": 2, "x": 1.41, "y": -0.08, "z": z_top + 0.01},
        {"index": 3, "x": 1.00, "y": 0.00, "z": z_top - 0.40},
    ]
    result = infer_top_sequence(points, _config(), returned_count=4, candidate_count=6)
    check("缺槽顶层成功", result["success"], result["message"])
    check("槽位是 1、3、4", result["top_ids"] == [1, 3, 4], str(result["top_ids"]))
    message = result["message"]
    check("能解析箱型", _TYPE.search(message).group(1) == "type5")
    check("能解析朝向", _ORIENTATION.search(message).group(1) == "pose_1")
    top_ids = [
        int(item.strip())
        for item in _TOP_IDS.search(message).group(1).split(",")
        if item.strip()
    ]
    check("message 槽位与结果一致", top_ids == [1, 3, 4], message)
    check("估算总数字段存在", _TOTAL.search(message) is not None, message)


def test_uncalibrated_slots_do_not_emit_sequence():
    config = _config()
    config["slots_calibrated"] = False
    result = infer_top_sequence(
        [{"index": 0, "x": 1.0, "y": 0.0, "z": 0.4}],
        config,
        returned_count=1,
        candidate_count=1,
    )
    check("未标定失败", not result["success"])
    check(
        "失败文本不能被当成箱序",
        _TOP_IDS.search(result["message"]) is None,
        result["message"],
    )


def test_shipped_config_stays_uncalibrated():
    config_path = (
        Path(__file__).resolve().parents[5]
        / "orchestration/scenarios/scenarios_internal/carton_obb_trt_service_v2/top_sequence.yaml"
    )
    config = load_config(str(config_path))
    check("现场配置先不标定", config["slots_calibrated"] is False)
    check("现场配置是 type5 每层 5 箱", config["boxes_per_layer"] == 5)
    check("五个槽位都还是原点", set(config["slot_xy_map"].values()) == {(0.0, 0.0)})


def main():
    test_gapped_top_slots()
    test_uncalibrated_slots_do_not_emit_sequence()
    test_shipped_config_stays_uncalibrated()
    if _FAILED:
        print(f"FAILED {len(_FAILED)}")
        return 1
    print("OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
