# -*- coding: utf-8 -*-
"""由基座系三维点反推顶层箱序。

输入是已经投影好的点，不调用模型。手里箱和邻箱在进入这里之前已经合成一类。
输出文本与原先 ``/infer_top_carton_ids`` 的 ``message`` 一致，供主流程原样解析。
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np
import yaml


Point = Mapping[str, float]


def load_config(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle)
    if not isinstance(raw, dict):
        raise ValueError(f"箱序配置必须是映射：{path}")
    return _normalize_config(raw)


def infer_top_sequence(
    points: Sequence[Point],
    config: Mapping[str, Any],
    returned_count: int,
    candidate_count: int,
) -> Dict[str, Any]:
    """按高度留顶层，再按平面位置匹配槽位，生成原来的 message。"""
    if not config.get("slots_calibrated", False):
        return _failure("槽位坐标尚未标定，拒绝生成箱序")

    kept = _filter_xy(points, config)
    if not kept:
        return _failure("投影点落在基座系平面范围外")

    top_layer = _select_top_layer(kept, float(config["layer_z_threshold_m"]))
    assignments = _assign_slots(
        top_layer,
        config["slot_xy_map"],
        float(config["slot_match_max_dist_m"]),
    )
    matched = [point for point in top_layer if int(point["index"]) in assignments]
    if not matched:
        return _failure("顶层箱与槽位模板的距离都超过匹配阈值")

    top_ids = sorted(int(assignments[int(point["index"])]) for point in matched)
    z_mean = float(np.mean([float(point["z"]) for point in matched]))
    z_center = z_mean + float(config["surface_to_center_z_m"])
    layer_count = _estimate_layer_count(
        z_center,
        float(config["box_height_m"]),
        float(config["pallet_height_m"]),
        int(config["max_layers"]),
    )
    boxes_per_layer = int(config["boxes_per_layer"])
    total_count = boxes_per_layer * (layer_count - 1) + len(top_ids)
    message = format_top_sequence_message(
        carton_type=str(config["carton_type"]),
        top_orientation=str(config["top_orientation"]),
        top_ids=top_ids,
        total_count=total_count,
        layer_count=layer_count,
        top_count=len(top_ids),
        returned_count=int(returned_count),
        candidate_count=int(candidate_count),
    )
    return {
        "success": True,
        "message": message,
        "top_ids": top_ids,
        "layer_count": layer_count,
        "total_count": total_count,
        "matched": matched,
    }


def format_top_sequence_message(
    carton_type: str,
    top_orientation: str,
    top_ids: Sequence[int],
    total_count: int,
    layer_count: int,
    top_count: int,
    returned_count: int,
    candidate_count: int,
) -> str:
    id_text = ", ".join(str(int(slot_id)) for slot_id in top_ids)
    return (
        f"carton_type={carton_type}; "
        f"Returned {int(returned_count)} instances from {int(candidate_count)} candidates; "
        f"top ids: [{id_text}]; "
        f"estimated total cartons={int(total_count)} "
        f"(layers={int(layer_count)}, top={int(top_count)}); "
        f"top_orientation={top_orientation}."
    )


def _normalize_config(raw: Mapping[str, Any]) -> Dict[str, Any]:
    boxes_per_layer = _positive_int(raw.get("boxes_per_layer"), "boxes_per_layer")
    slot_xy_map = _parse_slot_map(raw.get("slot_xy_map"), boxes_per_layer)
    calibrated = bool(raw.get("slots_calibrated", False))
    if calibrated:
        if sorted(slot_xy_map) != list(range(1, boxes_per_layer + 1)):
            raise ValueError(
                f"已标定槽位必须正好覆盖 1..{boxes_per_layer}，当前 {sorted(slot_xy_map)}"
            )
        if any(xy == (0.0, 0.0) for xy in slot_xy_map.values()):
            raise ValueError("已标定槽位不能是 (0, 0)")
    x_min = _finite(raw.get("base_x_min"), "base_x_min")
    x_max = _finite(raw.get("base_x_max"), "base_x_max")
    y_min = _finite(raw.get("base_y_min"), "base_y_min")
    y_max = _finite(raw.get("base_y_max"), "base_y_max")
    if x_min >= x_max or y_min >= y_max:
        raise ValueError("基座系平面范围无效")
    return {
        "carton_type": _token(raw.get("carton_type"), "carton_type"),
        "top_orientation": _token(raw.get("top_orientation"), "top_orientation"),
        "boxes_per_layer": boxes_per_layer,
        "max_layers": _positive_int(raw.get("max_layers"), "max_layers"),
        "box_height_m": _positive(raw.get("box_height_m"), "box_height_m"),
        "pallet_height_m": _finite(raw.get("pallet_height_m"), "pallet_height_m"),
        "layer_z_threshold_m": _positive(raw.get("layer_z_threshold_m"), "layer_z_threshold_m"),
        "surface_to_center_z_m": _finite(
            raw.get("surface_to_center_z_m", 0.0),
            "surface_to_center_z_m",
        ),
        "slot_match_max_dist_m": _positive(
            raw.get("slot_match_max_dist_m"),
            "slot_match_max_dist_m",
        ),
        "base_x_min": x_min,
        "base_x_max": x_max,
        "base_y_min": y_min,
        "base_y_max": y_max,
        "slots_calibrated": calibrated,
        "slot_xy_map": slot_xy_map,
        "confidence": _unit_interval(raw.get("confidence", 0.25), "confidence"),
        "nms_iou": _unit_interval(raw.get("nms_iou", 0.45), "nms_iou"),
        "depth_shrink": _shrink(raw.get("depth_shrink", 0.6)),
        "depth_mad_scale": _positive(raw.get("depth_mad_scale", 3.5), "depth_mad_scale"),
        "depth_outlier_floor_m": _non_negative(
            raw.get("depth_outlier_floor_m", 0.02),
            "depth_outlier_floor_m",
        ),
        "min_depth_samples": _positive_int(raw.get("min_depth_samples", 30), "min_depth_samples"),
        "min_depth_m": _finite(raw.get("min_depth_m", 0.1), "min_depth_m"),
        "max_depth_m": _finite(raw.get("max_depth_m", 10.0), "max_depth_m"),
        "max_depth_age_s": _non_negative(raw.get("max_depth_age_s", 0.2), "max_depth_age_s"),
    }


def _filter_xy(points: Sequence[Point], config: Mapping[str, Any]) -> List[Point]:
    kept = []
    for point in points:
        x_val = float(point["x"])
        y_val = float(point["y"])
        z_val = float(point["z"])
        if not all(math.isfinite(value) for value in (x_val, y_val, z_val)):
            continue
        if not (config["base_x_min"] <= x_val <= config["base_x_max"]):
            continue
        if not (config["base_y_min"] <= y_val <= config["base_y_max"]):
            continue
        kept.append(point)
    return kept


def _select_top_layer(points: Sequence[Point], layer_z_threshold: float) -> List[Point]:
    max_z = max(float(point["z"]) for point in points)
    return [
        point
        for point in points
        if float(point["z"]) >= max_z - float(layer_z_threshold)
    ]


def _assign_slots(
    points: Sequence[Point],
    slot_xy_map: Mapping[int, Tuple[float, float]],
    max_match_dist_m: float,
) -> Dict[int, int]:
    remaining = {
        int(slot_id): np.array([float(xy[0]), float(xy[1])], dtype=np.float64)
        for slot_id, xy in slot_xy_map.items()
    }
    pending = {
        int(point["index"]): np.array(
            [float(point["x"]), float(point["y"])],
            dtype=np.float64,
        )
        for point in points
    }
    assignments: Dict[int, int] = {}
    while pending and remaining:
        best: Optional[Tuple[float, int, int]] = None
        for det_idx, det_xy in pending.items():
            for slot_id, slot_xy in remaining.items():
                dist = float(np.linalg.norm(det_xy - slot_xy))
                if best is None or dist < best[0]:
                    best = (dist, det_idx, slot_id)
        if best is None or best[0] > float(max_match_dist_m):
            break
        _, det_idx, slot_id = best
        assignments[det_idx] = int(slot_id)
        del pending[det_idx]
        del remaining[slot_id]
    return assignments


def _estimate_layer_count(
    z_center: float,
    box_height_m: float,
    pallet_height_m: float,
    max_layers: int,
) -> int:
    raw_layers = (float(z_center) - float(pallet_height_m)) / float(box_height_m) + 0.5
    layer_count = int(np.rint(raw_layers))
    return max(1, min(int(max_layers), layer_count))


def _parse_slot_map(raw: Any, boxes_per_layer: int) -> Dict[int, Tuple[float, float]]:
    if not isinstance(raw, dict) or not raw:
        raise ValueError("slot_xy_map 不能为空")
    slots: Dict[int, Tuple[float, float]] = {}
    for slot_id, xy in raw.items():
        slot_int = int(slot_id)
        if slot_int < 1 or slot_int > boxes_per_layer:
            raise ValueError(f"槽位号超出 1..{boxes_per_layer}：{slot_int}")
        if not isinstance(xy, (list, tuple)) or len(xy) != 2:
            raise ValueError(f"槽位 {slot_int} 必须是 [x, y]")
        slots[slot_int] = (
            _finite(xy[0], f"slot {slot_int} x"),
            _finite(xy[1], f"slot {slot_int} y"),
        )
    return slots


def _failure(message: str) -> Dict[str, Any]:
    return {
        "success": False,
        "message": message,
        "top_ids": [],
        "layer_count": 0,
        "total_count": 0,
        "matched": [],
    }


def _token(value: Any, name: str) -> str:
    text = str(value or "").strip()
    if not text or any(ch.isspace() for ch in text):
        raise ValueError(f"{name} 必须是不含空白的标识")
    return text


def _finite(value: Any, name: str) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{name} 必须是有限数")
    return number


def _positive(value: Any, name: str) -> float:
    number = _finite(value, name)
    if number <= 0.0:
        raise ValueError(f"{name} 必须为正")
    return number


def _non_negative(value: Any, name: str) -> float:
    number = _finite(value, name)
    if number < 0.0:
        raise ValueError(f"{name} 不能为负")
    return number


def _positive_int(value: Any, name: str) -> int:
    number = int(value)
    if number <= 0:
        raise ValueError(f"{name} 必须是正整数")
    return number


def _unit_interval(value: Any, name: str) -> float:
    number = _finite(value, name)
    if not 0.0 <= number <= 1.0:
        raise ValueError(f"{name} 必须在 0 到 1 之间")
    return number


def _shrink(value: Any) -> float:
    number = _positive(value, "depth_shrink")
    if number > 1.0:
        raise ValueError("depth_shrink 必须在 (0, 1]")
    return number
