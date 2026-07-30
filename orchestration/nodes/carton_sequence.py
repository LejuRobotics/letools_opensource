# -*- coding: utf-8 -*-
"""纸箱"不满垛"识别与抓取序列生成纯算法模块。

本模块从 kuavo_ros_application 的 CartonPerceptionControl / CartonContext 移植，
不依赖 ROS、kuavo_humanoid_sdk 或任何硬件接口，仅做字符串解析与算术运算，
可被 LeTools 行为树节点直接调用。

数据流:
  GDRNPP 服务 /infer_top_carton_ids 返回 response.message 字符串，例如:
    "carton_type=type3; Returned 5 instances from 5 candidates;
     top ids: [4, 5]; estimated total cartons=17 (layers=4, top=2);
     top_orientation=pose_2; ..."

  本模块解析该字符串，得 carton_type / top_orientation / top_ids / estimated_total，
  再按箱型配置 (total_cartons / cartons_per_layer) 计算"缺箱数"与"抓取序列":
    empty_slots = total_cartons - estimated_total
    顶层剩余 box_index + 下方满层 box_index = 完整抓取序列
"""

from __future__ import annotations

import re
from typing import Any, Dict, List, Tuple

# ============================================================================
# 箱型常量
# ============================================================================

CARTON_TYPE_1 = "type1"
CARTON_TYPE_3 = "type3"

TOP_ORIENTATION_POSE_1 = "pose_1"
TOP_ORIENTATION_POSE_2 = "pose_2"

# 箱型布局（只保留不满垛判定所需字段；物理点位由场景 board.json 维护）
CARTON_LAYOUTS: Dict[str, Dict[str, Any]] = {
    CARTON_TYPE_1: {
        "total_cartons": 36,
        "total_layers": 6,
        "boxes_per_layer": 6,
    },
    CARTON_TYPE_3: {
        "total_cartons": 20,
        "total_layers": 4,
        "boxes_per_layer": 5,
    },
}

# ============================================================================
# 归一化函数（移植自 CartonContext.normalize_carton_type / normalize_top_orientation）
# ============================================================================


def normalize_carton_type(value: Any) -> str:
    """把 "1"/"type1"/"TYPE1" 等变体归一化为 "type1" / "type3"。"""
    if value is None:
        return CARTON_TYPE_1
    text = str(value).strip().lower()
    if not text:
        return CARTON_TYPE_1
    aliases = {
        "1": CARTON_TYPE_1,
        "type1": CARTON_TYPE_1,
        "carton_type1": CARTON_TYPE_1,
        "carton_type_1": CARTON_TYPE_1,
        "box_type1": CARTON_TYPE_1,
        "box_type_1": CARTON_TYPE_1,
        "first": CARTON_TYPE_1,
        "3": CARTON_TYPE_3,
        "type3": CARTON_TYPE_3,
        "carton_type3": CARTON_TYPE_3,
        "carton_type_3": CARTON_TYPE_3,
        "box_type3": CARTON_TYPE_3,
        "box_type_3": CARTON_TYPE_3,
    }
    if text not in aliases:
        raise ValueError(f"不支持的箱型参数: {value}")
    return aliases[text]


def normalize_top_orientation(value: Any, default: str = TOP_ORIENTATION_POSE_1) -> str:
    """把 "pose1"/"1"/"2" 等归一化为 "pose_1" / "pose_2"。"""
    if value is None:
        return default
    text = str(value).strip().lower()
    if not text:
        return default
    aliases = {
        "pose1": TOP_ORIENTATION_POSE_1,
        "pose_1": TOP_ORIENTATION_POSE_1,
        "1": TOP_ORIENTATION_POSE_1,
        "pose2": TOP_ORIENTATION_POSE_2,
        "pose_2": TOP_ORIENTATION_POSE_2,
        "2": TOP_ORIENTATION_POSE_2,
    }
    if text not in aliases:
        raise ValueError(f"不支持的顶部纸箱朝向参数: {value}")
    return aliases[text]


# ============================================================================
# 箱型布局查询
# ============================================================================


def get_carton_layout(box_type: Any = CARTON_TYPE_1) -> Dict[str, Any]:
    """获取指定箱型的布局配置。未知箱型回退到 type1。"""
    normalized = normalize_carton_type(box_type)
    return CARTON_LAYOUTS[normalized]


def get_boxes_per_layer(box_type: Any = CARTON_TYPE_1) -> int:
    """获取指定箱型每层的箱子数。"""
    return int(get_carton_layout(box_type)["boxes_per_layer"])


def get_total_cartons_for_box_type(box_type: Any = CARTON_TYPE_1) -> int:
    """获取指定箱型应有的箱子总数。"""
    layout = get_carton_layout(box_type)
    return int(
        layout.get(
            "total_cartons",
            int(layout["total_layers"]) * int(layout["boxes_per_layer"]),
        )
    )


def get_total_layers(box_type: Any = CARTON_TYPE_1) -> int:
    """获取指定箱型的总层数。"""
    return int(get_carton_layout(box_type)["total_layers"])


# ============================================================================
# message 字符串解析（移植自 CartonPerceptionControl）
# ============================================================================


def parse_carton_type_from_top_carton_message(message: str) -> str:
    """从 message 中解析 carton_type。"""
    match = re.search(r"carton_type\s*=\s*([0-9A-Za-z_]+)", message)
    if not match:
        raise ValueError(f"无法从 message 解析 carton_type: {message}")
    return normalize_carton_type(match.group(1))


def parse_top_orientation_from_top_carton_message(
    message: str, default: str = TOP_ORIENTATION_POSE_1
) -> str:
    """从 message 中解析 top_orientation，缺失时返回 default。"""
    match = re.search(r"top_orientation\s*=\s*([0-9A-Za-z_]+)", message)
    if not match:
        return default
    return normalize_top_orientation(match.group(1), default=default)


def parse_top_carton_message(message: str) -> Tuple[List[int], int]:
    """解析 message 中的 top ids 与 estimated total cartons。

    返回:
        (top_ids, estimated_total)
        top_ids: 顶层仍存在的纸箱局部编号列表，如 [4, 5]
        estimated_total: 视觉识别出的纸箱总数，如 17
    """
    top_ids_match = re.search(r"top ids:\s*\[([0-9,\s]+)\]", message)
    total_match = re.search(r"estimated total cartons\s*=\s*(\d+)", message)
    if not top_ids_match or not total_match:
        raise ValueError(f"无法从 message 解析 top ids / estimated total: {message}")
    top_ids = [int(x.strip()) for x in top_ids_match.group(1).split(",") if x.strip()]
    estimated_total = int(total_match.group(1))
    return top_ids, estimated_total


# ============================================================================
# 抓取序列生成（不满垛核心算法）
# ============================================================================


def build_box_pick_sequence_from_message(
    message: str,
    total_cartons: int = 36,
    cartons_per_layer: int = 6,
) -> List[int]:
    """根据视觉识别结果生成抓取序列（不满垛判定核心）。

    算法:
      1. 解析得 top_ids（顶层剩余纸箱局部编号）与 estimated_total（识别总数）
      2. empty_slots = total_cartons - estimated_total  ← 缺箱数（>0 即不满垛）
      3. top_layer_offset = (empty_slots // cartons_per_layer) * cartons_per_layer
         （缺箱数除以每层箱数，向下取整，得到顶层在全局编号中的起始偏移）
      4. top_layer_global_ids = [top_layer_offset + local_id for local_id in top_ids]
         （顶层剩余纸箱的全局 box_index）
      5. lower_full_ids = range(top_layer_offset + cartons_per_layer + 1, total_cartons + 1)
         （顶层下方所有满层的 box_index）
      6. 返回 top_layer_global_ids + lower_full_ids（先抓顶层剩余，再从上到下抓满层）

    示例（type3，total_cartons=20，cartons_per_layer=5，estimated_total=17，top_ids=[4,5]）:
      empty_slots = 3
      top_layer_offset = 0
      top_layer_global_ids = [4, 5]
      lower_full_ids = [6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20]
      返回 [4, 5, 6, 7, ..., 20]
    """
    top_ids, estimated_total = parse_top_carton_message(message)
    if estimated_total == 0:
        return []
    if len(top_ids) == 0:
        raise ValueError("top_ids 为空，无法生成抓取序列")

    # 以 top_ids（槽位匹配结果）为准修正 estimated_total
    # 视觉服务可能因重复检测导致 top_count 偏大，但 top_ids 经槽位匹配去重后更可靠
    layers = (estimated_total + cartons_per_layer - 1) // cartons_per_layer
    estimated_total = (layers - 1) * cartons_per_layer + len(top_ids)

    empty_slots = total_cartons - estimated_total
    top_layer_offset = (empty_slots // cartons_per_layer) * cartons_per_layer
    top_layer_global_ids = sorted(top_layer_offset + local_id for local_id in top_ids)
    lower_full_ids = list(
        range(top_layer_offset + cartons_per_layer + 1, total_cartons + 1)
    )
    return top_layer_global_ids + lower_full_ids


def build_pick_summary(message: str, box_type: Any = None) -> Dict[str, Any]:
    """一站式解析 message 并生成完整的不满垛摘要字典。

    Args:
        message: GDRNPP /infer_top_carton_ids 返回的 response.message
        box_type: 可选箱型（若未提供则从 message 解析）

    Returns:
        {
            "carton_type": "type3",
            "top_orientation": "pose_2",
            "top_ids": [4, 5],
            "estimated_total": 17,
            "total_cartons": 20,
            "cartons_per_layer": 5,
            "empty_slots": 3,                ← >0 即不满垛
            "is_partial_stack": True,        ← 不满垛标志
            "pick_sequence": [4, 5, 6, ..., 20],
        }
    """
    if box_type is None:
        try:
            box_type = parse_carton_type_from_top_carton_message(message)
        except ValueError:
            box_type = CARTON_TYPE_1  # 回退到默认箱型 type1
    else:
        box_type = normalize_carton_type(box_type)

    top_orientation = parse_top_orientation_from_top_carton_message(message)
    top_ids, estimated_total = parse_top_carton_message(message)
    total_cartons = get_total_cartons_for_box_type(box_type)
    cartons_per_layer = get_boxes_per_layer(box_type)

    # 以 top_ids 为准修正 estimated_total（与 build_box_pick_sequence_from_message 一致）
    if len(top_ids) > 0 and estimated_total > 0:
        layers = (estimated_total + cartons_per_layer - 1) // cartons_per_layer
        estimated_total = (layers - 1) * cartons_per_layer + len(top_ids)

    pick_sequence = build_box_pick_sequence_from_message(
        message, total_cartons=total_cartons, cartons_per_layer=cartons_per_layer
    )
    empty_slots = total_cartons - estimated_total
    return {
        "carton_type": box_type,
        "top_orientation": top_orientation,
        "top_ids": top_ids,
        "estimated_total": estimated_total,
        "total_cartons": total_cartons,
        "cartons_per_layer": cartons_per_layer,
        "empty_slots": empty_slots,
        "is_partial_stack": empty_slots > 0,
        "pick_sequence": pick_sequence,
    }
