#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""type6 专用：按**横向**顺序推算下一个码放槽位（与 `next_slot` 同 API）。

与 `next_slot.py` 的区别只有一处：槽位顺序表是 type6 垛型的（12 槽、只有横向），
其余（读记录、已码放判据、`allowed` 过滤、错误类型、CLI）**完全复用**，
这样 type1/3/4/5 的老行为一个字节都不受影响。

槽位顺序（操作员给定，2 层 × 6 槽）：

    横向：2, 1, 4, 3, 5, 6, 8, 7, 10, 9, 11, 12

⚠️ type6 **没有竖向**配置：收到 vertical 直接报错（宁可明确失败，也不要静默排错槽）。
⚠️ 本文件是**新增**文件；任何旧箱型都不会走到这里（由 profile 的
   `place_slot_order_module` 字段决定用哪个模块，默认仍是 `next_slot`）。

命令行（与 next_slot.py 一致）：

    python3 next_slot2.py --orientation 横
    python3 next_slot2.py --orientation h --json
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Iterable, Optional

try:                                                    # 正常：作为包被 import
    from skills.atomic.perception.pallet_servo_v2.next_slot import (
        RecordError,
        SlotFullError,
        _default_record_path,
        _slots_from_board,
        load_placed_slots,
        normalize_orientation,
    )
except ImportError:                                     # 直接 `python3 next_slot2.py`
    sys.path.insert(0, str(Path(__file__).resolve().parents[4]))
    from skills.atomic.perception.pallet_servo_v2.next_slot import (  # noqa: E402
        RecordError,
        SlotFullError,
        _default_record_path,
        _slots_from_board,
        load_placed_slots,
        normalize_orientation,
    )

# 操作员给定的占据顺序。type6 只有横向。
PLACE_ORDERS = {
    "horizontal": (2, 1, 4, 3, 5, 6, 8, 7, 10, 9, 11, 12),
}

SUPPORTED_DIRECTIONS = ("horizontal",)


def _order_for(direction: str,
               allowed: Optional[Iterable[int]] = None) -> tuple:
    """该方向的槽位顺序；`allowed` 给了就只保留箱型表里实际存在的槽位。"""
    if direction not in PLACE_ORDERS:
        raise ValueError(
            f"type6 只有横向（horizontal）槽位表，收到 {direction!r}；"
            f"支持的方向：{list(SUPPORTED_DIRECTIONS)}"
        )
    order = PLACE_ORDERS[direction]
    if allowed is None:
        return order
    allowed_set = {int(v) for v in allowed}
    return tuple(slot for slot in order if slot in allowed_set)


def next_slot(orientation: str, placed: Iterable[int],
              allowed: Optional[Iterable[int]] = None) -> int:
    """按方向返回第一个尚未码放的槽位；该方向全满抛 `SlotFullError`。"""
    direction = normalize_orientation(orientation)
    order = _order_for(direction, allowed)
    occupied = {int(v) for v in placed}
    for slot in order:
        if slot not in occupied:
            return slot
    raise SlotFullError("横" if direction == "horizontal" else "竖", order)


def remaining_slots(orientation: str, placed: Iterable[int],
                     allowed: Optional[Iterable[int]] = None) -> list:
    """该方向还空着的槽位（按占据顺序；`allowed` 限定当前箱型的槽位表）。"""
    direction = normalize_orientation(orientation)
    occupied = {int(v) for v in placed}
    return [slot for slot in _order_for(direction, allowed)
            if slot not in occupied]


def main(argv: Optional[list] = None) -> int:
    ap = argparse.ArgumentParser(
        description="type6：按箱子横向方向，从托盘记录推算下一个码放槽位")
    ap.add_argument("--orientation", "-o", required=True,
                    help="箱子方向：横/horizontal/h（type6 不支持竖）")
    ap.add_argument("--record", type=Path, default=None,
                    help="托盘记录文件；默认 <仓库根>/log/pallet_servo_pallet_record.json")
    ap.add_argument("--box-type", type=int, default=0,
                    help="箱型编号；不同箱型有独立的槽位表与已码放进度（默认 0）")
    ap.add_argument("--board", type=Path, default=None,
                    help="可选：board.json。给了就按该箱型的 box_types 槽位表过滤顺序")
    ap.add_argument("--json", action="store_true",
                    help="输出 JSON（含 placed_slots / remaining），而不是单个槽位号")
    args = ap.parse_args(argv)

    try:
        direction = normalize_orientation(args.orientation)
    except ValueError as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return 3

    record_path = args.record or _default_record_path()
    allowed = None
    if args.board is not None:
        try:
            allowed = _slots_from_board(args.board, args.box_type)
        except RecordError as exc:
            print(f"错误：{exc}", file=sys.stderr)
            return 2
    try:
        placed = load_placed_slots(record_path, box_type=args.box_type)
    except RecordError as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return 2

    try:
        order = _order_for(direction, allowed)
        slot = next_slot(direction, placed, allowed)
    except (ValueError, SlotFullError) as exc:
        if args.json:
            print(json.dumps({
                "ok": False,
                "orientation": direction,
                "box_type": int(args.box_type),
                "error": str(exc),
                "placed_slots": sorted(placed),
                "remaining": [],
            }, ensure_ascii=False))
        else:
            print(f"错误：{exc}", file=sys.stderr)
        return 1

    remaining = remaining_slots(direction, placed, allowed)
    if args.json:
        print(json.dumps({
            "ok": True,
            "orientation": direction,
            "box_type": int(args.box_type),
            "next_slot": slot,
            "placed_slots": sorted(placed),
            "order": list(order),
            "remaining": remaining,
            "record": str(record_path),
        }, ensure_ascii=False))
    else:
        print(slot)
        print(f"箱型 {int(args.box_type)}：已码放 {sorted(placed) or '无'}；"
              f"横向槽位顺序 {list(order)}；下一槽位 {slot}",
              file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
