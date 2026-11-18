#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""按手中箱子的横/竖方向，推算下一个要码放的托盘槽位。

只读 `pallet_servo` 的 call 0 记录文件（`pallet_record_path`），**不修改它**；
槽位"已码放"的判据是记录里的 `placements_by_box_type[<box_type>].placed_slots`
（伺服每次 **stage=0 最终伺服** 时写入）。老记录的扁平 `placed_slots` /
`placements` 只按**箱型 0** 读取；两者都没有 = 尚未码放任何箱子。

不同箱型有独立的槽位表与进度，所以命令行要带 `--box-type`（默认 0）。

槽位顺序（操作员给定，横竖各自独立）：

    横：3, 2, 5, 8, 7, 10
    竖：1, 4, 6, 9

命令行（默认只把槽位号打到 stdout，方便上层脚本直接取）：

    python3 skills/atomic/perception/pallet_servo_v2/next_slot.py --orientation 横
    python3 .../next_slot.py --orientation 竖 --record /path/to/pallet_record.json
    python3 .../next_slot.py --orientation h --json

退出码：0 = 有可用槽位；1 = 该方向已满；2 = 记录文件缺失/损坏。
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Iterable, Optional

# 操作员给定的占据顺序。**横竖各自独立**，不要把两张表混在一起排序。
PLACE_ORDERS = {
    "horizontal": (3, 2, 5, 8, 7, 10),
    "vertical": (1, 4, 6, 9),
}

# 命令行别名（大小写不敏感；中文直接匹配）。
_ORIENTATION_ALIASES = {
    "h": "horizontal",
    "horizontal": "horizontal",
    "horiz": "horizontal",
    "横": "horizontal",
    "横向": "horizontal",
    "v": "vertical",
    "vertical": "vertical",
    "vert": "vertical",
    "竖": "vertical",
    "竖向": "vertical",
    "竖着": "vertical",
}


class SlotFullError(RuntimeError):
    """该方向的槽位已经全部码放，又来一个同方向的箱子。"""

    def __init__(self, orientation: str, order: Iterable[int]):
        self.orientation = orientation
        self.order = tuple(order)
        if self.order:
            message = (
                f"{orientation}向槽位已全部码放"
                f"（{', '.join(map(str, self.order))}）"
                f" —— 又来一个{orientation}向箱子，无可用槽位")
        else:
            message = (f"{orientation}向在当前箱型的槽位表里没有可用槽位"
                       f" —— 检查 board.json 的 box_types")
        super().__init__(message)


class RecordError(RuntimeError):
    """记录文件缺失、读不了或内容不合法。"""


def normalize_orientation(value: str) -> str:
    """`横/竖/h/v/...` → `horizontal` / `vertical`；认不出来抛 ValueError。"""
    key = str(value).strip().lower()
    try:
        return _ORIENTATION_ALIASES[key]
    except KeyError:
        raise ValueError(
            f"orientation={value!r} 不认识 —— 只认 横/horizontal/h 或 竖/vertical/v"
        ) from None


def _placed_from_bucket(bucket: dict, label: str) -> set[int]:
    """一个箱型的 placements 桶 → 已码放槽位集合。"""
    placed = bucket.get("placed_slots")
    if placed is not None:
        if not isinstance(placed, (list, tuple)):
            raise RecordError(f"{label}.placed_slots 不是列表")
        occupied = set()
        for item in placed:
            try:
                occupied.add(int(item))
            except (TypeError, ValueError):
                raise RecordError(
                    f"{label}.placed_slots 里有非整数：{item!r}") from None
        return occupied
    entries = bucket.get("slots")
    if entries is not None:
        if not isinstance(entries, dict):
            raise RecordError(f"{label}.slots 不是对象")
        occupied = set()
        for key in entries:
            try:
                occupied.add(int(key))
            except (TypeError, ValueError):
                raise RecordError(
                    f"{label}.slots 的键不是槽位号：{key!r}") from None
        return occupied
    return set()


def _occupied_from_record(record: dict, box_type: int = 0) -> set[int]:
    """从记录 JSON 取出**指定箱型**已码放的槽位集合。

    新记录按箱型分表（`placements_by_box_type`）；旧记录的扁平
    `placed_slots` / `placements` 只属于箱型 0 —— 拿它去当别的箱型的
    进度会让 next_slot 算错。
    """
    by_type = record.get("placements_by_box_type")
    if by_type is not None:
        if not isinstance(by_type, dict):
            raise RecordError("placements_by_box_type 不是对象")
        bucket = by_type.get(str(int(box_type)))
        if bucket is None:
            return set()
        if not isinstance(bucket, dict):
            raise RecordError(
                f"placements_by_box_type[{box_type}] 不是对象")
        return _placed_from_bucket(
            bucket, f"placements_by_box_type[{box_type}]")

    # 旧记录的扁平字段只对箱型 0 有效。
    if int(box_type) != 0:
        return set()

    placed = record.get("placed_slots")
    if placed is not None:
        if not isinstance(placed, (list, tuple)):
            raise RecordError("placed_slots 不是列表")
        occupied = set()
        for item in placed:
            try:
                occupied.add(int(item))
            except (TypeError, ValueError):
                raise RecordError(f"placed_slots 里有非整数：{item!r}") from None
        return occupied

    placements = record.get("placements")
    if placements is not None:
        if not isinstance(placements, dict):
            raise RecordError("placements 不是对象")
        occupied = set()
        for key in placements:
            try:
                occupied.add(int(key))
            except (TypeError, ValueError):
                raise RecordError(f"placements 的键不是槽位号：{key!r}") from None
        return occupied

    # 老记录（还没有位置字段）= 尚未码放任何箱子。
    return set()


def load_placed_slots(record_path: Path, box_type: int = 0) -> set[int]:
    """读托盘记录文件，返回**指定箱型**已码放的槽位号集合。"""
    path = Path(record_path).expanduser()
    if not path.is_file():
        raise RecordError(f"托盘记录文件不存在：{path}")
    try:
        with open(path, "r", encoding="utf-8") as stream:
            record = json.load(stream)
    except Exception as exc:                        # noqa: BLE001
        raise RecordError(f"读取托盘记录失败（{type(exc).__name__}: {exc}）：{path}") from exc
    if not isinstance(record, dict):
        raise RecordError(f"托盘记录不是 JSON 对象：{path}")
    return _occupied_from_record(record, box_type=box_type)


def _order_for(direction: str, allowed: Optional[Iterable[int]] = None):
    order = PLACE_ORDERS[direction]
    if allowed is None:
        return order
    allowed_set = {int(v) for v in allowed}
    return tuple(slot for slot in order if slot in allowed_set)


def next_slot(orientation: str, placed: Iterable[int],
              allowed: Optional[Iterable[int]] = None) -> int:
    """按方向返回第一个尚未码放的槽位；该方向全满抛 `SlotFullError`。

    `allowed` 给了就只在该箱型实际配置的槽位里选（`--board` 读到的槽位表）。
    """
    direction = normalize_orientation(orientation)
    order = _order_for(direction, allowed)
    occupied = {int(v) for v in placed}
    for slot in order:
        if slot not in occupied:
            return slot
    raise SlotFullError("横" if direction == "horizontal" else "竖", order)


def remaining_slots(orientation: str, placed: Iterable[int],
                    allowed: Optional[Iterable[int]] = None) -> list[int]:
    """该方向还空着的槽位（按占据顺序；`allowed` 限定当前箱型的槽位表）。"""
    direction = normalize_orientation(orientation)
    occupied = {int(v) for v in placed}
    return [slot for slot in _order_for(direction, allowed)
            if slot not in occupied]


def _slots_from_board(board_path: Path, box_type: int) -> set[int]:
    """从 board.json 的 `box_types` 取某个箱型实际配置的槽位号。"""
    path = Path(board_path).expanduser()
    try:
        with open(path, "r", encoding="utf-8") as stream:
            board = json.load(stream)
    except Exception as exc:                        # noqa: BLE001
        raise RecordError(
            f"读取 board.json 失败（{type(exc).__name__}: {exc}）：{path}") from exc
    if not isinstance(board, dict):
        raise RecordError(f"board.json 不是 JSON 对象：{path}")
    items = [item for group in board.values() if isinstance(group, list)
             for item in group if isinstance(item, dict)]
    box_item = next((item for item in items
                     if item.get("key") == "box_types"), None)
    if box_item is None:
        raise RecordError(f"board.json 里没有 box_types：{path}")
    profiles = box_item.get("value")
    if not isinstance(profiles, list):
        raise RecordError(f"board.json 的 box_types 不是数组：{path}")
    profile = next((profile for profile in profiles
                    if isinstance(profile, dict)
                    and int(profile.get("box_type", -1)) == int(box_type)), None)
    if profile is None:
        known = [profile.get("box_type") for profile in profiles
                 if isinstance(profile, dict)]
        raise RecordError(
            f"board.json 里没有箱型 {box_type}（已配置 {known}）：{path}")
    slots = profile.get("slots")
    if not isinstance(slots, list) or not slots:
        raise RecordError(f"箱型 {box_type} 的 slots 不是非空数组：{path}")
    try:
        return {int(slot["slot"]) for slot in slots if isinstance(slot, dict)}
    except (KeyError, TypeError, ValueError):
        raise RecordError(f"箱型 {box_type} 的槽位号不合法：{path}") from None


def _default_record_path() -> Path:
    """仓库根/log/pallet_servo_pallet_record.json（与 servo 节点默认一致）。"""
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "skills").is_dir() and (parent / "orchestration").is_dir():
            return parent / "log" / "pallet_servo_pallet_record.json"
    return Path.cwd() / "log" / "pallet_servo_pallet_record.json"


def main(argv: Optional[list] = None) -> int:
    ap = argparse.ArgumentParser(
        description="按手中箱子的横/竖方向，从托盘记录推算下一个码放槽位")
    ap.add_argument("--orientation", "-o", required=True,
                    help="箱子方向：横/horizontal/h 或 竖/vertical/v")
    ap.add_argument("--record", type=Path, default=None,
                    help="托盘记录文件；默认 <仓库根>/log/pallet_servo_pallet_record.json")
    ap.add_argument("--box-type", type=int, default=0,
                    help="箱型编号；不同箱型有独立的槽位表与已码放进度（默认 0）")
    ap.add_argument("--board", type=Path, default=None,
                    help="可选：board.json。给了就按该箱型的 box_types 槽位表"
                         "过滤顺序，避免算出该箱型不存在的位置")
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

    known = set(PLACE_ORDERS["horizontal"]) | set(PLACE_ORDERS["vertical"])
    unknown = sorted(placed - known)
    if unknown:
        print(f"警告：记录里有不属于横/竖槽位表的槽位 {unknown}，已忽略",
              file=sys.stderr)

    order = _order_for(direction, allowed)
    remaining = remaining_slots(direction, placed, allowed)
    try:
        slot = next_slot(direction, placed, allowed)
    except SlotFullError as exc:
        if args.json:
            print(json.dumps({
                "ok": False,
                "orientation": direction,
                "box_type": int(args.box_type),
                "error": str(exc),
                "placed_slots": sorted(placed),
                "order": list(order),
                "remaining": [],
            }, ensure_ascii=False))
        else:
            print(f"错误：{exc}", file=sys.stderr)
        return 1

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
        # 机器可读：stdout 只有一个整数；解释信息走 stderr。
        print(slot)
        label = "横" if direction == "horizontal" else "竖"
        print(f"箱型 {int(args.box_type)}：已码放 {sorted(placed) or '无'}；"
              f"{label}向槽位顺序 {list(order)}；下一槽位 {slot}",
              file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
