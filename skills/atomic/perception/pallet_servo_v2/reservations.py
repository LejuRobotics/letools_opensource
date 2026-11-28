"""Vision-owned candidate slots in a V2 pallet observation record."""
from copy import deepcopy
from typing import Iterable, Mapping, Optional, Tuple

from .next_slot import _occupied_from_record, next_slot


def reserve(record: dict, box_type: int, orientation: str, owner: str,
            allowed: Optional[Iterable[int]],
            orders: Optional[Mapping[str, Iterable[int]]] = None) -> Tuple[dict, int]:
    if not isinstance(record, dict):
        raise ValueError("未取得有效的目标托盘观测")
    updated = deepcopy(record)
    reserved = updated.setdefault("reserved_by_box_type", {}).setdefault(str(box_type), {})
    occupied = _occupied_from_record(record, box_type)
    for key, existing in reserved.items():
        if existing == owner:
            return updated, int(key)
    slot = next_slot(orientation, occupied.union(map(int, reserved)), allowed, orders=orders)
    reserved[str(slot)] = owner
    return updated, slot


def settle(record: dict, box_type: int, slot: int, owner: str, outcome: str) -> dict:
    if outcome not in ("placed", "dropped"):
        raise ValueError("未知落位结果")
    updated = deepcopy(record)
    key = str(slot)
    reserved = updated.get("reserved_by_box_type", {}).get(str(box_type), {})
    if reserved.get(key) != owner:
        raise ValueError("本箱与当前托盘保留记录不匹配")
    del reserved[key]
    if outcome == "placed":
        previously_placed = _occupied_from_record(updated, box_type)
        by_type = updated.setdefault("placements_by_box_type", {})
        # Migrate older flat type-0 records when a second bucket is first created.
        if "placements_by_box_type" not in record:
            by_type["0"] = {"placed_slots": sorted(_occupied_from_record(record, 0)),
                            "slots": deepcopy(record.get("placements", {}))}
        bucket = by_type.setdefault(str(box_type), {})
        bucket.setdefault("slots", {})[key] = {"slot": slot, "owner": owner}
        bucket["placed_slots"] = sorted(previously_placed | {slot})
        bucket["placed_count"] = len(bucket["placed_slots"])
    return updated
