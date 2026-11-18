# -*- coding: utf-8 -*-
"""托盘伺服 SlotPlan：把「箱型 + 槽位 + 初/终阶段」解析成一份生效参数。

服务只传三个数：`slot` / `box_type` / `stage`。节点在 call 服务时调用一次
`resolve_slot_plan`，之后运行路径只读这一份 `SlotPlan`，不再自己拼
`ref_edges`、偏置、箱高和法向旋转。

配置两种来源：

* `box_types`（新）：每个箱型一张槽位表，槽位里可以写 `initial` / `final`
  两个阶段的覆盖；
* 旧的 `ref_edges` + `servo_slots`：等价于「箱型 0、只有 final 阶段」的一张表，
  由 `legacy_box_type` 转换。节点里两种来源互斥，不会同时生效。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

from skills.atomic.perception.pallet_servo_v2.algorithm import parse_ref_edges

#: 阶段常量。与 `SetServoSlot.srv` 里的常量**必须逐字一致**。
STAGE_FINAL = 0
STAGE_INITIAL = 1
_STAGE_NAMES = {STAGE_FINAL: "final", STAGE_INITIAL: "initial"}

#: 槽位/箱型级允许的字段。
_SLOT_KEYS = {
    "slot", "mode", "side", "ref_edges", "box_plane_height_mm",
    "e_right_bias_px", "e_bottom_bias_px", "normal_rot_deg",
    "initial", "final",
}
#: 阶段覆盖只允许改「随阶段变化」的量；路线（mode/side）与法向旋转是槽位属性。
_STAGE_KEYS = {
    "ref_edges", "box_plane_height_mm",
    "e_right_bias_px", "e_bottom_bias_px",
}
_BIAS_KEYS = ("e_right_bias_px", "e_bottom_bias_px")
_SERVO_MODES = ("pallet", "neighbor")
_NEIGHBOR_SIDES = ("left", "right", "none")


@dataclass(frozen=True)
class StageOverride:
    """一个阶段对槽位配置的覆盖；未写的字段为 `None`（回落槽位/箱型）。"""

    ref_edges: Optional[Tuple[str, str]] = None
    box_plane_height_mm: Optional[float] = None
    e_right_bias_px: Optional[float] = None
    e_bottom_bias_px: Optional[float] = None


@dataclass(frozen=True)
class SlotConfig:
    """一个箱型里的一个槽位。"""

    slot: int
    mode: str = "pallet"
    side: str = "none"
    ref_edges: Optional[Tuple[str, str]] = None
    initial: Optional[StageOverride] = None
    final: Optional[StageOverride] = None
    box_plane_height_mm: Optional[float] = None
    e_right_bias_px: Optional[float] = None
    e_bottom_bias_px: Optional[float] = None
    normal_rot_deg: Tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class BoxTypeProfile:
    """一个箱型的完整槽位表。"""

    box_type: int
    name: str
    slots: Tuple[SlotConfig, ...]
    box_plane_height_mm: Optional[float] = None
    e_right_bias_px: float = 0.0
    e_bottom_bias_px: float = 0.0


@dataclass(frozen=True)
class SlotPlan:
    """一次 call 服务最终生效的全部参数。"""

    box_type: int
    box_type_name: str
    slot: int
    stage: int
    mode: str
    side: str
    ref_edges: Optional[Tuple[str, str]]
    box_plane_height_mm: float
    e_right_bias_px: float
    e_bottom_bias_px: float
    normal_rot_deg: Tuple[float, float, float]

    @property
    def stage_name(self) -> str:
        return _STAGE_NAMES[self.stage]


def stage_name(stage: int) -> str:
    """`0/1` → `final/initial`；未知值原样转成字符串，只给日志用。"""
    return _STAGE_NAMES.get(stage, str(stage))


def _finite_float(value, what: str, *, minimum: Optional[float] = None):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None, f"{what}={value!r} 不是数值"
    if not math.isfinite(number):
        return None, f"{what}={value!r} 不是有限数"
    if minimum is not None and number < minimum:
        return None, f"{what}={value!r} 不能小于 {minimum:g}"
    return number, None


def _parse_ref_edges(value, what: str):
    if value is None:
        return None, None
    parsed = parse_ref_edges(value)
    if isinstance(parsed, tuple):
        return parsed, None
    return None, f"{what} 非法：{parsed}"


def _parse_stage(raw, what: str):
    if raw is None:
        return None, None
    if not isinstance(raw, dict):
        return None, f"{what} 必须是对象"
    unknown = sorted(set(raw) - _STAGE_KEYS)
    if unknown:
        return None, (
            f"{what} 不允许 {unknown} —— 阶段覆盖只认 "
            f"{sorted(_STAGE_KEYS)}（mode/side/normal_rot_deg 是槽位属性，"
            f"不随初/终阶段变化）")
    ref_edges, err = _parse_ref_edges(raw.get("ref_edges"),
                                      f"{what}.ref_edges")
    if err:
        return None, err
    height = None
    if "box_plane_height_mm" in raw:
        height, err = _finite_float(raw["box_plane_height_mm"],
                                    f"{what}.box_plane_height_mm", minimum=0.0)
        if err:
            return None, err
    biases = {}
    for key in _BIAS_KEYS:
        if key in raw:
            value, err = _finite_float(raw[key], f"{what}.{key}")
            if err:
                return None, err
            biases[key] = value
    return StageOverride(
        ref_edges=ref_edges,
        box_plane_height_mm=height,
        e_right_bias_px=biases.get("e_right_bias_px"),
        e_bottom_bias_px=biases.get("e_bottom_bias_px"),
    ), None


def _parse_slot(raw, *, profile_index: int):
    if not isinstance(raw, dict):
        return None, f"箱型第 {profile_index} 项的 slots 里有非对象项：{raw!r}"
    unknown = sorted(set(raw) - _SLOT_KEYS)
    if unknown:
        return None, f"槽位配置里有不认识的键 {unknown}（只认 {sorted(_SLOT_KEYS)}）"
    slot_raw = raw.get("slot")
    try:
        slot = int(slot_raw)
    except (TypeError, ValueError):
        return None, f"槽位的 slot={slot_raw!r} 不是整数"
    if slot < 1:
        return None, f"槽位的 slot={slot} 必须从 1 开始"

    mode = str(raw.get("mode", "pallet")).strip()
    if mode not in _SERVO_MODES:
        return None, f"槽位 {slot} 的 mode={raw.get('mode')!r} 不认识（只认 pallet/neighbor）"
    side = raw.get("side", "none")
    side = "none" if side is None else str(side).strip()
    if side not in _NEIGHBOR_SIDES:
        return None, (f"槽位 {slot} 的 side={raw.get('side')!r} 不认识"
                      f"（只认 left/right/none，画面口径）")

    ref_edges, err = _parse_ref_edges(raw.get("ref_edges"),
                                      f"槽位 {slot} 的 ref_edges")
    if err:
        return None, err
    height = None
    if "box_plane_height_mm" in raw:
        height, err = _finite_float(raw["box_plane_height_mm"],
                                    f"槽位 {slot} 的 box_plane_height_mm",
                                    minimum=0.0)
        if err:
            return None, err
    biases = {}
    for key in _BIAS_KEYS:
        if key in raw:
            value, err = _finite_float(raw[key], f"槽位 {slot} 的 {key}")
            if err:
                return None, err
            biases[key] = value

    raw_rot = raw.get("normal_rot_deg", [0.0, 0.0, 0.0])
    if not isinstance(raw_rot, (list, tuple)):
        return None, f"槽位 {slot} 的 normal_rot_deg={raw_rot!r} 必须是三个数"
    try:
        rot = tuple(float(v) for v in raw_rot)
    except (TypeError, ValueError):
        return None, f"槽位 {slot} 的 normal_rot_deg={raw_rot!r} 不是数值列表"
    if len(rot) != 3 or not all(math.isfinite(v) for v in rot):
        return None, f"槽位 {slot} 的 normal_rot_deg={raw_rot!r} 必须是三个有限数"

    initial, err = _parse_stage(raw.get("initial"), f"槽位 {slot} 的 initial")
    if err:
        return None, err
    final, err = _parse_stage(raw.get("final"), f"槽位 {slot} 的 final")
    if err:
        return None, err

    slot_config = SlotConfig(
        slot=slot,
        mode=mode,
        side=side,
        ref_edges=ref_edges,
        initial=initial,
        final=final,
        box_plane_height_mm=height,
        e_right_bias_px=biases.get("e_right_bias_px"),
        e_bottom_bias_px=biases.get("e_bottom_bias_px"),
        normal_rot_deg=rot,
    )

    if mode == "pallet":
        for label, override in (("initial", initial), ("final", final)):
            resolved = (override.ref_edges if override is not None else None) \
                or ref_edges
            if resolved is None:
                return None, (f"槽位 {slot} 是 pallet 档，但 {label} 阶段没有 "
                              f"ref_edges —— 请写槽位级 ref_edges，或两个阶段各写一份")
    return slot_config, None


def parse_box_types(raw) -> Tuple[Tuple[BoxTypeProfile, ...], Optional[str]]:
    """解析 `box_types`；返回 `(profiles, error)`，出错时 profiles 为空。"""
    if not isinstance(raw, (list, tuple)) or not raw:
        return (), "box_types 必须是非空数组"
    profiles = []
    seen_types = set()
    for index, item in enumerate(raw, start=1):
        if not isinstance(item, dict):
            return (), f"box_types 第 {index} 项不是对象"
        unknown = sorted(set(item) - {
            "box_type", "name", "slots", "box_plane_height_mm",
            "e_right_bias_px", "e_bottom_bias_px",
        })
        if unknown:
            return (), f"box_types 第 {index} 项有不认识的键 {unknown}"
        try:
            box_type = int(item.get("box_type"))
        except (TypeError, ValueError):
            return (), f"box_types 第 {index} 项的 box_type={item.get('box_type')!r} 不是整数"
        if box_type < 0:
            return (), f"box_types 的 box_type={box_type} 不能为负"
        if box_type in seen_types:
            return (), f"box_types 的 box_type={box_type} 重复"
        seen_types.add(box_type)

        name = str(item.get("name", "") or f"箱型{box_type}")
        height = None
        if "box_plane_height_mm" in item:
            height, err = _finite_float(item["box_plane_height_mm"],
                                        f"箱型 {box_type} 的 box_plane_height_mm",
                                        minimum=0.0)
            if err:
                return (), err
        biases = {}
        for key in _BIAS_KEYS:
            value = 0.0
            if key in item:
                value, err = _finite_float(item[key], f"箱型 {box_type} 的 {key}")
                if err:
                    return (), err
            biases[key] = value

        raw_slots = item.get("slots")
        if not isinstance(raw_slots, (list, tuple)) or not raw_slots:
            return (), f"箱型 {box_type} 的 slots 必须是非空数组"
        slots = []
        for slot_raw in raw_slots:
            slot_config, err = _parse_slot(slot_raw, profile_index=index)
            if err:
                return (), f"箱型 {box_type}：{err}"
            slots.append(slot_config)
        slots.sort(key=lambda s: s.slot)
        actual = [s.slot for s in slots]
        expected = list(range(1, len(slots) + 1))
        if actual != expected:
            return (), (f"箱型 {box_type} 的槽位必须从 1 连续编号（1..{len(slots)}），"
                        f"实际 {actual}")
        profiles.append(BoxTypeProfile(
            box_type=box_type,
            name=name,
            slots=tuple(slots),
            box_plane_height_mm=height,
            e_right_bias_px=biases["e_right_bias_px"],
            e_bottom_bias_px=biases["e_bottom_bias_px"],
        ))
    return tuple(profiles), None


def legacy_box_type(ref_edge_slots: Sequence[Sequence[str]],
                    slot_modes: Sequence[Sequence],
                    *, name: str = "默认箱型") -> BoxTypeProfile:
    """旧 `ref_edges` + `servo_slots` → 箱型 0 的槽位表（只有 final 阶段）。

    `slot_modes` 的项沿用节点里 `_normalize_servo_slots` 的元组形状：
    `(mode, side, e_right_bias_px, e_bottom_bias_px, height, normal_rot_deg)`；
    二元组 `(mode, side)` 也兼容。
    """
    total = max(len(ref_edge_slots), len(slot_modes))
    slots = []
    for index in range(total):
        mode, side = "pallet", "none"
        right_bias = bottom_bias = 0.0
        height = None
        rot = (0.0, 0.0, 0.0)
        if index < len(slot_modes):
            row = slot_modes[index]
            mode = row[0]
            side = row[1] if len(row) > 1 else "none"
            if len(row) > 2:
                right_bias = float(row[2])
            if len(row) > 3:
                bottom_bias = float(row[3])
            if len(row) > 4 and row[4] is not None:
                height = float(row[4])
            if len(row) > 5 and row[5] is not None:
                rot = tuple(float(v) for v in row[5])
        ref_edges = None
        if index < len(ref_edge_slots):
            ref_edges = tuple(str(edge) for edge in ref_edge_slots[index])
        slots.append(SlotConfig(
            slot=index + 1,
            mode=mode,
            side=side,
            ref_edges=ref_edges,
            box_plane_height_mm=height,
            e_right_bias_px=right_bias,
            e_bottom_bias_px=bottom_bias,
            normal_rot_deg=rot,
        ))
    return BoxTypeProfile(box_type=0, name=name, slots=tuple(slots))


def _stage_of(slot: SlotConfig, stage: int) -> Optional[StageOverride]:
    return slot.initial if stage == STAGE_INITIAL else slot.final


def resolve_slot_plan(profiles: Sequence[BoxTypeProfile], *,
                      box_type: int, slot: int, stage: int,
                      global_height_mm: float) -> Tuple[Optional[SlotPlan], Optional[str]]:
    """按 `(box_type, slot, stage)` 解析出生效参数；出错返回 `(None, 原因)`。"""
    if stage not in _STAGE_NAMES:
        return None, (f"stage={stage!r} 不认识 —— 只认 {STAGE_FINAL}"
                      f"（最终伺服）/ {STAGE_INITIAL}（初伺服）")
    profile = next((p for p in profiles if p.box_type == box_type), None)
    if profile is None:
        known = [p.box_type for p in profiles]
        return None, f"箱型 {box_type} 不存在（已配置 {known}）"
    slot_config = next((s for s in profile.slots if s.slot == slot), None)
    if slot_config is None:
        count = len(profile.slots)
        return None, (f"箱型 {box_type} 的槽位 {slot} 不存在（本箱型有 1..{count} 档）")

    override = _stage_of(slot_config, stage)
    ref_edges = (override.ref_edges if override is not None else None) \
        or slot_config.ref_edges
    if slot_config.mode == "pallet" and ref_edges is None:
        return None, (f"箱型 {box_type} 槽位 {slot} 的 {stage_name(stage)} 阶段"
                      f"没有 ref_edges —— pallet 路线必须有参考边")
    height = (override.box_plane_height_mm if override is not None else None)
    if height is None:
        height = slot_config.box_plane_height_mm
    if height is None:
        height = profile.box_plane_height_mm
    if height is None:
        height = float(global_height_mm)
    right_bias = (override.e_right_bias_px if override is not None else None)
    if right_bias is None:
        right_bias = slot_config.e_right_bias_px
    if right_bias is None:
        right_bias = profile.e_right_bias_px
    bottom_bias = (override.e_bottom_bias_px if override is not None else None)
    if bottom_bias is None:
        bottom_bias = slot_config.e_bottom_bias_px
    if bottom_bias is None:
        bottom_bias = profile.e_bottom_bias_px

    return SlotPlan(
        box_type=profile.box_type,
        box_type_name=profile.name,
        slot=slot,
        stage=stage,
        mode=slot_config.mode,
        side=slot_config.side,
        ref_edges=ref_edges,
        box_plane_height_mm=float(height),
        e_right_bias_px=float(right_bias),
        e_bottom_bias_px=float(bottom_bias),
        normal_rot_deg=slot_config.normal_rot_deg,
    ), None
