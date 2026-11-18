#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""头部接口完整验证（经 Ros2WheeledHardware，默认 dry-run）。

覆盖我们这层的全部入口：
  HeadMixin: wait_head_ready / read_head_limits_deg / read_head_current_deg /
             read_head_measured_deg / head_frames / head_measured_frames /
             wait_head_frame / send_head_target
  IHardwareCompatMixin: control_head(yaw_deg, pitch_deg)

动作：yaw/pitch 各走 +、- 两个方向，最后回中；每次等 current_head_cmd 新帧
再等 3 s 看 lowstate 实测。pitch 判定按幅值（关节符号约定由实测记录）。

用法：
  /usr/bin/python3 apps/ros2_new_sdk/test_mixin_head.py
  /usr/bin/python3 apps/ros2_new_sdk/test_mixin_head.py --execute
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from mixin_test_common import (  # noqa: E402
    Ros2WheeledHardware, record, report, set_log_dir, show, skip, start_segment,
)


def move(hw, label, yaw, pitch, settle, via_compat=False):
    before = hw.head_frames
    if via_compat:
        sent = hw.control_head(yaw, pitch)
    else:
        sent = hw.send_head_target(yaw, pitch)
    if not show(label, sent):
        return

    frame = hw.wait_head_frame(before, 3.0)
    show(f"{label}.wait_frame", frame)
    if frame.success:
        got = frame.data
        record(f"{label}.target_readback",
               "PASS" if abs(got[0] - yaw) < 0.5
               and abs(abs(got[1]) - abs(pitch)) < 0.5 else "FAIL",
               f"yaw={got[0]:.2f} pitch={got[1]:.2f}")

    hw.spin_for(settle)
    measured = hw.read_head_measured_deg()
    if measured.success:
        yaw_err = abs(measured.data[0] - yaw)
        pitch_err = abs(abs(measured.data[1]) - abs(pitch))
        record(f"{label}.track",
               "PASS" if yaw_err < 2.0 and pitch_err < 2.0 else "FAIL",
               f"measured yaw={measured.data[0]:.2f} pitch={measured.data[1]:.2f} "
               f"(target {yaw:+.1f}/{pitch:+.1f})")


def main():
    parser = argparse.ArgumentParser(description="头部接口完整验证")
    parser.add_argument("--execute", action="store_true", help="真正下发头部动作")
    parser.add_argument("--yaw-deg", type=float, default=12.0)
    parser.add_argument("--pitch-deg", type=float, default=8.0)
    parser.add_argument("--settle-sec", type=float, default=3.0)
    parser.add_argument("--log-dir", default="")
    args = parser.parse_args()

    set_log_dir(args.log_dir or None)
    start_segment("S4 head")

    with Ros2WheeledHardware(node_name="mixin_head_test") as hw:
        show("wait_head_ready", hw.wait_head_ready(10.0))

        limits = None
        limits_res = hw.read_head_limits_deg()
        if show("read_head_limits_deg", limits_res):
            limits = limits_res.data
            low, high = limits
            record("limits_range", "PASS",
                   f"yaw [{low[0]:.2f}, {high[0]:.2f}] "
                   f"pitch [{low[1]:.2f}, {high[1]:.2f}]")

        show("read_head_current_deg", hw.read_head_current_deg())
        show("read_head_measured_deg", hw.read_head_measured_deg())

        frames0 = hw.head_frames
        measured0 = hw.head_measured_frames
        record("frames_baseline", "PASS",
               f"cmd_frames={frames0} measured_frames={measured0}")

        yaw = float(args.yaw_deg)
        pitch = float(args.pitch_deg)
        if limits:
            low, high = limits
            yaw = max(low[0] + 5.0, min(high[0] - 5.0, yaw))
            pitch = max(low[1] + 5.0, min(high[1] - 5.0, pitch))
            record("test_angles_clamped", "PASS",
                   f"yaw={yaw:+.1f} pitch={pitch:+.1f}")

        if not args.execute:
            for name in ("control_head", "send_head_target(-)", "send_head_target(0)",
                         "frames_increment", "measured_frames_increment"):
                skip(name, "dry-run；加 --execute")
            return report("S4 head")

        move(hw, "control_head(+yaw,+pitch)", yaw, pitch, args.settle_sec,
             via_compat=True)
        move(hw, "send_head_target(-yaw,-pitch)", -yaw, -pitch, args.settle_sec)
        move(hw, "send_head_target(0,0)", 0.0, 0.0, args.settle_sec)

        record("frames_increment",
               "PASS" if hw.head_frames - frames0 >= 3 else "FAIL",
               f"{hw.head_frames - frames0} new cmd frames")
        record("measured_frames_increment",
               "PASS" if hw.head_measured_frames - measured0 > 0 else "FAIL",
               f"{hw.head_measured_frames - measured0} new measured frames")

    return report("S4 head")


if __name__ == "__main__":
    raise SystemExit(main())
