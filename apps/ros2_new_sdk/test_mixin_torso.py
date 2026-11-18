#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""躯干 / 下肢接口测试。默认只读；--execute 才动机器人。

覆盖接口：
  /control/torso/get_torso_init_pose
  /control/torso/cmd_torso_pose                 （50Hz 流式）
  /control/torso/cmd_leg_joints                 （50Hz 流式）
  /control/torso/set_torso_pose_timed_target
  /control/torso/set_leg_joints_timed_target
  /control/torso/reset_torso
  /control/torso/state_torso_pose_reach_time
  /control/torso/state_leg_reach_time

用法：
  /usr/bin/python3 apps/ros2_new_sdk/test_mixin_torso.py
  /usr/bin/python3 apps/ros2_new_sdk/test_mixin_torso.py --execute
"""
from __future__ import annotations

import argparse
import sys
import time

from mixin_test_common import (
    Ros2WheeledHardware, record, show, skip, substitute, report,
    set_log_dir, start_segment, prepare_modes,
)


def stream(hw, publish, values, seconds, rate=50):
    period = 1.0 / rate
    end = time.time() + seconds
    while time.time() < end:
        publish(values)
        hw.spin_once(0.0)
        time.sleep(period)


def reach(hw, name, kind, frames, fallback):
    result = hw.wait_torso_reach_time(kind, frames, 3.0, fallback_actual_time=fallback)
    if result.success:
        if "service response" in result.message:
            substitute(name, f"actual_time={float(result.data):.2f}s")
        else:
            show(name, result, f"{float(result.data):.2f}s")
    else:
        show(name, result)


def main():
    parser = argparse.ArgumentParser(description="躯干/下肢接口测试")
    parser.add_argument("--execute", action="store_true", help="真正下发动作")
    parser.add_argument("--log-dir", default="")
    parser.add_argument("--z-delta", type=float, default=0.02, help="躯干 z 增量（米）")
    parser.add_argument("--leg-delta-deg", type=float, default=1.0)
    args = parser.parse_args()

    set_log_dir(args.log_dir or None)
    start_segment("S5 torso")

    with Ros2WheeledHardware(node_name="mixin_torso_test") as hw:
        show("wait_arm_ready", hw.wait_arm_ready(10.0))
        show("read_torso_deg", hw.read_torso_deg())
        init = hw.read_torso_init_pose_deg()
        show("get_torso_init_pose", init, str(init.data))
        for kind in ("torso", "leg"):
            r = hw.torso_reach_topic_available(kind)
            if r.success:
                show(f"state_{kind}_reach_time(avail)", r)
            else:
                skip(f"state_{kind}_reach_time(avail)",
                     "无发布者；定时目标执行期间可能按需出现")

        if not args.execute:
            for name in ("set_torso_pose_timed_target", "cmd_torso_pose",
                         "reset_torso", "set_leg_joints_timed_target",
                         "cmd_leg_joints"):
                skip(name, "需要 --execute")
            return report("S5 torso")

        if not init.success:
            return report("S5 torso")
        if not prepare_modes(hw, quick=0):
            return report("S5 torso")

        base_joints = hw.read_torso_deg().data
        pose0 = list(init.data)

        # 1) 躯干定时目标
        target = [pose0[0], pose0[1] + args.z_delta, pose0[2], pose0[3]]
        f0 = hw.torso_reach_frames("torso")
        r = hw.send_torso_pose_timed(target, 2.0)
        if show("set_torso_pose_timed_target", r, str(r.data)):
            reach(hw, "state_torso_pose_reach_time", "torso", f0,
                  (r.data or {}).get("actual_time"))
            hw.spin_for(2.5)
            cur = hw.read_torso_deg().data
            delta = max(abs(a - b) for a, b in zip(cur, base_joints))
            record("torso_pose_track", "PASS" if delta > 0.3 else "FAIL",
                   f"max joint delta={delta:.3f} deg")
            show("reset_torso", hw.reset_torso())
            hw.spin_for(3.0)

        # 2) 下肢定时目标
        base_joints = hw.read_torso_deg().data
        tgt = list(base_joints)
        tgt[3] += args.leg_delta_deg
        f1 = hw.torso_reach_frames("leg")
        r = hw.send_leg_joints_timed(tgt, 2.0)
        if show("set_leg_joints_timed_target", r, str(r.data)):
            reach(hw, "state_leg_reach_time", "leg", f1,
                  (r.data or {}).get("actual_time"))
            hw.spin_for(2.5)
            cur = hw.read_torso_deg().data
            record("leg_joint_track",
                   "PASS" if abs(cur[3] - tgt[3]) < 0.4 else "FAIL",
                   f"J3={cur[3]:.3f} target={tgt[3]:.3f}")
            hw.send_leg_joints_timed(base_joints, 2.0)
            hw.spin_for(2.5)

        # 3) cmd_torso_pose 流式
        base_joints = hw.read_torso_deg().data
        stream(hw, hw.send_torso_pose_frame, target, 1.5)
        hw.spin_for(1.0)
        cur = hw.read_torso_deg().data
        delta = max(abs(a - b) for a, b in zip(cur, base_joints))
        record("cmd_torso_pose", "PASS" if delta > 0.3 else "FAIL",
               f"max joint delta={delta:.3f} deg")
        stream(hw, hw.send_torso_pose_frame, pose0, 1.5)
        hw.spin_for(1.0)

        # 4) cmd_leg_joints 流式
        base_joints = hw.read_torso_deg().data
        tgt = list(base_joints); tgt[3] += args.leg_delta_deg
        stream(hw, hw.send_leg_joints_frame, tgt, 1.5)
        hw.spin_for(1.0)
        cur = hw.read_torso_deg().data
        record("cmd_leg_joints",
               "PASS" if abs(cur[3] - tgt[3]) < 0.4 else "FAIL",
               f"J3={cur[3]:.3f}")
        stream(hw, hw.send_leg_joints_frame, base_joints, 1.5)
        hw.spin_for(1.0)

    return report("S5 torso")


if __name__ == "__main__":
    sys.exit(main())
