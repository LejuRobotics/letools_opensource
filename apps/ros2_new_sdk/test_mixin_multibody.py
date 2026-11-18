#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""多体定时 / 离线轨迹测试（需 --execute）。

覆盖接口：
  /control/wheel_mpc/set_multi_body_timed_target      （躯干 + 双臂关节）
  /control/wheel_mpc/set_multi_body_offline_traj      （--offline）
  /control/wheel_mpc/enable_multi_body_offline_traj   （--offline）

planner_index（定时多体）：2 躯干、3 下肢、4/5 双臂末端世界系、6/7 本体系、8/9 双臂关节。
离线轨迹的 planner_index 是另一套：2 躯干 4 维、3 左臂关节 7 维（默认按厂商用例走 3）。

用法：
  /usr/bin/python3 apps/ros2_new_sdk/test_mixin_multibody.py --execute
  /usr/bin/python3 apps/ros2_new_sdk/test_mixin_multibody.py --execute --offline
  /usr/bin/python3 apps/ros2_new_sdk/test_mixin_multibody.py --execute --offline --offline-only

离线轨迹的 desire_time 以“使能时刻”为 0，厂商服务端要求第一帧必须为 0、后续严格递增。
"""
from __future__ import annotations

import argparse
import math
import sys

from mixin_test_common import (
    Ros2WheeledHardware, record, show, skip, report,
    set_log_dir, start_segment, prepare_modes,
)


def main():
    parser = argparse.ArgumentParser(description="多体定时/离线轨迹测试")
    parser.add_argument("--execute", action="store_true", help="真正下发动作")
    parser.add_argument("--log-dir", default="")
    parser.add_argument("--z-delta", type=float, default=0.04)
    parser.add_argument("--arm-joint", type=int, default=2, choices=range(7))
    parser.add_argument("--arm-delta-deg", type=float, default=15.0)
    parser.add_argument("--offline", action="store_true", help="附带离线轨迹测试")
    parser.add_argument("--offline-only", action="store_true",
                        help="跳过前面的多体定时，只跑离线轨迹（需同时加 --offline）")
    args = parser.parse_args()

    if args.offline_only and not args.offline:
        print("--offline-only 需要同时加 --offline")
        return 0

    set_log_dir(args.log_dir or None)
    start_segment("S6 multibody")

    if not args.execute:
        print("这是动作用例，需要 --execute。只读检查请跑 test_mixin_readonly.py")
        return 0

    with Ros2WheeledHardware(node_name="mixin_multibody_test") as hw:
        show("wait_arm_ready", hw.wait_arm_ready(10.0))
        init = hw.read_torso_init_pose_deg()
        # init = list(0.20, 1.10,0,0)
        if not show("get_torso_init_pose", init, str(init.data)):
            return report("S6 multibody")
        if not prepare_modes(hw, quick=0):
            return report("S6 multibody")

        pose0 = list(init.data)
        base_arm = hw.read_arm_deg().data
        base_torso = hw.read_torso_deg().data
        left, right = list(base_arm[:7]), list(base_arm[7:])
        z_timed = 0.0 if args.offline_only else args.z_delta
        arm_timed = 0.0 if args.offline_only else args.arm_delta_deg
        left[args.arm_joint] += arm_timed
        right[args.arm_joint] += arm_timed
        # target_torso = [pose0[0], pose0[1] + z_timed, pose0[2], pose0[3]]
        target_torso = [pose0[0]+0.1, pose0[1] + z_timed, 0.0, 0.0]   

        commands = [
            {"planner_index": 2, "desire_time": 2.0, "cmd_vec": target_torso},
            # {"planner_index": 8, "desire_time": 2.0, "cmd_vec": left},
            # {"planner_index": 9, "desire_time": 2.0, "cmd_vec": right},
        ]
        r = hw.send_multi_body_timed(commands, is_sync=True)
        if show("set_multi_body_timed_target", r, str(r.data)):
            hw.spin_for(3.0)
            arm = hw.read_arm_deg().data
            torso = hw.read_torso_deg().data
            arm_delta = abs(arm[args.arm_joint] - base_arm[args.arm_joint])
            torso_delta = max(abs(a - b) for a, b in zip(torso, base_torso))
            if args.offline_only:
                skip("multibody_arm_track", "--offline-only：定时段不动作")
                skip("multibody_torso_track", "--offline-only：定时段不动作")
            else:
                record("multibody_arm_track", "PASS" if arm_delta > 0.3 else "FAIL",
                       f"left J{args.arm_joint} delta={arm_delta:.3f} deg")
                record("multibody_torso_track", "PASS" if torso_delta > 0.3 else "FAIL",
                       f"max torso delta={torso_delta:.3f} deg")

            restore = [
                {"planner_index": 2, "desire_time": 2.0, "cmd_vec": pose0},
                {"planner_index": 8, "desire_time": 2.0, "cmd_vec": base_arm[:7]},
                {"planner_index": 9, "desire_time": 2.0, "cmd_vec": base_arm[7:]},
            ]
            rr = hw.send_multi_body_timed(restore, is_sync=True)
            show("multibody_restore", rr, str(rr.data))
            hw.spin_for(3.0)

        if args.offline:
            # 与厂商 aria_wheel_offline_trajectory 对齐：左臂 7 关节，J3 -0.08 rad，0/2/4 s
            base_left = list(hw.read_arm_deg().data[:7])
            end_left = list(base_left)
            end_left[3] -= math.degrees(0.08)
            hw.enable_multi_body_offline_traj(False)
            traj = [{
                "planner_index": 3,
                "frame": 0,
                "points": [
                    {"desire_time": 0.0, "cmd_vec": base_left},
                    {"desire_time": 2.0, "cmd_vec": end_left},
                    {"desire_time": 4.0, "cmd_vec": base_left},
                ],
            }]
            r = hw.set_multi_body_offline_traj(traj)
            if show("set_multi_body_offline_traj", r):
                show("enable_offline(true)", hw.enable_multi_body_offline_traj(True))
                hw.spin_for(5.0)
                show("enable_offline(false)", hw.enable_multi_body_offline_traj(False))
                cur = hw.read_arm_deg().data[:7]
                delta = max(abs(a - b) for a, b in zip(cur, base_left))
                record("offline_return", "PASS" if delta < 3.44 else "FAIL",
                       f"left arm max delta={delta:.3f} deg")
        else:
            skip("set_multi_body_offline_traj", "需要 --offline")
            skip("enable_multi_body_offline_traj", "需要 --offline")

    return report("S6 multibody")


if __name__ == "__main__":
    sys.exit(main())
