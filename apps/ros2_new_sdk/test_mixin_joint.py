#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""关节类动作测试（需 --execute）。覆盖文件：mixin_test_common.py + 本文件。

覆盖接口：
  /control/arm/set_left_joints_timed_target
  /control/arm/set_right_joints_timed_target
  /control/arm/cmd_joints                     （50Hz 流式）
  /control/arm/cmd_arm_joints_traj            （三点轨迹）
  /control/arm/state_*_arm_joints_reach_time  观察是否出帧（无 -> 用 actual_time）

用法：
  /usr/bin/python3 apps/ros2_new_sdk/test_mixin_joint.py --execute
"""
from __future__ import annotations

import argparse
import sys

from mixin_test_common import (
    Ros2WheeledHardware, record, show, skip, substitute, report,
    set_log_dir, start_segment, prepare_modes, restore_joint, stream_joint,
)


def main():
    parser = argparse.ArgumentParser(description="关节类动作测试")
    parser.add_argument("--execute", action="store_true", help="真正下发动作")
    parser.add_argument("--delta-deg", type=float, default=5.0)
    parser.add_argument("--log-dir", default="",
                        help="结果落盘目录；默认 ~/mixin_test_logs/<日期>/")
    args = parser.parse_args()

    set_log_dir(args.log_dir or None)
    start_segment("S1 joint")
    if not args.execute:
        print("这是动作用例，需要 --execute。只读检查请跑 test_mixin_readonly.py")
        return 0

    with Ros2WheeledHardware(node_name="mixin_joint_test") as hw:
        show("wait_arm_ready", hw.wait_arm_ready(10.0))
        if not prepare_modes(hw, quick=0):
            return report("S1 joint")

        base = hw.read_arm_deg().data
        left, right = list(base[:7]), list(base[7:])

        # 左臂定时目标
        tgt = list(left); tgt[3] += args.delta_deg
        r = hw.send_arm_joint_timed("left", tgt, 2.0)
        if show("set_left_joints_timed_target", r, str(r.data)):
            hw.spin_for(3.0)
            cur = hw.read_arm_deg().data
            record("track_left_joint_timed",
                   "PASS" if abs(cur[3] - tgt[3]) < 0.15 else "FAIL",
                   f"J3={cur[3]:.3f} target={tgt[3]:.3f}")
            substitute("state_left_arm_joints_reach_time",
                       f"actual_time={float(r.data['actual_time']):.2f}s")
            restore_joint(hw, "left", left)
            hw.spin_for(2.5)

        # 右臂定时目标（14 维里右臂 J3 = 下标 10）
        tgt = list(right); tgt[3] += args.delta_deg
        r = hw.send_arm_joint_timed("right", tgt, 2.0)
        if show("set_right_joints_timed_target", r, str(r.data)):
            hw.spin_for(3.0)
            cur = hw.read_arm_deg().data
            record("track_right_joint_timed",
                   "PASS" if abs(cur[10] - tgt[3]) < 0.15 else "FAIL",
                   f"J3={cur[10]:.3f} target={tgt[3]:.3f}")
            substitute("state_right_arm_joints_reach_time",
                       f"actual_time={float(r.data['actual_time']):.2f}s")
            restore_joint(hw, "right", right)
            hw.spin_for(2.5)

        # cmd_joints 流式
        tgt = list(left); tgt[3] += args.delta_deg
        stream_joint(hw, tgt + right, 1.5)
        hw.spin_for(1.0)
        cur = hw.read_arm_deg().data
        record("cmd_joints", "PASS" if abs(cur[3] - tgt[3]) < 0.15 else "FAIL",
               f"J3={cur[3]:.3f}")
        stream_joint(hw, base, 1.5)
        hw.spin_for(1.0)

        # cmd_arm_joints_traj 三点轨迹
        tgt_full = list(base); tgt_full[3] += args.delta_deg
        r = hw.send_arm_joint_traj([0.0, 1.5, 3.0], [base, tgt_full, base])
        show("cmd_arm_joints_traj", r)
        hw.spin_for(4.5)
        cur = hw.read_arm_deg().data
        record("traj_return", "PASS" if abs(cur[3] - base[3]) < 0.15 else "FAIL",
               f"J3={cur[3]:.3f} base={base[3]:.3f}")

    return report("S1 joint")


if __name__ == "__main__":
    sys.exit(main())
