#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""末端类动作测试（需 --execute）。覆盖文件：mixin_test_common.py + 本文件。

覆盖接口：
  /control/arm/set_left_ee_pose_timed_target
  /control/arm/set_right_ee_pose_timed_target
  /control/arm/cmd_ee_poses            （30Hz 流式）
  /control/arm/cmd_ee_poses_traj       （三点轨迹）
  /control/arm/state_*_arm_pose_reach_time  观察是否出帧（无 -> 用 actual_time）

用法：
  /usr/bin/python3 apps/ros2_new_sdk/test_mixin_ee.py --execute
"""
from __future__ import annotations

import argparse
import sys

from mixin_test_common import (
    Ros2WheeledHardware, record, show, skip, substitute, report,
    set_log_dir, start_segment, prepare_modes, restore_ee, read_pose, stream_ee, quat_to_zyx_deg,
)


def main():
    parser = argparse.ArgumentParser(description="末端类动作测试")
    parser.add_argument("--execute", action="store_true", help="真正下发动作")
    parser.add_argument("--delta-m", type=float, default=0.03)
    parser.add_argument("--log-dir", default="",
                        help="结果落盘目录；默认 ~/mixin_test_logs/<日期>/")
    args = parser.parse_args()

    set_log_dir(args.log_dir or None)
    start_segment("S2 ee")
    if not args.execute:
        print("这是动作用例，需要 --execute。只读检查请跑 test_mixin_readonly.py")
        return 0

    with Ros2WheeledHardware(node_name="mixin_ee_test") as hw:
        show("wait_arm_ready", hw.wait_arm_ready(10.0))
        if not prepare_modes(hw, quick=0):
            return report("S2 ee")

        # 左臂定时末端
        pos, zyx, err = read_pose(hw, "left")
        if err is None:
            tgt = [pos[0] + args.delta_m, pos[1], pos[2]] + zyx
            r = hw.send_arm_ee_timed("left", tgt, 2.0, 1)
            if show("set_left_ee_pose_timed_target", r, str(r.data)):
                hw.spin_for(3.0)
                pos2, _, _ = read_pose(hw, "left")
                record("track_left_ee_timed",
                       "PASS" if pos2 and abs(pos2[0] - tgt[0]) < 0.02 else "FAIL",
                       f"dx={pos2[0]-pos[0]:.4f}" if pos2 else "")
                substitute("state_left_arm_pose_reach_time",
                           f"actual_time={float(r.data['actual_time']):.2f}s")
                restore_ee(hw, "left", pos + zyx)
                hw.spin_for(3.0)

        # 右臂定时末端
        pos, zyx, err = read_pose(hw, "right")
        if err is None:
            tgt = [pos[0] + args.delta_m, pos[1], pos[2]] + zyx
            r = hw.send_arm_ee_timed("right", tgt, 2.0, 1)
            if show("set_right_ee_pose_timed_target", r, str(r.data)):
                hw.spin_for(3.0)
                pos2, _, _ = read_pose(hw, "right")
                record("track_right_ee_timed",
                       "PASS" if pos2 and abs(pos2[0] - tgt[0]) < 0.02 else "FAIL",
                       f"dx={pos2[0]-pos[0]:.4f}" if pos2 else "")
                substitute("state_right_arm_pose_reach_time",
                           f"actual_time={float(r.data['actual_time']):.2f}s")
                restore_ee(hw, "right", pos + zyx)
                hw.spin_for(3.0)

        # cmd_ee_poses / cmd_ee_poses_traj 要求 7 维四元数位姿，不是 6 维 ZYX
        fk = hw.solve_arm_fk()
        if fk.success:
            left_pos = list(fk.data["left"]["pos"])
            left_quat = list(fk.data["left"]["quat"])
            left7 = left_pos + left_quat
            right7 = list(fk.data["right"]["pos"]) + list(fk.data["right"]["quat"])
            moved = [left_pos[0] + args.delta_m, left_pos[1], left_pos[2]] + left_quat

            stream_ee(hw, moved, right7, 1.5)
            hw.spin_for(1.0)
            pos2, _, _ = read_pose(hw, "left")
            record("cmd_ee_poses",
                   "PASS" if pos2 and abs(pos2[0] - moved[0]) < 0.02 else "FAIL",
                   f"dx={pos2[0]-left_pos[0]:.4f}" if pos2 else "")
            stream_ee(hw, left7, right7, 1.5)
            hw.spin_for(1.0)

            r = hw.send_ee_traj([0.0, 1.5, 3.0],
                                [left7, moved, left7],
                                [right7, right7, right7], frame=1)
            show("cmd_ee_poses_traj", r)
            hw.spin_for(4.5)
            pos3, _, _ = read_pose(hw, "left")
            record("traj_return",
                   "PASS" if pos3 and abs(pos3[0] - left_pos[0]) < 0.02 else "FAIL",
                   f"dx={pos3[0]-left_pos[0]:.4f}" if pos3 else "")

    return report("S2 ee")


if __name__ == "__main__":
    sys.exit(main())
