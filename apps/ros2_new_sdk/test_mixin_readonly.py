#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""只读接口测试（不产生运动）。覆盖文件：mixin_test_common.py + 本文件。

覆盖接口（全部经 Ros2WheeledHardware）：
  /lowstate                              14 维臂角、18 维模型角、头部实测角
  /control/controller/get_status
  /control/arm/get_control_mode
  /control/wheel_mpc/get_control_mode
  /control/wheel_mpc/get_arm_quick_mode
  /control/head/get_head_limits
  /control/head/current_head_cmd
  /planning/kinematics/solve_arm_fk
  /planning/kinematics/solve_ee_poses_ik
  /control/wheel_mpc/check_ee_accessibility
  /control/arm/get_ee_pose_reach_error     空闲 999 -> SKIP
  /control/wheel_mpc/state_arm_control_mode 仅变化时发 -> 可能 SKIP
  4 × state_*_reach_time                   无发布者 -> SKIP（actual_time 平替）

用法：
  /usr/bin/python3 apps/ros2_new_sdk/test_mixin_readonly.py
"""
from __future__ import annotations

import argparse
import sys

from mixin_test_common import (
    Ros2WheeledHardware, record, show, skip, substitute, report,
    quat_to_zyx_deg, dist, set_log_dir, start_segment,
)


def main():
    parser = argparse.ArgumentParser(description="只读接口测试")
    parser.add_argument("--log-dir", default="",
                        help="结果落盘目录；默认 ~/mixin_test_logs/<日期>/")
    args = parser.parse_args()
    set_log_dir(args.log_dir or None)
    start_segment("S0 readonly")

    with Ros2WheeledHardware(node_name="mixin_readonly_test") as hw:
        show("wait_arm_ready", hw.wait_arm_ready(10.0))
        show("wait_head_ready", hw.wait_head_ready(10.0))

        ctrl = hw.controller()
        if ctrl.success:
            record("get_status", "PASS",
                   f"{ctrl.data['current']!r} switching={ctrl.data['switching']}")
            if not ctrl.data["current"] or ctrl.data["switching"]:
                record("controller_running", "FAIL", "controller 未 RUNNING")
        else:
            show("get_status", ctrl)

        modes = hw.get_modes()
        if modes.success:
            record("get_modes", "PASS",
                   f"arm={modes.data['arm']} wheel_mpc={modes.data['wheel_mpc']} "
                   f"quick={modes.data['quick']}")
            record("get_arm_control_mode", "PASS", f"arm={modes.data['arm']}")
            # setter 只设为当前值，不改变状态，用来验证接口连通
            show("set_arm_control_mode(same)",
                 hw.set_arm_control_mode(modes.data["arm"]))
            show("set_wheel_control_mode(same)",
                 hw.set_wheel_control_mode(modes.data["wheel_mpc"]))
        else:
            show("get_modes", modes)

        show("read_arm_deg", hw.read_arm_deg())
        show("read_torso_deg", hw.read_torso_deg())
        show("read_model_deg", hw.read_model_deg())
        show("read_head_limits_deg", hw.read_head_limits_deg())
        show("read_head_current_deg", hw.read_head_current_deg())
        show("read_head_measured_deg", hw.read_head_measured_deg())

        pose = hw.get_map_pose(max_age_s=3.0)
        if pose.success:
            data = pose.data
            show("get_map_pose", pose,
                 "x=%.3f y=%.3f yaw=%.1f° age=%s%s"
                 % (data["x"], data["y"], data["yaw_deg"],
                    None if data["age_s"] is None else round(data["age_s"], 2),
                    " frame_id=''（底盘未填）" if not data["frame_id"] else ""))
        else:
            skip("get_map_pose", pose.message)

        fk = hw.solve_arm_fk()
        if not show("solve_arm_fk", fk):
            return report("S0 readonly")
        pos_l = list(fk.data["left"]["pos"]); zyx_l = quat_to_zyx_deg(fk.data["left"]["quat"])
        pos_r = list(fk.data["right"]["pos"]); zyx_r = quat_to_zyx_deg(fk.data["right"]["quat"])

        arm_deg = hw.read_arm_deg().data
        ik = hw.solve_arm_ik("left", pos_l + zyx_l, frame=1,
                             other_pose_zyx_deg=pos_r + zyx_r,
                             arm_positions_deg=arm_deg)
        if show("solve_ee_poses_ik", ik):
            fk2 = hw.solve_arm_fk(arm_positions_deg=ik.data["q_arm_deg"])
            if fk2.success:
                err = dist(pos_l, fk2.data["left"]["pos"])
                record("ik_fk_roundtrip", "PASS" if err < 0.01 else "FAIL",
                       f"{err * 1000:.2f} mm")

        target = [pos_l[0] + 0.03, pos_l[1], pos_l[2]] + zyx_l
        show("check_ee_accessibility", hw.check_arm_reachability("left", target, frame=1))

        for side in ("left", "right"):
            r = hw.get_arm_reach_error(side)
            err = (r.data or {}).get("err_vector") or []
            if err and all(abs(float(v) - 999.0) < 1e-6 for v in err):
                skip(f"get_ee_pose_reach_error({side})",
                     "空闲返回哨兵 999；只在定时目标执行期间有数据")
            else:
                show(f"get_ee_pose_reach_error({side})", r)

        r = hw.read_arm_control_mode_topic()
        if r.success:
            show("state_arm_control_mode", r, str(r.data))
        elif modes.success:
            substitute("state_arm_control_mode",
                       f"get_control_mode -> arm={modes.data['arm']}, "
                       f"wheel_mpc={modes.data['wheel_mpc']}")
        else:
            skip("state_arm_control_mode", "话题与模式服务都不可用")

    return report("S0 readonly")


if __name__ == "__main__":
    sys.exit(main())
