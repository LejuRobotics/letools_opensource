#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""JiBot 底盘只读接口测试（不改变任何状态）。

覆盖（接口 → mixin 入口）：
  /control/wheel_mpc/enable_vel_control        → get_vel_control_state_jibot
  /move_base/base_cmd_vel_status               → get_motion_gate

用法：
  source /opt/ros/jazzy/setup.bash
  export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp LEJU_HW_SKIP_ROS1=1
  /usr/bin/python3 apps/ros2_new_sdk/test_mixin_chassis_readonly.py
"""
from __future__ import annotations

import argparse

from mixin_test_common import (Ros2WheeledHardware, record, report,
                               set_log_dir, show, skip, start_segment)


def main():
    parser = argparse.ArgumentParser(description="JiBot 底盘只读接口测试")
    parser.add_argument("--log-dir", default="")
    args = parser.parse_args()

    set_log_dir(args.log_dir or None)
    start_segment("S10 chassis readonly")

    with Ros2WheeledHardware(
            node_name="mixin_chassis_readonly",
            config={"skip_camera": True}) as hw:
        hw.spin_for(1.5)   # 等 best_effort 订阅收到真实新帧（volatile 不保留历史）

        state = hw.get_vel_control_state_jibot()
        if state.success:
            data = state.data if isinstance(state.data, dict) else {}
            show("get_vel_control_state_jibot", state,
                 "state=%s source=%s" % (data.get("state"), data.get("source")))
        else:
            skip("get_vel_control_state_jibot",
                 state.message + "（该话题只在受理服务时发一帧，只读模式拿不到）")

        gate = hw.get_motion_gate()
        record("get_motion_gate", "PASS",
               "effective=%s reason=%s age=%s"
               % (gate.get("cmd_vel_effective"), gate.get("reason"),
                  gate.get("effective_age_s")))
        print("  gate:", gate)
        print("  说明：有效性为 True 且 age=None 表示尚未收到状态，按有效处理（与 ROS 1 一致）")

        pose = hw.get_map_pose(max_age_s=3.0)
        if pose.success:
            data = pose.data
            show("get_map_pose", pose,
                 "x=%.3f y=%.3f yaw=%.1f° age=%s%s"
                 % (data["x"], data["y"], data["yaw_deg"],
                    None if data["age_s"] is None else round(data["age_s"], 2),
                    " frame_id=''（底盘未填）" if not data["frame_id"] else ""))
            print("  map pose: x=%.3f m y=%.3f m yaw=%.1f° age=%s"
                  % (data["x"], data["y"], data["yaw_deg"],
                     None if data["age_s"] is None else round(data["age_s"], 2)))
        else:
            skip("get_map_pose", pose.message)

    return report("S10 chassis readonly")


if __name__ == "__main__":
    raise SystemExit(main())
