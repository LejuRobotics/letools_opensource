#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""底盘 8 条接口 mixin 级测试（Ros2WheeledHardware）。默认只读；--execute 才动。

覆盖（接口 → mixin 入口）：
  /control/wheel_mpc/reset_vel_ruckig              → reset_vel_ruckig
  /control/locomotion/state_base_reach_time        → base_reach_topic_available /
                                                     base_reach_frames / base_reach_value /
                                                     wait_base_reach_time
  /control/locomotion/set_base_pose_timed_target   → send_base_pose_timed（world→frame 0，local→frame 1）
  /control/locomotion/cmd_vel_base                 → send_base_velocity(frame='local')，流式+自动归零
  /control/locomotion/cmd_vel_odom                 → send_base_velocity(frame='world')，需 --world
  /control/locomotion/cmd_pose_base                → send_base_pose(frame='local')，本体系增量
  /control/locomotion/cmd_pose_odom                → send_base_pose(frame='world')，需 --world + --odom-x
  /control/locomotion/get_velocity_limits          → get_velocity_limits

用法：
  source /opt/ros/jazzy/setup.bash
  export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp LEJU_HW_SKIP_ROS1=1
  /usr/bin/python3 apps/ros2_new_sdk/test_mixin_base.py                # dry-run：只读+打印计划
  /usr/bin/python3 apps/ros2_new_sdk/test_mixin_base.py --execute      # 本体系用例（每条都很小）
  /usr/bin/python3 apps/ros2_new_sdk/test_mixin_base.py --execute --only vel_base
  /usr/bin/python3 apps/ros2_new_sdk/test_mixin_base.py --execute --world --odom-x 0 --odom-y 0 --odom-yaw-deg 0
                                                                       # 世界系用例最后跑，绝对坐标必须显式给

only 取值：limits vel_base pose_base pose_timed vel_odom pose_timed_world pose_odom all

安全：--execute 会移动底盘；默认每条 0.05 m / 1 s；留空间、手放急停。
世界系是**运控启动时建立的里程计参考系**（启动瞬间为原点 0,0,0；不是 SLAM/amcl，也不是 /odom）；
运控重启后原点重置。cmd_pose_odom 会在
底盘任务系统里留下持久任务，reset_vel_ruckig 只清规划器，善后要人工确认。
上位机没有 /odom，位移全部以目视为准（对照录像）。
"""
from __future__ import annotations

import argparse
import sys

from mixin_test_common import (
    Ros2WheeledHardware, record, show, skip, substitute, report,
    set_log_dir, start_segment,
)

STEPS = ["limits", "vel_base", "pose_base", "pose_timed",
         "vel_odom", "pose_timed_world", "pose_odom"]
WORLD_STEPS = ("vel_odom", "pose_timed_world", "pose_odom")


def reach_available(hw):
    """state_base_reach_time 是普通 bool 入口；无发布者时按需出现，给 SKIP。"""
    if hw.base_reach_topic_available():
        record("state_base_reach_time(avail)", "PASS",
               f"frames={hw.base_reach_frames} value={hw.base_reach_value}")
    else:
        skip("state_base_reach_time(avail)", "无发布者；定时目标执行期间可能按需出现")


def base_reach(hw, name, frames, fallback):
    """定时目标后等一帧新的 reach_time；无新帧时回退服务的 actual_time 并标平替。"""
    result = hw.wait_base_reach_time(frames, 3.0, fallback_actual_time=fallback)
    if not result.success:
        show(name, result)
    elif "fallback" in result.message:
        substitute(name, f"actual_time={float(result.data['actual_time']):.2f}s")
    else:
        show(name, result, f"{float(result.data['actual_time']):.2f}s")


def run_vel(hw, args, frame, name):
    """速度是流式话题：mixin 内部按 rate_hz 连续发 duration 秒后自动归零。"""
    print("\n## %s (%s) vx=%.3f m/s %.1fs @%.0fHz"
          % (name, frame, args.velocity, args.vel_seconds, args.rate_hz))
    r = hw.send_base_velocity(args.velocity, 0.0, 0.0, frame=frame,
                              duration=args.vel_seconds, rate_hz=args.rate_hz)
    show(name, r, "已归零；位移目视")
    hw.spin_for(args.settle)   # 等速度输入失效，避免下一条位姿目标被忽略
    show("reset_vel_ruckig", hw.reset_vel_ruckig())


def run_pose_base(hw, args):
    print("\n## cmd_pose_base 本体系增量 %.3f m" % args.distance)
    hw.spin_for(args.settle)   # 位姿目标前先确保没有速度输入在有效期内
    r = hw.send_base_pose(args.distance, 0.0, 0.0, frame="local")
    show("cmd_pose_base", r, "已下发一次；位移目视")
    hw.spin_for(args.seconds + args.margin)
    show("reset_vel_ruckig", hw.reset_vel_ruckig())


def run_pose_timed(hw, args, frame, name):
    if frame == "world" and args.odom_x is None:
        skip(name, "世界系定时目标是地图绝对坐标，必须显式给 --odom-x/--odom-y/--odom-yaw-deg")
        return
    if frame == "world":
        x, y, yaw = args.odom_x, args.odom_y, args.odom_yaw_deg
        print("\n## %s 世界系绝对 x=%.3f y=%.3f yaw=%.1f deg / %.1fs"
              % (name, x, y, yaw, args.seconds))
    else:
        x, y, yaw = args.distance, 0.0, 0.0
        print("\n## %s 本体系增量 %.3f m / %.1fs" % (name, x, args.seconds))
    frames = hw.base_reach_frames
    r = hw.send_base_pose_timed(x, y, yaw, frame=frame,
                                desire_time=args.seconds)
    if not show(name, r, str(r.data)):
        show("reset_vel_ruckig", hw.reset_vel_ruckig())
        return
    base_reach(hw, "state_base_reach_time", frames, (r.data or {}).get("actual_time"))
    actual = float((r.data or {}).get("actual_time") or 0.0)
    print("    等待 actual_time + margin（期间不发 reset）")
    hw.spin_for(actual + args.margin)
    print("    请目视确认：底盘到目标位后停住")
    show("reset_vel_ruckig", hw.reset_vel_ruckig())


def run_pose_odom(hw, args):
    if args.odom_x is None:
        skip("cmd_pose_odom", "未给绝对目标（--odom-x）；世界系绝对位姿必须明确给出")
        return
    print("\n## cmd_pose_odom 世界系绝对 x=%.3f y=%.3f yaw=%.1f deg"
          % (args.odom_x, args.odom_y, args.odom_yaw_deg))
    print("    注意：cmd_pose_odom 会留下持久底盘任务，执行后请目视确认并善后")
    hw.spin_for(args.settle)
    r = hw.send_base_pose(args.odom_x, args.odom_y, args.odom_yaw_deg, frame="world")
    show("cmd_pose_odom", r, "已下发一次；到位目视")
    hw.spin_for(args.seconds + args.margin)
    show("reset_vel_ruckig", hw.reset_vel_ruckig())


def warn_world():
    print("\n!! 世界系用例（cmd_vel_odom / set_base_pose_timed_target world / cmd_pose_odom）")
    print("!! 目标是运控里程计参考系绝对坐标（启动瞬间为原点 0,0,0；不是 SLAM/amcl，也不是 /odom）；")
    print("!! cmd_pose_odom 会在底盘任务系统里留下持久任务，reset 只清规划器，善后需人工确认。")


def main():
    parser = argparse.ArgumentParser(description="底盘 8 条接口 mixin 级测试")
    parser.add_argument("--execute", action="store_true", help="真正下发动作")
    parser.add_argument("--world", action="store_true",
                        help="允许世界系用例（vel_odom / pose_timed_world / pose_odom）")
    parser.add_argument("--only", default="all", choices=["all"] + STEPS)
    parser.add_argument("--log-dir", default="")
    parser.add_argument("--distance", type=float, default=0.05, help="本体系位移（米），默认 0.05")
    parser.add_argument("--velocity", type=float, default=0.05, help="速度用例 vx（m/s），默认 0.05")
    parser.add_argument("--vel-seconds", type=float, default=1.0, help="速度持续时间（秒），默认 1.0")
    parser.add_argument("--rate-hz", type=float, default=25.0, help="速度流式频率，默认 25Hz")
    parser.add_argument("--seconds", type=float, default=2.0,
                        help="定时目标 desire_time / 位姿用例等待基数（秒），默认 2.0")
    parser.add_argument("--margin", type=float, default=2.0, help="到位后额外等待（秒），默认 2.0")
    parser.add_argument("--settle", type=float, default=2.5,
                        help="速度阶段后的等待（秒），让速度输入失效，默认 2.5")
    parser.add_argument("--odom-x", type=float, default=None, help="世界系绝对 x（米）")
    parser.add_argument("--odom-y", type=float, default=0.0)
    parser.add_argument("--odom-yaw-deg", type=float, default=0.0)
    args = parser.parse_args()

    set_log_dir(args.log_dir or None)
    start_segment("S9 base")

    steps = STEPS if args.only == "all" else [args.only]
    print("计划: only=%s execute=%s world=%s distance=%.3fm velocity=%.3fm/s %.1fs"
          % (args.only, args.execute, args.world, args.distance, args.velocity,
             args.vel_seconds))
    for step in steps:
        if step in WORLD_STEPS and not args.world:
            print("  - %-18s 跳过（世界系用例需要 --world）" % step)

    with Ros2WheeledHardware(node_name="mixin_base_test") as hw:
        try:
            # ---- 只读：限位 + reach 可用性（dry-run 也跑）----
            if "limits" in steps:
                lim = hw.get_velocity_limits()
                if show("get_velocity_limits", lim, str(lim.data)):
                    print("    vx=%s vy=%s wz=%s" % tuple(lim.data.values()))
                else:
                    print("    get_velocity_limits 不可用: %s" % lim.message)
            reach_available(hw)

            motion = [s for s in steps if s != "limits"]
            if not args.execute:
                for step in motion:
                    if step in WORLD_STEPS and not args.world:
                        skip(step, "需要 --world（世界系用例）")
                    else:
                        skip(step, "需要 --execute")
                record("visual_position_check", "SKIP",
                       "本机无 /odom；位移全部对照录像目视确认")
                return report("S9 base")

            for step in motion:
                if step in WORLD_STEPS and not args.world:
                    skip(step, "需要 --world（世界系用例）")
                    continue
                if step in WORLD_STEPS:
                    warn_world()
                if step == "vel_base":
                    run_vel(hw, args, "local", "cmd_vel_base")
                elif step == "pose_base":
                    run_pose_base(hw, args)
                elif step == "pose_timed":
                    run_pose_timed(hw, args, "local", "set_base_pose_timed_target(local)")
                elif step == "vel_odom":
                    run_vel(hw, args, "world", "cmd_vel_odom")
                elif step == "pose_timed_world":
                    run_pose_timed(hw, args, "world", "set_base_pose_timed_target(world)")
                elif step == "pose_odom":
                    run_pose_odom(hw, args)

            record("visual_position_check", "SKIP",
                   "本机无 /odom；位移全部对照录像目视确认")
        finally:
            # 每条运动后已各自 reset；这里兜底再清一次规划器。
            try:
                hw.reset_vel_ruckig()
            except Exception:  # noqa: BLE001
                pass

    return report("S9 base")


if __name__ == "__main__":
    sys.exit(main())
