#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""JiBot 底盘导航接口测试（默认只打印计划，--execute 才动）。

覆盖（接口 → mixin 入口）：
  /move_base/base_move                 → base_move_relative_jibot（本体系相对）
  /move_base/move_to_target            → base_move_to_target_jibot（世界/定位系绝对，需 --target-x）
  /move_base/check_arrived             → check_arrived_jibot
  wheel_mpc 控制权交接                 → base_move_* 自动 suspend，终态自动 restore

用法：
  /usr/bin/python3 apps/ros2_new_sdk/test_mixin_chassis.py                 # 只打印计划+只读
  /usr/bin/python3 apps/ros2_new_sdk/test_mixin_chassis.py --execute       # 前进 0.10 m 并等到达
  /usr/bin/python3 apps/ros2_new_sdk/test_mixin_chassis.py --execute --distance 0.05
  /usr/bin/python3 apps/ros2_new_sdk/test_mixin_chassis.py --execute \
      --target-x 7.10 --target-y 1.30 --target-yaw-deg 172    # 绝对目标（地图/定位系）

安全：--execute 会移动底盘；默认前方留 30 cm 净空、手放急停。
本体系是相对当前位姿的增量；世界系是底盘定位坐标系（与 /move_base/amcl_pose 同一套）。
"""
from __future__ import annotations

import argparse
import time

from mixin_test_common import (Ros2WheeledHardware, record, report,
                               set_log_dir, show, skip, start_segment)
from core.domain.chassis_options import MoveToTargetOptions

TERMINAL_STATUS = (2, 3)


def build_options(args):
    return MoveToTargetOptions(
        avoid_enabled=args.avoid,
        avoid_distance=0.5,
        linear_velocity=args.linear_velocity,
        angular_velocity=args.angular_velocity,
        position_threshold=args.threshold,
        angle_threshold=args.angle_threshold,
        allow_rotation=True,
    )


def wait_arrival(hw, task_id, timeout):
    """轮询 check_arrived_jibot 直到到达/失败/超时，返回最后一次 Result。"""
    deadline = time.time() + timeout
    last = None
    while time.time() < deadline:
        last = hw.check_arrived_jibot(task_id, blocking=False)
        if not last.success:
            return last
        data = last.data or {}
        if data.get("arrived") or int(data.get("status", 0)) in TERMINAL_STATUS:
            return last
        hw.spin_for(0.5)
    return last


def main():
    parser = argparse.ArgumentParser(description="JiBot 底盘导航接口测试")
    parser.add_argument("--execute", action="store_true", help="真正下发导航目标")
    parser.add_argument("--distance", type=float, default=0.10,
                        help="本体系前进距离（米），默认 0.10")
    parser.add_argument("--threshold", type=float, default=0.01,
                        help="位置到位容差（米），默认 0.01")
    parser.add_argument("--angle-threshold", type=float, default=0.02,
                        help="角度到位容差（弧度），默认 0.02")
    parser.add_argument("--linear-velocity", type=float, default=0.15,
                        help="目标线速度（m/s），默认 0.15")
    parser.add_argument("--angular-velocity", type=float, default=0.25,
                        help="目标角速度（rad/s），默认 0.25")
    parser.add_argument("--avoid", action="store_true",
                        help="启用避障（默认关，和现场调试一致）")
    parser.add_argument("--target-x", type=float, default=None,
                        help="世界系绝对目标 x（米）；给了就走 move_to_target")
    parser.add_argument("--target-y", type=float, default=0.0)
    parser.add_argument("--target-yaw-deg", type=float, default=0.0)
    parser.add_argument("--timeout", type=float, default=60.0,
                        help="等待到达上限（秒），默认 60")
    parser.add_argument("--log-dir", default="")
    args = parser.parse_args()

    set_log_dir(args.log_dir or None)
    start_segment("S11 chassis nav")

    absolute = args.target_x is not None
    print("计划: execute=%s 目标=%s 距离=%.3f m 容差=%.3f m/%.3f rad timeout=%.0fs"
          % (args.execute,
             ("world(%.3f, %.3f, %.1f°)" % (args.target_x, args.target_y,
                                            args.target_yaw_deg)) if absolute
             else "relative(+%.3f m)" % args.distance,
             args.distance, args.threshold, args.angle_threshold, args.timeout))

    with Ros2WheeledHardware(
            node_name="mixin_chassis_nav",
            config={"skip_camera": True}) as hw:
        try:
            hw.spin_for(1.0)
            before = hw.get_vel_control_state_jibot()
            if before.success:
                print("  控制权（执行前）: %s" % bool(before.data))
            else:
                print("  控制权（执行前）: 读不到（%s）" % before.message)

            if not args.execute:
                skip("base_move_relative_jibot", "默认只读；加 --execute 才会移动")
                skip("check_arrived_jibot", "默认只读")
                return report("S11 chassis nav")

            options = build_options(args)
            if absolute:
                launch = hw.base_move_to_target_jibot(
                    args.target_x, args.target_y,
                    args.target_yaw_deg * 3.141592653589793 / 180.0, options)
                label = "base_move_to_target_jibot"
            else:
                launch = hw.base_move_relative_jibot(
                    args.distance, 0.0, 0.0, options)
                label = "base_move_relative_jibot"
            ok = show(label, launch,
                      "task_id=%s" % ((launch.data or {}).get("task_id", "")))
            task_id = (launch.data or {}).get("task_id", "") if launch.success else ""
            if not ok or not task_id:
                return report("S11 chassis nav")

            result = wait_arrival(hw, task_id, args.timeout)
            if result is None:
                record("check_arrived_jibot", "FAIL", "未取得查询结果")
            else:
                show("check_arrived_jibot", result, str(result.data))
                if result.success and not (result.data or {}).get("arrived"):
                    record("check_arrived_jibot(arrived)", "FAIL",
                           "任务结束但未到达: %s" % (result.data,))

            hw.spin_for(0.5)
            after = hw.get_vel_control_state_jibot()
            if before.success and after.success:
                record("控制权已恢复", "PASS" if bool(after.data) == bool(before.data)
                       else "FAIL", "before=%s after=%s" % (bool(before.data), bool(after.data)))
            else:
                skip("控制权已恢复", "执行前后有一侧读不到")

            restore = hw.restore_vel_control()
            print("  restore_vel_control():", restore.message)

            gate = hw.get_motion_gate()
            record("get_motion_gate", "PASS",
                   "effective=%s reason=%s age=%s"
                   % (gate.get("cmd_vel_effective"), gate.get("reason"),
                      gate.get("effective_age_s")))
        finally:
            try:
                hw.restore_vel_control()
            except Exception:
                pass

    return report("S11 chassis nav")


if __name__ == "__main__":
    raise SystemExit(main())
