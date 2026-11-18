#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""JiBot 底盘原始速度通道测试（默认只打印计划，--execute 才动）。

覆盖（接口 → mixin 入口）：
  /move_base/base_cmd_vel     → send_base_cmd_vel_tick / start_cmd_vel_stream /
                                update_cmd_vel_setpoint / stop_cmd_vel_stream
  wheel_mpc 控制权            → enable_vel_control_jibot / get_vel_control_state_jibot
  /move_base/base_cmd_vel_status → get_motion_gate

注意：这是**底盘原始通道**，与 publish_cmd_vel（wheel_mpc 的 cmd_vel_base）是两条路。
      wheel_mpc 运行时会以 ~48 Hz 往同一话题发零速；所以 --execute 先关掉
      wheel_mpc 底盘速度控制（enable_vel_control_jibot(false)），测完再恢复。

用法：
  /usr/bin/python3 apps/ros2_new_sdk/test_mixin_chassis_cmdvel.py              # 计划+零速单帧
  /usr/bin/python3 apps/ros2_new_sdk/test_mixin_chassis_cmdvel.py --execute    # 0.10 m/s × 1 s
  /usr/bin/python3 apps/ros2_new_sdk/test_mixin_chassis_cmdvel.py --execute --velocity 0.05 --seconds 1.5

安全：--execute 会移动底盘（默认前进约 10 cm）；手放急停、前方留净空。
"""
from __future__ import annotations

import argparse
import time

from mixin_test_common import (Ros2WheeledHardware, record, report,
                               set_log_dir, show, skip, start_segment)


def main():
    parser = argparse.ArgumentParser(description="JiBot 底盘原始速度通道测试")
    parser.add_argument("--execute", action="store_true", help="真正发速度（会移动）")
    parser.add_argument("--velocity", type=float, default=0.10,
                        help="前进线速度（m/s），默认 0.10")
    parser.add_argument("--seconds", type=float, default=1.0,
                        help="速度持续时间（秒），默认 1.0")
    parser.add_argument("--rate-hz", type=float, default=100.0,
                        help="流式发布频率，默认 100Hz")
    parser.add_argument("--log-dir", default="")
    args = parser.parse_args()

    set_log_dir(args.log_dir or None)
    start_segment("S12 chassis cmd_vel")

    print("计划: execute=%s vx=%.3f m/s %.1fs @%.0fHz"
          % (args.execute, args.velocity, args.seconds, args.rate_hz))
    if not args.execute:
        print("  （默认只读：只发一帧零速验证通道，不动车）")

    hw = Ros2WheeledHardware(node_name="mixin_chassis_cmdvel",
                             config={"skip_camera": True})
    try:
        hw.spin_for(1.0)
        state = hw.get_vel_control_state_jibot()
        if state.success:
            print("  控制权（执行前）: %s" % bool(state.data))
        else:
            print("  控制权（执行前）: 读不到（%s）" % state.message)

        tick = hw.send_base_cmd_vel_tick(0.0, 0.0, 0.0)
        if tick.success:
            show("send_base_cmd_vel_tick(0,0,0)", tick,
                 "零速单帧，停车由底盘 0.3 s 超时决定")
        elif not args.execute:
            skip("send_base_cmd_vel_tick(0,0,0)",
                 tick.message + "（下位机 bridge/运控未运行？）")
        else:
            show("send_base_cmd_vel_tick(0,0,0)", tick)

        if not args.execute:
            skip("start_cmd_vel_stream", "默认只读；加 --execute 才会流式发速度")
            skip("update_cmd_vel_setpoint", "默认只读")
            skip("stop_cmd_vel_stream", "默认只读")
            skip("enable_vel_control_jibot", "默认只读")
            return report("S12 chassis cmd_vel")

        # 关掉 wheel_mpc 底盘速度控制，否则 48 Hz 零速流会和本测试的原始通道打架
        show("enable_vel_control_jibot(false)", hw.enable_vel_control_jibot(False),
             "已停 wheel_mpc 的 base_cmd_vel 零速流")
        hw.spin_for(0.5)
        flag_off = hw.get_vel_control_state_jibot()
        if flag_off.success:
            state_off = (flag_off.data or {}).get("state") if isinstance(flag_off.data, dict) else flag_off.data
            record("flag readback after false",
                   "PASS" if state_off is False else "FAIL",
                   "flag=%s" % state_off)
        else:
            skip("flag readback after false", flag_off.message)
        show("start_cmd_vel_stream", hw.start_cmd_vel_stream(rate_hz=args.rate_hz),
             "%.0fHz" % args.rate_hz)
        show("update_cmd_vel_setpoint(vx)", hw.update_cmd_vel_setpoint(args.velocity, 0.0, 0.0),
             "vx=%.3f" % args.velocity)
        hw.spin_for(args.seconds)
        show("update_cmd_vel_setpoint(0)", hw.update_cmd_vel_setpoint(0.0, 0.0, 0.0))
        hw.spin_for(0.3)
        show("stop_cmd_vel_stream", hw.stop_cmd_vel_stream(0.2), "零保持 0.2 s")

        show("enable_vel_control_jibot(true)", hw.enable_vel_control_jibot(True),
             "恢复 wheel_mpc 底盘速度控制")
        hw.spin_for(0.5)
        flag_on = hw.get_vel_control_state_jibot()
        if flag_on.success:
            state_on = (flag_on.data or {}).get("state") if isinstance(flag_on.data, dict) else flag_on.data
            record("flag readback after true",
                   "PASS" if state_on is True else "FAIL",
                   "flag=%s" % state_on)
        else:
            skip("flag readback after true", flag_on.message)

        gate = hw.get_motion_gate()
        record("get_motion_gate", "PASS",
               "effective=%s reason=%s age=%s"
               % (gate.get("cmd_vel_effective"), gate.get("reason"),
                  gate.get("effective_age_s")))
    finally:
        try:
            hw.stop_cmd_vel_stream(0.0)
        except Exception:
            pass
        try:
            hw.enable_vel_control_jibot(True)
        except Exception:
            pass
        hw.close()

    return report("S12 chassis cmd_vel")


if __name__ == "__main__":
    raise SystemExit(main())
