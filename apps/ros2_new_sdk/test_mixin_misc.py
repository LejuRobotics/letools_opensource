#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""辅助服务测试（需 --execute）。覆盖文件：mixin_test_common.py + 本文件。

覆盖接口：
  /control/wheel_mpc/enable_single_timed_target_async_exec
  /control/arm/execute_arm_action
  /control/arm/interrupt_arm_traj

用法：
  /usr/bin/python3 apps/ros2_new_sdk/test_mixin_misc.py --execute [--action welcome]
"""
from __future__ import annotations

import argparse
import sys

from mixin_test_common import (Ros2WheeledHardware, show, report,
                                set_log_dir, start_segment, prepare_modes)


def main():
    parser = argparse.ArgumentParser(description="辅助服务测试")
    parser.add_argument("--execute", action="store_true", help="真正下发动作")
    parser.add_argument("--action", default="welcome")
    parser.add_argument("--log-dir", default="",
                        help="结果落盘目录；默认 ~/mixin_test_logs/<日期>/")
    args = parser.parse_args()

    set_log_dir(args.log_dir or None)
    start_segment("S3 misc")
    if not args.execute:
        print("这是动作用例，需要 --execute。只读检查请跑 test_mixin_readonly.py")
        return 0

    with Ros2WheeledHardware(node_name="mixin_misc_test") as hw:
        show("wait_arm_ready", hw.wait_arm_ready(10.0))
        # 命名动作要求 quick mode=1；prepare 会在 close 时恢复
        if not prepare_modes(hw, quick=1):
            return report("S3 misc")

        show("enable_async(true)", hw.set_async_exec(True))
        show("enable_async(false)", hw.set_async_exec(False))

        r = hw.send_arm_action(args.action)
        detail = (r.message or "").lower()
        if not r.success and ("dexterous hand feedback" in detail
                              or "end effector" in detail
                              or "末端执行器" in (r.message or "")):
            # 无末端执行器的机型：运控在播放前就拒绝，属环境不适用，不是接口故障。
            skip(f"execute_arm_action({args.action})",
                 r.message + "（本机型无末端执行器，动作不可用）")
        else:
            show(f"execute_arm_action({args.action})", r)
        hw.spin_for(1.0)
        show("interrupt_arm_traj", hw.interrupt_arm_action())

    return report("S3 misc")


if __name__ == "__main__":
    sys.exit(main())
