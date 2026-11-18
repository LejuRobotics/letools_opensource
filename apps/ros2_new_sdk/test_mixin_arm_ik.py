#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""手臂 IK/FK 缺口补测（经 Ros2WheeledHardware，默认 dry-run）。

覆盖本轮新封装的端点：
  /planning/kinematics/solve_arm_fk_with_frame
  /planning/kinematics/solve_ee_poses_ik_multi_refer
  /control/kinematics/cmd_ee_poses_ik
  /control/kinematics/cmd_ee_poses_advance_ik
  IHardware.solve_ik(IkSolveRequest)（multi -> single 回退）
以及已封装但未实测的 get_joint_torque / get_reach_time。

安全边界：--execute 只做“求解 + 发 IK 输入话题”，IK 输入只产出
ik_result_arm_joints，不驱动关节；--arm-reset 才会让手臂回中。

用法：
  /usr/bin/python3 apps/ros2_new_sdk/test_mixin_arm_ik.py
  /usr/bin/python3 apps/ros2_new_sdk/test_mixin_arm_ik.py --execute
  /usr/bin/python3 apps/ros2_new_sdk/test_mixin_arm_ik.py --execute --arm-reset
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from core.domain.ik import IkSolution, IkSolveRequest

from mixin_test_common import (
    Ros2WheeledHardware, dist, quat_to_zyx_deg, record, report,
    set_log_dir, show, skip, start_segment,
)


def finite(values) -> bool:
    return bool(values) and all(math.isfinite(float(v)) for v in values)


def main():
    parser = argparse.ArgumentParser(description="手臂 IK/FK 缺口补测")
    parser.add_argument("--execute", action="store_true",
                        help="执行求解与 IK 输入话题（不驱动关节）")
    parser.add_argument("--arm-reset", action="store_true",
                        help="额外执行 arm_reset（手臂会回中）")
    parser.add_argument("--z-offset", type=float, default=0.02,
                        help="IK 目标相对 FK 的 z 偏移（米），默认 0.02")
    parser.add_argument("--log-dir", default="")
    args = parser.parse_args()

    set_log_dir(args.log_dir or None)
    start_segment("S7 arm ik/fk")

    with Ros2WheeledHardware(node_name="mixin_arm_ik_test") as hw:
        hw.spin_for(0.5)
        if not hw.wait_arm_ready(10.0):
            record("wait_arm_ready", "FAIL", "10 s 内没有 /lowstate")
            return report("S7")
        record("wait_arm_ready", "PASS", "lowstate 就绪")

        fk = hw.solve_arm_fk()
        if not show("solve_arm_fk", fk):
            return report("S7")
        left_pos = [float(v) for v in fk.data["left"]["pos"]]
        right_pos = [float(v) for v in fk.data["right"]["pos"]]
        left_zyx = quat_to_zyx_deg(fk.data["left"]["quat"])
        right_zyx = quat_to_zyx_deg(fk.data["right"]["quat"])
        record("fk_pose_readable", "PASS",
               f"left={[round(v, 4) for v in left_pos]} zyx_deg={[round(v, 2) for v in left_zyx]}")

        frame_fk = hw.solve_arm_fk_with_frame()
        if show("solve_arm_fk_with_frame", frame_fk):
            delta = dist(frame_fk.data["left"]["pos"], left_pos)
            record("fk_with_frame_matches_fk",
                   "PASS" if delta < 0.001 else "FAIL",
                   f"left pos diff {delta * 1000.0:.3f} mm")

        target_left = left_pos[:2] + [left_pos[2] + args.z_offset] + left_zyx
        target_right = right_pos[:2] + [right_pos[2] + args.z_offset] + right_zyx
        record("ik_target_built", "PASS",
               f"left z {left_pos[2]:.4f} -> {target_left[2]:.4f} m")

        if not args.execute:
            skip("solve_arm_ik(single)", "dry-run；加 --execute")
            skip("solve_arm_ik_multi_refer", "dry-run；加 --execute")
            skip("cmd_ee_poses_ik", "dry-run；加 --execute")
            skip("cmd_ee_poses_advance_ik", "dry-run；加 --execute")
            skip("solve_ik(IkSolveRequest)", "dry-run；加 --execute")
            skip("get_joint_torque", "dry-run；加 --execute")
            skip("get_reach_time", "dry-run；加 --execute")
            skip("arm_reset", "dry-run；加 --execute --arm-reset")
            return report("S7")

        measured = hw.read_arm_deg()
        q_measured_deg = ([float(v) for v in measured.data]
                          if measured.success else None)

        single = hw.solve_arm_ik("left", target_left, frame=0,
                                 other_pose_zyx_deg=target_right)
        if show("solve_arm_ik(single)", single):
            q_deg = single.data.get("q_arm_deg") or []
            record("single_ik_q14", "PASS" if len(q_deg) == 14 and finite(q_deg) else "FAIL",
                   f"{len(q_deg)} joints")
            poses = single.data.get("poses") or {}
            if poses.get("left"):
                err = dist(poses["left"]["pos"], target_left[:3])
                record("single_ik_pose_error",
                       "PASS" if err < 0.005 else "FAIL",
                       f"{err * 1000.0:.2f} mm")

        multi = hw.solve_arm_ik_multi_refer("left", target_left, frame=0,
                                            other_pose_zyx_deg=target_right)
        if show("solve_arm_ik_multi_refer", multi):
            q_deg = multi.data.get("q_arm_deg") or []
            record("multi_ik_q14", "PASS" if len(q_deg) == 14 and finite(q_deg) else "FAIL",
                   f"{len(q_deg)} joints")
            poses = multi.data.get("poses") or {}
            if poses.get("left"):
                err = dist(poses["left"]["pos"], target_left[:3])
                record("multi_ik_pose_error",
                       "PASS" if err < 0.005 else "FAIL",
                       f"{err * 1000.0:.2f} mm")

        # 求解结果的只读订阅/缓存由 ArmClient 持有，经 Mixin 取新帧计数。
        hw.spin_for(0.5)

        before = hw.arm_ik_result_frames
        result = hw.send_ee_poses_ik(target_left, target_right, repeats=5)
        for _ in range(30):
            hw.spin_for(0.1)
            if hw.arm_ik_result_frames > before:
                break
        record("cmd_ee_poses_ik",
               "PASS" if result.success and hw.arm_ik_result_frames > before else "FAIL",
               f"{result.message}; ik_result +{hw.arm_ik_result_frames - before} frames")

        before = hw.arm_ik_result_frames
        result = hw.send_ee_poses_advance_ik(target_left, target_right, repeats=5)
        for _ in range(30):
            hw.spin_for(0.1)
            if hw.arm_ik_result_frames > before:
                break
        record("cmd_ee_poses_advance_ik",
               "PASS" if result.success and hw.arm_ik_result_frames > before else "FAIL",
               f"{result.message}; ik_result +{hw.arm_ik_result_frames - before} frames")

        request = IkSolveRequest(
            left_pose=left_pos + [float(v) for v in fk.data["left"]["quat"]],
            right_pose=right_pos + [float(v) for v in fk.data["right"]["quat"]],
        )
        solved = hw.solve_ik(request)
        if show("solve_ik(IkSolveRequest)", solved):
            solution = solved.data
            record("solve_ik_type", "PASS" if isinstance(solution, IkSolution) else "FAIL",
                   type(solution).__name__)
            if isinstance(solution, IkSolution):
                q14 = [float(v) for v in solution.q_arm]
                ok = len(q14) == 14 and finite(q14)
                record("solve_ik_q14", "PASS" if ok else "FAIL", f"{len(q14)} joints")
                step_deg = 0.0
                if q_measured_deg and len(q_measured_deg) == 14:
                    step_deg = max(
                        abs(math.degrees(q14[i]) - q_measured_deg[i]) for i in range(14))
                record("solve_ik_continuity",
                       "PASS" if step_deg <= float(request.max_joint_step_deg) else "FAIL",
                       f"max step {step_deg:.2f}° (limit {request.max_joint_step_deg}°)")
                record("solve_ik_solver", "PASS",
                       f"solver={solution.solver} elapsed={solution.elapsed_ms:.1f} ms")

        torque = hw.get_joint_torque()
        arm_torques = torque.get("arm_torques") if isinstance(torque, dict) else None
        record("get_joint_torque",
               "PASS" if arm_torques and finite(arm_torques) else "FAIL",
               f"{len(arm_torques)} arm torques" if arm_torques else "no data")

        reach = hw.get_reach_time("arm_joint")
        if reach is None:
            skip("get_reach_time(arm_joint)", "空闲无 reach 帧")
        else:
            record("get_reach_time(arm_joint)", "PASS", f"{float(reach):.3f} s")

        if args.arm_reset:
            show("arm_reset", hw.arm_reset())
        else:
            skip("arm_reset", "未加 --arm-reset（会动臂）")

    return report("S7")


if __name__ == "__main__":
    raise SystemExit(main())
