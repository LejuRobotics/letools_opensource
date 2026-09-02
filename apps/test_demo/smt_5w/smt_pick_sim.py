#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""SMT 料盘抓取仿真：使用手工填写的虚假二维码坐标，不控制底盘。"""

from __future__ import annotations

import sys
from typing import Any, Dict, Sequence, Tuple

import smt_scene as scene


# 仿真前填写二维码在 IK 坐标系中的虚假 XYZ，单位为米。
FAKE_TRAY_QR_XYZ = [...]


def _validate_parameters() -> Tuple[list, str]:
    xyz = scene._vector(FAKE_TRAY_QR_XYZ, 3, "FAKE_TRAY_QR_XYZ")
    hand = "left" if xyz[1] >= 0.0 else "right"
    ready_joints = (
        scene.LEFT_READY_JOINTS_DEG
        if hand == "left"
        else scene.RIGHT_READY_JOINTS_DEG
    )
    carry_joints = (
        scene.LEFT_CARRY_JOINTS_DEG
        if hand == "left"
        else scene.RIGHT_CARRY_JOINTS_DEG
    )
    for name, value, length in (
        ("INITIAL_RESET_JOINTS_DEG", scene.INITIAL_RESET_JOINTS_DEG, 14),
        ("SCAN_READY_JOINTS_DEG", scene.SCAN_READY_JOINTS_DEG, 14),
        ("%s_READY_JOINTS_DEG" % hand.upper(), ready_joints, 14),
        ("%s_CARRY_JOINTS_DEG" % hand.upper(), carry_joints, 14),
        ("PICK_CLEARANCE_XYZ", scene.PICK_CLEARANCE_XYZ, 3),
        ("PICK_OFFSET_XYZ", scene.PICK_OFFSET_XYZ, 3),
    ):
        scene._vector(value, length, name)
    return xyz, hand


def _build_arm_runtime() -> scene.SceneRuntime:
    end_effector_type = "lejuclaw"
    params: Dict[str, Any] = {
        "robot_type": "wheel",
        "end_effector_type": end_effector_type,
        "end_effector": {"type": end_effector_type},
    }
    robot_io = scene.build_robot_io(
        "wheel",
        params=params,
        node_name="smt_pick_sim",
        init_node=True,
    )
    return scene.SceneRuntime(
        io=robot_io,
        motion=None,
        qr=None,
        arm=scene.ArmController(robot_io, params=params),
        end_effector=scene.EndEffectorController(robot_io, config=params),
    )


def run_pick(runtime: scene.SceneRuntime, qr_xyz: Sequence[float], hand: str) -> None:
    end_effector_type = "lejuclaw"
    gripper_settle_s = 0.5
    ready_joints = (
        scene.LEFT_READY_JOINTS_DEG
        if hand == "left"
        else scene.RIGHT_READY_JOINTS_DEG
    )
    inactive_scan_pose = (
        scene.RIGHT_SCAN_READY_POSE
        if hand == "left"
        else scene.LEFT_SCAN_READY_POSE
    )
    carry_joints = (
        scene.LEFT_CARRY_JOINTS_DEG
        if hand == "left"
        else scene.RIGHT_CARRY_JOINTS_DEG
    )
    fake_qr = {"x": qr_xyz[0], "y": qr_xyz[1], "z": qr_xyz[2]}

    scene._require_success(
        runtime.arm.enter_external_mode(),
        "进入手臂外部控制模式",
    )
    current_joints = runtime.arm.require_current_arm_joints_deg()
    current_joints = scene._move_arms(
        runtime,
        scene.INITIAL_RESET_JOINTS_DEG,
        current_joints,
        "仿真抓取初始姿态",
    )
    current_joints = scene._move_arms(
        runtime,
        scene.SCAN_READY_JOINTS_DEG,
        current_joints,
        "仿真抓取前双臂准备姿态",
    )
    scene._require_success(
        runtime.end_effector.open(hand, end_effector_type=end_effector_type),
        "%s夹爪打开" % hand,
    )
    runtime.io.sleep(gripper_settle_s)

    pick_stage2 = scene._add_xyz(fake_qr, scene.PICK_OFFSET_XYZ)
    pick_stage1 = scene._offset_xyz(pick_stage2, scene.PICK_CLEARANCE_XYZ)
    first, second = scene._solve_two_stage(
        runtime,
        hand,
        pick_stage1,
        pick_stage2,
        ready_joints,
        "仿真抓取",
        inactive_arm_joints_deg=inactive_scan_pose,
        inactive_pose_label="扫码姿态",
    )
    current_joints = scene._execute_two_stage(
        runtime,
        first,
        second,
        current_joints,
        "仿真抓取",
    )
    scene._require_success(
        runtime.end_effector.close(hand, end_effector_type=end_effector_type),
        "%s夹爪闭合" % hand,
    )
    runtime.io.sleep(gripper_settle_s)
    scene._move_arms(
        runtime,
        carry_joints,
        current_joints,
        "%s臂持物姿态" % hand,
    )
    runtime.io.loginfo(
        "抓取仿真完成：hand=%s fake_qr=(%.4f, %.4f, %.4f)，夹爪保持闭合",
        hand,
        qr_xyz[0],
        qr_xyz[1],
        qr_xyz[2],
    )


def main() -> int:
    runtime = None
    try:
        qr_xyz, hand = _validate_parameters()
        runtime = _build_arm_runtime()
        run_pick(runtime, qr_xyz, hand)
        return 0
    except KeyboardInterrupt:
        if runtime is not None:
            runtime.io.logwarn("抓取仿真被中断；未自动松爪或复位手臂")
        return 130
    except Exception as exc:
        if runtime is not None:
            runtime.io.logerr("抓取仿真失败；未自动松爪或复位手臂: %s", exc)
        else:
            print("抓取仿真启动失败: %s" % exc, file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
