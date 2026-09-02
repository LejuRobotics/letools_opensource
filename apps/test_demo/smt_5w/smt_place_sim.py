#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""SMT 料盘放置仿真：使用手工填写的虚假二维码坐标，不控制底盘。"""

from __future__ import annotations

import sys
from typing import Any, Dict, Sequence, Tuple

import smt_scene as scene


# 仿真前填写箱子二维码在 IK 坐标系中的虚假 XYZ，单位为米。
FAKE_BOX_QR_XYZ = [...]

# 放置必须与此前抓取使用同一只手；填写 "left" 或 "right"。
TEST_HAND = ...


def _validate_parameters() -> Tuple[list, str]:
    xyz = scene._vector(FAKE_BOX_QR_XYZ, 3, "FAKE_BOX_QR_XYZ")
    hand = str(TEST_HAND).strip().lower()
    if TEST_HAND is Ellipsis or hand not in ("left", "right"):
        raise scene.SMTSceneError('TEST_HAND 必须填写为 "left" 或 "right"')
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
        ("%s_READY_JOINTS_DEG" % hand.upper(), ready_joints, 14),
        ("%s_CARRY_JOINTS_DEG" % hand.upper(), carry_joints, 14),
        ("PLACE_CLEARANCE_XYZ", scene.PLACE_CLEARANCE_XYZ, 3),
        ("PLACE_OFFSET_XYZ", scene.PLACE_OFFSET_XYZ, 3),
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
        node_name="smt_place_sim",
        init_node=True,
    )
    return scene.SceneRuntime(
        io=robot_io,
        motion=None,
        qr=None,
        arm=scene.ArmController(robot_io, params=params),
        end_effector=scene.EndEffectorController(robot_io, config=params),
    )


def run_place(runtime: scene.SceneRuntime, qr_xyz: Sequence[float], hand: str) -> None:
    end_effector_type = "lejuclaw"
    gripper_settle_s = 0.5
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
        "仿真放置初始姿态",
    )
    scene._require_success(
        runtime.end_effector.close(hand, end_effector_type=end_effector_type),
        "%s夹爪闭合并模拟持物" % hand,
    )
    runtime.io.sleep(gripper_settle_s)
    current_joints = scene._move_arms(
        runtime,
        carry_joints,
        current_joints,
        "%s臂模拟持物姿态" % hand,
    )
    current_joints = scene._move_arms(
        runtime,
        ready_joints,
        current_joints,
        "%s臂箱前准备姿态" % hand,
    )

    place_stage1 = scene._add_xyz(fake_qr, scene.PLACE_CLEARANCE_XYZ)
    place_stage2 = scene._add_xyz(fake_qr, scene.PLACE_OFFSET_XYZ)
    first, second = scene._solve_two_stage(
        runtime,
        hand,
        place_stage1,
        place_stage2,
        ready_joints,
        "仿真放置",
    )
    current_joints = scene._execute_two_stage(
        runtime,
        first,
        second,
        current_joints,
        "仿真放置",
    )
    scene._require_success(
        runtime.end_effector.open(hand, end_effector_type=end_effector_type),
        "%s夹爪打开并模拟放料" % hand,
    )
    runtime.io.sleep(gripper_settle_s)
    runtime.io.loginfo(
        "放置仿真完成：hand=%s fake_qr=(%.4f, %.4f, %.4f)；"
        "因未执行底盘后退，机械臂保持第二段放置姿态",
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
        run_place(runtime, qr_xyz, hand)
        return 0
    except KeyboardInterrupt:
        if runtime is not None:
            runtime.io.logwarn("放置仿真被中断；未自动松爪或复位手臂")
        return 130
    except Exception as exc:
        if runtime is not None:
            runtime.io.logerr("放置仿真失败；未自动松爪或复位手臂: %s", exc)
        else:
            print("放置仿真启动失败: %s" % exc, file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
