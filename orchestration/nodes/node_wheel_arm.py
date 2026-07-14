# -*- coding: utf-8 -*-
"""NodeWheelArm:读黑板关节/eef 轨迹 → 走手臂。"""
import os
import py_trees
from py_trees.common import Status

from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware


def _is_dry_run() -> bool:
    return os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


class NodeWheelArm(BaseAction):
    """读 traj → 调 arm_joint_traj / arm_eef_traj。"""

    def __init__(self, name, label, namespace, params):
        super(NodeWheelArm, self).__init__(name, label, namespace, params)
        self._control_type = self.params.get("control_type", "joint")
        self._executed = False
        for k in [
            "left_arm_joint_traj", "right_arm_joint_traj",
            "left_arm_eef_traj", "right_arm_eef_traj",
        ]:
            self.global_blackboard.register_key(key=k, access=py_trees.common.Access.READ)

    def initialise(self):
        self._executed = False

    def _read_traj_pair(self, traj_key: str):
        try:
            l = getattr(self.global_blackboard, f"left_{traj_key}", None)
            r = getattr(self.global_blackboard, f"right_{traj_key}", None)
        except KeyError:
            return None, None
        return l, r

    def update(self):
        if _is_dry_run():
            return Status.SUCCESS

        if self._executed:
            return Status.SUCCESS

        hw = get_shared_hardware()
        try:
            if self._control_type == "joint":
                l, r = self._read_traj_pair("arm_joint_traj")
            else:
                l, r = self._read_traj_pair("arm_eef_traj")

            if l is None and r is None:
                return Status.RUNNING

            if not l or not r:
                self.feedback_message = "arm traj empty, skip"
                self._executed = True
                return Status.SUCCESS

            if self._control_type == "joint":
                hw.send_arm_joint_trajectory(list(l) + list(r))
            else:
                self._send_eef_via_sdk(hw, l, r)

            self._executed = True
            return Status.SUCCESS

        except Exception as e:
            self.feedback_message = f"wheel_arm failed: {e}"
            return Status.FAILURE

    def _send_eef_via_sdk(self, hw, left_poses, right_poses):
        fn = getattr(hw, "send_arm_ee_traj_sdk", None)
        if fn is None:
            raise RuntimeError("Hardware 不支持 send_arm_ee_traj_sdk()")

        left_traj = [_pose6d_to_quat_point(p) for p in left_poses]
        right_traj = [_pose6d_to_quat_point(p) for p in right_poses]
        if len(left_traj) < 2:
            left_traj.append(left_traj[-1])
            right_traj.append(right_traj[-1])

        result = fn(
            left_traj=left_traj, right_traj=right_traj,
            total_time=2.0, frame="base_link",
        )
        if not result.success:
            raise RuntimeError(result.message or "send_arm_ee_traj_sdk failed")


def _pose6d_to_quat_point(pose):
    qx, qy, qz, qw = pose.to_quaternion()
    return [pose.x, pose.y, pose.z, qx, qy, qz, qw]
