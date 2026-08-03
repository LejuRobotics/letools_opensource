# -*- coding: utf-8 -*-
"""CalcLegMove：根据 TargetTag 计算并控制躯干/腿姿态（蹲、抬、恢复）。

- leg_mode="target"：stand_in_tag 位姿 + offset → tag 系 → odom → base_link，再下发躯干
- leg_mode 其他值（搬箱树里用 "leg"）：直接用 offset_x/y/z + offset_yaw 作为 base 系躯干目标
- 走 LeTools 标准接口：set_mpc_mode(ARM_ONLY) + send_torso_pose
"""

import ast
import math
import os

import numpy as np
import py_trees
from py_trees.common import Status

from core.common.transform import matrix_to_pose6d, pose6d_to_matrix, transform_pose
from core.domain.enums import MPCControlMode
from core.domain.pose import Pose6D
from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


def _parse_list(raw, default):
    if raw is None:
        return list(default)
    if isinstance(raw, (list, tuple)):
        return [float(x) for x in raw]
    try:
        return [float(x) for x in ast.literal_eval(str(raw))]
    except Exception:
        return list(default)


def _base_from_odom(pose_odom: Pose6D):
    """将 odom 系位姿换算到 base_link 系（经 tf2）。失败返回 None。"""
    try:
        import rospy
        import tf2_ros
        import tf2_geometry_msgs  # noqa: F401
        from geometry_msgs.msg import PoseStamped
        from core.common.math_utils import quaternion_to_euler

        if not hasattr(_base_from_odom, "_buffer"):
            _base_from_odom._buffer = tf2_ros.Buffer()
            _base_from_odom._listener = tf2_ros.TransformListener(_base_from_odom._buffer)

        ps = PoseStamped()
        ps.header.frame_id = "odom"
        ps.header.stamp = rospy.Time(0)
        ps.pose.position.x = pose_odom.x
        ps.pose.position.y = pose_odom.y
        ps.pose.position.z = pose_odom.z
        qx, qy, qz, qw = pose_odom.to_quaternion()
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw

        transform = _base_from_odom._buffer.lookup_transform(
            "base_link", "odom", rospy.Time(0), rospy.Duration(0.5))
        out = tf2_geometry_msgs.do_transform_pose(ps, transform)
        roll, pitch, yaw = quaternion_to_euler(
            out.pose.orientation.x, out.pose.orientation.y,
            out.pose.orientation.z, out.pose.orientation.w)
        return Pose6D(x=out.pose.position.x, y=out.pose.position.y, z=out.pose.position.z,
                      roll=roll, pitch=pitch, yaw=yaw)
    except Exception:
        return None


@define_manifest(
    label="计算并控制躯干/腿",
    category=["motion", "torso"],
    tree_type="grasp_mtbf",
    description="根据 TargetTag 计算躯干目标（或直接用 offset），下发躯干位姿控制（蹲/抬/恢复）",
    params=[
        {"name": "stand_in_tag_pos", "type": "string", "default": "[-0.04, 0.15, 0.37]", "description": "站立位置在Tag坐标系下的位置 [x,y,z]（米）"},
        {"name": "stand_in_tag_euler", "type": "string", "default": "[-1.57, 1.57, 0.0]", "description": "站立姿态在Tag坐标系下的欧拉角 [r,p,y]（弧度）"},
        {"name": "leg_mode", "type": "string", "default": "target", "description": "腿部控制模式", "options": ["target", "leg"]},
        {"name": "tag_id", "type": "int", "default": "0", "description": "leg_mode=target 时读 latest_tag_<tag_id>（NodePercep 输出）"},
        {"name": "offset_x", "type": "float", "default": "0.0", "description": "X方向偏移（米）"},
        {"name": "offset_y", "type": "float", "default": "0.0", "description": "Y方向偏移（米）"},
        {"name": "offset_z", "type": "float", "default": "0.0", "description": "Z方向偏移（米）"},
        {"name": "offset_yaw", "type": "float", "default": "0.0", "description": "偏航角偏移（度）"},
        {"name": "offset_pitch", "type": "float", "default": "0.0", "description": "俯仰角偏移（度），正值前倾"},
        {"name": "control_base", "type": "bool", "default": "false", "description": "是否控制底盘（保留，暂不使用）"},
        {"name": "total_time", "type": "float", "default": "2.0", "description": "躯干执行时长(秒)，调大降速"},
    ],
    inputs=[
        {"name": "target_tag", "type": "object", "required": False, "default_key": "TargetTag", "description": "目标 Tag（leg_mode=target 时需要）"},
    ],
    outputs=[],
)
class CalcLegMove(BaseAction):
    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._done = False
        self._success = False

    def initialise(self):
        self._done = False
        self._success = False

        if _DRY_RUN:
            self._done = True
            self._success = True
            return

        hw = get_shared_hardware()
        hw.set_mpc_mode(MPCControlMode.ARM_ONLY)

        offset_x = float(self.params.get("offset_x", 0.0))
        offset_y = float(self.params.get("offset_y", 0.0))
        offset_z = float(self.params.get("offset_z", 0.0))
        offset_yaw_deg = float(self.params.get("offset_yaw", 0.0))
        offset_pitch_deg = float(self.params.get("offset_pitch", 0.0))
        leg_mode = str(self.params.get("leg_mode", "target"))

        # leg_mode=target 需要从 latest_tag_<tag_id> 读 tag（NodePercep 写入）
        self._tag_id = int(self.params.get("tag_id", 0) or 0)
        if leg_mode == "target" and self._tag_id > -1:
            try:
                self.global_blackboard.register_key(
                    key=f"latest_tag_{self._tag_id}", access=py_trees.common.Access.READ)
            except Exception:
                pass

        target_torso_base = None

        if leg_mode == "target":
            tag = None
            if self._tag_id > -1:
                tag = getattr(self.global_blackboard, f"latest_tag_{self._tag_id}", None)
            if tag is None or getattr(tag, "pose_in_world", None) is None:
                self.feedback_message = f"latest_tag_{self._tag_id} 未就绪"
                self._done = True
                return

            stand_pos = _parse_list(self.params.get("stand_in_tag_pos"), [0.0, 0.1, 0.2])
            stand_euler = _parse_list(self.params.get("stand_in_tag_euler"), [-1.57, 1.57, 0.0])
            stand_pos = [stand_pos[0] + offset_x, stand_pos[1] + offset_y, stand_pos[2] + offset_z]

            # stand_in_tag × tag_odom → stand_in_odom → base_link
            t = tag.pose_in_world
            tag_fixed = Pose6D(x=t.x, y=t.y, z=t.z, roll=math.pi / 2, pitch=0.0, yaw=t.yaw)
            stand_in_tag = Pose6D(x=stand_pos[0], y=stand_pos[1], z=stand_pos[2],
                                  roll=stand_euler[0], pitch=stand_euler[1], yaw=stand_euler[2])
            stand_in_odom = transform_pose(stand_in_tag, pose6d_to_matrix(tag_fixed))
            target_torso_base = _base_from_odom(stand_in_odom)
            if target_torso_base is None:
                self.feedback_message = "odom→base_link 变换失败"
                self._done = True
                return

            if offset_pitch_deg != 0.0 or offset_yaw_deg != 0.0:
                target_torso_base = Pose6D(
                    x=target_torso_base.x, y=target_torso_base.y, z=target_torso_base.z,
                    roll=target_torso_base.roll,
                    pitch=target_torso_base.pitch + math.radians(offset_pitch_deg),
                    yaw=target_torso_base.yaw,
                )
        else:
            # 搬箱树里的 "leg" 模式：直接用 offset 作为 base 系躯干目标
            target_torso_base = Pose6D(
                x=offset_x, y=offset_y, z=offset_z,
                roll=0.0,
                pitch=math.radians(offset_pitch_deg),
                yaw=math.radians(offset_yaw_deg),
            )

        # 用 TimedCmd 路径控制躯干（planner 2，可指定执行时长 desire_time 降速）；
        # send_torso_pose 是瞬时单点话题，无速度控制。
        total_time = float(self.params.get("total_time", 3.0))
        result = hw.send_torso_pose_timed(
            x=target_torso_base.x,
            z=target_torso_base.z,
            yaw=math.degrees(target_torso_base.yaw),
            pitch=math.degrees(target_torso_base.pitch),
            desire_time=total_time,
        )
        if result.success:
            self.feedback_message = (
                f"躯干目标已下发(TimedCmd {total_time}s): x={target_torso_base.x:.3f}, z={target_torso_base.z:.3f}, "
                f"pitch={math.degrees(target_torso_base.pitch):.1f}°, yaw={math.degrees(target_torso_base.yaw):.1f}°"
            )
            self._success = True
        else:
            self.feedback_message = f"躯干控制失败: {result.message}"
        self._done = True

    def update(self):
        if not self._done:
            return Status.RUNNING
        return Status.SUCCESS if self._success else Status.FAILURE
