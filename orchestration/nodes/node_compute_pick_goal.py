# -*- coding: utf-8 -*-
"""NodeComputePickGoal:算底盘 ODOM 系目标位姿(spec § 3.1)。

输入:黑板 latest_tag_{id}(ODOM 系 TAG→world 变换,来自 PerceptionAdapter 订阅 _odom)
      + stand_in_tag_pos/euler(TAG 系相对偏移)
输出:黑板 walk_goal(ODOM 系底盘目标)+ is_walk_goal_new=True

公式:walk_goal = tag.pose_in_world.(x,y,z) + stand_in_tag_pos
"""
import os
import py_trees
from py_trees.common import Status

from orchestration.nodes.base_node import BaseAction


def _is_dry_run() -> bool:
    return os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


class NodeComputePickGoal(BaseAction):
    """读 tag,算 walk_goal,写黑板。"""

    def __init__(self, name, label, namespace, params):
        super(NodeComputePickGoal, self).__init__(name, label, namespace, params)
        self._tag_id = int(self.params.get("tag_id", 0))
        # 注册需要读取的 tag key，避免 py_trees 2.x 中 KeyError
        self.global_blackboard.register_key(
            key=f"latest_tag_{self._tag_id}", access=py_trees.common.Access.READ
        )
        self.global_blackboard.register_key(
            key="walk_goal", access=py_trees.common.Access.WRITE
        )
        self.global_blackboard.register_key(
            key="is_walk_goal_new", access=py_trees.common.Access.WRITE
        )

    def initialise(self):
        pass

    def update(self):
        if _is_dry_run():
            self._write_walk_goal((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
            return Status.SUCCESS

        tag = getattr(self.global_blackboard, f"latest_tag_{self._tag_id}", None)
        if tag is None:
            self.feedback_message = f"tag {self._tag_id} not on blackboard"
            return Status.RUNNING

        stand_pos = self.params.get("stand_in_tag_pos", [0.0, 0.0, 0.0])
        stand_euler = self.params.get("stand_in_tag_euler", [0.0, 0.0, 0.0])

        #   1) stand_pose_in_tag = (stand_pos, stand_euler)（TAG 系，board 默认值 [-90°,90°,0]）
        #   2) stand_pose_in_world = tag_odom_matrix × stand_in_tag_matrix（CalcMoveDest 的 transform_pose_from_tag_to_world）
        import math
        import numpy as np
        from scipy.spatial.transform import Rotation as R

        def _mat(pos, euler_rad):
            m = np.eye(4)
            m[:3, 3] = pos
            m[:3, :3] = R.from_euler("xyz", euler_rad).as_matrix()
            return m

        tag_t = tag.pose_in_world
        T_tag = _mat(
            [tag_t.x, tag_t.y, tag_t.z],
            [math.pi / 2, 0.0, tag_t.yaw],          # ← embodied fix roll
        )
        T_stand = _mat(
            [stand_pos[0], stand_pos[1], stand_pos[2]],
            [stand_euler[0], stand_euler[1], stand_euler[2] if len(stand_euler) > 2 else 0.0],
        )
        T_world = T_tag @ T_stand

        new_pos = (float(T_world[0, 3]), float(T_world[1, 3]), float(T_world[2, 3]))
        face_yaw = float(np.arctan2(T_world[1, 0], T_world[0, 0]))

        self._write_walk_goal(new_pos, (0.0, 0.0, face_yaw))
        print(f"[NodeComputePickGoal] tag_odom=({tag_t.x:.3f},{tag_t.y:.3f},{tag_t.z:.3f},yaw={tag_t.yaw:.3f}) "
              f"walk_goal=({new_pos[0]:.3f},{new_pos[1]:.3f}) yaw={face_yaw:.3f}rad ({math.degrees(face_yaw):.1f}°)")
        return Status.SUCCESS

    def _write_walk_goal(self, pos, euler):
        from kuavo_humanoid_sdk.kuavo_strategy_pytree.common.data_type import Pose
        setattr(
            self.global_blackboard, "walk_goal",
            Pose.from_euler(pos=pos, euler=euler, frame="odom", degrees=False),
        )
        setattr(self.global_blackboard, "is_walk_goal_new", True)
        self.feedback_message = f"walk_goal={pos}"
