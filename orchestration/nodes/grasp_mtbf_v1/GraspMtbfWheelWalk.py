# -*- coding: utf-8 -*-
"""GraspMtbfWheelWalk:读黑板 walk_goal → 走底盘(cmd_pos_world/cmd_pos/cmd_vel)。"""
import os
import math
import py_trees
from py_trees.common import Status

from orchestration.nodes.base_node import BaseAction
from .scene_io import get_scene_io


def _is_dry_run() -> bool:
    return os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


class GraspMtbfWheelWalk(BaseAction):
    """读 walk_goal → 调 chassis API。"""

    def __init__(self, name, label, namespace, params):
        super(GraspMtbfWheelWalk, self).__init__(name, label, namespace, params)
        self._walk_mode = self.params.get("walk_mode", "cmd_pos_world")
        # READ walk_goal; READ+WRITE is_walk_goal_new(消费后清零)
        self.global_blackboard.register_key(key="walk_goal", access=py_trees.common.Access.READ)
        self.global_blackboard.register_key(key="is_walk_goal_new", access=py_trees.common.Access.WRITE)

    def initialise(self):
        pass

    def update(self):
        if _is_dry_run():
            self.feedback_message = f"dry-run wheel_walk mode={self._walk_mode}"
            return Status.SUCCESS

        is_new = getattr(self.global_blackboard, "is_walk_goal_new", False)
        if not is_new:
            return Status.RUNNING

        goal = getattr(self.global_blackboard, "walk_goal", None)
        if goal is None:
            return Status.RUNNING

        hw = get_scene_io()
        try:
            # 降低底盘规划器速度，避免猛冲
            config_result = hw.set_ruckig_planner_params(
                planner_index=0, is_sync=False,
                velocity_max=[0.15, 0.15, 0.3],
                acceleration_max=[0.1, 0.1, 0.2],
                jerk_max=[1.0, 1.0, 2.0],
            )
            if config_result is None or not config_result.success:
                raise RuntimeError(f"底盘规划器限速设置失败: {getattr(config_result, 'message', '缺少响应')}")
            # goal 为 Pose(pos=[x,y,z], quat)，get_euler() 返回 [roll, pitch, yaw]（弧度）
            yaw = float(goal.get_euler()[2])
            if not all(math.isfinite(v) for v in (float(goal.pos[0]), float(goal.pos[1]), yaw)):
                raise ValueError("底盘目标包含非有限数")
            if self._walk_mode == "cmd_pos_world":
                result = hw.send_world_position(float(goal.pos[0]), float(goal.pos[1]), yaw)
            elif self._walk_mode == "cmd_pos":
                result = hw.send_base_position(float(goal.pos[0]), float(goal.pos[1]), yaw)
            elif self._walk_mode == "cmd_vel":
                result = hw.send_base_velocity(float(goal.pos[0]), float(goal.pos[1]), yaw)
            else:
                raise ValueError(f"不支持的 walk_mode: {self._walk_mode}")
            if result is None or not result.success:
                raise RuntimeError(f"底盘指令被拒绝: {getattr(result, 'message', '缺少响应')}")
            # SUCCESS means accepted; the scenario retains an arrival confirmation.
            setattr(self.global_blackboard, "is_walk_goal_new", False)
            return Status.SUCCESS
        except Exception as e:
            self.feedback_message = f"wheel_walk failed: {e}"
            return Status.FAILURE
