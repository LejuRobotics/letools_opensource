# -*- coding: utf-8 -*-
"""NodeWaitForBlackboard:双源注入器(spec § 6)。

触发条件:
- 干跑(STUDIO_DRY_RUN=1):立即写 fake tag
- use_virtual_tag=true:立即写 fake tag
- use_virtual_tag=false + 真 tag 已写:SUCCESS
- use_virtual_tag=false + 超时:降级写 fake tag + SUCCESS

fake tag 公式:pose_in_world = virtual_tag_pose_in_odom - stand_in_tag_pos/euler
→ NodeComputePickGoal 算 walk_goal = virtual_tag_pose_in_odom
"""
import os
import time
import py_trees
from py_trees.common import Status

from orchestration.nodes.base_node import BaseAction


def _is_dry_run() -> bool:
    return os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


class NodeWaitForBlackboard(BaseAction):
    """双源 tag 注入器,封装"真 tag / 虚拟 tag"双链行为。"""

    def __init__(self, name, label, namespace, params):
        super(NodeWaitForBlackboard, self).__init__(name, label, namespace, params)
        self._tag_id = int(self.params.get("tag_id", 0))
        # 注册黑板 key 并初始化（py_trees 2.x 中 register_key 只注册权限，
        # 不会在 Blackboard.storage 中创建 key，导致后续 getattr 抛 KeyError）
        key = f"latest_tag_{self._tag_id}"
        self.global_blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)
        self._start_time = 0.0

    def initialise(self):
        self._start_time = time.time()

    def update(self):
        # 链 1(干跑) / 链 2(use_virtual_tag=true):立即注入 fake tag
        if _is_dry_run() or bool(self.params.get("use_virtual_tag", False)):
            self._inject_virtual_tag()
            self.feedback_message = "fake tag injected (dry-run or virtual mode)"
            return Status.SUCCESS

        # 链 3(真机):先看真 tag
        if getattr(self.global_blackboard, f"latest_tag_{self._tag_id}", None) is not None:
            self.feedback_message = f"real tag {self._tag_id} found"
            return Status.SUCCESS

        # 超时降级
        timeout = float(self.params.get("real_tag_timeout_sec", 30.0))
        if time.time() - self._start_time > timeout:
            self._inject_virtual_tag()
            self.feedback_message = "real tag timeout → fake tag injected"
            return Status.SUCCESS

        self.feedback_message = f"waiting for real tag {self._tag_id}..."
        return Status.RUNNING

    def _inject_virtual_tag(self):
        """按 spec § 6.3 公式构造 fake tag。"""
        from core.domain.perception import TagDetection
        from core.domain.pose import Pose6D

        virtual = self.params.get("virtual_tag_pose_in_odom", [0.0] * 6)
        stand_pos = self.params.get("stand_in_tag_pos", [0.0, 0.0, 0.0])
        stand_euler = self.params.get("stand_in_tag_euler", [0.0, 0.0, 0.0])

        # virtual 格式 [x,y,z,roll,pitch,yaw] → 减 stand_in (pos+roll/pitch/yaw)
        pose = Pose6D(
            x=virtual[0] - stand_pos[0],
            y=virtual[1] - stand_pos[1],
            z=virtual[2] - stand_pos[2],
            roll=virtual[3] - stand_euler[0],
            pitch=virtual[4] - stand_euler[1],
            yaw=virtual[5] - stand_euler[2],
        )
        fake = TagDetection(
            tag_id=self._tag_id,
            pose_in_world=pose,
            size=0.1,
            confidence=1.0,
            timestamp=time.time(),
            frame_id="odom",
        )
        setattr(self.global_blackboard, f"latest_tag_{self._tag_id}", fake)
