# -*- coding: utf-8 -*-
"""NodeWaitForBlackboard:双源注入器(spec § 6)。

触发条件:
- 干跑(STUDIO_DRY_RUN=1):立即写 fake tag
- use_virtual_tag=true:写 fake tag（可配置为先等待一轮扫头）
- use_virtual_tag=false + 真 tag 已写:SUCCESS
- use_virtual_tag=false + 超时 + fallback_to_virtual=true:降级写 fake tag + SUCCESS
- use_virtual_tag=false + 超时 + fallback_to_virtual=false:不注入目标 + FAILURE
- require_fresh_tag=true:仅接受本次等待开始后采集的有效目标，拒绝旧缓存

fallback_to_virtual 默认 false：真机等不到真实 Tag 时返回 FAILURE，不朝虚拟位姿运动；
仿真需要超时降级时请在场景中显式设为 true。require_fresh_tag 默认 false。
检测时间戳使用 ROS 时钟（未启动 ROS 时使用系统时钟），等待超时使用单调时钟。

fake tag 公式:pose_in_world = virtual_tag_pose_in_odom - stand_in_tag_pos/euler
→ NodeComputePickGoal 算 walk_goal = virtual_tag_pose_in_odom
"""
import os
import math
import time
import py_trees
from py_trees.common import Status

from orchestration.nodes.base_node import BaseAction


def _is_dry_run() -> bool:
    return os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


def _tag_time() -> float:
    """与感知消息的 ROS 时间戳保持同一时钟，离线测试使用系统时间。"""
    try:
        import rospy
    except ImportError:
        return time.time()
    return rospy.get_time() if rospy.core.is_initialized() else time.time()


class NodeWaitForBlackboard(BaseAction):
    """双源 tag 注入器,封装"真 tag / 虚拟 tag"双链行为。"""

    def __init__(self, name, label, namespace, params):
        super(NodeWaitForBlackboard, self).__init__(name, label, namespace, params)
        self._tag_id = int(self.params.get("tag_id", 0))
        # 注册黑板 key 并初始化（py_trees 2.x 中 register_key 只注册权限，
        # 不会在 Blackboard.storage 中创建 key，导致后续 getattr 抛 KeyError）
        key = f"latest_tag_{self._tag_id}"
        self.global_blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)
        self.global_blackboard.register_key(
            key=f"{key}_xyz", access=py_trees.common.Access.WRITE
        )
        if self.params.get("wait_for_head_scan", False):
            self.global_blackboard.register_key(
                key="virtual_head_scan_done", access=py_trees.common.Access.READ
            )
        self._fallback_to_virtual = self.params.get("fallback_to_virtual", False)
        self._require_fresh_tag = self.params.get("require_fresh_tag", False)
        for name, value in (("fallback_to_virtual", self._fallback_to_virtual),
                            ("require_fresh_tag", self._require_fresh_tag)):
            if not isinstance(value, bool):
                raise ValueError(f"{name} 必须是 bool")
        if self._require_fresh_tag:
            self.global_blackboard.register_key(
                key=f"{key}_version", access=py_trees.common.Access.READ
            )
        self._start_time = 0.0
        self._tag_start_time = 0.0
        self._start_version = 0

    def initialise(self):
        self._start_time = time.monotonic()
        if self._require_fresh_tag and not _is_dry_run() and not self.params.get("use_virtual_tag", False):
            self._tag_start_time = _tag_time()
            version_key = f"latest_tag_{self._tag_id}_version"
            self._start_version = (
                self.global_blackboard.get(version_key)
                if self.global_blackboard.exists(version_key) else 0
            )
            # 清除下游旧 XYZ，但不清除检测对象，避免与并行感知写入互相覆盖。
            self.global_blackboard.unset(f"latest_tag_{self._tag_id}_xyz")

    def update(self):
        # 干跑不驱动头部，维持立即注入的静态检查语义。
        if _is_dry_run():
            self._inject_virtual_tag()
            self.feedback_message = "fake tag injected (dry-run)"
            return Status.SUCCESS

        if bool(self.params.get("use_virtual_tag", False)):
            if self.params.get("wait_for_head_scan", False) and not self.global_blackboard.get("virtual_head_scan_done"):
                self.feedback_message = "waiting for virtual head scan"
                return Status.RUNNING
            self._inject_virtual_tag()
            self.feedback_message = "fake tag injected (virtual mode)"
            return Status.SUCCESS

        # 链 3(真机):严格模式同时检查更新版本、采集时间和有效位姿。
        key = f"latest_tag_{self._tag_id}"
        tag = self.global_blackboard.get(key) if self.global_blackboard.exists(key) else None
        if tag is not None and (not self._require_fresh_tag or self._is_fresh_tag(tag)):
            self._sync_xyz()
            self.feedback_message = f"real tag {self._tag_id} found"
            return Status.SUCCESS

        timeout = float(self.params.get("real_tag_timeout_sec", 30.0))
        if time.monotonic() - self._start_time >= timeout:
            if not self._fallback_to_virtual:
                self.global_blackboard.unset(f"latest_tag_{self._tag_id}_xyz")
                self.feedback_message = f"real tag {self._tag_id} timeout; virtual fallback disabled"
                return Status.FAILURE
            self._inject_virtual_tag()
            self.feedback_message = "real tag timeout → fake tag injected"
            return Status.SUCCESS

        self.feedback_message = f"waiting for real tag {self._tag_id}..."
        return Status.RUNNING

    def _is_fresh_tag(self, tag):
        """版本更新不代表新图像：必须同时检查采集时间，拒绝旧帧反复写入。"""
        version_key = f"latest_tag_{self._tag_id}_version"
        version = self.global_blackboard.get(version_key) if self.global_blackboard.exists(version_key) else 0
        try:
            timestamp = float(tag.timestamp)
            pose = tag.pose_in_world
            xyz = (float(pose.x), float(pose.y), float(pose.z))
            return (
                version > self._start_version
                and tag.tag_id == self._tag_id
                and math.isfinite(timestamp)
                and timestamp > 0.0
                and self._tag_start_time <= timestamp <= _tag_time()
                and all(math.isfinite(value) for value in xyz)
            )
        except (AttributeError, TypeError, ValueError, OverflowError):
            return False

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
        self._sync_xyz()

    def _sync_xyz(self):
        """为只依赖 XYZ 的下游节点提供与完整 Tag 对象一致的位置。"""
        tag = getattr(self.global_blackboard, f"latest_tag_{self._tag_id}", None)
        if tag is None or tag.pose_in_world is None:
            raise ValueError(f"latest_tag_{self._tag_id} 未就绪")
        pose = tag.pose_in_world
        self.global_blackboard.set(
            f"latest_tag_{self._tag_id}_xyz", [pose.x, pose.y, pose.z]
        )
