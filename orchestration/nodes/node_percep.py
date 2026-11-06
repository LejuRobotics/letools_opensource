# -*- coding: utf-8 -*-
"""NodePercep:持续读 PerceptionAdapter,匹配 tag_ids → 写黑板 latest_tag_<id> + version。

每次检测到目标 tag,自增对应 version(用于其他节点判断"tag 更新过")。
"""
import os
import py_trees
from py_trees.common import Status

from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware


def _is_dry_run() -> bool:
    return os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


class NodePercep(BaseAction):
    """持续读 PerceptionAdapter,匹配 tag_ids → 写黑板。

    - dry-run:SUCCESS(不调硬件)
    - 进入节点 start()，离开节点 stop()；检测停止时保留相机。
    - 非 dry-run:每 tick 拉取一次 tag detections,匹配 → 写黑板 + version++ → RUNNING
    """

    def __init__(self, name, label, namespace, params):
        super(NodePercep, self).__init__(name, label, namespace, params)
        self._perception = None
        self._start_error = ""
        # 注册黑板 key(对每个 tag_id) + 初始化 version=0
        for tag_id in self.params.get("tag_ids", []):
            self.global_blackboard.register_key(
                key=f"latest_tag_{tag_id}", access=py_trees.common.Access.WRITE
            )
            self.global_blackboard.register_key(
                key=f"latest_tag_{tag_id}_version", access=py_trees.common.Access.WRITE
            )
            # 初始化 key 值，防止其他节点 getattr 时抛 KeyError（py_trees 2.x 中 register_key 只注册权限）
            self.global_blackboard.set(f"latest_tag_{tag_id}", None)
            # 初始 version=0,保证 getattr 不 KeyError
            setattr(self.global_blackboard, f"latest_tag_{tag_id}_version", 0)

    def initialise(self):
        self.feedback_message = "percep scanning"
        self._perception = None
        self._start_error = ""
        if _is_dry_run() or self.params.get("use_virtual_tag", False):
            return
        try:
            self._perception = get_shared_hardware().perception
            if self._perception is None:
                raise RuntimeError("PerceptionAdapter 未初始化，请配置 launch_apriltag: true")
            result = self._perception.start()
            if not result.success:
                self._start_error = result.message
        except Exception as exc:
            self._start_error = str(exc)
        if self._start_error:
            self.feedback_message = f"AprilTag 启动失败: {self._start_error}"

    def terminate(self, new_status):
        if self._perception is not None:
            result = self._perception.stop()
            if not result.success:
                self.feedback_message = f"AprilTag 停止失败: {result.message}"
            self._perception = None

    def update(self):
        if _is_dry_run():
            return Status.SUCCESS

        if self.params.get("use_virtual_tag", False):
            # 仿真仍扫头，但没有相机；由等待节点在扫头结束后注入 Tag。
            self.feedback_message = "virtual tag mode: camera skipped"
            return Status.RUNNING

        if self._start_error or self._perception is None:
            return Status.FAILURE
        detections = self._perception.get_tag_detections()
        target_ids = set(self.params.get("tag_ids", []))

        for det in detections:
            if det.tag_id in target_ids:
                setattr(self.global_blackboard, f"latest_tag_{det.tag_id}", det)
                # 自增 version(spec § 12 黑板契约)
                cur = getattr(self.global_blackboard, f"latest_tag_{det.tag_id}_version", 0)
                setattr(self.global_blackboard, f"latest_tag_{det.tag_id}_version", cur + 1)
                self.feedback_message = f"percep detected tag {det.tag_id}"
                # 持续 RUNNING(由 Parallel 父节点根据其他子节点状态决定何时终止)
                return Status.RUNNING

        return Status.RUNNING
