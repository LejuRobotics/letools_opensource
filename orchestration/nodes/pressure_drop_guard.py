# -*- coding: utf-8 -*-
"""PressureDropGuard：气压掉落检测装饰器 → pressure_drop_detection 原子技能。"""

from typing import Iterator

from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.decorators import Decorator

from orchestration.shared_hardware import get_shared_hardware
from skills.atomic.refactored_sdk.pressure_drop_detection import (
    PressureDropDetectionParams,
    PressureDropDetectionSkill,
)


class PressureDropGuard(Decorator):
    """气压掉落检测装饰器。

    直接持有 PressureDropDetectionParams，不再逐字段硬编码默认值。
    参数解析统一由 PressureDropDetectionParams.from_node_params 处理。
    """

    def __init__(self, name, child, params: PressureDropDetectionParams = None):
        super().__init__(name=name, child=child)
        self._params = params or PressureDropDetectionParams()
        self._skill: PressureDropDetectionSkill = None

    def initialise(self):
        self._skill = PressureDropDetectionSkill(hardware=get_shared_hardware())
        self._skill.initialize(self._params)

    def tick(self) -> Iterator[Behaviour]:
        """每次 tick 前通过 Skill 检查气压，掉落时中断子节点。"""
        if self.status != Status.RUNNING:
            self.initialise()

        # 调用 Skill 执行气压检测
        result = self._skill.execute()
        if not result.success:
            # 掉落警报：中断子节点执行
            self.status = Status.FAILURE
            self.feedback_message = result.message or "气压掉落警报，中断执行"
            yield self
            return

        # 掉落已处理（用户已按 Enter）：重启子节点以恢复导航
        # enable_vel_control 中断了导航任务，需要重新 initialise() 发送新导航目标
        if self._skill.is_finished():
            self.decorated.stop(Status.INVALID)
            self._skill.reset_after_alarm()  # 不重新检测，但允许 is_finished() 返回 False
            self.feedback_message = "掉落已处理，重启子节点"
            # 给导航模块恢复时间（enable_vel_control(False) 刚执行完）
            try:
                import rospy
                rospy.sleep(1.0)
            except Exception:
                pass

        # 气压正常：放行子节点 tick（若子节点刚被 stop(INVALID) 则会重新 initialise）
        for node in self.decorated.tick():
            yield node

        new_status = self.decorated.status
        if new_status != Status.RUNNING:
            self.stop(new_status)
        self.status = new_status
        yield self

    def update(self):
        return self.decorated.status
