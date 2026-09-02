# -*- coding: utf-8 -*-
"""PlaceRing Skill：放置圆环。

将手中的圆环放到放置台上：运动到放置位 → 张开夹爪。
使用局部坐标系（local），不受导航旋转影响。
"""

import time
from dataclasses import dataclass
from typing import Optional

from core.common.logger import get_logger
from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.interfaces.i_hardware import IHardware
from skills.base.skill_base import SkillBase

logger = get_logger(__name__)


@dataclass
class PlaceRingParams(SkillParams):
    """放置圆环参数。"""
    skill_name: str = "place_ring"
    place_target_x: float = 0.7
    place_target_y: float = -0.3
    place_target_z: float = 1.05
    gripper_pre_position: float = 50.0
    timeout: float = 60.0


class PlaceRingSkill(SkillBase):
    """放置圆环 Skill：运动到放置位 → 张开夹爪。"""

    def __init__(self, hardware: IHardware):
        super().__init__(name="place_ring")
        self.hardware = hardware
        self.params: Optional[PlaceRingParams] = None
        self._phase = 0
        self._success = False
        self._pre_place_pose = None

    def on_initialize(self, params: PlaceRingParams) -> Result:
        if not isinstance(params, PlaceRingParams):
            return Result.fail("Invalid parameters for PlaceRingSkill")
        self.params = params
        self._phase = 0
        self._success = False
        return Result.ok()

    def on_execute(self) -> Result:
        p = self.params

        # Phase 0: 计算放置位姿
        if self._phase == 0:
            yaw, pitch, roll = 0.0, 90.0, 0.0
            self._pre_place_pose = [p.place_target_x, p.place_target_y,
                                    p.place_target_z + 0.15,
                                    yaw, pitch, roll]
            print(f"[PlaceRingSkill] 预放位(local): {self._pre_place_pose}")
            self._phase = 1
            return Result.ok("phase 0 done")

        # Phase 1: 运动到预放位（局部坐标系）
        if self._phase == 1:
            print("[PlaceRingSkill] 右手运动到放置位")
            result = self.hardware.send_right_arm_ee_local_timed(
                pose=self._pre_place_pose, desire_time=3.0
            )
            if not result.success:
                return Result.fail(f"运动到放置位失败: {result.message}")
            time.sleep(4.0)
            self._phase = 3
            return Result.ok("phase 1 done")

        # Phase 3: 张开夹爪
        if self._phase == 3:
            print("[PlaceRingSkill] 张开右手夹爪")
            self._open_gripper()
            self._phase = 4
            return Result.ok("phase 3 done")

        # Phase 4: 完成
        if self._phase == 4:
            print("[PlaceRingSkill] 放置完成")
            self._success = True
            return Result.ok("done")

        return Result.fail("unknown phase")

    def on_is_finished(self) -> bool:
        return self._phase >= 4 and self._success

    def on_cancel(self) -> Result:
        self._phase = 4
        return Result.ok("cancelled")

    def _open_gripper(self):
        """张开夹爪（ROS topic 持续发布，与 grasp_ring_letools.py 同方式）。"""
        import rospy
        from kuavo_msgs.msg import lejuClawCommand, endEffectorData

        pub = rospy.Publisher('/leju_claw_command', lejuClawCommand, queue_size=1)
        msg = lejuClawCommand()
        msg.data = endEffectorData()
        msg.data.name = ['left_claw', 'right_claw']
        msg.data.position = [float(self.params.gripper_pre_position),
                             float(self.params.gripper_pre_position)]
        msg.data.velocity = [50.0, 50.0]
        msg.data.effort = [1.0, 1.0]
        for _ in range(10):
            pub.publish(msg)
            rospy.sleep(0.2)
        print(f"[PlaceRingSkill] 夹爪已张开: pos={self.params.gripper_pre_position}")
