# -*- coding: utf-8 -*-
"""GraspRing Skill：动态抓取圆环。

从 /mujoco/ring_01/pose 获取圆环位置，用 FK 计算抓取位姿，
执行：预抓位 → 下抓位 → 闭合夹爪 → 抬起 的完整序列。

对齐 grasp_ring_letools.py 的核心逻辑。
"""

import math
import time
from dataclasses import dataclass, field
from typing import List, Optional

from core.common.logger import get_logger
from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.interfaces.i_hardware import IHardware
from skills.base.skill_base import SkillBase

logger = get_logger(__name__)


@dataclass
class GraspRingParams(SkillParams):
    """抓取圆环参数。"""
    skill_name: str = "grasp_ring"
    q_pre: List[float] = field(default_factory=lambda: [
        40, 0, 0, -120, -90, -40, 0,
        50, 0, 0, -140, 90, 40, 0,
    ])
    grasp_offset_z: float = 0.05
    pre_grasp_offset_z: float = 0.15
    ee_to_tip_z: float = -0.1
    ring_name: str = "ring_01"
    gripper_pre_position: float = 50.0
    gripper_close_position: float = 0.0
    timeout: float = 60.0


class GraspRingSkill(SkillBase):
    """抓取圆环 Skill：读圆环位置 → FK → 预抓 → 下抓 → 闭合 → 抬起。"""

    def __init__(self, hardware: IHardware):
        super().__init__(name="grasp_ring")
        self.hardware = hardware
        self.params: Optional[GraspRingParams] = None
        self._phase = 0
        self._success = False
        self._ring_pos = None
        self._pre_grasp_pose = None
        self._grasp_pose = None
        self._lift_pose = None

    def on_initialize(self, params: GraspRingParams) -> Result:
        if not isinstance(params, GraspRingParams):
            return Result.fail("Invalid parameters for GraspRingSkill")
        self.params = params
        self._phase = 0
        self._success = False

        # 半开夹爪
        self._send_claw(params.gripper_pre_position, params.gripper_pre_position)
        return Result.ok()

    def on_execute(self) -> Result:
        p = self.params

        # Phase 0: 获取圆环位置 + FK 计算位姿
        if self._phase == 0:
            self._ring_pos = self._get_ring_position(p.ring_name)
            if self._ring_pos is None:
                return Result.fail(f"无法获取 {p.ring_name} 的位置")

            from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot
            if not KuavoSDK().Init(options=KuavoSDK.Options.WithIK):
                return Result.fail("KuavoSDK Init failed")

            robot = KuavoRobot()
            q_pre_rad = [math.radians(x) for x in p.q_pre]
            _, r_pose = robot.arm_fk(q_pre_rad)

            from scipy.spatial.transform import Rotation as R
            rot_ee = R.from_quat(r_pose.orientation)
            rpy = rot_ee.as_euler('ZYX', degrees=True)
            ee_yaw, ee_pitch, ee_roll = rpy[0], rpy[1], rpy[2]

            ee_to_tip_world = rot_ee.apply([0.0, 0.0, p.ee_to_tip_z])
            ee_to_tip_z_world = ee_to_tip_world[2]

            tip_target = [self._ring_pos[0] + 0.02, self._ring_pos[1],
                          self._ring_pos[2] + p.grasp_offset_z]
            grasp_target = [tip_target[0], tip_target[1],
                             tip_target[2] - ee_to_tip_z_world]
            pre_grasp_target = [grasp_target[0], grasp_target[1],
                                grasp_target[2] + p.pre_grasp_offset_z]

            self._pre_grasp_pose = [*pre_grasp_target, ee_yaw, ee_pitch, ee_roll]
            self._grasp_pose = [*grasp_target, ee_yaw, ee_pitch, ee_roll]
            self._lift_pose = [self._pre_grasp_pose[0], self._pre_grasp_pose[1],
                               self._pre_grasp_pose[2] + 0.10,
                               ee_yaw, ee_pitch, ee_roll]

            print(f"[GraspRingSkill] 圆环位置: {self._ring_pos}")
            print(f"[GraspRingSkill] 预抓位: {self._pre_grasp_pose}")
            print(f"[GraspRingSkill] 抓取位: {self._grasp_pose}")
            self._phase = 1
            return Result.ok("phase 0 done")

        # Phase 1: 运动到预抓位
        if self._phase == 1:
            print("[GraspRingSkill] 右手运动到圆环正上方")
            result = self.hardware.send_right_arm_ee_world_timed(
                pose=self._pre_grasp_pose, desire_time=3.0
            )
            if not result.success:
                return Result.fail(f"运动到预抓位失败: {result.message}")
            time.sleep(4.0)
            self._phase = 2
            return Result.ok("phase 1 done")

        # Phase 2: 垂直下抓
        if self._phase == 2:
            print("[GraspRingSkill] 垂直向下抓取")
            result = self.hardware.send_right_arm_ee_world_timed(
                pose=self._grasp_pose, desire_time=2.0
            )
            if not result.success:
                return Result.fail(f"下抓失败: {result.message}")
            time.sleep(3.0)
            self._phase = 3
            return Result.ok("phase 2 done")

        # Phase 3: 闭合夹爪
        if self._phase == 3:
            print("[GraspRingSkill] 关闭右手夹爪")
            self._send_claw(p.gripper_pre_position, p.gripper_close_position)
            time.sleep(1.0)
            self._phase = 4
            return Result.ok("phase 3 done")

        # Phase 4: 抬起10cm
        if self._phase == 4:
            print("[GraspRingSkill] 右臂抬高10cm")
            result = self.hardware.send_right_arm_ee_world_timed(
                pose=self._lift_pose, desire_time=2.0
            )
            if not result.success:
                print(f"[GraspRingSkill] 右臂抬高失败: {result.message}")
            time.sleep(3.0)
            self._phase = 5
            return Result.ok("phase 4 done")

        # Phase 5: 完成
        if self._phase == 5:
            print("[GraspRingSkill] 抓取完成")
            self._success = True
            return Result.ok("done")

        return Result.fail("unknown phase")

    def on_is_finished(self) -> bool:
        return self._phase >= 5 and self._success

    def on_cancel(self) -> Result:
        self._phase = 5
        return Result.ok("cancelled")

    def _get_ring_position(self, ring_name):
        import rospy
        from geometry_msgs.msg import PoseStamped
        topic = f"/mujoco/{ring_name}/pose"
        try:
            msg = rospy.wait_for_message(topic, PoseStamped, timeout=5)
            return (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        except Exception as e:
            print(f"[GraspRingSkill] 获取 {ring_name} 位置失败: {e}")
            return None

    def _send_claw(self, left_pos, right_pos):
        import rospy
        from kuavo_msgs.msg import lejuClawCommand, endEffectorData
        pub = rospy.Publisher('/leju_claw_command', lejuClawCommand, queue_size=1)
        msg = lejuClawCommand()
        msg.data = endEffectorData()
        msg.data.name = ['left_claw', 'right_claw']
        msg.data.position = [float(left_pos), float(right_pos)]
        msg.data.velocity = [50.0, 50.0]
        msg.data.effort = [1.0, 1.0]
        pub.publish(msg)
