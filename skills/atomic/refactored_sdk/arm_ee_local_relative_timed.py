# -*- coding: utf-8 -*-
"""Atomic skill: arm_ee_local_relative_timed.

读取 /humanoid_wheel/eePoses 当前末端 odom 位姿，经 TF 转成 base_link/local 位姿，
在 x/y/z 上叠加 offset，姿态使用固定欧拉角，然后通过 send_arm_ee_local_timed
一次性下发双臂目标位姿。

对齐 apps/test_kuavo_5w_sdk_adapter/timed/04_arm/test_arm_ee_local_relative_servo.py

安全/运行参数（与示例脚本一致，一般不用改）：
- desire_time       = 0.6
- wait_feedback      = 2.0
- tf_wait            = 0.5
- tf_timeout         = 0.5
- ee_poses_topic     = /humanoid_wheel/eePoses
- ee_poses_source_frame = odom
- target_local_frame    = base_link
- release_after      = True
"""

from dataclasses import dataclass
from typing import List, Optional, Tuple
import math
import time

from core.common.logger import get_logger
from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.interfaces.i_hardware import IHardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.base.skill_base import SkillBase

logger = get_logger(__name__)

Pose = List[float]
_raw_ee_poses: Optional[List[float]] = None
_subscribed = False

# ── 与示例脚本一致的安全/运行参数（固定，不暴露给黑板） ──
DESIRE_TIME = 2.0
WAIT_FEEDBACK = 2.0
TF_WAIT = 0.5
TF_TIMEOUT = 0.5
EE_POSES_TOPIC = "/humanoid_wheel/eePoses"
EE_POSES_SOURCE_FRAME = "odom"
TARGET_LOCAL_FRAME = "base_link"
RELEASE_AFTER = True


def _normalize_deg(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


def _raw_callback(msg):
    global _raw_ee_poses
    _raw_ee_poses = list(msg.data)


def _ensure_subscribed():
    global _subscribed
    if _subscribed:
        return
    import rospy
    from std_msgs.msg import Float64MultiArray

    rospy.Subscriber(EE_POSES_TOPIC, Float64MultiArray, _raw_callback)
    _subscribed = True
    logger.info("arm_ee_local_relative_timed: 已订阅 %s", EE_POSES_TOPIC)


def _make_tf_listener():
    import tf

    listener = tf.TransformListener()
    time.sleep(TF_WAIT)
    return listener


def _raw_pose_to_local(listener, raw_pose: List[float]) -> Pose:
    import rospy
    import tf
    from geometry_msgs.msg import PoseStamped

    yaw = float(raw_pose[3])
    pitch = float(raw_pose[4])
    roll = float(raw_pose[5])

    msg = PoseStamped()
    msg.header.frame_id = EE_POSES_SOURCE_FRAME
    msg.header.stamp = rospy.Time(0)
    msg.pose.position.x = float(raw_pose[0])
    msg.pose.position.y = float(raw_pose[1])
    msg.pose.position.z = float(raw_pose[2])

    qx, qy, qz, qw = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    msg.pose.orientation.x = qx
    msg.pose.orientation.y = qy
    msg.pose.orientation.z = qz
    msg.pose.orientation.w = qw

    transformed = listener.transformPose(TARGET_LOCAL_FRAME, msg)
    quat = transformed.pose.orientation
    local_roll, local_pitch, local_yaw = tf.transformations.euler_from_quaternion(
        [quat.x, quat.y, quat.z, quat.w]
    )

    return [
        float(transformed.pose.position.x),
        float(transformed.pose.position.y),
        float(transformed.pose.position.z),
        _normalize_deg(math.degrees(local_yaw)),
        _normalize_deg(math.degrees(local_pitch)),
        _normalize_deg(math.degrees(local_roll)),
    ]


def _read_current_local_ee_poses(listener) -> Tuple[Pose, Pose]:
    import rospy

    if not _raw_ee_poses:
        raise RuntimeError("未读取到 /humanoid_wheel/eePoses 原始数据")
    if len(_raw_ee_poses) < 12:
        raise RuntimeError(f"/humanoid_wheel/eePoses 数据长度不足: {len(_raw_ee_poses)}，期望至少 12")

    listener.waitForTransform(
        TARGET_LOCAL_FRAME,
        EE_POSES_SOURCE_FRAME,
        rospy.Time(0),
        rospy.Duration(TF_TIMEOUT),
    )

    left_raw = _raw_ee_poses[0:6]
    right_raw = _raw_ee_poses[6:12]
    return (
        _raw_pose_to_local(listener, left_raw),
        _raw_pose_to_local(listener, right_raw),
    )


@dataclass
class ArmEELocalRelativeTimedParams(SkillParams):
    """双臂末端 local/base_link 相对补偿参数。

    只暴露 6 个调参字段：xyz 补偿量 + 末端三个欧拉角。
    其余运行参数（desire_time、tf、topic、frame 等）固定在模块常量中，
    与示例脚本保持一致。
    """

    skill_name: str = "arm_ee_local_relative_timed"
    offset_x: float = 0.05
    offset_y: float = 0.0
    offset_z: float = -0.05
    fixed_yaw: float = 0.0
    fixed_pitch: float = -90.0
    fixed_roll: float = 0.0
    timeout: float = 30.0


@define_manifest(
    label="双臂末端 local 相对补偿（TimedCmd）",
    category=["motion", "arm"],
    tree_type="studio_smoke",
    description=(
        "读取当前 /humanoid_wheel/eePoses，经 TF 转为 base_link/local，"
        "叠加 x/y/z offset，姿态使用固定 yaw/pitch/roll，然后下发 send_arm_ee_local_timed。"
        "其余运行参数（desire_time/tf/topic/frame 等）固定与示例脚本一致。"
    ),
    params=[
        {"name": "offset_x", "type": "float", "default": "0.05", "description": "local x 偏移，正值向前，单位 m"},
        {"name": "offset_y", "type": "float", "default": "0.0", "description": "local y 偏移，正值向左，单位 m"},
        {"name": "offset_z", "type": "float", "default": "-0.05", "description": "local z 偏移，正值向上，单位 m"},
        {"name": "fixed_yaw", "type": "float", "default": "0.0", "description": "目标 yaw，单位 deg"},
        {"name": "fixed_pitch", "type": "float", "default": "-90.0", "description": "目标 pitch，单位 deg"},
        {"name": "fixed_roll", "type": "float", "default": "0.0", "description": "目标 roll，单位 deg"},
    ],
    inputs=[],
    outputs=[],
)
class ArmEELocalRelativeTimedSkill(SkillBase):
    """双臂末端 local/base_link 相对补偿（TimedCmd）。"""

    def __init__(self, hardware: IHardware):
        super().__init__(name="arm_ee_local_relative_timed")
        self.hardware = hardware
        self.params: Optional[ArmEELocalRelativeTimedParams] = None
        self._done = False
        self._listener = None

    def on_initialize(self, params: ArmEELocalRelativeTimedParams) -> Result:
        if not isinstance(params, ArmEELocalRelativeTimedParams):
            return Result.fail("Invalid parameters for ArmEELocalRelativeTimedSkill")
        self.params = params
        self._done = False
        return Result.ok()

    def on_execute(self) -> Result:
        if self._done:
            return Result.ok("ArmEELocalRelativeTimedSkill already finished")

        try:
            total_start_ts = time.monotonic()
            subscribe_start_ts = time.monotonic()
            _ensure_subscribed()
            logger.info("[Perf][arm_ee_local_relative_timed] ensure_subscribed elapsed=%.3fs", time.monotonic() - subscribe_start_ts)
            if WAIT_FEEDBACK > 0:
                wait_start_ts = time.monotonic()
                time.sleep(WAIT_FEEDBACK)
                logger.info(
                    "[Perf][arm_ee_local_relative_timed] wait_feedback elapsed=%.3fs planned=%.3fs",
                    time.monotonic() - wait_start_ts,
                    WAIT_FEEDBACK,
                )
            if self._listener is None:
                listener_start_ts = time.monotonic()
                self._listener = _make_tf_listener()
                logger.info("[Perf][arm_ee_local_relative_timed] make_tf_listener elapsed=%.3fs", time.monotonic() - listener_start_ts)

            read_start_ts = time.monotonic()
            left_current, right_current = _read_current_local_ee_poses(self._listener)
            logger.info("[Perf][arm_ee_local_relative_timed] read_current_local elapsed=%.3fs", time.monotonic() - read_start_ts)
            left_target, right_target = self._build_targets(left_current, right_current)

            logger.info("当前 local/base_link left =%s", [round(v, 4) for v in left_current])
            logger.info("当前 local/base_link right=%s", [round(v, 4) for v in right_current])
            logger.info("目标 local left =%s", [round(v, 4) for v in left_target])
            logger.info("目标 local right=%s", [round(v, 4) for v in right_target])

            # ── 前置: 切换手臂外部控制模式 + focus_ee/focus_z 关闭 ──
            if hasattr(self.hardware, "set_focus_ee"):
                focus_ee_start_ts = time.monotonic()
                focus_ee_result = self.hardware.set_focus_ee(False)
                logger.info(
                    "[Perf][arm_ee_local_relative_timed] set_focus_ee elapsed=%.3fs success=%s",
                    time.monotonic() - focus_ee_start_ts,
                    focus_ee_result.success,
                )
            if hasattr(self.hardware, "set_focus_z"):
                focus_z_start_ts = time.monotonic()
                focus_z_result = self.hardware.set_focus_z(False)
                logger.info(
                    "[Perf][arm_ee_local_relative_timed] set_focus_z elapsed=%.3fs success=%s",
                    time.monotonic() - focus_z_start_ts,
                    focus_z_result.success,
                )

            prepare_fn = getattr(self.hardware, "set_arm_control_mode", None)
            if prepare_fn is not None:
                mode_start_ts = time.monotonic()
                prep_result = prepare_fn(2)
                logger.info(
                    "[Perf][arm_ee_local_relative_timed] set_arm_control_mode mode=2 elapsed=%.3fs success=%s",
                    time.monotonic() - mode_start_ts,
                    prep_result.success,
                )
                if not prep_result.success:
                    self._done = True
                    return Result.fail(f"set_arm_control_mode(2) failed: {prep_result.message}")
                logger.info("arm_ee_local_relative_timed 前置完成: set_arm_control_mode(2)")
                time.sleep(0.3)

            fn = getattr(self.hardware, "send_arm_ee_local_timed", None)
            if fn is None:
                self._done = True
                return Result.fail("Hardware does not implement send_arm_ee_local_timed()")

            send_start_ts = time.monotonic()
            logger.info(
                "[Perf][arm_ee_local_relative_timed] timed_cmd_start desire_time=%.3fs",
                DESIRE_TIME,
            )
            result = fn(
                left_pose=left_target,
                right_pose=right_target,
                desire_time=DESIRE_TIME,
            )
            logger.info(
                "[Perf][arm_ee_local_relative_timed] timed_cmd_done success=%s elapsed=%.3fs desire_time=%.3fs",
                result.success,
                time.monotonic() - send_start_ts,
                DESIRE_TIME,
            )
            if not result.success:
                self._done = True
                return Result.fail(f"末端相对补偿指令发送失败: {result.message}")

            sleep_start_ts = time.monotonic()
            time.sleep(DESIRE_TIME + 0.2)
            logger.info(
                "[Perf][arm_ee_local_relative_timed] post_sleep_done elapsed=%.3fs planned=%.3fs",
                time.monotonic() - sleep_start_ts,
                DESIRE_TIME + 0.2,
            )
            if RELEASE_AFTER:
                release_result = self._release_external_hold()
                if not release_result.success:
                    self._done = True
                    return release_result

            self._done = True
            logger.info("[Perf][arm_ee_local_relative_timed] total_elapsed=%.3fs", time.monotonic() - total_start_ts)
            return Result.ok("arm_ee_local_relative_timed done", data={
                "left_current": left_current,
                "right_current": right_current,
                "left_target": left_target,
                "right_target": right_target,
            })
        except Exception as e:
            self._done = True
            logger.error("arm_ee_local_relative_timed 异常: %s", e, exc_info=True)
            return Result.fail(f"arm_ee_local_relative_timed error: {e}")

    def _build_targets(self, left_current: Pose, right_current: Pose) -> Tuple[Pose, Pose]:
        fixed_orientation = [self.params.fixed_yaw, self.params.fixed_pitch, self.params.fixed_roll]
        left_target = [
            left_current[0] + float(self.params.offset_x),
            left_current[1] + float(self.params.offset_y),
            left_current[2] + float(self.params.offset_z),
            *fixed_orientation,
        ]
        right_target = [
            right_current[0] + float(self.params.offset_x),
            right_current[1] + float(self.params.offset_y),
            right_current[2] + float(self.params.offset_z),
            *fixed_orientation,
        ]
        return left_target, right_target

    def _release_external_hold(self) -> Result:
        fn = getattr(self.hardware, "set_arm_control_mode", None)
        if fn is None:
            return Result.fail("Hardware does not implement set_arm_control_mode()")
        mode_start_ts = time.monotonic()
        result = fn(0)
        logger.info(
            "[Perf][arm_ee_local_relative_timed] set_arm_control_mode mode=0 elapsed=%.3fs success=%s",
            time.monotonic() - mode_start_ts,
            result.success,
        )
        if not result.success:
            return Result.fail(f"set_arm_control_mode(0) failed: {result.message}")
        logger.info("arm_ee_local_relative_timed 后置完成: set_arm_control_mode(0)")
        return Result.ok()

    def on_is_finished(self) -> bool:
        return self._done
