"""ROS calls owned only by grasp_mtbf_v1; shared hardware and SDK stay unchanged.

Scene arm/torso angles are degrees; ROS requests are radians. Only planner 2
(torso) and 6/7 (local arm end effectors) belong to this scene interface.
"""

import copy
import math
import threading

from core.domain.result import Result
from .jibot_io import JiBotIO
from .ros_service_call import RosServiceGate


class GraspSceneIO(JiBotIO):
    SERVICE_TIMEOUT = 5.0

    def __init__(self, hardware):
        self._hardware = hardware
        self._command_calls = RosServiceGate()
        self._stop_calls = RosServiceGate()
        self._init_jibot_io()
        self._feedback_lock = threading.Lock()
        self._subscription_lock = threading.Lock()
        self._feedback = {}
        self._subscribers = []

    def send_timed_multi_commands(self, commands, is_sync=False, cancel_event=None):
        import rospy
        from kuavo_msgs.msg import timedSingleCmd
        from kuavo_msgs.srv import lbMultiTimedPosCmd, lbMultiTimedPosCmdRequest

        if not rospy.core.is_initialized():
            return Result.fail("ROS 未初始化")
        if not commands:
            return Result.fail("指令列表为空")
        try:
            request = lbMultiTimedPosCmdRequest()
            request.isSync = bool(is_sync)
            for command in commands:
                planner = command['planner_index']
                if planner not in (2, 6, 7):
                    raise ValueError("本场景仅支持躯干2和局部系双臂6/7")
                size, angle_start = (4, 2) if planner == 2 else (6, 3)
                values = [float(value) for value in command['cmd_vec']]
                duration = float(command['desire_time'])
                if len(values) != size or not all(math.isfinite(v) for v in values):
                    raise ValueError("指令向量维度错误或包含非有限数")
                if not math.isfinite(duration) or duration <= 0:
                    raise ValueError("期望时长必须为正有限数")
                values[angle_start:] = [math.radians(v) for v in values[angle_start:]]
                item = timedSingleCmd()
                item.planner_index = int(planner)
                item.desireTime = duration
                item.cmdVec = values
                request.timedCmdVec.append(item)
            response = self._command_calls.call(
                '/mobile_manipulator_timed_multi_cmd', lbMultiTimedPosCmd, request,
                timeout=self.SERVICE_TIMEOUT, cancel_event=cancel_event,
            )
            if not response.isSuccess:
                return Result.fail(response.message)
            return Result.ok(data={'actual_time': response.actualTime})
        except Exception as exc:
            return Result.fail(f"场景定时指令失败: {exc}")

    def send_torso_pose_timed(self, x, z, yaw, pitch, desire_time=2.0):
        return self.send_timed_multi_commands([{
            'planner_index': 2, 'desire_time': desire_time,
            'cmd_vec': [x, z, yaw, pitch],
        }])

    def set_ruckig_planner_params(self, planner_index, is_sync, velocity_max,
                                  acceleration_max, jerk_max, velocity_min=None,
                                  acceleration_min=None):
        import rospy
        from kuavo_msgs.srv import setRuckigPlannerParams, setRuckigPlannerParamsRequest

        if not rospy.core.is_initialized():
            return Result.fail("ROS 未初始化")
        try:
            request = setRuckigPlannerParamsRequest()
            request.planner_index = planner_index
            request.is_sync = is_sync
            request.velocity_max = velocity_max
            request.acceleration_max = acceleration_max
            request.jerk_max = jerk_max
            if velocity_min is not None:
                request.velocity_min = velocity_min
            if acceleration_min is not None:
                request.acceleration_min = acceleration_min
            response = self._command_calls.call(
                '/mobile_manipulator_set_ruckig_planner_params', setRuckigPlannerParams,
                request, timeout=self.SERVICE_TIMEOUT,
            )
            if not response.result:
                return Result.fail(response.message)
            return Result.ok(data={'message': response.message})
        except Exception as exc:
            return Result.fail(f"场景规划器配置失败: {exc}")

    def set_motion_control_enabled(self, enabled):
        """Independent fault-stop channel. Acknowledgement is not measured standstill."""
        from std_srvs.srv import SetBool, SetBoolRequest
        try:
            response = self._stop_calls.call(
                '/enable_control', SetBool, SetBoolRequest(data=bool(enabled)), timeout=2.0,
            )
            return Result.ok(response.message) if response.success else Result.fail(response.message)
        except Exception as exc:
            return Result.fail(f"场景停止请求失败: {exc}")

    def _ensure_feedback(self):
        """Subscribe only when this scene requests torso safety feedback."""
        import rospy
        from std_msgs.msg import Float64MultiArray
        from geometry_msgs.msg import Twist

        with self._subscription_lock:
            if self._subscribers:
                return

            def motor(msg):
                self._store_feedback('motor_error_codes', list(msg.data))

            def velocity(msg):
                self._store_feedback('base_cmd_vel', {
                    'vx': msg.linear.x, 'vy': msg.linear.y, 'wz': msg.angular.z,
                })

            def torso(msg):
                values = list(msg.data)
                self._store_feedback('torso_target_6d',
                                     {'values': values[:6]} if len(values) >= 6 else None)

            subscribers = []
            try:
                for topic, kind, callback in (
                    ('/sensor_data_motor/motor_error_code', Float64MultiArray, motor),
                    ('/move_base/base_cmd_vel', Twist, velocity),
                    ('/mobile_manipulator/torso_target_6D', Float64MultiArray, torso),
                ):
                    subscribers.append(rospy.Subscriber(topic, kind, callback, queue_size=1))
            except Exception:
                for subscriber in subscribers:
                    subscriber.unregister()
                raise
            self._subscribers = subscribers

    def _store_feedback(self, key, value):
        with self._feedback_lock:
            self._feedback[key] = value

    def _get_feedback(self, key):
        self._ensure_feedback()
        with self._feedback_lock:
            return copy.deepcopy(self._feedback.get(key))

    def get_motor_error_codes(self):
        return self._get_feedback('motor_error_codes')

    def get_base_cmd_vel(self):
        return self._get_feedback('base_cmd_vel')

    def get_torso_target_6d(self):
        return self._get_feedback('torso_target_6d')

    # These existing public operations need no behavior or API changes.
    def control_end_effector(self, side, command):
        return self._hardware.control_end_effector(side, command)

    def send_world_position(self, x, y, yaw):
        return self._hardware.send_world_position(x, y, yaw)

    def send_base_position(self, x, y, yaw):
        return self._hardware.send_base_position(x, y, yaw)

    def send_base_velocity(self, vx, vy, wz):
        return self._hardware.send_base_velocity(vx, vy, wz)


_scene_io = None
_scene_lock = threading.Lock()


def get_scene_io():
    """One private instance per process; never patch shared hardware or its classes."""
    global _scene_io
    with _scene_lock:
        if _scene_io is None:
            from orchestration.shared_hardware import get_shared_hardware
            _scene_io = GraspSceneIO(get_shared_hardware())
        return _scene_io
