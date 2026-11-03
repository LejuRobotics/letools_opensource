# -*- coding: utf-8 -*-
"""PlaceRing Skill：放置圆环。

将手中的圆环放到放置台上：运动到放置位 → 张开夹爪 → 末端后撤 → 场景分段归位。
放置和后撤均使用底盘局部坐标；后撤可根据实际反馈小幅调整朝向。
"""

import math
import time
from dataclasses import dataclass
from typing import List, Optional

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
    retreat_distance: float = 0.30
    retreat_time: float = 9.5
    retreat_pitch_offset_deg: Optional[float] = None
    retreat_orientation_tolerance_deg: float = 5.0
    return_joint_stages: Optional[List[List[Optional[float]]]] = None
    return_left_joint_degrees: Optional[List[float]] = None
    return_match_left: bool = False
    return_sync: bool = False
    return_time: float = 4.0
    return_max_forward: float = 0.45
    timeout: float = 60.0


class PlaceRingSkill(SkillBase):
    """放置圆环 Skill：运动到放置位 → 张开夹爪 → 末端后撤 → 场景分段归位。"""

    def __init__(self, hardware: IHardware):
        super().__init__(name="place_ring")
        self.hardware = hardware
        self.params: Optional[PlaceRingParams] = None
        self._phase = 0
        self._success = False
        self._pre_place_pose = None
        self._right_feedback_state = None
        self._right_fk_chain = None
        self._return_chains = {}
        self._return_index = 0

    def on_initialize(self, params: PlaceRingParams) -> Result:
        if not isinstance(params, PlaceRingParams):
            return Result.fail("Invalid parameters for PlaceRingSkill")
        if not (0 < params.retreat_distance <= 0.30 and 1.0 <= params.retreat_time <= 10.0):
            return Result.fail("末端后撤距离或执行时间无效")
        if (params.retreat_pitch_offset_deg is not None
                and not (math.isfinite(params.retreat_pitch_offset_deg)
                         and abs(params.retreat_pitch_offset_deg) <= 5.0)):
            return Result.fail("后撤朝向调整角度无效")
        if not (math.isfinite(params.retreat_orientation_tolerance_deg)
                and 0 < params.retreat_orientation_tolerance_deg <= 10.0):
            return Result.fail("后撤姿态验收容差无效")
        if not isinstance(params.return_match_left, bool) or not isinstance(params.return_sync, bool):
            return Result.fail("归位对称姿态及同步开关必须为布尔值")
        if ((params.return_match_left or params.return_sync)
                and params.return_joint_stages is None):
            return Result.fail("对称姿态或同步归位需要完整归位轨迹")
        if params.return_joint_stages is not None:
            stages = params.return_joint_stages
            if (not isinstance(stages, list) or not 1 <= len(stages) <= 6
                    or not isinstance(params.return_left_joint_degrees, list)
                    or len(params.return_left_joint_degrees) != 7):
                return Result.fail("分段归位需要右臂轨迹及左臂7个关节角")
            for stage in stages + [params.return_left_joint_degrees]:
                if (not isinstance(stage, list) or len(stage) != 7
                        or all(value is None for value in stage)
                        or any(value is not None and
                               (isinstance(value, bool) or not isinstance(value, (int, float))
                                or not math.isfinite(value)) for value in stage)):
                    return Result.fail("归位轨迹必须为7个有限角度；右臂可用null保持当前角度")
            if (any(value is None for value in stages[-1] + params.return_left_joint_degrees)
                    or not 1.0 <= params.return_time <= 10.0
                    or not 0.20 <= params.return_max_forward <= 0.50):
                return Result.fail("归位终点、执行时间或前伸边界无效")
        elif params.return_left_joint_degrees is not None:
            return Result.fail("左臂归位参数需要同时配置右臂轨迹")
        if params.retreat_pitch_offset_deg is not None or params.return_joint_stages is not None:
            # 预先加载依赖，避免首次导入占用后撤反馈的有效时间。
            import importlib
            try:
                importlib.import_module('scipy.spatial.transform')
            except ImportError as e:
                return Result.fail(f"后撤朝向计算依赖缺失: {e}")
        self._right_feedback_state = None
        self._return_index = 0
        self._return_chains = {}
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

        # Phase 4: 按实际反馈后撤，可小幅调整朝向；允许躯干继续协调
        if self._phase == 4:
            try:
                position = self._read_stable_right_position()
                target_position = position.copy()
                target_position[0] -= p.retreat_distance
                # 保留原位置参考作为偏差补偿初值，实际到位目标始终单独检查。
                reference_pose = self._pre_place_pose.copy()
                reference_pose[0] -= p.retreat_distance
                adjust_orientation = p.retreat_pitch_offset_deg is not None
                if adjust_orientation:
                    start_orientation = self._read_retreat_orientation(position, 0.0)
                    orientation = self._read_retreat_orientation(position, p.retreat_pitch_offset_deg)
                    reference_pose[3:] = orientation
                    logger.info("[PlaceRingSkill] 使用右臂局部系末端控制，保留位置参考并调整朝向")
                retreat_pose = reference_pose.copy()
                logger.info("[PlaceRingSkill] 末端后撤%.3fm，起点(local): %s，目标: %s",
                            p.retreat_distance, position, target_position + reference_pose[3:])
                for attempt in range(3):
                    logger.info("[PlaceRingSkill] 后撤第%d次控制参考(local): %s", attempt + 1, retreat_pose)
                    result = self.hardware.send_right_arm_ee_local_timed(
                        pose=retreat_pose, desire_time=p.retreat_time
                    )
                    if not result.success:
                        return Result.fail(f"右手末端后撤失败: {result.message}")
                    data = result.data if isinstance(result.data, dict) else {}
                    duration = float(data.get("actual_time", p.retreat_time))
                    if not math.isfinite(duration) or duration < 0:
                        return Result.fail("末端后撤执行时间无效")
                    time.sleep(max(p.retreat_time, duration) + 0.5)
                    actual = self._read_stable_right_position()
                    errors = [goal - value for goal, value in zip(target_position, actual)]
                    error = math.sqrt(sum(v * v for v in errors))
                    logger.info("[PlaceRingSkill] 后撤反馈(local): %s，位置误差=%.4fm", actual, error)
                    orientation_error = 0.0
                    if adjust_orientation:
                        from scipy.spatial.transform import Rotation
                        actual_orientation = self._read_retreat_orientation(actual, 0.0)
                        desired = Rotation.from_euler('ZYX', orientation, degrees=True)
                        measured = Rotation.from_euler('ZYX', actual_orientation, degrees=True)
                        rotation_error = desired * measured.inv()
                        orientation_error = math.degrees(rotation_error.magnitude())
                        logger.info("[PlaceRingSkill] 后撤朝向误差=%.2f度", orientation_error)
                        initial = Rotation.from_euler('ZYX', start_orientation, degrees=True)
                        orientation_change = math.degrees((initial.inv() * measured).magnitude())
                        if self._retreat_clearance_reached(position, actual, orientation_change):
                            logger.info("[PlaceRingSkill] 撤离条件满足，进入归位阶段")
                            break
                    elif error <= 0.01:
                        break
                    if (attempt == 2 or error > 0.03 or actual[2] < position[2] - 0.02
                            or orientation_error > 10.0):
                        if adjust_orientation:
                            return Result.fail(f"未满足撤离条件：后撤{position[0] - actual[0]:.3f}m，"
                                               f"侧移{actual[1] - position[1]:.3f}m，"
                                               f"高度变化{actual[2] - position[2]:.3f}m，"
                                               f"姿态变化{orientation_change:.2f}度，停止归位")
                        return Result.fail(f"末端后撤未到位，位置误差{error:.4f}m，停止归位")
                    # 只对剩余误差做小幅补偿，不重复叠加后撤距离。
                    for i in range(3):
                        correction = max(-0.01, min(0.01, 0.5 * errors[i]))
                        retreat_pose[i] = max(reference_pose[i] - 0.02,
                                              min(reference_pose[i] + 0.02, retreat_pose[i] + correction))
                    if adjust_orientation and orientation_error > 5.0:
                        # 在底盘系中补偿朝向静差；最多两次，每次不超过3度，实际目标不变。
                        rotation_step = 0.5 * rotation_error.as_rotvec()
                        step_angle = math.sqrt(sum(v * v for v in rotation_step))
                        if step_angle > math.radians(3.0):
                            rotation_step *= math.radians(3.0) / step_angle
                        reference_rotation = Rotation.from_euler('ZYX', retreat_pose[3:], degrees=True)
                        reference_rotation = Rotation.from_rotvec(rotation_step) * reference_rotation
                        retreat_pose[3:] = reference_rotation.as_euler('ZYX', degrees=True).tolist()
                        logger.info("[PlaceRingSkill] 朝向参考小幅补偿%.2f度，实际到位目标保持不变",
                                    math.degrees(min(step_angle, math.radians(3.0))))
            except Exception as e:
                return Result.fail(f"末端后撤失败: {e}")
            self._phase = 5
            return Result.ok("phase 4 done")

        # Phase 5: 场景内分段关节归位，不切换全局自动归位模式
        if self._phase == 5:
            if p.return_joint_stages is not None:
                stages = [('right', stage) for stage in p.return_joint_stages[:-1]]
                if p.return_match_left:
                    stages.append(('match_left', None))
                stages.append(('both' if p.return_sync else 'right', p.return_joint_stages[-1]))
                if not p.return_sync:
                    stages.append(('left', p.return_left_joint_degrees))
                if self._return_index < len(stages):
                    side, target = stages[self._return_index]
                    try:
                        if side == 'match_left':
                            self._read_stable_right_position()
                            state = self._return_state()
                            # 左右关节轴定义不同；该符号映射使右臂姿态与左臂对称。
                            signs = [1, -1, -1, 1, -1, -1, 1]
                            target = [math.degrees(value) * sign
                                      for value, sign in zip(state[7:14], signs)]
                            logger.info("[PlaceRingSkill] 右臂调整到左臂对称姿态，目标(度)=%s", target)
                            # 对齐只是过渡姿态：验收肩肘1～4，腕部继续参考目标并监控。
                            result = self._return_arm_stage('right', target, checked_joints=[0, 1, 2, 3])
                        elif side == 'both':
                            result = self._return_arm_targets([
                                ('left', p.return_left_joint_degrees), ('right', target)
                            ])
                        else:
                            result = self._return_arm_stage(side, target)
                    except Exception as e:
                        return Result.fail(f"分段归位失败: {e}")
                    if not result.success:
                        return result
                    self._return_index += 1
                    return Result.ok("return stage done")
            print("[PlaceRingSkill] 放置完成")
            self._success = True
            return Result.ok("done")

        return Result.fail("unknown phase")

    def on_is_finished(self) -> bool:
        return self._phase >= 5 and self._success

    def on_cancel(self) -> Result:
        self._phase = 5
        return Result.ok("cancelled")

    def _retreat_clearance_reached(self, start, actual, orientation_change):
        """按实际撤离量、侧移、高度和相对起点的姿态变化检查，不追求精确位姿。"""
        distance = start[0] - actual[0]
        lateral = actual[1] - start[1]
        height = actual[2] - start[2]
        logger.info("[PlaceRingSkill] 撤离检查：实际后撤=%.2fcm，侧移=%.2fcm，"
                    "高度变化=%.2fcm，姿态变化=%.2f度",
                    distance * 100.0, lateral * 100.0, height * 100.0, orientation_change)
        epsilon = 1e-9
        return (self.params.retreat_distance - epsilon <= distance
                <= self.params.retreat_distance + 0.02 + epsilon
                and abs(lateral) <= 0.01 + epsilon
                and -0.01 - epsilon <= height <= 0.02 + epsilon
                and 0.0 <= orientation_change <= self.params.retreat_orientation_tolerance_deg + epsilon)

    def _return_arm_stage(self, side, stage, checked_joints=None):
        """单臂过渡段可指定验收关节，默认仍验收所有显式目标。"""
        return self._return_arm_targets([(side, stage)], checked_joints=checked_joints)

    def _return_arm_targets(self, stages, checked_joints=None):
        """双臂最终归位始终全关节验收；单臂过渡可单独选择验收关节。"""
        if checked_joints is not None:
            if (len(stages) != 1 or not isinstance(checked_joints, (list, tuple))
                    or not checked_joints
                    or any(isinstance(i, bool) or not isinstance(i, int) or not 0 <= i < 7
                           for i in checked_joints)
                    or any(stages[0][1][i] is None for i in checked_joints)):
                return Result.fail("验收关节设置无效，只允许单臂过渡段指定有效目标关节")
        self._read_stable_right_position()
        state = self._return_state()
        targets = []
        for side, stage in stages:
            offset = 14 if side == 'right' else 7
            current = [math.degrees(value) for value in state[offset:offset + 7]]
            target = [current[i] if value is None else float(value)
                      for i, value in enumerate(stage)]
            chain, limits = self._return_model(side)
            for i, (lower, upper) in enumerate(limits):
                if stage[i] is None and lower - 0.5 <= target[i] <= upper + 0.5:
                    target[i] = max(lower, min(upper, target[i]))
                if not lower <= target[i] <= upper:
                    return Result.fail(f"{side}臂归位第{i + 1}关节目标越限，停止后续动作")
            maximum = self._return_path_forward(chain, state, current, target)
            if maximum > self.params.return_max_forward:
                return Result.fail(f"{side}臂归位标称路径前伸{maximum:.3f}m，超过场景边界，停止后续动作")
            targets.append((side, stage, offset, target, chain, limits))
            logger.info("[PlaceRingSkill] 分段归位%d：%s臂，关节目标(度)=%s，标称最前位置=%.3fm",
                        self._return_index + 1, side, target, maximum)
        if len(targets) == 2:
            # 此接口使用适配器的角度单位（当前为度），内部转换为弧度。
            commands = [{'planner_index': 8 if side == 'left' else 9,
                         'desire_time': self.params.return_time, 'cmd_vec': target}
                        for side, _, _, target, _, _ in targets]
            logger.info("[PlaceRingSkill] 双臂同时归位，单次多指令请求，时间=%.3fs", self.params.return_time)
            result = self.hardware.send_timed_multi_commands(commands=commands, is_sync=True)
        else:
            side, _, _, target, _, _ = targets[0]
            send = (self.hardware.send_right_arm_joint_timed if side == 'right'
                    else self.hardware.send_left_arm_joint_timed)
            result = send(joint_angles=target, desire_time=self.params.return_time)
        if not result.success:
            return Result.fail(f"分段归位指令失败: {result.message}")
        data = result.data if isinstance(result.data, dict) else {}
        duration = float(data.get('actual_time', self.params.return_time))
        if not math.isfinite(duration) or duration < 0 or duration > 20.0:
            return Result.fail("归位指令返回的执行时间无效，停止后续动作")
        time.sleep(max(self.params.return_time, duration) + 0.5)
        # 单臂检查7个、双臂检查14个实际关节，连续三次新反馈变化小于0.3度。
        previous, stable = None, 0
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            self._read_right_position_local()
            actual_state = self._return_state()
            combined = [math.degrees(value) for _, _, offset, _, _, _ in targets
                        for value in actual_state[offset:offset + 7]]
            change = max(abs(a - b) for a, b in zip(combined, previous)) if previous is not None else math.inf
            stable = stable + 1 if change <= 0.3 else 1
            previous = combined
            if stable >= 3:
                break
            time.sleep(0.1)
        if stable < 3:
            return Result.fail("归位关节反馈未稳定，停止后续动作")
        for side, stage, offset, target, chain, limits in targets:
            actual = [math.degrees(value) for value in actual_state[offset:offset + 7]]
            # 对称姿态仅验收肩肘；其余段默认检查显式目标，最终双臂仍逐一验收7关节。
            checked = (list(checked_joints) if checked_joints is not None else
                       [i for i, value in enumerate(stage) if value is not None])
            errors = [abs(a - b) for a, b in zip(actual, target)]
            largest = max(checked, key=lambda i: errors[i])
            error = errors[largest]
            other_error = max((errors[i] for i in range(7) if i not in checked), default=0.0)
            for i, (lower, upper) in enumerate(limits):
                if not lower - 0.5 <= actual[i] <= upper + 0.5:
                    return Result.fail(f"{side}臂第{i + 1}关节实际反馈越限，停止后续动作")
            forward = self._return_path_forward(chain, actual_state, actual, actual)
            logger.info("[PlaceRingSkill] %s臂归位反馈：验收关节=%s，最大验收误差=%.2f度"
                        "（第%d关节，目标=%.2f度，实际=%.2f度），非验收关节最大偏移=%.2f度，最前位置=%.3fm",
                        side, [i + 1 for i in checked], error, largest + 1,
                        target[largest], actual[largest], other_error, forward)
            if error > 8.0 or forward > self.params.return_max_forward + 0.02:
                return Result.fail(f"{side}臂分段归位未到位：验收第{largest + 1}关节误差{error:.2f}度，"
                                   f"最前位置{forward:.3f}m")
        return Result.ok("return stage reached")

    def _return_state(self):
        """只使用配对读取器刚收到的完整实际状态。"""
        feedback = self._right_feedback_state
        if not isinstance(feedback, tuple) or len(feedback) != 2:
            raise RuntimeError("归位缺少实际关节反馈")
        state, received = feedback
        if (len(state) != 21 or not all(math.isfinite(v) for v in state)
                or not 0 <= time.monotonic() - received <= 0.25):
            raise RuntimeError("归位实际关节反馈无效或已过期")
        return state

    def _return_model(self, side):
        """读取本机URDF关节链和限位，仅供上位机标称路径检查。"""
        from pathlib import Path
        import xml.etree.ElementTree as ET
        import numpy as np
        import rospy
        from scipy.spatial.transform import Rotation

        if side in self._return_chains:
            return self._return_chains[side]
        description = rospy.get_param('/robot_description', None)
        if description is None:
            model_path = (Path(__file__).resolve().parents[4] / 'kuavo-ros-opensource'
                          / 'src/kuavo_assets/models/biped_s62/urdf/biped_s62.urdf')
            description = model_path.read_text()
        root = ET.fromstring(description)
        if root.tag != 'robot':
            raise RuntimeError("归位模型不是有效URDF")
        prefix = 'r' if side == 'right' else 'l'
        names = ['knee_joint', 'leg_joint', 'waist_pitch_joint', 'waist_yaw_joint']
        names += ['zarm_%s%d_joint' % (prefix, i) for i in range(1, 8)]
        by_child = {joint.find('child').get('link'): joint for joint in root.findall('joint')}
        joints, visited = [], set()
        link = 'zarm_%s7_end_effector' % prefix
        while link != 'base_link':
            if link in visited or link not in by_child:
                raise RuntimeError("归位URDF缺少有效手臂关节链")
            visited.add(link)
            joint = by_child[link]
            joints.append(joint)
            link = joint.find('parent').get('link')
        chain, limits, active = [], {}, []
        for joint in reversed(joints):
            origin = joint.find('origin')
            xyz = [float(v) for v in (origin.get('xyz', '0 0 0') if origin is not None else '0 0 0').split()]
            rpy = [float(v) for v in (origin.get('rpy', '0 0 0') if origin is not None else '0 0 0').split()]
            if len(xyz) != 3 or len(rpy) != 3 or not np.all(np.isfinite(xyz + rpy)):
                raise RuntimeError("归位URDF关节原点无效")
            transform = np.eye(4)
            transform[:3, 3] = xyz
            transform[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
            index, axis = None, None
            if joint.get('type') != 'fixed':
                if joint.get('type') != 'revolute' or joint.get('name') not in names:
                    raise RuntimeError("归位URDF活动关节不匹配")
                index = names.index(joint.get('name'))
                active.append(index)
                axis_node = joint.find('axis')
                axis = np.array([float(v) for v in (axis_node.get('xyz', '1 0 0') if axis_node is not None else '1 0 0').split()])
                if axis.shape != (3,) or not np.all(np.isfinite(axis)) or np.linalg.norm(axis) < 1e-9:
                    raise RuntimeError("归位URDF关节转轴无效")
                axis /= np.linalg.norm(axis)
                if index >= 4:
                    limit = joint.find('limit')
                    lower, upper = [math.degrees(float(limit.get(k))) for k in ('lower', 'upper')]
                    if not math.isfinite(lower + upper) or lower >= upper:
                        raise RuntimeError("归位URDF关节限位无效")
                    limits[index - 4] = (lower, upper)
            chain.append((transform, index, axis))
        if sorted(active) != list(range(11)) or sorted(limits) != list(range(7)):
            raise RuntimeError("归位URDF关节数量不匹配")
        model = (chain, [limits[i] for i in range(7)])
        self._return_chains[side] = model
        return model

    def _return_path_forward(self, chain, state, start, target):
        """采样关节线性标称路径的臂部原点及夹爪末端，不代替完整碰撞检测。"""
        import numpy as np
        from scipy.spatial.transform import Rotation

        maximum = -math.inf
        for fraction in np.linspace(0.0, 1.0, 61):
            q = np.r_[state[3:7], np.radians(np.array(start) * (1 - fraction) + np.array(target) * fraction)]
            pose = np.eye(4)
            for origin, index, axis in chain:
                pose = pose @ origin
                if index is not None:
                    motion = np.eye(4)
                    motion[:3, :3] = Rotation.from_rotvec(axis * q[index]).as_matrix()
                    pose = pose @ motion
                if index is not None and index >= 4:
                    maximum = max(maximum, float(pose[0, 3]))
            # 夹爪从腕部向末端延伸约15厘米，额外计入该参考点。
            maximum = max(maximum, float((pose @ np.array([0.0, 0.0, -0.15, 1.0]))[0]), float(pose[0, 3]))
        if not math.isfinite(maximum):
            raise RuntimeError("归位标称路径计算无效")
        return maximum

    def _read_stable_right_position(self):
        """稳定等待窗口3秒，连续三次新反馈位置变化小于2毫米后检查到位。"""
        previous = None
        previous_time = None
        stable_count = 0
        sample_count = 0
        max_change = 0.0
        last_change = 0.0
        last_interval = 0.0
        started = time.monotonic()
        deadline = started + 3.0
        while time.monotonic() < deadline:
            position = self._read_right_position_local()
            now = time.monotonic()
            sample_count += 1
            last_change = (math.sqrt(sum((a - b) ** 2 for a, b in zip(position, previous)))
                           if previous is not None else 0.0)
            last_interval = now - previous_time if previous_time is not None else 0.0
            max_change = max(max_change, last_change)
            stable_count = stable_count + 1 if last_change <= 0.002 else 1
            if stable_count >= 3:
                logger.info("[PlaceRingSkill] 末端反馈已稳定，采样%d次，等待%.2fs",
                            sample_count, now - started)
                return position
            previous = position
            previous_time = now
            time.sleep(0.1)
        logger.warning("[PlaceRingSkill] 等待稳定%.2fs，共%d次反馈，最大相邻变化=%.4fm，"
                       "末次相邻变化=%.4fm，末次采样间隔=%.3fs",
                       time.monotonic() - started, sample_count, max_change,
                       last_change, last_interval)
        raise RuntimeError("末端反馈等待3秒仍未稳定，停止归位")

    def _read_right_position_local(self):
        """配对新收到的WBC实际状态与末端位置，避免混用仿真/墙钟时间。"""
        import threading
        import rospy
        from std_msgs.msg import Float64MultiArray

        feedback = {}
        lock = threading.Lock()
        updated = threading.Event()
        subscriptions = []
        started = time.monotonic()

        def callback(msg, name):
            with lock:
                feedback[name] = (msg, time.monotonic())
            updated.set()

        try:
            for name, topic in [('ee', '/humanoid_wheel/eePoses'),
                                ('state', '/mobile_manipulator_wbc_observation/state')]:
                subscriptions.append(rospy.Subscriber(
                    topic, Float64MultiArray, lambda msg, key=name: callback(msg, key),
                    queue_size=1, tcp_nodelay=True
                ))
            while time.monotonic() - started < 1.0:
                updated.wait(0.05)
                updated.clear()
                with lock:
                    pair = feedback.copy()
                if not all(key in pair for key in ('ee', 'state')):
                    continue
                ee, ee_time = pair['ee']
                state, state_time = pair['state']
                now = time.monotonic()
                if (min(ee_time, state_time) < started or now - min(ee_time, state_time) > 0.25
                        or abs(ee_time - state_time) > 0.05):
                    continue
                headers = [getattr(msg, '_connection_header', {}) or {} for msg in (ee, state)]
                if any(h.get('latching') == '1' for h in headers):
                    raise RuntimeError("末端反馈为锁存旧值，停止撤手")
                callers = [h.get('callerid') for h in headers]
                if all(callers) and callers[0] != callers[1]:
                    raise RuntimeError("末端位置与WBC状态来源不一致")
                position = list(ee.data[6:9])
                base = list(state.data[:3])
                if (len(ee.data) != 12 or len(state.data) != 21
                        or not all(math.isfinite(v) for v in position + base)):
                    raise RuntimeError("末端位置或WBC实际状态无效")
                dx, dy = position[0] - base[0], position[1] - base[1]
                self._right_feedback_state = (list(state.data), min(ee_time, state_time))
                return [math.cos(base[2]) * dx + math.sin(base[2]) * dy,
                        -math.sin(base[2]) * dx + math.cos(base[2]) * dy, position[2]]
            raise RuntimeError("未收到可配对的新末端位置与WBC实际状态")
        finally:
            for sub in subscriptions:
                sub.unregister()

    def _read_retreat_orientation(self, position, pitch_offset_deg):
        """由配对的实际关节反馈计算末端朝向，再绕底盘Y轴小幅转动。"""
        from pathlib import Path
        import xml.etree.ElementTree as ET
        import numpy as np
        import rospy
        from scipy.spatial.transform import Rotation

        feedback = getattr(self, '_right_feedback_state', None)
        if not isinstance(feedback, tuple) or len(feedback) != 2:
            raise RuntimeError("没有可用于计算末端朝向的实际状态")
        state, received = feedback
        state = np.asarray(state, dtype=float)
        position = np.asarray(position, dtype=float)
        age = time.monotonic() - float(received)
        if (state.shape != (21,) or position.shape != (3,)
                or not np.all(np.isfinite(state)) or not np.all(np.isfinite(position))
                or not math.isfinite(age) or not 0 <= age <= 0.25
                or not math.isfinite(pitch_offset_deg)):
            raise RuntimeError("计算末端朝向的状态无效或已过期")

        names = ['knee_joint', 'leg_joint', 'waist_pitch_joint', 'waist_yaw_joint']
        names += ['zarm_r%d_joint' % i for i in range(1, 8)]
        chain = getattr(self, '_right_fk_chain', None)
        if chain is None:
            description = rospy.get_param('/robot_description', None)
            if description is None:
                model_path = (Path(__file__).resolve().parents[4] / 'kuavo-ros-opensource'
                              / 'src/kuavo_assets/models/biped_s62/urdf/biped_s62.urdf')
                description = model_path.read_text()
            root = ET.fromstring(description)
            if root.tag != 'robot':
                raise RuntimeError("末端朝向模型不是有效URDF")
            by_child = {joint.find('child').get('link'): joint
                        for joint in root.findall('joint')}
            joints = []
            visited = set()
            link = 'zarm_r7_end_effector'
            while link != 'base_link':
                if link in visited or link not in by_child:
                    raise RuntimeError("URDF缺少有效的底盘到右手末端关节链")
                visited.add(link)
                joint = by_child[link]
                joints.append(joint)
                link = joint.find('parent').get('link')

            chain = []
            active = []
            for joint in reversed(joints):
                origin = joint.find('origin')
                xyz = [float(v) for v in (origin.get('xyz', '0 0 0')
                       if origin is not None else '0 0 0').split()]
                rpy = [float(v) for v in (origin.get('rpy', '0 0 0')
                       if origin is not None else '0 0 0').split()]
                if len(xyz) != 3 or len(rpy) != 3 or not np.all(np.isfinite(xyz + rpy)):
                    raise RuntimeError("URDF关节原点无效")
                transform = np.eye(4)
                transform[:3, 3] = xyz
                transform[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
                index, axis = None, None
                if joint.get('type') != 'fixed':
                    if joint.get('type') not in ('revolute', 'continuous') or joint.get('name') not in names:
                        raise RuntimeError("URDF右手链包含不支持的活动关节")
                    index = names.index(joint.get('name'))
                    axis_node = joint.find('axis')
                    axis = np.asarray([float(v) for v in (axis_node.get('xyz', '1 0 0')
                                      if axis_node is not None else '1 0 0').split()])
                    if axis.shape != (3,) or not np.all(np.isfinite(axis)) or np.linalg.norm(axis) < 1e-9:
                        raise RuntimeError("URDF关节转轴无效")
                    axis = axis / np.linalg.norm(axis)
                    active.append(index)
                chain.append((transform, index, axis))
            if sorted(active) != list(range(11)):
                raise RuntimeError("URDF右手链与实际状态关节不匹配")
            self._right_fk_chain = chain

        if not isinstance(chain, list) or not chain:
            raise RuntimeError("右手正运动学缓存无效")
        q = np.r_[state[3:7], state[14:21]]
        actual = np.eye(4)
        for origin, index, axis in chain:
            if np.shape(origin) != (4, 4) or not np.all(np.isfinite(origin)):
                raise RuntimeError("右手正运动学缓存变换无效")
            actual = actual @ origin
            if index is not None:
                if not isinstance(index, int) or not 0 <= index < 11 or np.shape(axis) != (3,):
                    raise RuntimeError("右手正运动学缓存关节无效")
                motion = np.eye(4)
                motion[:3, :3] = Rotation.from_rotvec(axis * q[index]).as_matrix()
                actual = actual @ motion
        error = float(np.linalg.norm(actual[:3, 3] - position))
        if not np.all(np.isfinite(actual)) or error > 0.003:
            raise RuntimeError("URDF末端位置与实际反馈不一致，误差%.4fm" % error)
        actual_rotation = Rotation.from_matrix(actual[:3, :3])
        target_rotation = Rotation.from_euler('Y', pitch_offset_deg, degrees=True) * actual_rotation
        actual_ypr = actual_rotation.as_euler('ZYX', degrees=True).tolist()
        target_ypr = target_rotation.as_euler('ZYX', degrees=True).tolist()
        logger.info("[PlaceRingSkill] 实际末端朝向YPR(度): %s，底盘Y轴调整%.1f度后目标: %s",
                    actual_ypr, pitch_offset_deg, target_ypr)
        return target_ypr

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
