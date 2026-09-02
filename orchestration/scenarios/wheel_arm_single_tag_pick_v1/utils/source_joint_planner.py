# -*- coding: utf-8 -*-
"""源 NodeTagToArmGoal 的 joint 语义（场景专用实现）。"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Callable, List, Optional, Sequence, Tuple

import numpy as np
from scipy.spatial.transform import Rotation

from core.common.logger import get_logger


_LOGGER = get_logger(__name__)



@dataclass(frozen=True)
class SourcePose:
    """Source SDK arm_ik 接受的 position/orientation DTO；单位为米和弧度。"""

    position: Tuple[float, float, float]
    orientation: Tuple[float, float, float, float]


@dataclass(frozen=True)
class ScenePose:
    position: Tuple[float, float, float]
    orientation: Tuple[float, float, float, float]
    frame: str


@dataclass(frozen=True)
class KuavoIKParamsCompat:
    """与源 KuavoIKParams 同名同单位的兼容 DTO。"""

    major_optimality_tol: float = 9e-3
    major_feasibility_tol: float = 9e-3
    minor_feasibility_tol: float = 9e-3
    major_iterations_limit: float = 50
    oritation_constraint_tol: float = 19e-3
    pos_constraint_tol: float = 9e-3
    pos_cost_weight: float = 10.0
    constraint_mode: int = 0




def split_source_arm_ik_result(
    result: Any,
) -> Optional[Tuple[List[float], List[float]]]:
    """解析源 ``arm_ik`` 返回的裸 14 维关节弧度列表。"""
    if result is None:
        return None
    if isinstance(result, (str, bytes)):
        raise TypeError("source arm_ik result must be a numeric sequence")
    try:
        values = [float(value) for value in result]
    except (TypeError, ValueError) as exc:
        raise TypeError("source arm_ik result must be a numeric sequence") from exc
    if len(values) != 14:
        raise ValueError("source arm_ik result must contain 14 joint values")
    if not all(math.isfinite(value) for value in values):
        raise ValueError("source arm_ik result must contain finite joint values")
    return values[:7], values[7:]

def pose_from_euler(position: Sequence[float], euler_degree: Sequence[float], frame: str) -> ScenePose:
    """按源 ``Pose.from_euler`` 协议用 intrinsic XYZ 构造场景位姿。

    源 ``data_type.Pose.from_euler`` 内部调用 ``R.from_euler('xyz', euler, degrees=True)``，
    参数顺序为 (x_angle, y_angle, z_angle)；这里刻意保持完全一致，避免 ZYX/xyz
    在非对称欧拉角下产生不同四元数。
    """
    if len(position) != 3 or len(euler_degree) != 3:
        raise ValueError("position and euler_degree must contain three values")
    rotation = Rotation.from_euler("xyz", euler_degree, degrees=True)
    return ScenePose(
        tuple(float(value) for value in position),
        tuple(float(value) for value in rotation.as_quat()),
        frame,
    )


def generate_source_keypoints(
    box_width: float,
    box_behind_tag: float,
    box_beneath_tag: float,
    box_left_tag: float,
    hand_pitch_degree: float = 0.0,
) -> Tuple[List[ScenePose], List[ScenePose]]:
    """按源 case 的实际实现生成四个 BASE 坐标系关键点。

    源函数保留了其余参数，但当前实现没有使用它们；这里也刻意保持该语义。
    """
    del box_behind_tag, box_beneath_tag, box_left_tag, hand_pitch_degree
    euler = (0.0, -90.0, 0.0)
    left = [
        pose_from_euler((0.3, box_width * 4 / 2, 0.1), euler, "base_link"),
        pose_from_euler((0.5, box_width * 3 / 2, 0.2), euler, "base_link"),
        pose_from_euler((0.5, box_width / 2, 0.2), euler, "base_link"),
        pose_from_euler((0.5, box_width / 2, 0.4), euler, "base_link"),
    ]
    right = [
        pose_from_euler((0.3, -box_width * 4 / 2, 0.1), euler, "base_link"),
        pose_from_euler((0.5, -box_width * 3 / 2, 0.2), euler, "base_link"),
        pose_from_euler((0.5, -box_width / 2, 0.2), euler, "base_link"),
        pose_from_euler((0.5, -box_width / 2, 0.4), euler, "base_link"),
    ]
    return left, right


def _matrix(pose: ScenePose) -> np.ndarray:
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(pose.orientation).as_matrix()
    matrix[:3, 3] = pose.position
    return matrix


def _pose_from_matrix(matrix: np.ndarray, frame: str) -> ScenePose:
    return ScenePose(
        tuple(float(value) for value in matrix[:3, 3]),
        tuple(float(value) for value in Rotation.from_matrix(matrix[:3, :3]).as_quat()),
        frame,
    )


def _compose(parent: ScenePose, child: ScenePose, frame: str) -> ScenePose:
    return _pose_from_matrix(_matrix(parent) @ _matrix(child), frame)


def _identity_transform(source: str, target: str) -> ScenePose:
    return ScenePose((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), target)


def _transform_from_sdk(value: Any, target: str) -> ScenePose:
    if isinstance(value, np.ndarray) and value.shape == (4, 4):
        return _pose_from_matrix(value, target)
    position = getattr(value, "pos", getattr(value, "position", None))
    orientation = getattr(value, "quat", getattr(value, "orientation", None))
    if position is None or orientation is None:
        raise TypeError("TF result must expose pos/quat or position/orientation")
    return ScenePose(tuple(float(x) for x in position), tuple(float(x) for x in orientation), target)


def _lookup_transform(hardware: Any, source: str, target: str) -> ScenePose:
    if source == target:
        return _identity_transform(source, target)
    getter = getattr(hardware, "get_current_transform", None)
    if callable(getter):
        return _transform_from_sdk(
            getter(source_frame=source, target_frame=target), target
        )
    sdk = getattr(hardware, "robot_sdk", None)
    tools = getattr(sdk, "tools", None)
    getter = getattr(tools, "get_tf_transform", None)
    if callable(getter):
        value = getter(target_frame=target, source_frame=source)
        if value is None:
            raise RuntimeError(f"无法获取 {source} 到 {target} 的坐标变换")
        return _transform_from_sdk(value, target)
    raise RuntimeError(f"无法获取 {source} 到 {target} 的坐标变换")


def _to_base(hardware: Any, tag_pose: Any, keypoint: ScenePose) -> SourcePose:
    if keypoint.frame in ("base", "base_link"):
        return SourcePose(keypoint.position, keypoint.orientation)
    if keypoint.frame == "odom":
        in_base = _compose(
            _lookup_transform(hardware, "odom", "base_link"),
            keypoint,
            "base_link",
        )
        return SourcePose(in_base.position, in_base.orientation)
    if keypoint.frame != "tag":
        raise ValueError(f"unsupported keypoint frame: {keypoint.frame}")
    tag = ScenePose(
        (float(tag_pose.x), float(tag_pose.y), float(tag_pose.z)),
        tuple(float(x) for x in _pose_quaternion(tag_pose)),
        "odom",
    )
    in_odom = _compose(tag, keypoint, "odom")
    in_base = _compose(
        _lookup_transform(hardware, "odom", "base_link"),
        in_odom,
        "base_link",
    )
    return SourcePose(in_base.position, in_base.orientation)


def _pose_quaternion(pose: Any) -> Tuple[float, float, float, float]:
    quaternion = getattr(pose, "quat", None)
    if quaternion is not None:
        return tuple(float(x) for x in quaternion)
    # 与源 ``Pose.from_euler`` 一致，使用 intrinsic XYZ (x_angle, y_angle, z_angle)。
    rotation = Rotation.from_euler("xyz", [pose.yaw, pose.pitch, pose.roll])
    return tuple(float(x) for x in rotation.as_quat())


def _elbow_position(
    hardware: Any,
    target: SourcePose,
    is_left: bool,
) -> List[float]:
    """复现源 calculate_elbow_y/get_elbow_position 的联合 IK 输入。"""
    target_y = target.position[1]
    if abs(target_y) < 0.4:
        default_y = 0.4 if is_left else -0.4
    else:
        default_y = target_y + 0.05 if is_left else target_y - 0.05
    link_name = "zarm_l4_link" if is_left else "zarm_r4_link"
    sdk = getattr(hardware, "robot_sdk", None)
    tools = getattr(sdk, "tools", None)
    getter = getattr(tools, "get_link_position", None)
    if callable(getter):
        try:
            if getter(link_name) is not None:
                return [0.05, 0.3 if is_left else -0.3, 0.0]
        except Exception as exc:
            _LOGGER.warning(
                "获取肘关节 %s 失败，使用默认值: %s", link_name, exc
            )
    return [0.0, default_y, 0.0]


def _mirror(joints: List[float], is_left_source: bool) -> List[float]:
    left = list(joints[:7])
    right = list(joints[7:14])
    if is_left_source:
        right = [left[0], -left[1], -left[2], left[3], -left[4], -left[5], left[6]]
    else:
        left = [right[0], -right[1], -right[2], right[3], -right[4], -right[5], right[6]]
    return left + right


def _bezier_segment(start: Sequence[float], end: Sequence[float], points: int) -> List[List[float]]:
    if points < 2:
        raise ValueError("points must be at least two")
    start_array = np.asarray(start, dtype=float)
    end_array = np.asarray(end, dtype=float)
    control = (start_array + end_array) / 2.0 + (end_array - start_array) * 0.1
    result = []
    for t in np.linspace(0.0, 1.0, points):
        value = (1 - t) ** 2 * start_array + 2 * (1 - t) * t * control + t ** 2 * end_array
        result.append([float(x) for x in value])
    return result


def _get_sdk_compat(hardware: Any):
    """获取源 SDK 兼容 facade（延迟导入，可被测试 monkeypatch）。"""
    return __import__(
        "adapters.hardware.leju_wheeled.source_sdk_compat",
        fromlist=["get_source_sdk_compat"],
    ).get_source_sdk_compat(hardware)


def _read_arm_q0(hardware: Any) -> List[float]:
    result = hardware.get_arm_joint_positions()
    if not getattr(result, "success", False):
        raise RuntimeError(getattr(result, "message", "获取关节状态失败"))
    values = [float(x) for x in result.data]
    if len(values) != 14 or not all(math.isfinite(x) for x in values):
        raise ValueError("arm joint state must contain 14 finite radians")
    return values


def plan_source_joint_trajectory(
    hardware: Any,
    tag_pose: Any,
    keypoints: Tuple[Sequence[ScenePose], Sequence[ScenePose]],
    *,
    enable_joint_mirroring: bool = True,
    enable_high_position_accuracy: bool = False,
    traj_point_num: int = 100,
    ik_retry_count: int = 5,
) -> Tuple[List[List[float]], List[List[float]]]:
    """完整执行源 joint 分支；任一点失败时抛出异常且不返回部分轨迹。"""
    left_keypoints, right_keypoints = keypoints
    if len(left_keypoints) != len(right_keypoints) or not left_keypoints:
        raise ValueError("bimanual keypoints must be non-empty and equal length")
    sdk = _get_sdk_compat(hardware)
    current = _read_arm_q0(hardware)
    left_trajectory: List[List[float]] = []
    right_trajectory: List[List[float]] = []
    params = KuavoIKParamsCompat(
        major_optimality_tol=1e-3 if enable_high_position_accuracy else 9e-3,
        major_feasibility_tol=1e-3 if enable_high_position_accuracy else 9e-3,
        minor_feasibility_tol=3e-3 if enable_high_position_accuracy else 9e-3,
        major_iterations_limit=100 if enable_high_position_accuracy else 50,
        oritation_constraint_tol=1e-3 if enable_high_position_accuracy else 19e-3,
        pos_constraint_tol=1e-3 if enable_high_position_accuracy else 9e-3,
        pos_cost_weight=10.0,
        constraint_mode=6 if enable_high_position_accuracy else 0,
    )
    for index, (left_keypoint, right_keypoint) in enumerate(zip(left_keypoints, right_keypoints)):
        left_pose = _to_base(hardware, tag_pose, left_keypoint)
        right_pose = _to_base(hardware, tag_pose, right_keypoint)
        left_elbow = _elbow_position(hardware, left_pose, True)
        right_elbow = _elbow_position(hardware, right_pose, False)
        result = None
        last_error: Optional[Exception] = None
        for retry in range(ik_retry_count):
            try:
                result = sdk.arm.arm_ik(
                    left_pose=left_pose,
                    right_pose=right_pose,
                    left_elbow_pos_xyz=list(left_elbow),
                    right_elbow_pos_xyz=list(right_elbow),
                    arm_q0=list(current),
                    params=params,
                )
                parsed = split_source_arm_ik_result(result)
                if parsed is None:
                    raise RuntimeError("联合 IK 返回失败: None")
                candidate, right_candidate = parsed
                target = candidate + right_candidate
                if enable_joint_mirroring:
                    target = _mirror(target, candidate[1] > 0)
                break
            except Exception as exc:
                last_error = exc
                if retry + 1 < ik_retry_count:
                    left_elbow[1] -= 0.02
                    right_elbow[1] += 0.02
        else:
            raise RuntimeError(f"关键点 {index + 1} 联合 IK 失败: {type(last_error).__name__}: {last_error}")
        segment = _bezier_segment(current, target, traj_point_num)
        points = segment if index == 0 else segment[1:]
        left_trajectory.extend(point[:7] for point in points)
        right_trajectory.extend(point[7:14] for point in points)
        current = list(target)
    if not left_trajectory or len(left_trajectory) != len(right_trajectory):
        raise RuntimeError("生成的双臂轨迹为空或长度不一致")
    return left_trajectory, right_trajectory
