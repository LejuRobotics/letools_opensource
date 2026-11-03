"""坐标系变换模型（纯数据 + 纯数学，零 ROS 依赖）。

``Transform`` 表示 ``source_frame`` 在 ``target_frame`` 下的位姿，也就是
``get_transform(target_frame, source_frame)`` 的返回值：

- ``position``：位置 [x, y, z]，单位米
- ``orientation``：四元数 [qx, qy, qz, qw]

配套的 ``apply`` / ``apply_inverse`` 用于在两个坐标系之间搬运位姿，
位姿一律为 7 维 ``[x, y, z, qx, qy, qz, qw]``。
"""

import math
from dataclasses import dataclass
from typing import List, Sequence, Tuple

Quaternion = Tuple[float, float, float, float]


def _normalize(quaternion: Sequence[float]) -> Quaternion:
    """归一化四元数；非法输入直接抛 ValueError（不返回静默的坏值）。"""
    if len(quaternion) != 4:
        raise ValueError(f"四元数需要 4 个分量，收到 {len(quaternion)} 个")
    norm = math.sqrt(sum(float(v) * float(v) for v in quaternion))
    if norm < 1e-12 or not math.isfinite(norm):
        raise ValueError(f"无效四元数: {list(quaternion)}")
    return tuple(float(v) / norm for v in quaternion)  # type: ignore[return-value]


def _multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _conjugate(quaternion: Quaternion) -> Quaternion:
    x, y, z, w = quaternion
    return (-x, -y, -z, w)


def rotate_vector(orientation: Sequence[float], vector: Sequence[float]) -> Tuple[float, float, float]:
    """用四元数旋转三维向量。"""
    x, y, z, w = _normalize(orientation)
    vx, vy, vz = float(vector[0]), float(vector[1]), float(vector[2])
    # v' = v + w*t + q_v × t，其中 t = 2 * (q_v × v)
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def quat_angle_deg(a: Sequence[float], b: Sequence[float]) -> float:
    """两个四元数的夹角（度，0~180）。取绝对值，q 与 -q 视为同一姿态。"""
    qa = _normalize(a)
    qb = _normalize(b)
    dot = abs(sum(qa[index] * qb[index] for index in range(4)))
    return 2.0 * math.degrees(math.acos(min(1.0, max(-1.0, dot))))


def _split_pose(pose: Sequence[float]) -> Tuple[Tuple[float, float, float], Quaternion]:
    """把 7 维位姿拆成 (位置, 四元数)。"""
    if len(pose) != 7:
        raise ValueError(f"位姿需要 7 个分量 [x,y,z,qx,qy,qz,qw]，收到 {len(pose)} 个")
    return (
        (float(pose[0]), float(pose[1]), float(pose[2])),
        _normalize(pose[3:7]),
    )


@dataclass(frozen=True)
class Transform:
    """``source_frame`` 在 ``target_frame`` 下的位姿。"""

    position: Tuple[float, float, float]
    orientation: Quaternion

    @staticmethod
    def from_lists(position: Sequence[float], orientation: Sequence[float]) -> "Transform":
        """从 [x,y,z] + [qx,qy,qz,qw] 构造。"""
        if len(position) != 3:
            raise ValueError(f"位置需要 3 个分量，收到 {len(position)} 个")
        return Transform(
            position=(float(position[0]), float(position[1]), float(position[2])),
            orientation=_normalize(orientation),
        )

    def to_pose_list(self) -> List[float]:
        """转成 7 维位姿 ``[x, y, z, qx, qy, qz, qw]``。"""
        return list(self.position) + list(self.orientation)

    def to_euler_zyx(self) -> Tuple[float, float, float]:
        """转成 ``(yaw, pitch, roll)``，弧度。

        顺序与 MPC 离线轨迹 ``cmdVec`` 的姿态字段一致（绕 Z、Y、X 依次）。
        奇异位形（pitch=±90°）下 roll 与 yaw 不可分，本式取 roll=0 分支。
        """
        x, y, z, w = self.orientation
        r00 = 1.0 - 2.0 * (y * y + z * z)
        r10 = 2.0 * (x * y + z * w)
        r20 = 2.0 * (x * z - y * w)
        r21 = 2.0 * (y * z + x * w)
        r22 = 1.0 - 2.0 * (x * x + y * y)
        yaw = math.atan2(r10, r00)
        pitch = math.atan2(-r20, math.hypot(r00, r10))
        roll = math.atan2(r21, r22)
        return yaw, pitch, roll

    def apply(self, pose: Sequence[float]) -> List[float]:
        """``source_frame`` 下的位姿 → ``target_frame`` 下的位姿。"""
        position, orientation = _split_pose(pose)
        rotated = rotate_vector(self.orientation, position)
        new_position = tuple(rotated[i] + self.position[i] for i in range(3))
        new_orientation = _multiply(self.orientation, orientation)
        return list(new_position) + list(new_orientation)

    def apply_inverse(self, pose: Sequence[float]) -> List[float]:
        """``target_frame`` 下的位姿 → ``source_frame`` 下的位姿。"""
        position, orientation = _split_pose(pose)
        relative = tuple(position[i] - self.position[i] for i in range(3))
        inverse_orientation = _conjugate(self.orientation)
        new_position = rotate_vector(inverse_orientation, relative)
        new_orientation = _multiply(inverse_orientation, orientation)
        return list(new_position) + list(new_orientation)
