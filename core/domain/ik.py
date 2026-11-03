"""双臂 IK 求解的请求与解（纯数据模型，零 ROS 依赖）。"""

from dataclasses import dataclass
from typing import List, Optional


@dataclass
class IkSolveRequest:
    """一次双臂 IK 求解请求，字段对齐源脚本 ``arms_ik_api.build_two_arm_cmd()``。

    位姿一律四元数 ``[x, y, z, qx, qy, qz, qw]``（7 维、米），坐标原点是 IK 模型的
    根坐标系——轮臂为 ``waist_yaw_link``（手臂安装系），**不是** ``base_link``，
    两者相差约 1m。

    - ``left_joint_angles`` / ``right_joint_angles``：7 维 q0 初值（弧度）。
      ``None`` 表示由适配器读取当前关节角；连续求解时传上一帧解可保持分支连续。
    - ``max_joint_step_deg``：单个关节相对 q0 的最大允许跳变；超过视为不连续解，
      主服务的结果被丢弃并回退备用服务。
    - ``constraint_mode``：转发给 IK 服务的约束模式（源脚本默认 3）。
    """

    left_pose: List[float]
    right_pose: List[float]
    left_joint_angles: Optional[List[float]] = None
    right_joint_angles: Optional[List[float]] = None
    joint_angles_as_q0: bool = True
    max_joint_step_deg: float = 8.0
    constraint_mode: int = 3


@dataclass
class IkSolution:
    """IK 求解结果。

    ``left_hand_pose`` / ``right_hand_pose`` 是服务端对自己返回的关节解做的 FK，
    同样是 7 维 ``[x, y, z, qx, qy, qz, qw]``，用于衡量求解精度。
    """

    q_arm: List[float]                  # 14 维关节角（弧度）
    left_hand_pose: List[float]         # 7 维
    right_hand_pose: List[float]        # 7 维
    elapsed_ms: float                   # 服务调用耗时（毫秒）
    solver: str = "multi"               # 'multi' 主服务 / 'single' 回退服务
