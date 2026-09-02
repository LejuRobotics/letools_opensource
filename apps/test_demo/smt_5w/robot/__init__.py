"""商超任务的机器人能力层。

该包封装 ROS、IK、底盘、二维码、视觉和末端执行器等基础能力。业务状态机
只依赖这里导出的稳定接口，不直接导入 ``rospy`` 或旧 ``psdk``。

所有 ROS 依赖均在 RobotIO 实例化或能力首次使用时延迟加载，因此本包可以在
没有 ROS 的开发机上安全导入和测试。
"""

from .arm import (
    ArmControlError,
    ArmController,
    ArmReadinessError,
    IkSolution,
    IkSolveError,
    TrajectoryExecutionError,
)
from .end_effector import (
    EndEffectorCommandError,
    EndEffectorConfigError,
    EndEffectorController,
    EndEffectorError,
    EndEffectorRuntimeError,
    EndEffectorTimeout,
    normalize_end_effector_type,
)
from .motion import (
    BaseMotionController,
    HumanoidMotionController,
    MotionError,
    WheelMotionController,
    build_motion_controller,
)
from .qr import (
    QRAlignmentError,
    QRError,
    QRObservation,
    QRRecognizer,
    QRRuntimeError,
    QRScanTimeout,
)
from .robot_io import (
    HumanoidRobotIO,
    RobotIO,
    RobotControlError,
    RosDependencyError,
    RosOperationError,
    RosReadinessError,
    WheelRobotIO,
    build_robot_io,
)
from .vision import (
    ObjectObservation,
    ObjectPositionTracker,
    VisionError,
    VisionRuntimeError,
    VisionTimeoutError,
)


__all__ = [
    "ArmControlError",
    "ArmController",
    "ArmReadinessError",
    "BaseMotionController",
    "EndEffectorCommandError",
    "EndEffectorConfigError",
    "EndEffectorController",
    "EndEffectorError",
    "EndEffectorRuntimeError",
    "EndEffectorTimeout",
    "HumanoidMotionController",
    "HumanoidRobotIO",
    "IkSolution",
    "IkSolveError",
    "MotionError",
    "ObjectObservation",
    "ObjectPositionTracker",
    "QRAlignmentError",
    "QRError",
    "QRObservation",
    "QRRecognizer",
    "QRRuntimeError",
    "QRScanTimeout",
    "RobotControlError",
    "RosDependencyError",
    "RosOperationError",
    "RosReadinessError",
    "RobotIO",
    "TrajectoryExecutionError",
    "VisionError",
    "VisionRuntimeError",
    "VisionTimeoutError",
    "WheelMotionController",
    "WheelRobotIO",
    "build_motion_controller",
    "build_robot_io",
    "normalize_end_effector_type",
]
