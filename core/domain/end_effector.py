# LeTools/core/domain/end_effector.py
import math
from enum import Enum
from dataclasses import dataclass, field
from typing import List, Optional

class EndEffectorType(Enum):
    """末端执行器类型"""
    LEJU_CLAW = "leju_claw"       # 乐聚二指夹爪
    QIANGNAO_HAND = "qiangnao"    # 强脑灵巧手
    SG100_HAND = "sg100"          # 黑漫 SG100 11-DOF 灵巧手
    SUCTION_CUP = "suction_cup"   # 吸盘
    NONE = "none"                 # 无末端

class GripperStatus(Enum):
    """夹爪/手部状态 (对应 lejuClawState.msg)"""
    ERROR = -1
    UNKNOWN = 0
    MOVING = 1
    REACHED = 2
    GRABBED = 3

@dataclass
class GripperCommand:
    """
    通用夹爪控制指令。
    适用于二指夹爪或简单的开合动作。
    """
    position: float = 0.0   # 行程占比 [0, 100], 0为张开, 100为闭合
    velocity: float = 50.0  # 速度 [0, 100]
    effort: float = 1.0     # 力矩/电流 (A)


@dataclass
class DualGripperCommand:
    """双侧夹爪命令；位置为 ``None`` 的一侧不参与本次控制。"""
    left_position: Optional[float] = None
    right_position: Optional[float] = None
    velocity: float = 50.0
    effort: float = 1.0

@dataclass
class HandFingerCommand:
    """
    灵巧手手指控制指令。
    对应 robotHandPosition.msg，通常包含6个手指关节的位置。
    """
    positions: List[float] = field(default_factory=lambda: [0.0] * 6) # [0, 100]

@dataclass
class EndEffectorState:
    """
    末端执行器实时状态。
    """
    status: GripperStatus = GripperStatus.UNKNOWN
    current_position: float = 0.0
    current_velocity: float = 0.0
    current_effort: float = 0.0
    finger_positions: Optional[List[float]] = None # 仅灵巧手有效

@dataclass
class DualEndEffectorState:
    """
    双臂末端状态封装。
    """
    left: EndEffectorState = field(default_factory=EndEffectorState)
    right: EndEffectorState = field(default_factory=EndEffectorState)


# 每只 SG100 手的关节数
SG100_JOINT_COUNT = 11

# 关节位置软限位（rad）。SG100 各关节物理限位约 ±1.5~2.0 rad，
# 保守取 π≈3.14 rad 既能挡住 [0,100] 归一化误用（100 rad≈5729°），
# 又不过度限制合法弧度目标。精确逐关节限位应由驱动侧读取 joint_angle_map。
SG100_POSITION_LIMIT = math.pi


class SG100ControlMode(Enum):
    """
    SG100 灵巧手控制模式。

    编号与 SG100 SDK ``CtrlMode_e`` 及 ``SG100HandCommand.msg`` 常量一致，
    硬件驱动仅支持 MODE_JOINT_POSITION(7) 与 MODE_JOINT_IMPEDANCE(9)。
    """
    JOINT_POSITION = 7    # 位置控制
    JOINT_IMPEDANCE = 9  # 阻抗/力控（需配合 kp/kd/torque_ff）


@dataclass(frozen=True)
class SG100HandCommand:
    """
    SG100 11-DOF 灵巧手单手控制指令。

    关节位置单位为**弧度 rad**（非 [0,100] 归一化），关节顺序固定为：
    [TH_CMC_ABD, TH_MCP_FLEX, TH_IP_FLEX,
     IF_MCP_ABD, IF_MCP_FLEX, IF_PIP_FLEX,
     MF_MCP_FLEX, MF_PIP_FLEX,
     LF_MCP_ABD, LF_MCP_FLEX, LF_PIP_FLEX]

    :param positions: 11 关节目标位置（rad）
    :param control_mode: 整手控制模式（标量回退），默认位置控制
    :param enable_mask: 关节使能位掩码，0x07FF=全部 11 关节
    :param kp: 每关节刚度（仅 JOINT_IMPEDANCE 有效），11 元素或 None
    :param kd: 每关节阻尼（仅 JOINT_IMPEDANCE 有效），11 元素或 None
    :param torque_ff: 每关节前馈力矩 Nm（仅 JOINT_IMPEDANCE 有效），11 元素或 None
    :param per_joint_modes: 每关节独立控制模式（混合模式），11 元素或 None。
        非 None 时覆盖标量 control_mode，允许同一只手的不同关节用不同模式
        （如拇指位置控制 + 食指阻抗控制）。阻抗类关节须在 kp/kd 中提供增益。

    .. note::
        frozen=True 保证构造后不可篡改字段绕过校验。校验仅在构造时触发。
        JOINT_IMPEDANCE 模式必须同时提供 kp 与 kd，否则关节将因零刚度失控下垂。
    """
    positions: List[float] = field(default_factory=lambda: [0.0] * SG100_JOINT_COUNT)
    control_mode: SG100ControlMode = SG100ControlMode.JOINT_POSITION
    enable_mask: int = 0x07FF
    kp: Optional[List[float]] = None
    kd: Optional[List[float]] = None
    torque_ff: Optional[List[float]] = None
    per_joint_modes: Optional[List[SG100ControlMode]] = None

    def __post_init__(self) -> None:
        """构造时校验关节参数，尽早暴露配置错误。"""
        if len(self.positions) != SG100_JOINT_COUNT:
            raise ValueError(
                f"SG100 positions 长度须为 {SG100_JOINT_COUNT}，实际为 {len(self.positions)}"
            )
        for v in self.positions:
            if not math.isfinite(v):
                raise ValueError(f"SG100 positions 含非有限值: {v}")
            if abs(v) > SG100_POSITION_LIMIT:
                raise ValueError(
                    f"SG100 positions 含超限值 {v} rad（限位 ±{SG100_POSITION_LIMIT} rad）；"
                    "请确认单位为弧度而非 [0,100] 归一化值"
                )
        for name in ("kp", "kd", "torque_ff"):
            gains = getattr(self, name)
            if gains is not None and len(gains) != SG100_JOINT_COUNT:
                raise ValueError(
                    f"SG100 {name} 长度须为 {SG100_JOINT_COUNT}，实际为 {len(gains)}"
                )
        if not isinstance(self.control_mode, SG100ControlMode):
            # 兼容从 JSON 反序列化得到的 int 值（frozen dataclass 需用 object.__setattr__）
            object.__setattr__(self, "control_mode", SG100ControlMode(self.control_mode))
        # 归一化 per_joint_modes：兼容 int 值
        if self.per_joint_modes is not None:
            if len(self.per_joint_modes) != SG100_JOINT_COUNT:
                raise ValueError(
                    f"per_joint_modes 长度须为 {SG100_JOINT_COUNT}，"
                    f"实际为 {len(self.per_joint_modes)}"
                )
            normalized = []
            for i, m in enumerate(self.per_joint_modes):
                if not isinstance(m, SG100ControlMode):
                    try:
                        m = SG100ControlMode(m)
                    except ValueError:
                        raise ValueError(
                            f"per_joint_modes[{i}]={m!r} 不是有效控制模式"
                        )
                normalized.append(m)
            object.__setattr__(self, "per_joint_modes", normalized)
        # 安全校验：所有走到阻抗的关节都必须有 kp/kd 增益
        # 混合模式下可能只有部分关节阻抗，标量模式下整手阻抗
        impedance_joints = self.impedance_joint_indices()
        if impedance_joints:
            missing = []
            if self.kp is None:
                missing.append("kp")
            if self.kd is None:
                missing.append("kd")
            if missing:
                raise ValueError(
                    f"关节 {impedance_joints} 为 JOINT_IMPEDANCE 模式但未提供 "
                    f"{' 与 '.join(missing)}，否则关节将因零刚度/零阻尼失控下垂"
                )

    def impedance_joint_indices(self) -> List[int]:
        """返回所有走 JOINT_IMPEDANCE 模式的关节索引。

        公共方法：driver 的安全闸门与增益转发均依赖此判定，故公开
        （非下划线约定），core 重命名时编译期即可发现调用点。
        """
        if self.per_joint_modes is not None:
            return [
                i for i, m in enumerate(self.per_joint_modes)
                if m == SG100ControlMode.JOINT_IMPEDANCE
            ]
        if self.control_mode == SG100ControlMode.JOINT_IMPEDANCE:
            return list(range(SG100_JOINT_COUNT))
        return []
