"""灵巧手与乐聚夹爪的统一末端执行器接口。

ROS 消息类型、通信端点与硬超时由 RobotIO 统一管理。任何配置、连接或响应
异常都会抛出明确异常，不再静默返回失败。
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

from .robot_io import RosReadinessError


LEJUCLAW_SERVICE = "/control_robot_leju_claw"
LEJUCLAW_NAMES = ["left_claw", "right_claw"]
LEJUCLAW_OPEN_POSITION = [10.0, 10.0]
LEJUCLAW_CLOSE_POSITION = [90.0, 90.0]
LEJUCLAW_VELOCITY = [50.0, 50.0]
LEJUCLAW_EFFORT = [1.0, 1.0]
DEFAULT_SERVICE_TIMEOUT = 3.0
DEFAULT_PUBLISHER_TIMEOUT = 2.0
DEFAULT_ZERO_HAND = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
DEFAULT_OPEN_HAND = [0.0, 100.0, 0.0, 0.0, 0.0, 0.0]
DEFAULT_CLOSE_HAND = [80.0, 100.0, 80.0, 75.0, 75.0, 75.0]


class EndEffectorError(RuntimeError):
    """末端执行器基础异常。"""


class EndEffectorConfigError(EndEffectorError, ValueError):
    """末端类型或配置值无效。"""


class EndEffectorRuntimeError(EndEffectorError):
    """RobotIO 缺少必要 ROS 能力。"""


class EndEffectorTimeout(EndEffectorError):
    """等待服务、订阅者或服务响应超时。"""


class EndEffectorCommandError(EndEffectorError):
    """服务/发布调用明确失败。"""


def normalize_end_effector_type(value: Any) -> str:
    """把末端类型别名归一化为 ``qiangnao`` 或 ``lejuclaw``。"""

    normalized = str(value or "qiangnao").strip().lower()
    if normalized in ("qiangnao", "dexterous", "hand", "dexterous_hand"):
        return "qiangnao"
    if normalized in ("lejuclaw", "leju_claw", "claw"):
        return "lejuclaw"
    raise EndEffectorConfigError("不支持的末端类型: %s" % normalized)


def lejuclaw_config(config: Optional[Mapping[str, Any]] = None) -> Dict[str, Any]:
    """读取并严格校验乐聚夹爪配置。"""

    root = config if isinstance(config, Mapping) else {}
    end_effector = root.get("end_effector", {})
    end_effector = end_effector if isinstance(end_effector, Mapping) else {}
    claw = end_effector.get("lejuclaw", {})
    claw = claw if isinstance(claw, Mapping) else {}

    result = {
        "service": str(claw.get("service", LEJUCLAW_SERVICE)),
        "service_timeout": _positive_float(
            claw.get("service_timeout", DEFAULT_SERVICE_TIMEOUT),
            "end_effector.lejuclaw.service_timeout",
        ),
        "open_position": _numeric_vector(
            claw.get("open_position", LEJUCLAW_OPEN_POSITION),
            2,
            "end_effector.lejuclaw.open_position",
        ),
        "close_position": _numeric_vector(
            claw.get("close_position", LEJUCLAW_CLOSE_POSITION),
            2,
            "end_effector.lejuclaw.close_position",
        ),
        "velocity": _numeric_vector(
            claw.get("velocity", LEJUCLAW_VELOCITY),
            2,
            "end_effector.lejuclaw.velocity",
            minimum=0.0,
        ),
        "effort": _numeric_vector(
            claw.get("effort", LEJUCLAW_EFFORT),
            2,
            "end_effector.lejuclaw.effort",
            minimum=0.0,
        ),
    }
    if not result["service"]:
        raise EndEffectorConfigError("乐聚夹爪 service 不能为空")
    return result


def lejuclaw_hand_positions(
    hand: Any,
    action: Any,
    config: Optional[Mapping[str, Any]] = None,
) -> Tuple[List[float], List[float], List[float]]:
    """计算单侧夹爪动作对应的双夹爪位置、速度和力矩。"""

    return _lejuclaw_hand_positions(
        hand,
        action,
        lejuclaw_config(config),
    )


def _lejuclaw_hand_positions(
    hand: Any,
    action: Any,
    claw: Mapping[str, Any],
) -> Tuple[List[float], List[float], List[float]]:
    normalized_hand = _normalize_hand(hand)
    normalized_action = _normalize_action(action)
    index = 0 if normalized_hand == "left" else 1
    positions = list(claw["open_position"])
    positions[index] = (
        claw["close_position"][index]
        if normalized_action == "close"
        else claw["open_position"][index]
    )
    return positions, list(claw["velocity"]), list(claw["effort"])


class EndEffectorController:
    """通过 RobotIO 控制灵巧手或乐聚夹爪。

    Args:
        robot_io: 机器人 IO，必须实现
            ``publish_hand_target_pos`` 和 ``call_lejuclaw`` 正式接口。
        config: 完整任务配置。
    """

    def __init__(self, robot_io: Any, config: Optional[Mapping[str, Any]] = None):
        if robot_io is None:
            raise EndEffectorRuntimeError("EndEffectorController 必须注入 robot_io")
        self.robot_io = robot_io
        inherited = robot_io.params
        self.config = dict(config if config is not None else (inherited or {}))
        end_effector = self.config.get("end_effector", {})
        end_effector = end_effector if isinstance(end_effector, Mapping) else {}
        configured_type = self.config.get("end_effector_type") or end_effector.get("type")
        self.end_effector_type = normalize_end_effector_type(configured_type)
        self.claw_config = lejuclaw_config(self.config)
        self.publisher_timeout = _positive_float(
            end_effector.get("publisher_timeout", DEFAULT_PUBLISHER_TIMEOUT),
            "end_effector.publisher_timeout",
        )

    def command(
        self,
        hand: Any,
        action: Any,
        close_hand_pose: Optional[Sequence[float]] = None,
        timeout: Optional[float] = None,
        end_effector_type: Any = None,
    ) -> Any:
        """按指定末端类型执行单手打开或闭合。

        ``end_effector_type`` 允许不同物品选择不同末端；未指定时使用全局配置。
        类型分支集中在 SDK 内，Agent 不再判断灵巧手或夹爪。
        """

        selected_type = normalize_end_effector_type(
            self.end_effector_type
            if end_effector_type is None
            else end_effector_type
        )
        if selected_type == "lejuclaw":
            return self.command_claw_for_hand(hand, action, timeout=timeout)
        return self.command_dexterous_hand(
            hand,
            action,
            close_hand_pose=close_hand_pose,
        )

    def open(
        self,
        hand: Any,
        timeout: Optional[float] = None,
        end_effector_type: Any = None,
    ) -> Any:
        """打开指定一侧末端。"""

        return self.command(
            hand,
            "open",
            timeout=timeout,
            end_effector_type=end_effector_type,
        )

    def close(
        self,
        hand: Any,
        close_hand_pose: Optional[Sequence[float]] = None,
        timeout: Optional[float] = None,
        end_effector_type: Any = None,
    ) -> Any:
        """闭合指定一侧末端。"""

        return self.command(
            hand,
            "close",
            close_hand_pose=close_hand_pose,
            timeout=timeout,
            end_effector_type=end_effector_type,
        )

    def command_claw_for_hand(
        self,
        hand: Any,
        action: Any,
        timeout: Optional[float] = None,
    ) -> Any:
        """下发单侧夹爪动作，另一侧保持打开配置。"""

        positions, velocity, effort = _lejuclaw_hand_positions(
            hand,
            action,
            self.claw_config,
        )
        return self.command_claw_positions(
            positions,
            velocity,
            effort,
            timeout=timeout,
        )

    def command_claw_positions(
        self,
        positions: Sequence[float],
        velocity: Optional[Sequence[float]] = None,
        effort: Optional[Sequence[float]] = None,
        timeout: Optional[float] = None,
    ) -> Any:
        """通过有界服务调用下发双夹爪位置。"""

        positions_value = _numeric_vector(positions, 2, "positions")
        velocity_value = _numeric_vector(
            self.claw_config["velocity"] if velocity is None else velocity,
            2,
            "velocity",
            minimum=0.0,
        )
        effort_value = _numeric_vector(
            self.claw_config["effort"] if effort is None else effort,
            2,
            "effort",
            minimum=0.0,
        )
        timeout_value = self.claw_config["service_timeout"] if timeout is None else _positive_float(timeout, "timeout")

        try:
            response = self.robot_io.call_lejuclaw(
                positions_value,
                velocity_value,
                effort_value,
                timeout=timeout_value,
                service_name=self.claw_config["service"],
            )
        except RosReadinessError as exc:
            raise EndEffectorTimeout("乐聚夹爪服务调用超时: %s" % exc) from exc
        except Exception as exc:
            raise EndEffectorCommandError("乐聚夹爪服务调用失败: %s" % exc) from exc
        return _validate_response(response, "乐聚夹爪")

    def command_dexterous_hand(
        self,
        hand: Any,
        action: Any,
        close_hand_pose: Optional[Sequence[float]] = None,
    ) -> bool:
        """发布灵巧手双手位置，仅目标手执行动作，另一手使用 zero 配置。"""

        normalized_hand = _normalize_hand(hand)
        normalized_action = _normalize_action(action)
        hand_config = _hand_config(self.config)
        if normalized_action == "open":
            target = hand_config["open_hand"]
        elif close_hand_pose is not None:
            target = _numeric_vector(close_hand_pose, 6, "close_hand_pose")
        else:
            target = hand_config["close_hand"]
        zero = hand_config["zero_hand"]
        left, right = (target, zero) if normalized_hand == "left" else (zero, target)
        return self.publish_hand_positions(left, right)

    def publish_hand_positions(
        self,
        left_hand: Sequence[float],
        right_hand: Sequence[float],
    ) -> bool:
        """发布灵巧手位置；发布接口缺失或显式失败时抛出异常。"""

        left = _numeric_vector(left_hand, 6, "left_hand")
        right = _numeric_vector(right_hand, 6, "right_hand")
        try:
            result = self.robot_io.publish_hand_target_pos(
                left,
                right,
                timeout=self.publisher_timeout,
            )
        except RosReadinessError as exc:
            raise EndEffectorTimeout("等待灵巧手 Publisher 超时: %s" % exc) from exc
        except Exception as exc:
            raise EndEffectorCommandError("发布灵巧手位置失败: %s" % exc) from exc
        if result is False:
            raise EndEffectorCommandError("publish_hand_target_pos 返回 False")
        return True


def _validate_response(response: Any, label: str) -> Any:
    if response is None or response is False:
        raise EndEffectorCommandError("%s返回空或 False 响应" % label)
    for field in ("success", "result"):
        if hasattr(response, field) and not bool(getattr(response, field)):
            message = getattr(response, "message", getattr(response, "error_reason", ""))
            raise EndEffectorCommandError("%s执行失败: %s" % (label, message))
    return response


def _hand_config(config: Mapping[str, Any]) -> Dict[str, List[float]]:
    hand = config.get("hand", {}) if isinstance(config, Mapping) else {}
    hand = hand if isinstance(hand, Mapping) else {}
    return {
        "zero_hand": _numeric_vector(hand.get("zero_hand", DEFAULT_ZERO_HAND), 6, "hand.zero_hand"),
        "open_hand": _numeric_vector(hand.get("open_hand", DEFAULT_OPEN_HAND), 6, "hand.open_hand"),
        "close_hand": _numeric_vector(hand.get("close_hand", DEFAULT_CLOSE_HAND), 6, "hand.close_hand"),
    }


def _normalize_hand(value: Any) -> str:
    normalized = str(value or "").strip().lower()
    if normalized in ("left", "l", "左", "左手"):
        return "left"
    if normalized in ("right", "r", "右", "右手"):
        return "right"
    raise EndEffectorConfigError("hand 必须是 left 或 right，当前=%r" % (value,))


def _normalize_action(value: Any) -> str:
    normalized = str(value or "").strip().lower()
    if normalized in ("open", "打开", "松开"):
        return "open"
    if normalized in ("close", "闭合", "抓取"):
        return "close"
    raise EndEffectorConfigError("action 必须是 open 或 close，当前=%r" % (value,))


def _numeric_vector(
    values: Sequence[Any],
    length: int,
    name: str,
    minimum: Optional[float] = None,
) -> List[float]:
    if isinstance(values, (str, bytes)):
        raise EndEffectorConfigError("%s 必须是长度 %d 的数值序列" % (name, length))
    try:
        result = [float(value) for value in values]
    except (TypeError, ValueError) as exc:
        raise EndEffectorConfigError("%s 必须是数值序列" % name) from exc
    if len(result) != length:
        raise EndEffectorConfigError("%s 必须恰好包含 %d 个值" % (name, length))
    if not all(math.isfinite(value) for value in result):
        raise EndEffectorConfigError("%s 包含非有限值" % name)
    if minimum is not None and any(value < minimum for value in result):
        raise EndEffectorConfigError("%s 的值不能小于 %s" % (name, minimum))
    return result


def _positive_float(value: Any, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise EndEffectorConfigError("%s 必须是数字" % name) from exc
    if not math.isfinite(result) or result <= 0.0:
        raise EndEffectorConfigError("%s 必须是有限正数" % name)
    return result


__all__ = [
    "DEFAULT_SERVICE_TIMEOUT",
    "EndEffectorCommandError",
    "EndEffectorConfigError",
    "EndEffectorController",
    "EndEffectorError",
    "EndEffectorRuntimeError",
    "EndEffectorTimeout",
    "LEJUCLAW_CLOSE_POSITION",
    "LEJUCLAW_EFFORT",
    "LEJUCLAW_NAMES",
    "LEJUCLAW_OPEN_POSITION",
    "LEJUCLAW_SERVICE",
    "LEJUCLAW_VELOCITY",
    "lejuclaw_config",
    "lejuclaw_hand_positions",
    "normalize_end_effector_type",
]
