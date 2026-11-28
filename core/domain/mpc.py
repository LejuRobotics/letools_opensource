"""MPC三种独立配置的公共整数契约，不依赖SDK枚举。"""

MPC_VALUES = {
    "mode": {0: "NoControl", 1: "ArmOnly", 2: "BaseOnly", 3: "BaseArm"},
    "flow": {0: "ThroughFullBodyMpc", 1: "DirectToWbc"},
    "frame": {0: "KeepCurrentFrame", 1: "WorldFrame", 2: "LocalFrame",
              3: "VRFrame", 4: "ManipulationWorldFrame"},
}


def validate_mpc_value(kind: str, value: int) -> int:
    """只接受严格整数，不接受bool/字符串/浮点数/错误枚举值。"""
    if kind not in MPC_VALUES:
        raise ValueError("未知MPC配置类型")
    if type(value) is not int or value not in MPC_VALUES[kind]:
        raise ValueError(f"{kind} 必须为整数 {list(MPC_VALUES[kind])}，不接受bool")
    return value
