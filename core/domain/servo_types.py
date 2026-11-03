# -*- coding: utf-8 -*-
"""伺服接口契约（飞书文档 §4）。

三个 frozen dataclass 定义了底盘伺服闭环的数据流：
- AlignObservation：视觉感知模块输出 → 控制器/判断模块输入（§4.2）
- ServoCommand：控制器输出 → 执行模块输入（§4.3）

与 dev 视觉链路（node_pallet_servo 的 ServoError）对齐：底盘在地面上只有
2 平移 + 1 旋转三个自由度，观测只带三被控量（两路像素平移偏差 + 偏差角）。
像素→米的换算系数（mm_per_px）不在观测里，由 calc 参数携带（真机标定）。
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class AlignObservation:
    """对齐观测量（飞书 §4.2，对齐 dev 视觉链路 ServoError 的三被控量）。

    由伺服视觉感知模块输出：两路像素平移偏差 + 偏差角 + 有效性。
    字段名与 node_pallet_servo 写到黑板 latest_servo_error 的 ServoError
    一致（e_bottom_px / e_right_px / theta_rad / valid / reject），
    桥接时直接搬运即可。
    """

    e_bottom_px: float          # 图像空间底边方向平移偏差（px，带符号）
    e_right_px: float           # 图像空间右边方向平移偏差（px，带符号）
    theta_rad: float            # 偏差角，带符号，正常 ±45°
    valid: bool                 # 观测是否有效（消费方必须先判）
    reject: str = ""            # 观测无效原因码（valid=False 时给）


def is_observation_like(obj) -> bool:
    """鸭子类型判别：obj 是否带伺服闭环要的三个被控量字段。

    真实视觉链路（node_pallet_servo）写到黑板 latest_servo_error 的是 dev 的
    ServoError，模拟注入节点写的是本模块的 AlignObservation——两者三被控量
    字段同名，只是 ServoError 多一堆诊断量、没有 valid/reject。下游统一按
    本函数判别，两种观测源不用互相转换就能接。
    """
    return all(hasattr(obj, f) for f in ("e_bottom_px", "e_right_px", "theta_rad"))


def observation_field(obj, name, default=None):
    """宽容读观测字段：有就用，没有给默认（ServoError 没有 valid/reject）。"""
    return getattr(obj, name, default)


@dataclass(frozen=True)
class ServoCommand:
    """底盘伺服指令（飞书 §4.3）。

    控制器模块输出，执行模块输入。纯底盘路线，单步位移受 calc 的 max_step_m 限制。
    注意：单步位移必须**明显大于**执行侧的 position_threshold（到达容差），
    否则底盘判「已到达」、一步都不走。

    位置/速度双模：mode 决定后面哪一组字段有意义 —— "position" 用 dx/dy/dtheta，
    "velocity" 用 vx/vy/wz。速度量纲全 SI（m/s、rad/s），与
    hardware.send_base_velocity_tick 的入参口径一致（不经度→弧度换算）。
    mode 默认 "position"，使 chassis_servo_v1 / chassis_servo_real_v1 零改动。
    """

    mode: str = "position"      # "position" | "velocity"，决定下列哪组字段生效
    dx_m: float = 0.0           # 位置模式：底盘 x 位移增量（base 系），上限 = calc 的 max_step_m
    dy_m: float = 0.0           # 位置模式：底盘 y 位移增量
    dtheta_rad: float = 0.0     # 位置模式：底盘旋转增量，弧度
    vx_mps: float = 0.0         # 速度模式：底盘 x 线速度，m/s
    vy_mps: float = 0.0         # 速度模式：底盘 y 线速度，m/s
    wz_radps: float = 0.0       # 速度模式：底盘偏航角速度，rad/s
    iter_index: int = 0         # 当前迭代序号（从 0 起）
    cumulative_xy_m: float = 0.0  # 截至本步的 xy 位移累积量（模）
    stop_reason: str = ""       # ""=正常步；converged/cum_limit/iter_limit/guard
