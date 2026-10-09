# -*- coding: utf-8 -*-
"""托盘观测的配对与平滑：两路观测（托盘位姿 + 箱子四角）按图像时间戳配成对，
再在成对序列上做窗。

**为什么要配对**：伺服算的是「箱子相对托盘」的位置，而托盘与箱子的检测耗时
可能差一个量级（箱子实测 ~60ms）。两者必须来自**同一帧图像**才能相减 ——
否则误差里混着相机运动，而三个数照样算得出来（静默错）。

`algorithm.py` 是纯函数库（零框架依赖、零 ROS），`window.py` 是**有状态**的
`PalletFrameWindow`（也不 import ROS）。

⚠️ 本模块**不是** `box_frame/window.py` 的扩展：那个管「一路观测 + 多帧平均」，
本模块管「两路观测 + 配对」，是两件事。
"""
from .algorithm import (  # noqa: F401
    PairedFrame,
    PalletObservation,
    PairReject,
    average_pairs,
    nearest_pair,
)
from .window import PalletFrameWindow  # noqa: F401

__all__ = [
    "PairedFrame",
    "PalletFrameWindow",
    "PalletObservation",
    "PairReject",
    "average_pairs",
    "nearest_pair",
]
