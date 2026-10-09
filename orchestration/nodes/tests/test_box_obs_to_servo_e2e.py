# -*- coding: utf-8 -*-
"""端到端合成：**ROS 消息 → NodeBoxObs → 黑板 → NodePalletServo → 三个误差量**。

零 maduo 数据、零硬件、零 ROS、零 `box_detection_msgs` —— 全在 CI 里跑
（`verify:opensource` 跑的正是 `pytest orchestration/nodes/tests/ -m unit`）。

这是整条链路**第一条真正的端到端测试**。它钉住的是**接缝**：箱子节点写出来的
东西，必须和伺服节点手头那份契约（`BoxObservation`）**完全是同一个东西**。
两条路径各跑一遍、三个量逐位比对 —— 任何一边改了形状（字段名、四角顺序、
AABB 合成方式、"同一观测不重写"的门禁），这条都会红。

```
  BoxDetection 消息 ──┐                        ┌── 直接构造的 BoxObservation
                      │                        │
              NodeBoxObs._on_message()          │（不经过节点）
                      │                        │
              黑板 latest_box_obs         黑板 box_direct
                      │                        │
                      └──→ NodePalletServo ←───┘
                                  │
                        latest_servo_error
                        （两条路径的三个量必须逐位相同）
```

运行：
    pytest orchestration/nodes/tests/test_box_obs_to_servo_e2e.py -m unit -v
"""
import math
from types import SimpleNamespace

import numpy as np
import py_trees
import pytest
from py_trees.common import Access, Status

from core.common.transform import matrix_to_pose6d
from orchestration.nodes.node_box_obs import NodeBoxObs
from orchestration.nodes.node_pallet_servo import NodePalletServo
from skills.atomic.perception.pallet_servo.algorithm import BoxObservation

pytestmark = pytest.mark.unit

FX = FY = 1000.0
CX, CY = 640.0, 400.0
PALLET_W_MM, PALLET_H_MM = 1200.0, 800.0

# 一个**转过角**的箱子四角，契约顺序：右下 → 左下 → 左上 → 右上。
# 转着才有意义 —— 轴对齐的框从 AABB 也能合成，区分不出"旋转恢复没恢复"。
QUAD = [[980.0, 1080.0], [700.0, 1120.0], [680.0, 800.0], [960.0, 760.0]]


def _straight_down() -> np.ndarray:
    """托盘在相机正上方 1 m、垂直向下看；FX=1000 时 1 mm = 1 px。"""
    T = np.eye(4)
    T[2, 3] = 1.0
    return T


def _box_detection(corners, valid=True, source="window", stamp=1.0):
    """`box_detection_msgs/BoxDetection` 的形状。"""
    def pt(u, v):
        return SimpleNamespace(x=u, y=v)
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda: stamp)),
        corners_uv=[pt(u, v) for u, v in corners],
        valid=valid, source=source, spread_px=1.5, rejects="",
        n_used=5, n_slots=5, n_failed=0, angle_deg=1.05,
        box_uv=[pt(700.0, 800.0), pt(980.0, 1120.0)], latency_ms=66.0)


def _pallet_pose():
    return matrix_to_pose6d(_straight_down())


def _writer(*keys):
    client = py_trees.blackboard.Client(name="e2e_upstream")
    for key in keys:
        client.register_key(key=key, access=Access.WRITE)
    return client


def _make_servo(box_key, out_key):
    node = NodePalletServo("servo", "servo", "ns", {
        "ref_edges": ["y=0", "x=W"],
        "pallet_size_mm": [PALLET_W_MM, PALLET_H_MM],
        "K": [[FX, 0.0, CX], [0.0, FY, CY], [0.0, 0.0, 1.0]],
        "image_size": [3840, 2400],
        "box_key": box_key,
        "key": out_key,
    })
    node.initialise()
    # ★ 节点启动时是**未激活**的（要 call `/pallet_servo/slot` 才出数）。本文件
    #   这几条钉的是"激活之后"的接缝，所以直接激活第 1 组 —— 与
    #   `test_node_pallet_servo.py` 的 `make_node` 同一处、同一个理由。
    node._activate_slot(1)
    return node


def _three(node, out_key):
    err = getattr(node.global_blackboard, out_key)
    assert err is not None, "伺服没有产出误差"
    assert not isinstance(err, str), err
    return err.e_bottom_px, err.e_right_px, err.theta_rad, err.box_source, err.warn


def _run(send_box, box_key, out_key):
    """跑一遍：托盘位姿由测试写、箱子由 `send_box` 提供、伺服算三个量。

    ⚠️ **托盘必须带一个 > 0 的 `latest_pallet_stamp`**：伺服现在按**输入图像的
    采集时刻**把两路观测配成对（两个检测器耗时差一个量级，必须同帧才能相减）。
    stamp 缺失或 ≤ 0 的那一帧**不参与配对**（`NodePalletObs` 的契约：上游没填
    `header.stamp` 时它写原始值 0.0，下游据此显式失败而不是编一个时刻）。
    这里给 1.0，与 `_box_detection` 的默认 stamp 一致 —— 两条路径**必须给同一套
    stamp**，否则逐位比对必然红。
    """
    writer = _writer(box_key, f"{box_key}_version",
                     "latest_pallet", "latest_pallet_version",
                     "latest_pallet_stamp")
    setattr(writer, "latest_pallet", _pallet_pose())
    setattr(writer, "latest_pallet_version", 1)
    setattr(writer, "latest_pallet_stamp", 1.0)
    send_box(writer)
    servo = _make_servo(box_key, out_key)
    assert servo.update() == Status.RUNNING, servo.feedback_message
    return _three(servo, out_key)


def test_the_box_node_output_equals_a_hand_built_observation():
    """★ 两条路径的三个量必须**逐位相同**。"""
    direct_key, node_key = "box_direct", "latest_box_obs"

    def send_direct(writer):
        setattr(writer, "box_direct",
                BoxObservation(quad=QUAD, label="box", confidence=1.0,
                               stamp=1.0))      # 与托盘 stamp 同一帧
        setattr(writer, "box_direct_version", 1)

    def send_via_node(writer):
        # 这条路径的箱子**不由 writer 写**，而是走 NodeBoxObs —— 所以只是占位，
        # 真正写入发生下面。
        setattr(writer, node_key, None)
        setattr(writer, f"{node_key}_version", 0)
        box_node = NodeBoxObs("box", "box", "ns", {"box_key": node_key})
        box_node._subscriber = object()      # 测试环境没有 ROS，假装订阅好了
        box_node._setup_error = ""
        box_node._on_message(_box_detection(QUAD))
        assert box_node.update() == Status.RUNNING

    a = _run(send_direct, direct_key, "err_direct")

    # 先把箱子节点跑完（它要写黑板），再让伺服读
    writer = _writer(node_key, f"{node_key}_version",
                     "latest_pallet", "latest_pallet_version",
                     "latest_pallet_stamp")
    setattr(writer, "latest_pallet", _pallet_pose())
    setattr(writer, "latest_pallet_version", 1)
    setattr(writer, "latest_pallet_stamp", 1.0)
    box_node = NodeBoxObs("box", "box", "ns", {"box_key": node_key})
    box_node._subscriber = object()
    box_node._setup_error = ""
    box_node._on_message(_box_detection(QUAD))
    assert box_node.update() == Status.RUNNING
    servo = _make_servo(node_key, "err_via_node")
    assert servo.update() == Status.RUNNING, servo.feedback_message
    b = _three(servo, "err_via_node")

    assert a[0] == pytest.approx(b[0], abs=1e-9), (a, b)
    assert a[1] == pytest.approx(b[1], abs=1e-9), (a, b)
    assert a[2] == pytest.approx(b[2], abs=1e-12), (a, b)
    assert a[3] == b[3] == "quad", (a, b)
    print(f"    两条路径一致：e_bottom={b[0]:+.1f}px e_right={b[1]:+.1f}px "
          f"theta={math.degrees(b[2]):+.2f}° box_source={b[3]}")


def test_the_pipeline_actually_produces_a_meaningful_theta():
    """三个量得是**真的算出来了**，不是恰好两边都是零。

    用一个转过角的箱子，`theta` 必须明显非零 —— 否则上面那条"两条路径一致"
    可能只是两边都退化成了 0。
    """
    err = _run(lambda w: (
        setattr(w, "box_direct", BoxObservation(quad=QUAD, stamp=1.0)),
        setattr(w, "box_direct_version", 1)), "box_direct", "err_x")
    e_bottom, e_right, theta, source, warn = err
    assert abs(theta) > math.radians(1.0), f"theta={math.degrees(theta):.3f}° 太小，链路可能退化了"
    assert abs(e_bottom) > 1.0 or abs(e_right) > 1.0, (e_bottom, e_right)
    print(f"    非退化：theta={math.degrees(theta):+.2f}°，"
          f"e_bottom={e_bottom:+.1f}px，e_right={e_right:+.1f}px")


def test_a_valid_false_frame_never_reaches_the_servo():
    """★ 上游说 `valid=false` 时，**伺服一帧都算不出来** —— 它不该拿到东西。

    走完整条链路验一遍：`yolo_fallback` 的四角就是**原始 YOLO 轴对齐框**
    （QUAD 是转过的，而 fallback 给的是它的 AABB）。要是节点把它放过去，
    伺服会照算不误 —— 用一个**轴对齐的四角**算出一个**看着合理但其实错了**的
    `theta`。这条断言的是"它压根到不了伺服那里"。
    """
    box_key = "latest_box_obs"
    writer = _writer(box_key, f"{box_key}_version",
                     "latest_pallet", "latest_pallet_version")
    setattr(writer, "latest_pallet", _pallet_pose())
    setattr(writer, "latest_pallet_version", 1)
    setattr(writer, box_key, None)
    setattr(writer, f"{box_key}_version", 0)

    box_node = NodeBoxObs("box", "box", "ns", {"box_key": box_key})
    box_node._subscriber = object()
    box_node._setup_error = ""
    # fallback 的四角 = QUAD 的 AABB（轴对齐），这正是上游会发的东西
    us = [p[0] for p in QUAD]
    vs = [p[1] for p in QUAD]
    aabb_quad = [[max(us), max(vs)], [min(us), max(vs)],
                 [min(us), min(vs)], [max(us), min(vs)]]
    box_node._on_message(_box_detection(aabb_quad, valid=False,
                                        source="yolo_fallback"))
    assert box_node.update() == Status.RUNNING

    assert getattr(box_node.global_blackboard, box_key, "缺") is None, \
        "valid=false 的四角被写进黑板了"
    assert getattr(box_node.global_blackboard, f"{box_key}_version") == 0

    servo = _make_servo(box_key, "err_fallback")
    assert servo.update() == Status.RUNNING
    assert "等输入" in servo.feedback_message, servo.feedback_message
    print("    valid=false 的帧到不了伺服：黑板空、伺服停在「等输入」")
