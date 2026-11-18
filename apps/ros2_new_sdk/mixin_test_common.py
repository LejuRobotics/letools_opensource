#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Mixin 测试公共工具（不是用例文件）。

所有用例都通过 Ros2WheeledHardware（Hardware + Mixin → Client）调用接口，
不直接使用 driver/adapter。
"""
from __future__ import annotations

import math
import sys
import time
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

# ROS 2 测试入口：导入 adapters 前跳过 ROS 1 硬件链（adapters/hardware eager import rospy）
import os
os.environ.setdefault("LEJU_HW_SKIP_ROS1", "1")

try:
    from adapters.hardware_ros2 import Ros2WheeledHardware  # noqa: E402
except ModuleNotFoundError as exc:
    if exc.name in ("rclpy", "leju"):
        raise SystemExit(
            "未加载 ROS 2 环境。请先执行：\n"
            "  source /opt/ros/jazzy/setup.bash\n"
            "  export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp\n"
            "并用 /usr/bin/python3 运行（不要用 conda/venv 的 python3）。"
        ) from exc
    raise

RESULTS = []
LOG_DIR = None


def set_log_dir(path=None):
    """设置结果落盘目录；默认 ~/mixin_test_logs/<YYYYMMDD>/。"""
    global LOG_DIR
    base = (Path(path).expanduser() if path
            else Path.home() / "mixin_test_logs" / datetime.now().strftime("%Y%m%d"))
    base.mkdir(parents=True, exist_ok=True)
    LOG_DIR = base
    return base


def start_segment(title):
    """打印带时间戳的分段横幅，方便和视频对齐。"""
    now = datetime.now()
    bar = "=" * 62
    print(f"\n{bar}\n[{title}] start {now.strftime('%Y-%m-%d %H:%M:%S')}\n{bar}")
    return now


def record(name, status, detail=""):
    RESULTS.append((name, status, detail))


def show(name, result, detail=""):
    ok = bool(result.success)
    text = (result.message or "") + ((" " + detail) if detail else "")
    record(name, "PASS" if ok else "FAIL", text.strip())
    return ok


def skip(name, detail):
    record(name, "SKIP", detail)


def substitute(name, detail):
    """平替输出：用替代接口的真实值代替原接口，并显式标记。"""
    record(name, "SUB", "平替输出: " + detail)


def report(title=""):
    now = datetime.now()
    lines = []
    if title:
        lines.append(f"[{title}] report {now.strftime('%Y-%m-%d %H:%M:%S')}")
    lines.append("接口".ljust(40) + "结果")
    lines.append("-" * 78)
    failed = 0
    skipped = 0
    substituted = 0
    for name, status, detail in RESULTS:
        if status == "FAIL":
            failed += 1
        elif status == "SKIP":
            skipped += 1
        elif status == "SUB":
            substituted += 1
        lines.append(f"{name:40s} {status:5s} {detail}")
    lines.append("-" * 78)
    lines.append(f"合计: {len(RESULTS)} 项, FAIL={failed}, SUB={substituted}, SKIP={skipped}")
    lines.append(f"RESULT: {'PASS' if failed == 0 else 'FAIL'}")
    text = "\n".join(lines)
    print("\n" + text)

    if LOG_DIR is not None:
        slug = (title or "report").replace(" ", "_").replace("/", "_")
        path = LOG_DIR / f"{slug}_{now.strftime('%Y%m%d_%H%M%S')}.log"
        path.write_text(text + "\n", encoding="utf-8")
        print(f"[log] {path}")

    return 0 if failed == 0 else 1


def quat_to_zyx_deg(quat):
    x, y, z, w = [float(v) for v in quat]
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return [math.degrees(yaw), math.degrees(pitch), math.degrees(roll)]


def dist(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def read_pose(hw, side):
    fk = hw.solve_arm_fk()
    if not fk.success:
        return None, None, fk.message
    return (list(fk.data[side]["pos"]),
            quat_to_zyx_deg(fk.data[side]["quat"]),
            None)


def stream_joint(hw, q14, seconds, rate=50):
    """cmd_joints 是流式话题：一次一帧，这里按 50Hz 连续发。"""
    period = 1.0 / rate
    end = time.time() + seconds
    while time.time() < end:
        hw.send_arm_joint_frame(q14)
        hw.spin_once(0.0)
        time.sleep(period)


def stream_ee(hw, left, right, seconds, rate=30):
    """cmd_ee_poses 是流式话题：一次一帧，这里按 30Hz 连续发。"""
    period = 1.0 / rate
    end = time.time() + seconds
    while time.time() < end:
        hw.send_ee_poses(left, right, frame=1)
        hw.spin_once(0.0)
        time.sleep(period)


def prepare_modes(hw, quick=0):
    return show("prepare_arm_modes", hw.prepare_arm_modes(quick_source=quick))


def restore_joint(hw, side, base7, desire=2.0, tries=3):
    reply = None
    for _ in range(tries):
        reply = hw.send_arm_joint_timed(side, base7, desire)
        if reply.success:
            return reply
        hw.spin_for(2.0)
    return reply


def restore_ee(hw, side, pose6, desire=2.0, tries=3):
    reply = None
    for _ in range(tries):
        reply = hw.send_arm_ee_timed(side, pose6, desire, 1)
        if reply.success:
            return reply
        hw.spin_for(2.0)
    return reply
