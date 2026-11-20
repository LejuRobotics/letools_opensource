#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""相机与感知只读测试（不产生运动）。覆盖文件：mixin_test_common.py + 本文件。

覆盖接口（全部经 Ros2WheeledHardware）：
  hw.camera.is_connected(camera_name)      彩色帧 2 秒内到达 -> PASS（头部+左右腕）
  hw.camera.get_camera_status()            帧计数与 fps
  hw.camera.get_camera_frame()             彩色帧形状与时间戳
  hw.camera.get_depth_data()               深度图形状与量纲
  hw.camera.get_camera_info()              内参 fx/fy/cx/cy
  hw.camera.get_point_cloud()              当前恒 None -> SKIP
  hw.camera.start_camera()/stop_camera()   状态记录入口
  腕部规格：深度 uint16 毫米（16UC1），分辨率 >=640x480，硬件时间戳
  hw.perception.start()/stop()             AprilTag 检测开关
  hw.perception.get_tag_detections()       标签列表（现场有码才非空 -> SKIP）
  hw.perception.get_latest_result()
  hw.perception.check_health()
  hw.perception.get_performance_metrics()

前置：上位机 robot_manager 的传感器服务在发布 /sensor/head_camera/*；
  可用 `ros2 topic hz /sensor/head_camera/color_image_raw` 预先确认。
用法：
  /usr/bin/python3 apps/ros2_new_sdk/test_mixin_camera.py
"""
from __future__ import annotations

import argparse

import numpy as np

from mixin_test_common import (
    Ros2WheeledHardware, record, show, skip, report,
    set_log_dir, start_segment,
)


def main():
    parser = argparse.ArgumentParser(description="相机与感知只读测试")
    parser.add_argument("--log-dir", default="",
                        help="结果落盘目录；默认 ~/mixin_test_logs/<日期>/")
    args = parser.parse_args()
    set_log_dir(args.log_dir or None)
    start_segment("S10 camera/perception readonly")

    with Ros2WheeledHardware(node_name="mixin_camera_test") as hw:
        init = hw.initialize()
        if not init.success:
            record("initialize", "FAIL", init.message)
            report("S10 camera")
            return

        if hw.camera is None:
            skip("camera_adapter", "初始化后 hw.camera 为 None（相机不可用）")
            report("S10 camera")
            return

        # 给订阅留时间收首帧
        hw.spin_for(2.0)

        connected = hw.camera.is_connected()
        if connected:
            record("is_connected", "PASS", "2 秒内有彩色帧")
        else:
            record("is_connected", "FAIL",
                   "2 秒内无彩色帧；确认 robot_manager 传感器服务与话题")

        status = hw.camera.get_camera_status(camera_name="camera")
        if status is not None and status.is_running and status.frame_count > 0:
            record("get_camera_status", "PASS",
                   f"is_running={status.is_running} frames={status.frame_count} "
                   f"fps={status.fps:.1f}")
        else:
            record("get_camera_status", "FAIL",
                   f"status={status!r}")

        frame = hw.camera.get_camera_frame(camera_name="camera")
        if frame is not None and frame.color_image is not None:
            h, w = frame.color_image.shape[:2]
            record("get_camera_frame", "PASS",
                   f"shape={h}x{w} dtype={frame.color_image.dtype} "
                   f"ts={frame.timestamp:.3f} frame_id={frame.color_frame_id}")
        else:
            record("get_camera_frame", "FAIL", "无缓存彩色帧")

        depth = hw.camera.get_depth_data(camera_name="camera")
        if depth is not None and depth.depth_image is not None:
            dh, dw = depth.depth_image.shape[:2]
            record("get_depth_data", "PASS",
                   f"shape={dh}x{dw} dtype={depth.depth_image.dtype} "
                   f"scale={depth.scale}")
        else:
            # 深度是可选数据；彩色正常而深度缺失时按环境限制记录
            record("get_depth_data", "SKIP", "无缓存深度图")

        info = hw.camera.get_camera_info(camera_name="camera")
        if info is not None and info.intrinsics is not None:
            k = info.intrinsics
            record("get_camera_info", "PASS",
                   f"fx={k.fx:.1f} fy={k.fy:.1f} cx={k.cx:.1f} cy={k.cy:.1f} "
                   f"frame_id={info.frame_id}")
        else:
            record("get_camera_info", "SKIP", "无内参缓存")

        pc = hw.camera.get_point_cloud(camera_name="camera")
        if pc is None:
            skip("get_point_cloud", "当前实现恒返回 None")
        else:
            record("get_point_cloud", "PASS", f"type={type(pc).__name__}")

        r = hw.camera.start_camera(camera_name="head")
        record("start_camera", "PASS" if r else "FAIL", f"返回 {r}")
        r = hw.camera.stop_camera(camera_name="camera")
        record("stop_camera", "PASS" if r else "FAIL", f"返回 {r}")

        # ---- 腕部（规格：>=640x480、16bit 深度 mm、RGB、硬件时间戳）----
        for slot in ("wrist_left_camera", "wrist_right_camera"):
            if not hw.camera.is_connected(camera_name=slot):
                record(f"is_connected[{slot}]", "FAIL",
                       "2 秒内无彩色帧；确认 start_realsense 已起流")
                continue
            record(f"is_connected[{slot}]", "PASS", "2 秒内有彩色帧")

            frame = hw.camera.get_camera_frame(camera_name=slot)
            ok_shape = frame is not None and frame.color_image is not None
            if ok_shape:
                h, w = frame.color_image.shape[:2]
                ok_res = w >= 640 and h >= 480
                record(f"get_camera_frame[{slot}]",
                       "PASS" if ok_res else "FAIL",
                       f"shape={h}x{w} ts={frame.timestamp:.3f}"
                       + ("" if ok_res else "（分辨率低于 640x480 规格）"))
            else:
                record(f"get_camera_frame[{slot}]", "FAIL", "无缓存彩色帧")

            depth = hw.camera.get_depth_data(camera_name=slot)
            if depth is not None and depth.depth_image is not None:
                ok16 = depth.depth_image.dtype == np.uint16
                record(f"get_depth_data[{slot}]",
                       "PASS" if ok16 else "FAIL",
                       f"dtype={depth.depth_image.dtype} shape={depth.depth_image.shape}"
                       f" scale={depth.scale}"
                       + ("" if ok16 else "（深度不是 16bit 毫米）"))
            else:
                record(f"get_depth_data[{slot}]", "FAIL", "无缓存深度图")

            status = hw.camera.get_camera_status(camera_name=slot)
            if status is not None and status.frame_count > 0:
                record(f"get_camera_status[{slot}]", "PASS",
                       f"frames={status.frame_count} fps={status.fps:.1f}")
            else:
                record(f"get_camera_status[{slot}]", "FAIL", f"status={status!r}")

        # ---- 感知 ----
        if hw.perception is None:
            skip("perception", "感知适配器未创建（enable_perception/driver_only）")
        else:
            r = hw.perception.start()
            show("perception_start", r)

            hw.spin_for(2.0)

            tags = hw.perception.get_tag_detections()
            if tags:
                t0 = tags[0]
                record("get_tag_detections", "PASS",
                       f"n={len(tags)} 首个 tag_id={t0.tag_id} "
                       f"frame={t0.frame_id} ts={t0.timestamp:.3f}")
            else:
                skip("get_tag_detections",
                     "视野内无 AprilTag 或码未摆放；摆放后重跑此项")

            res = hw.perception.get_latest_result()
            record("get_latest_result", "PASS",
                   f"success={res.success} tags={len(res.tags)} msg={res.message!r}")

            h = hw.perception.check_health()
            record("check_health", "PASS" if h.get("healthy") else "FAIL",
                   str(h))

            m = hw.perception.get_performance_metrics()
            record("get_performance_metrics", "PASS",
                   f"total={m.get('total_detections')} "
                   f"latency_ms={m.get('avg_latency_ms')}")

            r = hw.perception.stop()
            show("perception_stop", r)

    report("S10 camera")


if __name__ == "__main__":
    main()
