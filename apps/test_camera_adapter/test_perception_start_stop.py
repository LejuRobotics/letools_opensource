#!/usr/bin/env python3
"""公用 AprilTag 实机启停测试（不控制机器人运动）。

运行前加载 ROS/本仓库 catkin 工作空间环境，并启动 ROS master。
必须先停掉其他启动相机或 AprilTag 的程序，本脚本会检查 ROS 节点冲突。

示例（在仓库根目录）：
    python3 apps/test_camera_adapter/test_perception_start_stop.py --head-only
    python3 apps/test_camera_adapter/test_perception_start_stop.py \
        --head-only --cycles 3 --duration 15 --require-tag --tag-id 1

按 config/camera_config.yaml 启动相机；不修改文件。driver_only 模式也支持，
通过独立 ROS 订阅检查图像是否持续更新。默认每轮允许没有 Tag；--require-tag
要求每轮看到标签，--tag-id 可重复指定并要求每轮看到全部指定 ID。
退出码：0=全部通过，1=测试/清理失败，130=用户中断。
"""

import argparse
import math
import signal
import sys
import threading
import time
from pathlib import Path

# 支持从任意工作目录直接运行本脚本。
REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

import os
import rosnode
import rospy
import yaml
from apriltag_ros.msg import AprilTagDetectionArray
from sensor_msgs.msg import Image

from adapters.hardware.leju_wheeled.camera_adapter import CameraAdapter
from adapters.hardware.leju_wheeled.perception_adapter import PerceptionAdapter


class TestFailure(RuntimeError):
    """验收条件未满足。"""


_CLK_TCK = os.sysconf("SC_CLK_TCK") if hasattr(os, "sysconf") else 100


class ProcSampler:
    """用 /proc 读取单个进程的状态、CPU 时间（不依赖 psutil，仅 Linux）。

    CPU 百分比按两次采样的 utime+stime 差值 / 墙钟间隔计算，与 psutil.cpu_percent 口径一致：
    100% 表示占满一个 CPU 核。
    """

    def __init__(self, pid):
        self.pid = pid
        self._last_ticks = None
        self._last_wall = None

    def _read(self):
        try:
            with open(f"/proc/{self.pid}/stat", "r") as stat_file:
                raw = stat_file.read()
        except OSError:
            return None
        return raw

    def state(self):
        """进程状态字符；进程不存在返回 None。"""
        raw = self._read()
        return None if raw is None else raw.rsplit(")", 1)[1].split()[0]

    def alive(self):
        return self.state() not in (None, "Z")

    def name(self):
        raw = self._read()
        if raw is None:
            return ""
        return raw[raw.index("(") + 1: raw.rindex(")")]

    def create_time(self):
        """进程启动的 jiffies（stat 第 22 个字段），用于区分 PID 复用。"""
        raw = self._read()
        if raw is None:
            return None
        return int(raw.rsplit(")", 1)[1].split()[19])

    def cpu_percent(self):
        """自上次调用以来的 CPU 百分比；首次调用只记录基线并返回 0.0。"""
        raw = self._read()
        if raw is None:
            return 0.0
        fields = raw.rsplit(")", 1)[1].split()
        ticks = int(fields[11]) + int(fields[12])  # utime + stime
        wall = time.monotonic()
        if self._last_ticks is None:
            self._last_ticks, self._last_wall = ticks, wall
            return 0.0
        elapsed = wall - self._last_wall
        delta = ticks - self._last_ticks
        self._last_ticks, self._last_wall = ticks, wall
        if elapsed <= 0:
            return 0.0
        return 100.0 * (delta / _CLK_TCK) / elapsed

    @staticmethod
    def children(pid):
        """pid 的所有后代 pid（通过 /proc 的 ppid 递归）。"""
        children_of = {}
        try:
            entries = os.listdir("/proc")
        except OSError:
            return []
        for entry in entries:
            if not entry.isdigit():
                continue
            try:
                with open(f"/proc/{entry}/stat", "r") as stat_file:
                    raw = stat_file.read()
                ppid = int(raw.rsplit(")", 1)[1].split()[1])
            except (OSError, ValueError, IndexError):
                continue
            children_of.setdefault(ppid, []).append(int(entry))
        found, stack = [], [pid]
        while stack:
            for child in children_of.get(stack.pop(), []):
                found.append(child)
                stack.append(child)
        return found


class MessageProbe:
    """分别记录最新帧 ID 和本轮曾见 ID，不转换/复制图像。"""

    def __init__(self):
        self._lock = threading.Lock()
        self._count = 0
        self._stamp = 0.0
        self._tag_ids = set()
        self._latest_tag_ids = set()
        self._received_at = None

    def callback(self, message):
        with self._lock:
            self._count += 1
            self._stamp = message.header.stamp.to_sec()
            self._received_at = time.monotonic()
            # 每条消息覆盖最新结果，包括空检测；累计集合只用于整轮验收。
            self._latest_tag_ids = {
                tag_id
                for detection in getattr(message, "detections", [])
                for tag_id in detection.id
            }
            self._tag_ids.update(self._latest_tag_ids)

    def snapshot(self):
        """消息计数、帧时间戳、本轮曾见 ID（用于验收，不表示当前可见）。"""
        with self._lock:
            return self._count, self._stamp, set(self._tag_ids)

    def latest_snapshot(self):
        """最新帧 ID 及消息接收距今秒数；未收到消息时 age 为 None。"""
        with self._lock:
            age = None if self._received_at is None else time.monotonic() - self._received_at
            return set(self._latest_tag_ids), age

    def clear_ids(self):
        with self._lock:
            self._tag_ids.clear()
            self._latest_tag_ids.clear()
            self._received_at = None


def positive_float(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("必须是有限正数")
    return number


def positive_int(value):
    number = int(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("必须是正整数")
    return number


def load_camera_config(path, head_only=False):
    with Path(path).open(encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict) or not isinstance(data.get("camera"), dict):
        raise TestFailure("配置文件必须包含 camera 映射，请对照 camera_config.yaml.template")
    config = dict(data["camera"])
    if config.get("enable_head", True) is not True:
        raise TestFailure("本测试使用 /camera/color/image_raw，需要 enable_head: true")
    if head_only:
        config["enable_wrist_camera"] = False
    elif config.get("enable_wrist_camera", False):
        for side in ("left", "right"):
            if config.get(f"has_{side}_wrist", False):
                serial = config.get(f"{side}_wrist_camera_sn")
                if not isinstance(serial, str) or not serial.strip():
                    raise TestFailure(f"{side}_wrist_camera_sn 必须填写非空字符串，或使用 --head-only")
    # 只覆盖本次感知测试的允许启动开关，不改磁盘配置。
    config["launch_apriltag"] = True
    return config


class PerceptionStartStopTest:
    DETECTOR_NODE = "/apriltag_ros_continuous_node"
    CONTROL_NODE = "/ar_control_node"
    IMAGE_TOPIC = "/camera/color/image_raw"

    def __init__(self, args):
        self.args = args
        self.camera = None
        self.perception = None
        self.image_probe = MessageProbe()
        self.tag_probe = MessageProbe()
        self.subscribers = []
        self.processes = {}  # (pid, create_time) -> ProcSampler，仅跟踪本次创建的进程。
        self.checks = []
        self._interrupted = False

    def _signal_handler(self, signum, frame):
        # 不在 ROS/信号回调中做阻塞清理，由 run() 的 finally 顺序释放资源。
        if not self._interrupted:
            self._interrupted = True
            raise KeyboardInterrupt

    def check(self, condition, label):
        self.checks.append((label, bool(condition)))
        print(f"[{'PASS' if condition else 'FAIL'}] {label}", flush=True)
        if not condition:
            raise TestFailure(label)

    def wait_until(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while not predicate():
            if rospy.is_shutdown():
                raise TestFailure("ROS 已关闭")
            if time.monotonic() >= deadline:
                return False
            threading.Event().wait(0.1)
        return True

    @staticmethod
    def process_alive(sampler):
        return sampler.alive()

    def detection_nodes(self):
        return set(rosnode.get_node_names()) & {self.DETECTOR_NODE, self.CONTROL_NODE}

    def remember_processes(self):
        """记录本次启动的父进程及其后代；键为 (pid, create_time)，避免 PID 复用误判。"""
        current = {}
        if self.perception is None:
            return current
        for attr in ("_ros_process", "_ar_control_process"):
            handle = getattr(self.perception, attr)
            if handle is None:
                continue
            pids = [handle.pid] + ProcSampler.children(handle.pid)
            for pid in pids:
                sampler = ProcSampler(pid)
                create_time = sampler.create_time()
                if create_time is None:
                    continue  # 已退出。
                key = (pid, create_time)
                current[key] = self.processes.setdefault(key, sampler)
        return current

    def assert_fresh_images(self, label):
        count, stamp, _ = self.image_probe.snapshot()
        fresh = self.wait_until(
            lambda: self.image_probe.snapshot()[0] >= count + 2
            and self.image_probe.snapshot()[1] > stamp,
            self.args.frame_timeout,
        )
        self.check(fresh, label)

    def assert_idle(self, label):
        baseline_images, _, _ = self.image_probe.snapshot()
        baseline_tags, _, _ = self.tag_probe.snapshot()
        deadline = time.monotonic() + self.args.idle_duration
        while time.monotonic() < deadline:
            self.check_idle_cache()
            if rospy.is_shutdown():
                raise TestFailure("ROS 已关闭")
            threading.Event().wait(0.1)
        images, _, _ = self.image_probe.snapshot()
        tags, _, _ = self.tag_probe.snapshot()
        self.check(tags == baseline_tags, f"{label}：无新的检测消息")
        self.check(not self.detection_nodes(), f"{label}：检测 ROS 节点不存在")
        self.check(not any(self.process_alive(p) for p in self.processes.values()),
                   f"{label}：自有检测进程及子进程全部退出")
        self.assert_fresh_images(f"{label}：相机仍持续出新帧")
        print(f"  待机 {self.args.idle_duration:.1f}s 收到图像 {images - baseline_images} 帧；"
              "检测进程 CPU: 无（相机驱动仍运行）", flush=True)

    def check_idle_cache(self):
        if self.perception.get_tag_detections():
            raise TestFailure("停止/未启动时 get_tag_detections() 非空")
        if self.perception.get_latest_result().success:
            raise TestFailure("停止/未启动时 get_latest_result() 返回成功")
        if self.perception._ros_process is not None or self.perception._ar_control_process is not None:
            raise TestFailure("读取缓存意外启动了检测进程")
        if self.perception.check_health()["apriltag_started"]:
            raise TestFailure("待机状态仍标记为 apriltag_started")

    def observe_running(self, cycle):
        self.tag_probe.clear_ids()
        initial_images, initial_stamp, _ = self.image_probe.snapshot()
        initial_tags, _, _ = self.tag_probe.snapshot()
        started = time.monotonic()
        deadline = started + self.args.duration
        next_sample = started + 1.0
        samples = []
        current = self.remember_processes()
        for sampler in current.values():
            if self.process_alive(sampler):
                sampler.cpu_percent()  # 预热采样；首次值不计入统计。
        while time.monotonic() < deadline:
            if rospy.is_shutdown():
                raise TestFailure("ROS 已关闭")
            health = self.perception.check_health()
            if not health["healthy"]:
                raise TestFailure(f"持续检测期间故障: {health['reason']}")
            self.perception.get_tag_detections()
            now = time.monotonic()
            if now >= next_sample:
                updated = self.remember_processes()
                rows = []
                for key, sampler in updated.items():
                    if key not in current:
                        continue  # 新发现的进程先预热，下一轮再计入。
                    if not self.process_alive(sampler):
                        continue  # 已退出的进程不计入。
                    rows.append((sampler.pid, sampler.cpu_percent(), sampler.name()))
                total = sum(row[1] for row in rows)
                samples.append(total)
                seen_ids = sorted(self.tag_probe.snapshot()[2])
                latest_ids, age = self.tag_probe.latest_snapshot()
                age_text = "未收到" if age is None else f"{age:.2f}s"
                # 显示真实 Adapter 缓存，避免累计验收 ID 被误认为仍可见。
                cached_ids = sorted({tag.tag_id for tag in self.perception.get_tag_detections()})
                print(f"  第 {cycle} 轮 {now - started:.1f}s | "
                      f"最新帧 Tag={sorted(latest_ids)} | Adapter 缓存={cached_ids} | "
                      f"本轮曾见={seen_ids} | 距末消息={age_text} | "
                      f"检测进程合计 CPU={total:.1f}%", flush=True)
                for pid, cpu, name in rows:
                    print(f"    PID={pid} CPU={cpu:.1f}% {name}", flush=True)
                current = updated
                next_sample = now + 1.0
            threading.Event().wait(0.1)
        images, stamp, _ = self.image_probe.snapshot()
        messages, _, ids = self.tag_probe.snapshot()
        self.check(images > initial_images and stamp > initial_stamp, f"第 {cycle} 轮：相机有新帧")
        self.check(messages >= initial_tags + 2, f"第 {cycle} 轮：持续收到检测消息（含空检测）")
        self.check(self.detection_nodes() == {self.DETECTOR_NODE, self.CONTROL_NODE},
                   f"第 {cycle} 轮：检测和坐标转换节点均在运行")
        if self.args.tag_id:
            missing = set(self.args.tag_id) - ids
            self.check(not missing, f"第 {cycle} 轮：看到指定 ID，缺失={sorted(missing)}")
        elif self.args.require_tag:
            self.check(bool(ids), f"第 {cycle} 轮：至少看到一个 Tag")
        elif not ids:
            print("[WARN] 没看到 Tag；本轮只验证持续检测消息和启停，可加 --require-tag 严格验收。")
        elapsed = time.monotonic() - started
        print(f"  图像接收约 {(images - initial_images) / elapsed:.1f}fps，"
              f"检测消息约 {(messages - initial_tags) / elapsed:.1f}Hz")
        if samples:
            print(f"  检测进程 CPU：平均 {sum(samples) / len(samples):.1f}%，峰值 {max(samples):.1f}%"
                  "（100% 为一个 CPU 核；不含相机驱动和本测试进程）")

    def run_checks(self):
        config = load_camera_config(self.args.config, self.args.head_only)
        print(f"配置: {Path(self.args.config).resolve()}")
        print(f"driver_only={config.get('driver_only', False)}，"
              f"腕部相机={config.get('enable_wrist_camera', False)}，本次允许 launch_apriltag=True")
        print("请将 config/apriltag_config.yaml 中 standalone_tags 定义的标签放入视野；脚本不执行机器人运动。")
        try:
            nodes = set(rosnode.get_node_names())
        except Exception as exc:
            raise TestFailure(f"无法连接 ROS master，请检查 ROS 环境/roscore: {exc}") from exc
        prefixes = ("/camera/",)
        if config.get("enable_wrist_camera", False):
            prefixes += ("/left_wrist_camera/", "/right_wrist_camera/")
        conflicts = sorted(n for n in nodes if n in {self.DETECTOR_NODE, self.CONTROL_NODE}
                           or any(n == prefix.rstrip("/") or n.startswith(prefix) for prefix in prefixes))
        self.check(not conflicts, f"启动前无相机/AprilTag 节点冲突: {conflicts}")
        rospy.init_node("test_perception_start_stop", anonymous=True, disable_signals=True)
        tag_topic = config.get("apriltag_topic", "/robot_tag_info")
        self.subscribers.append(rospy.Subscriber(self.IMAGE_TOPIC, Image, self.image_probe.callback,
                                                 queue_size=1, buff_size=16 * 1024 * 1024))
        self.subscribers.append(rospy.Subscriber(tag_topic, AprilTagDetectionArray, self.tag_probe.callback,
                                                 queue_size=10))
        self.camera = CameraAdapter()
        result = self.camera.initialize(config)
        self.check(result.success, f"相机初始化: {result.message}")
        self.assert_fresh_images("相机启动后收到连续新图像")
        self.perception = PerceptionAdapter()
        self.check(self.perception.initialize(self.camera, config), "感知初始化成功")
        self.assert_idle("初始化待机")
        last_pids = None
        for cycle in range(1, self.args.cycles + 1):
            print(f"\n========== 启停第 {cycle}/{self.args.cycles} 轮 ==========", flush=True)
            before = time.monotonic()
            result = self.perception.start()
            self.remember_processes()  # 失败时也尽量保留本次资源的观测记录。
            self.check(result.success, f"start(): {result.message}，耗时 {time.monotonic() - before:.2f}s")
            pids = (self.perception._ros_process.pid, self.perception._ar_control_process.pid)
            result = self.perception.start()
            self.check(result.success and pids == (self.perception._ros_process.pid,
                                                   self.perception._ar_control_process.pid),
                       "重复 start() 成功且未创建额外父进程")
            print(f"  本轮 roslaunch/ar_control PIDs={pids}，上轮={last_pids}")
            self.observe_running(cycle)
            before = time.monotonic()
            result = self.perception.stop()
            self.check(result.success, f"stop(): {result.message}，耗时 {time.monotonic() - before:.2f}s")
            self.check(self.wait_until(lambda: not self.detection_nodes()
                                      and not any(self.process_alive(p) for p in self.processes.values()),
                                      self.args.exit_timeout), "停止后检测节点和所有已记录子进程退出")
            result = self.perception.stop()
            self.check(result.success, "重复 stop() 成功")
            self.check(self.perception.check_health()["initialized"], "stop() 后感知保持初始化")
            self.assert_idle(f"第 {cycle} 轮停止待机")
            last_pids = pids
        # 从运行态直接 shutdown，覆盖不先 stop 的资源释放路径。
        result = self.perception.start()
        self.remember_processes()
        self.check(result.success, f"shutdown 测试前启动: {result.message}")
        self.perception.shutdown()
        self.check(not self.perception.check_health()["initialized"], "shutdown() 关闭感知模块")
        self.check(self.wait_until(lambda: not self.detection_nodes()
                                  and not any(self.process_alive(p) for p in self.processes.values()),
                                  self.args.exit_timeout), "shutdown() 后检测子进程退出")
        self.assert_fresh_images("感知 shutdown() 后相机仍持续出新帧")
        self.check(not self.perception.start().success, "shutdown() 后未重新初始化不能 start()")

    def cleanup(self):
        errors = []
        if self.perception is not None:
            try:
                result = self.perception.stop()
                self.perception.shutdown()
                if not result.success:
                    errors.append(result.message)
            except Exception as exc:
                errors.append(f"清理感知失败: {exc}")
        if self.camera is not None:
            try:
                self.camera.shutdown()
            except Exception as exc:
                errors.append(f"清理相机失败: {exc}")
        for subscriber in self.subscribers:
            try:
                subscriber.unregister()
            except Exception as exc:
                errors.append(f"注销测试订阅失败: {exc}")
        alive = [p.pid for p in self.processes.values() if self.process_alive(p)]
        if alive:
            errors.append(f"仍有本次检测进程存活: {alive}")
        for error in errors:
            print(f"[FAIL] {error}", flush=True)
        return not errors

    def run(self):
        old_handlers = {s: signal.getsignal(s) for s in (signal.SIGINT, signal.SIGTERM)}
        for sig in old_handlers:
            signal.signal(sig, self._signal_handler)
        code = 1
        print("========== 公用 AprilTag 完整启停实机测试 ==========", flush=True)
        try:
            self.run_checks()
            code = 0
        except KeyboardInterrupt:
            print("\n[INTERRUPTED] 正在释放本次启动的检测和相机资源…", flush=True)
            code = 130
        except Exception as exc:
            print(f"\n[FAIL] {exc}", flush=True)
        finally:
            # 清理期间忽略第二次 Ctrl+C，避免中断 stop() 留下子进程。
            for sig in old_handlers:
                signal.signal(sig, signal.SIG_IGN)
            try:
                if not self.cleanup() and code == 0:
                    code = 1
            finally:
                for sig, handler in old_handlers.items():
                    signal.signal(sig, handler)
        passed = sum(ok for _, ok in self.checks)
        print(f"\n========== {'全部通过' if code == 0 else '失败/中断'}："
              f"{passed}/{len(self.checks)} 项通过，退出码 {code} ==========", flush=True)
        return code


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", type=Path, default=REPO_ROOT / "config/camera_config.yaml",
                        help="本机相机配置路径")
    parser.add_argument("--head-only", action="store_true", help="本次仅启动头部相机，不启动腕部；不改文件")
    parser.add_argument("--cycles", type=positive_int, default=2, help="启停轮数（默认 2）")
    parser.add_argument("--duration", type=positive_float, default=10.0, help="每轮持续检测秒数（默认 10）")
    parser.add_argument("--idle-duration", type=positive_float, default=3.0, help="每次待机观察秒数（默认 3）")
    parser.add_argument("--frame-timeout", type=positive_float, default=10.0, help="等待新图像超时秒数")
    parser.add_argument("--exit-timeout", type=positive_float, default=5.0, help="停止后进程/ROS 节点退出超时秒数")
    parser.add_argument("--require-tag", action="store_true", help="每轮必须实际检测到至少一个 Tag")
    parser.add_argument("--tag-id", type=int, action="append", default=[], help="每轮必须看到的 ID，可重复指定")
    return PerceptionStartStopTest(parser.parse_args()).run()


if __name__ == "__main__":
    sys.exit(main())
