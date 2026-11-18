# -*- coding: utf-8 -*-
"""托盘伺服 bag 记录：按激活阶段起停，500MB 分卷，按天清理。

**为什么放在节点进程里而不是单独一个 ROS 节点**：记录的起停时刻必须与
`(box_type, slot, stage)` 的激活/退出严格同源 —— call 初始伺服就开、切到最终
伺服就换一份、call -1 就停。把这些时刻从服务回调里再转成一个话题/服务给外部
记录器，只会多一层可能不同步的状态。

记录器本身只做三件事：

* `start()`：`rosbag record -o <日期目录>/<前缀> --split --size=<MB> <topics>`；
* `stop()`：先 `terminate()`，超时再 `kill()`；
* 启动时删掉 `keep_days` 之前的日期目录（只删 `YYYYMMDD` 形状的目录）。

**任何 bag 失败都不许影响伺服**：rosbag 不在 PATH、写盘失败、进程退出码非 0，
都只记日志。
"""
from __future__ import annotations

import atexit
import shutil
import subprocess
from datetime import date, datetime, timedelta
from pathlib import Path
from typing import Callable, Optional, Sequence

#: 默认话题。用户要求的是 v1 话题名；当前 v2 链路实际发的是 `*_v2` 名字，
#: 所以默认**两套都录** —— 只录 v1 名字时 bag 里会没有托盘/纸箱检测结果，
#: 而那正是排查最需要的两路。只要 v1 名单就在 board 的 `bag_topics` 里删掉
#: 三项 v2 的即可。
DEFAULT_TOPICS = (
    "/camera/color/camera_info",
    "/camera/color/image_raw/compressed",
    "/pallet/detection",
    "/box/detection",
    "/pallet_servo/dis",
    "/pallet_v2/detection",
    "/box_v2/detection",
    "/pallet_servo_v2/dis",
    "/tf",
    "/tf_static",
)


class BagRecorder:
    """一个 `rosbag record` 子进程的生命周期管理。纯 stdlib，可单测。"""

    def __init__(self, *, enabled: bool, root_dir, topics: Sequence[str] = DEFAULT_TOPICS,
                 split_mb: int = 500, keep_days: int = 1,
                 stop_timeout_s: float = 5.0, logger,
                 popen: Optional[Callable] = None,
                 which: Optional[Callable] = None,
                 today: Optional[Callable[[], date]] = None):
        self._enabled = bool(enabled)
        self._root = Path(root_dir).expanduser()
        self._topics = tuple(str(topic) for topic in topics)
        self._split_mb = max(1, int(split_mb))
        self._keep_days = max(1, int(keep_days))
        self._stop_timeout = max(0.1, float(stop_timeout_s))
        self._logger = logger
        self._popen = popen or subprocess.Popen
        self._which = which or shutil.which
        self._today = today or date.today
        self._proc = None
        self._prefix: Optional[Path] = None
        self._warned_missing = False
        self._closed = False
        if self._enabled:
            self._cleanup_old()
            atexit.register(self.close)

    @property
    def enabled(self) -> bool:
        return self._enabled

    @property
    def recording(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def start(self, *, box_type: int, slot: int, stage: int,
              stage_name: str) -> bool:
        """开始一份新记录；旧的先停掉。失败只告警，返回 False。"""
        if not self._enabled:
            return False
        self.stop("restart")
        exe = self._which("rosbag")
        if not exe:
            if not self._warned_missing:
                self._warned_missing = True
                self._logger.warning(
                    "bag 记录未启动：PATH 里找不到 rosbag —— 伺服照常运行，"
                    "只是没有 bag；确认机器人上装了 rosbag 并 source 了 ROS")
            return False

        day = self._today().strftime("%Y%m%d")
        day_dir = self._root / day
        try:
            day_dir.mkdir(parents=True, exist_ok=True)
        except Exception as exc:                      # noqa: BLE001
            self._logger.warning("bag 目录建不了（%s: %s）—— 本阶段不记录 bag",
                                 type(exc).__name__, exc)
            return False
        stamp = datetime.now().strftime("%H%M%S_%f")
        prefix = day_dir / (
            f"{stamp}_box{int(box_type)}_slot{int(slot)}_"
            f"stage{int(stage)}_{stage_name}")
        cmd = [str(exe), "record", "-o", str(prefix),
               "--split", f"--size={self._split_mb}", *self._topics]
        try:
            proc = self._popen(
                cmd,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
        except Exception as exc:                      # noqa: BLE001
            self._logger.warning("bag 记录进程起不来（%s: %s）—— 伺服照常运行",
                                 type(exc).__name__, exc)
            return False
        self._proc = proc
        self._prefix = prefix
        self._logger.info(
            "bag 开始记录：box_type=%s slot=%s stage=%s 目录=%s "
            "（每 %dMB 分卷，%d 个话题）",
            box_type, slot, stage_name, day_dir, self._split_mb,
            len(self._topics))
        return True

    def stop(self, reason: str = "") -> None:
        """停止当前记录；没有在录时什么都不做。"""
        proc, prefix = self._proc, self._prefix
        if proc is None:
            return
        self._proc = None
        self._prefix = None
        if proc.poll() is None:
            try:
                proc.terminate()
            except Exception:                          # noqa: BLE001
                pass
            try:
                proc.wait(timeout=self._stop_timeout)
            except Exception:                          # noqa: BLE001
                try:
                    proc.kill()
                except Exception:                      # noqa: BLE001
                    pass
        code = proc.poll()
        if code not in (0, None, -15):
            self._logger.warning(
                "bag 记录进程退出码 %s（停止原因=%s，前缀=%s）—— 文件可能不完整，"
                "用 rosbag info 核对", code, reason or "?", prefix or "?")
        else:
            self._logger.info("bag 停止（%s）：%s", reason or "?", prefix or "?")

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self.stop("close")

    def _cleanup_old(self) -> None:
        """删掉 `keep_days` 之前的日期目录；只认 `YYYYMMDD` 形状。

        **整段不许抛**：它在节点构造期执行，清理失败最多是旧 bag 占盘，
        不能因此让伺服节点起不来。
        """
        try:
            if not self._root.is_dir():
                return
            cutoff = self._today() - timedelta(days=max(self._keep_days - 1, 0))
            children = sorted(self._root.iterdir())
        except Exception as exc:                      # noqa: BLE001
            self._logger.warning("bag 清理跳过（读取 %s 失败：%s: %s）",
                                 self._root, type(exc).__name__, exc)
            return
        for child in children:
            try:
                if not child.is_dir():
                    continue
                day = datetime.strptime(child.name, "%Y%m%d").date()
                if day >= cutoff:
                    continue
                shutil.rmtree(child)
                self._logger.info("bag 清理：删除前一天的目录 %s", child)
            except ValueError:
                continue                              # 不是日期目录
            except Exception as exc:                  # noqa: BLE001
                self._logger.warning("bag 清理失败（%s: %s）：%s",
                                     type(exc).__name__, exc, child)
