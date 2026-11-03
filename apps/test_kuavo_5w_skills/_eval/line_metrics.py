"""直线搬运验收采样与统计（apps 层，不进 CI）。

对齐 kuavo-ros-control ``test_palletizing_line_mpc_ik.py`` 的 ``Recorder``：

- **按段统计**：每段单独算实测点到「本段」直线的垂距，不拿整条矩形任意一条边
  冒充直线度；
- **姿态漂移**：以第一段第一个样本的 TCP 姿态为基准（技能在预备姿态之后冻结姿态），
  整条行程都应与它重合；
- **高度波动**：只对水平段统计，量的是实测 z 相对该段起点的最大偏移；
- **去重**：同一 TF 发布周期内重复查到的同一帧不算独立样本，否则样本数虚高。

采样在后台线程里跑：MPC 技能提交离线轨迹后会阻塞等整段走完，
主线程没有机会插采样。
"""

import math
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

from core.common.logger import get_logger
from core.domain.transform import quat_angle_deg, rotate_vector

logger = get_logger(__name__)

_AXIS = {
    'x': (1.0, 0.0, 0.0),
    'y': (0.0, 1.0, 0.0),
    'z': (0.0, 0.0, 1.0),
}


def _axis_vector(label: str) -> Tuple[float, float, float]:
    """``'z'`` / ``'-x'`` → 单位向量（与源脚本 ``axis_vector`` 同义，允许带符号）。"""
    sign = -1.0 if label.startswith('-') else 1.0
    key = label.lstrip('+-').lower()
    if key not in _AXIS:
        raise ValueError(f"未知轴名: {label}")
    return tuple(sign * value for value in _AXIS[key])  # type: ignore[return-value]


def _unit(vector: Sequence[float]) -> Tuple[float, float, float]:
    length = math.sqrt(sum(float(value) ** 2 for value in vector))
    if length < 1e-8:
        raise ValueError(f"无法归一化的向量: {list(vector)}")
    return tuple(float(value) / length for value in vector)  # type: ignore[return-value]


def segment_distance(point: Sequence[float], start: Sequence[float],
                     end: Sequence[float]) -> float:
    """点到线段的最短距离（米）；与源脚本 ``segment_distance`` 同式。"""
    delta = [float(end[index]) - float(start[index]) for index in range(3)]
    length2 = sum(value * value for value in delta)
    if length2 < 1e-12:
        return math.sqrt(sum((float(point[i]) - float(start[i])) ** 2 for i in range(3)))
    projection = sum((float(point[i]) - float(start[i])) * delta[i]
                     for i in range(3)) / length2
    projection = min(1.0, max(0.0, projection))
    return math.sqrt(sum(
        (float(point[i]) - float(start[i]) - projection * delta[i]) ** 2 for i in range(3)))


@dataclass
class StageStats:
    """一段直线、一条臂的实测统计。"""

    count: int = 0
    line_mm: float = 0.0          # 到本段直线的最大垂距
    ori_deg: float = 0.0          # 相对冻结姿态的最大漂移
    height_dev_mm: float = 0.0    # 水平段：z 相对段起点的最大偏移
    heights: List[float] = field(default_factory=list)


class LineMetricsRecorder:
    """后台按段采样双臂 TCP(TF)，产出直线度 / 姿态 / 高度 / 垂直度统计。

    用法（技能执行前后包一层）::

        with LineMetricsRecorder(hardware, skill, 'odom', ['left', 'right'],
                                 {'left': 'zarm_l7_end_effector',
                                  'right': 'zarm_r7_end_effector'}) as rec:
            ok = run_skill(skill, params)

    技能需暴露 ``current_segment`` / ``current_segment_index`` 与
    ``current_segment_bounds(arm)``（两个 palletizing_line_* 技能都提供）。
    """

    def __init__(self, hardware, skill, world_frame: str, arms: Sequence[str],
                 ee_frames: Dict[str, str], rate: float = 50.0, tf_timeout: float = 0.2):
        self.hardware = hardware
        self.skill = skill
        self.world_frame = world_frame
        self.arms = list(arms)
        self.ee_frames = dict(ee_frames)
        self.rate = float(rate)
        self.tf_timeout = float(tf_timeout)

        self.stats: Dict[Tuple[str, str], StageStats] = {}
        self.rows: List[dict] = []
        self.initial_quat: Dict[str, Tuple[float, float, float, float]] = {}
        self.missing_tf = 0

        self._last_pose: Dict[str, List[float]] = {}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    # ------------------------------------------------------------------
    # 生命周期
    # ------------------------------------------------------------------
    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._sample_loop,
                                        name='line-metrics', daemon=True)
        self._thread.start()

    def stop(self) -> None:
        if self._thread is None:
            return
        self._stop.set()
        self._thread.join(timeout=self.tf_timeout * 2.0 + 1.0)
        if self._thread.is_alive():
            logger.warning("直线验收采样线程未在超时内退出，统计可能缺少最后几帧")
        self._thread = None

    def __enter__(self) -> "LineMetricsRecorder":
        self.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.stop()

    # ------------------------------------------------------------------
    # 采样
    # ------------------------------------------------------------------
    def _sample_loop(self) -> None:
        interval = 1.0 / self.rate if self.rate > 0 else 0.02
        while not self._stop.is_set():
            try:
                self._sample_once()
            except Exception as exc:  # 采样线程不能因一次查询异常拖垮整场验收
                logger.warning("直线验收采样异常: %s", exc)
            self._stop.wait(interval)

    def _sample_once(self) -> None:
        for arm in self.arms:
            result = self.hardware.get_transform(
                self.world_frame, self.ee_frames[arm], self.tf_timeout)
            if not result.success:
                self.missing_tf += 1
                continue
            transform = result.data
            pose = transform.to_pose_list()
            if self._last_pose.get(arm) == pose:
                continue  # 同一个 TF 发布周期里的重复查询
            self._last_pose[arm] = pose

            segment = getattr(self.skill, 'current_segment', None)
            if segment is None:
                continue  # 段间空档 / 预备姿态阶段不计入任何一段
            bounds = self.skill.current_segment_bounds(arm)
            if bounds is None:
                continue
            start, end = bounds

            with self._lock:
                reference = self.initial_quat.setdefault(arm, transform.orientation)
                stats = self.stats.setdefault((segment.name, arm), StageStats())
                lateral_mm = segment_distance(transform.position, start, end) * 1000.0
                drift_deg = quat_angle_deg(reference, transform.orientation)
                height_dev_mm = abs(transform.position[2] - float(start[2])) * 1000.0
                stats.count += 1
                stats.line_mm = max(stats.line_mm, lateral_mm)
                stats.ori_deg = max(stats.ori_deg, drift_deg)
                if segment.kind == 'horizontal':
                    stats.heights.append(float(transform.position[2]))
                    stats.height_dev_mm = max(stats.height_dev_mm, height_dev_mm)
                self.rows.append({
                    'stage': segment.name, 'arm': arm,
                    'x': transform.position[0], 'y': transform.position[1],
                    'z': transform.position[2],
                    'line_dev_mm': lateral_mm, 'ori_drift_deg': drift_deg,
                    'height_dev_mm': height_dev_mm if segment.kind == 'horizontal' else None,
                })

    # ------------------------------------------------------------------
    # 统计结果
    # ------------------------------------------------------------------
    def height_range_mm(self, stats: StageStats) -> Optional[float]:
        """水平段的 z 波动范围（mm）；竖直段或样本不足返回 None。"""
        if len(stats.heights) < 2:
            return None
        return (max(stats.heights) - min(stats.heights)) * 1000.0

    def perpendicularity(self, box_frame: Optional[str], tool_normal_axis: Optional[str],
                         box_normal_axis: Optional[str]) -> Optional[Dict[str, float]]:
        """箱面垂直度（度）：末端工具轴与箱面法向的夹角，越小越正对箱面。

        三个参数任一为 None 时不测，返回 None——与源脚本默认「未开启垂直度验收」
        一致；要用就同时给出箱面坐标系名和两根轴。
        """
        if not (box_frame and tool_normal_axis and box_normal_axis):
            return None
        box = self.hardware.get_transform(self.world_frame, box_frame, self.tf_timeout)
        if not box.success:
            logger.warning("箱面垂直度无法测量: %s", box.message)
            return None
        normal = _unit(rotate_vector(box.data.orientation, _axis_vector(box_normal_axis)))
        results: Dict[str, float] = {}
        for arm in self.arms:
            tcp = self.hardware.get_transform(
                self.world_frame, self.ee_frames[arm], self.tf_timeout)
            if not tcp.success:
                logger.warning("%s 臂垂直度无法测量: %s", arm, tcp.message)
                continue
            approach = _unit(rotate_vector(tcp.data.orientation,
                                           _axis_vector(tool_normal_axis)))
            cosine = abs(sum(normal[index] * approach[index] for index in range(3)))
            results[arm] = math.degrees(math.acos(min(1.0, max(0.0, cosine))))
        return results or None

    def summary(self) -> List[str]:
        """每段每臂一行的可读统计；正负号/单位与源脚本文本一致。"""
        lines = []
        for (stage, arm), stats in self.stats.items():
            height = self.height_range_mm(stats)
            height_text = f"{height:6.2f}mm" if height is not None else "   N/A "
            lines.append(
                f"{stage:<16} {arm:<5} 样本{stats.count:4d}  直线{stats.line_mm:6.2f}mm  "
                f"姿态{stats.ori_deg:5.2f}°  高度波动{height_text}")
        lines.append(f"TF 缺失 {self.missing_tf} 次")
        return lines
