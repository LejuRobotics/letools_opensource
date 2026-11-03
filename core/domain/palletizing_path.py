"""轮臂码垛直线搬运的路径几何（纯计算，零 ROS 依赖）。

对齐 kuavo-ros-control 的 ``test_palletizing_line_mpc_ik.py`` 的 ``build_stages()``
（transfer 场景）：一条完整的搬运行程由 10 段纯笛卡尔直线组成——

    接近(-Z) → 抓取停顿 → 提升(+Z) → 前后(X) → 左右(Y) → 下放(-Z)
    → 放置停顿 → 撤离(+Z) → 高处返程(左右 → 前后 → 竖直)

两个停顿由调用方处理，不在本模块；本模块只产出可执行的运动段。
双臂共用同一组世界方向向量，不做左右镜像——这是源脚本的约定。
"""

from dataclasses import dataclass
from typing import List, Sequence, Tuple

Point3 = Tuple[float, float, float]


@dataclass(frozen=True)
class PalletizingPathParams:
    """一组搬运行程的几何参数（单位：米）。"""

    x_distance: float = 0.80     # 前后行程
    y_offset: float = 0.30       # 左右行程（源脚本 y_width）
    z_height: float = 0.50       # 提升高度
    approach_distance: float = 0.05  # 接近/撤离距离；0 表示不做接近段


@dataclass(frozen=True)
class PathSegment:
    """一段直线：世界系起点 → 终点，给定执行时长。"""

    name: str
    kind: str          # 'vertical' / 'horizontal'
    start: Point3
    end: Point3
    duration: float


def quintic_progress(tau: float) -> float:
    """五次多项式缓动，tau 截断到 [0, 1]。与源脚本 ``quintic_progress`` 同式。"""
    tau = min(1.0, max(0.0, float(tau)))
    return 10 * tau ** 3 - 15 * tau ** 4 + 6 * tau ** 5


def _shift(point: Sequence[float], direction: Sequence[float], distance: float) -> Point3:
    return (
        float(point[0]) + distance * float(direction[0]),
        float(point[1]) + distance * float(direction[1]),
        float(point[2]) + distance * float(direction[2]),
    )


def build_palletizing_path(
    start: Sequence[float],
    x_axis: Sequence[float],
    y_axis: Sequence[float],
    z_axis: Sequence[float],
    params: PalletizingPathParams,
    edge_duration: float = 3.0,
    approach_duration: float = 1.5,
) -> List[PathSegment]:
    """把行程参数展开成一条臂的搬运路径。

    :param start: 该臂 TCP 的世界系起点 [x, y, z]
    :param x_axis / y_axis / z_axis: 世界系方向向量（单位向量）
    :param params: 行程几何参数
    :param edge_duration: 提升/横移/下放/返程各段的时长
    :param approach_duration: 接近与撤离段的时长
    :return: 段列表；``approach_distance > 0`` 时为 10 段，否则 7 段
    :raises ValueError: Z 行程不大于接近距离（无法先抬起再高处返程）
    """
    if params.z_height <= params.approach_distance:
        raise ValueError(
            f"Z 行程({params.z_height})须大于接近距离({params.approach_distance})，"
            "才能先向上撤离并在高处返程")

    approach = (float(start[0]), float(start[1]), float(start[2]))
    grasp = _shift(approach, z_axis, -params.approach_distance)
    raised = _shift(grasp, z_axis, params.z_height)
    shifted_x = _shift(raised, x_axis, params.x_distance)
    shifted_xy = _shift(shifted_x, y_axis, params.y_offset)
    placed = _shift(shifted_xy, z_axis, -params.z_height)

    def segment(name: str, kind: str, seg_start: Point3, seg_end: Point3,
                duration: float = None) -> PathSegment:
        return PathSegment(name=name, kind=kind, start=seg_start, end=seg_end,
                           duration=edge_duration if duration is None else duration)

    segments: List[PathSegment] = []
    if params.approach_distance > 0:
        segments.append(segment("approach", "vertical", approach, grasp, approach_duration))
    segments.append(segment("lift_z", "vertical", grasp, raised))
    segments.append(segment("translate_x", "horizontal", raised, shifted_x))
    segments.append(segment("translate_y", "horizontal", shifted_x, shifted_xy))
    segments.append(segment("lower_z", "vertical", shifted_xy, placed))
    if params.approach_distance > 0:
        retreat = _shift(placed, z_axis, params.approach_distance)
        segments.append(segment("retreat", "vertical", placed, retreat, approach_duration))
    else:
        retreat = placed
    # 返程走先前已执行的高处走廊，避免在箱体高度直接横向回移
    segments.append(segment("return_lift_z", "vertical", retreat, shifted_xy))
    segments.append(segment("return_y", "horizontal", shifted_xy, shifted_x))
    segments.append(segment("return_x", "horizontal", shifted_x, raised))
    segments.append(segment("return_home_z", "vertical", raised, approach))
    return segments


def sample_positions(segment: PathSegment, rate: float) -> List[Tuple[float, Point3]]:
    """按 quintic 缓动采样一段直线，返回 ``[(t, (x, y, z)), ...]``。

    t 从 0 递增到 ``segment.duration``，末点严格等于 ``segment.end``；
    采样点数与源脚本 ``samples_between()`` 一致：``max(2, round(duration*rate)+1)``。
    """
    if rate <= 0:
        raise ValueError(f"采样率必须为正，收到 {rate}")
    count = max(2, int(round(segment.duration * rate)) + 1)
    samples = []
    for index in range(count):
        progress = index / float(count - 1)
        blend = quintic_progress(progress)
        point = tuple(
            segment.start[axis] + blend * (segment.end[axis] - segment.start[axis])
            for axis in range(3)
        )
        samples.append((segment.duration * progress, point))
    return samples
