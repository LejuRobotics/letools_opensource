# -*- coding: utf-8 -*-
"""Atomic skill: palletizing_line_mpc.

轮臂拆码垛直线搬运 —— MPC（离线轨迹）路径。

把 ``core.domain.palletizing_path.build_palletizing_path()`` 展开出的 10 段直线逐段
下发，每段：

    enable_offline_trajectory(False) → set_offline_trajectory → enable_offline_trajectory(True)
    → 按段时长等待 → enable_offline_trajectory(False)

对齐 kuavo-ros-control 的 ``test_palletizing_line_mpc_ik.py`` / ``run_mpc_stage()``：

- 离线轨迹 ``frame=0``（世界系），``planner_index`` 0=左臂 / 1=右臂；
- ``cmd_vec`` 姿态字段是**世界系 RPY（弧度）**，取自预备姿态完成后冻结的 TCP 姿态，
  整条行程不变——所以「直线」是等姿态平移，姿态漂移就是跟踪误差；
- 提交新段前先关使能，避免上一段轨迹残留；段末无论成败都关；
- 双臂共用同一组世界方向向量，不做左右镜像（源脚本约定）。

职责边界：
- 预备姿态（J4=-90° / J6=0° / J7=0°）在本技能内——路径起点和冻结姿态都取自它之后；
- 不切 MPC 控制模式、不设 focus_ee/focus_z —— 由案例入口负责；
- 不做结束还原 —— 由测试 teardown 负责；
- 不碰夹爪/吸盘：抓取与放置只是虚拟停顿。
"""

import math
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from core.common.logger import get_logger
from core.domain.palletizing_path import (
    PalletizingPathParams,
    PathSegment,
    build_palletizing_path,
    sample_positions,
)
from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.interfaces.i_hardware import IHardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.base.skill_base import SkillBase

logger = get_logger(__name__)

# 世界系方向（源脚本：z_axis 固定 +Z，参考水平 Y 取世界 +Y，X = Y × Z）。
_WORLD_Z = (0.0, 0.0, 1.0)
_WORLD_Y = (0.0, 1.0, 0.0)
_WORLD_X = (1.0, 0.0, 0.0)

# 关掉离线轨迹使能后，等上一段真正停下再提交新段。
_DISABLE_SETTLE = 0.1


@dataclass
class PalletizingLineMpcParams(SkillParams):
    """MPC 直线搬运参数。

    行程单位米、时长单位秒；``x_direction`` / ``lateral_direction`` 决定搬运方向
    （源脚本默认向前、向左）。
    """

    skill_name: str = "palletizing_line_mpc"
    arms: str = "both"                      # 'both' / 'left' / 'right'
    world_frame: str = "odom"
    left_ee_frame: str = "zarm_l7_end_effector"
    right_ee_frame: str = "zarm_r7_end_effector"
    x_direction: str = "forward"            # 'forward' / 'backward'
    lateral_direction: str = "left"         # 'left' / 'right'
    x_distance: float = 0.80                # 前后行程
    y_offset: float = 0.30                  # 左右行程
    z_height: float = 0.50                  # 提升/下放高度
    approach_distance: float = 0.05         # 接近/撤离距离，0 表示不做接近段
    approach_duration: float = 1.5
    edge_duration: float = 3.0
    rate: float = 20.0                      # 离线轨迹采样率（点/秒）
    grasp_dwell: float = 0.5                # 接近段后的虚拟抓取停顿
    place_dwell: float = 0.5                # 下放段后的虚拟放置停顿
    segment_settle: float = 0.25
    prepare_pose: bool = True
    prep_settle: float = 2.0
    tf_timeout: float = 5.0
    timeout: float = 600.0


@define_manifest(
    label="码垛直线搬运（MPC）",
    category=["motion", "arm", "workflow"],
    tree_type="studio_smoke",
    description=(
        "10 段纯笛卡尔直线的轮臂搬运：接近→抓→提升→前后→左右→下放→放→撤离→高处返程。"
        "每段走 set_offline_trajectory + enable_offline_trajectory，姿态全程锁定。"
    ),
    params=[
        {"name": "arms", "type": "string", "default": "both",
         "description": "手臂: 'both' / 'left' / 'right'"},
        {"name": "world_frame", "type": "string", "default": "odom",
         "description": "世界坐标系（TF 根）"},
        {"name": "x_distance", "type": "float", "default": "0.8", "description": "前后行程（米）"},
        {"name": "y_offset", "type": "float", "default": "0.3", "description": "左右行程（米）"},
        {"name": "z_height", "type": "float", "default": "0.5", "description": "提升高度（米）"},
        {"name": "approach_distance", "type": "float", "default": "0.05",
         "description": "接近/撤离距离（米），0 表示不做接近段"},
        {"name": "edge_duration", "type": "float", "default": "3.0",
         "description": "提升/横移/下放/返程各段时长（秒）"},
        {"name": "rate", "type": "float", "default": "20.0",
         "description": "离线轨迹采样率（点/秒）"},
        {"name": "prepare_pose", "type": "bool", "default": "true",
         "description": "执行前先下发预备姿态 J4=-90°/J6=0°/J7=0°"},
    ],
    inputs=[],
    outputs=[],
)
class PalletizingLineMpcSkill(SkillBase):
    """MPC 路径：10 段直线逐段提交离线轨迹并等待走完。"""

    def __init__(self, hardware: IHardware):
        super().__init__(name="palletizing_line_mpc")
        self.hardware = hardware
        self.params: Optional[PalletizingLineMpcParams] = None
        # 当前正在执行的段；供验收采样（apps 侧）判断样本归属，运行结束置 None。
        self.current_segment: Optional[PathSegment] = None
        self.current_segment_index: Optional[int] = None
        self._done = False
        self._paths: Dict[str, List[PathSegment]] = {}

    def on_initialize(self, params: PalletizingLineMpcParams) -> Result:
        if not isinstance(params, PalletizingLineMpcParams):
            return Result.fail("Invalid parameters for PalletizingLineMpcSkill")
        if params.arms not in ("both", "left", "right"):
            return Result.fail(f"arms 必须是 'both'/'left'/'right'，收到: {params.arms}")
        if params.x_direction not in ("forward", "backward"):
            return Result.fail(f"x_direction 必须是 'forward'/'backward'，收到: {params.x_direction}")
        if params.lateral_direction not in ("left", "right"):
            return Result.fail(
                f"lateral_direction 必须是 'left'/'right'，收到: {params.lateral_direction}")
        if params.z_height <= params.approach_distance:
            return Result.fail(
                f"z_height({params.z_height}) 必须大于 approach_distance({params.approach_distance})")
        if params.rate <= 0:
            return Result.fail(f"rate 必须为正，收到: {params.rate}")

        self.params = params
        self._done = False
        self.current_segment = None
        self.current_segment_index = None
        self._paths = {}
        return Result.ok()

    def on_execute(self) -> Result:
        if self._done:
            return Result.ok("PalletizingLineMpcSkill already finished")

        p = self.params
        arms = _active_arms(p.arms)

        if p.prepare_pose:
            prepared = _prepare_carry_joints(self.hardware, arms, p.prep_settle)
            if not prepared.success:
                self._done = True
                return prepared

        starts: Dict[str, List[float]] = {}
        orientations: Dict[str, Tuple[float, float, float]] = {}
        for arm in arms:
            frame = p.right_ee_frame if arm == "right" else p.left_ee_frame
            transform = self.hardware.get_transform(p.world_frame, frame, p.tf_timeout)
            if not transform.success:
                self._done = True
                return Result.fail(f"读取 {arm} 臂末端位姿失败: {transform.message}")
            starts[arm] = list(transform.data.position)
            orientations[arm] = transform.data.to_euler_zyx()

        x_dir, y_dir = _world_directions(p.x_direction, p.lateral_direction)
        path_params = PalletizingPathParams(
            x_distance=p.x_distance,
            y_offset=p.y_offset,
            z_height=p.z_height,
            approach_distance=p.approach_distance,
        )
        try:
            for arm in arms:
                self._paths[arm] = build_palletizing_path(
                    starts[arm], x_dir, y_dir, _WORLD_Z, path_params,
                    edge_duration=p.edge_duration, approach_duration=p.approach_duration)
        except ValueError as exc:
            self._done = True
            return Result.fail(f"路径参数非法: {exc}")

        # 各臂段结构相同（同一组几何参数），取第一臂的段序列驱动循环。
        segments = self._paths[arms[0]]
        logger.info("palletizing_line_mpc: %d 段 × %d 臂, rate=%.1f, X/Y/Z=%.2f/%.2f/%.2f",
                    len(segments), len(arms), p.rate,
                    p.x_distance, p.y_offset, p.z_height)

        for index, segment in enumerate(segments):
            self.current_segment = segment
            self.current_segment_index = index
            trajectories = []
            for arm in arms:
                yaw, pitch, roll = orientations[arm]
                points = [
                    {"desire_time": elapsed,
                     "cmd_vec": [position[0], position[1], position[2], yaw, pitch, roll]}
                    for elapsed, position in sample_positions(self._paths[arm][index], p.rate)
                ]
                trajectories.append({
                    "planner_index": 0 if arm == "left" else 1,
                    "frame": 0,  # 世界系
                    "timed_traj": points,
                })

            result = self._run_segment(segment, trajectories)
            if not result.success:
                self.current_segment = None
                self.current_segment_index = None
                self._done = True
                return result

            dwell = 0.0
            if segment.name == "approach":
                dwell = p.grasp_dwell
            elif segment.name == "lower_z":
                dwell = p.place_dwell
            if dwell > 0:
                logger.info("  [%s] 虚拟停顿 %.2fs；不执行吸盘/夹爪动作", segment.name, dwell)
                time.sleep(dwell)

        self.current_segment = None
        self.current_segment_index = None
        self._done = True
        return Result.ok(f"MPC 直线搬运完成：{len(segments)} 段，{len(arms)} 臂")

    def _run_segment(self, segment: PathSegment, trajectories: List[dict]) -> Result:
        """提交并跑完一段离线轨迹；成功与否都确保关闭使能。"""
        p = self.params
        logger.info("  [%s] %s %.3fs, %d 条轨迹", segment.name, segment.kind,
                    segment.duration, len(trajectories))

        disable = self.hardware.enable_offline_trajectory(False)
        if not disable.success:
            logger.warning("离线轨迹关闭服务返回失败: %s", disable.message)
        # 关使能后留一点时间让上一段真正停下，再提交新段（源脚本同样 sleep 0.1）。
        time.sleep(_DISABLE_SETTLE)

        submitted = self.hardware.set_offline_trajectory(trajectories)
        if not submitted.success:
            return Result.fail(f"离线轨迹提交失败 [{segment.name}]: {submitted.message}")

        enabled = self.hardware.enable_offline_trajectory(True)
        if not enabled.success:
            return Result.fail(f"离线轨迹未启用 [{segment.name}]: {enabled.message}")

        try:
            time.sleep(float(segment.duration))
            if p.segment_settle > 0:
                time.sleep(float(p.segment_settle))
        finally:
            closed = self.hardware.enable_offline_trajectory(False)
        if not closed.success:
            return Result.fail(f"离线轨迹关闭失败 [{segment.name}]: {closed.message}")
        return Result.ok()

    def on_is_finished(self) -> bool:
        return self._done

    def current_segment_bounds(self, arm: str):
        """当前段的（起点, 终点）世界系坐标；无当前段或该臂无路径时返回 None。

        验收采样用它算实测点到「本段」直线的垂距——每段单独算，
        不能拿整条矩形任意一条边冒充直线度。
        """
        if self.current_segment is None or self.current_segment_index is None:
            return None
        arm_segments = self._paths.get(arm) or []
        if self.current_segment_index >= len(arm_segments):
            return None
        segment = arm_segments[self.current_segment_index]
        return segment.start, segment.end


def _active_arms(selection: str) -> List[str]:
    """'both'/'left'/'right' → 参与运动的手臂列表（顺序固定为左、右）。"""
    if selection == "both":
        return ["left", "right"]
    return [selection]


def _world_directions(x_direction: str, lateral_direction: str):
    """搬运方向向量：X=Y×Z 为「前」，Y 为「左」。"""
    x_dir = _WORLD_X if x_direction == "forward" else tuple(-v for v in _WORLD_X)
    y_dir = _WORLD_Y if lateral_direction == "left" else tuple(-v for v in _WORLD_Y)
    return x_dir, y_dir


def _prepare_carry_joints(hardware: IHardware, arms: List[str], settle: float) -> Result:
    """下发预备搬运姿态：J4=-90°、J6=0°、J7=0°，其余关节保持当前角度。

    不检查是否到位（源脚本同样只在之后靠固定等待过渡）——路径起点的实测位姿
    由随后的 TF 读取决定，读到的才是真实起点。
    """
    current = hardware.get_arm_joint_positions()
    if not current.success:
        return Result.fail(f"读取手臂关节角失败，无法下发预备姿态: {current.message}")
    goal = [float(value) for value in current.data]
    if len(goal) != 14:
        return Result.fail(f"预备姿态需要 14 个关节角，收到 {len(goal)} 个")

    for arm in arms:
        offset = 0 if arm == "left" else 7
        goal[offset + 3] = -math.pi / 2.0
        goal[offset + 5] = 0.0
        goal[offset + 6] = 0.0

    # send_arm_joint_trajectory 收度（控制器不经弧度转换），关节状态读出来是弧度。
    degrees = [math.degrees(value) for value in goal]
    logger.info("预备搬运姿态: %s J4=-90°, J6=0°, J7=0°（不检查到位）", ", ".join(arms))
    result = hardware.send_arm_joint_trajectory(degrees, time_sec=float(settle))
    if not result.success:
        return Result.fail(f"预备姿态下发失败: {result.message}")
    if settle > 0:
        time.sleep(float(settle))
    return Result.ok("预备姿态已下发")
