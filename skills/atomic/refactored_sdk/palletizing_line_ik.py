# -*- coding: utf-8 -*-
"""Atomic skill: palletizing_line_ik.

轮臂拆码垛直线搬运 —— IK（逐帧逆解）路径。

对齐 kuavo-ros-control 的 ``test_palletizing_line_mpc_ik.py`` / ``run_ik_stage()``：

1. 预备姿态（J4=-90° / J6=0° / J7=0°）之后读取双臂 TCP 世界位姿，作为路径起点，
   并冻结此刻的姿态——整条行程姿态不变；
2. 把 ``build_palletizing_path()`` 的 10 段直线按 ``ik_rate`` 采样成世界系位姿；
3. 每帧用 ``get_transform`` 拿到 IK 模型根坐标系（``waist_yaw_link``）当帧位姿，
   把世界目标搬进根坐标系后 ``solve_ik``，以**上一帧解**作 q0 保证分支连续；
4. 解出来的 14 关节角转成度后 ``publish_arm_joint_setpoint`` 逐帧下发。

**帧率与调度**：默认 ``ik_rate=40Hz``，与源脚本一致。逐帧下发走非阻塞的
``publish_arm_joint_setpoint``（只发设定点，不等到位），所以能按帧周期跑；
``send_arm_joint_trajectory`` 是「下发-等到位」的阻塞接口，这里只用于预备姿态。
调度用绝对 deadline（``started_at + (idx+1) * period``），不做每帧固定 sleep
累加。某帧落后超过一个完整周期时中止本段并返回失败，由调用方走既有的关节还原
路径；不带着一条追不上的时间轴继续发设定点。

职责边界：
- 预备姿态在本技能内——首帧 q0 与冻结姿态都依赖它；
- 不切 MPC 控制模式、不设 focus_ee/focus_z —— 由案例入口负责；
- 不做结束还原 —— 由测试 teardown 负责；
- 默认不调 MoveIt 碰撞校验（源脚本的 ``--ik-collision-check`` 默认关闭）。
"""

import math
import time
from dataclasses import dataclass
from typing import Dict, List, Optional

from core.common.logger import get_logger
from core.domain.ik import IkSolveRequest
from core.domain.palletizing_path import (
    PalletizingPathParams,
    PathSegment,
    build_palletizing_path,
    sample_positions,
)
from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.domain.transform import quat_angle_deg
from core.interfaces.i_hardware import IHardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.base.skill_base import SkillBase

logger = get_logger(__name__)

_WORLD_Z = (0.0, 0.0, 1.0)
_WORLD_Y = (0.0, 1.0, 0.0)
_WORLD_X = (1.0, 0.0, 0.0)


@dataclass
class PalletizingLineIkParams(SkillParams):
    """IK 直线搬运参数。

    默认行程 0.14 / 0.07 / 0.18m 是源脚本实测值，不是随手取的数：
    14 自由度手臂（躯干固定）肩→TCP 最大伸展 0.709m，锁死末端姿态时可用半径更小；
    竖直方向受 J7（腕俯仰）±40° 限制，最大抬升 0.189m。再大就必须换分支或解不出来。
    """

    skill_name: str = "palletizing_line_ik"
    arms: str = "both"                      # 'both' / 'left' / 'right'
    world_frame: str = "odom"
    ik_parent_frame: str = "waist_yaw_link"  # IK 模型根坐标系（手臂安装系），不是 base_link
    left_ee_frame: str = "zarm_l7_end_effector"
    right_ee_frame: str = "zarm_r7_end_effector"
    x_direction: str = "forward"            # 'forward' / 'backward'
    lateral_direction: str = "left"         # 'left' / 'right'
    x_distance: float = 0.14
    y_offset: float = 0.07
    z_height: float = 0.18
    approach_distance: float = 0.05
    approach_duration: float = 1.5
    edge_duration: float = 3.0
    ik_rate: float = 40.0                   # 与源脚本一致；非阻塞设定点发布，见文件头
    ik_max_step_deg: float = 8.0            # 单关节单帧最大跳变，超过视为跳到另一分支
    constraint_mode: int = 3
    grasp_dwell: float = 0.5
    place_dwell: float = 0.5
    # 每段轨迹发布结束后的实际手臂收敛等待（源脚本 segment_settle 同值），
    # 不是虚拟抓取/放置停顿——那两个由 grasp_dwell / place_dwell 负责。
    segment_settle: float = 0.25
    prepare_pose: bool = True
    prep_settle: float = 2.0
    tf_timeout: float = 5.0
    timeout: float = 900.0


@define_manifest(
    label="码垛直线搬运（IK）",
    category=["motion", "arm", "workflow"],
    tree_type="studio_smoke",
    description=(
        "沿 10 段纯笛卡尔直线逐帧求双臂 IK 并下发关节角；q0 用上一帧解保证分支连续，"
        "单关节跳变超 8° 即拒绝下发。"
    ),
    params=[
        {"name": "arms", "type": "string", "default": "both",
         "description": "手臂: 'both' / 'left' / 'right'"},
        {"name": "world_frame", "type": "string", "default": "odom",
         "description": "世界坐标系（TF 根）"},
        {"name": "ik_parent_frame", "type": "string", "default": "waist_yaw_link",
         "description": "IK 模型根坐标系（手臂安装系）"},
        {"name": "x_distance", "type": "float", "default": "0.14", "description": "前后行程（米）"},
        {"name": "y_offset", "type": "float", "default": "0.07", "description": "左右行程（米）"},
        {"name": "z_height", "type": "float", "default": "0.18", "description": "提升高度（米）"},
        {"name": "approach_distance", "type": "float", "default": "0.05",
         "description": "接近/撤离距离（米），0 表示不做接近段"},
        {"name": "ik_rate", "type": "float", "default": "40.0",
         "description": "IK 帧率（Hz），与源脚本一致"},
        {"name": "ik_max_step_deg", "type": "float", "default": "8.0",
         "description": "单关节单帧最大跳变（度），超过拒绝下发"},
        {"name": "prepare_pose", "type": "bool", "default": "true",
         "description": "执行前先下发预备姿态 J4=-90°/J6=0°/J7=0°"},
    ],
    inputs=[],
    outputs=[],
)
class PalletizingLineIkSkill(SkillBase):
    """IK 路径：10 段直线逐帧逆解 → 关节下发。"""

    def __init__(self, hardware: IHardware):
        super().__init__(name="palletizing_line_ik")
        self.hardware = hardware
        self.params: Optional[PalletizingLineIkParams] = None
        # 当前正在执行的段；供验收采样（apps 侧）判断样本归属，运行结束置 None。
        self.current_segment: Optional[PathSegment] = None
        self.current_segment_index: Optional[int] = None
        self._done = False
        self._paths: Dict[str, List[PathSegment]] = {}

    def on_initialize(self, params: PalletizingLineIkParams) -> Result:
        if not isinstance(params, PalletizingLineIkParams):
            return Result.fail("Invalid parameters for PalletizingLineIkSkill")
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
        if params.ik_rate <= 0:
            return Result.fail(f"ik_rate 必须为正，收到: {params.ik_rate}")
        if params.ik_max_step_deg <= 0:
            return Result.fail(f"ik_max_step_deg 必须为正，收到: {params.ik_max_step_deg}")

        self.params = params
        self._done = False
        self.current_segment = None
        self.current_segment_index = None
        self._paths = {}
        return Result.ok()

    def on_execute(self) -> Result:
        if self._done:
            return Result.ok("PalletizingLineIkSkill already finished")

        p = self.params
        arms = _active_arms(p.arms)

        if p.prepare_pose:
            prepared = _prepare_carry_joints(self.hardware, arms, p.prep_settle)
            if not prepared.success:
                self._done = True
                return prepared

        current = self.hardware.get_arm_joint_positions()
        if not current.success:
            self._done = True
            return Result.fail(f"读取手臂关节角失败，无法开始 IK: {current.message}")
        q_prev = [float(value) for value in current.data]
        if len(q_prev) != 14:
            self._done = True
            return Result.fail(f"需要 14 个关节角，收到 {len(q_prev)} 个")

        # 两条臂的起点位姿都要读：IK 服务一次要左右两个目标，单臂模式下
        # 另一臂用它的起点位姿占位（解出来的值随后会被 q_prev 覆盖）。
        starts = {}
        for arm in ("left", "right"):
            frame = p.right_ee_frame if arm == "right" else p.left_ee_frame
            transform = self.hardware.get_transform(p.world_frame, frame, p.tf_timeout)
            if not transform.success:
                self._done = True
                return Result.fail(f"读取 {arm} 臂末端位姿失败: {transform.message}")
            starts[arm] = list(transform.data.position) + list(transform.data.orientation)

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
                    starts[arm][:3], x_dir, y_dir, _WORLD_Z, path_params,
                    edge_duration=p.edge_duration, approach_duration=p.approach_duration)
        except ValueError as exc:
            self._done = True
            return Result.fail(f"路径参数非法: {exc}")

        segments = self._paths[arms[0]]
        logger.info("palletizing_line_ik: %d 段 × %d 臂, ik_rate=%.1f, X/Y/Z=%.2f/%.2f/%.2f",
                    len(segments), len(arms), p.ik_rate,
                    p.x_distance, p.y_offset, p.z_height)

        for index, segment in enumerate(segments):
            self.current_segment = segment
            self.current_segment_index = index
            result = self._run_segment(segment, self._paths, index, starts, q_prev)
            if not result.success:
                self.current_segment = None
                self.current_segment_index = None
                self._done = True
                return result
            q_prev = result.data

            # 该段全部帧发完、等手臂真正收敛后再进入该段对应的虚拟停顿或下一段，
            # 与源 run_ik_stage 段尾的 segment_settle 一致。段失败已在上方返回。
            if p.segment_settle > 0:
                time.sleep(float(p.segment_settle))

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
        return Result.ok(f"IK 直线搬运完成：{len(segments)} 段，{len(arms)} 臂")

    def _run_segment(self, segment: PathSegment, paths: Dict[str, List[PathSegment]],
                     index: int, starts: Dict[str, List[float]],
                     q_prev: List[float]) -> Result:
        """跑完一段：逐帧 IK → 设定点发布。data 返回该段最后一帧的 14 关节角（弧度）。"""
        p = self.params
        arms = _active_arms(p.arms)
        inactive = [arm for arm in ("left", "right") if arm not in arms]

        samples = {arm: sample_positions(paths[arm][index], p.ik_rate) for arm in arms}
        count = len(samples[arms[0]])
        period = 1.0 / float(p.ik_rate)
        started_at = time.monotonic()
        overruns = 0
        acc_pos_mm = 0.0
        acc_ori_deg = 0.0
        q_last = list(q_prev)

        logger.info("  [%s] %s %.3fs, %d 帧", segment.name, segment.kind, segment.duration, count)

        for frame_index in range(count):
            # 每帧重读根坐标系：世界目标固定，根在动（腰部/底盘）时也不能跟着偏。
            # 与源脚本 run_ik_stage 每帧取一次 root 位姿的做法一致。
            root = self.hardware.get_transform(p.world_frame, p.ik_parent_frame, p.tf_timeout)
            if not root.success:
                return Result.fail(
                    f"读取 IK 根坐标系失败 [{segment.name}] 帧 {frame_index}: {root.message}")

            targets = {}
            for arm in arms:
                position = samples[arm][frame_index][1]
                # 冻结姿态：起点读到什么姿态，整段就用什么姿态。
                targets[arm] = root.data.apply_inverse(
                    [position[0], position[1], position[2]] + starts[arm][3:7])
            # 单臂模式下另一臂保持起点位姿，服务端解出来的值随后会被 q_prev 覆盖。
            for arm in inactive:
                targets[arm] = root.data.apply_inverse(list(starts[arm]))

            request = IkSolveRequest(
                left_pose=targets["left"],
                right_pose=targets["right"],
                left_joint_angles=list(q_prev[:7]),
                right_joint_angles=list(q_prev[7:14]),
                max_joint_step_deg=float(p.ik_max_step_deg),
                constraint_mode=int(p.constraint_mode),
            )
            solved = self.hardware.solve_ik(request)
            if not solved.success:
                return Result.fail(
                    f"IK 无连续可行解 [{segment.name}] 帧 {frame_index}/{count - 1}: {solved.message}")
            solution = solved.data
            candidate = [float(value) for value in solution.q_arm]

            # 非运动臂不参与本次运动：保持上一帧解，避免服务端把它挪走。
            for arm in inactive:
                offset = 0 if arm == "left" else 7
                candidate[offset:offset + 7] = list(q_prev[offset:offset + 7])

            acc_pos_mm = max(acc_pos_mm, _pose_distance_mm(
                solution.left_hand_pose, targets["left"]) if "left" in arms else acc_pos_mm)
            acc_pos_mm = max(acc_pos_mm, _pose_distance_mm(
                solution.right_hand_pose, targets["right"]) if "right" in arms else acc_pos_mm)
            for arm in arms:
                solved_pose = (solution.left_hand_pose if arm == "left"
                               else solution.right_hand_pose)
                acc_ori_deg = max(acc_ori_deg, quat_angle_deg(solved_pose[3:7], targets[arm][3:7]))

            # 设定点接口收度（控制器不经弧度转换），IK 解是弧度。
            sent = self.hardware.publish_arm_joint_setpoint(
                [math.degrees(value) for value in candidate])
            if not sent.success:
                return Result.fail(f"关节设定点发布失败 [{segment.name}] 帧 {frame_index}: {sent.message}")
            q_prev = candidate
            q_last = candidate

            deadline = started_at + (frame_index + 1) * period
            remaining = deadline - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
            else:
                overruns += 1
                lag = -remaining
                # 落后不足一个周期还能追上下一帧的死线；落后超过一个周期说明
                # 整帧预算被挤掉，继续发就是在追一条追不上的时间轴，中止本段。
                if lag > period:
                    return Result.fail(
                        f"帧调度超时 [{segment.name}] 帧 {frame_index}/{count - 1}: "
                        f"落后 deadline {lag * 1000.0:.1f}ms 超过周期 "
                        f"{period * 1000.0:.1f}ms，已中止本段（累计超时 {overruns} 帧）")

        if overruns:
            logger.warning("[%s] %d/%d 帧超出 deadline（均未超过一个周期，已追平）",
                           segment.name, overruns, count)
        logger.info("[IK求解精度] %s 解末端位置最大误差 %.2fmm，姿态最大误差 %.2f°",
                    segment.name, acc_pos_mm, acc_ori_deg)
        return Result.ok(f"{segment.name} 完成", data=q_last)

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


def _pose_distance_mm(solved_pose: List[float], target_pose: List[float]) -> float:
    """服务端 FK 位姿与目标的线距离（毫米）。"""
    squared = sum((float(solved_pose[i]) - float(target_pose[i])) ** 2 for i in range(3))
    return math.sqrt(squared) * 1000.0


def _prepare_carry_joints(hardware: IHardware, arms: List[str], settle: float) -> Result:
    """下发预备搬运姿态：J4=-90°、J6=0°、J7=0°，其余关节保持当前角度。

    不检查是否到位（源脚本同样只靠在之后固定等待过渡）——路径起点的实测位姿
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
