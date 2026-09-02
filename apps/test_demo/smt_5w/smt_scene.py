#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""轮臂 SMT 料盘搬运独立场景。

脚本只编排 ``robot`` 包已有的二维码、底盘、双臂 IK、关节插值和夹爪能力。
二维码消息 ``/robot_tag_info`` 中的位置单位应为米；二维码观测四元数只供底盘
接近使用，不参与夹爪末端朝向计算。

实机运行前必须先填写 ``config/smt_scene.yaml`` 和下方现场参数，并逐项验收。
"""

from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass, replace
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

import yaml

from robot import (
    ArmController,
    EndEffectorController,
    QRRecognizer,
    QRScanTimeout,
    build_motion_controller,
    build_robot_io,
)
from robot.geometry import normalize_angle


# ---------------------------------------------------------------------------
# 实机参数：启动前按现场标定值修改。
# ---------------------------------------------------------------------------

# 单臂复位姿态，单位：度。
ARM_RESET_POSE = [20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0]
# 左臂持物姿态，单位：度。
LEFT_ARM_CARRY_POSE = [-20.0, 40.0, 0.0, -100.0, -50.0, 0.0, 0.0]
# 右臂持物姿态，单位：度。
RIGHT_ARM_CARRY_POSE = [-20.0, -40.0, 0.0, -100.0, 50.0, 0.0, 0.0]
# 左臂抓取准备姿态，单位：度。
LEFT_ARM_READY_POSE = [-20.0, 0.0, 0.0, -130.0, -90.0, -50.0, 0.0]
# 右臂抓取准备姿态，单位：度。
RIGHT_ARM_READY_POSE = [-20.0, 0.0, 0.0, -130.0, 90.0, 50.0, 0.0]

# 双臂初始/复位姿态，排列：左臂 7 维 + 右臂 7 维。
INITIAL_RESET_JOINTS_DEG = ARM_RESET_POSE + ARM_RESET_POSE

# 左臂抓取时的双臂准备姿态。
LEFT_READY_JOINTS_DEG = LEFT_ARM_READY_POSE + ARM_RESET_POSE
# 右臂抓取时的双臂准备姿态。
RIGHT_READY_JOINTS_DEG = ARM_RESET_POSE + RIGHT_ARM_READY_POSE
# 扫描料盘小码时的左臂避让姿态，单位：度。
LEFT_SCAN_READY_POSE = [10.0, 50.0, 0.0, -130.0, -50.0, -10.0, 0.0]
# 扫描料盘小码时的右臂避让姿态，单位：度。
RIGHT_SCAN_READY_POSE = [10.0, -50.0, 0.0, -130.0, 50.0, 10.0, 0.0]
# 双臂扫码避让姿态，排列：左臂 7 维 + 右臂 7 维。
SCAN_READY_JOINTS_DEG = LEFT_SCAN_READY_POSE + RIGHT_SCAN_READY_POSE

# 下层两次抓取共用的左右臂准备姿态，单位：度。
LOWER_LEFT_ARM_READY_POSE = [20.0, 0.0, 0.0, -90.0, -90.0, 0.0, 0.0]
LOWER_RIGHT_ARM_READY_POSE = [20.0, 0.0, 0.0, -90.0, 90.0, 0.0, 0.0]
# 下层料盘扫码时的左右臂避让姿态，单位：度。
LOWER_LEFT_SCAN_READY_POSE = [40.0, 20.0, 0.0, -100.0, -90.0, 0.0, 0.0,]
LOWER_RIGHT_SCAN_READY_POSE = [40.0, -20.0, 0.0, -100.0, 90.0, 0.0, 0.0]
LOWER_SCAN_READY_JOINTS_DEG = (
    LOWER_LEFT_SCAN_READY_POSE + LOWER_RIGHT_SCAN_READY_POSE
)
# 左右臂持物时的双臂姿态。
LEFT_CARRY_JOINTS_DEG = LEFT_ARM_CARRY_POSE + ARM_RESET_POSE
RIGHT_CARRY_JOINTS_DEG = ARM_RESET_POSE + RIGHT_ARM_CARRY_POSE

# 抓取预备点相对抓取终点的 XYZ 偏移，单位：米。
PICK_CLEARANCE_XYZ = [-0.08, 0, 0.04]
# 抓取终点相对料盘二维码中心的 XYZ 偏移，单位：米。
PICK_OFFSET_XYZ = [0.00, 0, 0.08]
# 夹爪闭合后，料盘沿轮臂 IK 坐标系 Z 轴的上抬距离，单位：米。
PICK_LIFT_M = 0.03

# 所有下层料盘共用的抓取预备点和终点偏移，单位：米。
LOWER_PICK_CLEARANCE_XYZ = [-0.08, 0, 0.04]  
LOWER_PICK_OFFSET_XYZ = [-0.03, 0, 0.08]   

# 放置预备点相对放置终点的 XYZ 偏移，单位：米。
PLACE_CLEARANCE_XYZ = [0.0, 0, 0]
# 放置终点相对箱子二维码中心的 XYZ 偏移，单位：米。
PLACE_OFFSET_XYZ = [-0.10, 0.06, 0.25]

# 右臂抓取/放置的末端目标姿态，四元数顺序：(x, y, z, w)。
RIGHT_IK_TARGET_QUAT_XYZW = (
    -0.5303363209255061, -0.5380661871965968, 0.46334979806218324, 0.4631793708042808
)
# 左臂抓取/放置的末端目标姿态，四元数顺序：(x, y, z, w)。
LEFT_IK_TARGET_QUAT_XYZW = (
    0.5303909216058731, -0.5380774698864563, -0.4633182919047632, 0.4631352578514348
)

# 下层抓取专用的末端目标姿态；仅用于货架抓取，放置仍使用上方原有姿态。
LOWER_LEFT_IK_TARGET_QUAT_XYZW = (
    0.34424521404906167,
    -0.36798824454776136,
    -0.6070537476869988,
    0.6144636945321847,
)
LOWER_RIGHT_IK_TARGET_QUAT_XYZW = (
    -0.3437167919753731,
    -0.3661259703876549,
    0.6062660058477203,
    0.6166458228799642,
)

# 底盘转身角度，单位：度。
TURN_ANGLE_DEG = 180
# 抓取后底盘后退距离，单位：米。
PICK_BACKOFF_M = 0.40
# 放置后底盘后退距离，单位：米。
PLACE_BACKOFF_M = 0.50

# 货架大码粗接近距离，单位：米。
SHELF_COARSE_APPROACH_DISTANCE_M = 0.35
# 底盘横向对齐速度，单位：米/秒。
SHELF_Y_ALIGN_SPEED_MPS = 0.12
# 二维码扫描超时时间，单位：秒。
QR_SCAN_TIMEOUT_S = 15.0
# 扫描料盘小码时的头部俯仰角，单位：度。
NEAR_SCAN_PITCH_DEG = 20
# 扫描上层货架大码时的固定头部俯仰角，单位：度；按现场二维码高度调整。
UPPER_COARSE_SCAN_PITCH_DEG = 0
# 扫描下层货架大码时的固定头部俯仰角，单位：度；按现场二维码高度调整。
LOWER_COARSE_SCAN_PITCH_DEG = -20


# 参考抓取流程使用的被动臂安全目标，坐标系与轮臂 IK frame 一致。
INACTIVE_LEFT_XYZ = (0.11075,0.253,0.48781)
INACTIVE_RIGHT_XYZ = (0.11075,-0.253,0.48781)

# 货架小码不再对齐到底盘中心 y=0，而是对齐到选中夹爪的前向工作线。
# 现场若发现夹爪仍有侧向偏差，只需分别微调这两个 Y 值。
LEFT_PICK_ALIGNMENT_Y_M = 0.253
RIGHT_PICK_ALIGNMENT_Y_M = -0.253

# 任务开始/结束时躯干 Z 轴抬升/回落高度（单位：米）。
TORSO_LIFT_M = 0.20

# 轮臂 IK 求解参数。字段名与 kuavo_msgs/ikSolveParam 保持一致；其中
# ``oritation_constraint_tol`` 是上游 ROS 消息中的既有拼写。
WHEEL_IK_PARAMS = {
    "frame": 0,
    "major_optimality_tol": 1e-3,
    "major_feasibility_tol": 1e-3,
    "minor_feasibility_tol": 1e-3,
    "major_iterations_limit": 100,
    "oritation_constraint_tol": 1e-3,
    "pos_constraint_tol": 1e-3,
    "pos_cost_weight": 1.0,
    "constraint_mode": 6,
}

class SMTSceneError(RuntimeError):
    """场景无法安全继续时抛出的统一异常。"""


class TrayQRNotFound(SMTSceneError):
    """已到达货架分区，但目标料盘小二维码在扫描时限内未出现。"""

    def __init__(self, tray_qr_id: int, stage: str) -> None:
        self.tray_qr_id = int(tray_qr_id)
        self.stage = str(stage)
        super().__init__(
            "料盘小码 ID=%s 在%s阶段未识别" % (self.tray_qr_id, self.stage)
        )


@dataclass(frozen=True)
class TrayConfig:
    """单个料盘的小二维码与所属货架分区大二维码映射。"""

    tray_qr_id: int
    coarse_qr_id: int
    pick_strategy: str = "upper"


@dataclass(frozen=True)
class PickStrategy:
    """仅描述允许因货架层级变化的抓取参数。"""

    name: str
    lift_torso: bool
    left_ready_pose: Tuple[float, ...]
    right_ready_pose: Tuple[float, ...]
    left_scan_ready_pose: Tuple[float, ...]
    right_scan_ready_pose: Tuple[float, ...]
    clearance_xyz: Tuple[float, ...]
    offset_xyz: Tuple[float, ...]
    left_target_quat_xyzw: Tuple[float, ...] = LEFT_IK_TARGET_QUAT_XYZW
    right_target_quat_xyzw: Tuple[float, ...] = RIGHT_IK_TARGET_QUAT_XYZW


@dataclass(frozen=True)
class SceneConfig:
    """从 YAML 读取的二维码搬运配置。"""

    trays: Tuple[TrayConfig, ...]
    box_qr_id: int


@dataclass
class SceneRuntime:
    """共享同一个 RobotIO 的场景能力集合。"""

    io: Any
    motion: Any
    qr: QRRecognizer
    arm: ArmController
    end_effector: EndEffectorController


def load_scene_config(config_path: Optional[str] = None) -> SceneConfig:
    """读取并校验料盘小码、货架分区大码和公共箱子二维码 ID。"""

    path = config_path or os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "config",
        "smt_scene.yaml",
    )
    try:
        with open(path, "r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
    except (OSError, yaml.YAMLError) as exc:
        raise SMTSceneError("无法读取 SMT 场景配置 %s: %s" % (path, exc)) from exc
    if not isinstance(data, Mapping):
        raise SMTSceneError("SMT 场景配置顶层必须是字典: %s" % path)

    trays_data = data.get("trays")
    if isinstance(trays_data, (str, bytes, Mapping)):
        raise SMTSceneError("trays 必须是非空料盘配置列表")
    try:
        tray_items = list(trays_data)
    except TypeError as exc:
        raise SMTSceneError("trays 必须是非空料盘配置列表") from exc
    if not tray_items:
        raise SMTSceneError("trays 不能为空；请先填写料盘小码和货架分区大码 ID")

    trays = []
    for index, item in enumerate(tray_items):
        name = "trays[%d]" % index
        if not isinstance(item, Mapping):
            raise SMTSceneError("%s 必须是字典" % name)
        trays.append(
            TrayConfig(
                tray_qr_id=_qr_id(item.get("tray_qr_id"), "%s.tray_qr_id" % name),
                coarse_qr_id=_qr_id(
                    item.get("coarse_qr_id"), "%s.coarse_qr_id" % name
                ),
                pick_strategy=_pick_strategy_name(
                    item.get("pick_strategy", "upper"),
                    "%s.pick_strategy" % name,
                ),
            )
        )

    box_id = _qr_id(data.get("box_qr_id"), "box_qr_id")
    tray_ids = [tray.tray_qr_id for tray in trays]
    coarse_ids = [tray.coarse_qr_id for tray in trays]
    if len(set(tray_ids)) != len(tray_ids):
        raise SMTSceneError("trays 中的 tray_qr_id 不能重复")
    conflicts = set(tray_ids) & set(coarse_ids)
    if conflicts:
        raise SMTSceneError(
            "料盘小码和货架分区大码 ID 不能冲突: %s" % sorted(conflicts)
        )
    if box_id in set(tray_ids) | set(coarse_ids):
        raise SMTSceneError("box_qr_id 不能与料盘小码或货架分区大码 ID 相同")
    seen_lower = False
    for tray in trays:
        if tray.pick_strategy == "upper" and seen_lower:
            raise SMTSceneError(
                "上层料盘必须排在所有下层料盘之前；ID=%s 的策略仍为 upper"
                % tray.tray_qr_id
            )
        if tray.pick_strategy != "upper":
            seen_lower = True
    return SceneConfig(trays=tuple(trays), box_qr_id=box_id)


def validate_parameters() -> None:
    """在创建 ROS 节点和执行任何动作之前校验现场姿态与偏置参数。"""

    for name, value in (
        ("PICK_CLEARANCE_XYZ", PICK_CLEARANCE_XYZ),
        ("PICK_OFFSET_XYZ", PICK_OFFSET_XYZ),
        ("LOWER_PICK_CLEARANCE_XYZ", LOWER_PICK_CLEARANCE_XYZ),
        ("LOWER_PICK_OFFSET_XYZ", LOWER_PICK_OFFSET_XYZ),
        ("PLACE_CLEARANCE_XYZ", PLACE_CLEARANCE_XYZ),
        ("PLACE_OFFSET_XYZ", PLACE_OFFSET_XYZ),
    ):
        _vector(value, 3, name)

    for name, value in (
        ("INITIAL_RESET_JOINTS_DEG", INITIAL_RESET_JOINTS_DEG),
        ("SCAN_READY_JOINTS_DEG", SCAN_READY_JOINTS_DEG),
        ("LOWER_SCAN_READY_JOINTS_DEG", LOWER_SCAN_READY_JOINTS_DEG),
        ("LEFT_READY_JOINTS_DEG", LEFT_READY_JOINTS_DEG),
        ("RIGHT_READY_JOINTS_DEG", RIGHT_READY_JOINTS_DEG),
        ("LEFT_CARRY_JOINTS_DEG", LEFT_CARRY_JOINTS_DEG),
        ("RIGHT_CARRY_JOINTS_DEG", RIGHT_CARRY_JOINTS_DEG),
    ):
        _vector(value, 14, name)

    for name, value in (
        ("LOWER_LEFT_ARM_READY_POSE", LOWER_LEFT_ARM_READY_POSE),
        ("LOWER_RIGHT_ARM_READY_POSE", LOWER_RIGHT_ARM_READY_POSE),
    ):
        _vector(value, 7, name)

    for name, value in (
        ("LEFT_IK_TARGET_QUAT_XYZW", LEFT_IK_TARGET_QUAT_XYZW),
        ("RIGHT_IK_TARGET_QUAT_XYZW", RIGHT_IK_TARGET_QUAT_XYZW),
        ("LOWER_LEFT_IK_TARGET_QUAT_XYZW", LOWER_LEFT_IK_TARGET_QUAT_XYZW),
        ("LOWER_RIGHT_IK_TARGET_QUAT_XYZW", LOWER_RIGHT_IK_TARGET_QUAT_XYZW),
    ):
        target_quaternion = _vector(value, 4, name)
        if math.sqrt(sum(item * item for item in target_quaternion)) <= 0.0:
            raise SMTSceneError("%s 不能为零四元数" % name)

    _finite_float(TURN_ANGLE_DEG, "TURN_ANGLE_DEG")
    backoff = _finite_float(PLACE_BACKOFF_M, "PLACE_BACKOFF_M")
    if backoff <= 0.0:
        raise SMTSceneError("PLACE_BACKOFF_M 必须大于 0")
    for name, value in (
        ("SHELF_COARSE_APPROACH_DISTANCE_M", SHELF_COARSE_APPROACH_DISTANCE_M),
        ("SHELF_Y_ALIGN_SPEED_MPS", SHELF_Y_ALIGN_SPEED_MPS),
        ("QR_SCAN_TIMEOUT_S", QR_SCAN_TIMEOUT_S),
        ("PICK_BACKOFF_M", PICK_BACKOFF_M),
        ("PICK_LIFT_M", PICK_LIFT_M),
    ):
        if _finite_float(value, name) <= 0.0:
            raise SMTSceneError("%s 必须大于 0" % name)
    _finite_float(LEFT_PICK_ALIGNMENT_Y_M, "LEFT_PICK_ALIGNMENT_Y_M")
    _finite_float(RIGHT_PICK_ALIGNMENT_Y_M, "RIGHT_PICK_ALIGNMENT_Y_M")
    _finite_float(UPPER_COARSE_SCAN_PITCH_DEG, "UPPER_COARSE_SCAN_PITCH_DEG")
    _finite_float(LOWER_COARSE_SCAN_PITCH_DEG, "LOWER_COARSE_SCAN_PITCH_DEG")
def build_runtime() -> SceneRuntime:
    """固定构建轮臂 RobotIO 和 lejuclaw 场景能力。"""

    end_effector_type = "lejuclaw"
    params: Dict[str, Any] = {
    "robot_type": "wheel",
    "end_effector_type": end_effector_type,
    "end_effector": {
        "type": end_effector_type,
        "lejuclaw": {
            "close_position": [100.0, 100.0],
            "open_position": [10.0, 10.0],
        },
    },
    "wheel": {"ik": dict(WHEEL_IK_PARAMS)},
    }
    robot_io = build_robot_io(
        "wheel",
        params=params,
        node_name="smt_scene",
        init_node=True,
    )
    motion = build_motion_controller(robot_io, params=params)
    return SceneRuntime(
        io=robot_io,
        motion=motion,
        qr=QRRecognizer(robot_io, params=params),
        arm=ArmController(robot_io, params=params),
        end_effector=EndEffectorController(robot_io, config=params),
    )


def run_scene(runtime: SceneRuntime, config: SceneConfig) -> None:
    """按二维码列表顺序完成所有料盘的抓取和公共箱放置。"""

    _require_success(runtime.motion.stop(), "任务启动停车")
    _require_success(runtime.arm.enter_external_mode(), "进入手臂外部控制模式")
    torso_lifted = config.trays[0].pick_strategy == "upper"
    if torso_lifted:
        _require_success(
            runtime.motion.move_torso_relative_xyz(
                dz=float(TORSO_LIFT_M),
                duration=2.0,
                steps=30,
                wait_reach_time=True,
            ),
            "任务开始躯干 Z 轴升高 %.3fm" % float(TORSO_LIFT_M),
        )
    else:
        runtime.io.loginfo("首个料盘使用下层策略，任务开始时不上升躯干")
    current_joints = runtime.arm.require_current_arm_joints_deg()
    current_joints = _move_arms(
        runtime,
        INITIAL_RESET_JOINTS_DEG,
        current_joints,
        "任务初始姿态",
    )

    missing_tray_ids = []
    completed_count = 0
    positioned_coarse_qr_id: Optional[int] = None
    for index, tray in enumerate(config.trays, start=1):
        strategy = _pick_strategy(tray.pick_strategy)
        if torso_lifted and not strategy.lift_torso:
            _require_success(
                runtime.motion.move_torso_relative_xyz(
                    dz=-float(TORSO_LIFT_M),
                    duration=2.0,
                    steps=30,
                    wait_reach_time=True,
                ),
                "进入下层策略前躯干 Z 轴回落 %.3fm" % float(TORSO_LIFT_M),
            )
            torso_lifted = False
        runtime.io.loginfo(
            "开始搬运第 %d/%d 个料盘，小码 ID=%s，分区大码 ID=%s，抓取策略=%s",
            index,
            len(config.trays),
            tray.tray_qr_id,
            tray.coarse_qr_id,
            strategy.name,
        )
        # 扫描前双臂进入当前层级的相机避让姿态，防止遮挡料盘小二维码。
        scan_ready_joints = (
            list(strategy.left_scan_ready_pose)
            + list(strategy.right_scan_ready_pose)
        )
        current_joints = _move_arms(
            runtime,
            scan_ready_joints,
            current_joints,
            "%s货架扫描前双臂避让姿态" % strategy.name,
        )
        try:
            current_joints = move_one_tray(
                runtime,
                tray,
                config.box_qr_id,
                current_joints,
                strategy=strategy,
                skip_coarse_approach=(
                    positioned_coarse_qr_id == tray.coarse_qr_id
                ),
            )
            completed_count += 1
            # 成功搬运会离开货架前往箱子；下一项必须重新做货架粗定位。
            positioned_coarse_qr_id = None
        except TrayQRNotFound as exc:
            _require_success(
                runtime.motion.stop(),
                "跳过缺失料盘前停车 ID=%s" % tray.tray_qr_id,
            )
            missing_tray_ids.append(tray.tray_qr_id)
            # 小码超时发生在大码接近成功之后；连续处理同一分区时可直接扫小码。
            positioned_coarse_qr_id = tray.coarse_qr_id
            runtime.io.logwarn(
                "%s；跳到配置序列中的下一个料盘",
                exc,
            )

    _require_success(runtime.motion.stop(), "任务完成停车")
    if missing_tray_ids:
        raise SMTSceneError(
            "料盘序列已轮询完成，仍未识别小码 ID=%s；已完成 %d/%d 个料盘"
            % (missing_tray_ids, completed_count, len(config.trays))
        )
    if torso_lifted:
        _require_success(
            runtime.motion.move_torso_relative_xyz(
                dz=-float(TORSO_LIFT_M),
                duration=2.0,
                steps=30,
                wait_reach_time=True,
            ),
            "任务结束躯干 Z 轴回落 %.3fm" % float(TORSO_LIFT_M),
        )
    # 正常完成时关节已回到任务初始/复位姿态；不再执行额外复位轨迹。
    runtime.io.loginfo(
        "SMT 料盘搬运完成，共处理 %d 个料盘",
        len(config.trays),
    )


def move_one_tray(
    runtime: SceneRuntime,
    tray: TrayConfig,
    box_qr_id: int,
    current_joints: Sequence[float],
    strategy: PickStrategy,
    skip_coarse_approach: bool = False,
) -> list:
    """完成单个料盘的货架抓取、带料转运、箱内放置及返回。"""

    end_effector_type = "lejuclaw"
    gripper_open_settle_s = 0.5
    gripper_close_settle_s = 1.0

    tray_qr, hand = _approach_shelf_qr(
        runtime,
        tray_qr_id=tray.tray_qr_id,
        coarse_qr_id=tray.coarse_qr_id,
        skip_coarse_approach=skip_coarse_approach,
        coarse_scan_pitch_deg=(
            UPPER_COARSE_SCAN_PITCH_DEG
            if strategy.lift_torso
            else LOWER_COARSE_SCAN_PITCH_DEG
        ),
    )
    # 放置始终使用原有上层准备姿态；策略差异只作用于货架抓取。
    place_ready_joints = (
        LEFT_READY_JOINTS_DEG if hand == "left" else RIGHT_READY_JOINTS_DEG
    )
    grasp_seed_joints = (
        list(strategy.left_ready_pose) + ARM_RESET_POSE
        if hand == "left"
        else ARM_RESET_POSE + list(strategy.right_ready_pose)
    )
    inactive_scan_pose = (
        strategy.right_scan_ready_pose
        if hand == "left"
        else strategy.left_scan_ready_pose
    )
    grasp_ready_joints = (
        list(strategy.left_ready_pose) + list(strategy.right_scan_ready_pose)
        if hand == "left"
        else list(strategy.left_scan_ready_pose) + list(strategy.right_ready_pose)
    )
    runtime.io.loginfo(
        "料盘近处位姿: ID=%s xyz=(%.4f, %.4f, %.4f)，选择%s臂",
        tray.tray_qr_id,
        float(tray_qr["x"]),
        float(tray_qr["y"]),
        float(tray_qr["z"]),
        "右" if hand == "right" else "左",
    )
    current_joints = _move_arms(
        runtime,
        grasp_ready_joints,
        current_joints,
        "%s臂进入抓取准备姿态，非抓取臂保持扫码姿态" % hand,
    )

    _require_success(
        runtime.end_effector.open(hand, end_effector_type=end_effector_type),
        "%s夹爪打开" % hand,
    )
    runtime.io.sleep(gripper_open_settle_s)

    # 先算实际抓取点，再相对该点叠加预备留位偏移。
    pick_stage2 = _add_xyz(tray_qr, strategy.offset_xyz)
    pick_stage1 = _offset_xyz(pick_stage2, strategy.clearance_xyz)
    pick_first, pick_second = _solve_two_stage(
        runtime,
        hand,
        pick_stage1,
        pick_stage2,
        grasp_seed_joints,
        "抓取",
        target_quat_xyzw=(
            strategy.left_target_quat_xyzw
            if hand == "left"
            else strategy.right_target_quat_xyzw
        ),
        inactive_arm_joints_deg=inactive_scan_pose,
        inactive_pose_label="扫码姿态",
    )
    # 两段均已成功求解后，才允许机械臂开始运动。
    current_joints = _execute_two_stage(
        runtime, pick_first, pick_second, current_joints, "抓取"
    )
    _require_success(
        runtime.end_effector.close(hand, end_effector_type=end_effector_type),
        "%s夹爪闭合" % hand,
    )
    runtime.io.loginfo(
        "%s夹爪闭合指令已返回，等待 %.1fs 后上抬料盘",
        hand,
        gripper_close_settle_s,
    )
    runtime.io.sleep(gripper_close_settle_s)

    # 保持抓取末端姿态，以抓取终点 IK 解为种子沿 IK 坐标系 Z 轴抬升料盘；
    # 非活动臂继续保持扫码避让姿态，避免抬升时产生无关动作。
    pick_lift_xyz = _offset_xyz(pick_stage2, (0.0, 0.0, PICK_LIFT_M))
    pick_lift = _solve_pose_stage(
        runtime,
        hand,
        pick_lift_xyz,
        pick_second.seed_joints_rad,
        "抓取后料盘 Z 轴上抬 %.3fm" % float(PICK_LIFT_M),
        target_quat_xyzw=(
            strategy.left_target_quat_xyzw
            if hand == "left"
            else strategy.right_target_quat_xyzw
        ),
        inactive_arm_joints_deg=inactive_scan_pose,
        inactive_pose_label="扫码姿态",
    )
    current_joints = _move_arms(
        runtime,
        pick_lift.trajectory_joints_deg,
        current_joints,
        "抓取后料盘 Z 轴上抬 %.3fm" % float(PICK_LIFT_M),
    )

    # 保持抬升末态沿当前机体 X 轴退出货架；随后活动臂进入持物姿态，
    # 非活动臂从扫码姿态回到 ARM_RESET_POSE，再执行转向。
    _move_base_x_relative(runtime, -float(PICK_BACKOFF_M))
    carry_joints = (
        LEFT_CARRY_JOINTS_DEG if hand == "left" else RIGHT_CARRY_JOINTS_DEG
    )
    current_joints = _move_arms(
        runtime, carry_joints, current_joints, "%s臂持物姿态" % hand
    )
    _turn_relative(runtime, TURN_ANGLE_DEG)
    runtime.io.sleep(1.0)
    box_qr = _approach_box_qr(runtime, box_qr_id)
    # 夹爪在转身、箱子接近和手臂回准备姿态期间始终保持闭合。
    current_joints = _move_arms(
        runtime, place_ready_joints, current_joints, "箱前准备姿态"
    )

    # 先算实际放置点，再相对该点叠加预备留位偏移。
    place_stage2 = _add_xyz(box_qr, PLACE_OFFSET_XYZ)
    place_stage1 = _offset_xyz(place_stage2, PLACE_CLEARANCE_XYZ)
    place_first, place_second = _solve_two_stage(
        runtime,
        hand,
        place_stage1,
        place_stage2,
        place_ready_joints,
        "放置",
    )
    current_joints = _execute_two_stage(
        runtime, place_first, place_second, current_joints, "放置"
    )
    _require_success(
        runtime.end_effector.open(hand, end_effector_type=end_effector_type),
        "%s夹爪打开并放料" % hand,
    )
    runtime.io.sleep(gripper_open_settle_s)

    # 机械臂保持放置末态，底盘沿此刻机体 X 轴后退。
    _move_base_x_relative(runtime, -float(PLACE_BACKOFF_M))
    current_joints = _move_arms(
        runtime,
        INITIAL_RESET_JOINTS_DEG,
        current_joints,
        "放置后复位姿态",
    )
    _turn_relative(runtime, -float(TURN_ANGLE_DEG))
    return current_joints


def _approach_shelf_qr(
    runtime: SceneRuntime,
    tray_qr_id: int,
    coarse_qr_id: int,
    skip_coarse_approach: bool = False,
    coarse_scan_pitch_deg: float = UPPER_COARSE_SCAN_PITCH_DEG,
) -> Tuple[Mapping[str, Any], str]:
    """大码调整 X/Y/朝向，小码仅调整 Y，并返回最终观测和抓取手。"""

    if skip_coarse_approach:
        runtime.io.loginfo(
            "已位于货架分区大码 ID=%s 附近，跳过大码复扫，直接扫描小码 ID=%s",
            coarse_qr_id,
            tray_qr_id,
        )
    else:
        # 已知货架大码位于上方：只将头部抬到固定角度，不进行左右/上下摆头。
        coarse_qr = runtime.qr.scan(
            coarse_qr_id,
            yaw_range=0.0,
            pitch_center=coarse_scan_pitch_deg,
            pitch_range=0.0,
            initial_yaw=0.0,
            initial_pitch=coarse_scan_pitch_deg,
            timeout=QR_SCAN_TIMEOUT_S,
        )
        move_x, move_y, move_yaw = runtime.motion.approach_target_from_qr_pose(
            coarse_qr,
            SHELF_COARSE_APPROACH_DISTANCE_M,
        )
        runtime.io.loginfo(
            "按 waist_yaw_link 接近货架大码 ID=%s: qr_xyz=(%.3f, %.3f, %.3f) "
            "approach=%.3fm move_xyyaw=(%.3f, %.3f, %.3f rad)",
            coarse_qr_id,
            float(coarse_qr["x"]),
            float(coarse_qr["y"]),
            float(coarse_qr["z"]),
            float(SHELF_COARSE_APPROACH_DISTANCE_M),
            move_x,
            move_y,
            move_yaw,
        )
        target = runtime.motion.relative_target_to_odom(move_x, move_y, move_yaw)
        _require_success(
            runtime.motion.xy_velocity_walk_to_target(*target, 0.40),
            "根据货架分区大码粗接近 ID=%s" % coarse_qr_id,
        )

    # 不配置槽位横移量：粗接近后直接扫描目标小码，避免依据未知货架尺寸
    # 臆造横移。小码超时由 run_scene 记录并跳到配置序列中的下一个料盘。
    try:
        tray_qr = runtime.qr.scan(
            tray_qr_id,
            pitch_center=NEAR_SCAN_PITCH_DEG,
            timeout=QR_SCAN_TIMEOUT_S,
        )
    except QRScanTimeout as exc:
        raise TrayQRNotFound(tray_qr_id, "精接近前") from exc
    walk_qr = runtime.io.transform_qr_for_walk(tray_qr)
    observed_y = float(walk_qr["y"])
    hand = "left" if observed_y >= 0.0 else "right"
    target_qr_y = (
        LEFT_PICK_ALIGNMENT_Y_M
        if hand == "left"
        else RIGHT_PICK_ALIGNMENT_Y_M
    )
    # 二维码相对机器人横向位置约等于 observed_y - base_motion_y，
    # 因此底盘应移动 observed_y - target_qr_y，而不是把小码移到 y=0。
    base_lateral_delta = observed_y - target_qr_y
    y_timeout = 10.0
    runtime.io.loginfo(
        "按%s夹爪工作线对齐小码 ID=%s: observed_y=%.3f target_y=%.3f "
        "base_dy=%.3f",
        "左" if hand == "left" else "右",
        tray_qr_id,
        observed_y,
        target_qr_y,
        base_lateral_delta,
    )
    if abs(base_lateral_delta) > 0.02:
        _require_success(
            runtime.motion.lateral_adjust(
                base_lateral_delta,
                SHELF_Y_ALIGN_SPEED_MPS,
                y_timeout,
                pos_tolerance=0.08,
                min_lateral_speed=0.05,
                log_label="料盘小码y",
            ),
            "根据料盘小码仅调整 Y 对齐 ID=%s" % tray_qr_id,
        )
    try:
        final_qr = runtime.qr.scan_after_walk(
            tray_qr_id,
            pitch_deg=NEAR_SCAN_PITCH_DEG,
            motion=runtime.motion,
            align_y=False,
            timeout=QR_SCAN_TIMEOUT_S,
        )
    except QRScanTimeout as exc:
        raise TrayQRNotFound(tray_qr_id, "精接近后复扫") from exc
    return final_qr, hand


def _approach_box_qr(runtime: SceneRuntime, tag_id: int) -> Mapping[str, Any]:
    """箱码只扫描一次，底盘移动后直接扣除相对位移供放置 IK 使用。"""

    approach_distance_m = 0.50
    qr = runtime.qr.scan(tag_id, timeout=QR_SCAN_TIMEOUT_S)
    start_x, start_y, start_yaw = runtime.io.get_robot_pose()
    move_x, move_y, move_yaw = runtime.motion.approach_target_from_qr_pose(
        qr,
        approach_distance_m,
    )
    runtime.io.loginfo(
        "按 waist_yaw_link 接近箱码 ID=%s: qr_xyz=(%.3f, %.3f, %.3f) "
        "approach=%.3fm move_xyyaw=(%.3f, %.3f, %.3f rad)",
        tag_id,
        float(qr["x"]),
        float(qr["y"]),
        float(qr["z"]),
        approach_distance_m,
        move_x,
        move_y,
        move_yaw,
    )
    target = runtime.motion.relative_target_to_odom(move_x, move_y, move_yaw)
    _require_success(
        runtime.motion.xy_velocity_walk_to_target(*target, 0.40),
        "接近箱子二维码 ID=%s" % tag_id,
    )
    end_x, end_y, end_yaw = runtime.io.get_robot_pose()
    odom_dx, odom_dy = end_x - start_x, end_y - start_y
    moved_x = math.cos(start_yaw) * odom_dx + math.sin(start_yaw) * odom_dy
    moved_y = -math.sin(start_yaw) * odom_dx + math.cos(start_yaw) * odom_dy
    moved_yaw = normalize_angle(end_yaw - start_yaw)

    updated_qr = dict(qr)
    qr_x, qr_y = float(qr["x"]) - moved_x, float(qr["y"]) - moved_y
    updated_qr["x"] = math.cos(moved_yaw) * qr_x + math.sin(moved_yaw) * qr_y
    updated_qr["y"] = -math.sin(moved_yaw) * qr_x + math.cos(moved_yaw) * qr_y
    runtime.io.loginfo(
        "箱码不复扫坐标更新 ID=%s: base_move=(%.3f, %.3f, %.3f rad) "
        "old_ik_xyz=(%.3f, %.3f, %.3f) new_ik_xyz=(%.3f, %.3f, %.3f)",
        tag_id,
        moved_x,
        moved_y,
        moved_yaw,
        float(qr["x"]),
        float(qr["y"]),
        float(qr["z"]),
        float(updated_qr["x"]),
        float(updated_qr["y"]),
        float(updated_qr["z"]),
    )
    runtime.io.logwarn(
        "箱码近处复扫已禁用：放置 IK 使用初始坐标减去底盘相对位移 ID=%s",
        tag_id,
    )
    return updated_qr


def _solve_two_stage(
    runtime: SceneRuntime,
    hand: str,
    stage1_xyz: Sequence[float],
    stage2_xyz: Sequence[float],
    ready_joints_deg: Sequence[float],
    action: str,
    target_quat_xyzw: Optional[Sequence[float]] = None,
    inactive_arm_joints_deg: Optional[Sequence[float]] = None,
    inactive_pose_label: str = "ARM_RESET_POSE",
) -> Tuple[Any, Any]:
    """从准备姿态 seed 连续求解两段 IK，但不执行任何轨迹。"""

    if hand not in ("left", "right"):
        raise SMTSceneError("活动臂必须为 left 或 right")
    q0 = runtime.arm.convert_traj_joints_for_ik_seed(ready_joints_deg)
    first = _solve_pose_stage(
        runtime,
        hand,
        stage1_xyz,
        q0,
        "%s第一段留位" % action,
        target_quat_xyzw=target_quat_xyzw,
        inactive_arm_joints_deg=inactive_arm_joints_deg,
        inactive_pose_label=inactive_pose_label,
    )
    second = _solve_pose_stage(
        runtime,
        hand,
        stage2_xyz,
        first.seed_joints_rad,
        "%s第二段终点" % action,
        target_quat_xyzw=target_quat_xyzw,
        inactive_arm_joints_deg=inactive_arm_joints_deg,
        inactive_pose_label=inactive_pose_label,
    )
    return first, second


def _solve_pose_stage(
    runtime: SceneRuntime,
    hand: str,
    target_xyz: Sequence[float],
    q0_joints_rad: Sequence[float],
    label: str,
    target_quat_xyzw: Optional[Sequence[float]] = None,
    inactive_arm_joints_deg: Optional[Sequence[float]] = None,
    inactive_pose_label: str = "ARM_RESET_POSE",
) -> Any:
    """把单段 SMT 目标交给 ArmController 的统一完整位姿 IK 入口。"""

    target = _vector(target_xyz, 3, "IK target_xyz")
    seed = _vector(q0_joints_rad, 14, "IK q0_joints_rad")
    quaternion_name = "%s target_quat_xyzw" % label
    configured_quaternion = (
        LEFT_IK_TARGET_QUAT_XYZW if hand == "left" else RIGHT_IK_TARGET_QUAT_XYZW
    )
    target_quaternion = _vector(
        configured_quaternion if target_quat_xyzw is None else target_quat_xyzw,
        4,
        quaternion_name,
    )
    if hand == "left":
        left_xyz, left_quat = target, target_quaternion
        right_xyz, right_quat = INACTIVE_RIGHT_XYZ, (0.0, 0.0, 0.0, 1.0)
    else:
        left_xyz, left_quat = INACTIVE_LEFT_XYZ, (0.0, 0.0, 0.0, 1.0)
        right_xyz, right_quat = target, target_quaternion

    solution = runtime.arm.solve_pose(
        left_xyz=left_xyz,
        right_xyz=right_xyz,
        left_quat=left_quat,
        right_quat=right_quat,
        label=label,
        q0_joints=seed,
    )
    inactive_joints = (
        ARM_RESET_POSE
        if inactive_arm_joints_deg is None
        else inactive_arm_joints_deg
    )
    locked = _lock_inactive_arm_to_pose(solution, hand, inactive_joints)
    runtime.io.loginfo(
        "固定%s非活动臂为%s: %s",
        "右" if hand == "left" else "左",
        inactive_pose_label,
        list(inactive_joints),
    )
    return locked


def _lock_inactive_arm_to_pose(
    solution: Any,
    hand: str,
    inactive_arm_joints_deg: Sequence[float],
) -> Any:
    """固定非活动臂执行姿态；IK q0 仍与其安全笛卡尔目标保持复位一致。"""

    if hand not in ("left", "right"):
        raise SMTSceneError("活动臂必须为 left 或 right")
    trajectory = _vector(
        solution.trajectory_joints_deg,
        14,
        "IK trajectory_joints_deg",
    )
    seed = _vector(solution.seed_joints_rad, 14, "IK seed_joints_rad")
    inactive_deg = _vector(
        inactive_arm_joints_deg,
        7,
        "inactive_arm_joints_deg",
    )
    reset_seed_deg = _vector(ARM_RESET_POSE, 7, "ARM_RESET_POSE")
    reset_seed_rad = [math.radians(value) for value in reset_seed_deg]

    inactive_slice = slice(7, 14) if hand == "left" else slice(0, 7)
    trajectory[inactive_slice] = inactive_deg
    # IK 请求中的非活动臂笛卡尔目标仍是 INACTIVE_LEFT/RIGHT_XYZ，
    # 它对应复位姿态；因此链式第二段 q0 也保持复位，只把实际执行轨迹锁在
    # 调用方指定的准备/复位姿态。
    seed[inactive_slice] = reset_seed_rad
    return replace(
        solution,
        trajectory_joints_deg=tuple(trajectory),
        seed_joints_rad=tuple(seed),
    )


def _execute_two_stage(
    runtime: SceneRuntime,
    first: Any,
    second: Any,
    start_joints: Sequence[float],
    action: str,
) -> list:
    current = _move_arms(
        runtime,
        first.trajectory_joints_deg,
        start_joints,
        "%s第一段留位" % action,
    )
    return _move_arms(
        runtime,
        second.trajectory_joints_deg,
        current,
        "%s第二段终点" % action,
    )


def _move_arms(
    runtime: SceneRuntime,
    target_joints: Sequence[float],
    start_joints: Sequence[float],
    label: str,
) -> list:
    move_duration_s = 2.0
    move_steps = 20

    runtime.io.loginfo("执行手臂轨迹: %s", label)
    result = runtime.arm.move_joints_interpolated(
        target_joints,
        duration=move_duration_s,
        steps=move_steps,
        start_joints=start_joints,
    )
    if result is None or result is False:
        raise SMTSceneError("手臂轨迹失败: %s" % label)
    return list(result)


def _turn_relative(runtime: SceneRuntime, angle_deg: float) -> None:
    """相对当前 odom yaw 执行任意带符号角度的闭环转动。"""

    angular_speed_rad_s = 0.40
    yaw_tolerance_rad = math.radians(2.0)
    timeout_s = 30.0

    _, _, target_yaw = runtime.motion.relative_target_to_odom(
        0.0, 0.0, math.radians(float(angle_deg))
    )
    target_yaw = normalize_angle(target_yaw)
    ok = runtime.motion.turn_to_yaw(
        target_yaw,
        yaw_tolerance_rad,
        angular_speed_rad_s,
        timeout_s,
        0.1,
    )
    stopped = runtime.motion.stop()
    if not ok or not stopped:
        raise SMTSceneError("底盘相对转动失败: %.3f°" % float(angle_deg))


def _move_base_x_relative(runtime: SceneRuntime, distance_m: float) -> None:
    """沿动作开始时的机体 X 轴执行闭环相对位移。"""

    target_x, target_y, target_yaw = runtime.motion.relative_target_to_odom(
        float(distance_m), 0.0, 0.0
    )
    _require_success(
        runtime.motion.xy_velocity_walk_to_target(target_x, target_y, target_yaw, 0.30),
        "底盘沿机体 X 轴相对移动 %.3fm" % float(distance_m),
    )


def _add_xyz(
    qr: Mapping[str, Any], offset: Sequence[float]
) -> Tuple[float, float, float]:
    """二维码 XYZ 加上带符号偏置。"""

    base = _vector((qr["x"], qr["y"], qr["z"]), 3, "二维码 XYZ")
    return _offset_xyz(base, offset)


def _offset_xyz(
    base_xyz: Sequence[float], offset: Sequence[float]
) -> Tuple[float, float, float]:
    """对已有 XYZ 点再叠加带符号偏置。"""

    base = _vector(base_xyz, 3, "XYZ 基点")
    delta = _vector(offset, 3, "XYZ 偏置")
    return tuple(value + adjustment for value, adjustment in zip(base, delta))


def _qr_id(value: Any, name: str) -> int:
    if isinstance(value, bool):
        raise SMTSceneError("%s 必须是整数二维码 ID" % name)
    try:
        result = int(value)
    except (TypeError, ValueError) as exc:
        raise SMTSceneError("%s 必须是整数二维码 ID" % name) from exc
    if result < 0 or isinstance(value, float) and not value.is_integer():
        raise SMTSceneError("%s 必须是非负整数二维码 ID" % name)
    return result


def _pick_strategy_name(value: Any, name: str) -> str:
    """校验配置中按料盘 ID 关联的抓取策略名。"""

    result = str(value).strip().lower()
    allowed = ("upper", "lower")
    if result not in allowed:
        raise SMTSceneError(
            "%s 必须是 %s 之一，当前=%r" % (name, ", ".join(allowed), value)
        )
    return result


def _pick_strategy(name: str) -> PickStrategy:
    """构造抓取参数快照；放置参数不属于策略，始终复用原值。"""

    normalized = _pick_strategy_name(name, "pick_strategy")
    if normalized == "upper":
        left_ready = LEFT_ARM_READY_POSE
        right_ready = RIGHT_ARM_READY_POSE
        left_scan_ready = LEFT_SCAN_READY_POSE
        right_scan_ready = RIGHT_SCAN_READY_POSE
        clearance = PICK_CLEARANCE_XYZ
        offset = PICK_OFFSET_XYZ
        left_target_quat = LEFT_IK_TARGET_QUAT_XYZW
        right_target_quat = RIGHT_IK_TARGET_QUAT_XYZW
        lift_torso = True
    else:
        left_ready = LOWER_LEFT_ARM_READY_POSE
        right_ready = LOWER_RIGHT_ARM_READY_POSE
        left_scan_ready = LOWER_LEFT_SCAN_READY_POSE
        right_scan_ready = LOWER_RIGHT_SCAN_READY_POSE
        clearance = LOWER_PICK_CLEARANCE_XYZ
        offset = LOWER_PICK_OFFSET_XYZ
        left_target_quat = LOWER_LEFT_IK_TARGET_QUAT_XYZW
        right_target_quat = LOWER_RIGHT_IK_TARGET_QUAT_XYZW
        lift_torso = False
    return PickStrategy(
        name=normalized,
        lift_torso=lift_torso,
        left_ready_pose=tuple(_vector(left_ready, 7, "%s left ready" % normalized)),
        right_ready_pose=tuple(_vector(right_ready, 7, "%s right ready" % normalized)),
        left_scan_ready_pose=tuple(
            _vector(left_scan_ready, 7, "%s left scan ready" % normalized)
        ),
        right_scan_ready_pose=tuple(
            _vector(right_scan_ready, 7, "%s right scan ready" % normalized)
        ),
        clearance_xyz=tuple(_vector(clearance, 3, "%s clearance" % normalized)),
        offset_xyz=tuple(_vector(offset, 3, "%s offset" % normalized)),
        left_target_quat_xyzw=tuple(
            _vector(left_target_quat, 4, "%s left target quaternion" % normalized)
        ),
        right_target_quat_xyzw=tuple(
            _vector(right_target_quat, 4, "%s right target quaternion" % normalized)
        ),
    )


def _vector(value: Any, length: int, name: str) -> list:
    if isinstance(value, (str, bytes)):
        raise SMTSceneError("%s 必须是 %d 维数值序列" % (name, length))
    try:
        result = [float(item) for item in list(value)]
    except (TypeError, ValueError) as exc:
        raise SMTSceneError("%s 必须是 %d 维数值序列" % (name, length)) from exc
    if len(result) != length or not all(math.isfinite(item) for item in result):
        raise SMTSceneError("%s 必须是 %d 维有限数值" % (name, length))
    return result


def _finite_float(value: Any, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise SMTSceneError("%s 必须是数值" % name) from exc
    if not math.isfinite(result):
        raise SMTSceneError("%s 必须是有限数值" % name)
    return result


def _require_success(result: Any, label: str) -> Any:
    if result is False or result is None:
        raise SMTSceneError("%s 返回失败" % label)
    return result


def _best_effort_stop(runtime: Optional[SceneRuntime]) -> None:
    """失败路径只停车；不松爪，也不在未知关节位置强制复位。"""

    if runtime is None:
        return
    try:
        runtime.motion.stop()
    except Exception as exc:
        try:
            runtime.io.logerr("故障后停车仍失败: %s", exc)
        except Exception:
            pass


def _close_runtime(runtime: Optional[SceneRuntime]) -> None:
    if runtime is None:
        return
    try:
        runtime.qr.close()
    except Exception as exc:
        runtime.io.logerr("关闭二维码订阅失败: %s", exc)


def main() -> int:
    runtime: Optional[SceneRuntime] = None
    try:
        config = load_scene_config()
        validate_parameters()
        runtime = build_runtime()
        run_scene(runtime, config)
        return 0
    except KeyboardInterrupt:
        _best_effort_stop(runtime)
        if runtime is not None:
            runtime.io.logwarn("收到键盘中断，任务已停车；自动松爪或复位手臂")
        else:
            print("收到键盘中断", file=sys.stderr)
        return 130
    except Exception as exc:
        _best_effort_stop(runtime)
        if runtime is not None:
            runtime.io.logerr(
                "SMT 搬运失败，任务已终止并停车；自动松爪或复位手臂: %s",
                exc,
            )
        else:
            print("SMT 搬运启动失败: %s" % exc, file=sys.stderr)
        return 1
    finally:
        _close_runtime(runtime)


if __name__ == "__main__":
    raise SystemExit(main())
