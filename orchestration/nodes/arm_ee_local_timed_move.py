# -*- coding: utf-8 -*-
"""ArmEeLocalTimedMove：双臂末端 local 坐标系 TimedCmd 薄节点。

对齐 apps/test_kuavo_5w_sdk_adapter/timed/04_arm/test_arm_ee_local.py:
- 调用 hardware.send_arm_ee_local_timed()
- 位姿格式 [x, y, z, yaw, pitch, roll]（位置：米，角度：度）
- 默认提供 default / forward 两组测试位姿，执行前请先把折叠臂apps/test_kuavo_5w_sdk_adapter/timed/03_leg/test_leg_joint.py运行到[42, -25, 0, 0]这个高度。

"""

import json
import os
import sys
import time
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from py_trees.common import Status

from core.domain.enums import MPCControlMode
from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import (
    get_shared_hardware,
    reset_shared_hardware,
    set_hardware_config,
)
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.arm_ee_dual_timed import (
    ArmEEDualTimedParams,
    ArmEEDualTimedSkill,
)

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")

DEFAULT_LEFT = [1.1, 0.1, 0.7, 0.0, -110.0, 0.0]
DEFAULT_RIGHT = [1.1, -0.1, 0.7, 0.0, -110.0, 0.0]

# 双臂前伸
FORWARD_LEFT = [1.18, 0.1, 0.8, 0.0, -125.0, 0.0]
FORWARD_RIGHT = [1.18, -0.1, 0.8, 0.0, -125.0, 0.0]


def _to_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "y", "on")
    return False


@define_manifest(
    label="双臂末端局部位姿（TimedCmd）",
    category=["motion", "arm"],
    tree_type="studio_smoke",
    description=(
        "双臂末端 local 坐标系 6D 位姿控制，调用 send_arm_ee_local_timed。"
        "内置 test_arm_ee_local.py 中的 default / forward 两组位姿。"
    ),
    params=[
        {"name": "pose_name", "type": "string", "default": "sequence",
         "description": "目标位姿: sequence(default→forward) / default / forward / custom"},
        {"name": "left_pose", "type": "json", "default": "[1.1,0.1,0.7,0,-110,0]",
         "description": "custom 模式左臂位姿 [x,y,z,yaw,pitch,roll]（米, 度）"},
        {"name": "right_pose", "type": "json", "default": "[1.1,-0.1,0.7,0,-110,0]",
         "description": "custom 模式右臂位姿 [x,y,z,yaw,pitch,roll]（米, 度）"},
        {"name": "desire_time", "type": "float", "default": "3.0",
         "description": "期望执行时间（秒）"},
        {"name": "focus_ee", "type": "bool", "default": "False",
         "description": "False=躯干优先/躯干不被末端带动；默认 False"},
        {"name": "focus_z", "type": "bool", "default": "False",
         "description": "False=禁用 Z 轴焦点跟随；默认 False"},
        {"name": "prepare_mpc_control", "type": "bool", "default": "True",
         "description": "执行前切换 MPC 到 ARM_EE_ONLY，行为树运行时应保持 True"},
        {"name": "prepare_arm_control", "type": "bool", "default": "True",
         "description": "执行前切换到手臂外部控制模式 set_arm_control_mode(2)"},
        {"name": "release_arm_control", "type": "bool", "default": "True",
         "description": "执行完成后释放外部末端保持 set_arm_control_mode(0)"},
    ],
    inputs=[],
    outputs=[],
)
class ArmEeLocalTimedMove(BaseAction):
    """双臂末端 local 坐标系 TimedCmd 节点。

    pose_name=sequence 时依次执行 default -> forward，并在最后统一释放手臂控制。
    单独使用 default / forward / custom 时仍只执行对应的一组位姿。
    """

    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._skill = None
        self._dry_done = False
        self._hardware = None
        self._pose_queue = []
        self._pose_index = 0
        self._desire_time = 3.0
        self._focus_ee = False
        self._focus_z = False
        self._prepare_arm_control = True
        self._release_arm_control = True

    def initialise(self):
        self._dry_done = False
        self._skill = None
        self._hardware = None
        self._pose_queue = []
        self._pose_index = 0

        pose_name = str(self.params.get("pose_name", "sequence")).strip().lower()
        self._pose_queue = self._resolve_pose_sequence(pose_name)
        if not self._pose_queue:
            self.feedback_message = (
                "arm_ee_local_timed: pose_name must be sequence/default/forward/custom "
                "and custom mode requires valid left_pose/right_pose"
            )
            return

        self._desire_time = float(self.params.get("desire_time", 3.0))
        self._focus_ee = _to_bool(self.params.get("focus_ee", False))
        self._focus_z = _to_bool(self.params.get("focus_z", False))
        self._prepare_arm_control = _to_bool(
            self.params.get("prepare_arm_control", True)
        )
        self._release_arm_control = _to_bool(
            self.params.get("release_arm_control", True)
        )

        if _DRY_RUN:
            self.feedback_message = (
                f"dry-run arm_ee_local_timed pose={pose_name} "
                f"steps={len(self._pose_queue)} desire_time={self._desire_time:.2f}s"
            )
            self._dry_done = True
            return

        self._hardware = get_shared_hardware()
        if _to_bool(self.params.get("prepare_mpc_control", True)):
            result = self._hardware.set_mpc_mode(MPCControlMode.ARM_EE_ONLY)
            if not result.success:
                self.feedback_message = (
                    result.message or "set_mpc_mode(ARM_EE_ONLY) failed"
                )
                return

        self._start_skill()

    def _start_skill(self):
        left_pose, right_pose = self._pose_queue[self._pose_index]
        is_first = self._pose_index == 0
        is_last = self._pose_index == len(self._pose_queue) - 1
        skill_params = ArmEEDualTimedParams(
            frame="local",
            left_pose=left_pose,
            right_pose=right_pose,
            desire_time=self._desire_time,
            focus_ee=self._focus_ee,
            focus_z=self._focus_z,
            prepare_arm_control=self._prepare_arm_control if is_first else False,
            release_arm_control=self._release_arm_control if is_last else False,
        )
        self._skill = ArmEEDualTimedSkill(hardware=self._hardware)
        result = self._skill.initialize(skill_params)
        if not result.success:
            self.feedback_message = result.message or "arm_ee_local_timed init failed"
            self._skill = None

    def update(self):
        if _DRY_RUN:
            return Status.SUCCESS if self._dry_done else Status.FAILURE
        if self._skill is None:
            return Status.FAILURE

        if not self._skill.is_finished():
            result = self._skill.execute()
            if not result.success:
                self.feedback_message = result.message or "arm_ee_local_timed failed"
                return Status.FAILURE

        if not self._skill.is_finished():
            return Status.RUNNING
        if self._pose_index == len(self._pose_queue) - 1:
            return Status.SUCCESS

        self._pose_index += 1
        self._start_skill()
        if self._skill is None:
            return Status.FAILURE
        return Status.RUNNING

    def _resolve_pose_sequence(self, pose_name: str):
        if pose_name == "sequence":
            return [
                (list(DEFAULT_LEFT), list(DEFAULT_RIGHT)),
                (list(FORWARD_LEFT), list(FORWARD_RIGHT)),
            ]
        if pose_name == "default":
            return [(list(DEFAULT_LEFT), list(DEFAULT_RIGHT))]
        if pose_name == "forward":
            return [(list(FORWARD_LEFT), list(FORWARD_RIGHT))]
        if pose_name == "custom":
            left_pose = self._resolve_pose("left_pose")
            right_pose = self._resolve_pose("right_pose")
            return [(left_pose, right_pose)] if left_pose and right_pose else []
        return []

    def _resolve_pose(self, key: str):
        raw = self.params.get(key, None)
        if raw is None:
            return None
        if isinstance(raw, list):
            return [float(value) for value in raw] if len(raw) == 6 else None
        if isinstance(raw, str) and raw.strip():
            try:
                parsed = json.loads(raw)
            except Exception:
                return None
            if isinstance(parsed, list) and len(parsed) == 6:
                return [float(value) for value in parsed]
        return None


def _parse_pose_arg(raw: str):
    try:
        parsed = json.loads(raw)
    except Exception as exc:
        raise ValueError(
            "位姿必须是 JSON 数组，例如 '[1.1,0.1,0.7,0,-110,0]': "
            f"{exc}"
        )
    if not isinstance(parsed, list) or len(parsed) != 6:
        raise ValueError("位姿必须包含 6 个值 [x,y,z,yaw,pitch,roll]")
    return [float(value) for value in parsed]


def _run_cli() -> int:
    import argparse

    parser = argparse.ArgumentParser(
        description="直接单测 ArmEeLocalTimedMove，调用 send_arm_ee_local_timed。"
    )
    parser.add_argument(
        "pose_name",
        nargs="?",
        choices=("sequence", "default", "forward", "custom"),
        default="sequence",
        help="目标位姿，默认 sequence；custom 时需要传 --left-pose/--right-pose",
    )
    parser.add_argument("--left-pose", default="", help="custom 左臂 6D 位姿 JSON")
    parser.add_argument("--right-pose", default="", help="custom 右臂 6D 位姿 JSON")
    parser.add_argument("--desire-time", type=float, default=3.0, help="期望执行时间，单位秒")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="只验证节点参数和执行流程，不连接真实硬件",
    )
    parser.add_argument(
        "--no-release-arm-control",
        action="store_true",
        help="执行后不调用 set_arm_control_mode(0)，保持外部控制模式",
    )
    args = parser.parse_args()

    if args.dry_run:
        os.environ["STUDIO_DRY_RUN"] = "1"
        global _DRY_RUN
        _DRY_RUN = True

    params = {
        "pose_name": args.pose_name,
        "desire_time": args.desire_time,
        "release_arm_control": not args.no_release_arm_control,
    }
    if args.pose_name == "custom":
        if not args.left_pose or not args.right_pose:
            parser.error("custom 模式必须同时传 --left-pose 和 --right-pose")
        params["left_pose"] = _parse_pose_arg(args.left_pose)
        params["right_pose"] = _parse_pose_arg(args.right_pose)

    if not _DRY_RUN:
        from apps.test_kuavo_5w_sdk_adapter._scaffold import (
            factory_setup,
            factory_teardown,
        )
        from core.domain.enums import MPCControlMode

        set_hardware_config({
            "robot_type": "leju_wheeled",
            "sdk_managers_whitelist": ["timed"],
            "skip_end_effector": True,
            "skip_camera": True,
            "skip_state_manager": True,
            "skip_force_publishers": True,
        })
        hardware = get_shared_hardware()
        factory_setup(
            hardware,
            need_arm_reset=False,
            need_torso_reset=False,
            focus_ee=False,
            focus_z=False,
            mpc_mode=MPCControlMode.ARM_EE_ONLY,
        )
    else:
        hardware = None

    node = ArmEeLocalTimedMove(
        name="ArmEeLocalTimedMove",
        label=f"ee_local_{args.pose_name}",
        namespace="",
        params=params,
    )

    try:
        node.initialise()
        while True:
            status = node.update()
            print(f"status={status}, feedback={node.feedback_message}")
            if status == Status.SUCCESS:
                return 0
            if status == Status.FAILURE:
                return 1
            time.sleep(0.1)
    finally:
        if not _DRY_RUN and hardware is not None:
            factory_teardown(
                hardware,
                need_arm_reset=False,
                need_torso_reset=False,
            )
            reset_shared_hardware()


if __name__ == "__main__":
    sys.exit(_run_cli())
