# -*- coding: utf-8 -*-
"""带双臂与下肢关节快照恢复的 MPC 码垛直线搬运。

继承 :mod:`palletizing_line_mpc` 的离线轨迹控制逻辑，默认沿用原码垛路径，也可选择
X/Y/Z 单轴往返。首次执行前记录双臂 14 关节与下肢 4 关节，并在搬运正常完成、失败或
异常后尝试恢复全部 18 个关节。恢复命令发送成功不代表到位：本技能会重新读取关节角，
误差超过阈值即返回失败。

关节单位沿用适配器现有契约：手臂快照为弧度，下发前转成角度；下肢快照经
``_from_rad`` 返回用户单位，当前验收入口使用默认 ``angle_unit=deg``，与
``send_leg_joint_command`` 的角度命令一致。
"""

import math
import time
from dataclasses import dataclass
from typing import List, Optional

from core.common.logger import get_logger
from core.domain.palletizing_path import PathSegment
from core.domain.result import Result
from core.interfaces.i_hardware import IHardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.palletizing_line_mpc import (
    PalletizingLineMpcParams,
    PalletizingLineMpcSkill,
)

logger = get_logger(__name__)

_ARM_JOINT_COUNT = 14
_LEG_JOINT_COUNT = 4


@dataclass
class PalletizingLineMpcRestoreParams(PalletizingLineMpcParams):
    """带关节快照恢复的 MPC 直线搬运参数。"""

    skill_name: str = "palletizing_line_mpc_restore"
    prepare_pose: bool = False
    roundtrip_axis: str = "full"  # full=原码垛路径；x/y/z=对应单轴往返
    restore_time: float = 5.0
    restore_verify_delay: float = 0.2
    restore_arm_tolerance_deg: float = 2.0
    restore_leg_tolerance: float = 2.0  # 与 get_leg_joint_positions 相同的用户单位


@define_manifest(
    label="码垛直线搬运（MPC，恢复关节快照）",
    category=["motion", "arm", "workflow"],
    tree_type="studio_smoke",
    description=(
        "沿用 MPC 笛卡尔直线搬运，可选原 10 段路径或 X/Y/Z 单轴往返；"
        "执行前记录双臂和下肢 18 关节，完成、失败或异常后均尝试恢复并读回验证。"
    ),
    params=[
        {"name": "arms", "type": "string", "default": "both",
         "description": "参与搬运的手臂: 'both' / 'left' / 'right'；恢复快照始终包含双臂"},
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
        {"name": "prepare_pose", "type": "bool", "default": "false",
         "description": "是否在快照后下发预备姿态；默认关闭"},
        {"name": "roundtrip_axis", "type": "string", "default": "full",
         "options": ["full", "x", "y", "z"],
         "description": "full=原码垛路径；x/y/z=对应单轴往返"},
        {"name": "restore_time", "type": "float", "default": "5.0",
         "description": "双臂恢复轨迹时长（秒）；下肢恢复沿用适配器阻塞等待"},
        {"name": "restore_arm_tolerance_deg", "type": "float", "default": "2.0",
         "description": "手臂恢复验证的单关节最大允许误差（度）"},
        {"name": "restore_leg_tolerance", "type": "float", "default": "2.0",
         "description": "下肢恢复验证误差阈值（与下肢读取接口相同的用户单位）"},
    ],
    inputs=[],
    outputs=[],
)
class PalletizingLineMpcRestoreSkill(PalletizingLineMpcSkill):
    """在原 MPC 搬运外包裹 18 关节快照、恢复与到位验证。"""

    def __init__(self, hardware: IHardware):
        super().__init__(hardware=hardware)
        self._name = "palletizing_line_mpc_restore"
        self.params: Optional[PalletizingLineMpcRestoreParams] = None
        self._initial_arm_positions: Optional[List[float]] = None
        self._initial_leg_positions: Optional[List[float]] = None

    def on_initialize(self, params: PalletizingLineMpcRestoreParams) -> Result:
        if not isinstance(params, PalletizingLineMpcRestoreParams):
            return Result.fail("Invalid parameters for PalletizingLineMpcRestoreSkill")
        initialized = super().on_initialize(params)
        if not initialized.success:
            return initialized
        if params.roundtrip_axis not in ("full", "x", "y", "z"):
            return Result.fail(
                f"roundtrip_axis 仅支持 full/x/y/z，收到: {params.roundtrip_axis}")
        if params.roundtrip_axis != "full":
            distance = {
                "x": params.x_distance,
                "y": params.y_offset,
                "z": params.z_height,
            }[params.roundtrip_axis]
            if distance <= 0:
                return Result.fail(
                    f"{params.roundtrip_axis} 轴往返距离必须为正，收到: {distance}")
        if params.restore_time <= 0:
            return Result.fail(f"restore_time 必须为正，收到: {params.restore_time}")
        if params.restore_verify_delay < 0:
            return Result.fail(
                f"restore_verify_delay 不能为负，收到: {params.restore_verify_delay}")
        if params.restore_arm_tolerance_deg <= 0 or params.restore_leg_tolerance <= 0:
            return Result.fail("手臂与下肢恢复验证阈值必须为正")

        self._initial_arm_positions = None
        self._initial_leg_positions = None
        return Result.ok()

    def _build_path(self, start, x_dir, y_dir, z_dir, path_params):
        """full 保持原码垛路径；x/y/z 仅生成目标轴伸出与原路返回两段。"""
        axis = self.params.roundtrip_axis
        if axis == "full":
            return super()._build_path(start, x_dir, y_dir, z_dir, path_params)

        start_point = tuple(float(value) for value in start)
        distance = {
            "x": self.params.x_distance,
            "y": self.params.y_offset,
            "z": self.params.z_height,
        }[axis]
        direction = {"x": x_dir, "y": y_dir, "z": z_dir}[axis]
        end_point = tuple(
            start_point[index] + float(distance) * float(direction[index])
            for index in range(3))
        kind = "vertical" if axis == "z" else "horizontal"
        return [
            PathSegment(name=f"roundtrip_{axis}_out", start=start_point, end=end_point,
                        duration=self.params.edge_duration, kind=kind),
            PathSegment(name=f"roundtrip_{axis}_back", start=end_point, end=start_point,
                        duration=self.params.edge_duration, kind=kind),
        ]

    def on_execute(self) -> Result:
        if self._done:
            return Result.ok("PalletizingLineMpcRestoreSkill already finished")

        snapshot = self._capture_joint_snapshot()
        if not snapshot.success:
            self._done = True
            return Result.fail(f"无法记录初始关节快照，已取消搬运: {snapshot.message}")

        try:
            movement = super().on_execute()
        except Exception as exc:
            logger.exception("MPC 直线搬运异常，开始恢复双臂与下肢关节快照")
            movement = Result.fail(f"MPC 直线搬运异常: {exc}")
        finally:
            # finally 覆盖原技能的每个 return；KeyboardInterrupt 等异常也会先尝试恢复。
            restored = self._restore_joint_snapshot()
            self._done = True

        if restored.success:
            if movement.success:
                return Result.ok(f"{movement.message}；{restored.message}")
            return Result.fail(f"{movement.message}；{restored.message}")
        if movement.success:
            return Result.fail(f"MPC 直线搬运完成，但{restored.message}")
        return Result.fail(f"{movement.message}；{restored.message}")

    def _capture_joint_snapshot(self) -> Result:
        arms = self._read_positions(
            "get_arm_joint_positions", _ARM_JOINT_COUNT, "双臂关节角")
        if not arms.success:
            return arms
        legs = self._read_positions(
            "get_leg_joint_positions", _LEG_JOINT_COUNT, "下肢关节角")
        if not legs.success:
            return legs

        self._initial_arm_positions = list(arms.data)
        self._initial_leg_positions = list(legs.data)
        logger.info(
            "已记录初始关节快照：双臂 14 关节 + 下肢 4 关节（knee/leg/waist_pitch/waist_yaw）")
        return Result.ok()

    def _restore_joint_snapshot(self) -> Result:
        if self._initial_arm_positions is None or self._initial_leg_positions is None:
            return Result.fail("关节恢复失败: 缺少完整初始快照")

        errors = []
        logger.info("开始恢复双臂与下肢共 18 个关节到初始快照")

        # 只确保外部控制模式；mode=1 会运动到固件默认目标，不能用于快照恢复。
        # 即使模式切换失败，仍继续尝试两组关节命令，避免遗漏可恢复的关节。
        mode_result = self._call_hardware("set_arm_control_mode", 2)
        if not mode_result.success:
            errors.append(f"切换手臂外部控制失败: {mode_result.message}")

        arm_command = self._call_hardware(
            "send_arm_joint_trajectory",
            [math.degrees(value) for value in self._initial_arm_positions],
            self.params.restore_time,
        )
        if not arm_command.success:
            errors.append(f"双臂恢复命令失败: {arm_command.message}")

        # 不能改走 send_leg_joint_sdk：其 finally 会 reset_torso_to_initial，覆盖快照。
        # 当前入口使用默认 angle_unit=deg，与直接下肢命令的角度契约一致。
        leg_command = self._call_hardware(
            "send_leg_joint_command", self._initial_leg_positions)
        if not leg_command.success:
            errors.append(f"下肢恢复命令失败: {leg_command.message}")

        if self.params.restore_verify_delay > 0:
            time.sleep(self.params.restore_verify_delay)

        arm_error_deg = None
        leg_error = None
        current_arms = self._read_positions(
            "get_arm_joint_positions", _ARM_JOINT_COUNT, "恢复后的双臂关节角")
        if current_arms.success:
            arm_error_deg = max(
                abs(math.degrees(current - initial))
                for current, initial in zip(current_arms.data, self._initial_arm_positions)
            )
            if arm_error_deg > self.params.restore_arm_tolerance_deg:
                errors.append(
                    f"双臂未恢复到快照: 最大误差 {arm_error_deg:.3f}° > "
                    f"{self.params.restore_arm_tolerance_deg:.3f}°")
        else:
            errors.append(f"双臂恢复验证读取失败: {current_arms.message}")

        current_legs = self._read_positions(
            "get_leg_joint_positions", _LEG_JOINT_COUNT, "恢复后的下肢关节角")
        if current_legs.success:
            leg_error = max(
                abs(current - initial)
                for current, initial in zip(current_legs.data, self._initial_leg_positions)
            )
            if leg_error > self.params.restore_leg_tolerance:
                errors.append(
                    f"下肢未恢复到快照: 最大误差 {leg_error:.6f} > "
                    f"{self.params.restore_leg_tolerance:.6f}（用户单位）")
        else:
            errors.append(f"下肢恢复验证读取失败: {current_legs.message}")

        if errors:
            for error in errors:
                logger.error("%s", error)
            return Result.fail("关节恢复失败: " + "；".join(errors))

        message = (
            f"双臂与下肢 18 关节已恢复并验证（手臂最大误差 {arm_error_deg:.3f}°，"
            f"下肢最大误差 {leg_error:.6f} 用户单位）")
        logger.info("%s", message)
        return Result.ok(message)

    def _read_positions(self, method_name: str, expected_count: int, label: str) -> Result:
        method = getattr(self.hardware, method_name, None)
        if method is None:
            return Result.fail(f"硬件适配器缺少 {method_name} 接口")
        try:
            result = method()
            if not result.success:
                return Result.fail(f"读取{label}失败: {result.message}")
            positions = [float(value) for value in result.data]
            if len(positions) != expected_count:
                return Result.fail(
                    f"{label}数量错误: 期望 {expected_count} 个，实际 {len(positions)} 个")
            if not all(math.isfinite(value) for value in positions):
                return Result.fail(f"{label}包含非有限值: {positions}")
            return Result.ok(data=positions)
        except Exception as exc:
            return Result.fail(f"读取{label}异常: {exc}")

    def _call_hardware(self, method_name: str, *args) -> Result:
        method = getattr(self.hardware, method_name, None)
        if method is None:
            return Result.fail(f"硬件适配器缺少 {method_name} 接口")
        try:
            return method(*args)
        except Exception as exc:
            return Result.fail(f"调用 {method_name} 异常: {exc}")
