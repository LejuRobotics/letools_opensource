# -*- coding: utf-8 -*-
"""Atomic skill: arm_ee_setup_jibot.

对齐 test_arm_ee_single_both.py 的 main() 前置设置（不含手臂复位）：
- set_arm_control_mode(2)  → 切换到外部控制模式
- set_mpc_mode(ARM_EE_ONLY) → 仅控制手臂末端（躯干/底盘不参与正逆解）
- _ensure_ee_publisher()    → 预创建 Publisher/Subscriber

此 skill 是 ArmEePoseJibotSkill 的前置依赖，须在其之前执行。
"""

from dataclasses import dataclass
from typing import Optional

import time

from core.common.logger import get_logger
from core.domain.result import Result
from core.domain.enums import MPCControlMode
from core.domain.skill_params import SkillParams
from core.interfaces.i_hardware import IHardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.base.skill_base import SkillBase

logger = get_logger(__name__)


@dataclass
class ArmEESetupJibotParams(SkillParams):
    """对齐 test_arm_ee_single_both.py main() 前置设置。"""

    skill_name: str = "arm_ee_setup_jibot"
    timeout: float = 30.0


@define_manifest(
    label="手臂末端控制前置设置（MPC + 外部控制 + Publisher）",
    category=["motion", "arm"],
    tree_type="studio_smoke",
    description=(
        "对齐 test_arm_ee_single_both.py main()。"
        "执行 set_arm_control_mode(2) + set_mpc_mode(ARM_EE_ONLY) + _ensure_ee_publisher()。"
        "ARM_EE_ONLY 确保仅胳膊正逆解，躯干/底盘不参与。"
        "不含手臂复位(1)，保留下一步骤之前的当前臂姿。"
    ),
    params=[],
    inputs=[],
    outputs=[],
)
class ArmEESetupJibotSkill(SkillBase):
    """手臂末端控制前置设置：MPC 模式 + 外部控制 + Publisher 预建。"""

    def __init__(self, hardware: IHardware):
        super().__init__(name="arm_ee_setup_jibot")
        self.hardware = hardware
        self.params: Optional[ArmEESetupJibotParams] = None
        self._done = False

    def on_initialize(self, params: ArmEESetupJibotParams) -> Result:
        if not isinstance(params, ArmEESetupJibotParams):
            return Result.fail("Invalid parameters for ArmEESetupJibotSkill")
        self.params = params
        self._done = False
        return Result.ok()

    def on_execute(self) -> Result:
        if self._done:
            return Result.ok("ArmEESetupJibotSkill already finished")

        try:
            # Step 1: 切换到外部控制模式（接受话题指令）
            logger.info("[arm_ee_setup] 1/3 set_arm_control_mode(2) — 外部控制")
            result = self.hardware.set_arm_control_mode(2)
            if not result.success:
                self._done = True
                return Result.fail(f"set_arm_control_mode(2) failed: {result.message}")
            time.sleep(0.5)

            # Step 2: MPC 模式 → ARM_EE_ONLY（仅胳膊正逆解，躯干/底盘不参与）
            logger.info("[arm_ee_setup] 2/3 set_mpc_mode(ARM_EE_ONLY) — 仅手臂末端受控")
            result = self.hardware.set_mpc_mode(MPCControlMode.ARM_EE_ONLY)
            if not result.success:
                self._done = True
                return Result.fail(f"set_mpc_mode(ARM_EE_ONLY) failed: {result.message}")
            time.sleep(0.5)

            # Step 3: 预创建 Publisher/Subscriber（匹配源脚本 pub+sub 时序）
            logger.info("[arm_ee_setup] 3/3 _ensure_ee_publisher() — 预创建 Publisher")
            self.hardware._ensure_ee_publisher()

            self._done = True
            logger.info("[arm_ee_setup] ✅ 前置设置全部完成")
            return Result.ok("arm_ee_setup complete")

        except Exception as e:
            self._done = True
            logger.error(f"[arm_ee_setup] ❌ 异常: {e}")
            return Result.fail(f"arm_ee_setup error: {e}")

    def on_is_finished(self) -> bool:
        return self._done
