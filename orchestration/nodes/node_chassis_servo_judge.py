# -*- coding: utf-8 -*-
"""NodeChassisServoJudge：读黑板观测量 → 伺服判断 → 写回判决（纯计算节点）。

飞书 §3.3 / §4.4 的「判断」薄封装：读视觉感知模块写下的 AlignObservation 与
控制器写下的累积量/迭代号，调 ChassisServoJudgeSkill 判断收敛/继续/超限失败，
把判决写回黑板（latest_servo_verdict + _version）供上层编排收口。

本节点不访问硬件（判断是纯计算），故无 get_shared_hardware() 调用，
可脱离 ROS / 硬件单测。

黑板契约
--------
读 latest_servo_error              AlignObservation 或 dev 的 ServoError（鸭子类型判别）
读 servo_exec_idle               bool，False = 底盘有动作在途，此时不判决
读 latest_servo_command          ServoCommand（取 iter_index / cumulative_xy_m）
读 latest_servo_verdict           上一轮判决（取 prev_converge_count）
写 latest_servo_verdict           本轮判决
写 latest_servo_verdict_version   int，每判一次自增

状态语义
--------
* `converged` → SUCCESS（正常收敛收口）；
* `failed_cum` / `failed_iter` → FAILURE（未对准，明确失败）；
* `continue` / 等观测 / 节拍门禁未放开 → RUNNING。
"""

import os

import py_trees
from py_trees.common import Status

from core.common.logger import get_logger
from core.domain.servo_types import (
    ServoCommand, is_observation_like, observation_field)
from orchestration.nodes.base_node import BaseAction
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.chassis_servo_judge import (
    ChassisServoJudgeParams, ChassisServoJudgeSkill)

logger = get_logger(__name__)

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


def _read_blackboard(client, key, default=None):
    """读黑板键；py_trees Client.__getattr__ 抛 KeyError，须显式 try（同 node_pallet_pose）。"""
    try:
        return getattr(client, key)
    except KeyError:
        return default


def _read_version(client, key) -> int:
    try:
        return int(_read_blackboard(client, key, 0))
    except (TypeError, ValueError):
        return 0


def _read_idle(client, key) -> bool:
    """读节拍门禁键；从未写过（执行节点还没构造）当作空闲，别把树卡死。"""
    return bool(_read_blackboard(client, key, True))


@define_manifest(
    label="底盘伺服判断",
    category=["skill", "chassis", "servo"],
    tree_type="studio_smoke",
    description="读黑板观测量 → 收敛/继续/超限失败 → 写回判决（纯计算）",
    params=[
        {"name": "obs_key", "type": "string", "default": "latest_servo_error",
         "description": "读观测量的黑板键名"},
        {"name": "command_key", "type": "string", "default": "latest_servo_command",
         "description": "读 ServoCommand 的黑板键名（取 iter/cum）"},
        {"name": "verdict_key", "type": "string", "default": "latest_servo_verdict",
         "description": "写判决的黑板键名"},
        {"name": "idle_key", "type": "string", "default": "servo_exec_idle",
         "description": "读节拍门禁的黑板键名（False=底盘在途，本节点不判决、不收口）"},
        {"name": "servo_stop_threshold_px", "type": "float", "default": "10.0",
         "description": "收敛像素阈值（现场标定）"},
        {"name": "servo_stop_theta_rad", "type": "float", "default": "0.02",
         "description": "角度收敛阈值(rad)"},
        {"name": "servo_converge_frames", "type": "int", "default": "2",
         "description": "连续收敛帧数"},
        {"name": "servo_max_iters", "type": "int", "default": "8",
         "description": "最大迭代次数"},
        {"name": "servo_cumulative_limit_m", "type": "float", "default": "0.10",
         "description": "累积位移安全保护上限(m)"},
    ],
    inputs=[{"name": "latest_servo_error", "type": "object",
             "description": "AlignObservation/ServoError：两路像素偏差 + 偏差角 + valid"}],
    outputs=[{"name": "latest_servo_verdict", "type": "object",
              "description": "判决：verdict + converge_count + reason"}],
)
class NodeChassisServoJudge(BaseAction):
    """底盘伺服判断节点（纯计算）：观测量 → 收敛/继续/超限失败。"""

    _MISSING = object()

    def __init__(self, name, label, namespace, params):
        super(NodeChassisServoJudge, self).__init__(name, label, namespace, params)
        self._obs_key = str(self.params.get("obs_key", "latest_servo_error"))
        self._cmd_key = str(self.params.get("command_key", "latest_servo_command"))
        self._verdict_key = str(self.params.get("verdict_key", "latest_servo_verdict"))
        self._idle_key = str(self.params.get("idle_key", "servo_exec_idle"))

        self._skill = None
        self._version_seen = None

        # 读 obs + obs_version（版本门禁依赖后者）+ cmd（取 iter/cum）+ idle（节拍门禁）
        for key in (self._obs_key, f"{self._obs_key}_version", self._cmd_key,
                    self._idle_key):
            self.global_blackboard.register_key(
                key=key, access=py_trees.common.Access.READ)
        for key in (self._verdict_key, f"{self._verdict_key}_version"):
            self.global_blackboard.register_key(
                key=key, access=py_trees.common.Access.WRITE)
        self.global_blackboard.set(self._verdict_key, None)
        setattr(self.global_blackboard, f"{self._verdict_key}_version", 0)

    def initialise(self):
        self._skill = None
        self._version_seen = None

    def update(self):
        if _DRY_RUN:
            self.feedback_message = "dry-run chassis_servo_judge"
            return Status.SUCCESS

        # 节拍门禁：底盘还在走上一步时不判决。这一步是整个修复的关键 —— 否则本节点
        # 会在 iter=0 的位移还没走完时就 converged，SUCCESS 让 Parallel(success_on_one)
        # 立刻收口，NodeEnableVelControl.terminate() 把权限还原成 True，在途动作被掐断。
        if not _read_idle(self.global_blackboard, self._idle_key):
            self.feedback_message = "底盘在途，等到达后再判决"
            return Status.RUNNING

        # 读观测量
        obs = _read_blackboard(self.global_blackboard, self._obs_key, self._MISSING)
        if obs is self._MISSING or obs is None:
            self.feedback_message = "等观测（黑板上还没有 AlignObservation）"
            return Status.RUNNING
        if not is_observation_like(obs):
            self.feedback_message = (
                f"{self._obs_key} 不是观测量（缺 e_bottom_px/e_right_px/theta_rad）")
            return Status.FAILURE
        if not observation_field(obs, "valid", True):
            self.feedback_message = (
                f"观测无效：{observation_field(obs, 'reject', '') or 'unknown'}")
            return Status.FAILURE

        # 版本门禁：观测没更新就不重判
        newest_version = _read_version(self.global_blackboard, f"{self._obs_key}_version")
        if newest_version == self._version_seen:
            return Status.RUNNING

        # 读控制器写下的 ServoCommand，取 iter_index / cumulative_xy_m
        cmd = _read_blackboard(self.global_blackboard, self._cmd_key, self._MISSING)
        if cmd is self._MISSING or cmd is None or not isinstance(cmd, ServoCommand):
            iter_index = 0
            cumulative_xy_m = 0.0
        else:
            iter_index = int(getattr(cmd, "iter_index", 0))
            cumulative_xy_m = float(getattr(cmd, "cumulative_xy_m", 0.0))

        # 读上一轮判决的 converge_count，做连续帧递推
        prev_verdict = _read_blackboard(self.global_blackboard, self._verdict_key, self._MISSING)
        if prev_verdict is self._MISSING or prev_verdict is None:
            prev_count = 0
        else:
            prev_count = int(prev_verdict.get("converge_count", 0)) if isinstance(prev_verdict, dict) else 0

        skill_params = ChassisServoJudgeParams(
            observation=obs,
            iter_index=iter_index,
            cumulative_xy_m=cumulative_xy_m,
            prev_converge_count=prev_count,
            servo_stop_threshold_px=float(self.params.get("servo_stop_threshold_px", 10.0)),
            servo_stop_theta_rad=float(self.params.get("servo_stop_theta_rad", 0.02)),
            servo_converge_frames=int(self.params.get("servo_converge_frames", 2)),
            servo_max_iters=int(self.params.get("servo_max_iters", 8)),
            servo_cumulative_limit_m=float(self.params.get("servo_cumulative_limit_m", 0.10)),
        )

        if self._skill is None:
            self._skill = ChassisServoJudgeSkill()
        init = self._skill.initialize(skill_params)
        if not init.success:
            self.feedback_message = init.message
            return Status.FAILURE

        result = self._skill.execute()
        if not result.success or not result.data:
            self.feedback_message = result.message
            return Status.FAILURE

        verdict_data = result.data
        self.global_blackboard.set(self._verdict_key, verdict_data)
        cur_ver = getattr(self.global_blackboard, f"{self._verdict_key}_version", 0)
        setattr(self.global_blackboard, f"{self._verdict_key}_version", cur_ver + 1)
        self._version_seen = newest_version
        self.feedback_message = result.message

        # 收敛 → SUCCESS 收口；超累积量/超迭代 → FAILURE（明确未对准失败）；
        # 继续 → RUNNING。
        # 走到这里 servo_exec_idle 必为 True（开头的节拍门禁挡住了在途情况），
        # 所以收口时底盘没有在途动作，terminate() 还原速度权限是安全的。
        verdict = verdict_data.get("verdict", "")
        if verdict in ("failed_iter", "failed_cum"):
            logger.warning("chassis_servo_judge: %s → FAILURE（未对准）", verdict)
            return Status.FAILURE
        if verdict == "converged":
            return Status.SUCCESS
        return Status.RUNNING
