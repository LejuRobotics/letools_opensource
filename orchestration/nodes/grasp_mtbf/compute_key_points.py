# -*- coding: utf-8 -*-
"""根据已获取的 Tag 和箱子尺寸计算 pick/place 的各阶段 TimedCmd 命令。"""

import json
import math

from py_trees.common import Access, Status

from orchestration.nodes.base_node import BaseAction
from orchestration.utils.manifest_decorators import define_manifest


# 各阶段相对 Tag 基准位置的额外偏置：(x, y, z)。
# X 为负表示向后撤退；Y 在半箱宽基础上增减，左右臂围绕 Tag Y 对称。
_STAGE_OFFSETS = {
    "pick_align": (-0.15, 0.15, 0.0),
    "pick_approach": (0.0, 0.15, 0.0),
    "pick": (0.0, 0.0, 0.0),
    "pick_lift": (-0.15, 0.0, 0.15),
    "place_above": (-0.15, 0.0, 0.15),
    "place": (0.0, 0.0, 0.0),
    "place_release": (0.0, 0.15, 0.0),
    "place_retreat": (-0.15, 0.15, 0.0),
}
_STAGE_SUFFIXES = {
    "pick": ("align", "approach", "", "lift"),
    "place": ("above", "", "release", "retreat"),
}
_TORSO_STAGES = {"pick": "pick", "place": "place"}


@define_manifest(
    label="根据 Tag 计算搬箱关键点",
    category=["grasp_mtbf", "calc"],
    tree_type="studio_smoke",
    description="接收 Tag 的 XYZ，结合相对偏置和 BASE 系箱子尺寸，覆盖对应阶段命令",
    params=[
        {"name": "phase", "type": "string", "default": "pick",
         "description": "pick 或 place", "options": ["pick", "place"]},
        {"name": "tag_xyz", "type": "json", "default": "[0,0,0]",
         "description": "目标 Tag 的 [x,y,z]，单位米"},
        {"name": "position_offset", "type": "json", "default": "[0,0,0]",
         "description": "目标位置相对 Tag 的 [x,y,z] 偏置，单位米"},
        {"name": "box_size", "type": "json", "default": "[0.4,0.3,0.25]",
         "description": "箱子在 BASE 系下的尺寸 [x,y,z]，单位米"},
    ],
    inputs=[],
    outputs=[],
)
class ComputeKeyPoints(BaseAction):
    """将 Tag 位置、位置偏置和箱体尺寸转换为双臂/躯干目标。"""

    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._commands = {}
        self._error = ""
        self._tag_xyz_board_key = self.params.get("tag_xyz__board_key", "")
        self._phase = str(self.params.get("phase", "pick")).strip().lower()
        if self._phase not in ("pick", "place"):
            raise ValueError("phase 必须是 pick 或 place")
        self._output_keys = [
            f"{self._phase}_{suffix}" if suffix else self._phase
            for suffix in _STAGE_SUFFIXES[self._phase]
        ]
        self._eulers = {key: self._read_eulers(key) for key in self._output_keys}

    def _read_eulers(self, key):
        """读取黑板原始命令中的左右臂姿态，后续仅覆盖 XYZ。"""
        self.global_blackboard.register_key(key=key, access=Access.READ)
        planners = {
            command["planner_index"]: command
            for command in self.global_blackboard.get(key)
        }
        return {index: planners[index]["cmd_vec"][3:] for index in (6, 7)}

    def _read_tag_xyz(self):
        raw = self.params.get("tag_xyz", [0.0, 0.0, 0.0])
        if self._tag_xyz_board_key:
            self.global_blackboard.register_key(
                key=self._tag_xyz_board_key, access=Access.READ
            )
            if not self.global_blackboard.exists(self._tag_xyz_board_key):
                raise ValueError(f"{self._tag_xyz_board_key} 未就绪")
            raw = self.global_blackboard.get(self._tag_xyz_board_key)
        return _vector(raw, 3, "tag_xyz")

    def initialise(self):
        self._commands = {}
        self._error = ""
        try:
            tag_xyz = self._read_tag_xyz()
            size = _vector(self.params["box_size"], 3, "box_size", positive=True)
            offset = _vector(self.params["position_offset"], 3, "position_offset")
            # 当前约定：Tag 位置、position_offset 与 BASE 轴方向一致。
            # 若感知输出为 ODOM 等其他坐标系，必须先完成坐标变换。
            target = [value + delta for value, delta in zip(tag_xyz, offset)]
            torso = self._torso(_TORSO_STAGES[self._phase], target, size)
            self._commands = {
                key: self._stage_commands(
                    key, target, size, self._eulers[key], torso
                )
                for key in self._output_keys
            }
        except (AttributeError, KeyError, TypeError, ValueError) as exc:
            self._error = str(exc)

    def update(self):
        if self._error:
            self.feedback_message = f"关键点计算失败: {self._error}"
            return Status.FAILURE
        try:
            for key, commands in self._commands.items():
                self.global_blackboard.register_key(key=key, access=Access.WRITE)
                self.global_blackboard.set(key, commands)
        except Exception as exc:
            self.feedback_message = f"关键点写入黑板失败: {exc}"
            return Status.FAILURE
        self.feedback_message = f"已覆盖黑板关键点: {', '.join(self._commands)}"
        return Status.SUCCESS

    @staticmethod
    def _ee_xyz(stage, target, size):
        """计算末端 X、相对 Tag 中心的 Y 间距和 Z。"""
        dx, dy, dz = _STAGE_OFFSETS[stage]
        return (
            target[0] + size[0] / 2.0 + dx,
            size[1] / 2.0 + dy,
            target[2] + dz,
        )

    @classmethod
    def _torso(cls, stage, target, size):
        """依据 pick/place 末端位置计算躯干，同一抓/放序列内保持不变。"""
        x, _, z = cls._ee_xyz(stage, target, size)
        return [
            0.12 + 0.5 * (x - 0.6),
            1.0 + 0.5 * (z - 1.0),
            0.0,
            0.0,
        ]

    @classmethod
    def _stage_commands(cls, stage, target, size, eulers, torso):
        x, y_offset, z = cls._ee_xyz(stage, target, size)
        left_y = target[1] + y_offset
        right_y = target[1] - y_offset
        return [
            {"planner_index": 2, "cmd_vec": torso},
            {"planner_index": 6, "cmd_vec": [x, left_y, z] + eulers[6]},
            {"planner_index": 7, "cmd_vec": [x, right_y, z] + eulers[7]},
        ]


def _vector(raw, length, name, positive=False):
    if isinstance(raw, str):
        raw = json.loads(raw)
    if not isinstance(raw, (list, tuple)) or len(raw) != length:
        raise ValueError(f"{name} 必须包含 {length} 个数")
    values = []
    for raw_value in raw:
        value = float(raw_value)
        if not math.isfinite(value):
            raise ValueError(f"{name} 必须是有限数")
        values.append(value)
    if positive and any(value <= 0.0 for value in values):
        raise ValueError(f"{name} 中的尺寸必须大于 0")
    return values
