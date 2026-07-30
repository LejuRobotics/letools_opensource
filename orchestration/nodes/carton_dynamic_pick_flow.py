# -*- coding: utf-8 -*-
"""CartonDynamicPickFlow：不满垛动态抓取流程薄节点 → carton_dynamic_pick_flow Skill。

薄节点职责：
  - 参数解析（board JSON dict → SkillParams dataclass）
  - py_trees 生命周期（initialise / update / terminate）
  - 委托 CartonDynamicPickFlowSkill 执行全部业务逻辑

参考 chassis_short_move.py → chassis_velocity Skill 的薄节点模式。
"""

import os

from py_trees.common import Status

from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.atomic.refactored_sdk.carton_dynamic_pick_flow import (
    CartonDynamicPickFlowParams,
    CartonDynamicPickFlowSkill,
)

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


@define_manifest(
    label="不满垛动态抓取流程",
    category=["perception", "vision", "planning", "workflow"],
    tree_type="studio_smoke",
    description=(
        "薄节点：不满垛识别+跳过空位+抓取，委托 carton_dynamic_pick_flow Skill。"
        "调用 GDRNPP /infer_top_carton_ids → 解析 pick_sequence → "
        "加载模板子树并按 top_ids 过滤组数 → tick 内部子树。"
    ),
    params=[
        {"name": "box_type_override", "type": "string", "default": "",
         "description": "箱型覆盖(type1/type3)；留空则从 message 解析"},
        {"name": "fallback_pick_sequence", "type": "string",
         "default": "1,2,3,4,5,6",
         "description": "视觉失败时的回退序列(逗号分隔 box_index)"},
        {"name": "prompt", "type": "string",
         "default": "按 Enter 确认开始抓取...",
         "description": "等待 Enter 时的提示语"},
        {"name": "skip_in_dry_run", "type": "bool", "default": "true",
         "description": "dry-run 模式下跳过 input()(仍打印序列)"},
        {"name": "template_subtree_path", "type": "string",
         "default": "orchestration/scenarios/dismantle_box_internal/"
                    "dismantle_box/py_tree_child.json",
         "description": "模板子树 JSON 路径(含6组点位)，按 top_ids 过滤组数"},
        {"name": "max_groups", "type": "int", "default": "6",
         "description": "最大执行组数(模板中组数上限)"},
    ],
    inputs=[],
    outputs=[],
)
class CartonDynamicPickFlow(BaseAction):
    """薄节点：参数解析 + py_trees 生命周期 → Skill。"""

    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._skill = None
        self._init_result = None

    def initialise(self):
        # dry-run 下 hardware 可能为 None，Skill 内部用 fallback 序列
        try:
            hw = get_shared_hardware()
        except Exception:
            hw = None

        skill_params = CartonDynamicPickFlowParams(
            box_type_override=str(self.params.get("box_type_override", "")),
            fallback_pick_sequence=str(
                self.params.get("fallback_pick_sequence", "1,2,3,4,5,6")
            ),
            prompt=str(self.params.get("prompt", "按 Enter 确认开始抓取...")),
            skip_in_dry_run=str(
                self.params.get("skip_in_dry_run", "true")
            ).lower() in ("1", "true", "yes"),
            template_subtree_path=str(self.params.get(
                "template_subtree_path",
                "orchestration/scenarios/dismantle_box_internal/"
                "dismantle_box/py_tree_child.json",
            )),
            max_groups=int(self.params.get("max_groups", 6)),
        )
        self._skill = CartonDynamicPickFlowSkill(
            hardware=hw,
            blackboard=self.global_blackboard,
        )
        self._init_result = self._skill.initialize(skill_params)
        if not self._init_result.success:
            self.feedback_message = (
                self._init_result.message or "init failed"
            )

    def update(self):
        if self._skill is None:
            return Status.FAILURE

        if not self._init_result or not self._init_result.success:
            self.feedback_message = (
                self._init_result.message if self._init_result
                else "init failed"
            )
            return Status.FAILURE

        if self._skill.is_finished():
            return (
                Status.SUCCESS if self._skill._success
                else Status.FAILURE
            )

        result = self._skill.execute()
        if not result.success:
            self.feedback_message = result.message or "execute failed"
            return Status.FAILURE

        return Status.RUNNING

    def terminate(self, new_status):
        if self._skill is not None:
            try:
                self._skill.cancel()
            except Exception:
                pass
        self._skill = None
