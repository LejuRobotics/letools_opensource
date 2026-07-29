# -*- coding: utf-8 -*-
"""不满垛动态抓取流程 Skill。

封装不满垛全流程：调视觉 → 解析 pick_sequence → 跳过空位 → 打印确认 →
加载模板子树并按 top_ids 过滤组数 → tick 内部子树。

参考 kuavo_ros_application 的 CartonMainFlow 嵌套行为树模式。
"""

import json
import os
from dataclasses import dataclass
from typing import List, Optional

import py_trees
from py_trees.common import Status

from core.common.logger import get_logger
from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.interfaces.i_hardware import IHardware
from skills.base.skill_base import SkillBase
from orchestration.nodes.carton_sequence import build_pick_summary
from orchestration.utils.manifest_decorators import define_manifest

logger = get_logger(__name__)

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")

try:
    import rospy
    HAS_ROSPY = True
except ImportError:
    HAS_ROSPY = False


@dataclass
class CartonDynamicPickFlowParams(SkillParams):
    """不满垛动态抓取流程参数。"""

    skill_name: str = "carton_dynamic_pick_flow"
    box_type_override: str = ""
    fallback_pick_sequence: str = "1,2,3,4,5,6"
    prompt: str = "按 Enter 确认开始抓取..."
    skip_in_dry_run: bool = True
    template_subtree_path: str = (
        "orchestration/scenarios/dismantle_box_internal/"
        "dismantle_box/py_tree_child.json"
    )
    max_groups: int = 6
    timeout: float = 7200.0  # 2小时，抓取流程很长


@define_manifest(
    label="不满垛动态抓取流程",
    category=["perception", "vision", "planning", "workflow"],
    tree_type="studio_smoke",
    description=(
        "单 Skill 封装不满垛全流程：调用 GDRNPP /infer_top_carton_ids → "
        "解析 message 生成 pick_sequence → 打印+等待 Enter 确认 → "
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
class CartonDynamicPickFlowSkill(SkillBase):
    """不满垛动态抓取流程 Skill（嵌套行为树模式）。

    执行流程:
      1. on_initialize(): 调视觉 → build_pick_summary → pick_sequence
         → 计算跳过空位 → 打印+等Enter → 加载模板+过滤组+构建内部子树
      2. on_execute(): tick 内部子树
      3. on_is_finished(): 内部子树是否完成
    """

    def __init__(self, hardware: IHardware, blackboard=None):
        super().__init__(name="carton_dynamic_pick_flow")
        self.hardware = hardware
        self.blackboard = blackboard
        self.params: Optional[CartonDynamicPickFlowParams] = None
        self._pick_sequence: List[int] = []
        self._summary: dict = {}
        self._tree = None
        self._kept_group_indices: List[int] = []
        self._done = False
        self._success = False

    # ==================== 生命周期 ====================

    def on_initialize(self, params: CartonDynamicPickFlowParams) -> Result:
        if not isinstance(params, CartonDynamicPickFlowParams):
            return Result.fail("Invalid parameters for CartonDynamicPickFlowSkill")

        self.params = params
        self._done = False
        self._success = False
        self._pick_sequence = []
        self._summary = {}
        self._tree = None
        self._kept_group_indices = []

        # --- 1. 调用视觉生成 pick_sequence ---
        self._pick_sequence = self._resolve_sequence()
        if not self._pick_sequence:
            self._done = True
            self._success = False
            return Result.fail("pick_sequence 为空")

        # --- 2. 根据 top_ids 计算需要执行的组号(跳过空位) ---
        self._compute_kept_groups()

        # --- 3. 打印序列 + 等待 Enter ---
        self._print_and_confirm()

        # --- 4. 构建内部子树 ---
        result = self._build_internal_tree()
        if not result.success:
            self._done = True
            self._success = False
            return result

        return Result.ok()

    def on_execute(self) -> Result:
        if self._done:
            return Result.ok() if self._success else Result.fail("已失败")

        if self._tree is None:
            self._done = True
            self._success = False
            return Result.fail("内部子树未构建")

        # --- tick 内部子树 ---
        self._tree.tick()
        root_status = self._tree.root.status

        if root_status == Status.RUNNING:
            return Result.ok()

        self._done = True
        self._success = (root_status == Status.SUCCESS)
        msg = f"内部子树完成, status={root_status}"
        if self._success:
            return Result.ok(msg)
        return Result.fail(msg)

    def on_is_finished(self) -> bool:
        return self._done

    def on_cancel(self) -> Result:
        self._tree = None
        self._done = True
        return Result.ok("cancelled")

    # ==================== 内部方法 ====================

    def _resolve_sequence(self):
        """调用视觉服务 → build_pick_summary → 返回 pick_sequence。"""
        if _DRY_RUN or self.hardware is None:
            return self._fallback()

        try:
            result = self.hardware.infer_top_carton()
        except Exception as e:
            logger.warning("视觉服务调用失败: %s, 使用回退序列", e)
            return self._fallback()

        if not result.success:
            logger.warning(
                "%s, 使用回退序列",
                result.message or "视觉推理失败",
            )
            return self._fallback()

        data = result.data or {}
        message = str(data.get("message", "") or "")
        if not message:
            logger.warning("message 为空, 使用回退序列")
            return self._fallback()

        box_type = str(self.params.box_type_override or "").strip()
        box_type_arg = box_type if box_type else None

        try:
            summary = build_pick_summary(message, box_type_arg)
        except Exception as e:
            logger.warning("摘要解析失败: %s, 使用回退序列", e)
            return self._fallback()

        self._summary = summary
        seq = summary["pick_sequence"]
        if not seq:
            logger.warning("pick_sequence 为空, 使用回退序列")
            return self._fallback()

        if HAS_ROSPY:
            rospy.loginfo(
                "[CartonDynamicPickFlow] carton_type=%s, estimated_total=%s/%s, "
                "empty_slots=%s, pick_sequence(len=%d)",
                summary["carton_type"], summary["estimated_total"],
                summary["total_cartons"], summary["empty_slots"], len(seq),
            )
        return seq

    def _fallback(self):
        """视觉失败时返回 fallback_pick_sequence。"""
        raw = str(self.params.fallback_pick_sequence)
        try:
            seq = [int(x.strip()) for x in raw.split(",") if x.strip()]
        except Exception:
            seq = [1, 2, 3, 4, 5, 6]
        self._summary = {
            "carton_type": "", "top_orientation": "",
            "top_ids": list(seq),
            "estimated_total": len(seq), "total_cartons": len(seq),
            "cartons_per_layer": len(seq), "empty_slots": 0,
            "is_partial_stack": False, "pick_sequence": seq,
        }
        if HAS_ROSPY:
            rospy.logwarn("[CartonDynamicPickFlow] 使用回退序列: %s", seq)
        return seq

    def _compute_kept_groups(self):
        """根据 top_ids 计算需要执行的组号(1-indexed)，跳过空位。

        6 组模板对应 6 个物理位置，top_ids 是顶层有箱的局部 ID(1-6)。
        如果某位置在 top_ids 中，则保留该组；否则跳过(空位)。
        视觉失败/回退时 top_ids 覆盖全部，跑全部组。
        """
        max_groups = self.params.max_groups
        top_ids = self._summary.get("top_ids", [])
        if not top_ids:
            self._kept_group_indices = list(range(1, max_groups + 1))
        else:
            self._kept_group_indices = [
                i for i in range(1, max_groups + 1) if i in top_ids
            ]
        skipped = [
            i for i in range(1, max_groups + 1)
            if i not in self._kept_group_indices
        ]
        if HAS_ROSPY:
            rospy.loginfo(
                "[CartonDynamicPickFlow] 执行组号=%s, 跳过空位组号=%s",
                self._kept_group_indices, skipped,
            )

    def _print_and_confirm(self):
        """打印不满垛识别结果 + 等待 Enter。"""
        s = self._summary
        seq_str = ", ".join(str(x) for x in self._pick_sequence) or "(空)"
        max_g = self.params.max_groups
        banner = (
            "\n" + "=" * 72 + "\n"
            "[CartonDynamicPickFlow] 不满垛识别结果\n"
            + "=" * 72 + "\n"
            f"  箱型 carton_type        = {s.get('carton_type', '')}\n"
            f"  顶部朝向 top_orientation = {s.get('top_orientation', '')}\n"
            f"  顶层 ids top_ids         = {s.get('top_ids', [])}\n"
            f"  识别总数 estimated_total = {s.get('estimated_total', 0)}\n"
            f"  应有总数 total_cartons   = {s.get('total_cartons', 0)}\n"
            f"  每层箱数 per_layer       = {s.get('cartons_per_layer', 0)}\n"
            f"  缺箱数 empty_slots       = {s.get('empty_slots', 0)}\n"
            f"  不满垛 is_partial_stack  = {s.get('is_partial_stack', False)}\n"
            f"  执行组号 groups_to_run   = {self._kept_group_indices} "
            f"(共{len(self._kept_group_indices)}组)\n"
            f"  跳过空位 skipped        = "
            f"{[i for i in range(1, max_g + 1) if i not in self._kept_group_indices] or '(无)'}\n"
            f"  抓取序列 pick_sequence   = (len={len(self._pick_sequence)}) [{seq_str}]\n"
            + "=" * 72
        )
        print(banner)
        if HAS_ROSPY:
            rospy.loginfo(banner)

        prompt = self.params.prompt
        if _DRY_RUN and self.params.skip_in_dry_run:
            print(f"[dry-run] {prompt} (已自动跳过)")
        else:
            try:
                input(prompt)
            except (EOFError, KeyboardInterrupt):
                print("[CartonDynamicPickFlow] 输入中断, 继续执行")

    def _build_internal_tree(self) -> Result:
        """加载模板 JSON, 按顶层有箱位置过滤组数, 用工厂构建内部子树。"""
        max_groups = self.params.max_groups
        if not self._kept_group_indices:
            self._done = True
            self._success = True
            return Result.ok("顶层无箱可抓(全部空位), 跳过")

        template_path = str(self.params.template_subtree_path)
        if not os.path.isabs(template_path):
            template_path = os.path.join(os.getcwd(), template_path)

        if not os.path.isfile(template_path):
            msg = f"模板子树不存在: {template_path}"
            if HAS_ROSPY:
                rospy.logerr(f"[CartonDynamicPickFlow] {msg}")
            return Result.fail(msg)

        # --- 加载模板 ---
        with open(template_path, "r", encoding="utf-8") as f:
            template = json.load(f)

        # 模板结构: {"<key>.json": {"tree": {...}}}
        tree_config = list(template.values())[0]["tree"]
        childs = tree_config["childs"]

        # 模板 childs: [backward, wait_backward, group_1..6, wait_finish]
        if len(childs) < 3:
            return Result.fail(f"模板 childs 过少({len(childs)}), 无法过滤")

        # 第 0,1 个是 backward, wait_backward (由主树负责), 最后一个是 wait_finish
        # 中间的是 6 组 nav_point_N_parallel
        # 只保留 top_ids 中有箱的组(跳过空位)
        available_groups = childs[2:2 + max_groups]
        kept_groups = [
            g for i, g in enumerate(available_groups)
            if (i + 1) in self._kept_group_indices
        ]
        filtered_childs = kept_groups + [childs[-1]]
        tree_config["childs"] = filtered_childs

        if HAS_ROSPY:
            rospy.loginfo(
                "[CartonDynamicPickFlow] 构建内部子树: %d 组(组号%s), 模板=%s",
                len(kept_groups), self._kept_group_indices, template_path,
            )

        # --- 用工厂构建子树 ---
        try:
            from orchestration.engine.behavior_tree_factory import (
                BehaviorTreeFactory,
            )
            factory = BehaviorTreeFactory(
                self.blackboard,
                subtree_json_path=template_path,
            )
            root = factory._build_tree_recursive(
                tree_config, parent_namespace=None
            )
            self._tree = py_trees.trees.BehaviourTree(root)
        except Exception as e:
            msg = f"构建内部子树失败: {e}"
            if HAS_ROSPY:
                rospy.logerr(f"[CartonDynamicPickFlow] {msg}")
            return Result.fail(msg)

        logger.info(
            "内部子树构建成功: %d 组(组号%s), pick_sequence(len=%d)",
            len(self._kept_group_indices),
            self._kept_group_indices,
            len(self._pick_sequence),
        )
        return Result.ok()
