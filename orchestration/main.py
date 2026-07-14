#!/usr/bin/env python3
"""
LeTools 行为树主入口（节点名 behavior_tree_main）。
默认加载 orchestration/scenarios/studio_smoke_v1/。
"""

import argparse
import os
import sys

_STUDIO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_ORCH_ROOT = os.path.join(_STUDIO_ROOT, "orchestration")
_DEFAULT_SCENARIO = os.path.join(_ORCH_ROOT, "scenarios", "studio_smoke_v1")

for _p in (_STUDIO_ROOT, _ORCH_ROOT):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import orchestration.engine.py_trees_compat  # noqa: F401,E402 — 须先于 factory

from orchestration.engine.behavior_tree_factory import BehaviorTreeFactory
from orchestration.engine.behavior_tree_controller import BehaviorTreeController
from orchestration.utils.blackboard_utils import (
    apply_blackboard_data_from_json,
    apply_flat_board_json,
)
from py_trees.blackboard import Client


def _default_tree_json():
    return os.path.join(_DEFAULT_SCENARIO, "py_tree.json")


def _default_board_json():
    return os.path.join(_DEFAULT_SCENARIO, "board.json")


def _load_board(blackboard_client, board_path):
    if not os.path.isfile(board_path):
        print(f"[main] 未找到 board: {board_path}")
        return
    with open(board_path, "r", encoding="utf-8") as f:
        import json

        data = json.load(f)
    if isinstance(data, dict) and "ArmJointTrajectories" in data:
        apply_flat_board_json(blackboard_client, board_path)
    else:
        apply_blackboard_data_from_json(
            blackboard_client, board_path, use_group_prefix=False
        )


def dry_run_load(tree_json, board_json, do_tick=False):
    os.environ.setdefault("STUDIO_DRY_RUN", "1")
    blackboard_client = Client(name="main_tree_blackboard", namespace="/")
    _load_board(blackboard_client, board_json)
    factory = BehaviorTreeFactory(blackboard_client)
    controller = BehaviorTreeController(factory)
    tree = controller.load_tree_only(tree_json, blackboard_client)
    if tree is None or not hasattr(tree, "root"):
        raise RuntimeError("行为树加载失败")
    print(f"[dry-run] 已加载树，根节点: {tree.root.name}")
    if do_tick:
        tree.tick()
        print(f"[dry-run] 首次 tick 后根状态: {tree.root.status}")
    return tree


def main():
    parser = argparse.ArgumentParser(description="LeTools 行为树主程序")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="无 ROS/实机：仅验证 import 与加载 studio_smoke_v1",
    )
    parser.add_argument(
        "--tick-once",
        action="store_true",
        help="与 --dry-run 合用：执行一次 tick（WaitForEnter 在无 TTY 时 EOF 即 SUCCESS）",
    )
    parser.add_argument("--tree", default=_default_tree_json(), help="py_tree.json 路径")
    parser.add_argument("--board", default=_default_board_json(), help="board.json 路径")
    parser.add_argument(
        "--spin",
        action="store_true",
        help="树跑完后保持 ROS 节点（rospy.spin），供 stop/pause 服务；默认跑完即退出",
    )
    args = parser.parse_args()

    if not os.path.isfile(args.tree):
        print(f"错误: 找不到行为树 JSON: {args.tree}")
        sys.exit(1)

    if args.dry_run:
        dry_run_load(args.tree, args.board, do_tick=args.tick_once)
        print("[dry-run] 完成：无 import 错误")
        return

    import rospy
    import py_trees.common

    rospy.init_node("behavior_tree_main", log_level=rospy.INFO)
    blackboard_client = Client(name="main_tree_blackboard", namespace="/")
    _load_board(blackboard_client, args.board)
    factory = BehaviorTreeFactory(blackboard_client)
    controller = BehaviorTreeController(factory)
    controller.init_services()
    rospy.loginfo(f"加载行为树: {args.tree}")
    final_status = controller.start_behavior_tree(args.tree, blackboard_client)

    if args.spin:
        rospy.loginfo("树已结束，--spin：保持节点运行（Ctrl+C 退出）")
        rospy.spin()
        return

    from orchestration.shared_hardware import reset_shared_hardware

    reset_shared_hardware()
    if final_status == py_trees.common.Status.SUCCESS:
        rospy.loginfo("studio_smoke 完成，进程退出 (0)")
        sys.exit(0)
    if final_status == py_trees.common.Status.FAILURE:
        rospy.logerr("行为树 FAILURE，进程退出 (1)")
        sys.exit(1)
    rospy.logwarn("行为树未达终态 (%s)，进程退出 (2)", final_status)
    sys.exit(2)


if __name__ == "__main__":
    main()
