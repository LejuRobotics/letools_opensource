#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
一键跑通 dismantle_box 系列 6 个场景。

!!!   此脚本未经过长时间测试，并且参数量较大，耦合后可能会出现一些问题，实机运行的时候万分小心   !!!

用法：
  # 跑全部 6 个场景，每个场景全部 6 组动作
  python3 apps/test_upper_init/run_all_dismantle_box.py

  # 只跑场景 1 和 3，每个场景只跑第 1、3 组动作
  python3 apps/test_upper_init/run_all_dismantle_box.py --scenarios 1,3 --action-groups 1,3

  # dry-run 验证（不连 ROS）
  python3 apps/test_upper_init/run_all_dismantle_box.py --dry-run

  # 指定场景子集 + 动作组子集
  python3 apps/test_upper_init/run_all_dismantle_box.py --scenarios 2,4,6 --action-groups 1
"""

import argparse
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))


def _resolve_studio_paths(from_file: str):
    script_path = os.path.abspath(from_file)
    studio_root = os.path.dirname(os.path.dirname(os.path.dirname(script_path)))
    orch_root = os.path.join(studio_root, "orchestration")
    return studio_root, orch_root


def _ensure_sys_path(*paths: str):
    for p in paths:
        if p and p not in sys.path:
            sys.path.insert(0, p)


def _load_board_into_blackboard(blackboard_client, board_path: str):
    from orchestration.utils.blackboard_utils import (
        apply_blackboard_data_from_json,
        apply_flat_board_json,
    )
    import json

    if not board_path or not os.path.isfile(board_path):
        print(f"[apps] board.json 不存在，跳过：{board_path}")
        return

    with open(board_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    if isinstance(data, dict) and "process" not in data:
        apply_flat_board_json(blackboard_client, board_path)
    else:
        apply_blackboard_data_from_json(blackboard_client, board_path, use_group_prefix=False)


def _build_scenario_name(num: int) -> str:
    """数字转场景目录名: 1 -> dismantle_box, 2 -> dismantle_box_2, ..."""
    if num == 1:
        return "dismantle_box"
    return f"dismantle_box_{num}"


def run_single_scenario(
    scenario_dir, blackboard_client, controller, action_group_filter, dry_run=False, tick_once=False
):
    """运行单个场景，返回 (scenario_name, success)。

    Args:
        controller: 复用的 BehaviorTreeController 实例，避免重复注册 ROS 服务。
    """
    scenario_name = os.path.basename(scenario_dir)
    tree_path = os.path.join(scenario_dir, "py_tree.json")
    subtrees_path = os.path.join(scenario_dir, "py_tree_child.json")
    board_path = os.path.join(scenario_dir, "board.json")

    print(f"\n{'='*60}")
    print(f"[run_all] 场景: {scenario_name}")
    print(f"  - tree: {tree_path}")
    print(f"  - subtrees: {subtrees_path}")
    print(f"  - board: {board_path}")
    if action_group_filter:
        print(f"  - action_groups: {sorted(action_group_filter)}")
    print(f"{'='*60}")

    # 每个场景重新加载 board
    _load_board_into_blackboard(blackboard_client, board_path)

    from orchestration.engine.behavior_tree_factory import BehaviorTreeFactory

    # 每轮重建 factory（subtree 配置不同），但复用 controller
    factory = BehaviorTreeFactory(
        blackboard_client,
        enable_parallel_loading=False,
        subtree_json_path=subtrees_path if os.path.isfile(subtrees_path) else None,
        action_group_filter=action_group_filter,
    )
    if os.path.isfile(subtrees_path):
        factory.reload_subtree_config()

    # 复用 controller：更新 factory 并重置树缓存
    controller.bt_core = factory
    controller.bt_instance = None

    if dry_run:
        tree = controller.load_tree_only(tree_path, blackboard_client)
        if tree is None or not hasattr(tree, "root") or tree.root is None:
            print(f"[run_all][FAIL] {scenario_name}: dry-run 加载失败")
            return scenario_name, False
        print(f"[run_all][dry-run] {scenario_name}: 已加载，根节点: {tree.root.name}")
        if tick_once:
            tree.tick()
            print(f"[run_all][dry-run] {scenario_name}: tick 后根状态: {tree.root.status}")
        return scenario_name, True

    # ROS 模式
    import py_trees.common

    final_status = controller.start_behavior_tree(tree_path, blackboard_client)

    success = final_status == py_trees.common.Status.SUCCESS
    status_name = final_status.name if final_status else "None"
    print(f"[run_all] {scenario_name}: 完成，状态: {status_name}")
    return scenario_name, success


def main():
    parser = argparse.ArgumentParser(
        description="一键跑通 dismantle_box 系列 6 个场景"
    )
    parser.add_argument(
        "--scenarios",
        default="1,2,3,4,5,6",
        help="要运行的场景编号（逗号分隔），默认 '1,2,3,4,5,6'",
    )
    parser.add_argument(
        "--action-groups",
        default="",
        help="每个场景只运行指定动作组（逗号分隔，如 '1,3,5'）。不指定则运行全部",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="不初始化 ROS，仅验证加载/导入",
    )
    parser.add_argument(
        "--tick-once",
        action="store_true",
        help="与 --dry-run 合用：执行一次 tick",
    )
    parser.add_argument(
        "--ros-node",
        default="run_all_dismantle_box",
        help="ROS node name",
    )
    args = parser.parse_args()

    studio_root, orch_root = _resolve_studio_paths(__file__)
    os.chdir(studio_root)
    _ensure_sys_path(studio_root, orch_root)

    import orchestration.engine.py_trees_compat  # noqa: F401

    # 解析场景编号
    try:
        scenario_nums = [int(s.strip()) for s in args.scenarios.split(",")]
    except ValueError:
        raise RuntimeError(f"--scenarios 格式错误: {args.scenarios}")

    # 解析动作组过滤
    action_group_filter = None
    if args.action_groups.strip():
        try:
            action_group_filter = set(int(g.strip()) for g in args.action_groups.split(","))
            print(f"[run_all] 动作组过滤: {sorted(action_group_filter)}")
        except ValueError:
            raise RuntimeError(f"--action-groups 格式错误: {args.action_groups}")

    # 检查场景目录是否存在
    scenarios = []
    for num in scenario_nums:
        scenario_name = _build_scenario_name(num)
        scenario_dir = os.path.join(studio_root, "orchestration", "scenarios", scenario_name)
        if not os.path.isdir(scenario_dir):
            print(f"[run_all] 跳过不存在的场景: {scenario_name}")
            continue
        scenarios.append((num, scenario_name, scenario_dir))

    if not scenarios:
        raise RuntimeError("没有有效的场景可运行")

    print(f"[run_all] 待运行场景: {[s[1] for s in scenarios]}")
    print(f"[run_all] 模式: {'dry-run' if args.dry_run else 'ROS'}")

    # --- 创建黑板 ---
    from py_trees.blackboard import Client

    blackboard_client = Client(name="main_tree_blackboard", namespace="/")

    # --- 创建 controller（只创建一次，避免重复注册 ROS 服务） ---
    from orchestration.engine.behavior_tree_factory import BehaviorTreeFactory
    from orchestration.engine.behavior_tree_controller import BehaviorTreeController

    # 用第一个场景的 factory 初始化 controller，后续场景复用
    first_factory = BehaviorTreeFactory(
        blackboard_client,
        enable_parallel_loading=False,
    )
    controller = BehaviorTreeController(first_factory)

    # --- ROS 模式初始化 ---
    if not args.dry_run:
        try:
            import rospy
        except Exception as e:
            raise RuntimeError(f"当前环境不可用 rospy（若非 ROS 环境请使用 --dry-run）：{e}")
        rospy.init_node(args.ros_node, log_level=rospy.INFO)
        controller.init_services()

    # --- 依次运行每个场景 ---
    results = []
    for num, scenario_name, scenario_dir in scenarios:
        try:
            name, success = run_single_scenario(
                scenario_dir,
                blackboard_client,
                controller,
                action_group_filter,
                dry_run=args.dry_run,
                tick_once=args.tick_once,
            )
            results.append((name, success, None))
        except Exception as e:
            print(f"[run_all][ERROR] {scenario_name}: {e}")
            import traceback
            traceback.print_exc()
            results.append((scenario_name, False, str(e)))

    # --- 汇总 ---
    print(f"\n{'='*60}")
    print("[run_all] 执行汇总")
    print(f"{'='*60}")
    all_ok = True
    for name, success, err in results:
        status = "OK" if success else "FAIL"
        line = f"  {name:20s} [{status}]"
        if err:
            line += f"  {err}"
        print(line)
        if not success:
            all_ok = False

    print(f"\n[run_all] {'全部成功' if all_ok else '存在失败'}")
    sys.exit(0 if all_ok else 1)


if __name__ == "__main__":
    main()
