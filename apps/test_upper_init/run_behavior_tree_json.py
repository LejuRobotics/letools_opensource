#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
薄启动器：从 JSON 启动 orchestration 行为树。

职责：
- 准备运行环境（workdir / sys.path / ROS init）
- 选择配置路径（主树/子树集合/黑板）
- 创建黑板并写入 board.json
- 创建并启动编排系统（factory/controller），把执行交给 orchestration

注意：不在 apps 层写任何业务动作控制逻辑。
"""

import argparse
import json
import logging
import os
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))


def _resolve_studio_paths(from_file: str):
    """返回 (studio_root, orch_root) ，其中 studio_root 始终为项目根目录。"""
    script_path = os.path.abspath(from_file)
    # 脚本位于 apps/test_upper_init/ 下，向上三级到达项目根
    studio_root = os.path.dirname(os.path.dirname(os.path.dirname(script_path)))
    orch_root = os.path.join(studio_root, "orchestration")
    return studio_root, orch_root


def _ensure_sys_path(*paths: str):
    for p in paths:
        if p and p not in sys.path:
            sys.path.insert(0, p)


def _quiet_shutdown_shared_hardware():
    """主动关闭共享硬件，避免解释器退出阶段由 __del__ 触发日志报错。"""
    try:
        from orchestration.shared_hardware import reset_shared_hardware
        previous_disable_level = logging.root.manager.disable
        logging.disable(logging.CRITICAL)
        try:
            reset_shared_hardware()
        finally:
            logging.disable(previous_disable_level)
    except Exception:
        pass


def _load_board_into_blackboard(blackboard_client, board_path: str):
    from orchestration.utils.blackboard_utils import (
        apply_blackboard_data_from_json,
        apply_flat_board_json,
    )

    if not board_path or not os.path.isfile(board_path):
        print(f"[apps] board.json 不存在，跳过：{board_path}")
        return

    # 兼容两种 board 结构：
    # 1) 扁平 dict（如 studio_smoke_v1/refactored_sdk_atomic_v1）
    # 2) 分组 list（含 process / key/value/type 等）
    try:
        with open(board_path, "r", encoding="utf-8") as f:
            data = json.load(f)
    except Exception as e:
        raise RuntimeError(f"读取 board.json 失败: {board_path}, err={e}")

    if isinstance(data, dict) and "process" not in data:
        apply_flat_board_json(blackboard_client, board_path)
    else:
        apply_blackboard_data_from_json(blackboard_client, board_path, use_group_prefix=False)


def main():
    parser = argparse.ArgumentParser(description="Run orchestration behavior tree from JSON")
    parser.add_argument(
        "--scenario",
        default="",
        help="场景文件夹（若提供，将默认从其中取 py_tree.json/py_tree_child.json/board.json）",
    )
    parser.add_argument("--tree", default="", help="主树 py_tree.json 路径")
    parser.add_argument("--subtrees", default="", help="子树集合 py_tree_child.json 路径（可选）")
    parser.add_argument("--board", default="", help="黑板 board.json 路径（可选）")
    parser.add_argument(
        "--ros-node",
        default="behavior_tree_main",
        help="ROS node name（仅在 ROS 环境生效）",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="不初始化 ROS，仅验证加载/导入（需要 py_trees 依赖可用）",
    )
    parser.add_argument(
        "--tick-once",
        action="store_true",
        help="与 --dry-run 合用：执行一次 tick",
    )
    parser.add_argument(
        "--spin",
        action="store_true",
        help="树跑完后 rospy.spin（仅 ROS 环境）",
    )
    parser.add_argument(
        "--parallel-load",
        action="store_true",
        help="启用并行构树（可能引发 Python import 死锁；默认关闭更稳定）",
    )
    parser.add_argument(
        "--action-groups",
        default="",
        help="只运行指定动作组（逗号分隔，如 '1,3,5'）。不指定则运行全部",
    )
    args = parser.parse_args()

    studio_root, orch_root = _resolve_studio_paths(__file__)
    os.chdir(studio_root)
    _ensure_sys_path(studio_root, orch_root)

    # [CRITICAL] 必须先导入 compat，避免 py_trees 版本差异
    print("[apps] 导入 py_trees ...", flush=True)
    import orchestration.engine.py_trees_compat  # noqa: F401, E402

    # 解析路径（优先显式参数，其次 scenario 目录）
    scenario_dir = os.path.abspath(args.scenario) if args.scenario else ""
    if scenario_dir:
        default_tree = os.path.join(scenario_dir, "py_tree.json")
        default_subtrees = os.path.join(scenario_dir, "py_tree_child.json")
        default_board = os.path.join(scenario_dir, "board.json")
    else:
        default_tree = ""
        default_subtrees = ""
        default_board = ""

    tree_path = os.path.abspath(args.tree) if args.tree else default_tree
    subtrees_path = os.path.abspath(args.subtrees) if args.subtrees else default_subtrees
    board_path = os.path.abspath(args.board) if args.board else default_board

    print("[apps] 启动参数")
    print(f"  - workdir: {os.getcwd()}")
    print(f"  - tree: {tree_path}")
    print(f"  - subtrees: {subtrees_path or '(none)'}")
    print(f"  - board: {board_path or '(none)'}")

    if not tree_path or not os.path.isfile(tree_path):
        raise RuntimeError(f"主树 py_tree.json 不存在: {tree_path}")

    # --- 创建黑板并加载 board.json ---
    from py_trees.blackboard import Client

    blackboard_client = Client(name="main_tree_blackboard", namespace="/")
    if board_path:
        _load_board_into_blackboard(blackboard_client, board_path)

    # --- 解析动作组过滤 ---
    action_group_filter = None
    if args.action_groups.strip():
        try:
            action_group_filter = set(int(g.strip()) for g in args.action_groups.split(","))
            print(f"[apps] 动作组过滤: {sorted(action_group_filter)}")
        except ValueError:
            raise RuntimeError(f"--action-groups 格式错误，请用逗号分隔数字（如 '1,3,5'）: {args.action_groups}")

    # --- 创建并启动编排系统 ---
    from orchestration.engine.behavior_tree_factory import BehaviorTreeFactory
    from orchestration.engine.behavior_tree_controller import BehaviorTreeController

    factory = BehaviorTreeFactory(
        blackboard_client,
        enable_parallel_loading=bool(args.parallel_load),
        subtree_json_path=subtrees_path if (subtrees_path and os.path.isfile(subtrees_path)) else None,
        action_group_filter=action_group_filter,
    )
    if subtrees_path:
        if os.path.isfile(subtrees_path):
            factory.reload_subtree_config()
            print(f"[apps] 子树集合已加载：{subtrees_path} (count={len(factory.subtree_config)})")
        else:
            print(f"[apps] 子树集合文件不存在，忽略：{subtrees_path}")

    controller = BehaviorTreeController(factory)

    # dry-run：只加载/可选 tick 一次
    if args.dry_run:
        tree = controller.load_tree_only(tree_path, blackboard_client)
        if tree is None or not hasattr(tree, "root") or tree.root is None:
            raise RuntimeError("dry-run 加载失败：root 不存在")
        print(f"[apps][dry-run] 已加载树，根节点: {tree.root.name}")
        if args.tick_once:
            tree.tick()
            print(f"[apps][dry-run] tick 后根状态: {tree.root.status}")
        return

    # ROS 模式：初始化节点、预热硬件、运行主循环
    try:
        import rospy
    except Exception as e:
        raise RuntimeError(f"当前环境不可用 rospy（若非 ROS 环境请使用 --dry-run）：{e}")

    rospy.init_node(args.ros_node, log_level=rospy.INFO)
    controller.init_services()

    # 预热硬件：提前触发 HardwareFactory.create + initialize，
    # 避免第一个 tick 在行为树 tick 循环内初始化阻塞 20+ 秒。
    try:
        from orchestration.shared_hardware import get_shared_hardware, set_hardware_config

        # 场景级硬件配置：若 scenario 目录下有 hardware_config.json 则应用
        if scenario_dir:
            hw_cfg_path = os.path.join(scenario_dir, "hardware_config.json")
            if os.path.isfile(hw_cfg_path):
                with open(hw_cfg_path, "r", encoding="utf-8") as f:
                    set_hardware_config(json.load(f))
                print(f"[apps] 已加载场景硬件配置: {hw_cfg_path}")

        _hw = get_shared_hardware()
        print(f"[apps] 硬件预热完成: {type(_hw).__name__}")
    except Exception as e:
        print(f"[apps] 硬件预热失败（将继续，树内首次访问时再初始化）: {e}")

    final_status = controller.start_behavior_tree(tree_path, blackboard_client)

    if args.spin:
        rospy.loginfo("[apps] --spin: 保持节点运行（Ctrl+C 退出）")
        rospy.spin()
        return

    # 退出码：SUCCESS=0, FAILURE=1, 其它=2
    try:
        import py_trees.common

        if final_status == py_trees.common.Status.SUCCESS:
            _quiet_shutdown_shared_hardware()
            logging.disable(logging.CRITICAL)
            sys.exit(0)
        if final_status == py_trees.common.Status.FAILURE:
            _quiet_shutdown_shared_hardware()
            logging.disable(logging.CRITICAL)
            sys.exit(1)
    except Exception:
        pass
    _quiet_shutdown_shared_hardware()
    logging.disable(logging.CRITICAL)
    sys.exit(2)


if __name__ == "__main__":
    main()