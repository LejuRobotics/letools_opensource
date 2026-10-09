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

from core.common.logger import init_logging


logger = logging.getLogger(__name__)


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
        logger.warning("[apps] board.json 不存在，跳过：%s", board_path)
        return

    # 兼容两种 board 结构：
    # 1) 扁平 dict（如 studio_smoke_v1/refactored_sdk_atomic_v1）—— 顶层键直接进黑板
    # 2) 分组 list（顶层是「组名 → 带 key/remark/type/value 的条目数组」）
    try:
        with open(board_path, "r", encoding="utf-8") as f:
            data = json.load(f)
    except Exception as e:
        raise RuntimeError(f"读取 board.json 失败: {board_path}, err={e}")

    # 分组格式的两个特征，**满足其一**就是分组格式：
    #   1) 顶层有 `process` 键（历史约定，现有分组 board 都有）
    #   2) 任一顶层值是「list 且首元素是带 key 的 dict」—— 这才是格式本身的样子
    #
    # 为什么不能只看 `process`：分组格式的文件**忘了写 process** 时，
    # `apply_flat_board_json` 会把每一组都跳过（它显式 skip 这种形状），于是黑板
    # 上一个键都没有、**且不报错** —— 现场表现为所有 READ_BOARD 全部回退到代码
    # 默认值。判据要认格式本身，不能认一个可选的历史记号。
    #
    # 写成「或」而不是替换：`process` 仍是一个有效信号（比如某份分组 board 的组
    # 全是空 list 时形状判不出来，`process` 兜住）。多一条形状判据只会让**更多**
    # 文件走分组路，原本走分组路的不会掉回扁平路。
    looks_grouped = False
    if isinstance(data, dict):
        looks_grouped = any(
            isinstance(v, list) and v and isinstance(v[0], dict) and "key" in v[0]
            for v in data.values())

    if isinstance(data, dict) and "process" not in data and not looks_grouped:
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
        "--hardware-config",
        default="",
        help=(
            "硬件配置覆盖 JSON（可选）；优先于 scenario/hardware_config.json，"
            "适合仅启用相机的安全测试"
        ),
    )
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

    # 节点统一通过该进程级标志判断是否允许访问 ROS/真实硬件。必须在构树和
    # 动态导入节点之前设置，否则节点构造阶段可能误触发共享硬件初始化。
    if args.dry_run:
        os.environ["STUDIO_DRY_RUN"] = "1"

    studio_root, orch_root = _resolve_studio_paths(__file__)
    os.chdir(studio_root)
    _ensure_sys_path(studio_root, orch_root)
    init_logging()

    # 行为树节点会动态导入 kuavo_msgs 等 catkin 生成包。
    # 在正式入口统一补齐路径，避免要求每个节点或 Driver 各自修改 sys.path。
    if not args.dry_run:
        from core.common.ros_environment import ensure_local_ros_python_path

        ensure_local_ros_python_path(Path(studio_root))

    # [CRITICAL] 必须先导入 compat，避免 py_trees 版本差异
    print("[apps] 导入 py_trees ...", flush=True)
    logger.info("[apps] 导入 py_trees ...")
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
    hardware_config_path = (
        os.path.abspath(args.hardware_config)
        if args.hardware_config
        else (
            os.path.join(scenario_dir, "hardware_config.json")
            if scenario_dir
            else ""
        )
    )

    print("[apps] 启动参数")
    print(f"  - workdir: {os.getcwd()}")
    print(f"  - tree: {tree_path}")
    print(f"  - subtrees: {subtrees_path or '(none)'}")
    print(f"  - board: {board_path or '(none)'}")
    print(f"  - hardware config: {hardware_config_path or '(default)'}")
    logger.info(
        "[apps] 启动参数\n"
        "  - workdir: %s\n"
        "  - tree: %s\n"
        "  - subtrees: %s\n"
        "  - board: %s\n"
        "  - hardware config: %s",
        os.getcwd(),
        tree_path,
        subtrees_path or "(none)",
        board_path or "(none)",
        hardware_config_path or "(default)",
    )

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
            logger.info("[apps] 动作组过滤: %s", sorted(action_group_filter))
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
            logger.info(
                "[apps] 子树集合已加载：%s (count=%d)",
                subtrees_path,
                len(factory.subtree_config),
            )
        else:
            print(f"[apps] 子树集合文件不存在，忽略：{subtrees_path}")
            logger.warning("[apps] 子树集合文件不存在，忽略：%s", subtrees_path)

    controller = BehaviorTreeController(factory)

    # dry-run：只加载/可选 tick 一次
    if args.dry_run:
        tree = controller.load_tree_only(tree_path, blackboard_client)
        if tree is None or not hasattr(tree, "root") or tree.root is None:
            raise RuntimeError("dry-run 加载失败：root 不存在")
        print(f"[apps][dry-run] 已加载树，根节点: {tree.root.name}")
        logger.info("[apps][dry-run] 已加载树，根节点: %s", tree.root.name)
        if args.tick_once:
            tree.tick()
            print(f"[apps][dry-run] tick 后根状态: {tree.root.status}")
            logger.info("[apps][dry-run] tick 后根状态: %s", tree.root.status)
        return

    # ROS 模式：初始化节点、预热硬件、运行主循环
    try:
        import rospy
    except Exception as e:
        raise RuntimeError(f"当前环境不可用 rospy（若非 ROS 环境请使用 --dry-run）：{e}")

    rospy.init_node(args.ros_node, log_level=rospy.INFO)

    # 【重要】rospy.init_node() 会重装 root 上的 handler，清除我们的日志配置，
    # 需立即重新收敛，否则此后到硬件初始化之间的日志不会落入 LeTools 日志文件。
    init_logging(force=True)

    controller.init_services()

    # 预热硬件：提前触发 HardwareFactory.create + initialize，
    # 避免第一个 tick 在行为树 tick 循环内初始化阻塞 20+ 秒。
    try:
        from orchestration.shared_hardware import get_shared_hardware, set_hardware_config

        # 显式硬件配置优先；否则回退到场景目录的 hardware_config.json。
        if hardware_config_path:
            if os.path.isfile(hardware_config_path):
                with open(hardware_config_path, "r", encoding="utf-8") as f:
                    set_hardware_config(json.load(f))
                print(f"[apps] 已加载硬件配置: {hardware_config_path}")
                logger.info("[apps] 已加载硬件配置: %s", hardware_config_path)
            elif args.hardware_config:
                raise RuntimeError(
                    f"显式指定的硬件配置不存在: {hardware_config_path}"
                )

        _hw = get_shared_hardware()
        print(f"[apps] 硬件预热完成: {type(_hw).__name__}")
        logger.info("[apps] 硬件预热完成: %s", type(_hw).__name__)
    except Exception as e:
        # 初始化失败后继续运行只会在首个硬件节点中重复初始化，并掩盖真正原因。
        print(f"[apps] 硬件预热失败: {e}")
        logger.exception("[apps] 硬件预热失败")
        _quiet_shutdown_shared_hardware()
        raise SystemExit(1)

    final_status = controller.start_behavior_tree(tree_path, blackboard_client)

    if final_status is not None and getattr(final_status, "name", "") == "FAILURE":
        root = (
            controller.bt_instance.root
            if controller.bt_instance is not None
            else None
        )
        feedback = getattr(root, "feedback_message", "") if root else ""
        if feedback:
            print(f"[apps] 行为树失败原因: {feedback}")
            logger.error("[apps] 行为树失败原因: %s", feedback)

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
