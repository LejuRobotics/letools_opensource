#!/usr/bin/env python3
"""行为树运行控制器（studio 精简版，阶段 1 主循环 + 可选 ROS 服务）。"""

import json
import os
import threading
import time

import py_trees
import py_trees.common

from core.common.logger import get_logger
from orchestration.engine.behavior_tree_factory import BehaviorTreeFactory
from orchestration.utils.blackboard_utils import apply_blackboard_data_from_json

try:
    import rospy
    from std_srvs.srv import Empty, EmptyResponse

    HAS_ROSPY = True
except ImportError:
    rospy = None
    HAS_ROSPY = False

# 黑板数据 ROS 服务暂时停用（2026-09-22）：pytrees_actions 是 ROS msg 包，须经
# catkin 编译才能 import，而 embodied 尚未编译，注册必然失败并打出"黑板服务未启动"
# 告警。该服务只把黑板暴露给外部调试工具，行为树本身用的是进程内的
# blackboard_client，停用不影响运行。
# 恢复方式：先 cd embodied && catkin_make 并 source 其 devel，再放开本段、
# __init__ 里的 blackboard_service 字段、以及 start_behavior_tree 里的注册段。
# try:
#     from orchestration.services.blackboard_service import BlackboardService
#
#     HAS_BOARD_SERVICE = True
# except Exception:
#     HAS_BOARD_SERVICE = False


logger = get_logger(__name__)


class BehaviorTreeController:
    def __init__(self, behavior_tree_core: BehaviorTreeFactory):
        self.bt_core = behavior_tree_core
        self.running_flag = False
        self.paused_flag = False
        self.bt_instance = None
        self.bt_thread = None
        self._last_tree_json_path = None
        self._last_blackboard_client = None
        # self.blackboard_service = None  # 黑板 ROS 服务暂时停用，见文件头说明
        self.max_iterations = 1
        self.current_iteration = 0
        self.last_root_status = None

    def start_behavior_tree(self, tree_json_path, blackboard_client=None):
        self._last_tree_json_path = tree_json_path
        self._last_blackboard_client = blackboard_client

        # 黑板数据 ROS 服务暂时停用——pytrees_actions 未编译，注册必然失败。
        # 恢复方式见文件头 BlackboardService import 处的说明。
        # if blackboard_client and HAS_BOARD_SERVICE and HAS_ROSPY and not self.blackboard_service:
        #     try:
        #         self.blackboard_service = BlackboardService(blackboard_client)
        #     except Exception as e:
        #         if HAS_ROSPY:
        #             rospy.logwarn(f"[BehaviorTree] 黑板服务未启动: {e}")

        self.running_flag = True
        self.paused_flag = False
        self.current_iteration = 0
        self.last_root_status = None

        try:
            if not self.bt_instance:
                self.bt_instance = self.bt_core.load_tree_from_json(tree_json_path)
                if not self.bt_instance:
                    if HAS_ROSPY:
                        rospy.logerr("[BehaviorTree] 构建失败")
                    self.running_flag = False
                    return

            rate_hz = 50
            if HAS_ROSPY:
                rate = rospy.Rate(rate_hz)
                rospy.loginfo("[BehaviorTree] 主循环启动 (50Hz)")
            else:
                rate = None

            while (not HAS_ROSPY or not rospy.is_shutdown()) and self.running_flag:
                if self.max_iterations > 0 and self.current_iteration >= self.max_iterations:
                    self.running_flag = False
                    break
                if not self.paused_flag:
                    self.bt_core.tick()
                    if self.bt_instance and hasattr(self.bt_instance, "root") and self.bt_instance.root:
                        new_root_status = self.bt_instance.root.status
                        previous_root_status = self.last_root_status
                        # 先保存本次 tick 的状态，保证终态分支 break 后调用者仍能
                        # 获得 SUCCESS/FAILURE，而不是上一次的 RUNNING。
                        self.last_root_status = new_root_status
                        _terminal = (
                            py_trees.common.Status.SUCCESS,
                            py_trees.common.Status.FAILURE,
                        )
                        if new_root_status in _terminal:
                            if previous_root_status not in _terminal:
                                self.current_iteration += 1
                                if HAS_ROSPY:
                                    rospy.loginfo(
                                        "[BehaviorTree] 完成: %s",
                                        new_root_status.name,
                                    )
                                    if (
                                        new_root_status
                                        == py_trees.common.Status.FAILURE
                                    ):
                                        self._log_failure_details()
                                else:
                                    print(
                                        f"[BehaviorTree] 完成: {new_root_status.name}"
                                    )
                            if (
                                self.max_iterations > 0
                                and self.current_iteration >= self.max_iterations
                            ):
                                self.running_flag = False
                                break
                        elif (
                            previous_root_status == py_trees.common.Status.RUNNING
                            and new_root_status is not None
                            and new_root_status != py_trees.common.Status.RUNNING
                        ):
                            self.current_iteration += 1
                            if self.max_iterations > 0 and self.current_iteration >= self.max_iterations:
                                self.running_flag = False
                                break
                if rate is not None:
                    rate.sleep()
                else:
                    time.sleep(1.0 / rate_hz)
        except Exception as e:
            logger.error("[BehaviorTree] 主循环异常: %s", e, exc_info=True)
            if HAS_ROSPY:
                rospy.logerr(f"[BehaviorTree] 主循环异常: {e}")
        finally:
            self.running_flag = False

        return self.last_root_status

    def _log_failure_details(self):
        """记录真正失败的叶节点，避免根节点 FAILURE 丢失业务原因。"""
        root = getattr(self.bt_instance, "root", None)
        if root is None or not HAS_ROSPY:
            return
        try:
            failed = [
                node
                for node in root.iterate()
                if node.status == py_trees.common.Status.FAILURE
            ]
        except Exception as exc:
            rospy.logerr("[BehaviorTree] 读取失败节点异常: %s", exc)
            return
        for node in failed:
            rospy.logerr(
                "[BehaviorTree] 失败节点: name=%s type=%s feedback=%s",
                getattr(node, "name", "?"),
                type(node).__name__,
                getattr(node, "feedback_message", ""),
            )

    def init_services(self):
        if not HAS_ROSPY:
            return
        rospy.Service("/stop_behavior_tree", Empty, self.stop_behavior_tree)
        rospy.Service("/pause_behavior_tree", Empty, self.pause_behavior_tree)
        rospy.Service("/resume_behavior_tree", Empty, self.resume_behavior_tree)
        rospy.loginfo("[BehaviorTree] 基础控制服务已注册 (stop/pause/resume)")

    def stop_behavior_tree(self, req=None):
        self.running_flag = False
        self.paused_flag = True
        if self.bt_instance and hasattr(self.bt_instance, "root") and self.bt_instance.root:
            try:
                self.bt_instance.root.stop(py_trees.common.Status.INVALID)
            except Exception:
                pass
        return EmptyResponse() if req is not None else None

    def pause_behavior_tree(self, req=None):
        self.paused_flag = True
        return EmptyResponse() if req is not None else None

    def resume_behavior_tree(self, req=None):
        self.paused_flag = False
        return EmptyResponse() if req is not None else None

    def load_tree_only(self, tree_json_path, blackboard_client=None):
        """干跑：仅加载树，不启动主循环。"""
        self._last_tree_json_path = tree_json_path
        self._last_blackboard_client = blackboard_client
        self.bt_instance = self.bt_core.load_tree_from_json(tree_json_path)
        return self.bt_instance
