#!/usr/bin/env python3
"""行为树运行控制器（studio 精简版，阶段 1 主循环 + 可选 ROS 服务）。"""

import json
import os
import threading
import time
import traceback

import py_trees
import py_trees.common

from orchestration.engine.behavior_tree_factory import BehaviorTreeFactory
from orchestration.utils.blackboard_utils import apply_blackboard_data_from_json

try:
    import rospy
    from std_srvs.srv import Empty, EmptyResponse

    HAS_ROSPY = True
except ImportError:
    rospy = None
    HAS_ROSPY = False

try:
    from orchestration.services.blackboard_service import BlackboardService

    HAS_BOARD_SERVICE = True
except Exception:
    HAS_BOARD_SERVICE = False


class BehaviorTreeController:
    def __init__(self, behavior_tree_core: BehaviorTreeFactory):
        self.bt_core = behavior_tree_core
        self.running_flag = False
        self.paused_flag = False
        self.bt_instance = None
        self.bt_thread = None
        self._last_tree_json_path = None
        self._last_blackboard_client = None
        self.blackboard_service = None
        self.max_iterations = 1
        self.current_iteration = 0
        self.last_root_status = None

    def start_behavior_tree(self, tree_json_path, blackboard_client=None):
        self._last_tree_json_path = tree_json_path
        self._last_blackboard_client = blackboard_client

        if blackboard_client and HAS_BOARD_SERVICE and HAS_ROSPY and not self.blackboard_service:
            try:
                self.blackboard_service = BlackboardService(blackboard_client)
            except Exception as e:
                if HAS_ROSPY:
                    rospy.logwarn(f"[BehaviorTree] 黑板服务未启动: {e}")

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
                        _terminal = (
                            py_trees.common.Status.SUCCESS,
                            py_trees.common.Status.FAILURE,
                        )
                        if new_root_status in _terminal:
                            if self.last_root_status not in _terminal:
                                self.current_iteration += 1
                                if HAS_ROSPY:
                                    rospy.loginfo(
                                        "[BehaviorTree] 完成: %s",
                                        new_root_status.name,
                                    )
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
                            self.last_root_status == py_trees.common.Status.RUNNING
                            and new_root_status is not None
                            and new_root_status != py_trees.common.Status.RUNNING
                        ):
                            self.current_iteration += 1
                            if self.max_iterations > 0 and self.current_iteration >= self.max_iterations:
                                self.running_flag = False
                                break
                        self.last_root_status = new_root_status
                if rate is not None:
                    rate.sleep()
                else:
                    time.sleep(1.0 / rate_hz)
        except Exception as e:
            if HAS_ROSPY:
                rospy.logerr(f"[BehaviorTree] 主循环异常: {e}")
            traceback.print_exc()
        finally:
            self.running_flag = False

        return self.last_root_status

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
