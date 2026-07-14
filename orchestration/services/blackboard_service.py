#!/usr/bin/env python3
"""黑板数据 ROS 服务（依赖 embodied 编译的 pytrees_actions srv）。"""

import json
import threading

try:
    import rospy
except ImportError:
    rospy = None

try:
    from pytrees_actions.srv import (
        GetBlackboardVariable,
        GetBlackboardVariableResponse,
        SetBlackboardVariable,
        SetBlackboardVariableResponse,
        GetAllBlackboardVariables,
        GetAllBlackboardVariablesResponse,
    )
    HAS_BOARD_SRV = True
except ImportError:
    HAS_BOARD_SRV = False


class BlackboardService:
    """线程安全的黑板读写 ROS 服务。"""

    def __init__(self, blackboard_client):
        if not HAS_BOARD_SRV or rospy is None:
            raise RuntimeError(
                "BlackboardService 需要 rospy 与 pytrees_actions.srv；"
                "请先 source embodied devel 或使用 --dry-run 跳过服务注册。"
            )
        self.blackboard = blackboard_client
        self.lock = threading.Lock()

        self.get_service = rospy.Service(
            "/kuavo_tree/get_blackboard_var",
            GetBlackboardVariable,
            self.handle_get_blackboard_variable,
        )
        self.set_service = rospy.Service(
            "/kuavo_tree/set_blackboard_var",
            SetBlackboardVariable,
            self.handle_set_blackboard_variable,
        )
        self.get_all_service = rospy.Service(
            "/kuavo_tree/get_all_blackboard_vars",
            GetAllBlackboardVariables,
            self.handle_get_all_blackboard_variables,
        )
        rospy.loginfo("黑板数据服务已启动")

    def handle_get_blackboard_variable(self, req):
        response = GetBlackboardVariableResponse()
        try:
            with self.lock:
                if hasattr(self.blackboard, req.key):
                    value = getattr(self.blackboard, req.key)
                    from types import SimpleNamespace

                    if isinstance(value, SimpleNamespace):
                        value = vars(value)
                    response.value = json.dumps(value, ensure_ascii=False)
                    response.success = True
                    response.message = f"成功获取变量: {req.key}"
                else:
                    response.success = False
                    response.value = ""
                    response.message = f"黑板中不存在键: {req.key}"
        except Exception as e:
            response.success = False
            response.value = ""
            response.message = f"获取变量时出错: {str(e)}"
        return response

    def handle_set_blackboard_variable(self, req):
        response = SetBlackboardVariableResponse()
        try:
            with self.lock:
                value = json.loads(req.value)
                if not hasattr(self.blackboard, req.key):
                    try:
                        import py_trees

                        self.blackboard.register_key(
                            key=req.key, access=py_trees.common.Access.WRITE
                        )
                    except Exception:
                        pass
                setattr(self.blackboard, req.key, value)
                response.success = True
                response.message = f"成功设置变量: {req.key}"
        except json.JSONDecodeError as e:
            response.success = False
            response.message = f"JSON 解析错误: {str(e)}"
        except Exception as e:
            response.success = False
            response.message = f"设置变量时出错: {str(e)}"
        return response

    def handle_get_all_blackboard_variables(self, req):
        response = GetAllBlackboardVariablesResponse()
        try:
            with self.lock:
                import py_trees

                bb = py_trees.blackboard.Blackboard()
                all_vars = {}
                for attr in dir(bb):
                    if not attr.startswith("_"):
                        try:
                            value = getattr(bb, attr)
                            if not callable(value):
                                all_vars[attr] = value
                        except Exception:
                            pass
                serializable_vars = {}
                for k, v in all_vars.items():
                    try:
                        json.dumps(v)
                        serializable_vars[k] = v
                    except (TypeError, OverflowError):
                        serializable_vars[k] = str(v)
                response.data = json.dumps(serializable_vars, ensure_ascii=False)
                response.success = True
                response.message = f"成功获取 {len(serializable_vars)} 个黑板变量"
        except Exception as e:
            response.success = False
            response.data = "{}"
            response.message = f"获取所有变量时出错: {str(e)}"
        return response

    def shutdown(self):
        if rospy is not None:
            rospy.loginfo("正在关闭黑板数据服务...")
        self.get_service.shutdown()
        self.set_service.shutdown()
        self.get_all_service.shutdown()
