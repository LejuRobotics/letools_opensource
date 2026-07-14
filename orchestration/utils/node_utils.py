# -*- coding: utf-8 -*-
"""
pytrees_actions 工具模块
提供 TF 坐标变换、参数解析和节点调试功能
兼容 Python 2/3
"""

from __future__ import print_function
import rospy
import tf
from tf.transformations import euler_from_quaternion
import py_trees
import threading


# ============================================================================
# RobotUtils - TF 坐标变换工具类（单例模式）
# ============================================================================

class RobotUtils(object):
    """
    机器人 TF 坐标变换工具类（单例模式）
    
    功能：
    1. 提供通用的 TF 坐标变换查询
    2. 封装常用的业务坐标变换（base_link <-> world/map, base_link <-> chassis）
    3. 支持自定义 frame 名称配置
    
    使用示例：
        # 获取单例实例
        robot_utils = RobotUtils.get_instance()
        
        # 或者自定义 frame 名称
        robot_utils = RobotUtils.get_instance(
            world_frame="map",
            base_frame="base_link", 
            chassis_frame="odom"
        )
        
        # 获取变换
        transform = robot_utils.get_base_to_world()
        if transform:
            pos, euler = transform
            print("位置:", pos)
            print("姿态(欧拉角):", euler)
    """
    
    _instance = None
    _lock = threading.Lock()
    
    def __init__(self, world_frame="map", base_frame="base_link", chassis_frame="odom"):
        """
        初始化 RobotUtils
        
        参数:
            world_frame: 世界坐标系名称，默认 "map"
            base_frame: 躯干坐标系名称，默认 "base_link"
            chassis_frame: 底盘坐标系名称，默认 "odom"
        """
        if RobotUtils._instance is not None:
            rospy.logwarn("[RobotUtils] 实例已存在，使用 get_instance() 获取单例")
        
        self.world_frame = world_frame
        self.base_frame = base_frame
        self.chassis_frame = chassis_frame
        
        # 初始化 TF Listener
        try:
            self.tf_listener = tf.TransformListener()
            rospy.loginfo("[RobotUtils] TF Listener 初始化成功")
            rospy.loginfo("[RobotUtils] Frame 配置: world={}, base={}, chassis={}".format(
                world_frame, base_frame, chassis_frame))
            # 给 TF buffer 一点时间填充
            rospy.sleep(0.5)
        except Exception as e:
            rospy.logerr("[RobotUtils] TF Listener 初始化失败: {}".format(e))
            self.tf_listener = None
    
    @classmethod
    def get_instance(cls, world_frame="map", base_frame="base_link", chassis_frame="odom"):
        """
        获取 RobotUtils 单例实例
        
        参数:
            world_frame: 世界坐标系名称，默认 "map"
            base_frame: 躯干坐标系名称，默认 "base_link"
            chassis_frame: 底盘坐标系名称，默认 "odom"
        
        返回:
            RobotUtils 单例实例
        """
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = cls(world_frame, base_frame, chassis_frame)
        return cls._instance
    
    def get_transform(self, target_frame, source_frame, timeout=1.0):
        """
        通用的 TF 坐标变换查询函数
        
        参数:
            target_frame: 目标坐标系
            source_frame: 源坐标系
            timeout: 等待超时时间（秒），默认 1.0
        
        返回:
            成功: (position, euler_angles)
                  position: [x, y, z] 位置（米）
                  euler_angles: [roll, pitch, yaw] 欧拉角（弧度）
            失败: None
        """
        if self.tf_listener is None:
            rospy.logerr("[RobotUtils] TF Listener 未初始化")
            return None
        
        try:
            # 等待变换可用
            self.tf_listener.waitForTransform(
                target_frame, source_frame, rospy.Time(0), rospy.Duration(timeout)
            )
            
            # 查询变换
            (trans, rot) = self.tf_listener.lookupTransform(
                target_frame, source_frame, rospy.Time(0)
            )
            
            # 转换四元数到欧拉角
            euler = euler_from_quaternion(rot)
            
            position = list(trans)  # [x, y, z]
            euler_angles = list(euler)  # [roll, pitch, yaw]
            
            return (position, euler_angles)
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("[RobotUtils] TF 查询失败 ({} -> {}): {}".format(
                source_frame, target_frame, e))
            return None
    
    def get_base_to_world(self):
        """
        获取 base_link 在世界坐标系（map）下的位姿
        
        用途：获取机器人在世界坐标系中的位置和朝向
        
        返回:
            成功: (position, euler_angles)
                  position: [x, y, z] 在世界坐标系中的位置
                  euler_angles: [roll, pitch, yaw] 姿态角
            失败: None
        """
        return self.get_transform(self.world_frame, self.base_frame)
    
    def get_world_to_base(self):
        """
        获取世界坐标系（map）原点在 base_link 下的位姿
        
        用途：将世界坐标系中的点转换到机器人本体坐标系
        
        返回:
            成功: (position, euler_angles)
                  position: [x, y, z] 世界原点在 base_link 下的位置
                  euler_angles: [roll, pitch, yaw] 姿态角
            失败: None
        """
        return self.get_transform(self.base_frame, self.world_frame)
    
    def get_base_to_chassis(self):
        """
        获取 base_link（躯干）到底盘坐标系的坐标变换
        
        用途：获取躯干相对于底盘的位置偏移
        注意：文档提到躯干到底盘前端有约 0.15m 的物理偏差，此函数通过 TF 动态查询实际值
        
        返回:
            成功: (position, euler_angles)
                  position: [x, y, z] base_link 在底盘坐标系中的位置（x方向通常有偏差）
                  euler_angles: [roll, pitch, yaw] 姿态角
            失败: None
        """
        return self.get_transform(self.chassis_frame, self.base_frame)


# ============================================================================
# ParamUtils - 参数解析工具类
# ============================================================================

class ParamUtils(object):
    """
    参数解析工具类
    
    功能：
    1. 从黑板读取参数（支持嵌套键，如 "Common.BoxWidth"）
    2. 根据参数配置解析参数值（支持从黑板或自定义值）
    3. 批量解析参数字典
    
    使用示例：
        # 初始化（需要传入黑板客户端）
        blackboard = py_trees.blackboard.Client(name="test")
        param_utils = ParamUtils(blackboard)
        
        # 读取嵌套键
        value = param_utils.get_value_from_blackboard("Common.BoxWidth")
        
        # 解析单个参数
        config = {"source": "BOARD", "key": "Common_BoxWidth"}
        value = param_utils.resolve_param(config)
        
        # 批量解析参数字典
        params = {
            "target_x": {"source": "BOARD", "key": "Common_TargetX"},
            "target_y": {"source": "CUSTOM", "value": "1.5"},
            "speed": 0.5  # 直接的数值也支持
        }
        resolved = param_utils.resolve_all_params(params)
        # 返回: {"target_x": <从黑板读取的值>, "target_y": 1.5, "speed": 0.5}
    """
    
    def __init__(self, blackboard_client):
        """
        初始化 ParamUtils
        
        参数:
            blackboard_client: py_trees.blackboard.Client 实例
        """
        self.blackboard = blackboard_client
    
    def get_value_from_blackboard(self, key):
        """
        从黑板读取值，支持嵌套键（用 "." 分隔）
        
        参数:
            key: 黑板键名，支持嵌套如 "Common.BoxWidth"
                 会被转换为 "Common_BoxWidth" 来读取
        
        返回:
            黑板中存储的值，如果不存在返回 None
        
        示例:
            value = get_value_from_blackboard("Common.BoxWidth")
            # 实际读取 blackboard.Common_BoxWidth
        """
        # 将 "." 转换为 "_" （适配现有的黑板键命名规范）
        actual_key = key.replace(".", "_")
        
        try:
            if hasattr(self.blackboard, actual_key):
                value = getattr(self.blackboard, actual_key)
                rospy.logdebug("[ParamUtils] 从黑板读取: {} = {}".format(actual_key, value))
                return value
            else:
                rospy.logwarn("[ParamUtils] 黑板中不存在键: {}".format(actual_key))
                return None
        except Exception as e:
            rospy.logerr("[ParamUtils] 读取黑板失败 ({}): {}".format(actual_key, e))
            return None
    
    def resolve_param(self, param_config):
        """
        解析单个参数配置
        
        参数:
            param_config: 参数配置，可以是：
                - 字典: {"source": "BOARD", "key": "xxx"} - 从黑板读取
                - 字典: {"source": "CUSTOM", "value": "xxx"} - 使用自定义值
                - 其他类型: 直接返回该值（兼容直接传值的情况）
        
        返回:
            解析后的参数值
        
        示例:
            # 从黑板读取
            config = {"source": "BOARD", "key": "Common_BoxWidth"}
            value = resolve_param(config)  # 返回黑板中的值
            
            # 使用自定义值
            config = {"source": "CUSTOM", "value": "0.5"}
            value = resolve_param(config)  # 返回 "0.5"
            
            # 直接传值
            value = resolve_param(0.5)  # 返回 0.5
        """
        # 如果不是字典，直接返回
        if not isinstance(param_config, dict):
            return param_config
        
        source = param_config.get("source", "CUSTOM")
        
        if source == "BOARD":
            # 从黑板读取
            key = param_config.get("key", "")
            return self.get_value_from_blackboard(key)
        elif source == "CUSTOM":
            # 使用自定义值
            return param_config.get("value")
        else:
            rospy.logwarn("[ParamUtils] 未知的参数源: {}".format(source))
            return param_config.get("value")
    
    def resolve_all_params(self, params_dict):
        """
        解析整个参数字典，返回只包含真实数值的干净字典
        
        参数:
            params_dict: 参数字典，值可以是：
                - {"source": "BOARD", "key": "xxx"}
                - {"source": "CUSTOM", "value": "xxx"}
                - 直接的数值
                - 嵌套字典（递归处理）
        
        返回:
            解析后的参数字典，所有参数都被替换为真实值
        
        示例:
            params = {
                "target_x": {"source": "BOARD", "key": "Common_TargetX"},
                "target_y": {"source": "CUSTOM", "value": "1.5"},
                "speed": 0.5,
                "nested": {
                    "box_width": {"source": "BOARD", "key": "Common_BoxWidth"}
                }
            }
            resolved = resolve_all_params(params)
            # 返回: {
            #     "target_x": 2.0,  # 从黑板读取
            #     "target_y": "1.5",
            #     "speed": 0.5,
            #     "nested": {"box_width": 0.35}  # 从黑板读取
            # }
        """
        if not isinstance(params_dict, dict):
            return params_dict
        
        resolved = {}
        
        for key, value in params_dict.items():
            if isinstance(value, dict):
                # 检查是否是参数配置字典
                if "source" in value:
                    # 是参数配置，解析它
                    resolved[key] = self.resolve_param(value)
                else:
                    # 是嵌套字典，递归处理
                    resolved[key] = self.resolve_all_params(value)
            else:
                # 直接的值
                resolved[key] = value
        
        return resolved


# ============================================================================
# NodeDebugger - 节点调试工具类
# ============================================================================

class NodeDebugger(object):
    """
    节点调试工具类
    
    功能：
    在节点运行时打印节点信息和局部变量，方便调试
    
    使用示例：
        class MyNode(py_trees.behaviour.Behaviour):
            def update(self):
                target_x = 1.5
                target_y = 2.0
                
                # 打印节点状态和局部变量
                NodeDebugger.log_node_state(self, locals())
                
                return Status.SUCCESS
    """
    
    @staticmethod
    def log_node_state(node_instance, local_vars=None):
        """
        打印节点状态和局部变量
        
        参数:
            node_instance: 节点实例（self），应该是 py_trees.behaviour.Behaviour 的子类
            local_vars: locals() 返回的局部变量字典，可选
        
        功能:
            1. 打印节点名称 (node_instance.name)
            2. 打印节点参数 (node_instance.params)，如果存在
            3. 打印所有非 self 的局部变量，方便查看中间计算结果
        
        示例:
            def update(self):
                target_x = 1.5
                target_y = 2.0
                result = target_x + target_y
                
                # 调试：打印节点状态
                NodeDebugger.log_node_state(self, locals())
                
                # 输出:
                # ========== 节点调试信息 ==========
                # 节点名称: MyNode
                # 节点参数: {'mode': 'test', 'timeout': 5}
                # 局部变量:
                #   target_x = 1.5
                #   target_y = 2.0
                #   result = 3.5
                # ===================================
        """
        print("\n" + "=" * 50)
        print("节点调试信息")
        print("=" * 50)
        
        # 打印节点名称
        if hasattr(node_instance, 'name'):
            print("节点名称: {}".format(node_instance.name))
        else:
            print("节点名称: <未知>")
        
        # 打印节点标签（如果有）
        if hasattr(node_instance, 'label'):
            print("节点标签: {}".format(node_instance.label))
        
        # 打印节点参数（如果有）
        if hasattr(node_instance, 'params'):
            print("节点参数: {}".format(node_instance.params))
        
        # 打印局部变量
        if local_vars is not None:
            print("\n局部变量:")
            for var_name, var_value in sorted(local_vars.items()):
                # 跳过 self 和内部变量（以 _ 开头）
                if var_name == 'self' or var_name.startswith('_'):
                    continue
                
                # 格式化输出，限制值的长度
                value_str = str(var_value)
                if len(value_str) > 100:
                    value_str = value_str[:100] + "..."
                
                print("  {} = {}".format(var_name, value_str))
        
        print("=" * 50 + "\n")


# ============================================================================
# 保留原有的 filter_tree_path 函数，保持向后兼容
# ============================================================================

def filter_tree_path(input_path):
    """
    过滤路径，只保留包含 'tree' 的部分
    
    参数:
        input_path: 输入路径字符串
    
    返回:
        过滤后的路径
    """
    if not input_path:
        return ""
    parts = input_path.split('/')

    tree_parts = [part for part in parts if 'tree' in part.lower()]
    return '/'.join(tree_parts)