import copy
import json
import os
import sys
import threading
import py_trees
import importlib
from py_trees.blackboard import Blackboard
from py_trees.common import ParallelPolicy
from typing import Dict, Any, Optional, List, Tuple
from concurrent.futures import ThreadPoolExecutor, as_completed

# 可选导入 rospy (在非 ROS 环境下也能工作)
try:
    import rospy
    import rospkg
    HAS_ROSPY = True
except ImportError:
    HAS_ROSPY = False

# === V3.0: 导入 Manifest 工具（可选，用于节点元数据）===
try:
    from orchestration.utils.manifest_decorators import get_manifest
    HAS_MANIFEST = True
except ImportError:
    try:
        from utils.manifest_decorators import get_manifest
        HAS_MANIFEST = True
    except ImportError:
        HAS_MANIFEST = False

try:
    from orchestration.utils.blackboard_utils import apply_board_params_to_blackboard
except ImportError:
    try:
        from utils.blackboard_utils import apply_board_params_to_blackboard
    except ImportError:
        apply_board_params_to_blackboard = None


def _snake_to_pascal(name: str) -> str:
    """wait_for_enter -> WaitForEnter"""
    return "".join(part.capitalize() for part in name.split("_"))


class ParamsWrapper:
    """参数包装类，支持通过点分隔的路径获取嵌套值"""
    def __init__(self, params: Dict[str, Any]):
        self.params = self._flatten_params(params)

    def _flatten_params(self, params: Dict[str, Any], parent_key: str = '', sep: str = '.') -> Dict[str, Any]:
        """将嵌套的参数展平为扁平字典"""
        items = []
        for k, v in params.items():
            new_key = f"{parent_key}{sep}{k}" if parent_key else k
            if isinstance(v, dict):
                items.extend(self._flatten_params(v, new_key, sep=sep).items())
            else:
                items.append((new_key, v))
        return dict(items)

    def get(self, key: str, default: Any = None) -> Any:
        """通过点分隔的路径获取值"""
        return self.params.get(key, default)

    def __getitem__(self, key: str) -> Any:
        return self.params[key]

    def __contains__(self, key: str) -> bool:
        return key in self.params


def _get_child_configs(node_config: Dict[str, Any]) -> List[Dict[str, Any]]:
    """
    读取子节点列表。兼容部分前端/导出使用 children 而非 childs；
    过滤非 dict 项（null、占位）。
    """
    childs = node_config.get("childs")
    children = node_config.get("children")
    if isinstance(childs, list) and len(childs) > 0:
        raw = childs
    elif isinstance(children, list) and len(children) > 0:
        raw = children
    elif isinstance(childs, list):
        raw = childs
    elif isinstance(children, list):
        raw = children
    else:
        raw = []
    return [c for c in raw if isinstance(c, dict)]


def _resolve_node_class_name(node_config: Dict[str, Any]) -> str:
    """
    从 JSON 解析 Python 节点类名。缺省 UnnamedNode 会误导入 node.UnnamedNode。
    支持常见别名：nodeType、className、action 等。
    注意：JSON 中 "name": null 在 Python 中为 None，需跳过，不能当成字符串 "None"。
    """
    _META_NODE_TYPES = frozenset(
        ("action", "control_flow", "decorator", "subtree", "Action", "Control", "Decorator")
    )
    for key in ("name", "nodeType", "className", "class", "action", "node_type"):
        if key not in node_config:
            continue
        val = node_config.get(key)
        if val is None:
            continue
        s = str(val).strip()
        if not s or s.lower() == "null":
            continue
        if key == "node_type" and s in _META_NODE_TYPES:
            continue
        return s
    return ""


class BehaviorTreeFactory:
    """行为树工厂类，支持子树构建、参数解析、namespace 管理和按末端执行器分类加载"""

    # 支持的末端执行器类型和特殊目录
    SUPPORTED_END_EFFECTORS = ['splint_gripper', 'two_finger_gripper', 'four_finger_gripper', 'common']

    def __init__(
        self,
        blackboard_client,
        tree_dir: str = None,
        enable_parallel_loading: bool = True,
        subtree_json_path: str = None,
        action_group_filter: Optional[set] = None,
    ):
        # === LeTools orchestration 根目录 ===
        _engine_dir = os.path.dirname(os.path.abspath(__file__))
        self.package_path = os.path.dirname(_engine_dir)
        _studio_root = os.path.dirname(self.package_path)
        for _p in (_studio_root, self.package_path):
            if _p not in sys.path:
                sys.path.insert(0, _p)

        # === 配置目录路径（兼容 embodied 布局）===
        self.trees_root = os.path.join(self.package_path, "config", "trees")
        if not os.path.isdir(self.trees_root):
            self.trees_root = os.path.join(self.package_path, "scenarios")

        self.tree_dir = tree_dir or self.package_path
        self.node_dir = os.path.join(self.package_path, "nodes")
        
        self.global_blackboard = blackboard_client
        self.tree = None
        # 缓存已导入的模块，避免重复导入
        self._module_cache = {}
        # 并行加载配置
        self.enable_parallel_loading = enable_parallel_loading
        self._executor = None
        self._node_index_lock = threading.Lock()  # 防止并行构建时索引竞争
        # 动作组过滤器：指定只保留哪些动作组（如 {1, 3, 5}），None 表示不过滤
        self.action_group_filter = action_group_filter
        
        # [UPDATED] 子树配置路径：
        # - 若显式传入 subtree_json_path：使用它（用于场景级配置）
        # - 否则：优先包根（部署推送），回退到 config/trees/common/
        self.subtree_json = (
            os.path.abspath(subtree_json_path)
            if subtree_json_path
            else os.path.join(self.package_path, "py_tree_child.json")
        )
        _fallback_subtree = os.path.join(
            self.package_path, "config", "trees", "common", "py_tree_child.json"
        )
        self._node_library_path = os.path.join(self.package_path, 'config', 'definitions', 'node_library.json')
        self._node_library = None  # 延迟加载
        
        if HAS_ROSPY:
            rospy.loginfo(f"[TreeFactory] 行为树库路径: {self.trees_root}")
            rospy.loginfo(f"[TreeFactory] 子树配置路径: {self.subtree_json}")

        # 加载子树配置：显式路径/包根优先，否则使用 config 中的默认配置
        if os.path.exists(self.subtree_json):
            with open(self.subtree_json) as f:
                self.subtree_config = json.load(f)
        elif os.path.exists(_fallback_subtree):
            self.subtree_json = _fallback_subtree
            with open(self.subtree_json) as f:
                self.subtree_config = json.load(f)
            if HAS_ROSPY:
                rospy.loginfo(f"[TreeFactory] 使用默认子树配置: {self.subtree_json}")
        else:
            if HAS_ROSPY:
                rospy.logwarn(f"[TreeFactory] 子树配置文件不存在: {self.subtree_json}")
            self.subtree_config = {}

    def reload_subtree_config(self):
        """
        重新从磁盘加载子树配置（subtree_config）。

        服务模式下，前端可能在服务启动后推送新的 py_tree_child.json，
        而 __init__ 只在启动时加载一次，导致新推送的子树定义不在内存中。
        在每次接收到新任务时调用此方法，可将 subtree_config 刷新为最新版本。
        """
        if os.path.exists(self.subtree_json):
            with open(self.subtree_json) as f:
                self.subtree_config = json.load(f)
            if HAS_ROSPY:
                rospy.loginfo(f"[TreeFactory] 子树配置已重新加载: {self.subtree_json} ({len(self.subtree_config)} 个子树)")
        else:
            if HAS_ROSPY:
                rospy.logwarn(f"[TreeFactory] 子树配置文件不存在，无法重载: {self.subtree_json}")

    # ==================== 按末端执行器加载方法 (Sprint 3) ====================
    
    def load_tree_by_skill(self, end_effector: str, skill_name: str, override_params: Dict[str, Any] = None) -> py_trees.trees.BehaviourTree:
        """
        根据末端类型和技能名加载行为树
        
        Args:
            end_effector: 末端类型 (splint_gripper, two_finger_gripper, four_finger_gripper)
            skill_name: 技能名称 (不带 .json 后缀, e.g., 'grasp_box')
            override_params: 可选的覆盖参数字典，用于持久化参数注入
            
        Returns:
            py_trees.trees.BehaviourTree: 构建好的行为树实例，失败返回 None
        """
        # 校验末端类型
        if end_effector not in self.SUPPORTED_END_EFFECTORS:
            msg = f"[TreeFactory] 不支持的末端类型: {end_effector}. 支持: {self.SUPPORTED_END_EFFECTORS}"
            if HAS_ROSPY:
                rospy.logerr(msg)
            else:
                print(msg)
            return None
        
        # 自动补全 .json 后缀
        filename = skill_name if skill_name.endswith('.json') else f"{skill_name}.json"
        
        # 拼接完整路径
        tree_path = os.path.join(self.trees_root, end_effector, filename)
        
        # 校验文件是否存在
        if not os.path.exists(tree_path):
            if HAS_ROSPY:
                rospy.logerr(f"[TreeFactory] 错误: 找不到对应的技能文件！")
                rospy.logerr(f"  - 末端类型: {end_effector}")
                rospy.logerr(f"  - 技能名称: {skill_name}")
                rospy.logerr(f"  - 搜索路径: {tree_path}")
            return None
            
        if HAS_ROSPY:
            rospy.loginfo(f"[TreeFactory] 正在加载技能: [{end_effector}] -> {skill_name}")
            
        return self.load_tree_from_json(tree_path, override_params)

    def get_skill_path(self, end_effector: str, skill_name: str) -> str:
        """
        获取技能文件的完整路径
        
        Args:
            end_effector: 末端类型
            skill_name: 技能名称
            
        Returns:
            str: 完整文件路径，如果不存在返回 None
        """
        filename = skill_name if skill_name.endswith('.json') else f"{skill_name}.json"
        tree_path = os.path.join(self.trees_root, end_effector, filename)
        
        if not os.path.exists(tree_path):
            return None
        return tree_path

    def list_available_skills(self, end_effector: str = None) -> Dict[str, List[str]]:
        """
        列出可用的技能
        
        Args:
            end_effector: 可选，指定末端类型。如果为 None，列出所有末端的技能
            
        Returns:
            Dict[str, List[str]]: 按末端类型分组的技能名称列表
        """
        result = {}
        effectors = [end_effector] if end_effector else self.SUPPORTED_END_EFFECTORS
        
        for eff in effectors:
            eff_path = os.path.join(self.trees_root, eff)
            skills = []
            
            if os.path.exists(eff_path):
                # 1. 扫描根目录的 JSON 文件
                for f in os.listdir(eff_path):
                    if f.endswith('.json'):
                        skills.append(f[:-5])
                
                # 2. 扫描 skills 子目录（如果存在）
                skills_dir = os.path.join(eff_path, 'skills')
                if os.path.isdir(skills_dir):
                    for f in os.listdir(skills_dir):
                        if f.endswith('.json'):
                            skills.append(f'skills/{f[:-5]}')
                
                # 3. 对于 common 目录，扫描 utils 子目录
                if eff == 'common':
                    utils_dir = os.path.join(eff_path, 'utils')
                    if os.path.isdir(utils_dir):
                        for f in os.listdir(utils_dir):
                            if f.endswith('.json'):
                                skills.append(f'utils/{f[:-5]}')
            
            result[eff] = sorted(skills)
                
        return result

    def validate_skill(self, end_effector: str, skill_name: str) -> bool:
        """
        验证技能是否适用于当前末端
        
        Args:
            end_effector: 末端类型
            skill_name: 技能名称
            
        Returns:
            bool: True 如果技能适用，否则抛出 ValueError
            
        Raises:
            ValueError: 如果技能不适用于指定末端
        """
        # 检查末端类型是否支持
        if end_effector not in self.SUPPORTED_END_EFFECTORS:
            raise ValueError(
                f"不支持的末端类型: {end_effector}. "
                f"支持的末端: {', '.join(self.SUPPORTED_END_EFFECTORS)}"
            )
        
        # 获取该末端的可用技能列表
        available = self.list_available_skills(end_effector)
        
        # 检查技能是否在可用列表中
        if skill_name not in available[end_effector]:
            raise ValueError(
                f"技能 '{skill_name}' 不适用于末端 '{end_effector}'. "
                f"可用技能: {', '.join(available[end_effector])}"
            )
        
        return True

    # ==================================================================


    def load_tree_from_json(self, json_file: str, override_params: Dict[str, Any] = None) -> py_trees.trees.BehaviourTree:
        """
        从 JSON 文件加载行为树
        
        [V3.0 Lite 增强]:
        - 支持 interface 字段：定义技能的输入输出契约
        - 枚举校验：如果定义了 options，传入值必须在列表中
        - 宏替换：树中的 ${var} 语法在加载时解析替换
        - 默认值注入：如果参数有 default 且黑板中不存在，自动写入
        
        Args:
            json_file: JSON 文件路径
            override_params: 可选的覆盖参数字典，用于持久化参数注入
            
        Returns:
            py_trees.trees.BehaviourTree: 构建的行为树
            
        Raises:
            ValueError: 如果 required 参数缺失，或枚举校验失败
        """
        with open(json_file) as f:
            tree_config = json.load(f)
        
        if "tree" not in tree_config:
            raise ValueError("Invalid JSON structure: missing 'tree' key")
        
        # === V3.0 Lite: 处理 interface 字段 + 宏替换 ===
        interface = tree_config.get("interface", {})
        if interface:
            macro_map = self._validate_and_apply_interface(interface, override_params, json_file)
            if macro_map:
                self._apply_macro_substitution(tree_config["tree"], macro_map)

        root_node = self._build_tree_recursive(tree_config["tree"], parent_namespace=None, override_params=override_params)
        self.tree = py_trees.trees.BehaviourTree(root=root_node)
        return self.tree
    
    # ==================== V3.0 Lite: Interface 校验 + 宏替换 ====================
    
    def _validate_and_apply_interface(self, interface: Dict[str, Any], override_params: Optional[Dict[str, Any]], source_file: str) -> Dict[str, Any]:
        """
        V3.0 Lite: 校验 interface 定义，解析参数值，返回宏替换映射表
        
        新格式字段:
        - internal_name: 树内宏变量名 (${internal_name})
        - external_key: 外部黑板键名 (从黑板读取的 key)
        - options: 可选，枚举值列表 (传入值必须在其中)
        - data_type / type: 数据类型 (string/int/float/bool)
        
        同时兼容旧格式 ("name" 字段) 以保持向后兼容。
        
        Args:
            interface: JSON 中的 interface 定义
            override_params: 覆盖参数
            source_file: 源文件路径（用于错误消息）
            
        Returns:
            Dict[str, Any]: 宏替换映射 {internal_name: resolved_value}
            
        Raises:
            ValueError: 如果 required 参数缺失，或枚举校验失败
        """
        inputs = interface.get("inputs", [])
        outputs = interface.get("outputs", [])
        skill_name = os.path.basename(source_file)
        
        macro_map = {}            # {internal_name: resolved_value}
        missing_required = []     # 缺失的必填参数
        enum_errors = []          # 枚举校验失败的参数
        applied_defaults = []     # 使用默认值的参数
        
        for param_def in inputs:
            # === 兼容新旧格式 ===
            internal_name = param_def.get("internal_name") or param_def.get("name")
            external_key = param_def.get("external_key") or internal_name
            param_type = param_def.get("type", "string")
            param_required = param_def.get("required", False)
            param_default = param_def.get("default")       # None = 无默认值
            param_options = param_def.get("options")        # None = 非枚举
            
            if not internal_name:
                continue
            
            # --- 1. 解析值（优先级：override_params > 黑板 > default）---
            resolved_value = None
            value_source = None
            
            # 优先级 1: override_params
            if override_params and (internal_name in override_params or external_key in override_params):
                resolved_value = override_params.get(internal_name, override_params.get(external_key))
                value_source = "override_params"
            
            # 优先级 2: 全局黑板 (按 external_key 查找)
            if resolved_value is None and self.global_blackboard:
                try:
                    self.global_blackboard.register_key(key=external_key, access=py_trees.common.Access.READ)
                    if self.global_blackboard.exists(external_key):
                        resolved_value = self.global_blackboard.get(external_key)
                        value_source = "blackboard"
                except Exception:
                    pass
            
            # 优先级 3: 默认值
            if resolved_value is None and param_default is not None:
                resolved_value = param_default
                value_source = "default"
                applied_defaults.append((internal_name, param_default))
            
            # --- 2. 必填校验 ---
            if resolved_value is None:
                if param_required:
                    missing_required.append(param_def)
                continue
            
            # --- 3. 类型转换 ---
            try:
                resolved_value = self._convert_type(resolved_value, param_type)
            except (ValueError, TypeError) as e:
                raise ValueError(
                    f"[V3.0 Lite Type Error] 技能 '{skill_name}' 参数 '{internal_name}' "
                    f"类型转换失败: 值 '{resolved_value}' 无法转换为 {param_type}. 原因: {e}"
                )
            
            # --- 4. 枚举校验 ---
            if param_options is not None:
                # 将 options 和 value 统一转为字符串比较
                str_options = [str(o) for o in param_options]
                if str(resolved_value) not in str_options:
                    enum_errors.append({
                        "name": internal_name,
                        "value": resolved_value,
                        "options": param_options
                    })
            
            # --- 5. 写入映射表 ---
            macro_map[internal_name] = resolved_value
            
            # 同步到黑板 (使用 external_key)
            if value_source and self.global_blackboard:
                try:
                    self.global_blackboard.register_key(key=external_key, access=py_trees.common.Access.WRITE)
                    self.global_blackboard.set(external_key, resolved_value)
                except Exception:
                    pass
        
        # === 错误汇总 ===
        if missing_required:
            error_msg = f"[V3.0 Lite Interface Error] 技能 '{skill_name}' 加载失败！\n以下必填参数缺失:\n"
            for p in missing_required:
                name = p.get("internal_name") or p.get("name")
                ext = p.get("external_key", name)
                desc = p.get("description", "无描述")
                error_msg += f"  - {name} (external_key: {ext}): {desc}\n"
            error_msg += (
                f"\n解决方法:\n"
                f"  1. 通过 override_params 传入\n"
                f"  2. 在黑板中设置对应的 external_key\n"
                f"  3. 在 interface 中设置 default 值"
            )
            if HAS_ROSPY:
                rospy.logerr(error_msg)
            raise ValueError(error_msg)
        
        if enum_errors:
            error_msg = f"[V3.0 Lite Enum Error] 技能 '{skill_name}' 枚举校验失败！\n"
            for err in enum_errors:
                error_msg += (
                    f"  - 参数 '{err['name']}': 传入值 '{err['value']}' "
                    f"不在允许列表 {err['options']} 中\n"
                )
            if HAS_ROSPY:
                rospy.logerr(error_msg)
            raise ValueError(error_msg)
        
        # --- 日志 ---
        if applied_defaults and HAS_ROSPY:
            rospy.loginfo(f"[V3.0 Lite] 自动应用默认值: {applied_defaults}")
        if macro_map and HAS_ROSPY:
            rospy.loginfo(f"[V3.0 Lite] 宏替换映射表: {macro_map}")
        
        return macro_map
    
    # ==================== V3.0 Lite: 宏替换引擎 ====================
    
    @staticmethod
    def _convert_type(value: Any, target_type: str) -> Any:
        """
        将值转换为目标类型
        
        Args:
            value: 原始值
            target_type: 目标类型 (string/int/float/bool)
            
        Returns:
            转换后的值
            
        Raises:
            ValueError: 转换失败
        """
        if value is None:
            return None
        
        target_type = target_type.lower().strip()
        
        if target_type == "string" or target_type == "str":
            return str(value)
        elif target_type == "int":
            # 支持 "50" -> 50, 50.0 -> 50
            return int(float(value)) if isinstance(value, str) else int(value)
        elif target_type == "float":
            return float(value)
        elif target_type == "bool":
            if isinstance(value, bool):
                return value
            if isinstance(value, str):
                if value.lower() in ("true", "1", "yes"):
                    return True
                elif value.lower() in ("false", "0", "no"):
                    return False
                else:
                    raise ValueError(f"无法将 '{value}' 转换为 bool")
            return bool(value)
        else:
            # 未知类型，原样返回
            return value
    
    def _resolve_subtree_params(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """
        将主树传入的 params 解析为扁平键值对，供子树宏替换使用。
        支持 CUSTOM、RESOLVED、INPUT 等 source。
        """
        resolved = {}
        for key, value in params.items():
            if isinstance(value, dict):
                source = (value.get("source") or "").upper()
                raw = value.get("value")
                data_type = value.get("data_type", "string")
                if source in ("CUSTOM", "RESOLVED", "INPUT"):
                    try:
                        resolved[key] = self._convert_type(raw, data_type)
                    except (ValueError, TypeError):
                        resolved[key] = raw
                # 其他 source 忽略或按需扩展
            else:
                resolved[key] = value
        return resolved

    def _apply_macro_substitution_to_params(self, params: Dict[str, Any], macro_map: Dict[str, Any]) -> None:
        """
        对 params 字典进行宏替换（原地修改）。
        用于子树 params 阶段：主树 params -> 子树 params 中的 ${var}。
        """
        self._apply_macro_substitution({"params": params, "childs": []}, macro_map)

    def _apply_macro_substitution(self, tree_config: Dict[str, Any], macro_map: Dict[str, Any]) -> None:
        """
        递归遍历树配置，将 params 中的 ${var_name} 替换为 macro_map 中的实际值
        
        替换规则:
        - param.value == "${var_name}" 且 param.source == "INPUT"
            -> 用 macro_map[var_name] 替换 value，source 改为 "RESOLVED"
        - 如果 var_name 不在 macro_map 中，记录警告但不阻断
        
        Args:
            tree_config: 树节点配置 (会被原地修改)
            macro_map: 宏替换映射 {internal_name: resolved_value}
        """
        import re
        MACRO_PATTERN = re.compile(r'^\$\{(\w+)\}$')  # 匹配整个 ${var_name}
        
        # 处理当前节点的 params
        params = tree_config.get("params", {})
        for key, param_value in params.items():
            if not isinstance(param_value, dict):
                continue
            
            raw_value = param_value.get("value", "")
            source = param_value.get("source", "").upper()
            data_type = param_value.get("data_type", "string")
            
            # 只处理 source=INPUT 或包含 ${} 的值
            if not isinstance(raw_value, str):
                continue
            
            match = MACRO_PATTERN.match(raw_value)
            if match:
                var_name = match.group(1)
                if var_name in macro_map:
                    # 替换值并做类型转换
                    resolved = macro_map[var_name]
                    try:
                        resolved = self._convert_type(resolved, data_type)
                    except (ValueError, TypeError):
                        pass  # 保持原值，_parse_params 时再转换
                    
                    param_value["value"] = resolved
                    param_value["source"] = "RESOLVED"
                    
                    if HAS_ROSPY:
                        rospy.logdebug(
                            f"[V3.0 Lite Macro] {key}: ${{{var_name}}} -> {resolved} (type: {data_type})"
                        )
                else:
                    msg = f"[V3.0 Lite Macro] 警告: 变量 '{var_name}' 未在 interface 中定义"
                    if HAS_ROSPY:
                        rospy.logwarn(msg)
                    else:
                        print(msg)
        
        # 递归处理子节点（兼容 children）
        for child in _get_child_configs(tree_config):
            self._apply_macro_substitution(child, macro_map)
    

    def _build_tree_recursive(self, node_config: Dict[str, Any], parent_namespace: Optional[str], override_params: Dict[str, Any] = None) -> py_trees.behaviour.Behaviour:
        """
        递归构建行为树
        
        Args:
            node_config: 当前节点的配置
            parent_namespace: 父级 namespace
            override_params: 可选的覆盖参数，传递给所有子节点
            
        Returns:
            py_trees.behaviour.Behaviour: 构建的行为树节点
        """
        node_name = _resolve_node_class_name(node_config)
        if not node_name:
            lbl = node_config.get("label", "")
            if HAS_ROSPY:
                rospy.logerr(
                    f"[TreeFactory] 节点缺少有效类名（name / nodeType / className）。"
                    f" label={lbl!r} keys={list(node_config.keys())}"
                )
            raise ValueError(
                "行为树节点缺少有效类名（请在 JSON 中设置 name，或导出为 nodeType/className）。"
                f" label={lbl!r}"
            )
        node_label = node_config.get("label", node_name)
        node_params = node_config.get("params", {})
        childs = _get_child_configs(node_config)
        

        # 生成当前节点的 namespace
        namespace = parent_namespace

        # 如果是子树，处理子树逻辑
        if node_name.endswith(".json"):
            namespace = f"{parent_namespace}__{node_label}" if parent_namespace else node_label
            node_board = node_config.get("board")
            return self._handle_subtree(node_name, node_label, node_params, namespace, override_params, board=node_board)

        # 解析参数 (传入 override_params)
        parsed_params = self._parse_params(node_params, namespace, override_params)

        # 创建当前节点
        if node_name in ["Sequence", "Selector", "Parallel"]:
            return self._create_composite_node(node_name, node_label, parsed_params, namespace, childs, override_params)
        elif self._is_py_trees_decorator(node_name) and len(childs) >= 1:
            return self._create_decorator_node(node_name, node_label, namespace, childs, parsed_params, override_params)
        else:
            return self._create_node_instance(node_name, node_label, namespace, parsed_params)

    def _handle_subtree(self, json_file: str, label: str, params: Dict[str, Any], namespace: str, override_params: Dict[str, Any] = None, board: List[Dict] = None) -> py_trees.behaviour.Behaviour:
        """
        处理子树逻辑
        
        当有 board 时，为子树创建独立黑板空间（空间名为 label 字段的值），
        使多个相同子树实例之间的 board 参数相互隔离。
        
        Args:
            json_file: 子树 JSON 文件名
            label: 子树的标签（用于黑板空间名）
            params: 子树的参数
            namespace: 子树的 namespace
            override_params: 覆盖参数，传递给子树
            board: 子树所需的黑板参数数组（可选）
            
        Returns:
            py_trees.behaviour.Behaviour: 子树的根节点
        """
        # 构建子树的 namespace
        subtree_namespace = f"{json_file[:-5]}__{label}"

        # 当有 board 输入时，创建子树独立黑板空间（空间名 = label 字段的值）
        subtree_blackboard = None
        if board and apply_board_params_to_blackboard:
            subtree_blackboard = py_trees.blackboard.Client(
                name=f"subtree_board_{label}",
                namespace=label
            )
            apply_board_params_to_blackboard(subtree_blackboard, board, "", use_key_prefix=True)

        # 将 custom 类型的参数写入 namespace 对应的黑板
        namespace_blackboard = py_trees.blackboard.Client(name=subtree_namespace)
        for key, value in params.items():
            if isinstance(value, dict) and value.get("source") == "CUSTOM":
                namespace_blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)
                namespace_blackboard.set(key, value.get("value"))

        # 加载子树 config（必须深拷贝，避免污染共享配置）
        subtree_config = self.subtree_config[json_file]
        tree_config = copy.deepcopy(subtree_config["tree"])

        # === 子树级宏替换：主树 params -> 子树 macro_map ===
        subtree_macro_map = self._resolve_subtree_params(params)
        subtree_params_section = subtree_config.get("params", {})

        if subtree_params_section:
            # 两阶段替换：1) 主树 params 替换子树 params 中的 ${var}
            params_for_sub = copy.deepcopy(subtree_params_section)
            self._apply_macro_substitution_to_params(params_for_sub, subtree_macro_map)
            resolved_subtree_params = self._resolve_subtree_params(params_for_sub)
            # 2) 用解析后的子树 params 替换 tree 中的 ${var}
            self._apply_macro_substitution(tree_config, resolved_subtree_params)
        else:
            # 单阶段：主树 params 直接替换 tree 中的 ${var}
            self._apply_macro_substitution(tree_config, subtree_macro_map)

        # 动作组过滤：在递归构建之前移除未选中的动作组
        self._filter_action_groups(tree_config)

        # 递归构建子树
        subtree_root = self._build_tree_recursive(
            tree_config,
            parent_namespace=subtree_namespace,
            override_params=override_params
        )
        
        # [Patch] 标记子树源文件，以便前端可视化能显示文件名而非根节点类型
        subtree_root.subtree_file = json_file
        
        return subtree_root

    def _filter_action_groups(self, tree_config: Dict[str, Any]) -> None:
        """
        根据动作组过滤器，从子树根节点的 childs 列表中移除未选中的动作组。
        
        动作组的 label 格式为 nav_point_N_parallel，N 为组号（1-6）。
        仅当根节点是复合节点且设置了 action_group_filter 时生效。
        非动作组子节点（如 backward_0_2m、wait_backward）始终保留。
        """
        if not self.action_group_filter:
            return
        import re
        childs = tree_config.get("childs", [])
        filtered = []
        for child in childs:
            label = child.get("label", "")
            match = re.match(r"nav_point_(\d+)_parallel", label)
            if match:
                group_num = int(match.group(1))
                if group_num in self.action_group_filter:
                    filtered.append(child)
                else:
                    if HAS_ROSPY:
                        rospy.loginfo(f"[TreeFactory] 动作组过滤: 跳过 {label} (组 {group_num})")
                    else:
                        print(f"[TreeFactory] 动作组过滤: 跳过 {label} (组 {group_num})")
            else:
                filtered.append(child)
        tree_config["childs"] = filtered

    def _parse_params(self, params: Dict[str, Any], namespace: str, override_params: Dict[str, Any] = None) -> ParamsWrapper:
        """
        解析参数
        
        [Sprint 3 Req 5 改进]: 持久化覆盖策略
        当提供 override_params 时，不仅使用这些参数值，还会更新全局黑板，
        使得参数变更对后续任务持久生效（支持批处理模式）。
        
        Args:
            params: 参数字典
            namespace: 当前节点的 namespace
            override_params: 可选的覆盖参数，用于持久化注入
            
        Returns:
            ParamsWrapper: 包含解析后参数的包装类
        """
        parsed_params = {}
        
        for key, value in params.items():
            # === 1. 处理参数注入（最高优先级） ===
            if override_params and key in override_params:
                new_value = override_params[key]
                parsed_params[key] = new_value
                
                # [新逻辑]: 同步到全局黑板以实现持久化
                # 实现"持久化覆盖" - 改变后续任务的默认值
                if self.global_blackboard:
                    try:
                        # 强制注册/提权：确保当前 Client 拥有写权限
                        # 注意：py_trees 允许重复注册，这样可以避免权限问题
                        # 如果 board.json 已加载该 key 但只有 READ 权限，直接 set 会报错
                        self.global_blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)
                        
                        # 更新全局黑板
                        self.global_blackboard.set(key, new_value)
                        
                        if HAS_ROSPY:
                            rospy.loginfo(f"[TreeFactory] 持久化覆盖: 黑板键 '{key}' 已更新为 {new_value}")
                            
                    except Exception as e:
                        if HAS_ROSPY:
                            rospy.logwarn(f"[TreeFactory] 同步覆盖参数到黑板失败: {e}")

                # 跳过标准解析，因为已使用覆盖值
                continue

            # === 2. 标准解析（CUSTOM / RESOLVED / INPUT / READ_BOARD） ===
            if isinstance(value, dict):
                source = value.get("source", "").upper()  # 不区分大小写
                if source == "CUSTOM":
                    parsed_params[key] = value.get("value")
                elif source == "RESOLVED":
                    # V3.0 Lite: 已由宏替换引擎解析过的值
                    resolved_val = value.get("value")
                    data_type = value.get("data_type", "string")
                    try:
                        parsed_params[key] = self._convert_type(resolved_val, data_type)
                    except (ValueError, TypeError):
                        parsed_params[key] = resolved_val
                elif source == "INPUT":
                    # V3.0 Lite: 未被替换的 ${var}（macro_map 中没找到）
                    # 尝试从黑板按 value 原文读取
                    raw_val = value.get("value", "")
                    parsed_params[key] = raw_val
                elif source == "READ_BOARD":
                    # 从全局黑板读取
                    # 支持 board_key 字段：当参数名与黑板键不同时，用 board_key 指定黑板键
                    board_key = value.get("board_key", key)
                    self.global_blackboard.register_key(key=board_key, access=py_trees.common.Access.READ)
                    if self.global_blackboard.exists(board_key):
                        parsed_params[key] = self.global_blackboard.get(board_key)
                        # 额外存储 board_key，供节点运行时从黑板重新读取（动态注入）
                        parsed_params[f"{key}__board_key"] = board_key
                    else:
                        # 可选：处理缺失的键或使用默认值
                        pass
                else:
                    # 不识别的source或没有source，当作静态值处理
                    parsed_params[key] = value
            else:
                # 静态值
                parsed_params[key] = value
                
        return ParamsWrapper(parsed_params)


    def _create_composite_node(self, node_type: str, label: str, params: ParamsWrapper, namespace: str, childs: list, override_params: Dict[str, Any] = None) -> py_trees.composites.Composite:
        """
        创建复合节点（Sequence、Selector、Parallel）
        
        Args:
            override_params: 覆盖参数，传递给子节点
        """
        if node_type == "Sequence":
            memory = params.get("memory", False)
            node = py_trees.composites.Sequence(name=label, memory=memory)
        elif node_type == "Selector":
            memory = params.get("memory", False)
            node = py_trees.composites.Selector(name=label, memory=memory)
        elif node_type == "Parallel":
            policy = params.get("policy", "success_on_one")
            # 兼容py_trees 0.7.x和2.x版本的ParallelPolicy
            try:
                # py_trees 2.x: ParallelPolicy.SuccessOnOne 是类，需要实例化
                if policy == "success_on_one":
                    policy_class = getattr(ParallelPolicy, 'SuccessOnOne', None)
                    if policy_class is not None:
                        # 2.x 版本：实例化策略类
                        policy_enum = policy_class()
                    else:
                        # py_trees 0.7.x: 使用类属性（已经是实例）
                        policy_enum = ParallelPolicy.SUCCESS_ON_ONE
                else:
                    policy_class = getattr(ParallelPolicy, 'SuccessOnAll', None)
                    if policy_class is not None:
                        policy_enum = policy_class()
                    else:
                        policy_enum = ParallelPolicy.SUCCESS_ON_ALL
            except AttributeError:
                # fallback: 使用默认值
                policy_enum = ParallelPolicy.SuccessOnAll()
                
            node = py_trees.composites.Parallel(name=label, policy=policy_enum)
        else:
            raise ValueError(f"Unknown composite node type: {node_type}")

        # 并行构建子节点（如果启用且子节点数量较多）
        if self.enable_parallel_loading and len(childs) > 1:
            child_nodes = self._build_children_parallel(childs, namespace, override_params)
        else:
            # 顺序构建子节点 (传递 override_params)
            child_nodes = []
            for child_config in childs:
                child_node = self._build_tree_recursive(child_config, parent_namespace=namespace, override_params=override_params)
                child_nodes.append(child_node)

        # 按顺序添加子节点（保持原有顺序）
        for child_node in child_nodes:
            node.add_child(child_node)

        return node

    # py_trees 装饰器（仅需 name, child 参数）
    _PY_TREES_DECORATORS = frozenset({
        "FailureIsRunning", "FailureIsSuccess", "Inverter",
        "RunningIsFailure", "RunningIsSuccess", "SuccessIsFailure", "SuccessIsRunning",
        "PassThrough", "Count",
        # studio 自定义装饰器（由本工厂特殊处理）
        "Async",
        "RunIfIndex",
        "PressureDropGuard",
    })

    def _is_py_trees_decorator(self, node_name: str) -> bool:
        """判断是否为 py_trees 内置装饰器（仅需 name+child）"""
        return node_name in self._PY_TREES_DECORATORS

    def _create_decorator_node(self, node_name: str, label: str, namespace: Optional[str], childs: list, node_params: Dict[str, Any] = None, override_params: Dict[str, Any] = None) -> py_trees.behaviour.Behaviour:
        """
        创建 py_trees 装饰器节点（如 FailureIsRunning），包装子节点。
        支持 childs 长度 >= 1：单个子节点直接包装；多个子节点时用 Sequence（或 params 中 composite 指定的类型）包裹后作为装饰器的 child。
        """
        if len(childs) == 0:
            raise ValueError(f"Decorator '{node_name}' must have at least one child, got 0")
        node_params = node_params or {}
        composite_type = "Sequence"  # 默认用 Sequence 包裹多个子节点
        composite_val = node_params.get("composite", {})
        if isinstance(composite_val, dict) and "value" in composite_val:
            composite_type = str(composite_val.get("value", "Sequence")).strip()
        elif isinstance(composite_val, str):
            composite_type = composite_val.strip()
        if len(childs) == 1:
            child_node = self._build_tree_recursive(childs[0], parent_namespace=namespace, override_params=override_params)
        else:
            ct = composite_type.lower()
            composite_class = py_trees.composites.Selector if ct == "selector" else py_trees.composites.Sequence
            child_node = composite_class(name=f"{label}_wrapped", memory=True)
            for child_config in childs:
                sub = self._build_tree_recursive(child_config, parent_namespace=namespace, override_params=override_params)
                child_node.add_child(sub)
        try:
            if node_name == "Async":
                # 自定义：异步执行装饰器（避免阻塞 tick，用于 Parallel 真并行启动）
                from orchestration.nodes.async_decorator import Async

                tick_hz = 50.0
                raw = (node_params or {}).get("tick_hz")
                if isinstance(raw, dict) and "value" in raw:
                    try:
                        tick_hz = float(raw.get("value"))
                    except Exception:
                        tick_hz = 50.0
                elif raw is not None:
                    try:
                        tick_hz = float(raw)
                    except Exception:
                        tick_hz = 50.0
                return Async(name=label, child=child_node, tick_hz=tick_hz)

            if node_name == "RunIfIndex":
                # 自定义：按索引门控装饰器（仅当黑板 nav_point_index 匹配时执行子节点）
                from orchestration.nodes.run_if_index import RunIfIndex

                index = 1
                raw_idx = (node_params or {}).get("index")
                if isinstance(raw_idx, dict) and "value" in raw_idx:
                    try:
                        index = int(raw_idx.get("value"))
                    except Exception:
                        index = 1
                elif raw_idx is not None:
                    try:
                        index = int(raw_idx)
                    except Exception:
                        index = 1

                key = "nav_point_index"
                raw_key = (node_params or {}).get("nav_point_index_key")
                if isinstance(raw_key, dict) and "value" in raw_key:
                    key = str(raw_key.get("value"))
                elif raw_key is not None:
                    key = str(raw_key)

                return RunIfIndex(name=label, child=child_node, index=index, nav_point_index_key=key)

            if node_name == "PressureDropGuard":
                # 自定义：气压掉落检测装饰器（持续监控夹爪气压，检测到掉落时中断子节点）
                from orchestration.nodes.pressure_drop_guard import PressureDropGuard
                from skills.atomic.refactored_sdk.pressure_drop_detection import (
                    PressureDropDetectionParams,
                )

                params = PressureDropDetectionParams.from_node_params(node_params)
                return PressureDropGuard(name=label, child=child_node, params=params)

            dec_module = importlib.import_module("py_trees.decorators")
            dec_class = getattr(dec_module, node_name)
        except (ModuleNotFoundError, AttributeError) as e:
            raise ValueError(f"Cannot load py_trees decorator '{node_name}': {e}")
        return dec_class(name=label, child=child_node)

    def _build_children_parallel(self, child_configs: list, namespace: str, override_params: Dict[str, Any] = None) -> list:
        """
        并行构建子节点
        
        Args:
            override_params: 覆盖参数，传递给子节点
        """
        child_nodes = [None] * len(child_configs)  # 预分配列表，保持顺序

        # 使用线程池并行构建 (传递 override_params)
        with ThreadPoolExecutor(max_workers=min(len(child_configs), 8)) as executor:
            # 提交所有任务
            future_to_index = {
                executor.submit(self._build_tree_recursive, child_config, namespace, override_params): i
                for i, child_config in enumerate(child_configs)
            }

            # 收集结果
            for future in as_completed(future_to_index):
                index = future_to_index[future]
                try:
                    child_nodes[index] = future.result()
                except Exception as e:
                    raise RuntimeError(f"Failed to build child node at index {index}: {e}")

        return child_nodes

    def _create_node_instance(self, node_name: str, label: str, namespace: str, params: ParamsWrapper) -> py_trees.behaviour.Behaviour:
        """
        动态创建行为树节点实例
        
        [V3.0]: 支持自动扫描子目录
        
        Args:
            node_name: 节点类名
            label: 节点标签
            namespace: 命名空间
            params: 参数包装器
        
        Returns:
            创建的节点实例
        """
        # 动态导入模块（使用缓存避免重复导入）
        try:
            # 检查缓存
            if node_name in self._module_cache:
                node_class = self._module_cache[node_name]
            else:
                _studio_root = os.path.dirname(self.package_path)
                for _p in (_studio_root, self.package_path):
                    if _p not in sys.path:
                        sys.path.insert(0, _p)

                with self._node_index_lock:
                    if not hasattr(self, "_node_index"):
                        self._build_node_index()
                    module_path = self._node_index.get(node_name)

                if module_path is None and os.path.exists(self._node_library_path):
                    with self._node_index_lock:
                        if self._node_library is None:
                            with open(self._node_library_path) as f:
                                self._node_library = json.load(f)
                    node_entry = self._node_library.get("nodes", {}).get(node_name)
                    if node_entry and "module" in node_entry:
                        module_path = node_entry["module"]

                if module_path is None:
                    module_path = f"orchestration.nodes.{node_name}"
                
                if HAS_ROSPY:
                    rospy.logdebug(f"[TreeFactory] 导入节点: {node_name} -> {module_path}")
                
                module = importlib.import_module(module_path)
                node_class = getattr(module, node_name)
                # 缓存模块类
                self._module_cache[node_name] = node_class

        except ModuleNotFoundError as e:
            if HAS_ROSPY:
                rospy.logerr(f"[TreeFactory] 无法导入节点 '{node_name}': {e}")
                rospy.logerr(f"  包路径: {self.package_path}，请确认 node 目录完整且 config/definitions/node_library.json 存在")
            raise
        except AttributeError:
            raise ValueError(f"Node class '{node_name}' not found in py_trees.behaviours")

        print(f"node_name = {node_name}")
        
        return node_class(name=node_name, label=label, namespace=namespace, params=params)

    def _build_node_index(self):
        """
        [V3.0] 扫描所有子目录，建立 {类名: 模块路径} 索引
        
        支持 node/motion/MoveArm.py 等新目录结构
        """
        self._node_index = {}
        if HAS_ROSPY:
            rospy.loginfo(f"[TreeFactory] 正在构建节点索引: {self.node_dir}")
        
        for root, dirs, files in os.walk(self.node_dir):
            # 跳过 __pycache__ 目录
            dirs[:] = [d for d in dirs if d != '__pycache__']
            
            for file in files:
                if file.endswith(".py") and not file.startswith("__"):
                    rel_path = os.path.relpath(os.path.join(root, file), self.node_dir)
                    module_path = "orchestration.nodes." + rel_path.replace(os.path.sep, ".")[:-3]
                    stem = file[:-3]
                    self._node_index[stem] = module_path
                    pascal = _snake_to_pascal(stem)
                    if pascal != stem:
                        self._node_index[pascal] = module_path
        
        if HAS_ROSPY:
            rospy.loginfo(f"[TreeFactory] 节点索引构建完成，共 {len(self._node_index)} 个节点")

    def tick(self):
        """执行行为树的一次tick操作"""
        if not self.tree:
            print("[执行tick失败] 行为树实例未初始化")
            return False

        try:
            # 执行行为树的一次tick
            self.tree.tick()

            # 状态更新（仅刷新状态，不再递归全树检查FAILURE）
            self.update_bt_state()

            return True
        except AttributeError as e:
            # 特殊处理 TargetSMTIDRow 错误 - 这是 SMT 模式专用的键，非 SMT 模式可以忽略
            if 'TargetSMTIDRow' in str(e):
                # 只在第一次遇到时打印警告，避免刷屏
                if not hasattr(self, '_smt_warning_shown'):
                    print(f"[警告] 检测到 SMT 专用键访问，当前模式下忽略: {e}")
                    import traceback
                    traceback.print_exc()
                    self._smt_warning_shown = True
                return True  # 继续执行，不中断
            else:
                # 其他 AttributeError 正常处理
                print(f"[执行tick出错] {e}")
                import traceback
                traceback.print_exc()
                return False
        except Exception as e:
            print(f"[执行tick出错] {e}")
            return False

    def update_bt_state(self):
        """更新行为树状态，打印所有节点的状态"""
        if not self.tree or not self.tree.root:
            print("[Update Failed] Behavior tree is not initialized.")
            return

        # 禁用状态输出
        # print("\n[Behavior Tree State]")
        # self._traverse_and_print_node_status(self.tree.root, 0)

    def _traverse_and_print_node_status(self, node: py_trees.behaviour.Behaviour, level: int):
        """递归遍历并打印节点状态"""
        indent = "  " * level
        # print(f"{indent}{node.name}: {node.status}")

        if hasattr(node, "children") and node.children:
            for child in node.children:
                self._traverse_and_print_node_status(child, level + 1)

    def _check_for_failure(self, node):
        """递归检查节点及其子节点是否有失败状态

        Args:
            node: 要检查的节点

        Returns:
            bool: 如果任何节点返回失败状态则为True，否则为False
        """
        if not node:
            return False

        has_failure = False
        
        # 先递归检查子节点（从叶子节点开始报告）
        if hasattr(node, "children") and node.children:
            for child in node.children:
                if self._check_for_failure(child):
                    has_failure = True
        elif hasattr(node, "child") and node.child:
            # 处理具有单个子节点的节点类型
            if self._check_for_failure(node.child):
                has_failure = True

        # 然后检查当前节点是否失败
        if hasattr(node, "status"):
            from py_trees import common
            # 检查status是否为FAILURE（可能需要根据实际的Status枚举进行调整）
            if str(node.status) == "Status.FAILURE" or getattr(node.status, "value", None) == 3:  # 3通常是FAILURE的值
                print(f"[节点失败] {node.name}: {node.status}")
                has_failure = True

        return has_failure
