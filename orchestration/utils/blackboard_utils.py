import os
import json
import py_trees
from typing import Dict, Any, List, Optional


def get_pytrees_actions_root(from_file: str) -> Optional[str]:
    """
    从给定文件路径向上查找 pytrees_actions 根目录。
    适用于脚本可能放在 pytrees_actions 下任意子目录的情况。

    Args:
        from_file: 调用者的 __file__ 路径（如 start_skill_launch.py 或 get_skill_names_server.py）

    Returns:
        pytrees_actions 根目录的绝对路径，未找到则返回 None
    """
    current = os.path.dirname(os.path.abspath(from_file))
    while current and current != os.path.dirname(current):
        if os.path.basename(current) in ("pytrees_actions", "orchestration"):
            return current
        current = os.path.dirname(current)
    return None


def get_orchestration_root(from_file: str) -> Optional[str]:
    """向上查找 LeTools/orchestration 根目录。"""
    current = os.path.dirname(os.path.abspath(from_file))
    while current and current != os.path.dirname(current):
        if os.path.basename(current) == "orchestration":
            return current
        current = os.path.dirname(current)
    return None


def apply_flat_board_json(
    blackboard_client: py_trees.blackboard.Client,
    json_file_path: str,
) -> None:
    """
    加载扁平 dict 格式的 board.json（如 studio_smoke_v1）。
    顶层键直接写入黑板，值为 dict/list 等原生类型。
    """
    if not os.path.exists(json_file_path):
        print(f"警告: JSON文件不存在: {json_file_path}")
        return

    with open(json_file_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    if not isinstance(data, dict):
        print(f"警告: 扁平 board 格式应为 dict: {json_file_path}")
        return

    for key, value in data.items():
        if isinstance(value, list) and value and isinstance(value[0], dict) and "key" in value[0]:
            continue
        try:
            blackboard_client.register_key(key=key, access=py_trees.common.Access.WRITE)
        except Exception:
            pass
        try:
            blackboard_client.set(key, value)
        except Exception as e:
            print(f"设置黑板键 {key} 失败: {e}")


def apply_blackboard_data_from_json(blackboard_client: py_trees.blackboard.Client, json_file_path: str, use_group_prefix: bool = False) -> None:
    """
    从JSON文件加载键值对，并应用到blackboard客户端
    支持将分组和键解析为"group.key"格式，例如 common.box_length

    Args:新版本更新： 增加 use_group_prefix 参数，适配新版本 board.json 的键名格式
    """
    if not os.path.exists(json_file_path):
        print(f"警告: JSON文件不存在: {json_file_path}")
        return

    with open(json_file_path, 'r', encoding='utf-8') as f:
        data = json.load(f)

    # 预先收集所有需要注册的键
    all_keys = []
    if isinstance(data, dict):
        for group_name, group_items in data.items():
            if isinstance(group_items, list):
                for kv_item in group_items:
                    if isinstance(kv_item, dict) and 'key' in kv_item and 'value' in kv_item:
                        full_key = f"{group_name}_{kv_item['key']}" if use_group_prefix else kv_item['key']
                        all_keys.append(full_key)

    # 批量注册所有键的写入权限
    for key in all_keys:
        try:
            # 使用不带前导斜杠的键名
            blackboard_client.register_key(key=key, access=py_trees.common.Access.WRITE)
        except Exception as e:
            print(f"注册键 {key} 的写入权限失败: {e}")

    # 用于将字符串按类型转换为 Python 内部对象
    def _convert_value(raw_value: str, val_type: str) -> Any:
        if val_type == "int":
            return int(raw_value)
        elif val_type == "float":
            return float(raw_value)
        elif val_type == "bool":
            return str(raw_value).lower() in ("true", "1", "yes")
        elif val_type == "intArr":
            return [int(x.strip()) for x in str(raw_value).split(',')]
        elif val_type == "floatArr":
            return [float(x.strip()) for x in str(raw_value).split(',')]
        else:
            return raw_value  # 默认按原样/字符串返回

    # 现在设置所有键值对
    if isinstance(data, dict):
        # 遍历每个分组（如common、head等）
        for group_name, group_items in data.items():
            if isinstance(group_items, list):
                # 处理分组内的每个key-value对象
                for kv_item in group_items:
                    if isinstance(kv_item, dict) and 'key' in kv_item and 'value' in kv_item:
                        # 构建完整的键名：use_group_prefix=True 时 group_key，否则直接用 key
                        full_key = f"{group_name}_{kv_item['key']}" if use_group_prefix else kv_item['key']
                        
                        # 解析 value 以及可选的 type 字段
                        raw_value = kv_item['value']
                        val_type = kv_item.get('type', 'str')  # 如果没提供type，默认当字符串处理
                        try:
                            value = _convert_value(raw_value, val_type)
                        except Exception as e:
                            print(f"转换键 {full_key} (type: {val_type}) 的值 {raw_value} 失败: {e}")
                            value = raw_value  # 若转换失败则保留原有字符
                            
                        print(f"设置黑板键 {full_key} 为 {value} (type: {val_type})")

                        # 处理嵌套键路径（如pick_pose.stand_position_in_tag）
                        if '.' in kv_item['key']:
                            _apply_nested_dict_to_blackboard(blackboard_client, full_key, value)
                        else:
                            # 设置简单的键值对
                            try:
                                blackboard_client.set(full_key, value)
                            except Exception as e:
                                        print(f"设置黑板键 {full_key} 失败: {e}")

def apply_board_params_to_blackboard(blackboard_client: py_trees.blackboard.Client, board_params: List[Dict], namespace: str = "", use_key_prefix: bool = True) -> None:
    """
    将子树 board 参数数组注册并设置到黑板。
    当子树配置中有 board 字段时调用。
    - use_key_prefix=True（写入全局黑板）: full_key = f'{namespace}_{key}'，例如 grasp_box_subtree_ForceRatioZ
    - use_key_prefix=False（写入子树独立黑板）: full_key 为 param 的 key，例如 ForceRatioZ
    参数格式参考 board.json 中的单条参数（key, remark, type, value, ...）。
    """
    def _convert_value(raw_value: str, val_type: str) -> Any:
        if val_type == "int":
            return int(raw_value)
        elif val_type == "float":
            return float(raw_value)
        elif val_type == "bool":
            return str(raw_value).lower() in ("true", "1", "yes")
        elif val_type == "intArr":
            return [int(x.strip()) for x in str(raw_value).split(',')]
        elif val_type == "floatArr":
            return [float(x.strip()) for x in str(raw_value).split(',')]
        else:
            return raw_value

    if not board_params or not isinstance(board_params, list):
        return
    for kv_item in board_params:
        if not isinstance(kv_item, dict) or 'key' not in kv_item or 'value' not in kv_item:
            continue
        full_key = f"{namespace}_{kv_item['key']}" if use_key_prefix and namespace else kv_item['key']
        raw_value = kv_item['value']
        val_type = kv_item.get('type', 'str')
        try:
            value = _convert_value(str(raw_value), val_type)
        except Exception as e:
            print(f"转换键 {full_key} (type: {val_type}) 的值 {raw_value} 失败: {e}")
            value = raw_value
        try:
            blackboard_client.register_key(key=full_key, access=py_trees.common.Access.WRITE)
            blackboard_client.set(full_key, value)
        except Exception as e:
            print(f"注册/设置黑板键 {full_key} 失败: {e}")


def _apply_nested_dict_to_blackboard(blackboard_client: py_trees.blackboard.Client, prefix: str, data: Any) -> None:
    """
    递归地将嵌套字典应用到黑板上
    """
    if isinstance(data, dict):
        for key, value in data.items():
            # 构建嵌套键路径
            nested_key = f"{prefix}.{key}"
            _apply_nested_dict_to_blackboard(blackboard_client, nested_key, value)
    else:
        # 设置最终的键值对
        try:
            # 尝试注册键（如果尚未注册）
            try:
                blackboard_client.register_key(key=prefix, access=py_trees.common.Access.WRITE)
            except:
                pass  # 如果已经注册过，忽略错误

            blackboard_client.set(prefix, data)
        except Exception as e:
            print(f"设置嵌套黑板键 {prefix} 失败: {e}")

def create_blackboard_client_with_json(json_file_path: str, namespace: Optional[str] = None) -> py_trees.blackboard.Client:
    """
    创建一个新的blackboard客户端，并从JSON文件加载数据
    """
    # 使用更通用的客户端名称
    client = py_trees.blackboard.Client(name="main_blackboard_client", namespace=namespace)
    apply_blackboard_data_from_json(client, json_file_path)
    return client

def get_nested_value(data: Any, keys: str) -> Any:
    """
    从嵌套字典或列表中获取值，支持以点分隔的键路径
    特别处理列表中包含 {'key': 'xxx', 'value': 'xxx'} 格式的情况
    例如: get_nested_value(data, "head.percep_half_fov")
    """
    keys_list = keys.split('.')
    value = data

    for key in keys_list:
        if isinstance(value, dict):
            if key in value:
                value = value[key]
            else:
                return None
        elif isinstance(value, list):
            # 处理列表中包含 {'key': 'xxx', 'value': 'xxx'} 格式的情况
            found = False
            for item in value:
                if isinstance(item, dict) and item.get('key') == key:
                    value = item.get('value')
                    found = True
                    break
            if not found:
                return None
        else:
            return None

    return value
