"""公共只读黑板路径：dict、Studio 参数数组、对象属性及动态场景段。"""

import re
from collections.abc import Mapping
from typing import Any

import py_trees
from py_trees.common import Access

_SELECTOR = re.compile(r"\{([\w/-]+)\}")


def read_blackboard_path(client: py_trees.blackboard.Client, path: str) -> Any:
    """读取 ``config.pose.yaw`` 或 ``scenes.{scene_id}.pose.yaw``。

    ``{key}`` 从同一黑板客户端读取一个非空字符串，用作单个路径段；
    不执行表达式、不递归插值。仅对根键和选择键注册 READ 权限。
    缺失根键/字段、重复的 Studio key、非法路径均抛 KeyError，绝不回退。
    无点号的普通键保持原样读取；对象属性兼容 py_trees 原有点号语义。
    """
    if not isinstance(path, str) or not path or any(not part for part in path.split(".")):
        raise KeyError(f"非法黑板路径: {path!r}")
    parts = path.split(".")
    if "{" in parts[0] or "}" in parts[0]:
        raise KeyError(f"黑板路径根键不支持动态选择: {path!r}")
    try:
        client.register_key(key=parts[0], access=Access.READ)
        value = client.get(parts[0])
        for part in parts[1:]:
            selector = _SELECTOR.fullmatch(part)
            if selector:
                key = selector.group(1)
                client.register_key(key=key, access=Access.READ)
                part = client.get(key)
                if not isinstance(part, str) or not part.strip() or any(c in part for c in ".{}"):
                    raise KeyError(f"选择键 {key!r} 必须是非空的单段字符串")
                part = part.strip()
            elif "{" in part or "}" in part:
                raise KeyError(f"非法动态路径段: {part!r}")
            if isinstance(value, Mapping):
                value = value[part]
            elif isinstance(value, list):
                matches = [item for item in value if isinstance(item, Mapping) and item.get("key") == part]
                if len(matches) != 1 or "value" not in matches[0]:
                    raise KeyError(f"参数数组字段 {part!r} 缺失、重复或没有 value")
                value = matches[0]["value"]
            else:
                value = getattr(value, part)
        return value
    except (KeyError, AttributeError) as exc:
        raise KeyError(f"读取黑板路径 {path!r} 失败: {exc}") from exc
