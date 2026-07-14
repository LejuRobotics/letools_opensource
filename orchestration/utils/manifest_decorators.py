# -*- coding: utf-8 -*-
"""
V3.0 节点元数据装饰器模块

本模块提供 @define_manifest 装饰器，用于为行为树节点类附加元数据信息。
这些元数据用于：
1. 前端渲染节点面板和参数表单
2. 自动生成 node_library.json 供前端使用
3. 运行时的参数校验和数据映射

使用示例:
    @define_manifest(
        label="感知二维码节点",
        category=["skill", "perception"],
        description="扫描并识别场景中的所有二维码",
        params=[
            {"name": "max_percep_time", "type": "int", "default": 5, "description": "最大感知次数"}
        ],
        inputs=[
            {"name": "camera_feed", "type": "sensor_msgs/Image", "description": "相机图像输入"}
        ],
        outputs=[
            {"name": "AllTagInfoOfBase", "type": "list[Tag]", "description": "检测到的所有二维码信息"}
        ]
    )
    class FindAllTag(Behaviour):
        pass

兼容 Python 2/3
"""

from __future__ import print_function, absolute_import
import copy
from typing import Dict, List, Any, Optional, Callable, Type

# ============================================================================
# 全局 Manifest 注册表 - 用于追踪所有已装饰的节点类
# ============================================================================
_MANIFEST_REGISTRY = {}  # type: Dict[str, Dict[str, Any]]


# ============================================================================
# 参数/插座定义的数据类型
# ============================================================================

class ParamSpec:
    """
    参数规格定义类
    
    用于定义节点的配置参数（用户可在前端配置的值）
    
    Attributes:
        name: 参数名称
        type: 参数类型 (int, float, string, bool, intArr, floatArr, stringArr, json)
        default: 默认值
        description: 参数描述
        required: 是否必须
        options: 可选值列表（用于下拉选择）
        min_value: 最小值（数值类型）
        max_value: 最大值（数值类型）
    """
    VALID_TYPES = [
        'int', 'float', 'string', 'bool',
        'intArr', 'floatArr', 'stringArr',
        'json', 'pose', 'transform'
    ]
    
    def __init__(
        self,
        name,           # type: str
        type='string',  # type: str
        default=None,   # type: Any
        description='', # type: str
        required=False, # type: bool
        options=None,   # type: Optional[List[Any]]
        min_value=None, # type: Optional[float]
        max_value=None  # type: Optional[float]
    ):
        self.name = name
        self.type = type
        self.default = default
        self.description = description
        self.required = required
        self.options = options or []
        self.min_value = min_value
        self.max_value = max_value
        
    def to_dict(self):
        # type: () -> Dict[str, Any]
        """转换为字典格式"""
        result = {
            'name': self.name,
            'type': self.type,
            'description': self.description,
            'required': self.required,
        }
        if self.default is not None:
            result['default'] = self.default
        if self.options:
            result['options'] = self.options
        if self.min_value is not None:
            result['min'] = self.min_value
        if self.max_value is not None:
            result['max'] = self.max_value
        return result


class InputSpec:
    """
    输入插座规格定义类
    
    用于定义节点从黑板读取数据的"插座"
    
    Attributes:
        name: 插座名称（节点内部使用的标识符）
        type: 数据类型
        description: 插座描述
        required: 是否必须连接
        default_key: 默认绑定的黑板键名（可选，用于向后兼容）
    """
    def __init__(
        self,
        name,               # type: str
        type='any',         # type: str
        description='',     # type: str
        required=True,      # type: bool
        default_key=None,   # type: Optional[str]
        default_value=None, # type: Optional[Any]
        **kwargs            # type: Any
    ):
        self.name = name
        self.type = type
        self.description = description
        self.required = required
        self.default_key = default_key
        # 兼容字段：允许在输入插座上声明默认值
        self.default_value = default_value
        # 兼容历史/扩展字段，避免导入节点时抛异常
        self.extra_fields = kwargs
        
    def to_dict(self):
        # type: () -> Dict[str, Any]
        """转换为字典格式"""
        result = {
            'name': self.name,
            'type': self.type,
            'description': self.description,
            'required': self.required,
        }
        if self.default_key is not None:
            result['default_key'] = self.default_key
        if self.default_value is not None:
            result['default_value'] = self.default_value
        return result


class OutputSpec:
    """
    输出插座规格定义类
    
    用于定义节点向黑板写入数据的"产出"
    
    Attributes:
        name: 产出名称（默认的黑板键名）
        type: 数据类型
        description: 产出描述
        default_key: 默认绑定的黑板键名（可选，兼容旧配置）
    """
    def __init__(
        self,
        name,           # type: str
        type='any',     # type: str
        description='', # type: str
        default_key=None,   # type: Optional[str]
        **kwargs            # type: Any
    ):
        self.name = name
        self.type = type
        self.description = description
        self.default_key = default_key
        # 兼容历史字段，避免导入节点时因 manifest 扩展字段直接抛异常
        self.extra_fields = kwargs
        
    def to_dict(self):
        # type: () -> Dict[str, Any]
        """转换为字典格式"""
        result = {
            'name': self.name,
            'type': self.type,
            'description': self.description,
        }
        if self.default_key is not None:
            result['default_key'] = self.default_key
        # 保留历史扩展字段，避免信息丢失
        result.update(self.extra_fields)
        return result


# ============================================================================
# Manifest 数据类
# ============================================================================

class NodeManifest:
    """
    节点元数据清单类
    
    存储节点的完整元数据信息，用于前端渲染和运行时验证
    
    Attributes:
        label: 显示名称（中文）
        category: 分类路径 (如 ["skill", "perception"])
        description: 节点描述
        params: 配置参数列表
        inputs: 输入插座列表
        outputs: 输出产出列表
        version: 节点版本
        author: 作者
        tags: 标签列表（用于搜索）
    """
    def __init__(
        self,
        label='',                   # type: str
        category=None,              # type: Optional[List[str]]
        description='',             # type: str
        params=None,                # type: Optional[List[Dict[str, Any]]]
        inputs=None,                # type: Optional[List[Dict[str, Any]]]
        outputs=None,               # type: Optional[List[Dict[str, Any]]]
        version='1.0.0',            # type: str
        author='',                  # type: str
        tags=None                   # type: Optional[List[str]]
    ):
        self.label = label
        self.category = category or ['uncategorized']
        self.description = description
        self.version = version
        self.author = author
        self.tags = tags or []
        
        # 解析参数列表
        self.params = self._parse_specs(params, ParamSpec)
        # 解析输入插座列表
        self.inputs = self._parse_specs(inputs, InputSpec)
        # 解析输出产出列表
        self.outputs = self._parse_specs(outputs, OutputSpec)
    
    def _parse_specs(self, specs, spec_class):
        # type: (Optional[List], type) -> List
        """解析规格列表，支持字典和 Spec 对象两种格式"""
        if specs is None:
            return []
        
        result = []
        for spec in specs:
            if isinstance(spec, dict):
                result.append(spec_class(**spec))
            elif isinstance(spec, spec_class):
                result.append(spec)
            else:
                raise TypeError(
                    f"Invalid spec type: {type(spec)}. Expected dict or {spec_class.__name__}"
                )
        return result
    
    def to_dict(self):
        # type: () -> Dict[str, Any]
        """转换为完整的字典格式（用于导出 node_library.json）"""
        return {
            'label': self.label,
            'category': self.category,
            'description': self.description,
            'version': self.version,
            'author': self.author,
            'tags': self.tags,
            'params': [p.to_dict() for p in self.params],
            'inputs': [i.to_dict() for i in self.inputs],
            'outputs': [o.to_dict() for o in self.outputs],
        }
    
    def get_param(self, name):
        # type: (str) -> Optional[ParamSpec]
        """根据名称获取参数规格"""
        for param in self.params:
            if param.name == name:
                return param
        return None
    
    def get_input(self, name):
        # type: (str) -> Optional[InputSpec]
        """根据名称获取输入插座规格"""
        for inp in self.inputs:
            if inp.name == name:
                return inp
        return None
    
    def get_output(self, name):
        # type: (str) -> Optional[OutputSpec]
        """根据名称获取输出产出规格"""
        for out in self.outputs:
            if out.name == name:
                return out
        return None
    
    def get_required_params(self):
        # type: () -> List[ParamSpec]
        """获取所有必填参数"""
        return [p for p in self.params if p.required]
    
    def get_required_inputs(self):
        # type: () -> List[InputSpec]
        """获取所有必须连接的输入插座"""
        return [i for i in self.inputs if i.required]


# ============================================================================
# @define_manifest 装饰器
# ============================================================================

def define_manifest(
    label='',               # type: str
    category=None,          # type: Optional[List[str]]
    description='',         # type: str
    params=None,            # type: Optional[List[Dict[str, Any]]]
    inputs=None,            # type: Optional[List[Dict[str, Any]]]
    outputs=None,           # type: Optional[List[Dict[str, Any]]]
    version='1.0.0',        # type: str
    author='',              # type: str
    tags=None,               # type: Optional[List[str]]
    tree_type=''
):
    # type: (...) -> Callable[[Type], Type]
    """
    节点元数据装饰器

    用于为行为树节点类附加元数据信息（Manifest）。
    元数据存储在类的 `_manifest` 属性中，便于后续脚本扫描提取。
    
    Args:
        label: 显示名称（中文），如 "感知二维码节点"
        category: 分类路径，如 ["skill", "perception", "box"]
        description: 节点功能描述
        params: 配置参数列表，每个参数是一个字典，包含:
            - name: 参数名
            - type: 类型 (int, float, string, bool, intArr, floatArr, stringArr, json)
            - default: 默认值
            - description: 描述
            - required: 是否必填
            - options: 可选值列表
        inputs: 输入插座列表（从黑板读取的数据），每个插座是一个字典，包含:
            - name: 插座名称（节点内部标识）
            - type: 数据类型
            - description: 描述
            - required: 是否必须连接
            - default_key: 默认的黑板键名（向后兼容）
        outputs: 输出产出列表（写入黑板的数据），每个产出是一个字典，包含:
            - name: 产出名称（默认黑板键名）
            - type: 数据类型
            - description: 描述
            - default_key: 默认黑板键名（可选，兼容旧版）
        version: 节点版本号
        author: 作者
        tags: 标签列表，用于搜索
        tree_type: studio tree 分类 不传前端studio不读取
    
    Returns:
        装饰后的类，附加了 `_manifest` 属性
    
    Example:
        @define_manifest(
            label="感知二维码节点",
            category=["skill", "perception"],
            description="扫描并识别场景中的所有二维码",
            params=[
                {"name": "max_percep_time", "type": "int", "default": 5, "description": "最大感知次数"}
            ],
            inputs=[
                {"name": "camera_feed", "type": "sensor_msgs/Image", "required": False}
            ],
            outputs=[
                {"name": "AllTagInfoOfBase", "type": "list[Tag]", "description": "检测到的二维码"}
            ]
        )
        class FindAllTag(Behaviour):
            pass
    """
    def decorator(cls):
        # type: (Type) -> Type
        # 创建 Manifest 对象
        manifest = NodeManifest(
            label=label or cls.__name__,
            category=category,
            description=description or cls.__doc__ or '',
            params=params,
            inputs=inputs,
            outputs=outputs,
            version=version,
            author=author,
            tags=tags
        )
        
        # 附加到类的 _manifest 属性
        cls._manifest = manifest
        
        # 注册到全局注册表
        _MANIFEST_REGISTRY[cls.__name__] = {
            'class': cls,
            'manifest': manifest,
            'module': cls.__module__,
        }
        
        return cls
    
    return decorator


# ============================================================================
# 辅助函数
# ============================================================================

def get_manifest(cls_or_name):
    # type: (Any) -> Optional[NodeManifest]
    """
    获取节点类的 Manifest
    
    Args:
        cls_or_name: 节点类或类名字符串
    
    Returns:
        NodeManifest 对象，如果不存在返回 None
    """
    if isinstance(cls_or_name, str):
        # 从注册表查找
        entry = _MANIFEST_REGISTRY.get(cls_or_name)
        if entry:
            return entry['manifest']
        return None
    else:
        # 直接从类属性获取
        return getattr(cls_or_name, '_manifest', None)


def get_all_manifests():
    # type: () -> Dict[str, Dict[str, Any]]
    """
    获取所有已注册的节点 Manifest
    
    Returns:
        字典格式: {
            "NodeClassName": {
                "class": <class object>,
                "manifest": <NodeManifest object>,
                "module": "module.path"
            },
            ...
        }
    """
    return copy.copy(_MANIFEST_REGISTRY)


def export_manifests_to_dict():
    # type: () -> Dict[str, Any]
    """
    导出所有 Manifest 为字典格式（用于生成 node_library.json）
    
    Returns:
        {
            "version": "3.0",
            "nodes": {
                "FindAllTag": {...},
                "MoveHead": {...},
                ...
            }
        }
    """
    from datetime import datetime
    
    nodes = {}
    for name, entry in _MANIFEST_REGISTRY.items():
        manifest = entry['manifest']
        nodes[name] = manifest.to_dict()
        nodes[name]['module'] = entry['module']
    
    return {
        'version': '3.0',
        'generated_at': datetime.now().isoformat(),
        'node_count': len(nodes),
        'nodes': nodes,
    }



