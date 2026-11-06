"""相机命名空间解析；供发布端与独立感知进程共用，不加载硬件 SDK。"""

from typing import Any, Dict

from rosgraph.names import is_legal_name


_DEFAULT_NAMESPACES = {
    'head': '/camera',
    'left_wrist': '/left_wrist_camera',
    'right_wrist': '/right_wrist_camera',
}


def resolve_camera_namespace(config: Dict[str, Any], camera: str) -> str:
    """字段省略、空字符串或 YAML 空值均使用原命名空间；其他值严格校验。"""
    field = f'{camera}_camera_namespace'
    value = config.get(field)
    if value is None or value == '':
        return _DEFAULT_NAMESPACES[camera]
    if (not isinstance(value, str) or not value.startswith('/')
            or value.endswith('/') or not is_legal_name(value)):
        raise ValueError(
            f'camera.{field} 必须是合法的绝对 ROS 命名空间（如 /cam_h），实际: {value!r}'
        )
    return value


def resolve_camera_namespaces(config: Dict[str, Any]) -> Dict[str, str]:
    """发布端和订阅端共用解析规则，避免不同相机的图像与内参混流。"""
    namespaces = {camera: resolve_camera_namespace(config, camera)
                  for camera in _DEFAULT_NAMESPACES}
    enabled = {
        'head': config.get('enable_head', True),
        'left_wrist': config.get('enable_wrist_camera', False) and config.get('has_left_wrist', False),
        'right_wrist': config.get('enable_wrist_camera', False) and config.get('has_right_wrist', False),
    }
    active_namespaces = [namespace for camera, namespace in namespaces.items() if enabled[camera]]
    if len(set(active_namespaces)) != len(active_namespaces):
        raise ValueError('已启用相机的 namespace 必须不同，避免彩色、深度及内参混流')
    return namespaces
