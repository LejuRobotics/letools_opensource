# -*- coding: utf-8 -*-
"""仓库内 ROS 工作空间的 Python 运行环境辅助函数。

通常执行 ``source infrastructure/ros_packages/devel/setup.bash`` 后，catkin 会把
生成的消息、服务 Python 包加入 ``sys.path``。但 Python 入口很容易漏掉这一步，
从而出现 ROS 服务明明存在、进程却无法导入 ``kuavo_msgs`` 的情况。

这里仅补充当前 Python 进程所需的 ``dist-packages`` 路径；它不替代完整的 ROS
环境初始化。外部程序若还依赖 ``ROS_PACKAGE_PATH`` 等变量，仍应 source setup.bash。
"""

import sys
from pathlib import Path
from typing import Optional


def local_ros_python_path(project_root: Optional[Path] = None) -> Path:
    """返回本仓库 catkin 工作空间生成的 Python 包目录。"""
    root = (
        Path(project_root).resolve()
        if project_root is not None
        else Path(__file__).resolve().parents[2]
    )
    return (
        root
        / "infrastructure"
        / "ros_packages"
        / "devel"
        / "lib"
        / "python3"
        / "dist-packages"
    )


def ensure_local_ros_python_path(project_root: Optional[Path] = None) -> Path:
    """把仓库内已生成的 ROS Python 包目录加入当前进程。

    Returns:
        实际检查的目录，便于调用方在异常信息中给出明确诊断。

    Raises:
        FileNotFoundError: 工作空间尚未构建，生成目录不存在。
    """
    path = local_ros_python_path(project_root)
    if not path.is_dir():
        raise FileNotFoundError(
            f"ROS Python 包目录不存在: {path}；请先构建 "
            "infrastructure/ros_packages 工作空间"
        )

    path_text = str(path)
    if path_text not in sys.path:
        # 放在前面，确保使用与本仓库消息定义匹配的生成代码。
        sys.path.insert(0, path_text)
    return path
