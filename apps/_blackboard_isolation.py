# -*- coding: utf-8 -*-
"""底盘伺服测试共用的 py_trees 黑板隔离。

py_trees 的 ``Blackboard`` 是进程级全局单例，节点通过 ``global_blackboard``
读写。测试直接往全局黑板写数据，同一进程里跑多个测试文件时，前一个文件
留下的 ``servo_exec_idle`` / ``servo_phase`` 等键会让后一个文件的用例误判。

本模块只被测试使用：
- pytest 入口：``apps/conftest.py`` 的 autouse fixture 在每个用例前调用；
- ``python3 <file>`` 入口：各测试文件 ``main()`` 循环里每个用例前调用。
"""

from py_trees.blackboard import Blackboard


def reset_blackboard():
    """清空全局黑板的键值、元数据与客户端注册。

    只应在用例开始时调用：清空 ``clients`` 会让此前创建的 Client 失效，
    因此调用方必须在构造节点 / Client 之前重置。
    """
    Blackboard.clear()
