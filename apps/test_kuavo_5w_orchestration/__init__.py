# -*- coding: utf-8 -*-
"""orchestration 层测试包：单独运行/测试 LeTools/orchestration/nodes 下的行为树节点。

对齐 T4（apps/test_kuavo_5w_sdk_adapter）目录风格，但测试粒度为「编排层」——
构造 BaseAction 子类节点，走 PyTrees 节点生命周期（initialise → update），
节点内部通过 get_shared_hardware() 单例取硬件并驱动技能，体现分层架构中
编排层与技能层/适配器层的区别。
"""
