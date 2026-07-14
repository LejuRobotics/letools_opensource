# -*- coding: utf-8 -*-
"""V3.0 节点基类（从 embodied 迁入）。"""

from __future__ import print_function, absolute_import

import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status

try:
    import rospy
    HAS_ROSPY = True
except ImportError:
    HAS_ROSPY = False


class BaseAction(Behaviour):
    """V3.0 节点通用基类；新 Action 节点应继承此类。"""

    def __init__(self, name, label, namespace, params):
        super(BaseAction, self).__init__(name=name)
        self.label = label
        self.namespace = namespace
        self.params = params
        self.global_blackboard = self.attach_blackboard_client()

    def setup(self, **kwargs):
        super(BaseAction, self).setup(**kwargs)

    def initialise(self):
        super(BaseAction, self).initialise()
