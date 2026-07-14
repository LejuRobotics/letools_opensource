# -*- coding: utf-8 -*-
"""py_trees 0.7.x / 2.x 兼容补丁（须在 factory/controller 之前导入）。"""

import py_trees
from py_trees.behaviour import Behaviour

if not hasattr(py_trees.common, "Access"):

    class Access:
        READ = "READ"
        WRITE = "WRITE"
        EXCLUSIVE = "EXCLUSIVE"

    py_trees.common.Access = Access

if not hasattr(py_trees.blackboard, "Client"):
    from py_trees.blackboard import Blackboard

    _global_blackboard_singleton = Blackboard()

    class CompatibilityClient:
        def __init__(self, name=None, namespace=None):
            self.blackboard = _global_blackboard_singleton
            self.namespace = namespace
            self.name = name

        def register_key(self, key, access):
            pass

        def __getattr__(self, name):
            if name in ["blackboard", "namespace", "name"]:
                return object.__getattribute__(self, name)
            if hasattr(self.blackboard, "_Blackboard__shared_state"):
                shared_state = self.blackboard._Blackboard__shared_state
                if name not in shared_state:
                    raise AttributeError(f"Blackboard has no attribute '{name}'")
            return self.blackboard.get(name)

        def __setattr__(self, name, value):
            if name in ["blackboard", "namespace", "name"]:
                object.__setattr__(self, name, value)
            else:
                self.blackboard.set(name, value)

        def set(self, key, value):
            self.blackboard.set(key, value)

        def get(self, key):
            return self.blackboard.get(key)

        def exists(self, key):
            if hasattr(self.blackboard, "_Blackboard__shared_state"):
                return key in self.blackboard._Blackboard__shared_state
            return self.blackboard.get(key) is not None

    py_trees.blackboard.Client = CompatibilityClient

if not hasattr(Behaviour, "attach_blackboard_client"):

    def attach_blackboard_client(self, name=None, namespace=None):
        return py_trees.blackboard.Client(name=name or self.name, namespace=namespace)

    Behaviour.attach_blackboard_client = attach_blackboard_client
