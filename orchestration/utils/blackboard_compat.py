import py_trees
import rospy

try:
    from py_trees.blackboard import Client
    # Ensure Client is available
except ImportError:
    from py_trees.blackboard import Blackboard

    class Client:
        def __init__(self, name=None, namespace=None):
            self.blackboard = Blackboard()
            self.namespace = namespace
            self.name = name
            
        def register_key(self, key, access):
            # py_trees 0.7.x does not support access control registration
            pass
            
        def __getattr__(self, name):
            # Avoid infinite recursion for internal attributes
            if name in ['blackboard', 'namespace', 'name']:
                return super().__getattribute__(name)
            
            val = self.blackboard.get(name)
            if val is None:
                # py_trees 0.7.x might return None for missing keys
                # Optional: log warning if strictness is needed
                pass
            return val

        def __setattr__(self, name, value):
            if name in ['blackboard', 'namespace', 'name']:
                super().__setattr__(name, value)
            else:
                self.blackboard.set(name, value)
