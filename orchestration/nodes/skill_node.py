import py_trees
from abc import ABC, abstractmethod

class ISkill(ABC):
    @abstractmethod
    def execute(self):
        pass
    
    @abstractmethod
    def cancel(self):
        pass

class SkillNode(py_trees.behaviour.Behaviour):
    def __init__(self, name, skill):
        super().__init__(name)
        self.skill = skill
    
    def update(self):
        result = self.skill.execute()
        
        if result.get('success'):
            return py_trees.common.Status.SUCCESS
        elif result.get('running'):
            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.FAILURE