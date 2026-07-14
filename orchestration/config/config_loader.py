import json
import os

class ConfigLoader:
    def __init__(self, config_dir=None, skill_name=None):
        self.config_dir = config_dir or os.path.dirname(os.path.abspath(__file__))
        self.skill_name = skill_name
        self.board_data = {}
        self.tree_data = {}
        self.child_tree_data = {}
    
    def load_board(self, filename='board.json'):
        if self.skill_name:
            path = os.path.join(self.config_dir, 'skills', self.skill_name, filename)
        else:
            path = os.path.join(self.config_dir, 'boards', filename)
        with open(path, 'r') as f:
            self.board_data = json.load(f)
        return self.board_data
    
    def load_tree(self, filename='py_tree.json'):
        if self.skill_name:
            path = os.path.join(self.config_dir, 'skills', self.skill_name, filename)
        else:
            path = os.path.join(self.config_dir, 'trees', filename)
        with open(path, 'r') as f:
            self.tree_data = json.load(f)
        return self.tree_data
    
    def load_child_tree(self, filename='py_tree_child.json'):
        if self.skill_name:
            path = os.path.join(self.config_dir, 'skills', self.skill_name, filename)
        else:
            path = os.path.join(self.config_dir, 'trees', filename)
        with open(path, 'r') as f:
            self.child_tree_data = json.load(f)
        return self.child_tree_data
    
    def set_skill_name(self, skill_name):
        self.skill_name = skill_name
    
    def get_param(self, key, default=None):
        for item in self.board_data.get('process', []):
            if item.get('key') == key:
                return item.get('value', default)
        return default
    
    def apply_to_blackboard(self, blackboard_client):
        import py_trees
        for item in self.board_data.get('process', []):
            key = item.get('key')
            value = item.get('value')
            data_type = item.get('type')
            
            if data_type == 'float':
                value = float(value)
            elif data_type == 'int':
                value = int(value)
            elif data_type == 'bool':
                value = value.lower() == 'true'
            
            blackboard_client.register_key(key, py_trees.common.Access.WRITE)
            setattr(blackboard_client, key, value)