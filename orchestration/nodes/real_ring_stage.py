"""Run a guarded real-ring action through its configured execution backend."""
import os

from py_trees.common import Status
from orchestration.nodes.base_node import BaseAction
from orchestration.utils.manifest_decorators import define_manifest

@define_manifest(label='真机圆环阶段', category=['motion'], tree_type='real_ring',
                 description='有反馈保护的单阶段真机动作，推理阶段为空载跳过',
                 params=[{'name':'stage','type':'string','default':'prepare',
                          'description':'prepare/grasp/carry/place'},
                         {'name':'action','type':'string','default':'',
                          'description':'prepare: torso/claws/head/arms; carry: arms/torso/base'}], inputs=[], outputs=[])
class RealRingStage(BaseAction):
    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self.stage = None
        self.noop = False
        self.failed = False
        self.real_stage = None

    def initialise(self):
        self.noop, self.failed = False, False
        self.real_stage = None
        self.stage = str(self.params.get('stage', '')).strip()
        if self.stage not in ('prepare', 'carry'):
            self.failed = True
            self.feedback_message = 'Invalid real stage'
            return
        backend = str(self.params.get('real_backend', 'integrated')).strip().lower()
        if backend != 'integrated':
            self.failed = True
            self.feedback_message = 'Invalid real ring backend'
            return
        if os.environ.get('STUDIO_DRY_RUN', '').lower() in ('1', 'true', 'yes'):
            self.noop = True
            return
        try:
            from orchestration.nodes.real_ring_runtime import RealStage
            action = str(self.params.get('action', '')).strip()
            self.real_stage = RealStage(self.stage, action=action) if action else RealStage(self.stage)
            self.real_stage.start()
        except Exception as exc:
            self.failed = True
            self.feedback_message = str(exc)

    def update(self):
        if self.failed: return Status.FAILURE
        if self.noop: return Status.SUCCESS
        if self.real_stage is not None:
            state = self.real_stage.update()
            if state == 'failure':
                self.feedback_message = self.real_stage.error or 'real ring stage failed'
            return {'running': Status.RUNNING, 'success': Status.SUCCESS, 'failure': Status.FAILURE}[state]
        return Status.FAILURE

    def terminate(self, new_status):
        if self.real_stage is not None:
            self.real_stage.terminate(new_status == Status.SUCCESS)
            self.real_stage = None
