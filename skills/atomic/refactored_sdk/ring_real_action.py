"""In-process action runner for the real-ring behavior tree.

The caller supplies a cooperative action which must observe the cancellation
event. No ROS object is created and no hardware command is sent at import time.
"""
import threading
from dataclasses import dataclass
from typing import Callable

from core.domain.result import Result
from core.domain.skill_params import SkillParams
from skills.base.skill_base import SkillBase


@dataclass
class RingRealActionParams(SkillParams):
    skill_name: str = 'ring_real_action'
    action: Callable = None
    timeout: float = 120.0


class RingRealActionSkill(SkillBase):
    def __init__(self):
        super().__init__(name='ring_real_action')
        self.cancel_event = threading.Event()
        self.finished = threading.Event()
        self._thread = None
        self._result = None

    def on_initialize(self, params):
        if not isinstance(params, RingRealActionParams) or not callable(params.action):
            return Result.fail('Invalid real-ring action')
        if self._thread is not None and self._thread.is_alive():
            return Result.fail('Previous action still running')
        self.cancel_event.clear()
        self.finished.clear()
        self._result = None
        self._thread = None
        return Result.ok()

    def _run(self):
        try:
            self._params.action(self.cancel_event)
            self._result = Result.fail('Action cancelled') if self.cancel_event.is_set() else Result.ok()
        except BaseException as exc:
            self._result = Result.fail(str(exc))
        finally:
            self.finished.set()

    def on_execute(self):
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, name='ring-real-action', daemon=True)
            self._thread.start()
        if self.finished.is_set():
            return self._result
        return Result.ok('Running')

    def on_cancel(self):
        self.cancel_event.set()
        if self._thread is not None:
            self._thread.join(timeout=15.0)
            if self._thread.is_alive():
                return Result.fail('Action did not stop; retain control ownership and inspect the robot')
        return Result.ok()

    def on_is_finished(self):
        return self.finished.is_set()
