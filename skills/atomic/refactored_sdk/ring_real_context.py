"""Run-scoped real-ring handoff state, independent of ROS and hardware.

Robot observations are always obtained by the active action. This object holds
only the anchor and outcome that must remain consistent across actions.
"""
import math
import threading


class RingRealContext:
    def __init__(self):
        self._lock = threading.RLock()
        self._left_hold_rad = None
        self._grasp_outcome = None

    def set_grasp_outcome(self, outcome):
        if outcome not in ('success', 'timeout_skip', 'simulated_skip'):
            raise ValueError('Invalid grasp outcome')
        with self._lock:
            self._grasp_outcome = outcome
            self._left_hold_rad = None

    def capture_left_hold(self, sensor_rad):
        if len(sensor_rad) < 11:
            raise ValueError('Joint feedback does not contain the left arm')
        anchor = tuple(float(x) for x in sensor_rad[4:11])
        if not all(math.isfinite(x) for x in anchor):
            raise ValueError('Invalid left-arm feedback')
        with self._lock:
            if self._grasp_outcome is None:
                raise RuntimeError('Cannot capture carry anchor before grasp completes')
            if self._left_hold_rad is not None:
                raise RuntimeError('Carry anchor already captured for this run')
            self._left_hold_rad = anchor
            return list(anchor)

    def left_hold(self):
        with self._lock:
            if self._left_hold_rad is None:
                raise RuntimeError('Carry left-arm anchor is unavailable')
            return list(self._left_hold_rad)

    def grasp_outcome(self):
        with self._lock:
            if self._grasp_outcome is None:
                raise RuntimeError('Grasp outcome is unavailable')
            return self._grasp_outcome
