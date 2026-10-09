"""Bounded, single-flight ROS service calls with uncertain outcomes latched."""

import math
import threading
import time


class RosServiceGate:
    """Reject concurrent calls and block reuse after an uncertain RPC outcome.

    Timeout/cancellation cannot cancel server-side execution. There is no
    automatic reset: confirm controller state before restarting the caller.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._active = None
        self._fault = None

    def call(self, name, service_type, request, timeout=5.0, cancel_event=None):
        import rospy

        timeout = float(timeout)
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("service timeout must be finite and positive")
        deadline = time.monotonic() + timeout
        done = threading.Event()
        flight = {"sent": False, "abandoned": False}
        outcome = {}

        def cancelled():
            return cancel_event is not None and cancel_event.is_set()

        def uncertain():
            return (
                f"{name}: service outcome is unknown; stop the flow and confirm "
                "controller state before restarting (client timeout is not cancellation)"
            )

        with self._lock:
            if self._fault is not None:
                raise RuntimeError(self._fault)
            if self._active is not None:
                raise RuntimeError("another control service call is still in flight")
            if cancelled():
                raise RuntimeError(f"{name}: service call cancelled before sending")
            self._active = flight

        def invoke():
            proxy = None
            response = None
            error = None
            try:
                rospy.wait_for_service(name, timeout=timeout)
                proxy = rospy.ServiceProxy(name, service_type, persistent=True)
                with self._lock:
                    if cancelled():
                        raise RuntimeError(f"{name}: service call cancelled before sending")
                    if flight["abandoned"] or time.monotonic() >= deadline:
                        raise TimeoutError(f"{name}: service discovery timed out")
                    flight["sent"] = True
                response = proxy(request)
            except BaseException as exc:
                error = exc
            finally:
                # Publish completion atomically. A late response must never open
                # a window for a second call, even before the waiter wakes up.
                with self._lock:
                    late = time.monotonic() >= deadline
                    abandoned = flight["abandoned"] or cancelled()
                    if error is None and (late or abandoned):
                        error = (TimeoutError(f"{name}: service response arrived too late")
                                 if late else RuntimeError(f"{name}: service call cancelled"))
                    if flight["sent"] and error is not None:
                        self._fault = uncertain()
                    if error is not None:
                        outcome["error"] = error
                    else:
                        outcome["response"] = response
                    if self._active is flight:
                        self._active = None
                    done.set()
                # close() may also block; never hold the lock or tree thread.
                if proxy is not None:
                    try:
                        proxy.close()
                    except Exception:
                        pass

        worker = threading.Thread(target=invoke, name="bounded_ros_service", daemon=True)
        try:
            worker.start()
        except BaseException:
            with self._lock:
                if self._active is flight:
                    self._active = None
            raise

        while True:
            done.wait(max(0.0, min(0.05, deadline - time.monotonic())))
            with self._lock:
                # A worker may have finished on time while this thread was
                # descheduled. Prefer its atomically published result.
                if done.is_set():
                    break
                is_cancelled = cancelled()
                if is_cancelled or time.monotonic() >= deadline:
                    flight["abandoned"] = True
                    if flight["sent"]:
                        self._fault = uncertain()
                    if is_cancelled:
                        raise RuntimeError(f"{name}: service call cancelled")
                    raise TimeoutError(f"{name}: service response timeout after {timeout:.2f}s")
        if "error" in outcome:
            raise outcome["error"]
        return outcome["response"]
