# -*- coding: utf-8 -*-
"""根据 /leju_claw_state 的实测反馈确认抓取。

右夹爪先报告运动或到位，再连续报告抓到物体（state=3），
持续达到配置时长后确认成功。程序检查反馈时间戳、
实测开度和状态字段的有效性。
"""

import math
import threading
import time


class GraspFeedbackState:
    """检查当前抓取过程中的有效“运动或到位→抓到物体”状态序列。"""

    def __init__(self, config):
        self.config = dict(config)
        if self.config.get("mode") != "claw_only":
            raise ValueError("夹爪反馈判断模式必须为 claw_only")
        for key in ("hold_seconds", "max_feedback_age_seconds",
                    "max_header_age_seconds"):
            value = float(self.config[key])
            if not math.isfinite(value) or value <= 0:
                raise ValueError("夹爪反馈配置无效：" + key)
            self.config[key] = value
        self._sample = None
        self._last_stamp = None
        self._ready = False
        self._held_since = None
        self._lock = threading.Lock()
        self._received_count = 0
        self._accepted_count = 0
        self._last_received = None
        self._header_age = None
        self._last_rejection = None
        self._raw_state = None
        self._raw_opening = None

    def _reset(self):
        self._ready = False
        self._held_since = None

    def _fresh(self, now):
        return (self._sample is not None and
                0 <= now - self._sample[0] <= self.config["max_feedback_age_seconds"])

    def _reject(self, reason, clear=True):
        self._last_rejection = reason
        if clear:
            self._sample = None
            self._reset()
        return False

    def observe(self, source, values, now, stamp=None, ros_now=None, latched=False):
        """检查消息时间戳，接收有效且连续的夹爪反馈。"""
        if source != "claw":
            raise ValueError("反馈来源必须为 claw")
        with self._lock:
            self._received_count += 1
            self._last_received = now if math.isfinite(now) else None
            self._header_age = (ros_now - stamp if stamp is not None and ros_now is not None
                                and math.isfinite(stamp) and math.isfinite(ros_now) else None)
            self._raw_state = None
            self._raw_opening = None
            if isinstance(values, (tuple, list)) and len(values) == 3:
                if values[1] in (-1, 0, 1, 2, 3):
                    self._raw_state = int(values[1])
                try:
                    raw_opening = float(values[0])
                    if math.isfinite(raw_opening):
                        self._raw_opening = raw_opening
                except (TypeError, ValueError, OverflowError):
                    pass
            if not self._fresh(now):
                self._reset()
            if latched:
                return self._reject("latched_feedback")
            if (not math.isfinite(now) or stamp is None or
                    not math.isfinite(stamp) or stamp <= 0 or ros_now is None or
                    not math.isfinite(ros_now)):
                return self._reject("invalid_timestamp")
            if self._header_age > self.config["max_header_age_seconds"]:
                return self._reject("header_too_old")
            if self._header_age < -0.05:
                return self._reject("header_in_future")
            if self._last_stamp is not None and stamp <= self._last_stamp:
                return self._reject("duplicate_or_out_of_order_stamp", clear=False)
            if (self._last_stamp is not None and
                    stamp - self._last_stamp > self.config["max_feedback_age_seconds"]):
                self._reset()
            try:
                position, state, effort = values
                position = float(position)
                if not math.isfinite(position) or not 0 <= position <= 100:
                    raise ValueError("夹爪实测开度无效")
                # 状态字段必须是有效枚举值，避免将 3.5 等异常值转换为 state=3。
                if isinstance(state, bool) or state not in (-1, 0, 1, 2, 3):
                    raise ValueError("夹爪实测状态缺失或无效")
                state = int(state)
                effort = float(effort) if effort is not None else None
                if effort is not None and not math.isfinite(effort):
                    effort = None
            except (TypeError, ValueError, OverflowError):
                return self._reject("missing_or_invalid_claw_fields")

            self._sample = (now, position / 100.0, state, effort)
            self._accepted_count += 1
            self._last_rejection = None
            self._last_stamp = stamp
            if state in (1, 2):
                self._ready = True
                self._held_since = None
            elif state == 3 and self._ready:
                if self._held_since is None:
                    self._held_since = now
            else:
                # 状态未知、驱动报错，或订阅启动时已处于抓取状态。
                self._held_since = None
                if state != 3:
                    self._ready = False
            return True

    def evaluate(self, now):
        with self._lock:
            fresh = self._fresh(now)
            if not fresh:
                self._reset()
            _, opening, state, effort = self._sample or (None, None, None, None)
            held = max(0.0, now - self._held_since) if self._held_since is not None else 0.0
            success = bool(fresh and state == 3 and self._ready and
                           held >= self.config["hold_seconds"])
            if not fresh:
                reason = "missing_or_stale_feedback"
            elif state in (-1, 0):
                reason = "driver_error_or_unknown"
            elif not self._ready:
                reason = "waiting_for_this_attempt_moving_or_reached"
            elif state != 3:
                reason = "waiting_for_grabbed"
            else:
                reason = "driver_grabbed" if success else "holding_grabbed"
            return {
                "success_allowed": success,
                "reason": reason,
                "fresh": {"claw": fresh},
                "claw_opening": opening,
                "claw_state": state,
                "claw_effort": effort,
                "driver_reports_grabbed": bool(fresh and state == 3),
                "hold_seconds": held,
                "received_messages": self._received_count,
                "accepted_messages": self._accepted_count,
                "last_rejection": self._last_rejection,
                "header_age_at_receive_seconds": self._header_age,
                "seconds_since_receive": (now - self._last_received
                                          if self._last_received is not None else None),
                "seconds_since_accepted": now - self._sample[0] if self._sample else None,
                "raw_claw_state": self._raw_state,
                "raw_claw_opening_percent": self._raw_opening,
            }


class RosGraspFeedbackObserver:
    """订阅夹爪反馈，为仿真与真机提供统一的判断接口。"""

    def __init__(self, config):
        import rospy
        from kuavo_msgs.msg import lejuClawState

        self.state = GraspFeedbackState(config)
        self.config = self.state.config
        index = config.get("right_claw_index", 1)
        if isinstance(index, bool) or not isinstance(index, int) or index < 0:
            raise ValueError("right_claw_index 必须是非负整数")
        self._index = index
        self._rospy = rospy
        self._closed = False
        self._subscription = rospy.Subscriber(
            config["claw_topic"], lejuClawState, self._on_claw,
            queue_size=1, tcp_nodelay=True)
        rospy.on_shutdown(self.close)

    def _on_claw(self, msg):
        i = self._index
        positions, states, efforts = msg.data.position, msg.state, msg.data.effort
        header = getattr(msg, "header", None)
        stamp = header.stamp.to_sec() if header is not None else 0.0
        connection = getattr(msg, "_connection_header", {}) or {}
        self.state.observe("claw", (
            positions[i] if len(positions) > i else None,
            states[i] if len(states) > i else None,
            efforts[i] if len(efforts) > i else None),
            time.monotonic(), stamp=stamp, ros_now=self._rospy.Time.now().to_sec(),
            latched=connection.get("latching") == "1")

    def evaluate(self):
        return self.state.evaluate(time.monotonic())

    def close(self):
        if self._closed:
            return
        self._closed = True
        self._subscription.unregister()
