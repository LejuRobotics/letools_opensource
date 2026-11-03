# -*- coding: utf-8 -*-
"""PalletizingLineIkSkill 逐帧 IK 调度（假硬件 + 假时钟，无运动、无 ROS）。

守的是 DOCS-161 对主循环的约定：

1. 逐帧走非阻塞的 ``publish_arm_joint_setpoint``，只有预备姿态还走阻塞的
   ``send_arm_joint_trajectory``；
2. 默认 ``ik_rate=40Hz``，用绝对 deadline 调度（总时长落在帧网格上，不随每帧
   计算耗时累加漂移）；
3. 落后 deadline 不超过一个周期时追平并记录，超过一个周期明确失败中止，
   不带着追不上的时间轴继续发设定点；
4. ``q_prev`` 只在设定点发布成功后更新；发布失败或 IK 无连续解立即停手；
5. both / left / right 三模式下，未选臂的关节值保持不动。
"""

import itertools
import math
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import skills.atomic.refactored_sdk.palletizing_line_ik as ik_skill  # noqa: E402
from core.domain.ik import IkSolution  # noqa: E402
from core.domain.palletizing_path import (  # noqa: E402
    PalletizingPathParams,
    build_palletizing_path,
    sample_positions,
)
from core.domain.result import Result  # noqa: E402
from core.domain.transform import Transform  # noqa: E402
from skills.atomic.refactored_sdk.palletizing_line_ik import (  # noqa: E402
    PalletizingLineIkParams,
    PalletizingLineIkSkill,
)

RATE = 40.0
PERIOD = 1.0 / RATE
DEG_PER_FRAME = 0.5
SETTLE = 0.25
# 左臂 1~7°、右臂 8~14°（弧度）——两半明显不同，才能看出「未选臂没被动过」。
INITIAL = [math.radians(1.0 * (index + 1)) for index in range(14)]

# 与 _params() 的默认几何一致（假硬件 TF 为单位变换，方向为 forward/left），
# 只用来算「每段多少帧」，好把段间等待定位到段边界上。
_PATH_SEGMENTS = build_palletizing_path(
    (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0),
    PalletizingPathParams(x_distance=0.14, y_offset=0.07, z_height=0.18,
                          approach_distance=0.05),
    edge_duration=0.1, approach_duration=0.05)
_SEGMENT_FRAMES = [len(sample_positions(segment, RATE)) for segment in _PATH_SEGMENTS]
_CUMULATIVE_FRAMES = list(itertools.accumulate(_SEGMENT_FRAMES))


class _FakeClock:
    """可控时钟：sleep 推进时间，便于断言 deadline 调度而不是真实等待。"""

    def __init__(self):
        self.now = 0.0
        self.sleeps = []
        # 交替记录 ("publish", 设定点序号) / ("sleep", 秒数)，用于断言先后顺序。
        self.log = []

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        assert seconds >= 0
        self.sleeps.append(seconds)
        self.log.append(("sleep", seconds))
        self.now += seconds


class _FakeIkHardware:
    """IK 求解与设定点发布的替身；solve_ik 消耗假时钟模拟单帧计算耗时。"""

    def __init__(self, clock, joints=None, work_pattern=(0.0,),
                 publish_fail_at=None, ik_fail_at=None):
        self.clock = clock
        self.joints = list(INITIAL if joints is None else joints)
        self.work_pattern = tuple(work_pattern)
        self.publish_fail_at = publish_fail_at
        self.ik_fail_at = ik_fail_at
        self.setpoints = []
        self.trajectory_calls = []
        self.requests = []
        self._frame = 0

    def get_arm_joint_positions(self):
        return Result.ok(data=list(self.joints))

    def get_transform(self, target_frame, source_frame, timeout=0.2):
        return Result.ok(data=Transform.from_lists([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]))

    def solve_ik(self, request):
        frame = self._frame
        self._frame += 1
        self.requests.append(request)
        self.clock.now += self.work_pattern[frame % len(self.work_pattern)]
        if self.ik_fail_at is not None and frame == self.ik_fail_at:
            return Result.fail("模拟 IK 跳变 9.0° > 8.0°，拒绝下发")
        q0 = list(request.left_joint_angles) + list(request.right_joint_angles)
        q_arm = [value + math.radians(DEG_PER_FRAME) for value in q0]
        return Result.ok(data=IkSolution(
            q_arm=q_arm,
            left_hand_pose=list(request.left_pose),
            right_hand_pose=list(request.right_pose),
            elapsed_ms=1.0,
        ))

    def publish_arm_joint_setpoint(self, positions_deg):
        index = len(self.setpoints)
        self.setpoints.append(list(positions_deg))
        self.clock.log.append(("publish", index))
        if self.publish_fail_at is not None and index == self.publish_fail_at:
            return Result.fail("模拟设定点发布失败")
        return Result.ok()

    def send_arm_joint_trajectory(self, positions, time_sec=0.0):
        self.trajectory_calls.append((list(positions), time_sec))
        return Result.ok()


def _params(**overrides):
    """短周期参数：每段 3~5 帧，测试跑得快且覆盖段边界。"""
    base = dict(
        ik_rate=RATE,
        edge_duration=0.1,
        approach_duration=0.05,
        prepare_pose=False,
        prep_settle=0.0,
        grasp_dwell=0.0,
        place_dwell=0.0,
        segment_settle=0.0,
        ik_max_step_deg=8.0,
    )
    base.update(overrides)
    return PalletizingLineIkParams(**base)


def _run(hardware, params, monkeypatch):
    clock = hardware.clock
    monkeypatch.setattr(ik_skill, "time", clock)
    skill = PalletizingLineIkSkill(hardware=hardware)
    assert skill.initialize(params).success
    return skill.execute()


# ----------------------------------------------------------------------
# 下发接口：逐帧用非阻塞设定点
# ----------------------------------------------------------------------
@pytest.mark.unit
def test_frames_publish_setpoints_not_blocking_trajectory(monkeypatch):
    clock = _FakeClock()
    hardware = _FakeIkHardware(clock)

    result = _run(hardware, _params(), monkeypatch)

    assert result.success, result.message
    assert hardware.setpoints, "逐帧必须发布设定点"
    assert hardware.trajectory_calls == [], "逐帧不允许再走阻塞轨迹接口"


@pytest.mark.unit
def test_prepare_pose_still_uses_blocking_trajectory(monkeypatch):
    clock = _FakeClock()
    hardware = _FakeIkHardware(clock)

    result = _run(hardware, _params(prepare_pose=True), monkeypatch)

    assert result.success, result.message
    assert len(hardware.trajectory_calls) == 1, "预备姿态仍走一次阻塞下发"
    prepare_degrees, time_sec = hardware.trajectory_calls[0]
    assert time_sec == 0.0
    # J4=-90°、J6=J7=0°，左右臂各一组
    assert prepare_degrees[3] == pytest.approx(-90.0)
    assert prepare_degrees[5] == pytest.approx(0.0)
    assert prepare_degrees[6] == pytest.approx(0.0)
    assert prepare_degrees[10] == pytest.approx(-90.0)


@pytest.mark.unit
def test_setpoint_payload_is_14_degrees_left_then_right(monkeypatch):
    clock = _FakeClock()
    hardware = _FakeIkHardware(clock)

    result = _run(hardware, _params(), monkeypatch)

    assert result.success, result.message
    first = hardware.setpoints[0]
    expected = [math.degrees(value + math.radians(DEG_PER_FRAME)) for value in INITIAL]
    assert len(first) == 14
    assert first == pytest.approx(expected)
    # 左臂在前 7 个、右臂在后 7 个：若顺序写反，两半会整体对调。
    assert first[:7] != pytest.approx(expected[7:])


@pytest.mark.unit
def test_default_rate_is_40hz_and_continuity_limit_is_8deg(monkeypatch):
    params = PalletizingLineIkParams()
    assert params.ik_rate == pytest.approx(40.0)
    assert params.ik_max_step_deg == pytest.approx(8.0)

    clock = _FakeClock()
    hardware = _FakeIkHardware(clock)
    result = _run(hardware, params, monkeypatch)

    assert result.success, result.message
    for request in hardware.requests:
        assert request.max_joint_step_deg == pytest.approx(8.0)


# ----------------------------------------------------------------------
# 40Hz 绝对 deadline 调度
# ----------------------------------------------------------------------
@pytest.mark.unit
def test_absolute_deadline_keeps_total_time_on_frame_grid(monkeypatch):
    """即使每帧耗时波动，总时长仍等于帧数×周期（绝对 deadline，不累加漂移）。"""
    clock = _FakeClock()
    hardware = _FakeIkHardware(clock, work_pattern=(0.004, 0.001, 0.006, 0.002, 0.0))

    result = _run(hardware, _params(), monkeypatch)

    assert result.success, result.message
    frames = len(hardware.setpoints)
    assert frames > 0
    assert clock.now == pytest.approx(frames * PERIOD)
    assert all(seconds >= 0 for seconds in clock.sleeps)


@pytest.mark.unit
def test_overrun_within_one_period_is_tolerated(monkeypatch):
    """落后不足一个周期：追平下一帧，继续跑完。"""
    clock = _FakeClock()
    # 首帧计算耗时 1.2 个周期 → 落后 0.2 个周期，仍应继续
    hardware = _FakeIkHardware(clock, work_pattern=(PERIOD * 1.2, 0.0, 0.001))

    result = _run(hardware, _params(), monkeypatch)

    assert result.success, result.message
    assert len(hardware.setpoints) > 1


@pytest.mark.unit
def test_overrun_beyond_one_period_aborts_segment(monkeypatch):
    """落后超过一个完整周期：明确失败中止，不再继续发设定点。"""
    clock = _FakeClock()
    hardware = _FakeIkHardware(clock, work_pattern=(PERIOD * 2.5,))

    result = _run(hardware, _params(), monkeypatch)

    assert not result.success
    assert "帧调度超时" in result.message
    assert "超过周期" in result.message
    assert len(hardware.setpoints) == 1
    assert len(hardware.requests) == 1


# ----------------------------------------------------------------------
# 段间收敛等待（segment_settle）
# ----------------------------------------------------------------------
def _settle_counters(clock):
    """每个 segment_settle 睡眠发生时，已发布的设定点数量。"""
    counters = []
    published = 0
    for kind, value in clock.log:
        if kind == "publish":
            published += 1
        elif value == pytest.approx(SETTLE):
            counters.append(published)
    return counters


@pytest.mark.unit
def test_default_segment_settle_waits_after_every_segment(monkeypatch):
    """默认 0.25s，且每个成功段各一次——不是只等首段或末段。"""
    assert PalletizingLineIkParams().segment_settle == pytest.approx(SETTLE)

    clock = _FakeClock()
    hardware = _FakeIkHardware(clock)
    result = _run(hardware, _params(segment_settle=PalletizingLineIkParams().segment_settle),
                  monkeypatch)

    assert result.success, result.message
    assert len(hardware.setpoints) == _CUMULATIVE_FRAMES[-1], \
        "每段帧数假设与 _PATH_SEGMENTS 不一致"
    assert _settle_counters(clock) == _CUMULATIVE_FRAMES


@pytest.mark.unit
def test_segment_settle_runs_after_segment_frames_and_before_dwell(monkeypatch):
    clock = _FakeClock()
    hardware = _FakeIkHardware(clock)

    result = _run(hardware, _params(segment_settle=SETTLE, grasp_dwell=0.4, place_dwell=0.6),
                  monkeypatch)

    assert result.success, result.message
    # 每次等待都落在段末：此时该段全部帧都已发布（计数等于累计帧数）。
    assert _settle_counters(clock) == _CUMULATIVE_FRAMES
    for dwell in (0.4, 0.6):
        dwell_index = next(
            index for index, (kind, value) in enumerate(clock.log)
            if kind == "sleep" and value == pytest.approx(dwell))
        previous_kind, previous_value = clock.log[dwell_index - 1]
        assert previous_kind == "sleep" and previous_value == pytest.approx(SETTLE), \
            f"{dwell}s 的抓取/放置停顿必须排在本段收敛等待之后"


@pytest.mark.unit
@pytest.mark.parametrize("settle", [0.0, -0.5])
def test_non_positive_segment_settle_adds_no_wait(settle, monkeypatch):
    clock = _FakeClock()
    hardware = _FakeIkHardware(clock)

    result = _run(hardware, _params(segment_settle=settle), monkeypatch)

    assert result.success, result.message
    assert _settle_counters(clock) == []
    assert clock.now == pytest.approx(len(hardware.setpoints) * PERIOD)


@pytest.mark.unit
def test_failed_segment_does_not_settle(monkeypatch):
    clock = _FakeClock()
    # 第 6 次 solve_ik（帧序号 5）落在第二段 lift_z 内，该段失败立即返回。
    hardware = _FakeIkHardware(clock, ik_fail_at=5)

    result = _run(hardware, _params(segment_settle=SETTLE), monkeypatch)

    assert not result.success
    assert "IK 无连续可行解" in result.message
    assert _settle_counters(clock) == [_CUMULATIVE_FRAMES[0]], \
        "失败段不得执行该段的 segment_settle"


# ----------------------------------------------------------------------
# 失败策略与 q_prev 更新时机
# ----------------------------------------------------------------------
@pytest.mark.unit
def test_publish_failure_stops_immediately_and_keeps_q_prev(monkeypatch):
    clock = _FakeClock()
    hardware = _FakeIkHardware(clock, publish_fail_at=1)

    result = _run(hardware, _params(), monkeypatch)

    assert not result.success
    assert "关节设定点发布失败" in result.message
    # 失败帧之后不再求解、不再发布
    assert len(hardware.requests) == 2
    assert len(hardware.setpoints) == 2
    # 第二帧的 q0 是「第一帧发布成功后的解」，不是初始值
    expected_q0 = INITIAL[0] + math.radians(DEG_PER_FRAME)
    assert hardware.requests[1].left_joint_angles[0] == pytest.approx(expected_q0)
    assert expected_q0 != pytest.approx(INITIAL[0])


@pytest.mark.unit
def test_ik_failure_stops_before_publishing_that_frame(monkeypatch):
    clock = _FakeClock()
    hardware = _FakeIkHardware(clock, ik_fail_at=1)

    result = _run(hardware, _params(), monkeypatch)

    assert not result.success
    assert "IK 无连续可行解" in result.message
    assert len(hardware.setpoints) == 1, "求解失败的帧不能被下发"


# ----------------------------------------------------------------------
# 单/双臂模式
# ----------------------------------------------------------------------
@pytest.mark.unit
def test_both_mode_moves_both_arms(monkeypatch):
    clock = _FakeClock()
    hardware = _FakeIkHardware(clock)

    result = _run(hardware, _params(arms="both"), monkeypatch)

    assert result.success, result.message
    first, last = hardware.setpoints[0], hardware.setpoints[-1]
    assert first[:7] != pytest.approx(last[:7])
    assert first[7:] != pytest.approx(last[7:])


@pytest.mark.unit
@pytest.mark.parametrize("arms", ["left", "right"])
def test_single_arm_mode_keeps_unselected_arm_still(arms, monkeypatch):
    clock = _FakeClock()
    hardware = _FakeIkHardware(clock)

    result = _run(hardware, _params(arms=arms), monkeypatch)

    assert result.success, result.message
    still = [math.degrees(value) for value in (INITIAL[7:] if arms == "left" else INITIAL[:7])]
    moving_offset = 0 if arms == "left" else 7
    for setpoint in hardware.setpoints:
        held = setpoint[7:] if arms == "left" else setpoint[:7]
        assert held == pytest.approx(still), "未选臂必须保持当前角度不动"
    first, last = hardware.setpoints[0], hardware.setpoints[-1]
    assert first[moving_offset:moving_offset + 7] != pytest.approx(
        last[moving_offset:moving_offset + 7])
