# -*- coding: utf-8 -*-
"""码垛直线验收入口的单臂选择与收尾还原（无硬件、无 ROS、不运动）。

守的是 DOCS-155 的三条约定：

1. ``--arms`` 从命令行一路传到技能参数、TF 采样范围与指标判据；
2. 收尾只把**选中**手臂放回启动前角度，未选手臂保持当前角度不动；
3. 启动前 / 还原前读不到关节角必须明确报错，不能降级成一句警告后继续。

入口脚本本身会下发运动指令，这里只 import 模块、调纯函数与假硬件驱动 ``main()``
（硬件、技能、采样器、日志全部替换为内存替身），不触碰真实机器人。
"""

import logging
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from apps.test_kuavo_5w_skills import _scaffold  # noqa: E402
from apps.test_kuavo_5w_skills import test_palletizing_line_ik as ik_entry  # noqa: E402
from apps.test_kuavo_5w_skills import test_palletizing_line_mpc as mpc_entry  # noqa: E402
from apps.test_kuavo_5w_skills._eval.line_metrics import LineMetricsRecorder  # noqa: E402
from core.domain.result import Result  # noqa: E402
from core.domain.transform import Transform  # noqa: E402

pytestmark = pytest.mark.unit

ENTRIES = [pytest.param(mpc_entry, id='mpc'), pytest.param(ik_entry, id='ik')]
# main() 用到的技能类名按入口不同，装配替身时按这个字段取。
ENTRY_MAINS = [
    pytest.param(mpc_entry, 'PalletizingLineMpcSkill', id='mpc'),
    pytest.param(ik_entry, 'PalletizingLineIkSkill', id='ik'),
]
SELECTIONS = [
    pytest.param('both', ['left', 'right'], id='both'),
    pytest.param('left', ['left'], id='left'),
    pytest.param('right', ['right'], id='right'),
]

# 启动前 / 当前关节角故意取成两套明显不同的数，才能看出「还原了谁」。
INITIAL = [float(index) * 0.1 for index in range(14)]
CURRENT = [100.0 + float(index) for index in range(14)]


class FakeJointHardware:
    """只实现入口用到的关节读取 / 整组下发。"""

    def __init__(self, current, read_ok=True, send_ok=True):
        self.current = list(current)
        self.read_ok = read_ok
        self.send_ok = send_ok
        self.sent = None
        self.time_sec = None

    def get_arm_joint_positions(self):
        if not self.read_ok:
            return Result.fail("模拟读取失败")
        return Result.ok(data=list(self.current))

    def send_arm_joint_trajectory(self, joints_deg, time_sec=0.0):
        self.sent = list(joints_deg)
        self.time_sec = time_sec
        if not self.send_ok:
            return Result.fail("模拟下发失败")
        return Result.ok()


class FakeTfHardware:
    """每次 TF 查询都返回同一个位姿，并记录查询过的坐标系。"""

    def __init__(self):
        self.queried = []

    def get_transform(self, world_frame, frame, timeout):
        self.queried.append(frame)
        return Result.ok(data=Transform.from_lists(
            [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]))


class FakeSkill:
    """采样线程需要的最小技能接口：当前段 + 段端点。"""

    current_segment = SimpleNamespace(name='seg1', kind='horizontal')

    def current_segment_bounds(self, arm):
        return (0.0, 0.0, 0.0), (0.1, 0.0, 0.0)


# ----------------------------------------------------------------------
# 1. 参数传递
# ----------------------------------------------------------------------
@pytest.mark.parametrize('argv, expected', [
    ([], 'both'),
    (['--arms', 'both'], 'both'),
    (['--arms', 'left'], 'left'),
    (['--arms', 'right'], 'right'),
])
def test_parse_arms_arg_supports_both_and_single_arm(argv, expected):
    assert _scaffold.parse_arms_arg(argv) == expected


def test_parse_arms_arg_rejects_unknown_selection():
    with pytest.raises(SystemExit):
        _scaffold.parse_arms_arg(['--arms', 'middle'])


@pytest.mark.parametrize('selection, arms', SELECTIONS)
def test_selected_arms_maps_selection_to_sides(selection, arms):
    assert _scaffold.selected_arms(selection) == arms


def test_selected_arms_rejects_unknown_selection():
    with pytest.raises(ValueError):
        _scaffold.selected_arms('middle')


@pytest.mark.parametrize('entry', ENTRIES)
@pytest.mark.parametrize('selection, arms', SELECTIONS)
def test_params_carry_selection(entry, selection, arms):
    """MPC / IK 入口都把 --arms 原样交给技能参数。"""
    params = entry.build_params(selection)
    assert params.arms == selection
    assert _scaffold.selected_arms(selection) == arms


@pytest.mark.parametrize('entry', ENTRIES)
def test_default_selection_keeps_both_arms(entry):
    """不带参数时与改造前一致：双臂。"""
    assert entry.build_params(_scaffold.parse_arms_arg([])).arms == 'both'


# ----------------------------------------------------------------------
# 2. 采样与判据范围
# ----------------------------------------------------------------------
@pytest.mark.parametrize('entry', ENTRIES)
@pytest.mark.parametrize('selection, arms', SELECTIONS)
def test_sampling_only_touches_selected_arms_and_stats(entry, selection, arms):
    ee_frames = _scaffold.selected_ee_frames(entry.EE_FRAMES, arms)
    assert list(ee_frames) == arms

    hardware = FakeTfHardware()
    recorder = LineMetricsRecorder(
        hardware, FakeSkill(), entry.WORLD_FRAME, arms, ee_frames, rate=50.0)
    recorder._sample_once()

    assert hardware.queried == [entry.EE_FRAMES[arm] for arm in arms]
    assert {arm for (_stage, arm) in recorder.stats} == set(arms)


@pytest.mark.parametrize('entry', ENTRIES)
@pytest.mark.parametrize('selection, arms', SELECTIONS)
def test_evaluate_never_blames_unselected_arm(entry, selection, arms):
    """未选手臂没有样本，也不能被判成「样本不足」。"""
    ee_frames = _scaffold.selected_ee_frames(entry.EE_FRAMES, arms)
    recorder = LineMetricsRecorder(
        FakeTfHardware(), FakeSkill(), entry.WORLD_FRAME, arms, ee_frames, rate=50.0)
    recorder._sample_once()

    failures = entry.evaluate(recorder)
    assert failures, "单帧样本必然不足，这里只验归属，不验数值"
    for item in failures:
        assert any(arm in item for arm in arms), item


# ----------------------------------------------------------------------
# 3. 启动前读取
# ----------------------------------------------------------------------
def test_capture_returns_all_14_joints():
    assert _scaffold.capture_arm_joints(FakeJointHardware(INITIAL)) == INITIAL


def test_capture_raises_when_read_fails():
    with pytest.raises(RuntimeError, match="读取启动前手臂关节角失败"):
        _scaffold.capture_arm_joints(FakeJointHardware(INITIAL, read_ok=False))


def test_capture_raises_when_joint_count_is_wrong():
    with pytest.raises(RuntimeError, match="14"):
        _scaffold.capture_arm_joints(FakeJointHardware([0.0] * 7))


# ----------------------------------------------------------------------
# 4. 收尾还原：只替换选中 7 关节
# ----------------------------------------------------------------------
@pytest.mark.parametrize('selection, arms', SELECTIONS)
def test_restore_replaces_only_selected_arm(selection, arms):
    hardware = FakeJointHardware(CURRENT)

    assert _scaffold.restore_selected_arms(hardware, INITIAL, arms, 5.0) is True

    left_source = INITIAL if 'left' in arms else CURRENT
    right_source = INITIAL if 'right' in arms else CURRENT
    expected_left = [math.degrees(value) for value in left_source[:7]]
    expected_right = [math.degrees(value) for value in right_source[7:]]

    assert hardware.sent[:7] == pytest.approx(expected_left)
    assert hardware.sent[7:] == pytest.approx(expected_right)
    # 未选手臂送的是「当前角度」，不是启动前角度——两套数不同才说明确实替换了。
    if 'left' not in arms:
        assert hardware.sent[:7] != pytest.approx(
            [math.degrees(value) for value in INITIAL[:7]])
    assert hardware.time_sec == 5.0


def test_restore_reports_error_when_prerestore_read_fails(caplog):
    hardware = FakeJointHardware(CURRENT, read_ok=False)

    with caplog.at_level(logging.ERROR):
        restored = _scaffold.restore_selected_arms(hardware, INITIAL, ['left'], 5.0)

    assert restored is False
    assert hardware.sent is None
    assert "还原前读取当前关节角失败" in caplog.text


def test_restore_returns_false_without_captured_joints():
    hardware = FakeJointHardware(CURRENT)

    assert _scaffold.restore_selected_arms(hardware, [], ['left'], 5.0) is False
    assert hardware.sent is None


def test_restore_returns_false_when_send_fails():
    hardware = FakeJointHardware(CURRENT, send_ok=False)

    assert _scaffold.restore_selected_arms(hardware, INITIAL, ['left'], 5.0) is False


def test_restore_rejects_unknown_arm_name(caplog):
    """拼错臂名时宁可不发指令，也不能把左臂的命令值写进右臂。"""
    hardware = FakeJointHardware(CURRENT)

    with caplog.at_level(logging.ERROR):
        restored = _scaffold.restore_selected_arms(hardware, INITIAL, ['Left'], 5.0)

    assert restored is False
    assert hardware.sent is None
    assert "未知手臂名" in caplog.text


# ----------------------------------------------------------------------
# 5. 主入口装配：用替身完整驱动 main()
# ----------------------------------------------------------------------
class FakeEntryHardware:
    """main() 用到的硬件面：初始化、关节读写、MPC 前置、关停。"""

    def __init__(self, events, initial=INITIAL, current=CURRENT,
                 fail_reads=(), send_ok=True):
        self.events = events
        self.initial = list(initial)
        self.current = list(current)
        self.fail_reads = set(fail_reads)
        self.send_ok = send_ok
        self.read_calls = 0
        self.sent = None
        self.time_sec = None

    def initialize(self):
        self.events.append('initialize')

    def shutdown(self):
        self.events.append('shutdown')

    def get_arm_joint_positions(self):
        self.read_calls += 1
        self.events.append('read')
        if self.read_calls in self.fail_reads:
            return Result.fail("模拟读取失败")
        # 第一次读是启动前基准，之后是还原前的当前角度。
        return Result.ok(data=list(self.initial if self.read_calls == 1 else self.current))

    def send_arm_joint_trajectory(self, joints_deg, time_sec=0.0):
        self.events.append('send')
        self.sent = list(joints_deg)
        self.time_sec = time_sec
        return Result.ok() if self.send_ok else Result.fail("模拟下发失败")

    # MPC 入口的前置；IK 入口不调用。
    def set_mpc_mode(self, mode):
        self.events.append('set_mpc_mode')
        return Result.ok()

    def set_focus_ee(self, value):
        return Result.ok()

    def set_focus_z(self, value):
        return Result.ok()


class FakeSkillForMain:
    """main() 只把技能对象透传给采样器，这里留个可辨识的载体。"""

    def __init__(self, hardware=None):
        self.hardware = hardware


class FakeRecorder:
    """替代后台采样器：不启线程，统计为空即判据通过，同时留下构造入参。"""

    def __init__(self, hardware, skill, world_frame, arms, ee_frames, rate=50.0):
        self.hardware = hardware
        self.skill = skill
        self.world_frame = world_frame
        self.arms = list(arms)
        self.ee_frames = dict(ee_frames)
        self.rate = rate
        self.stats = {}

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        return False

    def summary(self):
        return []

    def perpendicularity(self, box_frame, tool_axis, box_axis):
        return None


def run_main(entry, skill_attr, monkeypatch, selection, initial=INITIAL,
             current=CURRENT, fail_reads=(), send_ok=True, run_ok=True):
    """用内存替身完整驱动一次 ``entry.main()``，返回可断言的观测结果。"""
    observed = SimpleNamespace(events=[], recorders=[], params=[], restore_calls=[],
                               teardowns=[], exit_code=None)
    hardware = FakeEntryHardware(observed.events, initial=initial, current=current,
                                 fail_reads=fail_reads, send_ok=send_ok)

    def fake_build_hardware(**kwargs):
        observed.events.append('build_hardware')
        return hardware

    def fake_skill_setup(hw, need_arm=False, need_torso_reset=True):
        observed.events.append('skill_setup')

    def fake_skill_teardown(hw, need_arm=False):
        observed.events.append('skill_teardown')
        observed.teardowns.append(need_arm)

    def fake_run_skill(skill, params, tick_interval=0.05):
        observed.events.append('run_skill')
        observed.params.append(params)
        return run_ok

    def fake_recorder(hw, skill, world_frame, arms, ee_frames, rate=50.0):
        observed.events.append('recorder')
        observed.recorders.append(
            FakeRecorder(hw, skill, world_frame, arms, ee_frames, rate))
        return observed.recorders[-1]

    def fake_skill_factory(**kwargs):
        observed.events.append('build_skill')
        return FakeSkillForMain(**kwargs)

    def spy_restore(hw, initial_joints, arms, time_sec):
        observed.restore_calls.append(SimpleNamespace(
            initial=list(initial_joints), arms=list(arms), time_sec=time_sec))
        # 仍然走真实实现，才能断言最终下发的 14 个值。
        return _scaffold.restore_selected_arms(hw, initial_joints, arms, time_sec)

    # 真实 argparse 也一并跑，避免替身把 --arms 的解析行为一起替掉。
    monkeypatch.setattr(sys, 'argv', ['test_palletizing_line', '--arms', selection])
    monkeypatch.setattr(entry, 'build_hardware', fake_build_hardware)
    monkeypatch.setattr(entry, 'skill_setup', fake_skill_setup)
    monkeypatch.setattr(entry, 'skill_teardown', fake_skill_teardown)
    monkeypatch.setattr(entry, 'run_skill', fake_run_skill)
    monkeypatch.setattr(entry, 'LineMetricsRecorder', fake_recorder)
    monkeypatch.setattr(entry, skill_attr, fake_skill_factory)
    monkeypatch.setattr(entry, 'restore_selected_arms', spy_restore)

    try:
        entry.main()
    except SystemExit as exc:
        observed.exit_code = exc.code
    observed.hardware = hardware
    return observed


@pytest.mark.parametrize('entry, skill_attr', ENTRY_MAINS)
@pytest.mark.parametrize('selection, arms', SELECTIONS)
def test_main_wires_selection_and_restores_only_selected_arm(
        entry, skill_attr, selection, arms, monkeypatch, caplog):
    """main() 的装配：读基准在建技能之前、参数/采样范围、以及还原实际下发值。"""
    caplog.set_level(logging.INFO)

    run = run_main(entry, skill_attr, monkeypatch, selection)

    # 启动前关节读取必须发生在构建技能之前——没有还原基准就不能开始动。
    assert run.events.index('read') < run.events.index('build_skill')
    # --arms 原样进技能参数。
    assert [params.arms for params in run.params] == [selection]
    # 采样范围与 ee_frames 只含选中的臂。
    recorder = run.recorders[0]
    assert recorder.arms == arms
    assert recorder.ee_frames == {arm: entry.EE_FRAMES[arm] for arm in arms}
    assert recorder.world_frame == entry.WORLD_FRAME
    assert recorder.rate == entry.SAMPLE_RATE
    # 还原拿到的是正确的 arms / 基准 / 时长。
    assert len(run.restore_calls) == 1
    assert run.restore_calls[0].arms == arms
    assert run.restore_calls[0].initial == INITIAL
    assert run.restore_calls[0].time_sec == entry.RESTORE_TIME_SEC

    # 实际下发值：选中臂 = 启动前角度，未选臂 = 当前角度（弧度 → 度）。
    sent = run.hardware.sent
    left_source = INITIAL if 'left' in arms else CURRENT
    right_source = INITIAL if 'right' in arms else CURRENT
    assert sent[:7] == pytest.approx([math.degrees(value) for value in left_source[:7]])
    assert sent[7:] == pytest.approx([math.degrees(value) for value in right_source[7:]])
    assert run.hardware.time_sec == entry.RESTORE_TIME_SEC
    if selection == 'both':
        # 双臂保持改造前的旧语义：整组回到启动前角度。
        assert sent == pytest.approx([math.degrees(value) for value in INITIAL])

    # 还原成功 → 不需要 teardown 兜底，验收正常通过、退出码 0（不抛 SystemExit）。
    assert run.teardowns == [False]
    assert run.exit_code is None
    assert "验收通过" in caplog.text
    assert "验收未通过" not in caplog.text


@pytest.mark.parametrize('entry, skill_attr', ENTRY_MAINS)
@pytest.mark.parametrize('selection, arms', SELECTIONS)
def test_main_restores_selected_arms_when_skill_fails(
        entry, skill_attr, selection, arms, monkeypatch, caplog):
    """搬运中途失败（含逐帧设定点发布失败）也要走还原，选中臂回到启动前角度。"""
    caplog.set_level(logging.INFO)

    run = run_main(entry, skill_attr, monkeypatch, selection, run_ok=False)

    assert run.restore_calls[0].arms == arms
    left_source = INITIAL if 'left' in arms else CURRENT
    right_source = INITIAL if 'right' in arms else CURRENT
    assert run.hardware.sent[:7] == pytest.approx(
        [math.degrees(value) for value in left_source[:7]])
    assert run.hardware.sent[7:] == pytest.approx(
        [math.degrees(value) for value in right_source[7:]])
    assert run.exit_code == 1
    assert "验收通过" not in caplog.text


@pytest.mark.parametrize('entry, skill_attr', ENTRY_MAINS)
def test_main_aborts_without_motion_when_initial_capture_fails(
        entry, skill_attr, monkeypatch, caplog):
    """读不到启动前角度就不搬运，并走 teardown 安全兜底。"""
    run = run_main(entry, skill_attr, monkeypatch, 'left', fail_reads={1})

    assert 'build_skill' not in run.events
    assert 'run_skill' not in run.events
    assert 'send' not in run.events
    assert run.hardware.sent is None
    assert run.teardowns == [True]
    assert run.exit_code == 1
    assert "取消本次验收，未下发任何搬运指令" in caplog.text
    assert "验收通过" not in caplog.text


@pytest.mark.parametrize('entry, skill_attr', ENTRY_MAINS)
def test_main_fails_when_restore_read_fails(entry, skill_attr, monkeypatch, caplog):
    """主流程成功但还原前读不到角度：兜底复位 + 结论判失败，不许再报「通过」。"""
    caplog.set_level(logging.INFO)

    run = run_main(entry, skill_attr, monkeypatch, 'left', fail_reads={2})

    assert run.hardware.sent is None
    assert run.teardowns == [True]
    assert run.exit_code == 1
    assert "还原前读取当前关节角失败" in caplog.text
    assert "验收未通过" in caplog.text
    assert "验收通过" not in caplog.text


@pytest.mark.parametrize('entry, skill_attr', ENTRY_MAINS)
def test_main_fails_when_restore_send_fails(entry, skill_attr, monkeypatch, caplog):
    """主流程成功但还原指令下发失败：同样判失败。"""
    caplog.set_level(logging.INFO)

    run = run_main(entry, skill_attr, monkeypatch, 'both', send_ok=False)

    assert run.hardware.sent is not None, "下发被尝试过，只是返回失败"
    assert run.teardowns == [True]
    assert run.exit_code == 1
    assert "恢复选中手臂关节角失败" in caplog.text
    assert "验收未通过" in caplog.text
    assert "验收通过" not in caplog.text
