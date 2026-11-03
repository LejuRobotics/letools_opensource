#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""底盘伺服阶段选择节点测试（编排层，硬件节点，伪硬件注入）。

被测对象: orchestration.nodes.node_chassis_servo_phase.NodeChassisServoPhase
节点生命周期: initialise() → 循环 update() → RUNNING（源节点口径，从不收口）
底层路径: 节点 get_shared_hardware() → enable_vel_control_jibot（权限）
                                    → send_base_velocity_tick（零速兜底）
测试策略（模块单独测试，不依赖真机 / ROS 服务）:
    monkeypatch 节点模块的 get_shared_hardware，注入伪硬件记录权限调用序列；
    只往黑板写 AlignObservation、读回节点写的 servo_phase，验证两阶段判决：
    - |theta| 连续 N 帧小于阈值才交棒（防抖）
    - 交棒后单向闩锁，theta 漂了也不回退
    - 权限开/关两条路径都在本节点内，不靠场景另配 NodeEnableVelControl

测试用例说明:
- test_dry_run: STUDIO_DRY_RUN=1 时 update() 短路 RUNNING
- test_initial_phase_written: 首 tick 把阶段初值写进黑板
- test_theta_above_threshold_stays_rotate: 角度未收敛维持 rotate
- test_theta_converges_after_n_frames: 连续 N 帧满足才交棒（单帧不够）
- test_noise_resets_frame_count: 中途一帧不满足就清零重数（防抖）
- test_same_observation_ticked_twice_counts_once: 同一条观测重复 tick 只计一次（按版本计时）
- test_latches_after_translate: 交棒后 theta 再漂也不回退（单向闩锁）
- test_missing_observation_running: 无观测时 RUNNING 且不交棒
- test_invalid_observation_running: 观测无效时 RUNNING 且不交棒
- test_illegal_initial_phase_falls_back: initial_phase 拼错退回 rotate
- test_opens_and_restores_vel_control: initialise 开权限 / terminate 还原（均先发零速）
- test_own_vel_control_false_skips_hardware: own_vel_control=false 完全不碰硬件

运行:
    python3 apps/test_kuavo_5w_orchestration/test_chassis_servo_phase_node.py
退出码 0 通过、1 失败。
"""
import sys
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(project_root))
sys.path.insert(0, str(project_root / "apps"))

from core.common.logger import init_logging, get_logger
init_logging()

from py_trees.blackboard import Client
from py_trees.common import Access, Status

from core.domain.result import Result
from core.domain.servo_types import AlignObservation
from orchestration.nodes.node_chassis_servo_phase import NodeChassisServoPhase
import orchestration.nodes.node_chassis_servo_phase as phase_mod
from _blackboard_isolation import reset_blackboard

logger = get_logger(__name__)

OBS_KEY = "latest_servo_error"
OBS_VER_KEY = "latest_servo_error_version"
PHASE_KEY = "servo_phase"
ROTATE = "rotate"
TRANSLATE = "translate"

THRESHOLD = 0.09
FRAMES = 2

# 观测版本号计数器：节点按**版本**推进连续帧计数（不按 tick），故每次写观测都要
# 连带涨版本，否则节点视为"同一条观测"直接返回。用模块级计数器模拟上游视觉模块。
_VER = [0]


class _FakeHardware:
    """伪硬件：记录权限切换与零速发布的调用序列。"""

    def __init__(self):
        self.enable_calls = []      # enable_vel_control_jibot 的入参序列
        self.vel_calls = []         # send_base_velocity_tick 的入参序列

    def enable_vel_control_jibot(self, enable):
        self.enable_calls.append(enable)
        return Result.ok(f"enable_vel_control({enable})", data={"state_after": enable})

    def send_base_velocity_tick(self, vx, vy, vyaw_rad):
        self.vel_calls.append((float(vx), float(vy), float(vyaw_rad)))
        return Result.ok("velocity tick")


class _PatchHw:
    """注入伪硬件（with 块），并记录本节点对权限的完整调用序列。

    节点在 initialise() 里开权限、terminate() 里还原，没有伪硬件就会去碰真硬件。
    """

    def __init__(self):
        self.fake = _FakeHardware()

    def __enter__(self):
        self._orig = phase_mod.get_shared_hardware
        phase_mod.get_shared_hardware = lambda: self.fake
        return self.fake

    def __exit__(self, *exc):
        phase_mod.get_shared_hardware = self._orig
        return False


def _obs(theta_rad=0.0, valid=True, reason=""):
    """构造一个 AlignObservation（只关心偏差角）。"""
    return AlignObservation(
        e_bottom_px=0.0, e_right_px=0.0, theta_rad=theta_rad,
        valid=valid, reject=reason,
    )


class _Writer:
    """模拟视觉感知模块：写观测的**同时**涨版本号。

    节点按观测版本推进连续帧计数（不按 tick），所以测试每"喂一条观测"都必须连带
    涨版本 —— 只写值不涨版本，节点会当成同一条观测重复 tick、直接返回不计数。
    接口按 `writer.set(key, value)` 提供，与 py_trees Client 一致。
    """

    def __init__(self, name):
        self._client = Client(name=name)
        self._client.register_key(key=OBS_KEY, access=Access.WRITE)
        self._client.register_key(key=OBS_VER_KEY, access=Access.WRITE)

    def set(self, key, value):
        self._client.set(key, value)
        if key == OBS_KEY:
            _VER[0] += 1
            self._client.set(OBS_VER_KEY, _VER[0])


def _make_node(initial_phase=ROTATE, threshold=THRESHOLD, frames=FRAMES,
               own_vel_control=False, settle_s=0.0):
    """构造阶段节点。

    `settle_s=0`：不真 sleep（权限层已有自己的 sleep）。
    `own_vel_control` 默认 **False**：大部分用例只验阶段判决，不想让 initialise()
    去碰硬件。两个权限用例显式传 True。
    """
    _reset_blackboard()
    return NodeChassisServoPhase(
        "t_servo_phase", "底盘伺服阶段选择", "ns",
        {"phase_key": PHASE_KEY, "obs_key": OBS_KEY,
         "initial_phase": initial_phase,
         "theta_threshold_rad": str(threshold), "converge_frames": str(frames),
         "own_vel_control": str(own_vel_control), "settle_s": str(settle_s)},
    )


def _make_writer(name="test_phase_writer"):
    """writer：模拟视觉感知模块写 AlignObservation + _version。"""
    return _Writer(name)


def _make_reader(name="test_phase_reader"):
    """reader client：读节点写回的阶段键。"""
    r = Client(name=name)
    r.register_key(key=PHASE_KEY, access=Access.READ)
    return r


def _read_phase(reader):
    try:
        return getattr(reader, PHASE_KEY)
    except KeyError:
        return None


def _reset_blackboard():
    """把阶段键复位成 rotate、观测清空，并把版本号计数器归零。

    黑板是进程内全局单例，跨用例不隔离：上一个用例交棒后写下的 `translate` 会被
    下一个用例首 tick 读到（节点首 tick 信黑板），于是它直接闩锁、用例互相污染。
    同 test_chassis_servo_mode_node.py 记录的那条教训，每个用例开头必须先复位。
    """
    w = Client(name=f"test_phase_reset_{id(object())}")
    w.register_key(key=PHASE_KEY, access=Access.WRITE)
    w.set(PHASE_KEY, ROTATE)
    w.register_key(key=OBS_KEY, access=Access.WRITE)
    w.set(OBS_KEY, None)
    w.register_key(key=OBS_VER_KEY, access=Access.WRITE)
    w.set(OBS_VER_KEY, 0)
    _VER[0] = 0


def test_dry_run():
    """STUDIO_DRY_RUN=1 时 update() 短路 RUNNING（不碰黑板）。"""
    import orchestration.nodes.node_chassis_servo_phase as phase_mod
    orig = phase_mod._DRY_RUN
    phase_mod._DRY_RUN = True
    try:
        node = _make_node()
        node.initialise()
        assert node.update() == Status.RUNNING, "dry-run 应 RUNNING"
        logger.info("    dry_run 短路 RUNNING")
    finally:
        phase_mod._DRY_RUN = orig


def test_initial_phase_written():
    """首 tick 把阶段初值写进黑板（下游 calc 才有得读）。"""
    node = _make_node()
    node.initialise()
    reader = _make_reader("r_init")
    assert node.update() == Status.RUNNING
    assert _read_phase(reader) == ROTATE, \
        f"首 tick 应写下 {ROTATE}，得到 {_read_phase(reader)}"
    logger.info("    首 tick 写下初始阶段 rotate")


def test_theta_above_threshold_stays_rotate():
    """角度未收敛（|theta| > 阈值）维持 rotate，且计数不涨。"""
    node = _make_node()
    node.initialise()
    writer = _make_writer("w_above")
    reader = _make_reader("r_above")
    for _ in range(5):
        writer.set(OBS_KEY, _obs(theta_rad=0.5))
        assert node.update() == Status.RUNNING
    assert _read_phase(reader) == ROTATE, \
        f"角度未收敛应维持 {ROTATE}，得到 {_read_phase(reader)}"
    assert node._theta_frames == 0, f"未满足阈值不该计数，得到 {node._theta_frames}"
    logger.info("    角度未收敛维持 rotate")


def test_theta_converges_after_n_frames():
    """连续 N 帧满足阈值才交棒 —— 单帧不够（防抖）。"""
    node = _make_node(frames=2)
    node.initialise()
    writer = _make_writer("w_conv")
    reader = _make_reader("r_conv")

    writer.set(OBS_KEY, _obs(theta_rad=0.05))     # 第 1 帧满足
    assert node.update() == Status.RUNNING
    assert _read_phase(reader) == ROTATE, \
        f"仅 1/{FRAMES} 帧不该交棒，得到 {_read_phase(reader)}"

    writer.set(OBS_KEY, _obs(theta_rad=0.04))     # 第 2 帧满足 → 交棒
    assert node.update() == Status.RUNNING
    assert _read_phase(reader) == TRANSLATE, \
        f"连续 {FRAMES} 帧满足应交棒 {TRANSLATE}，得到 {_read_phase(reader)}"
    logger.info("    连续 2 帧满足才交棒")


def test_noise_resets_frame_count():
    """中途一帧不满足就清零重数 —— 视觉噪声里一闪而过不该提前交棒。"""
    node = _make_node(frames=2)
    node.initialise()
    writer = _make_writer("w_noise")
    reader = _make_reader("r_noise")

    writer.set(OBS_KEY, _obs(theta_rad=0.05))     # 满足 1
    node.update()
    writer.set(OBS_KEY, _obs(theta_rad=0.5))      # 不满足 → 清零
    node.update()
    assert node._theta_frames == 0, f"不满足帧应清零，得到 {node._theta_frames}"
    writer.set(OBS_KEY, _obs(theta_rad=0.05))     # 又满足 1，仍不够
    node.update()
    assert _read_phase(reader) == ROTATE, \
        f"计数被清零后不该交棒，得到 {_read_phase(reader)}"
    logger.info("    噪声帧清零计数、不提前交棒")


def test_latches_after_translate():
    """交棒后单向闩锁：theta 再漂出阈值也不回退。

    断言两条：阶段键仍是 translate（**不能只断这条** —— 阶段机只会
    rotate→translate 单向迁移，就算去掉闩锁的短路分支键也不会退回，
    弱断言区分不出来），以及 feedback 不再自称「维持 rotate」。
    """
    node = _make_node(frames=1)
    node.initialise()
    writer = _make_writer("w_latch")
    reader = _make_reader("r_latch")

    writer.set(OBS_KEY, _obs(theta_rad=0.02))
    node.update()
    assert _read_phase(reader) == TRANSLATE, "1 帧阈值下应交棒"

    for _ in range(5):
        writer.set(OBS_KEY, _obs(theta_rad=0.8))   # 大幅漂出阈值
        node.update()
    assert _read_phase(reader) == TRANSLATE, \
        f"单向闩锁：不该退回 {ROTATE}，得到 {_read_phase(reader)}"
    assert ROTATE not in node.feedback_message, \
        f"闩锁后不该再自称维持 {ROTATE}，得到：{node.feedback_message}"
    logger.info("    交棒后单向闩锁、不回退")


def test_same_observation_ticked_twice_counts_once():
    """同一条观测被 tick 多次只计一次 —— 连续帧计数按**观测版本**而非 tick。

    主循环 ~50Hz 而真机视觉 ~10Hz，同一条观测会被 tick 三五次。若按 tick 计数，
    「连续 2 帧」实际只覆盖 40ms、连一条观测的寿命都不到，防抖形同虚设。
    """
    node = _make_node(frames=2)
    node.initialise()
    writer = _make_writer("w_version")
    reader = _make_reader("r_version")

    writer.set(OBS_KEY, _obs(theta_rad=0.05))     # 观测 #1，满足
    assert node.update() == Status.RUNNING
    assert node._theta_frames == 1, f"第 1 条观测应计数 1，得到 {node._theta_frames}"

    # 版本没变，再 tick 三次（模拟主循环高频 tick 同一条观测）
    for _ in range(3):
        writer._client.set(OBS_KEY, _obs(theta_rad=0.05))
        assert node.update() == Status.RUNNING
    assert node._theta_frames == 1, \
        f"同一条观测重复 tick 不该涨计数，得到 {node._theta_frames}"
    assert _read_phase(reader) == ROTATE, "不该仅凭一条观测就交棒"

    writer.set(OBS_KEY, _obs(theta_rad=0.04))     # 观测 #2 → 才交棒
    node.update()
    assert _read_phase(reader) == TRANSLATE, \
        f"第 2 条观测满足后应交棒，得到 {_read_phase(reader)}"
    logger.info("    同一条观测重复 tick 只计一次（按版本计数）")


def test_missing_observation_running():
    """黑板无观测时 RUNNING（源节点等数据），仍写下初始阶段。"""
    node = _make_node()
    node.initialise()
    reader = _make_reader("r_missing")
    assert node.update() == Status.RUNNING
    assert _read_phase(reader) == ROTATE, "无观测也该先写下初始阶段"
    assert node._theta_frames == 0, "无观测不该计数"
    logger.info("    无观测维持 RUNNING、不交棒")


def test_invalid_observation_running():
    """观测无效时 RUNNING 且不交棒（零速兜底归 exec）。"""
    node = _make_node(frames=1)
    node.initialise()
    writer = _make_writer("w_invalid")
    reader = _make_reader("r_invalid")
    for _ in range(3):
        writer.set(OBS_KEY, _obs(theta_rad=0.01, valid=False, reason="no_edge"))
        assert node.update() == Status.RUNNING
    assert _read_phase(reader) == ROTATE, \
        f"无效观测不该交棒，得到 {_read_phase(reader)}"
    logger.info("    无效观测维持 RUNNING、不交棒")


def test_illegal_initial_phase_falls_back():
    """initial_phase 拼错（如 transalte）退回 rotate，不静默用错阶段。"""
    node = _make_node(initial_phase="transalte")
    assert node._initial_phase == ROTATE, \
        f"非法初值应退回 {ROTATE}，得到 {node._initial_phase}"
    logger.info("    非法 initial_phase 退回 rotate")


def test_opens_and_restores_vel_control():
    """权限开关归本节点：initialise 开(True)+发零速，terminate 还原(False)+发零速。

    这是 2026-10-15 21:23 那次故障（模式说 velocity、权限始终 False、底盘不动）
    的正面修复 —— 开门动作必须与阶段判决在同一个节点里，不能靠场景另配
    NodeEnableVelControl。
    """
    with _PatchHw() as fake:
        node = _make_node(own_vel_control=True)
        node.initialise()
        assert fake.enable_calls == [True], \
            f"initialise 应开权限 True，得到 {fake.enable_calls}"
        assert fake.vel_calls and fake.vel_calls[0] == (0.0, 0.0, 0.0), \
            f"开权限前应先发零速，得到 {fake.vel_calls}"

        node.terminate(Status.SUCCESS)
        assert fake.enable_calls[-1] is False, \
            f"terminate 应还原权限 False，得到 {fake.enable_calls}"
        assert fake.vel_calls[-1] == (0.0, 0.0, 0.0), \
            f"terminate 应再发一次零速，得到 {fake.vel_calls[-1]}"
    logger.info("    initialise 开权限 / terminate 还原（均先发零速）")


def test_own_vel_control_false_skips_hardware():
    """own_vel_control=false 时本节点完全不碰硬件（权限留给别的节点）。"""
    with _PatchHw() as fake:
        node = _make_node(own_vel_control=False)
        node.initialise()
        node.terminate(Status.SUCCESS)
        assert fake.enable_calls == [], \
            f"不该碰权限，得到 {fake.enable_calls}"
        assert fake.vel_calls == [], f"不该发零速，得到 {fake.vel_calls}"
    logger.info("    own_vel_control=false 完全不碰硬件")


def main() -> int:
    cases = [
        ("dry_run 短路", test_dry_run),
        ("首 tick 写初始阶段", test_initial_phase_written),
        ("角度未收敛维持 rotate", test_theta_above_threshold_stays_rotate),
        ("连续 N 帧满足才交棒", test_theta_converges_after_n_frames),
        ("噪声帧清零计数", test_noise_resets_frame_count),
        ("同一条观测重复 tick 只计一次", test_same_observation_ticked_twice_counts_once),
        ("交棒后单向闩锁", test_latches_after_translate),
        ("无观测保持 RUNNING", test_missing_observation_running),
        ("无效观测保持 RUNNING", test_invalid_observation_running),
        ("非法 initial_phase 退回", test_illegal_initial_phase_falls_back),
        ("initialise 开权限 / terminate 还原", test_opens_and_restores_vel_control),
        ("own_vel_control=false 不碰硬件", test_own_vel_control_false_skips_hardware),
    ]
    failed = 0
    # 节点 initialise() 会碰权限（去 get_shared_hardware），整个用例集统一注入伪硬件；
    # 两个硬件用例在内部再套一层 _PatchHw（保存/恢复外层），互不干扰。
    outer = _PatchHw()
    outer.__enter__()
    try:
        for title, fn in cases:
            reset_blackboard()
            try:
                fn()
            except AssertionError as exc:
                failed += 1
                logger.error("  FAIL  %s: %s", title, exc)
            except Exception as exc:                     # noqa: BLE001
                failed += 1
                logger.error("  ERROR %s: %s: %s", title, type(exc).__name__, exc)
    finally:
        outer.__exit__()

    logger.info("")
    if failed:
        logger.error("FAILED: %d/%d", failed, len(cases))
        return 1
    logger.info("OK: %d/%d 全部通过 —— 底盘伺服阶段选择（纯计算节点）测试完成",
                len(cases), len(cases))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())