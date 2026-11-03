#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""底盘速度控制权限切换节点测试（编排层，硬件节点，伪硬件注入）。

被测对象: orchestration.nodes.node_enable_vel_control.NodeEnableVelControl
节点生命周期: initialise() 切权限 → 循环 update() → RUNNING（源节点口径，从不收口）
              → terminate() 恢复
底层路径: 节点 get_shared_hardware() → get_vel_control_state_jibot（读原状态）
                                    → enable_vel_control_jibot（切/恢复）
测试策略（模块单独测试，不依赖真机 / ROS 服务）:
    monkeypatch 节点模块的 get_shared_hardware 注入伪硬件，并把模块 logger
    换成记录器，从而：
    - 不需要 kuavo_msgs / jibot 下位机服务
    - 能断言「读原状态失败」与「原状态未知时跳过恢复」是否真的出声

测试用例说明:
- test_dry_run: STUDIO_DRY_RUN=1 时短路，不碰硬件
- test_restore_to_original: 读到原值 True，切 False，terminate 恢复 True
- test_read_fail_warns_and_skips_restore: 读原状态失败 → WARN + feedback 写明
  「原状态未知」，terminate 不复原且再 WARN 一次（2026-10-20 真机自锁泄漏）
- test_restore_to_param: restore_to=true 时即使读失败也恢复成 True
- test_restore_to_overrides_original: restore_to=false 优先于读到的原值 True
- test_read_skipped_when_restore_to_set: 配了 restore_to 就不再读原状态
- test_restore_disabled: restore_on_stop=false 时不读原值、不恢复
- test_switch_fail_no_restore: 切换失败 → update FAILURE，terminate 不恢复
- test_missing_method: 硬件无 enable_vel_control_jibot 时不阻断（RUNNING）

运行:
    python3 apps/test_kuavo_5w_orchestration/test_enable_vel_control_node.py
退出码 0 通过、1 失败。
"""
import sys
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(project_root))
sys.path.insert(0, str(project_root / "apps"))

from core.common.logger import init_logging, get_logger
init_logging()

from py_trees.common import Status

from core.domain.result import Result
from orchestration.nodes.node_enable_vel_control import NodeEnableVelControl
import orchestration.nodes.node_enable_vel_control as enable_mod

logger = get_logger(__name__)


class _RecordingLogger:
    """记录 WARNING 的假 logger（节点只用到 warning / info）。"""

    def __init__(self):
        self.warnings = []
        self.infos = []

    def warning(self, msg, *args):
        self.warnings.append(msg % args if args else msg)

    def info(self, msg, *args):
        self.infos.append(msg % args if args else msg)

    def _joined_warnings(self):
        return "\n".join(self.warnings)


class _FakeHardware:
    """伪硬件：记录权限切换与状态读取的调用序列。"""

    def __init__(self, orig_state=True, read_ok=True, switch_ok=True):
        self.orig_state = orig_state
        self.read_ok = read_ok
        self.switch_ok = switch_ok
        self.enable_calls = []      # enable_vel_control_jibot 的入参序列
        self.read_calls = 0         # get_vel_control_state_jibot 调用次数

    def get_vel_control_state_jibot(self, timeout: float = 3.0):
        self.read_calls += 1
        if not self.read_ok:
            return Result.fail("无法读取 /enable_vel_control_state")
        return Result.ok(f"vel_control_state={self.orig_state}",
                         data={"state": self.orig_state})

    def enable_vel_control_jibot(self, enable):
        self.enable_calls.append(enable)
        if not self.switch_ok:
            return Result.fail("enable_vel_control service unavailable")
        return Result.ok(f"enable_vel_control({enable})")


class _NonJibotHardware:
    """没有 jibot 权限接口的硬件（非 jibot 底盘）。"""


def _make_node(**params):
    """构造节点；params 以字符串传入，模拟 Studio 表单/JSON 的值口径。"""
    raw = {k: str(v) for k, v in params.items()}
    return NodeEnableVelControl("t_enable_vel", "速度控制权限切换", "ns", raw)


class _Patched:
    """一次性替换节点模块的 get_shared_hardware 与 logger。"""

    def __init__(self, hw):
        self.hw = hw
        self.log = _RecordingLogger()

    def __enter__(self):
        self._orig_hw = enable_mod.get_shared_hardware
        self._orig_log = enable_mod.logger
        enable_mod.get_shared_hardware = lambda: self.hw
        enable_mod.logger = self.log
        return self.log

    def __exit__(self, *exc):
        enable_mod.get_shared_hardware = self._orig_hw
        enable_mod.logger = self._orig_log
        return False


def test_dry_run():
    """STUDIO_DRY_RUN=1 时短路，不碰硬件。"""
    fake = _FakeHardware()
    with _Patched(fake):
        orig = enable_mod._DRY_RUN
        enable_mod._DRY_RUN = True
        try:
            node = _make_node(enable=False)
            node.initialise()
            assert node.update() == Status.RUNNING, "dry-run 应 RUNNING"
        finally:
            enable_mod._DRY_RUN = orig
    assert fake.enable_calls == [], f"dry-run 不应调权限服务，得到 {fake.enable_calls}"
    assert fake.read_calls == 0, "dry-run 不应读原状态"
    logger.info("    dry-run 短路、不碰硬件")


def test_restore_to_original():
    """读到原值 True，切 False，terminate 恢复 True。"""
    fake = _FakeHardware(orig_state=True)
    with _Patched(fake):
        node = _make_node(enable=False)
        node.initialise()
        assert node.update() == Status.RUNNING
        assert fake.enable_calls == [False], f"应切到 False，得到 {fake.enable_calls}"
        assert node._orig_state is True, f"应记录原值 True，得到 {node._orig_state}"
        node.terminate(Status.SUCCESS)
    assert fake.enable_calls == [False, True], \
        f"terminate 应恢复 True，得到 {fake.enable_calls}"
    logger.info("    读原值 → 切 → 收尾恢复原值")


def test_read_fail_warns_and_skips_restore():
    """读原状态失败：WARN + feedback 写明「原状态未知」，收尾不复原且再 WARN。

    这是 2026-10-20 真机自锁泄漏的现场：读超时导致原值 None，收尾静默跳过恢复，
    权限永久停在 False，之后每次都「正确地」恢复成 False。
    """
    fake = _FakeHardware(read_ok=False)
    with _Patched(fake) as log:
        node = _make_node(enable=False)
        node.initialise()
        assert node.update() == Status.RUNNING
        assert node._orig_state is None, "读失败时原值应为 None"
        assert fake.enable_calls == [False], f"应仍切到 False，得到 {fake.enable_calls}"
        assert "原状态未知" in node.feedback_message, \
            f"feedback 应写明原状态未知，得到 {node.feedback_message!r}"

        node.terminate(Status.SUCCESS)

    assert len(log.warnings) >= 2, f"读失败与跳过恢复各应 WARN，得到 {log.warnings}"
    assert "读取底盘速度控制权限原状态失败" in log.warnings[0], log.warnings[0]
    assert "跳过恢复" in log.warnings[-1] and "请人工确认" in log.warnings[-1], \
        log.warnings[-1]
    assert fake.enable_calls == [False], \
        f"原状态未知时不应猜着恢复，得到 {fake.enable_calls}"
    logger.info("    读失败 → WARN ×2、跳过恢复")


def test_restore_to_param():
    """restore_to=true：读失败也能兜底恢复成 True。"""
    fake = _FakeHardware(read_ok=False)
    with _Patched(fake) as log:
        node = _make_node(enable=False, restore_to="true")
        node.initialise()
        assert node.update() == Status.RUNNING
        assert fake.enable_calls == [False], f"应切到 False，得到 {fake.enable_calls}"
        node.terminate(Status.SUCCESS)
    assert fake.enable_calls == [False, True], \
        f"restore_to 应兜底恢复 True，得到 {fake.enable_calls}"
    assert not any("跳过恢复" in w for w in log.warnings), \
        f"配了 restore_to 就不该报跳过恢复，得到 {log.warnings}"
    logger.info("    restore_to=true 兜底恢复")


def test_restore_to_overrides_original():
    """restore_to=false 优先于读到的原值 True（显式配置赢）。"""
    fake = _FakeHardware(orig_state=True)
    with _Patched(fake):
        node = _make_node(enable="true", restore_to="false")
        node.initialise()
        assert fake.enable_calls == [True], f"应切到 True，得到 {fake.enable_calls}"
        node.terminate(Status.SUCCESS)
    assert fake.enable_calls == [True, False], \
        f"收尾应恢复到 restore_to=False，得到 {fake.enable_calls}"
    logger.info("    restore_to 覆盖读到的原值")


def test_read_skipped_when_restore_to_set():
    """配了 restore_to 就不必读原状态（省掉最长 3 s 的等待）。"""
    fake = _FakeHardware(orig_state=True)
    with _Patched(fake):
        node = _make_node(enable=False, restore_to="true")
        node.initialise()
    assert fake.read_calls == 0, f"不该读原状态，读了 {fake.read_calls} 次"
    logger.info("    restore_to 已给定 → 跳过读原状态")


def test_restore_disabled():
    """restore_on_stop=false：不读原值、不恢复。"""
    fake = _FakeHardware(orig_state=True)
    with _Patched(fake):
        node = _make_node(enable=False, restore_on_stop="false")
        node.initialise()
        node.terminate(Status.SUCCESS)
    assert fake.read_calls == 0, "关闭恢复后不该读原状态"
    assert fake.enable_calls == [False], \
        f"关闭恢复后不该再调权限服务，得到 {fake.enable_calls}"
    logger.info("    restore_on_stop=false → 不读不恢复")


def test_switch_fail_no_restore():
    """切换失败 → update FAILURE，terminate 不做恢复（没切成功就没什么可恢复）。"""
    fake = _FakeHardware(orig_state=True, switch_ok=False)
    with _Patched(fake):
        node = _make_node(enable=False)
        node.initialise()
        assert node.update() == Status.FAILURE, "切换失败应 FAILURE（整树停）"
        node.terminate(Status.FAILURE)
    assert fake.enable_calls == [False], \
        f"没切成功不该恢复，得到 {fake.enable_calls}"
    logger.info("    切换失败 → FAILURE、不恢复")


def test_missing_method():
    """硬件无 enable_vel_control_jibot（非 jibot）时不阻断树。"""
    with _Patched(_NonJibotHardware()):
        node = _make_node(enable=False)
        node.initialise()
        assert node.update() == Status.RUNNING, "非 jibot 硬件应 RUNNING、不阻断"
        node.terminate(Status.SUCCESS)
    logger.info("    无 jibot 权限接口 → RUNNING 不阻断")


def main() -> int:
    cases = [
        ("dry_run 短路", test_dry_run),
        ("读原值 → 切 → 收尾恢复", test_restore_to_original),
        ("读失败 WARN 且跳过恢复", test_read_fail_warns_and_skips_restore),
        ("restore_to 兜底恢复", test_restore_to_param),
        ("restore_to 覆盖原值", test_restore_to_overrides_original),
        ("restore_to 时跳过读原状态", test_read_skipped_when_restore_to_set),
        ("restore_on_stop=false 不读不恢复", test_restore_disabled),
        ("切换失败不恢复", test_switch_fail_no_restore),
        ("无 jibot 接口不阻断", test_missing_method),
    ]
    failed = 0
    for title, fn in cases:
        try:
            fn()
        except AssertionError as exc:
            failed += 1
            logger.error("  FAIL  %s: %s", title, exc)
        except Exception as exc:                     # noqa: BLE001
            failed += 1
            logger.error("  ERROR %s: %s: %s", title, type(exc).__name__, exc)

    logger.info("")
    if failed:
        logger.error("FAILED: %d/%d", failed, len(cases))
        return 1
    logger.info("OK: %d/%d 全部通过 —— 速度控制权限切换（编排层，伪硬件注入）测试完成",
                len(cases), len(cases))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
