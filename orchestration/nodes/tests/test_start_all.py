# -*- coding: utf-8 -*-
"""`start_all.sh` 的**启动口径**（2026-10-18 按现场流程重写）。

现场流程是「先起链路 → 去抓箱 → 到位才 call slot → 放完 call 0 → 再抓」，
所以启动脚本**不许等任何检测数据**：码放之前不一定看得见托盘和纸箱。

这里钉住六件事：

1. `--dry-run` 不连 ROS、不起任何东西、**很快**返回；
2. 两个检测器**都起**，且都带 `gated:=true`（待命 + 启动预热一帧）；
3. **不再有 `--slot`**（也不需要它了 —— 档位是运行中用服务切的）；
4. 脚本里**没有 `wait_topic`** —— 那正是旧版卡死的来源；
5. **报"就绪"前必须等预热完成** —— 注册是瞬间的，首帧推理要 28~35s；
6. **找不到 carton 虚拟环境就先说话**，不许静默回落成 system python
   （2026-10-25 现场：activate 里写死的是打包机器的路径 → `python3` 悄悄变成
   /usr/bin/python3 → `pip install colorlog` 报成功、行为树却 ModuleNotFoundError）。

跑法::

    pytest orchestration/nodes/tests/test_start_all.py -m unit -v
"""
import os
import subprocess
import sys
from pathlib import Path

import pytest

pytestmark = pytest.mark.unit

ROOT = Path(__file__).resolve().parents[3]
SCENARIO = ROOT / "orchestration" / "scenarios" / "pallet_servo_real_v1"
SCRIPT = SCENARIO / "start_all.sh"


@pytest.fixture()
def fake_venv(tmp_path):
    """造一个**最小 carton venv**，只满足 `start_all.sh` 的两条硬条件。

    `start_all.sh` 从 2026-10-25 起要求从 carton venv 起（`resolve_venv`，见
    `CARTON_VENV_DIR`）：`bin/python3` 可执行、`import colorlog` 能过。

    ⚠️ **单元测试不能依赖开发机上那个真 venv。** 有 venv 的机器上全绿、没 venv 的
    机器（CI、另一台电脑）第一条就红 —— 同一份代码两种结果，红的那条看起来像
    「脚本坏了」，其实是测试自己的环境假设错了。

    这里的 `bin/python3` 是个**转发壳**：只有被测脚本要的 `-c 'import colorlog'`
    直接放过，其余调用原样交给跑 pytest 的这个解释器 —— 所以 `board.json` 的解析
    仍然是真的（`board_get` / `check_dead_keys` 都靠它）。
    """
    bindir = tmp_path / "carton" / "bin"
    bindir.mkdir(parents=True)
    python3 = bindir / "python3"
    python3.write_text(
        "#!/bin/sh\n"
        "# 测试替身：骗过 start_all.sh 的 colorlog 前置检查，别的照转给真解释器。\n"
        'case "$*" in\n'
        '  *"import colorlog"*) exit 0 ;;\n'
        "esac\n"
        f'exec "{sys.executable}" "$@"\n',
        encoding="utf-8",
    )
    python3.chmod(0o755)
    return bindir.parent


def _run(venv, *args):
    env = dict(os.environ)
    # dry-run 只检查这两个**设没设**（不连 master），给个占位值即可。
    env.setdefault("ROS_MASTER_URI", "http://127.0.0.1:11311")
    env.setdefault("ROS_IP", "127.0.0.1")
    # **强制**指向本用例造的 venv：别让机器上碰巧存在的真 venv、或用户环境里残留的
    # CARTON_VENV_DIR 改变被测行为 —— 这个文件测的是启动口径，不是 venv 发现逻辑。
    env["CARTON_VENV_DIR"] = str(venv)
    return subprocess.run(["bash", str(SCRIPT), *args],
                          capture_output=True, text=True, env=env,
                          cwd=str(SCENARIO), timeout=120)


def test_dry_run_starts_both_detectors_and_does_not_wait(fake_venv):
    """★ `--dry-run` 必须**快速**成功 —— 任何"等话题"都会让它挂在启动阶段。"""
    proc = _run(fake_venv, "--dry-run")
    assert proc.returncode == 0, proc.stderr
    out = proc.stdout + proc.stderr
    assert "两个检测器都会起" in out, out
    assert "gated:=true" in out, out
    # 旧口径的两句必须消失
    assert "这一趟起托盘检测器" not in out, out
    assert "--slot" not in out, out


def test_dry_run_uses_the_venv_python(fake_venv):
    """★ `--dry-run` 必须**从 venv 起**，而不是静默用 system python3。

    2026-10-25 现场：`(carton)` 的 activate 写死 `/media/data/venvs/carton`，
    解压到别的目录后 `python3` 悄悄回落成 `/usr/bin/python3`；`pip install colorlog`
    报 Successfully，行为树却 ModuleNotFoundError。现在脚本必须先解析出 venv、
    把它放到 PATH 最前面，并在 `[1/4]` 里说出来。
    """
    proc = _run(fake_venv, "--dry-run")
    assert proc.returncode == 0, proc.stderr
    out = proc.stdout + proc.stderr
    assert "虚拟环境" in out, out
    assert str(fake_venv / "bin" / "python3") in out, out


def test_slot_option_is_gone(fake_venv):
    """★ 现场流程不需要 `--slot`：起链路时还不知道这一趟要不要托盘。

    它必须**当场被拒**（而不是静默忽略）—— 静默忽略会让操作员以为有额外行为。
    """
    proc = _run(fake_venv, "--slot", "5")
    assert proc.returncode == 2, proc.stdout + proc.stderr
    assert "未知参数" in (proc.stdout + proc.stderr)


def test_missing_venv_fails_loudly(tmp_path):
    """★ venv 找不到 / `CARTON_VENV_DIR` 写错 → **当场退出 2 并说清楚**。

    2026-10-25 踩的就是"环境不对但看起来正常"：一路没有任何硬错误，
    最后行为树才抛 traceback。这条前置条件必须先说话。
    """
    env = dict(os.environ)
    env.setdefault("ROS_MASTER_URI", "http://127.0.0.1:11311")
    env.setdefault("ROS_IP", "127.0.0.1")
    env["CARTON_VENV_DIR"] = str(tmp_path / "definitely-not-a-venv")
    proc = subprocess.run(["bash", str(SCRIPT), "--dry-run"],
                          capture_output=True, text=True, env=env,
                          cwd=str(SCENARIO), timeout=120)
    out = proc.stdout + proc.stderr
    assert proc.returncode == 2, out
    assert "CARTON_VENV_DIR" in out and "bin/python3" in out, out


def test_script_does_not_wait_for_detection_data():
    """★ 静态判据：脚本里**没有** `wait_topic`，两处 launch 都带 `gated:=true`。"""
    text = SCRIPT.read_text(encoding="utf-8")
    # 只认"真的调用"（`wait_topic()` 定义 / `wait_topic /话题` 调用）——
    # 注释里提到旧写法不算。
    assert "wait_topic()" not in text, "启动脚本仍然在等检测数据 —— 那正是要重构掉的东西"
    assert "wait_topic /" not in text, "启动脚本仍然在等检测数据"
    assert text.count('"gated:=true"') == 2, (
        "两个检测器都必须带 gated:=true（待命 + 启动预热一帧）；"
        f"实际 {text.count(chr(34) + 'gated:=true' + chr(34))} 处")
    assert "NEED_PALLET" not in text and "SLOT_PID" not in text, (
        "旧的按档位决定起哪些检测器的逻辑还在")


def test_script_waits_for_warmup_before_reporting_ready():
    """★ **注册 ≠ 就绪**：节点注册是瞬间的，首帧推理要 28~35s。

    不等预热的后果是脚本打印「已就绪」，而操作员立刻 call slot 后有 20~30s
    完全没结果 —— 现场会误判成检测/阈值坏了（2026-10-18 真机踩过）。
    """
    text = SCRIPT.read_text(encoding="utf-8")
    assert "wait_detector_warmup()" in text, "没有等预热的函数"
    assert "预热完成" in text, "预热判据（检测器日志里的「预热完成」）丢了"
    i_nodes = text.index("wait_detector_nodes || exit 1")
    i_warm = text.index("if wait_detector_warmup; then")
    i_ready = text.index("两个检测器已就绪")
    assert i_nodes < i_warm < i_ready, (
        "顺序必须是「注册 → 预热 → 报就绪」；提前报就绪正是这次要修的坑")


def test_help_works(fake_venv):
    proc = _run(fake_venv, "--help")
    assert proc.returncode == 0
    out = proc.stdout + proc.stderr
    assert "rosservice call /pallet_servo/slot" in out, out
