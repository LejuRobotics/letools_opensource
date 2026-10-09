# -*- coding: utf-8 -*-
"""`utils/blackboard.py` 的单元测试。

这五个 helper 原先是**五个节点各抄一份**（函数体逐字相同），2026-09-24 收敛到
一处。测试重点不是"函数算得对"（那几个函数都很短），而是**收敛时最容易弄丢的
那几条语义** —— 它们原先散在各自的 docstring 里，抄丢了不会有人发现：

* `read_blackboard` **只接 `KeyError`**，`AttributeError` 必须照旧抛出去
* `read_version` 把"没写过"和"写了个不是数字的"**都当 0**
* `bump_version` 读的是**黑板上的当前值**（不是缓存的），所以单调
* `num_or_none` **把 NaN 也当 None**（NaN 能过 `float()`，但比较全是 False）
* `is_dry_run` **不认 `"0"` / `"false"` 是假**（用 `bool()` 会全当真）
"""
import py_trees
import pytest

from orchestration.nodes.utils.blackboard import (
    bump_version,
    is_dry_run,
    num_or_none,
    read_blackboard,
    read_version,
)


# --------------------------------------------------------------------------- #
# is_dry_run
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("value", ["1", "true", "TRUE", "True", "yes", "YES"])
def test_dry_run_truthy(value, monkeypatch):
    monkeypatch.setenv("STUDIO_DRY_RUN", value)
    assert is_dry_run() is True


@pytest.mark.parametrize("value", ["0", "false", "FALSE", "no", "NO", "", "  "])
def test_dry_run_falsy(value, monkeypatch):
    """★ `"0"` / `"false"` **必须是假**。

    用 `bool(os.environ.get(...))` 写的话这两个都是 `True` —— 而它们恰恰是
    脚本里最常见的两种"关掉"写法。真机上一跑就是"以为在干跑、其实在动"。
    """
    monkeypatch.setenv("STUDIO_DRY_RUN", value)
    assert is_dry_run() is False


def test_dry_run_unset(monkeypatch):
    monkeypatch.delenv("STUDIO_DRY_RUN", raising=False)
    assert is_dry_run() is False


# --------------------------------------------------------------------------- #
# read_blackboard
# --------------------------------------------------------------------------- #
def _bb(*keys, write=True):
    """建一块注册了 `keys` 的黑板客户端（py_trees 的真实现）。

    默认给**读 + 写**权限 —— 写不了的话连"造一个已写过的键"都做不到，而那正是
    大半用例要测的。`py_trees` 的 `Access` 是**枚举不是位标志**（`READ | WRITE`
    会抛 `TypeError`），所以两种权限要**分开调两次** `register_key`。

    要专门测"没注册就抛"的那条另说 —— 它压根不注册。
    """
    client = py_trees.blackboard.Client(name="test")
    for k in keys:
        client.register_key(key=k, access=py_trees.common.Access.READ)
        if write:
            client.register_key(key=k, access=py_trees.common.Access.WRITE)
    return client


def test_read_blackboard_registered_but_never_written_returns_default():
    """★ 键**注册过但从没写过** → `default`（不是抛）。

    这是这个函数存在的全部理由：py_trees 在"键没写过"时抛 `KeyError`，而
    `getattr(client, key, None)` 接不住它 —— 会把一个正常的"等输入"变成异常，
    再一路抛穿 `update()`，整棵树连每帧日志一起没。
    """
    client = _bb("never_written")
    assert read_blackboard(client, "never_written") is None
    assert read_blackboard(client, "never_written", "兜底") == "兜底"


def test_read_blackboard_returns_written_value():
    client = _bb("written")
    client.written = 42
    assert read_blackboard(client, "written") == 42
    assert read_blackboard(client, "written", "兜底") == 42


def test_read_blackboard_falsey_value_is_not_replaced_by_default():
    """★ 写进去的**假值**（`0` / `False` / `""` / `[]`）必须原样返回。

    用 `getattr(...) or default` 写的话这些全会被兜底值顶掉 —— 而 `0` 是个
    完全合法的版本号 / 计数。
    """
    for i, value in enumerate((0, False, "", [], 0.0)):
        client = _bb(f"k{i}")
        setattr(client, f"k{i}", value)
        assert read_blackboard(client, f"k{i}", "兜底") == value, f"{value!r} 被顶掉了"


def test_read_blackboard_unregistered_key_raises():
    """★ 键**没注册**（权限写错）→ 照旧抛 `AttributeError`，**不吞**。

    "没注册"是调用方自己的 bug（register_key 写漏了），静默兜底会让它永远
    查不出来。这与"注册了但没写过"是两回事，必须分开。
    """
    client = _bb("registered")
    with pytest.raises(AttributeError):
        read_blackboard(client, "not_registered", "兜底")


# --------------------------------------------------------------------------- #
# read_version
# --------------------------------------------------------------------------- #
def test_read_version_never_written_is_zero():
    assert read_version(_bb("x_version"), "x_version") == 0


def test_read_version_non_numeric_is_zero():
    """★ 上游写了个字符串 / `None` → 当 0，**不抛**。

    旧代码这里直接 `int(...)`，上游写个 `"abc"` 就把 `update()` 抛穿。
    """
    for i, value in enumerate(("abc", None, [], {})):
        key = f"v{i}"
        client = _bb(key)
        setattr(client, key, value)
        assert read_version(client, key) == 0, f"{value!r} 没被当 0"


def test_read_version_numeric_string_is_parsed():
    """能解释成整数的字符串照常解析（`"7"` → 7，不是 0）。"""
    client = _bb("s")
    client.s = "7"
    assert read_version(client, "s") == 7


# --------------------------------------------------------------------------- #
# bump_version
# --------------------------------------------------------------------------- #
def test_bump_version_starts_at_one():
    client = _bb("latest_thing_version")
    assert bump_version(client, "latest_thing") == 1
    assert read_blackboard(client, "latest_thing_version") == 1


def test_bump_version_is_monotonic_across_calls():
    client = _bb("k_version")
    assert [bump_version(client, "k") for _ in range(4)] == [1, 2, 3, 4]


def test_bump_version_reads_current_value_not_a_cache():
    """★ 读的是**黑板上的当前值** —— 别人（或重进 `initialise()`）写过的数要接着涨。

    从 0 重新数会让消费侧的"单调门禁"把新数据当成陈旧数据永远跳过。
    """
    client = _bb("k_version")
    client.k_version = 41                      # 模拟"别人先写过"
    assert bump_version(client, "k") == 42


# --------------------------------------------------------------------------- #
# num_or_none
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("value,expected", [
    (3, 3.0), (3.5, 3.5), ("2.5", 2.5), ("-1", -1.0), (0, 0.0),
])
def test_num_or_none_accepts_numbers(value, expected):
    assert num_or_none(value) == expected


def test_num_or_none_accepts_numpy_scalar():
    np = pytest.importorskip("numpy")
    assert num_or_none(np.float32(1.25)) == pytest.approx(1.25)
    assert num_or_none(np.int64(7)) == 7.0


@pytest.mark.parametrize("value", [None, "abc", [], {}, object()])
def test_num_or_none_rejects_non_numbers(value):
    assert num_or_none(value) is None


def test_num_or_none_nan_is_none():
    """★ **NaN 也是 `None`**。

    NaN 能顺利通过 `float()`，但它在任何比较里都是 False —— 放过去会让下游
    的 `if x > threshold` 静默走错分支，而值本身"看起来是个数"。
    """
    assert num_or_none(float("nan")) is None


def test_num_or_none_inf_passes_through():
    """`inf` 是**数**，不是"没有" —— 保留它（该由调用方的范围判据去挡）。"""
    assert num_or_none(float("inf")) == float("inf")
