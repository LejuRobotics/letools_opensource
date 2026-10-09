# -*- coding: utf-8 -*-
"""行为树节点的**黑板读写与参数兜底** —— 五处逐字重复的实现收敛到这里。

## 为什么会有这个模块

`node_box_obs` / `node_pallet_obs` / `node_pallet_servo` /
`node_inject_servo_input` / `node_pallet_pose` 五个节点各自抄了一份
`_read_blackboard` / `_read_version` / `_num` / `_bump_version` / `_is_dry_run`
（函数体逐字相同，只有注释详略不同）。2026-09-24 盘点时把它们收敛到一处。

**这里只放"抄出来的那部分"**，不放任何节点的业务逻辑 —— 各节点的
`update()` / `initialise()` 一行没动。

## 本模块不碰的两处（有意保留重复）

* 各节点的 `_cleanup_subscriber`：它是**实例方法**，靠 `self._subscriber`
  这个属性名工作。收敛要么传字符串属性名（丑），要么改成"返回句柄、调用方赋值"
  （调用点更绕）。两处而已，不值得为它换一个更差的写法。
* 本链路之外的 14 处 `_is_dry_run`（`haichen_internal/`、`zhaofeng_internal/`
  等）：不属于本次范围，改动面会外溢。以后谁碰到谁迁。
"""
from __future__ import annotations

import os
from typing import Optional

__all__ = [
    "bump_version",
    "is_dry_run",
    "num_or_none",
    "read_blackboard",
    "read_version",
]


def is_dry_run() -> bool:
    """`STUDIO_DRY_RUN` 环境变量是不是真。

    ⚠️ **不能用 `bool(os.environ.get(...))`**：`STUDIO_DRY_RUN=0` 和
    `STUDIO_DRY_RUN=false` 都会是 `True` —— 而这两个写法在脚本里很常见，
    真机上一跑就是"以为在干跑、其实在动"。
    """
    return os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


def read_blackboard(client, key, default=None):
    """读黑板键；**键从未被写过时也返回 `default`**。

    py_trees 的 `Client.__getattr__` 在键不存在时抛的是 **`KeyError`**、不是
    `AttributeError`（见 `py_trees/blackboard.py` 的 `__getattr__`），所以
    `getattr(client, key, None)` **接不住它** —— 看上去在兜底，实际会把
    `KeyError` 一路抛穿 `update()`。而 py_trees 的 `Behaviour.tick()` **不接**
    `update()` 抛出的异常：**整棵树连每帧日志一起没**。

    仓库里另一条路是**生产者在构造时先写一次初值**（`NodePalletPose.__init__`
    写 `latest_pallet = None`），但那条路要求**每个未来的生产者都记得** ——
    漏写一次就是环路崩掉。所以兜底放在读的一侧：谁忘了预置都不会炸，
    "没有输入"就是"没有输入"。

    **故意只接 `KeyError`**：`AttributeError` 意味着键没注册（权限写错了），
    那是调用方自己的 bug，应该照旧响亮地抛出来，不该被 `default` 吞掉。
    """
    try:
        return getattr(client, key)
    except KeyError:
        return default


def read_version(client, key) -> int:
    """读 `key_version`；**从未写过 / 不是数字**都当 0（调用方随后 +1 写回）。

    旧代码有直接 `int(...)` 的写法，上游写个字符串就把 `update()` 抛穿 ——
    同样会掀翻整棵树。
    """
    try:
        return int(read_blackboard(client, key, 0))
    except (TypeError, ValueError):
        return 0


def bump_version(blackboard, key) -> int:
    """把 `key_version` 加一写回，返回写进去的那个数。

    读**黑板上的当前值**再加 —— 版本号因此单调，重进 `initialise()` 也不会
    掉回去（掉回去会让消费侧的"单调门禁"把它当成陈旧数据永远跳过）。
    """
    version = read_version(blackboard, f"{key}_version") + 1
    setattr(blackboard, f"{key}_version", version)
    return version


def num_or_none(value) -> Optional[float]:
    """宽容地取一个数：认 float / int / numpy 标量 / 数字字符串；其余 `None`。

    ⚠️ **NaN 也返回 `None`** —— NaN 能通过 `float()`，但它在比较里全是 False，
    放过去会让下游的 `if x > threshold` 静默走错分支。
    """
    if value is None:
        return None
    try:
        f = float(value)
    except (TypeError, ValueError):
        return None
    if f != f:                                  # NaN
        return None
    return f
