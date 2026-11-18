# -*- coding: utf-8 -*-
"""从 TF 链查台面法向先验。**这一层会 `import rospy`，所以放在基础设施层。**

`pallet_detect` 的算法与搬运层（`skills/atomic/perception/pallet_detect/`）**不许
import ROS** —— 那是分层硬约束。但"从 TF 查法向"这件事本质上要 ROS，所以它落在
这里，紧挨着调用它的节点。

    normal_from_tf(buf, target_frame, source_frame, stamp, missing, stale)
                                                              ->  (3,) | None

⚠️ **`CHAIN` 与 maduo 的 `tools/tf_normal_from_bag.py` 共用同一份定义**
（那条链 13 段，那边用来从 bag 离线查、这边在线查）。改链时**两边都要改**。

⚠️ **缺任何一段都返回 `None`，绝不静默返回单位阵** —— 单位阵 = 法向竖直向下，
而检测**不会崩、不会报错**，只是换个 score 继续算。调用方据此**不发布**并给出
明确报错。理由见 `pallet_detect/README.md` §3 第 1 条。

⚠️ **整条链按时刻查，不逐段混时刻。** 先按图像 `stamp` 把 13 段**整条**拼一遍；
只要有一段缺，就**整条**退回最新值（`Time(0)`）再拼一遍。逐段各自"缺了就取最新"
会把不同时刻的关节/头部位姿拼进同一个变换，在机器人运动时是一个静默的几何错。
退回最新值时通过 `stale` 把 `stamp` 的**秒数（float）**交出去，调用方必须把它
写进日志/心跳。
"""
from __future__ import annotations

import numpy as np

from skills.atomic.perception.pallet_detect_v2.payload import CHAIN  # noqa: F401

# 图像 stamp 比 TF 最新样本晚几十毫秒时，tf2 的精确 lookup 会拒绝外推。
# 对伺服这种准静态链路，允许把整条 13 段链统一回退到图像时刻之前的一小段，
# 但**绝不逐段混时刻**。这些偏移按顺序尝试；找到后仍用同一时刻拼完整链。
TF_STAMP_SHIFTS_S = (0.0, 0.02, 0.05, 0.10, 0.20)


def _stamp_sec(stamp) -> float:
    """`rospy.Time` / Time-like / int / float → 秒（float）。

    ⚠️ **不能直接 `float(rospy.Time)`**：它会抛
    `TypeError: float() argument must be a string or a number, not 'Time'`。
    节点传进来的正是 `color_msg.header.stamp`（`rospy.Time`），这里统一走
    `to_sec()`；只有没有 `to_sec` 的纯数字才走 `float()`。

    这条转换以前写成了 `float(stamp)`，有两个后果：
      1. `_ros_time()` 里抛出的 TypeError 被 `_lookup_edge` 的
         `except Exception` 吞掉 → **按图像时刻查 TF 永远失败**，每次都退回
         最新值（功能静默退化）；
      2. `stale.append(float(stamp))` 在 try 外面 → 直接把检测线程打挂。
    2026-11-06 现场启动报错即由此而来。
    """
    if stamp is None:
        return 0.0
    to_sec = getattr(stamp, 'to_sec', None)
    if callable(to_sec):
        return float(to_sec())
    return float(stamp)


def _compose_chain_near(buf, stamp, *, missing=None, shifted=None):
    """先按 `stamp`，失败时只在**整条链同一时刻**上向前试少量偏移。

    `missing` 记录精确 stamp 那次缺的边；`shifted` 记录实际使用的偏移秒数。
    `None` stamp 直接查最新值，不走偏移。
    """
    if stamp is None:
        return _compose_chain(buf, 0.0, missing=missing)
    stamp_sec = _stamp_sec(stamp)
    first_missing = None
    for off in TF_STAMP_SHIFTS_S:
        m = []
        T = _compose_chain(buf, stamp_sec - float(off), missing=m)
        if T is not None:
            if off > 0.0 and shifted is not None:
                shifted.append(float(off))
            if missing is not None:
                missing.extend(m if off == 0.0 else (first_missing or []))
            return T
        if off == 0.0:
            first_missing = m
    if missing is not None and first_missing:
        missing.extend(first_missing)
    return None


def normal_from_tf(buf, target_frame: str, source_frame: str = 'base_link',
                   stamp=None, missing: list | None = None,
                   stale: list | None = None,
                   shifted: list | None = None) -> np.ndarray | None:
    """从 TF 查 `source_frame` 的 +z 在 `target_frame`（相机系）里的方向（= 台面法向先验）。

    逐段查 `tools/tf_normal_from_bag.CHAIN`（13 段），拼出 `T_source_target`。

    ⚠️ **缺任何一段都返回 None**，不要静默返回单位阵 —— 与
    `tools/tf_normal_from_bag.py` 同一条规矩（那条链 13 段，缺一段就拼不出来）。
    调用方（节点）据此**不发布**并给出明确报错，而不是拿个瞎猜的法向继续算 ——
    detect 单独对法向极其敏感，错 5° 就是 ~180mm（见模块 docstring；
    但 refine 的平面拟合能修回来，前提是 `plane=True`）。

    ⚠️ **过期 TF 的宽容只在这里做一次，而且整条链一起退。** 先按 `stamp` 拼
    完整 13 段；任一段缺就**整条**按 `Time(0)`（最新值）重拼。绝不逐段混用
    不同时刻 —— 那样拼出来的 `T_base_cam` 不对应任何一个真实时刻，机器人运动时
    是一个静默的几何错。退回最新值时把 `stamp` 的**秒数（float）** append 到
    `stale`（调用方用它写日志/心跳）。`stamp=None` 只查最新值，不算 stale。

    ⚠️ 相机系是**光学系**（+x 右 +y 下 +z 前），所以：
        up_in_camera = T_base_camera[:3, :3].T @ [0, 0, 1]

    `missing`：给一个 list 时，缺的那一段 `(parent, child)` 会被 append 进去
    （调用方要拿它写日志/`rejects`）。**这是唯一的副作用，不影响返回值。**
    `stale`：给一个 list 时，发生"整条链退回最新值"会把 `stamp` 的**秒数**
    append 进去。**这是唯一的副作用。**
    """
    # ⚠️ 这里**不** `import tf2_ros`：本函数只用 `buf` 的 `lookup_transform` 接口，
    # 不依赖 tf2_ros 这个模块本身。加上那个 import 会让它**没有 ROS 就 import 不了**，
    # 而它是纯函数、单元测试要用假 buffer 直接调（`test_pallet_detection_node.py`）。
    T = _compose_chain_near(buf, stamp, missing=missing, shifted=shifted)
    if T is None and stamp is not None:
        # ★ **整条链**退回最新值，不是逐段退。`Time(0)` 是 tf2 的"最新值"约定。
        latest_missing: list = []
        T = _compose_chain(buf, 0.0, missing=latest_missing)
        if T is not None:
            if stale is not None:
                stale.append(_stamp_sec(stamp))
        elif missing is not None and not missing:
            missing.extend(latest_missing)
    if T is None:
        return None
    up = T[:3, :3].T @ np.array([0.0, 0.0, 1.0])
    norm = float(np.linalg.norm(up))
    if not np.isfinite(norm) or norm < 1e-9:
        if missing is not None:
            missing.append(('<normal 退化>', f'|up|={norm}'))
        return None
    return up / norm


def transform_from_tf(buf, target_frame: str, source_frame: str = 'base_link',
                      stamp=None, missing: list | None = None,
                      stale: list | None = None,
                      shifted: list | None = None):
    """拼 `T_target_source`（4×4，平移**米**），查不到返回 `None`。

    与 `normal_from_tf` 走**同一条 `CHAIN`**、同一套"整条链退回最新值"的规矩；
    只是它保留完整旋转与平移。`call 0` 记录里的法向/D 是 base_link 系的，检测器
    要把它们变到相机系，需要的正是这个矩阵（只需要一次，之后缓存）。
    """
    T = _compose_chain_near(buf, stamp, missing=missing, shifted=shifted)
    if T is None and stamp is not None:
        latest_missing: list = []
        T = _compose_chain(buf, 0.0, missing=latest_missing)
        if T is not None:
            if stale is not None:
                stale.append(_stamp_sec(stamp))
        elif missing is not None and not missing:
            missing.extend(latest_missing)
    if T is None:
        return None
    if not np.all(np.isfinite(T)):
        if missing is not None:
            missing.append(('<T 退化>', 'NaN/inf'))
        return None
    return T


def camera_origin_from_tf(buf, target_frame: str, source_frame: str = 'base_link',
                          stamp=None, missing: list | None = None,
                          stale: list | None = None,
                          shifted: list | None = None):
    """`source_frame`（相机光学系）原点在 `target_frame`（base_link）里的位置，**米**。

    返回 `(3,) | None`。**与 `normal_from_tf` 走同一条 `CHAIN`**，只是取的是平移
    而不是旋转 —— 两者拼的是同一个 `T_base_cam`，所以这里再走一遍只是为了避免
    把两个返回值捆成一个接口（调用方经常只要法向）。

    TF 的取法、过期宽容与 `missing` / `stale` 语义**与 `normal_from_tf` 完全一致**。

    ⚠️ **用来把"沿法向的相对高度"换算成"base_link 的绝对高度"**：
    某个点在相机系是 `P`（米），它沿法向的高度是 `h = P·n`，而它在 base_link 的
    绝对 z 是 `(T_base_cam @ P).z`。两者只差一个**常数**：

        K = t_base_cam.z + (origin_placeholder · n)

    其中 `origin_placeholder` 是 `detect_pallet_frame` 里那个占位平面原点。
    调用方把 `K` 算出来，就能把 `deck_h` 约束在"托盘应该在的绝对高度带"里
    （现场实测：托盘 z ≈ +43mm、地面 ≈ −107mm、**手里抱着的纸箱 ≈ +600mm**，
    所以一条 `[-40, +130]` 的带就能把纸箱整个排除掉）。

    ⚠️ 缺任何一段都返回 `None`，与 `normal_from_tf` 同一条规矩。
    """
    T = _compose_chain_near(buf, stamp, missing=missing, shifted=shifted)
    if T is None and stamp is not None:
        latest_missing: list = []
        T = _compose_chain(buf, 0.0, missing=latest_missing)
        if T is not None:
            if stale is not None:
                stale.append(_stamp_sec(stamp))
        elif missing is not None and not missing:
            missing.extend(latest_missing)
    if T is None:
        return None
    t = np.asarray(T[:3, 3], dtype=np.float64)
    if not np.all(np.isfinite(t)):
        if missing is not None:
            missing.append(('<origin 退化>', f't={t.tolist()}'))
        return None
    return t


def _compose_chain(buf, stamp, missing: list | None = None):
    """在**单一时刻**拼完 13 段；任一段缺就返回 None（不做逐段时间回退）。

    `stamp` 用 0 表示 tf2 的"最新值"。缺边时把 `(parent, child)` 写进 `missing`
    （给了才写）。
    """
    T = np.eye(4, dtype=np.float64)
    for parent, child in CHAIN:
        edge = _lookup_edge(buf, parent, child, stamp)
        if edge is None:
            if missing is not None:
                missing.append((parent, child))
            return None
        T = T @ edge
    return T


_EXPECTED_LOOKUP_ERRORS = {
    # tf2_ros 的正常"这个时刻查不到"异常
    'LookupException', 'ExtrapolationException', 'ConnectivityException',
    'InvalidArgumentException',
    # 某些 tf2 绑定直接抛基类；它仍然只表示"查不到"，不是代码写错
    'TransformException',
}


def _is_expected_lookup_error(exc) -> bool:
    """是不是 tf2 的正常查询失败（缺边/外推），而不是代码写错。

    ⚠️ 这里刻意**只放行这些**。以前 `_lookup_edge` 用 `except Exception` 把
    所有异常都当成"缺边"，结果 `float(rospy.Time)` 的 TypeError 被吞掉，
    表现为"按图像时刻查 TF 永远失败、每次都退回最新值"—— 功能静默退化，
    日志里看不出。测试里的假 buffer 抛的是内置 `LookupError`，也放行。
    """
    return (type(exc) is LookupError
            or type(exc).__name__ in _EXPECTED_LOOKUP_ERRORS)


def _lookup_edge(buf, parent: str, child: str, stamp):
    """查一段 `T_parent_child`（4x4），**只查给定的这一个时刻**。查不到返回 None。

    时间回退由 `_compose_chain` 的调用方在**整条链**上做（见 `normal_from_tf` 的
    docstring）：这里只负责"这个时刻、这一段，有没有"。反方向也试一次（TF 里
    只登记了一个方向是常事）—— 与 `tf_normal_from_bag.lookup` 同一条规矩：
    **缺边返回 None，不静默造一个**。

    ⚠️ 只吞"预期的 TF 查不到异常"；其它异常（类型错、属性错）必须往上抛，
    否则又会出现"每次都退回最新值"这种静默退化。
    """
    for target, source in ((parent, child), (child, parent)):
        try:
            tr = buf.lookup_transform(target, source, _ros_time(stamp))
        except Exception as exc:                   # noqa: BLE001
            if not _is_expected_lookup_error(exc):
                raise
            continue
        E = _transform_to_matrix(tr)
        if E is None:
            continue
        return E if (target, source) == (parent, child) else np.linalg.inv(E)
    return None


def _ros_time(stamp):
    """秒 / `rospy.Time` / Time-like → `rospy.Time`（0 = 最新值）。

    ⚠️ `rospy.Time` 必须**原样返回**，不能再 `float()`：`float(Time)` 会抛
    TypeError，而 `_lookup_edge` 的 `except Exception` 会把它吞掉 —— 症状不是
    报错，而是"按图像时刻查 TF 永远失败、每次都退回最新值"。
    """
    import rospy
    if isinstance(stamp, rospy.Time):
        return stamp
    return rospy.Time.from_sec(_stamp_sec(stamp))


def _transform_to_matrix(tr) -> np.ndarray | None:
    """`geometry_msgs/TransformStamped` → 4x4。取不出来返回 None。"""
    try:
        from tf.transformations import quaternion_matrix
    except ImportError:                              # pragma: no cover
        return None
    q = tr.transform.rotation
    p = tr.transform.translation
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
    T[:3, 3] = [p.x, p.y, p.z]
    return T
