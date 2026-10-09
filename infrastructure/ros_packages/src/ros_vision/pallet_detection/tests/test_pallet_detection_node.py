#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""`pallet_detection_node` 的契约自检。跑法（退出码 0 通过 / 1 失败）：

    cd <仓库根>
    python3 infrastructure/ros_packages/src/ros_vision/pallet_detection/tests/test_pallet_detection_node.py

**不需要 ROS master、不需要相机、不需要任何数据** —— 用合成场景（**已知真值**）
跑一遍检测，再逐条核对**契约**（不是算法）：

  * `T_cam_pallet` 的行主序、平移是**米**、`det` 自己算、`corners_uv` 的顺序与投影一致
  * 拒绝路径：`valid=False`、`rejects` 带**实际值与阈值**
  * `normal_from_tf`：**缺任何一段都返回 None**，不静默返回单位阵
  * `diag` 里**法向来源可见**（`normal_source=tf|param`）
  * `_to_msg` 逐字段搬运、`header.stamp` 只能是输入图那帧的

⚠️ **不 import ROS 也能跑**：`build_payload` / `normal_from_tf` / `format_*` 是纯函数
（在 `skills/atomic/perception/pallet_detect/payload.py`）。需要 ROS 的那几条
（`_to_msg`）在环境缺依赖时自己 SKIP。

算法本身的对错由
`skills/atomic/perception/pallet_detect/tests/test_pallet_detect.py` 负责，这里不重复。
"""
from __future__ import annotations

import sys
from pathlib import Path

_HERE = Path(__file__).resolve()
_REPO_ROOT = _HERE.parents[6]          # tests -> pallet_detection -> ros_vision -> src -> ros_packages -> infrastructure -> 仓库根
sys.path.insert(0, str(_REPO_ROOT))
sys.path.insert(0, str(_HERE.parent.parent / 'scripts'))

import ast  # noqa: E402

import numpy as np  # noqa: E402

import pallet_detection_node as pdn  # noqa: E402
from skills.atomic.perception.pallet_detect.tests.test_pallet_detect import (  # noqa: E402
    H_MM, K, W_MM, make_truth, render,
)
from skills.atomic.perception.pallet_detect.algorithm import (  # noqa: E402
    _corners_mm, _project_px,
)
# ⚠️ `MM_TO_M` 在纯函数层 `payload.py` 里（`build_payload` 的所在地），节点脚本
# 只是调用者 —— 断言要钉的是**搬运时实际用的那个常量**，所以从 payload 取。
from skills.atomic.perception.pallet_detect.payload import (  # noqa: E402
    CHAIN, MM_TO_M, build_payload, format_diag, format_rejects,
)
from tf_normal import normal_from_tf  # noqa: E402

FAILS: list = []


def check(name, cond, detail=""):
    if cond:
        print(f"  PASS  {name}")
    else:
        print(f"  FAIL  {name}  {detail}")
        FAILS.append(name)


# --------------------------------------------------------------------------- #
# 合成场景（只跑一次，~2s）
# --------------------------------------------------------------------------- #
_TRUTH = make_truth()
_COLOR, _DEPTH = render(_TRUTH)


def _detect(normal=None, truth=None):
    from skills.atomic.perception.pallet_detect.algorithm import detect_pallet_frame
    truth = truth or _TRUTH
    return detect_pallet_frame(_COLOR, _DEPTH, K,
                               normal=normal if normal is not None else truth['nrm'],
                               target_mm=(W_MM, H_MM), long_side_parallel=True)

def test_valid_payload():
    """成功路径：`T_cam_pallet` / `det` / `corners_uv` / 行主序 / 各字段。"""
    found, diag = _detect()
    check("合成场景检出（前置）", found is not None, f"reject={diag.get('reject')}")
    if found is None:
        return
    p = build_payload(found, diag, K, latency_ms=12.5, normal_source='tf',
                          normal=_TRUTH['nrm'])

    # ---- 1) T_cam_pallet 与 found 的 origin/E1/E2/nrm 逐位一致 ----
    T = np.asarray(p['T_cam_pallet'], np.float64)
    check("T_cam_pallet 是 16 个数", T.size == 16, str(T.size))
    T = T.reshape(4, 4)
    check("T[:3,0] == E1", np.allclose(T[:3, 0], found['E1'], atol=0, rtol=0),
          str(T[:3, 0]))
    check("T[:3,1] == E2", np.allclose(T[:3, 1], found['E2'], atol=0, rtol=0),
          str(T[:3, 1]))
    check("T[:3,2] == nrm", np.allclose(T[:3, 2], found['nrm'], atol=0, rtol=0),
          str(T[:3, 2]))
    # ⚠️⚠️ **单位**：契约要求 `T_cam_pallet` 的平移是**米**，而
    # `detect_pallet_frame` 的 `origin` 是相机系**毫米** —— 这里**必须**是
    # `origin * 0.001`。写成 `== origin` 是**最自然的错误**，而且填错不报错：
    # 1000mm 被下游读成 1000m，位姿飞到天上，而 `det` 照样 +1、`valid` 照样 true。
    # 所以这条断言**故意写成乘以常量**，而不是写成"等于某个数"——
    # 常量改了它会跟着改，但"忘了乘"会立刻 FAIL。
    check("T[:3,3] == origin * MM_TO_M（**米**，不是毫米）",
          np.allclose(T[:3, 3], np.asarray(found['origin'], np.float64) * MM_TO_M,
                      atol=0, rtol=0),
          f"{T[:3, 3]} vs origin={found['origin']}")
    # 反面：单位搞错时**必须**能看出来。这条钉住"上面那条断言真的在测东西"——
    # 若哪天有人把 `MM_TO_M` 改成 1.0（或把 `* MM_TO_M` 删掉），上面那条会跟着
    # 一起"通过"而失去意义，这一条则会 FAIL（它要求平移的**量级**是米）。
    _mag = float(np.linalg.norm(T[:3, 3]))
    check("平移的量级是米（相机到托盘 ~1.6m，不是 ~1600）",
          0.1 < _mag < 100.0, f"|T[:3,3]|={_mag:.4f}（若 ≈1600 说明忘了乘 0.001）")
    check("最后一行是 [0,0,0,1]", np.allclose(T[3], [0, 0, 0, 1], atol=0, rtol=0),
          str(T[3]))

    # ---- 2) det ≈ +1，而且是**算出来的**不是写死的 ----
    det = float(p['det'])
    check("det ≈ +1", abs(det - 1.0) < 1e-9, f"det={det!r}")
    check("det == np.linalg.det(T[:3,:3])",
          abs(det - float(np.linalg.det(T[:3, :3]))) < 1e-15, f"det={det!r}")

    # ---- 3) corners_uv 与投影逐位一致，且顺序是 左下→右下→右上→左上 ----
    px = _project_px(_corners_mm(found, found['W'], found['H']), K)
    uv = np.asarray([[u, v] for u, v, _ in p['corners_uv']], np.float64)
    check("corners_uv 有 4 个点", uv.shape == (4, 2), str(uv.shape))
    check("corners_uv 与 _project_px(_corners_mm(...)) 逐位一致",
          np.allclose(uv, px, atol=0, rtol=0), f"{uv.tolist()} vs {px.tolist()}")
    check("corners_uv 的 z 恒 0",
          all(z == 0.0 for _, _, z in p['corners_uv']),
          str([z for _, _, z in p['corners_uv']]))
    # ⚠️ 顺序判据用**四角之间的关系**，不用"corner0 的 v 最大"这条**过强**的写法：
    # 本场景是斜视（法向 [0.05,-0.40,-0.915]），透视会让近边那一对的右端
    # （corner1）比左端（corner0）低 **7.3 px** —— 实测 corner0 v=165.4、
    # corner1 v=172.7、上边那一对 v≈7.6/-3.0。corner0 仍是"左下"（近边、最左），
    # 但"v 最大"在这个斜视场景下**不成立**。所以钉的是：
    #   corner0 的 u 最小（最左）+ corner0 与 corner1 构成**下边那一对**
    #   （两者的 v 都明显大于上边那一对）+ corner1 在 corner0 右边（E1 指向画面右）
    # 这三条合起来等价于"左下→右下→右上→左上"，且对透视稳健。
    check("顺序是左下：corner0 的 u 最小", int(np.argmin(uv[:, 0])) == 0,
          f"argmin(u)={int(np.argmin(uv[:, 0]))}  uv={np.round(uv, 1).tolist()}")
    check("顺序是左下：corner1 在 corner0 右边（E1 指向画面右）", uv[1, 0] > uv[0, 0],
          f"uv={np.round(uv, 1).tolist()}")
    check("顺序是左下：corner0/corner1 是下边那一对（v 都大于上边那一对）",
          min(uv[0, 1], uv[1, 1]) > max(uv[2, 1], uv[3, 1]),
          f"uv={np.round(uv, 1).tolist()}")
    check("顺序是左下：corner0 落在近边（v 大于上边那一对的中位）",
          uv[0, 1] > float(np.median(uv[2:, 1])),
          f"uv={np.round(uv, 1).tolist()}")

    # ---- 4) corner0 的投影与 origin 的投影重合 ----
    o_px = _project_px(np.asarray(found['origin'], np.float64)[None, :], K)[0]
    check("corner0 == origin 的投影", np.allclose(uv[0], o_px, atol=0, rtol=0),
          f"{uv[0].tolist()} vs {o_px.tolist()}")

    # ---- 5) 行主序：T @ (W,0,0,1) == origin + W*E1 ----
    # 这条**专门钉行主序**：列主序会让它静默转置（平移跑到第 4 行、旋转块转置）。
    # ⚠️⚠️ **托盘系的点也必须用米** —— `T` 的平移是米，而托盘系里 `W` 是毫米。
    # 两者混在一个齐次点里是**不自洽**的：正确的写法是 `(W*MM_TO_M, 0, 0, 1)`，
    # 结果也乘 `MM_TO_M`。这条断言第一版就是在这里写错的（左边 W 是毫米、
    # 右边又整体乘了 0.001，两边差一个 1000 倍），**恰好说明单位这个坑有多容易踩**。
    p_pallet = np.array([found['W'] * MM_TO_M, 0.0, 0.0, 1.0], np.float64)
    got = (T @ p_pallet)[:3]
    want = (np.asarray(found['origin'], np.float64) + found['W'] * np.asarray(
        found['E1'], np.float64)) * MM_TO_M
    check("行主序：T@(W*MM_TO_M,0,0,1) == (origin + W*E1) * MM_TO_M",
          np.allclose(got, want, atol=1e-12),
          f"{got.tolist()} vs {want.tolist()}")
    # ⚠️ 反证要拿**平移那一列**比：这个场景的旋转块里 E2 的 y 分量≈-0.4、
    # E1 的 z≈0，转置之后差异落在第 4 行（[0,0,0,1] -> [x,y,z,1]）与旋转块里，
    # 而"转置后 T@(W,0,0,1)"恰好与正确的点**相等**（平移不变、E1 只有 x 分量），
    # 所以单靠上面那一条**钉不住行主序**。下面两条才是真正的钉子。
    check("行主序：平移只在第 4 列（T[3,3]=1，T[3,:3]=0）",
          np.allclose(T[3], [0, 0, 0, 1], atol=0, rtol=0), str(T[3].tolist()))
    check("行主序：T[0:3,3] 就是 origin * MM_TO_M（列主序会让它跑到最后一行）",
          np.allclose(T[0:3, 3], np.asarray(found['origin'], np.float64) * MM_TO_M,
                      atol=0, rtol=0),
          f"{T[0:3, 3].tolist()} vs {(np.asarray(found['origin']) * MM_TO_M).tolist()}")
    # 托盘系点 (W, 0, 0, 1) 走**列主序读法**（= 用 T.T）应当**不**满足：
    # 列主序的平移在第 4 行，T.T @ p 取到的平移是 0 —— 一眼能看出来。
    got_t = (T.T @ p_pallet)[:3]
    check("列主序读法会得到不同的点（行主序不是无关紧要的）",
          not np.allclose(got_t, want, atol=1e-6),
          f"{got_t.tolist()} vs {want.tolist()}")

    # ---- 各标量字段 ----
    check("valid=True", p['valid'] is True, repr(p['valid']))
    check("source 透传 diag['source']", p['source'] == diag['source'],
          f"{p['source']!r} vs {diag['source']!r}")
    check("size_mm == [W, H]", p['size_mm'] == [float(found['W']), float(found['H'])],
          str(p['size_mm']))
    check("n_used/n_slots/n_failed == 1/1/0",
          (p['n_used'], p['n_slots'], p['n_failed']) == (1, 1, 0),
          str((p['n_used'], p['n_slots'], p['n_failed'])))
    check("spread_mm/spread_deg 单帧都是 0.0",
          (p['spread_mm'], p['spread_deg']) == (0.0, 0.0),
          str((p['spread_mm'], p['spread_deg'])))
    check("latency_ms 透传", p['latency_ms'] == 12.5, repr(p['latency_ms']))
    check("成功时 rejects 是空串", p['rejects'] == '', repr(p['rejects']))

    # ---- diag 里必须带上选层判据 + 法向来源 ----
    for want in ('score=', 'coverage=', 'n_edges=', 'deck_h_mm='):
        check(f"diag 带「{want}」", want in p['diag'], p['diag'])
    check("diag 里法向来源可见（normal_source=tf）", 'normal_source=tf' in p['diag'],
          p['diag'])
    check("diag 里带着法向的值", 'normal=' in p['diag'], p['diag'])

    # ---- 字段名与消息契约逐字一致（消息包编译过时才有这一条）----
    try:
        from pallet_detection_msgs.msg import PalletDetection
    except ImportError:
        print("  SKIP  消息包没编译，跳过「字段名逐字一致」这一条")
    else:
        want = [f for f in PalletDetection.__slots__ if f != 'header']
        got = list(p.keys())
        check("payload 的键 == 消息字段名（逐字、同序）", got == want,
              f"payload={got} msg={want}")


def test_det_is_computed_not_hardcoded():
    """`det` 是**手性判据**，写死 1.0 就等于废掉它。喂一个镜面看它会不会变 -1。"""
    found, diag = _detect()
    if found is None:
        check("合成场景检出（前置）", False, f"reject={diag.get('reject')}")
        return
    mirrored = dict(found)
    mirrored['E2'] = -np.asarray(found['E2'], np.float64)
    p = build_payload(mirrored, diag, K, latency_ms=1.0)
    check("镜面的 det ≈ -1（不是写死的 1.0）", abs(float(p['det']) + 1.0) < 1e-9,
          f"det={p['det']!r}")


def test_reject_payload_from_real_detection():
    """真实的拒绝也要能产出 payload，`rejects` 必须带**实际值与阈值**。"""
    # 法向绕垂直轴偏 90° -> 实测拒绝 no_deck（见 task-10 简报的法向敏感度表）
    bad = np.array([0.0, 0.0, -1.0], float)
    found, diag = _detect(normal=bad)
    check("给个错法向不会崩（前置）", True)
    if found is not None:
        print(f"  NOTE  这个错法向居然检出了（reject=None），跳过拒绝路径用例")
        return
    check("拒绝时 diag['reject'] 有值", bool(diag.get('reject')), str(diag.get('reject')))
    p = build_payload(None, diag, K, latency_ms=3.0, normal_source='tf',
                          normal=bad)
    check("valid=False", p['valid'] is False, repr(p['valid']))
    check("T_cam_pallet 是单位阵（占位值）",
          np.allclose(np.asarray(p['T_cam_pallet']).reshape(4, 4), np.eye(4)),
          str(p['T_cam_pallet']))
    check("det=1.0（占位值）", p['det'] == 1.0, repr(p['det']))
    check("size_mm=[0,0]（占位值）", p['size_mm'] == [0.0, 0.0], str(p['size_mm']))
    check("corners_uv 空（占位值）", p['corners_uv'] == [], str(p['corners_uv']))
    check("n_failed=1", p['n_failed'] == 1, repr(p['n_failed']))
    check("rejects 里点名了原因", f"reject={diag['reject']}" in p['rejects'],
          p['rejects'])
    check("rejects 里带实际值与阈值（不是只写「失败」）",
          ('<' in p['rejects'] or '>' in p['rejects'] or 'source=' in p['rejects']),
          p['rejects'])
    print(f"        rejects = {p['rejects']}")
    print(f"        diag    = {p['diag']}")


def test_reject_strings_carry_values_and_thresholds():
    """逐条拒绝码：`rejects` 必须写实际值与阈值。"""
    cases = [
        (dict(reject='ambiguous_theta', score=0.1696, source='color'),
         ('0.1696', '0.35')),
        (dict(reject='mask_too_large', mask_area_ratio=9.707, source='depth'),
         ('9.707', '3')),
        (dict(reject='too_few_edges', n_observed_edges=1, score=0.9),
         ('1', '2')),
        (dict(reject='no_wood', n_wood=12, source='color'), ('12', '500')),
        (dict(reject='degenerate', score=0.9), ('degenerate', '0.9')),
        (dict(reject='no_theta_ref', source='color'), ('no_theta_ref', 'source')),
    ]
    for diag, wants in cases:
        s = format_rejects(diag)
        check(f"rejects[{diag['reject']}] 带实际值与阈值",
              all(w in s for w in wants), f"{s!r} 缺 {[w for w in wants if w not in s]}")
    check("成功时 rejects 是空串", format_rejects(dict(reject=None)) == '',
          repr(format_rejects(dict(reject=None))))
    check("diag 里法向来源可见（param）",
          'normal_source=param' in format_diag({}, normal_source='param',
                                                   normal=[0, 0, -1]),
          format_diag({}, normal_source='param', normal=[0, 0, -1]))


# --------------------------------------------------------------------------- #
# normal_from_tf —— 假 TF 缓冲
# --------------------------------------------------------------------------- #
class _FakeBuf:
    """只实现 `lookup_transform(target, source, time)` 的最小缓冲。"""

    def __init__(self, edges: dict):
        self.edges = dict(edges)             # (parent, child) -> 4x4
        self.calls: list = []

    def lookup_transform(self, target, source, time):
        self.calls.append((target, source))
        if (target, source) in self.edges:
            return _to_tr(self.edges[(target, source)])
        if (source, target) in self.edges:
            return _to_tr(np.linalg.inv(self.edges[(source, target)]))
        raise LookupError(f"no transform {target} <- {source}")


def _tf_available() -> bool:
    """`tf.transformations` 在不在 —— 不在时 `_transform_to_matrix` 只会返回 None，
    那几条 tf 用例会**误报失败**（其实是环境没装 ROS）。"""
    try:
        import tf.transformations  # noqa: F401
    except ImportError:
        return False
    return True


def _to_tr(T):
    from types import SimpleNamespace
    from tf.transformations import quaternion_from_matrix
    q = quaternion_from_matrix(T)
    return SimpleNamespace(transform=SimpleNamespace(
        rotation=SimpleNamespace(x=q[0], y=q[1], z=q[2], w=q[3]),
        translation=SimpleNamespace(x=T[0, 3], y=T[1, 3], z=T[2, 3])))


def _rot_x(deg):
    T = np.eye(4)
    a = np.radians(deg)
    T[1, 1], T[1, 2], T[2, 1], T[2, 2] = np.cos(a), -np.sin(a), np.sin(a), np.cos(a)
    return T


def test_normal_from_tf_full_chain():
    """13 段齐 → 拼出 T，法向 = `T[:3,:3].T @ [0,0,1]`。"""
    edges = {e: np.eye(4) for e in CHAIN}
    edges[('base_link', 'base_to_joint')] = _rot_x(30.0)
    buf = _FakeBuf(edges)
    n = normal_from_tf(buf, 'camera_color_optical_frame', 'base_link', stamp=1.0)
    check("13 段齐时查得出来", n is not None, repr(n))
    if n is None:
        return
    want = _rot_x(30.0)[:3, :3].T @ np.array([0.0, 0.0, 1.0])
    check("法向 == T[:3,:3].T @ [0,0,1]", np.allclose(n, want, atol=1e-12),
          f"{np.round(n, 6).tolist()} vs {np.round(want, 6).tolist()}")
    check("是单位向量", abs(float(np.linalg.norm(n)) - 1.0) < 1e-12,
          str(float(np.linalg.norm(n))))


def test_normal_from_tf_missing_edge_returns_none():
    """★ 缺任何一段都返回 None —— **绝不静默返回单位阵**。

    静默返回单位阵 = 法向变成"竖直向下"，而检测**不会崩、不会报错**，
    只是换个 score 继续算（5_test 实测：source 从 color 变 depth）。
    """
    full = {e: np.eye(4) for e in CHAIN}
    missing = []
    for drop in (CHAIN[0], CHAIN[6], CHAIN[-1]):
        edges = {k: v for k, v in full.items() if k != drop}
        got_missing: list = []
        n = normal_from_tf(_FakeBuf(edges), 'camera_color_optical_frame',
                               'base_link', stamp=1.0, missing=got_missing)
        check(f"缺 {drop[0]}->{drop[1]} 时返回 None", n is None, repr(n))
        check(f"缺 {drop[0]}->{drop[1]} 时 missing 里点名了那一段",
              got_missing == [drop], str(got_missing))
    check("全缺时返回 None", normal_from_tf(_FakeBuf({}), 'cam', 'base_link',
                                                stamp=1.0) is None)


def test_normal_from_tf_uses_the_reverse_edge_too():
    """TF 里只登记了一个方向是常事 —— 反方向也要能拼。"""
    edges = {e: np.eye(4) for e in CHAIN}
    edges.pop(('base_link', 'base_to_joint'))
    edges[('base_to_joint', 'base_link')] = np.linalg.inv(_rot_x(30.0))
    n = normal_from_tf(_FakeBuf(edges), 'camera_color_optical_frame',
                           'base_link', stamp=1.0)
    check("只有反向边时也查得出来", n is not None, repr(n))
    if n is not None:
        want = _rot_x(30.0)[:3, :3].T @ np.array([0.0, 0.0, 1.0])
        check("反向边拼出来的法向正确", np.allclose(n, want, atol=1e-12),
              str(np.round(n, 6).tolist()))


def test_a_wrong_edge_still_yields_a_plausible_normal():
    """★ 链**不缺段但有一段的位姿是错的**时，会拼出一个"看着像模像样"的法向 ——
    查不出、也不报错。这正是"法向来源与值必须写进 `diag`"的理由：
    本函数只能保证"拼得出来"，保证不了"拼得对"。

    数值上演示一下偏差的量级：一段偏 5.6°（= T6 实测的 click vs TF 差），
    拼出来的法向就偏 5.6° —— 而 detect 在这个量级上的误差是 **180mm**。

    ⚠️ **但这个数字不是部署路径的终点误差**（2026-09-23 更正）：refine 的
    `_refine_plane` 是 SVD 拟合台面平面，**法向 2 + 高度 1 是它自己解的**，
    实测能把 5.6° 修到 **终点 13.2mm / dθ 0.1°**（守卫 `PLANE_MAX_TILT_DEG = 8.0`）。
    所以"180mm"是 **detect 单独**的数字，别拿它当部署结论 —— 见
    `test_refine_plane_is_on_by_default` 与 `.sdd-detect/progress.md` 的更正条。
    """
    edges = {e: np.eye(4) for e in CHAIN}
    edges[('head_camera_base', 'head_camera_depth')] = _rot_x(5.6)
    n = normal_from_tf(_FakeBuf(edges), 'camera_color_optical_frame',
                           'base_link', stamp=1.0)
    check("错一段也能拼出法向（查不出来）", n is not None, repr(n))
    if n is None:
        return
    ang = float(np.degrees(np.arccos(np.clip(abs(float(n @ np.array([0, 0, 1.0]))),
                                            -1, 1))))
    check("偏差就是那一段的 5.6°（不是 0）", abs(ang - 5.6) < 1e-6, f"{ang:.6f}°")
    print(f"        —— 5.6° 的偏差让 **detect 单独**的误差到 ~180mm；"
          f"但 refine 的平面拟合能修回来（终点 13.2mm），**前提是 `plane=True`**")


def test_refine_plane_is_on_by_default():
    """★ **部署必须开 `plane=True`**，而 `refine_pallet_frame` 的 CLI 默认是
    `False` —— 这个不一致是刻意的（见 `pallet_detection_node` 的 docstring）。

    这条钉住的是**默认值**：`~plane` 没给时必须是 `True`。
    `plane=False` 时 5.6° 的法向偏差会让**朝向停在 5.7°**，伺服拿去会歪。
    """
    src = Path(pdn.__file__).read_text(encoding='utf-8')
    check("节点源码里 ~plane 的默认值是 True",
          "get_param('~plane', True)" in src,
          "没找到 get_param('~plane', True)")
    check("节点源码里 ~refine 的默认值是 True",
          "get_param('~refine', True)" in src,
          "没找到 get_param('~refine', True)")
    # 参数解析层（不需要 ROS master）
    check("_as_bool('false') 是 False（字符串布尔）",
          pdn.PalletDetectionNode._as_bool('false') is False)
    check("_as_bool('True') 是 True",
          pdn.PalletDetectionNode._as_bool('True') is True)


def test_refine_cannot_work_on_this_synthetic_scene_and_why():
    """★ **如实记一条局限**：合成场景上 `refine_pallet_frame` 必然拒 `no_edges`，
    与 `plane` 开关无关。**原因是渲染器，不是 refine。**

    实测（本机 2026-09-23）：

      * 台面色 `BGR(93,145,190)` 的**灰度** = 152.5，地面灰 `BGR(150,150,150)` = 150.0
        —— 只差 **2.5**；
      * 整幅图梯度幅值 **max 3.13**（p99.9 = 3.11），而 refine 的 `GRAD_MIN = 60.0`。

    也就是说合成场景**根本没有亮度边缘**：台面与地面在彩色上分得开（木色 vs 灰，
    detect 走的就是颜色路线），在灰度上几乎一样 —— 而 refine 的边搜索是在**灰度**
    上做的。所以 refine 在这个场景上一条边都采不到（四边 `n_inliers=0`）。

    本用例**不做"应该能过"的断言**，只把这个事实钉住：合成场景能验 detect，
    **验不了 refine**；refine 的验证要用真实数据（5_test / april_test7）。
    """
    import cv2
    from skills.atomic.perception.pallet_detect import refine as rpf
    found, diag = _detect()
    if found is None:
        check("合成场景检出（前置）", False, f"reject={diag.get('reject')}")
        return

    def _lum(bgr):
        return 0.114 * bgr[0] + 0.587 * bgr[1] + 0.299 * bgr[2]

    d_lum = abs(_lum((93, 145, 190)) - _lum((150, 150, 150)))
    check("合成场景台面与地面的灰度差 < 5（所以没有亮度边缘）", d_lum < 5.0,
          f"Δ灰度={d_lum:.1f}")
    gray = cv2.cvtColor(_COLOR, cv2.COLOR_BGR2GRAY).astype(np.float32)
    blur = cv2.GaussianBlur(gray, (0, 0), sigmaX=3.0)
    mag = np.hypot(cv2.Sobel(blur, cv2.CV_32F, 1, 0, ksize=3),
                   cv2.Sobel(blur, cv2.CV_32F, 0, 1, ksize=3))
    check("整幅图的梯度幅值 max < refine 的 GRAD_MIN",
          float(mag.max()) < float(rpf.GRAD_MIN),
          f"max={float(mag.max()):.2f} vs GRAD_MIN={rpf.GRAD_MIN:g}")

    for plane in (True, False):
        ref, rdiag = rpf.refine_pallet_frame(found, _COLOR, _DEPTH, K,
                                             target_mm=(W_MM, H_MM),
                                             opts=dict(plane=plane))
        check(f"合成场景 refine(plane={plane}) 拒 no_edges（已知事实）",
              ref is None and rdiag.get('reject') == 'no_edges',
              f"ref={'None' if ref is None else 'frame'} "
              f"reject={rdiag.get('reject')} plane={rdiag.get('plane')}")
        check(f"plane={plane} 时四边都是 0 内点（GRAD_MIN 之下一条边都采不到）",
              all(e['n_inliers'] == 0 for e in rdiag.get('edges', [])),
              str([(e['name'], e['n_inliers']) for e in rdiag.get('edges', [])]))
    print("        —— 这是**渲染器**的局限（台面与地面灰度只差 2.5、梯度 max 3.13 "
          "vs GRAD_MIN 60），不是 refine 的错；")
    print("           合成场景能验 detect，**验不了 refine** —— refine 要用真实数据"
          "（5_test / april_test7）。")
    print("           ⚠️ 所以本节点那条 `plane=True` 的**效果**在合成场景上验不了，"
          "只能验**默认值**（见上一条用例）。")


def test_build_payload_carries_refine_landing():
    """`refine` 的落点必须写进 `diag`：`refine=ok|none`、`plane_refined=1|0`、
    `plane_offset_mm`。**没有它们就看不出版位到底有没有被子细化。**"""
    found, diag = _detect()
    if found is None:
        check("合成场景检出（前置）", False, f"reject={diag.get('reject')}")
        return
    fake_refine_diag = dict(reject=None, plane=dict(refined=True, note=None),
                            plane_offset_mm=12.3)
    p = build_payload(found, diag, K, latency_ms=5.0, normal_source='tf',
                          normal=_TRUTH['nrm'], refine_diag=fake_refine_diag,
                          refined=True)
    check("diag 里 refine=ok", 'refine=ok' in p['diag'], p['diag'])
    check("diag 里 plane_refined=1", 'plane_refined=1' in p['diag'], p['diag'])
    check("diag 里 plane_offset_mm=12.3", 'plane_offset_mm=12.3' in p['diag'],
          p['diag'])
    p2 = build_payload(found, diag, K, latency_ms=5.0,
                           refine_diag=dict(plane=dict(refined=False,
                                                       note='plane_fit_failed')),
                           refined=False)
    check("没细化的那次也留痕：refine=none + plane_refined=0 + note",
          'refine=none' in p2['diag'] and 'plane_refined=0' in p2['diag']
          and 'plane_note=plane_fit_failed' in p2['diag'], p2['diag'])


def test_normal_source_visibility():
    """`diag` 里法向来源必须可见（fit / cache / tf / param）—— 本节点最重要的行为。

    ⚠️ **`fit` 与 `cache` 必须能分开**（2026-09-29）：前者是"这一帧真的从深度
    重新拟合了"，后者是"复用首帧那次拟合"。混成一个标签，就回答不了
    "它到底有没有跟着场景变" —— 而缓存的**前提是相机相对 base 的朝向不变**，
    前提被破坏（动了腰 / 换工位）时的症状正是"标签看着没问题、数就是不对"。
    """
    s_fit = format_diag({'score': 0.9}, normal_source='fit', normal=[0, 0, -1])
    s_cache = format_diag({'score': 0.9}, normal_source='cache', normal=[0, 0, -1])
    s_tf = format_diag({'score': 0.9}, normal_source='tf', normal=[0, 0, -1])
    s_param = format_diag({'score': 0.9}, normal_source='param',
                              normal=[0, 0, -1])
    check("fit 来源写进了 diag", 'normal_source=fit' in s_fit, s_fit)
    check("cache 来源写进了 diag", 'normal_source=cache' in s_cache, s_cache)
    check("tf 来源写进了 diag", 'normal_source=tf' in s_tf, s_tf)
    check("param 来源写进了 diag", 'normal_source=param' in s_param, s_param)
    check("四种来源两两能区分开",
          len({s_fit, s_cache, s_tf, s_param}) == 4,
          f"{s_fit!r} / {s_cache!r} / {s_tf!r} / {s_param!r}")


def test_normal_fit_defaults_and_fallback():
    """法向自检的两个默认值 + 失败时**回退 TF 初值而不是拒帧**。

    ⚠️ 这几条是**静态读源码**（节点类只在真机上构造，测试不实例化它）——
    与 `test_no_undefined_module_names` 同一策略：宁可钉得粗一点，也不要在
    真机上才发现"默认值反了"。

    * `~fit_normal` 默认 **true** —— 关掉就退回"TF 给什么用什么"，
      而现场实测 TF 偏 12.4°、那时 detect 是 37/37 全拒。
    * `~normal_refit` 默认 **false** —— 伺服时机器人只做平面内平移，
      法向是不变量，首帧拟合一次就够（~10ms）。
    * 拟合失败时 `_normal_for_frame` 返回的是 **`prior` 加 `'tf'` 标签**，
      不是 `None`：初值本来就在夹角窗内，它只是"没被纠正"，不是"错的"。
      返回 None 会让节点拒帧 —— 那等于把"拟合不灵"升级成"整帧作废"。
    """
    src = Path(pdn.__file__).read_text(encoding='utf-8')
    check("~fit_normal 默认 true",
          "_as_bool(rospy.get_param('~fit_normal', True))" in src)
    check("~normal_refit 默认 false",
          "_as_bool(rospy.get_param('~normal_refit', False))" in src)
    check("拟合失败走回退分支（return prior, 'tf'）",
          "return prior, 'tf'" in src)
    tree = ast.parse(src)
    # `_normal_for_frame` 里必须有 `is None` 的分支，且那个分支 return 的不是 None
    fn = next((n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)
               and n.name == '_normal_for_frame'), None)
    check("_normal_for_frame 存在", fn is not None)
    if fn is not None:
        rets = [r for r in ast.walk(fn) if isinstance(r, ast.Return)]
        check("它有多个返回点（拟合成功 / 失败 / 缓存 / 关掉自检）",
              len(rets) >= 4, str(len(rets)))


def test_chain_is_thirteen_segments():
    """`payload.CHAIN` 必须是 **13 段**（base_link -> camera_color_optical_frame）。

    ⚠️ 与 maduo 的 `tools/tf_normal_from_bag.CHAIN` 是**同一份定义**（那边用来从
    bag 离线查、这边在线查）。maduo 侧有测试钉住"两份逐段相同"；本仓库没有那份
    参考，所以这里只钉住段数与首尾 —— **改链时两边都要改**。
    """
    check("CHAIN 是 13 段", len(CHAIN) == 13, str(len(CHAIN)))
    check("CHAIN 起点是 base_link", CHAIN[0][0] == 'base_link', str(CHAIN[0]))
    check("CHAIN 终点是相机光学系",
          CHAIN[-1][1] == 'camera_color_optical_frame', str(CHAIN[-1]))
    check("CHAIN 是首尾相接的一条链",
          all(CHAIN[i][1] == CHAIN[i + 1][0] for i in range(len(CHAIN) - 1)),
          str(CHAIN))


def test_to_msg_field_by_field():
    """`_to_msg` 的搬运：**有 ROS 与消息包时**逐字段核对（没有就 SKIP）。

    这一条专门抓"字段名/顺序搬错"那类只在运行期才炸的错。不需要 master ——
    `_to_msg` 只构造消息，不订阅、不发布。
    """
    try:
        ros = pdn._ros()
    except SystemExit as exc:
        print(f"  SKIP  ROS 或消息包不可用，跳过 _to_msg 核对（{exc}）")
        return
    from types import SimpleNamespace
    found, diag = _detect()
    if found is None:
        check("合成场景检出（前置）", False, f"reject={diag.get('reject')}")
        return
    p = build_payload(found, diag, K, latency_ms=7.0, normal_source='tf',
                          normal=_TRUTH['nrm'])
    header = SimpleNamespace(stamp=ros['rospy'].Time.from_sec(123.5), frame_id='cam')
    m = pdn.PalletDetectionNode._to_msg(SimpleNamespace(_ros=ros), p, header)

    check("header.stamp 就是输入图那帧的 stamp（不是 now()）",
          abs(m.header.stamp.to_sec() - 123.5) < 1e-9,
          str(m.header.stamp.to_sec()))
    check("header.frame_id 透传", m.header.frame_id == 'cam', m.header.frame_id)
    check("T_cam_pallet 逐位一致",
          list(m.T_cam_pallet) == list(p['T_cam_pallet']), str(m.T_cam_pallet[:3]))
    check("det 一致", abs(m.det - p['det']) < 1e-12, str(m.det))
    check("valid 一致", m.valid == p['valid'], str(m.valid))
    check("source 一致", m.source == p['source'], m.source)
    check("size_mm 一致", list(m.size_mm) == list(p['size_mm']), str(m.size_mm))
    check("corners_uv 的 xy 一致、z=0",
          all(abs(pt.x - u) < 1e-12 and abs(pt.y - v) < 1e-12 and pt.z == 0.0
              for pt, (u, v, _) in zip(m.corners_uv, p['corners_uv'])),
          str([(pt.x, pt.y) for pt in m.corners_uv]))
    check("n_used/n_slots/n_failed 一致",
          (m.n_used, m.n_slots, m.n_failed)
          == (p['n_used'], p['n_slots'], p['n_failed']),
          str((m.n_used, m.n_slots, m.n_failed)))
    check("spread_mm/spread_deg 一致",
          (m.spread_mm, m.spread_deg) == (p['spread_mm'], p['spread_deg']),
          str((m.spread_mm, m.spread_deg)))
    check("latency_ms 一致", abs(m.latency_ms - 7.0) < 1e-6, str(m.latency_ms))
    check("diag 一致", m.diag == p['diag'], m.diag)
    check("rejects 一致", m.rejects == p['rejects'], m.rejects)
    # 序列化一遍：字段类型不对时这里会炸（比如 float64[16] 塞了 numpy 标量）
    try:
        import io
        buf = io.BytesIO()
        m.serialize(buf)
        n_bytes = len(buf.getvalue())
    except Exception as exc:                         # noqa: BLE001
        check("能序列化成 ROS wire 格式", False, f'{type(exc).__name__}: {exc}')
    else:
        check("能序列化成 ROS wire 格式", n_bytes > 16, f'{n_bytes} bytes')

    # ★ 源码级钉子：`_to_msg` 的**代码**里不许出现 `.now()` 调用 —— 出现就是拿
    # 处理完成时刻当采集时刻，那是契约里点名的错（见 msg 末尾那段注释）。
    # ⚠️ 用 AST 而不是字符串搜索：那个函数的 **docstring 里就写着** `rospy.Time.now()`
    # （在解释"为什么不能这么写"），字符串搜索会把注释当成代码。
    # ⚠️ `inspect.getsource` 拿到的源文本带缩进，`ast.parse` 直接喂会
    # `IndentationError` —— 先用 `textwrap.dedent` 剥掉公共缩进。
    import ast
    import inspect
    import textwrap
    src = textwrap.dedent(inspect.getsource(pdn.PalletDetectionNode._to_msg))
    tree = ast.parse(src)
    calls = [n for n in ast.walk(tree)
             if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
             and n.func.attr == 'now']
    check("_to_msg 的代码里没有 .now() 调用（stamp 只能来自输入图的 header）",
          not calls, f'发现 {len(calls)} 处 .now() 调用')
    # 发布点也必须把**输入图的 header** 传进去
    run_body = inspect.getsource(pdn.PalletDetectionNode._run)
    check("发布时传的是 color_msg.header（不是别的 header）",
          '_to_msg(payload, color_msg.header)' in run_body,
          '没找到 _to_msg(payload, color_msg.header)')


# --------------------------------------------------------------------------- #
# 静态检查：整条 ROS 路径没有任何测试覆盖，只能靠 AST 兜
# --------------------------------------------------------------------------- #
def _module_scope_names(tree):
    """模块里**任何地方**绑定过的名字（模块级、函数内、赋值、import、参数…）。"""
    bound = set()
    for n in ast.walk(tree):
        if isinstance(n, ast.Import):
            bound |= {(a.asname or a.name).split('.')[0] for a in n.names}
        elif isinstance(n, ast.ImportFrom):
            bound |= {(a.asname or a.name) for a in n.names}
        elif isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            bound.add(n.name)
            if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef)):
                a = n.args
                for arg in (a.posonlyargs + a.args + a.kwonlyargs):
                    bound.add(arg.arg)
                if a.vararg:
                    bound.add(a.vararg.arg)
                if a.kwarg:
                    bound.add(a.kwarg.arg)
        elif isinstance(n, ast.Name) and isinstance(n.ctx, (ast.Store, ast.Del)):
            bound.add(n.id)
        elif isinstance(n, ast.ExceptHandler) and n.name:
            bound.add(n.name)
        elif isinstance(n, (ast.Global, ast.Nonlocal)):
            bound |= set(n.names)
    return bound


def test_no_undefined_module_names():
    """★ **节点脚本里不许有解析不到的名字。**

    为什么这条必须存在：`PalletDetectionNode.__init__` / `_run` 这些**只在真机上
    才跑**的方法，**单元测试从来不构造这个类**（只调 `_as_bool` / `_to_msg` 这类
    静态方法，或用 `SimpleNamespace(_ros=...)` 造假）。于是里面任何一个
    **漏掉的 import** 都会：单元测试全绿、`import` 也不报错（Python 的模块级名字
    是运行到那一行才查的），**只有真机启动节点时才炸**。

    实测踩过：`DEFAULT_TARGET_MM` 与 `CHAIN` 定义在纯函数层
    `pallet_detect/payload.py` 里，合并进 LeTools 时**忘了 import** ——
    `__init__` 第 233 / 292 行才 NameError，`roslaunch` 一启动就挂。
    两个自检脚本一条都没红。

    判据：把模块里**任何地方**绑定过的名字（模块级 / 函数内 import / 赋值 /
    参数 / 循环变量 / except as …）全收进来，剩下还引用得到的名字里，
    除去内建，就都是**解析不到的**。`rospy` / `cv2` / `tf2_ros` 这些
    **在函数内部 import** 的会正常落进 bound，不会误报。
    """
    import ast
    import builtins
    src = Path(pdn.__file__).read_text(encoding='utf-8')
    tree = ast.parse(src)
    bound = _module_scope_names(tree)
    # `__file__` / `__name__` 这类模块级 dunder 由解释器提供，不在 `dir(builtins)` 里
    builtin = set(dir(builtins)) | {'__file__', '__name__', '__doc__', '__package__',
                                    '__spec__', '__loader__', '__builtins__'}
    missing = sorted({n.id for n in ast.walk(tree)
                      if isinstance(n, ast.Name) and isinstance(n.ctx, ast.Load)
                      and n.id not in bound and n.id not in builtin})
    check("节点脚本里没有解析不到的名字（漏 import 只有真机才炸）",
          not missing, f"解析不到：{missing}")


def test_launch_param_style():
    """★ `target_mm` 在 launch 里**必须走 `<rosparam>`**，不能走 `<param value=...>`。

    `roslaunch` 的 `<param>` 不带 `type` 时按 `'auto'` 转换，而 `auto` **认不出
    list**（实测 `roslaunch.loader.convert_value`）：

        <param value="[1200.0, 1000.0]"/>               -> '[1200.0, 1000.0]'  (str)
        <rosparam param="...">[1200.0, 1000.0]</rosparam> -> [1200.0, 1000.0]    (list)

    字符串那一路进到节点里逐**字符**迭代，第一个字符是 `[` ——
    `ValueError: could not convert string to float: '['`，**节点起不来**。

    这条是**纯 XML 静态检查**，不需要 roslaunch；装了 roslaunch 时再真解析一遍。
    """
    import xml.etree.ElementTree as ET
    launch = (Path(pdn.__file__).resolve().parent.parent / 'launch'
              / 'pallet_detection.launch')
    check("launch 文件在", launch.exists(), str(launch))
    if not launch.exists():
        return
    root = ET.parse(str(launch)).getroot()
    node = root.find('node')
    check("launch 里有 <node>", node is not None)

    # target_mm 必须由 <rosparam> 提供，且不能同时被 <param> 覆盖
    rosparams = {e.get('param') for e in node.findall('rosparam')}
    params = {e.get('name') for e in node.findall('param')}
    check("target_mm 走 <rosparam>（YAML，给出真正的 list）",
          'target_mm' in rosparams, f"rosparam={sorted(rosparams)}")
    check("target_mm **不**同时走 <param value=...>（那会给字符串）",
          'target_mm' not in params, f"param={sorted(params)}")
    ro = next((e for e in node.findall('rosparam') if e.get('param') == 'target_mm'), None)
    if ro is not None:
        check("target_mm 的 <rosparam> 开了 subst_value（否则 $(arg) 不展开）",
              (ro.get('subst_value') or '').lower() == 'true', str(ro.attrib))

    # 装了 roslaunch 就真解析一遍，钉住"给出来的是 list 不是 str"
    try:
        import roslaunch.config as rlc
        import roslaunch.xmlloader as xl
    except ImportError as exc:
        print(f"  SKIP  没有 roslaunch（{exc}），跳过真解析")
        return
    cfg = rlc.ROSLaunchConfig()
    xl.XmlLoader().load(str(launch), cfg, verbose=False, argv=[])
    got = {k: v.value for k, v in cfg.params.items() if k.endswith('target_mm')}
    check("roslaunch 解析出的 target_mm 只有一处", len(got) == 1, str(got))
    if got:
        val = next(iter(got.values()))
        check("roslaunch 给出的 target_mm 是 list（不是 str）",
              isinstance(val, (list, tuple)), f"{val!r} type={type(val).__name__}")
        check("节点那行能吃下它",
              tuple(float(x) for x in val) == (1200.0, 1000.0), str(val))


def test_parse_target_mm_accepts_what_ros_actually_gives():
    """★ `_parse_target_mm` 要把 ROS 的**几种真实形态**都吃下。

    实测各来源给出什么（不是推理，是跑出来的）：

      * `roslaunch` 的 `<param value="[1200.0, 1000.0]"/>`（无 `type`）-> **字符串**
        （`roslaunch.loader.convert_value` 的 `'auto'` 分支认不出 list）；
      * `roslaunch` 的 `<rosparam>` -> **list**（走 YAML）—— 所以 launch 用这个；
      * 命令行 `_target_mm:="[1200,1000]"` -> **list**
        （`rospy.client.load_command_line_node_params` 用 `yaml.safe_load`）；
      * 命令行 `_target_mm:="1200,1000"`（没方括号）-> 字符串；
      * 命令行 `_target_mm:=""` -> `None`。

    解析不了时**必须抛错并说清该怎么写** —— 不要静默退回默认值：尺寸是搜索用的
    已知先验，静默用错尺寸会一路算出个像模像样的错位姿。
    """
    ok_cases = [
        ([1200.0, 1000.0], (1200.0, 1000.0)),
        ((1200.0, 1000.0), (1200.0, 1000.0)),
        ('[1200.0, 1000.0]', (1200.0, 1000.0)),
        ('[1200,1000]', (1200.0, 1000.0)),
        ('1200,1000', (1200.0, 1000.0)),
        ('1200 1000', (1200.0, 1000.0)),
        ('[1200;1000]', (1200.0, 1000.0)),
        ([1200, 1000], (1200.0, 1000.0)),
    ]
    for raw, want in ok_cases:
        try:
            got = pdn._parse_target_mm(raw)
        except Exception as exc:                     # noqa: BLE001
            check(f"_parse_target_mm({raw!r}) 能解析", False,
                  f"{type(exc).__name__}: {exc}")
            continue
        check(f"_parse_target_mm({raw!r}) -> {want}", got == want, repr(got))

    for raw in ('', '   ', 'abc', '[1]', '[1,2,3]', '[1200,0]', '[1200,-1000]', None):
        try:
            got = pdn._parse_target_mm(raw)
        except ValueError:
            check(f"_parse_target_mm({raw!r}) 抛 ValueError（不静默用默认值）", True)
        else:
            check(f"_parse_target_mm({raw!r}) 抛 ValueError（不静默用默认值）",
                  False, f"返回了 {got!r}")

    # ★ 反面：老写法 `tuple(float(v) for v in raw)` 在字符串上会炸 —— 钉住这一点，
    #   免得以后有人把 _parse_target_mm 换回那一行
    try:
        tuple(float(v) for v in '[1200.0, 1000.0]')
    except ValueError as exc:
        check("老写法在字符串上确实会炸（所以这个函数不是多余的）",
              "could not convert string to float" in str(exc), str(exc))
    else:
        check("老写法在字符串上确实会炸（所以这个函数不是多余的）", False,
              "居然没炸")


def main():
    print("== 静态检查：ROS 路径零覆盖，只能靠 AST 兜 ==")
    test_no_undefined_module_names()
    test_launch_param_style()
    test_parse_target_mm_accepts_what_ros_actually_gives()

    print("== build_payload：成功路径（合成场景，已知真值） ==")
    test_valid_payload()

    print("== det 是算出来的，不是写死的 ==")
    test_det_is_computed_not_hardcoded()

    print("== 拒绝路径 ==")
    test_reject_payload_from_real_detection()
    test_reject_strings_carry_values_and_thresholds()

    if not _tf_available():
        # `_transform_to_matrix` 要 `tf.transformations`；没有它时假 buffer 的
        # 矩阵换算恒返回 None，那几条 tf 用例会**误报失败**（其实是环境没装 ROS）。
        print("== normal_from_tf：缺一段就返回 None ==")
        print("  SKIP  没有 `tf.transformations`（ROS 没进 PYTHONPATH），"
              "normal_from_tf 的用例全部跳过")
    else:
        print("== normal_from_tf：缺一段就返回 None ==")
        test_normal_from_tf_full_chain()
        test_normal_from_tf_missing_edge_returns_none()
        test_normal_from_tf_uses_the_reverse_edge_too()
        test_a_wrong_edge_still_yields_a_plausible_normal()

    print("== 法向来源可见 ==")
    test_normal_source_visibility()
    test_normal_fit_defaults_and_fallback()

    print("== TF 链 ==")
    test_chain_is_thirteen_segments()

    print("== refine：plane=True 是部署的硬要求 ==")
    test_refine_plane_is_on_by_default()
    test_refine_cannot_work_on_this_synthetic_scene_and_why()
    test_build_payload_carries_refine_landing()

    print("== _to_msg：字段搬运（需要 ROS + 消息包，缺了 SKIP） ==")
    test_to_msg_field_by_field()

    print()
    if FAILS:
        print(f"失败 {len(FAILS)} 条：{FAILS}")
        return 1
    print("全部通过")
    return 0


if __name__ == '__main__':
    sys.exit(main())
