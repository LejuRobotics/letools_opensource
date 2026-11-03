#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""`pallet_detection_node` 的契约自检。跑法（退出码 0 通过 / 1 失败）：

    cd <仓库根>
    python3 infrastructure/ros_packages/src/ros_vision/pallet_detection/tests/test_pallet_detection_node.py

**不需要 ROS master、不需要相机、不需要任何数据、不需要真模型** —— 用合成场景
（已知真值）+ **假 OBB session**（`pallet_detect/tests/_synthetic.FakeObbSession`）
跑一遍检测，再逐条核对**契约**（不是算法）：

  * `T_cam_pallet` 的行主序、平移是**米**、`det` 自己算、`corners_uv` 的顺序与投影一致
  * 拒绝路径：`valid=False`、`rejects` 带**实际值与阈值**
  * `normal_from_tf`：**缺任何一段都返回 None**，不静默返回单位阵
  * `diag` 里**法向来源可见**（`normal_source=tf|param`）
  * `_to_msg` 逐字段搬运、`header.stamp` 只能是输入图那帧的
  * 静态检查：节点脚本无解析不到的名字、launch 参数写法、OBB 模型是**硬失败**

⚠️ **不 import ROS 也能跑**：`build_payload` / `normal_from_tf` / `format_*` 是纯函数
（在 `skills/atomic/perception/pallet_detect/payload.py`）。需要 ROS 的那几条
（`_to_msg`）在环境缺依赖时自己 SKIP。

算法本身的对错（含与 maduo 侧 601 帧逐位一致）由
`skills/atomic/perception/pallet_detect/tests/test_obb_detect.py` 负责，这里不重复。
"""
from __future__ import annotations

import sys
from pathlib import Path

_HERE = Path(__file__).resolve()
# ⚠️ 从前这里写的是 `_HERE.parents[6]`（**数层数**）。那种写法把"本包在仓库里的
# 深度"钉进了代码，挪一层就断 —— 2026-10-15 挪 `carton_obb` 时确认过这是全仓
# 最硬的一处依赖。现在**复用节点里那个按特征找的 `_repo_root()`**：判据只有一份，
# 而且节点在真机上怎么找根、测试在本地就怎么找根，两边不会漂。
sys.path.insert(0, str(_HERE.parent.parent / 'scripts'))

import ast  # noqa: E402
import re  # noqa: E402

import numpy as np  # noqa: E402

# 节点模块**自己**会在 import 时把仓库根插进 sys.path（它也得能单独跑），
# 所以这一句之后就找得到 `skills.atomic.perception.pallet_detect`。
import pallet_detection_node as pdn  # noqa: E402

_REPO_ROOT = pdn._REPO_ROOT
from skills.atomic.perception.pallet_detect.geom import (  # noqa: E402
    _corners_mm, _project_px,
)
from skills.atomic.perception.pallet_detect.detect import (  # noqa: E402
    DEFAULT_MODEL_PATH, detect_pallet_frame,
)
# POSE 后端（2026-10-18）：这两条 import 同时钉住"节点用到的名字解析得到"。
from skills.atomic.perception.pallet_detect.detect_pose import (  # noqa: E402
    DEFAULT_POSE_MODEL_PATH, detect_pallet_frame_pose,
)
# ⚠️ `MM_TO_M` 在纯函数层 `payload.py` 里（`build_payload` 的所在地），节点脚本
# 只是调用者 —— 断言要钉的是**搬运时实际用的那个常量**，所以从 payload 取。
from skills.atomic.perception.pallet_detect.payload import (  # noqa: E402
    CHAIN, MM_TO_M, build_payload, format_diag, format_rejects,
)
from skills.atomic.perception.pallet_detect.tests import _synthetic as S  # noqa: E402
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
_TRUTH = S.make_truth()
_COLOR, _DEPTH = S.render(_TRUTH)


def _detect(session=None):
    sess = session if session is not None else S.FakeObbSession(
        S.extract_landmarks(_TRUTH), _COLOR)
    return detect_pallet_frame(_COLOR, _DEPTH, S.K, normal=_TRUTH['nrm'],
                               target_mm=(S.W_MM, S.H_MM),
                               opts={'session': sess})


def test_valid_payload():
    """成功路径：`T_cam_pallet` / `det` / `corners_uv` / 行主序 / 各字段。"""
    found, diag = _detect()
    check("合成场景检出（前置）", found is not None, f"reject={diag.get('reject')}")
    if found is None:
        return
    p = build_payload(found, diag, S.K, latency_ms=12.5, normal_source='tf',
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

    # ---- 3) corners_uv 与投影逐位一致 ----
    px = _project_px(_corners_mm(found, found['W'], found['H']), S.K)
    uv = np.asarray([[u, v] for u, v, _ in p['corners_uv']], np.float64)
    check("corners_uv 有 4 个点", uv.shape == (4, 2), str(uv.shape))
    check("corners_uv 与 _project_px(_corners_mm(...)) 逐位一致",
          np.allclose(uv, px, atol=0, rtol=0), f"{uv.tolist()} vs {px.tolist()}")
    check("corners_uv 的 z 恒 0",
          all(z == 0.0 for _, _, z in p['corners_uv']),
          str([z for _, _, z in p['corners_uv']]))
    # 顺序判据用**四角之间的关系**（对透视稳健），不用"corner0 的 v 最大"那条过强的写法
    check("顺序是左下：corner0 的 u 最小", int(np.argmin(uv[:, 0])) == 0,
          f"argmin(u)={int(np.argmin(uv[:, 0]))}  uv={np.round(uv, 1).tolist()}")
    check("顺序是左下：corner1 在 corner0 右边（E1 指向画面右）", uv[1, 0] > uv[0, 0],
          f"uv={np.round(uv, 1).tolist()}")
    check("顺序是左下：corner0/corner1 是下边那一对（v 都大于上边那一对）",
          min(uv[0, 1], uv[1, 1]) > max(uv[2, 1], uv[3, 1]),
          f"uv={np.round(uv, 1).tolist()}")

    # ---- 4) corner0 的投影与 origin 的投影重合 ----
    o_px = _project_px(np.asarray(found['origin'], np.float64)[None, :], S.K)[0]
    check("corner0 == origin 的投影", np.allclose(uv[0], o_px, atol=0, rtol=0),
          f"{uv[0].tolist()} vs {o_px.tolist()}")

    # ---- 5) 行主序 ----
    # ⚠️⚠️ 托盘系的点也必须用米：`T` 的平移是米，而托盘系里 `W` 是毫米。
    p_pallet = np.array([found['W'] * MM_TO_M, 0.0, 0.0, 1.0], np.float64)
    got = (T @ p_pallet)[:3]
    want = (np.asarray(found['origin'], np.float64) + found['W'] * np.asarray(
        found['E1'], np.float64)) * MM_TO_M
    check("行主序：T@(W*MM_TO_M,0,0,1) == (origin + W*E1) * MM_TO_M",
          np.allclose(got, want, atol=1e-12), f"{got.tolist()} vs {want.tolist()}")
    check("行主序：平移只在第 4 列（T[3,3]=1，T[3,:3]=0）",
          np.allclose(T[3], [0, 0, 0, 1], atol=0, rtol=0), str(T[3].tolist()))
    check("行主序：T[0:3,3] 就是 origin * MM_TO_M（列主序会让它跑到最后一行）",
          np.allclose(T[0:3, 3], np.asarray(found['origin'], np.float64) * MM_TO_M,
                      atol=0, rtol=0),
          f"{T[0:3, 3].tolist()}")
    got_t = (T.T @ p_pallet)[:3]
    check("列主序读法会得到不同的点（行主序不是无关紧要的）",
          not np.allclose(got_t, want, atol=1e-6), f"{got_t.tolist()} vs {want.tolist()}")

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

    # ---- diag 里法向来源必须可见 ----
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
    p = build_payload(mirrored, diag, S.K, latency_ms=1.0)
    check("镜面的 det ≈ -1（不是写死的 1.0）", abs(float(p['det']) + 1.0) < 1e-9,
          f"det={p['det']!r}")


def test_reject_payload():
    """真实的拒绝也要能产出 payload，`rejects` 必须带**实际值与阈值/来源**。"""
    # 喂一个"完全没有 OBB 候选"的假 session（conf 全 0 -> 过滤后无候选）
    sess = S.FakeObbSession(S.extract_landmarks(_TRUTH), _COLOR, conf=0.0)
    found, diag = _detect(session=sess)
    check("没有候选时 detect 返回 None（前置）", found is None, str(diag))
    check("拒绝时 diag['reject'] 有值", bool(diag.get('reject')), str(diag.get('reject')))
    p = build_payload(None, diag, S.K, latency_ms=3.0, normal_source='tf',
                      normal=_TRUTH['nrm'])
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
    print(f"        rejects = {p['rejects']}")


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


class _LatestOnlyBuf(_FakeBuf):
    """只有 `Time(0)`（最新值）能查到；指定 stamp 一律查不到。

    用来钉 2026-11-03 的"过期 TF 宽容"：整条 13 段链必须先按 stamp 失败，
    再**整条**退回最新值，并把原始 stamp 交给 `stale`。
    """

    def _sec(self, time):
        return float(getattr(time, 'to_sec', lambda: 0.0)())

    def lookup_transform(self, target, source, time):
        self.calls.append((target, source, self._sec(time)))
        if self._sec(time) != 0.0:
            raise LookupError("no transform at requested stamp")
        if (target, source) in self.edges:
            return _to_tr(self.edges[(target, source)])
        if (source, target) in self.edges:
            return _to_tr(np.linalg.inv(self.edges[(source, target)]))
        raise LookupError(f"no transform {target} <- {source}")


def _tf_available() -> bool:
    """`tf.transformations` 在不在 —— 不在时那几条 tf 用例会**误报失败**。"""
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
    只是换个 score 继续算。OBB 路线把法向锁死，偏了更没人纠。
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


def test_normal_from_tf_tolerates_stale_whole_chain():
    """★ 过期 TF：按 stamp 拼不齐 -> **整条链**退回最新值，并报 stale。"""
    edges = {e: np.eye(4) for e in CHAIN}
    edges[('base_link', 'base_to_joint')] = _rot_x(30.0)
    buf = _LatestOnlyBuf(edges)
    stale: list = []
    missing: list = []
    n = normal_from_tf(buf, 'camera_color_optical_frame', 'base_link',
                       stamp=1.0, missing=missing, stale=stale)
    check("stamp 查不到时整条链退回最新值", n is not None, repr(n))
    check("stale 带回原始 stamp", stale == [1.0], str(stale))
    check("退回成功时 missing 不误报", missing == [], str(missing))
    if n is not None:
        want = _rot_x(30.0)[:3, :3].T @ np.array([0.0, 0.0, 1.0])
        check("退回最新值的法向正确", np.allclose(n, want, atol=1e-12),
              str(np.round(n, 6).tolist()))
    check("先按 stamp 试过（13 段），再按最新值试过",
          any(c[2] == 1.0 for c in buf.calls) and any(c[2] == 0.0 for c in buf.calls),
          str(buf.calls[:3]) + ' ...')

    # 正常能按 stamp 查到 -> 不许标 stale。
    fresh_stale: list = []
    fresh = normal_from_tf(_FakeBuf(edges), 'camera_color_optical_frame',
                           'base_link', stamp=1.0, stale=fresh_stale)
    check("stamp 能查到时不标 stale", fresh is not None and fresh_stale == [],
          f"{fresh is not None} / {fresh_stale}")


def _fake_normal_node():
    """`_normal_for_frame` 的最小假节点（只测法向来源的分支，不跑拟合）。"""
    from types import SimpleNamespace

    class _R:
        @staticmethod
        def logwarn_throttle(*a, **k):
            pass

        @staticmethod
        def loginfo_throttle(*a, **k):
            pass

    return SimpleNamespace(
        normal_param=None, normal_refit=False, fit_normal=True,
        _normal_fit=None, _normal_fit_log=None, _last_normal_prior=None,
        _ros={'rospy': _R()},
    )


def test_normal_for_frame_uses_cached_normal_before_tf():
    """★ 已通过自检的缓存法向优先于 TF：TF 停更不应拖垮后续帧。"""
    node = _fake_normal_node()
    cache = np.array([0.0, 0.1, -0.99498744], float)
    node._normal_fit = cache
    node.fit_normal = True
    called = []

    def _boom(msg):
        called.append(True)
        raise AssertionError('缓存命中时不该查 TF')

    node._resolve_normal = _boom
    n, src = pdn.PalletDetectionNode._normal_for_frame(node, object(), None)
    check("缓存法向直接返回", n is not None and np.allclose(n, cache), repr(n))
    check("来源标签是 cache", src == 'cache', src)
    check("缓存命中时 TF 一次都没查", called == [], str(called))


def test_normal_for_frame_falls_back_to_last_prior_when_tf_missing():
    """★ TF 彻底查不到时，用上一次成功的 TF 先验兜底，不丢帧。"""
    node = _fake_normal_node()
    prior = np.array([0.0, 0.2, -0.9797959], float)
    node._last_normal_prior = prior
    node.fit_normal = False       # 只看初值来源，不跑拟合
    node._resolve_normal = lambda msg: (None, 'TF 链缺边 [x]')
    n, src = pdn.PalletDetectionNode._normal_for_frame(node, object(), None)
    check("没有缓存时用上一次 TF 先验", n is not None and np.allclose(n, prior),
          repr(n))
    check("来源标签是 tf_last", src == 'tf_last', src)

    # 连上一次先验都没有 -> 仍然返回 None（不能凭空造法向）。
    node2 = _fake_normal_node()
    node2.fit_normal = False
    node2._resolve_normal = lambda msg: (None, 'TF 链缺边 [x]')
    n2, src2 = pdn.PalletDetectionNode._normal_for_frame(node2, object(), None)
    check("没有任何可用先验时仍返回 None", n2 is None, repr(n2))
    check("原因原样透出", src2 == 'TF 链缺边 [x]', src2)


def test_chain_is_thirteen_segments():
    """`payload.CHAIN` 必须是 **13 段**（base_link -> camera_color_optical_frame）。"""
    check("CHAIN 是 13 段", len(CHAIN) == 13, str(len(CHAIN)))
    check("CHAIN 起点是 base_link", CHAIN[0][0] == 'base_link', str(CHAIN[0]))
    check("CHAIN 终点是相机光学系",
          CHAIN[-1][1] == 'camera_color_optical_frame', str(CHAIN[-1]))
    check("CHAIN 是首尾相接的一条链",
          all(CHAIN[i][1] == CHAIN[i + 1][0] for i in range(len(CHAIN) - 1)),
          str(CHAIN))


def test_normal_source_visibility():
    """`diag` 里法向来源必须可见（tf / param）—— 本节点最重要的行为之一。

    ⚠️ OBB 路线把法向**锁死**成先验，偏了没有任何下游步骤纠正它，所以
    "用的是哪个法向、从哪来的"必须可见。
    """
    s_tf = format_diag({'score': 0.9}, normal_source='tf', normal=[0, 0, -1])
    s_param = format_diag({'score': 0.9}, normal_source='param', normal=[0, 0, -1])
    check("tf 来源写进了 diag", 'normal_source=tf' in s_tf, s_tf)
    check("param 来源写进了 diag", 'normal_source=param' in s_param, s_param)
    check("两种来源能区分开", s_tf != s_param, f"{s_tf!r} / {s_param!r}")


def test_normal_tilt_helper():
    """日志里的 `倾角=` 必须算对：事故那次 69.8° 要能一眼看出是侧向。"""
    tilt = pdn.PalletDetectionNode._normal_tilt_deg
    good = tilt([0.0297, -0.2672, -0.9632])      # 17:49 那次（正常）
    bad = tilt([-0.0042, -0.9388, -0.3445])      # 17:44 那次（侧向）
    check("正常法向倾角 ~15.6°", abs(good - 15.6) < 0.2, good)
    check("事故侧向法向倾角 ~69.9°", abs(bad - 69.9) < 0.2, bad)
    check("缺法向 -> None", tilt(None) is None, tilt(None))
    check("非法法向 -> None", tilt([0, 0, 0]) is None, tilt([0, 0, 0]))


def test_tf_listener_rebuild_is_throttled():
    """TF 订阅线程死后重建 Buffer+Listener；5s 内不反复重建。"""
    from types import SimpleNamespace
    calls = []

    class _TF:
        @staticmethod
        def Buffer():
            return object()

        @staticmethod
        def TransformListener(buf):
            calls.append(buf)
            return SimpleNamespace(unregister=lambda: None)

    class _R:
        @staticmethod
        def logwarn_throttle(*a, **k):
            pass

        @staticmethod
        def logerr_throttle(*a, **k):
            pass

    fake = SimpleNamespace(
        normal_param=None, tf_buffer=object(),
        tf_listener=SimpleNamespace(unregister=lambda: None),
        _last_tf_rebuild_mono=0.0, _tf_rebuild_min_s=5.0,
        _ros={'rospy': _R, 'tf2_ros': _TF})
    pdn.PalletDetectionNode._maybe_rebuild_tf(fake, 'test')
    check('TF 失败后重建一次', len(calls) == 1, calls)
    pdn.PalletDetectionNode._maybe_rebuild_tf(fake, 'test')
    check('5s 内不重复重建', len(calls) == 1, calls)

    fake.normal_param = [0.0, 0.0, -1.0]
    pdn.PalletDetectionNode._maybe_rebuild_tf(fake, 'param 模式')
    check('~normal 手给法向时不碰 TF', len(calls) == 1, calls)


def test_node_obb_defaults():
    """节点源码里 OBB 路线的几个默认值（**静态读源码** —— 节点类只在真机上构造）。"""
    src = Path(pdn.__file__).read_text(encoding='utf-8')
    check("~refine 默认 true", "get_param('~refine', True)" in src,
          "没找到 get_param('~refine', True)")
    check("~imgsz 默认 640", "get_param('~imgsz', 640)" in src,
          "没找到 get_param('~imgsz', 640)")
    check("~conf 默认 0.25", "get_param('~conf', 0.25)" in src,
          "没找到 get_param('~conf', 0.25)")
    check("~nms_iou 默认 0.45", "get_param('~nms_iou', 0.45)" in src,
          "没找到 get_param('~nms_iou', 0.45)")
    check("模型在启动时建（_build_session 被调用）",
          "self._sess = self._build_session()" in src,
          "没找到 self._sess = self._build_session()")
    check("~fit_normal / ~normal_refit（法向第二级）还在",
          "get_param('~fit_normal', True)" in src
          and "get_param('~normal_refit', False)" in src,
          "缺 ~fit_normal / ~normal_refit —— 法向第二级（深度自检纠 0~10° 偏差）不能丢")
    # 2026-10-22 现场事故：首帧拟歪被永久缓存 -> D 闸门全拒（pub=0）。
    check("法向拟合走**合理性自检**（不合格不缓存）",
          "fit_floor_normal_checked(" in src,
          "没用 fit_floor_normal_checked —— 歪拟合会被缓存并在整个进程内生效")
    check("~normal_max_tilt_deg / ~normal_d_check_tol 可配",
          "get_param('~normal_max_tilt_deg', 60.0)" in src
          and "get_param('~normal_d_check_tol', 0.35)" in src,
          "缺合理性自检的两个旋钮")
    check("D 闸门拒绝时会丢缓存法向（自愈）",
          "get_param('~normal_refit_on_d_reject', True)" in src
          and "丢弃缓存法向" in src,
          "D 拒绝后不清缓存 —— 首帧拟歪只能重启进程")
    # 2026-10-22 日志可读性：法向/倾角/D 偏离必须进心跳与拒绝日志。
    check("心跳带法向来源与倾角",
          "法向={self._last_normal_src}" in src
          and "倾角={tilt_txt}" in src,
          "心跳没打法向来源/倾角 —— 又会变成只有 D=241mm 看不出原因")
    # 2026-11-02：TF 订阅线程死亡 / 工作线程异常后不能静默停摆。
    check("工作线程有 supervisor（异常后重启）",
          "target=self._worker_main" in src and "def _worker_main(self)" in src,
          "工作线程没有兜底，异常会静默带走检测")
    check("TF 监听器可限频重建",
          "def _maybe_rebuild_tf(self" in src
          and "self._maybe_rebuild_tf(" in src
          and "TransformListener(self.tf_buffer)" in src,
          "TF 订阅线程死后不会重建，只能重启进程")
    check("心跳带 TF 年龄与线程重启次数",
          "tf={time.monotonic() - self._last_tf_ok_mono" in src
          and "线程重启={self._worker_restarts}" in src,
          "心跳看不出 TF 是否还活着 / 线程是否重启过")
    check("心跳的 D 带相对 ref 的偏差",
          "偏{dev * 100:.1f}%" in src and "ref={self.d_mm:.0f}mm" in src,
          "心跳的 D 没有偏差百分比/参考值")
    # 2026-11-03：过期 TF 的宽容必须可见、且不能破坏缓存优先级。
    check("TF 过期时来源标签是 tf_stale（不是普通 tf）",
          "'tf_stale'" in src, "过期 TF 被当成普通 tf，现场看不出")
    check("TF 彻底失败时用上一次 TF 先验兜底（tf_last）",
          "self._last_normal_prior" in src and "'tf_last'" in src,
          "没有 last-prior 兜底，TF 停更时整帧被丢")
    cache_idx = src.find(
        "if self._normal_fit is not None and not self.normal_refit")
    resolve_idx = src.find("prior, why = self._resolve_normal")
    check("缓存法向优先于 TF 查询（TF 停更不拖垮已缓存答案）",
          cache_idx != -1 and resolve_idx != -1 and cache_idx < resolve_idx,
          f"cache_idx={cache_idx} resolve_idx={resolve_idx}")
    check("TF 查询把 stale 列表传给 normal_from_tf",
          "stale=stale" in src and "stale.append(float(stamp))" in Path(
              pdn.__file__).parent.joinpath('tf_normal.py').read_text(
                  encoding='utf-8'),
          "normal_from_tf 没有把整条链退回最新值的事实传出来")
    check("D 拒绝日志带法向倾角与病因提示",
          "倾角={tilt_txt}" in src and '不是"没检测到托盘"' in src,
          "D 拒绝没有指向法向拟合的提示")
    check("旧路线的 ~deck_z_mm / ~use_prior 已删（OBB 路线用不上）",
          all(k not in src for k in ("get_param('~deck_z_mm'",
                                     "get_param('~use_prior'")),
          "还残留旧路线参数")


def test_pose_backend_wiring():
    """★ 两种前端（2026-10-18）：detector 默认/校验、pose 参数、分发、launch 透传。

    节点类只在真机上构造，所以这里全部是**静态读源码 + 解析 launch**。
    """
    src = Path(pdn.__file__).read_text(encoding='utf-8')
    check("~detector 默认 obb", "get_param('~detector', 'obb')" in src,
          "没找到 get_param('~detector', 'obb')")
    check("detector 只认 obb/pose，写错 SystemExit（错误里带允许值）",
          "('obb', 'pose')" in src and '允许值只有' in src,
          "没找到 detector 的取值校验")
    for key, default in (("pose_model_path", "''"), ("pose_conf", "0.25"),
                         ("pose_kconf", "0.5"), ("pose_pick", "'bottom_center'")):
        check(f"读 ~{key}（默认 {default}）",
              f"get_param('~{key}', {default})" in src,
              f"没找到 get_param('~{key}', {default})")
    check("~pose_edge_refine 默认 true",
          "get_param('~pose_edge_refine', True)" in src,
          "没找到 get_param('~pose_edge_refine', True)")
    check("_build_session 按后端选默认模型",
          "DEFAULT_POSE_MODEL_PATH" in src and "DEFAULT_MODEL_PATH" in src,
          "两种默认模型路径没有同时接上")
    check("启动日志打后端/路径/providers",
          '前端=%s  后端=%s  模型=%s' in src,
          "启动日志没打全 前端/后端/模型")
    check("providers 缺 CUDA 时告警（不自动改环境）",
          '没有 CUDAExecutionProvider' in src and 'LD_LIBRARY_PATH' in src,
          "没找到 CPU 静默回退的告警")
    check("_pose_detect_opts 存在", "def _pose_detect_opts(self)" in src,
          "没找到 _pose_detect_opts")
    for frag in ("conf=self.pose_conf", "pick=self.pose_pick",
                 "kconf=self.pose_kconf", "edge_refine=self.pose_edge_refine",
                 "d_lock='depth'", "fit_normal=False"):
        check(f"POSE opts 带 {frag}", frag in src, f"没找到 {frag}")
    check("_run 按 detector 分发到两个入口",
          "detect_pallet_frame_pose(" in src and "detect_pallet_frame(" in src,
          "分发缺失")
    check("默认 POSE 模型路径指向 models/pallet_pose.onnx",
          str(DEFAULT_POSE_MODEL_PATH).endswith('models/pallet_pose.onnx'),
          str(DEFAULT_POSE_MODEL_PATH))
    check("detect_pallet_frame_pose 可调用", callable(detect_pallet_frame_pose),
          repr(detect_pallet_frame_pose))

    import xml.etree.ElementTree as ET
    launch = (Path(pdn.__file__).resolve().parent.parent / 'launch'
              / 'pallet_detection.launch')
    root = ET.parse(str(launch)).getroot()
    args = {e.get('name') for e in root.findall('arg')}
    node = root.find('node')
    params = {e.get('name') for e in node.findall('param')} if node is not None else set()
    for k in ('detector', 'pose_model_path', 'pose_conf', 'pose_kconf',
              'pose_edge_refine', 'pose_pick'):
        check(f"launch 有 <arg {k}>", k in args, f"args={sorted(args)}")
        check(f"launch 把 {k} 透传成 <param>", k in params, f"params={sorted(params)}")
    for k in ('model_path', 'imgsz', 'conf', 'nms_iou', 'obb_pick', 'refine'):
        check(f"原有 launch 参数 {k} 还在", k in args and k in params,
              f"args={sorted(args)} params={sorted(params)}")
    for k in ('normal_max_tilt_deg', 'normal_d_check_tol',
              'normal_refit_on_d_reject'):
        check(f"launch 有 <arg {k}>", k in args, f"args={sorted(args)}")
        check(f"launch 把 {k} 透传成 <param>", k in params,
              f"params={sorted(params)}")

    # start_all.sh：按 detector 选 refine 键，两个键都不能漏。
    start_all = (_REPO_ROOT / 'orchestration' / 'scenarios'
                 / 'pallet_servo_real_v1' / 'start_all.sh')
    if start_all.exists():
        s = start_all.read_text(encoding='utf-8')
        check("start_all 按 detector 选 refine 键",
              'refine_key=pallet_pose_refine' in s
              and 'refine_key=pallet_refine' in s,
              "没找到 refine_key 的选择分支")
        check("start_all 把 detector 与 pose 参数传给 launch",
              '"detector:=${detector}"' in s
              and all(f'"{k}:=' in s for k in (
                  'pose_model_path', 'pose_conf', 'pose_kconf',
                  'pose_edge_refine', 'pose_pick')),
              "launch 命令行缺 pose 参数")
    else:
        check("找得到 start_all.sh", False, str(start_all))


def test_unified_logging_is_wired_after_init_node():
    """★ 统一日志接上了，且**在 `rospy.init_node()` 之后**收敛。

    这个顺序是**静默失效**的：`rospy.init_node()` 会往 root logger 上重装它自己的
    handler，把我们配好的清掉 —— 顺序反了不会报错，只是日志没有 trace id、不落盘。
    仓库里的既定写法见 `orchestration/nodes/boxcarry.py`（那里有一条同样的注释）。

    另外钉住**回退分支必须在**：本节点是 catkin 包里的独立节点，单独
    `roslaunch pallet_detection …` 时仓库根不一定在 `sys.path` 上，
    import 不到就退回 `rospy.log*`，功能不能少。
    """
    src = Path(pdn.__file__).read_text(encoding='utf-8')
    check("logger 的 import 带 try/except 回退",
          "from core.common.logger import init_logging as _init_logging" in src
          and "_init_logging = None" in src,
          "没找到带回退的 import")
    check("回退时说清楚日志去哪了（只走 rospy、不落盘）",
          '日志只走 rospy' in src and '不落盘' in src,
          "没找到回退告警")
    check("回退告警指出真实原因（不是路径问题，是缺依赖/旧部署）",
          'colorlog' in src,
          "告警还在说'仓库根不在 sys.path 上' —— 现在节点自己会插根，那会把人带偏")
    check("main 里调了 _init_unified_logging",
          "_init_unified_logging(ros['rospy'], '_pallet')" in src,
          "没找到调用")

    main_src = src.split('def main()', 1)[-1] if 'def main()' in src else ''
    i_node = main_src.find("init_node('pallet_detection')")
    i_log = main_src.find('_init_unified_logging(')
    check("init_logging 在 rospy.init_node **之后**（反了就静默失效）",
          0 <= i_node < i_log, f'init_node@{i_node} _init_unified_logging@{i_log}')
    check("启动时打出日志文件路径（现场先排除'没写进你翻的文件'）",
          'get_active_log_file' in src,
          "没找到 get_active_log_file")


def test_repo_root_is_found_by_feature_not_by_depth():
    """★ 仓库根按**特征**找（含 `skills/` 与 `infrastructure/` 的祖先），不数层数。

    从前的写法是 `Path(__file__).resolve().parents[6]` —— 把"本包在仓库里的深度"
    钉进代码。2026-10-15 挪 `carton_obb` 时确认过：这是全仓最硬的一处层级依赖，
    挪一层就断，而且断在 `import` 上、**只有真机上起节点才现形**（单元测试从来不
    构造节点类）。

    ⚠️ 判据必须走 **AST**，不能对源码做子串/正则匹配：本文件和节点的 **docstring**
    里都写着"从前是 `parents[6]`"这类说明文字（那正是要留下的教训），
    文本匹配会把"我在警告你别这么写"当成"你这么写了"。
    """
    def _depth_lookups(path) -> list:
        """源码里所有 `xxx.parents[<整数>]` 的**真实代码**用法（忽略注释与字符串）。"""
        tree = ast.parse(Path(path).read_text(encoding='utf-8'))
        hits = []
        for node in ast.walk(tree):
            if (isinstance(node, ast.Subscript)
                    and isinstance(node.value, ast.Attribute)
                    and node.value.attr == 'parents'
                    and isinstance(node.slice, ast.Constant)
                    and isinstance(node.slice.value, int)):
                hits.append(node.lineno)
        return hits

    node_src = Path(pdn.__file__).read_text(encoding='utf-8')
    node_hits = _depth_lookups(pdn.__file__)
    test_hits = _depth_lookups(__file__)
    check("节点源码里没有 `parents[数字]`（不数层数）",
          not node_hits, f'还在数层数，行 {node_hits}')
    check("测试源码里也没有 `parents[数字]`",
          not test_hits, f'还在数层数，行 {test_hits}')
    check("节点有按特征找的 _repo_root()",
          'def _repo_root()' in node_src, '没找到 _repo_root')
    check("判据是同时含 skills/ 与 infrastructure/",
          "(parent / 'skills').is_dir()" in node_src
          and "(parent / 'infrastructure').is_dir()" in node_src,
          '没找到特征判据')
    def _reuses_node_root(path) -> bool:
        """源码里有没有 `_REPO_ROOT = pdn._REPO_ROOT` 这条**赋值**。

        走 AST 而不是子串匹配：子串匹配会**匹配到自己这一行**（判据本身就写着
        那个字符串），于是恒真 —— 那是在自证，不是检查。
        """
        tree = ast.parse(Path(path).read_text(encoding='utf-8'))
        for node in ast.walk(tree):
            if not isinstance(node, ast.Assign):
                continue
            if not any(getattr(t, 'id', None) == '_REPO_ROOT' for t in node.targets):
                continue
            v = node.value
            if (isinstance(v, ast.Attribute) and v.attr == '_REPO_ROOT'
                    and getattr(v.value, 'id', None) == 'pdn'):
                return True
        return False

    check("测试复用节点找到的那个根（判据只有一份，两边不会漂）",
          _reuses_node_root(__file__), '测试自己算了一份根')
    check("算出来的就是仓库根",
          (_REPO_ROOT / 'skills').is_dir()
          and (_REPO_ROOT / 'infrastructure').is_dir(),
          str(_REPO_ROOT))


def test_d_gate_defaults_and_wiring():
    """★ **D 闸门**：节点的两个默认值 + 判据形状 + 只能有一处取值（静态读源码）。

    D 闸门的后果是**静默的**：参考值给错不会崩，只会把所有帧判超限、
    `/pallet/detection` 一帧都不发布 —— 现场看到的是"检测坏了"。所以这几条钉住：
    默认值可查、`<=0` 的关闸语义没被改掉、拒绝走的是既有那条路径（不发布）。
    """
    src = Path(pdn.__file__).read_text(encoding='utf-8')
    check("~d_mm 默认 890.0", "get_param('~d_mm', 890.0)" in src,
          "没找到 get_param('~d_mm', 890.0)")
    check("~d_tol 默认 0.1", "get_param('~d_tol', 0.1)" in src,
          "没找到 get_param('~d_tol', 0.1)")
    check("闸门用反解的单一真相源 D_from_frame（不是就地手写点乘）",
          "D_from_frame(final)" in src,
          "没找到 D_from_frame(final)")
    check("超限时**不发布**（走 continue，与既有拒绝路径同款）",
          "self._n_d_reject += 1" in src and "D_out_of_tol" in src,
          "没找到 D 闸门的拒绝分支")
    check("拒绝串交给 format_rejects 渲染（契约只有一处实现）",
          "'reject': 'D_out_of_tol', 'D_mm': D_now," in src,
          "没找到喂给 format_rejects 的最小 diag")
    check("`<=0` = 关闸（不是判成全部超限）",
          "self.d_gate_on = (self.d_mm > 0.0) and (self.d_tol > 0.0)" in src,
          "没找到 d_gate_on 的判据")
    check("每个闸门参数只取一次 value（写两处必然打架）",
          src.count("get_param('~d_mm'") == 1
          and src.count("get_param('~d_tol'") == 1,
          f"d_mm={src.count(chr(39) + '~d_mm' + chr(39))} "
          f"d_tol={src.count(chr(39) + '~d_tol' + chr(39))}")
    check("心跳里带实测 D（现场核对 890 的唯一证据）",
          "D={self._last_D_mm:.0f}mm" in src
          and "d闸={self._n_d_reject}" in src,
          "心跳没带 d闸 / D")


def test_to_msg_field_by_field():
    """`_to_msg` 的搬运：**有 ROS 与消息包时**逐字段核对（没有就 SKIP）。"""
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
    p = build_payload(found, diag, S.K, latency_ms=7.0, normal_source='tf',
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
    try:
        import io
        buf = io.BytesIO()
        m.serialize(buf)
        n_bytes = len(buf.getvalue())
    except Exception as exc:                         # noqa: BLE001
        check("能序列化成 ROS wire 格式", False, f'{type(exc).__name__}: {exc}')
    else:
        check("能序列化成 ROS wire 格式", n_bytes > 16, f'{n_bytes} bytes')

    # ★ 源码级钉子：`_to_msg` 的代码里不许出现 `.now()`
    import inspect
    import textwrap
    src = textwrap.dedent(inspect.getsource(pdn.PalletDetectionNode._to_msg))
    tree = ast.parse(src)
    calls = [n for n in ast.walk(tree)
             if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
             and n.func.attr == 'now']
    check("_to_msg 的代码里没有 .now() 调用（stamp 只能来自输入图的 header）",
          not calls, f'发现 {len(calls)} 处 .now() 调用')
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
    """★ **节点脚本里不许有解析不到的名字。**（漏 import 只有真机才炸）

    判据：把模块里**任何地方**绑定过的名字全收进来，剩下还引用得到的名字里，
    除去内建，就都是解析不到的。`rospy` / `cv2` / `onnxruntime` 这些**在函数内部
    import** 的会正常落进 bound，不会误报。
    """
    import builtins
    src = Path(pdn.__file__).read_text(encoding='utf-8')
    tree = ast.parse(src)
    bound = _module_scope_names(tree)
    builtin = set(dir(builtins)) | {'__file__', '__name__', '__doc__', '__package__',
                                    '__spec__', '__loader__', '__builtins__'}
    missing = sorted({n.id for n in ast.walk(tree)
                      if isinstance(n, ast.Name) and isinstance(n.ctx, ast.Load)
                      and n.id not in bound and n.id not in builtin})
    check("节点脚本里没有解析不到的名字（漏 import 只有真机才炸）",
          not missing, f"解析不到：{missing}")


def test_inference_backend_single_source_of_truth():
    """★★ 推理后端的**唯一实现**在算法包；节点只调用，不再自己建 session。

    2026-10-20 起支持 TensorRT engine（`pallet_detect/trt_backend.py`）：
    `auto` 看 engine 文件在不在，`engine` 强制、`onnx` 强制旧路径。后端选择与
    provider 顺序都只在 `detect._resolve_session` / `detect_pose._resolve_pose_session`
    里写一份 —— 从前节点与算法层各写一份 provider 列表，改一处忘一处就会静默
    不一致。这里钉住"只剩一份"。

    仍保留的两条 ONNX 约束：CUDA 在 CPU 前；**不列 ONNX Runtime 的
    `TensorrtExecutionProvider`**（它启动时现场编译、不落盘，每起一次节点要
    三分钟以上 —— 要用 TRT 就走预导出的 engine）。
    """
    import re as _re
    det = Path(pdn.__file__).resolve()
    # 反查仓库根，与节点脚本同一手法。⚠️ 从**文件**开始往上找，
    # 所以要足够多层（…/scripts/x.py → scripts → pallet_detection → ros_vision
    # → src → ros_packages → infrastructure → <仓库根> = 7 层）。
    root = det
    for _ in range(10):
        root = root.parent
        if (root / 'skills').is_dir() and (root / 'infrastructure').is_dir():
            break
    detect_py = root / 'skills/atomic/perception/pallet_detect/detect.py'
    pose_py = root / 'skills/atomic/perception/pallet_detect/detect_pose.py'
    trt_py = root / 'skills/atomic/perception/pallet_detect/trt_backend.py'
    check("找得到 detect.py（反查仓库根）", detect_py.exists(), str(detect_py))
    if not detect_py.exists():
        return

    def provider_tuple(src):
        m = _re.search(r'providers\s*=\s*\[p for p in \(([^)]*)\)', src)
        return [x.strip().strip("'\"") for x in m.group(1).split(',') if x.strip()] if m else None

    a = provider_tuple(detect_py.read_text(encoding='utf-8'))
    check("detect.py 里解析出 provider 列表", a is not None, str(a))
    check("detect.py 不列 TensorrtExecutionProvider（TRT 走静态 engine）",
          'TensorrtExecutionProvider' not in (a or []), str(a))
    check("CUDA 排在 CPU 前面",
          (a or []).index('CUDAExecutionProvider') < (a or []).index('CPUExecutionProvider'),
          str(a))

    node_src = det.read_text(encoding='utf-8')
    check("节点不再自己建 ORT session（委托算法层）",
          'ort.InferenceSession(' not in node_src,
          '节点里还有 ort.InferenceSession —— 后端会分叉')
    check("节点调用 detect._resolve_session / detect_pose._resolve_pose_session",
          '_resolve_session' in node_src and '_resolve_pose_session' in node_src
          and 'resolver(opts)' in node_src,
          '没找到对算法层 resolver 的调用')
    check("节点不再自己写 provider 列表",
          provider_tuple(node_src) is None,
          '节点里还有 provider 列表')
    check("engine 后端文件存在", trt_py.exists(), str(trt_py))
    if trt_py.exists():
        trt_src = trt_py.read_text(encoding='utf-8')
        check("trt_backend 提供 TensorRTSession",
              'class TensorRTSession' in trt_src, '没找到 TensorRTSession')
        check("TensorRTSession 校验 engine 输入尺寸",
              'expected_input' in trt_src, '没校验 expected_input')
    for src, name in ((detect_py.read_text(encoding='utf-8'), 'detect.py'),
                      (pose_py.read_text(encoding='utf-8'), 'detect_pose.py')):
        check(f"{name} 有三档 backend（auto/engine/onnx）",
              "_BACKENDS = ('auto', 'engine', 'onnx')" in src,
              f'{name} 没找到 _BACKENDS')
        check(f"{name} auto 看 engine 文件是否存在",
              'engine_path.is_file()' in src, f'{name} 没找到 engine 文件判据')


def test_image_subscribers_use_a_large_buff_size():
    """★★ 收图的两个订阅**必须显式给 `buff_size`**，否则退回 rospy 的 64KB 默认值。

    `message_filters.Subscriber` 把参数原样转给 `rospy.Subscriber`，而
    `rospy.Subscriber` 的签名默认是 **`buff_size=65536`** —— 一条 640×480 的
    彩色图是 **921KB**，用 64KB 的缓冲去收要分 ~14 次读，全在 Python 层，
    **抢的是 worker 线程的 GIL**。

    实测（Orin AGX，同一份数据、同一个算法，只改相机喂帧率）：
        相机 6Hz  → 单帧 177ms
        相机 30Hz → 单帧 290ms（**慢 1.64 倍**），且偶发 5~6 秒的停顿
    而离线逐帧扫 816 帧、最慢一帧只要 202ms —— **数据是干净的**。

    仓库里别的收图节点都设了（`carton_box_yolo` / `node_pallet_servo` /
    `wrist_visual_servo`），**只有本节点漏了**，所以这条网钉住"别再漏"。
    """
    src = Path(pdn.__file__).read_text(encoding='utf-8')
    i = src.find('subs = [ros[')
    check('找得到建订阅那一段', i > 0)
    if i < 0:
        return
    seg = src[i:i + 900]
    check('彩色/深度订阅给了 buff_size', seg.count('buff_size') >= 2,
          f"只出现 {seg.count('buff_size')} 次")
    check('buff_size 不小于 2**24', '2 ** 24' in seg or '2**24' in seg, seg[:160])
    check('给了 queue_size=1（不给会退回**无界**队列）',
          seg.count('queue_size=1') >= 2, f"只出现 {seg.count('queue_size=1')} 次")
    check('开了 tcp_nodelay', 'tcp_nodelay' in seg)


def test_launch_param_style():
    """★ `target_mm` 在 launch 里**必须走 `<rosparam>`**，不能走 `<param value=...>`。

    `roslaunch` 的 `<param>` 不带 `type` 时按 `'auto'` 转换，而 `auto` **认不出
    list**：`"[1200.0, 1000.0]"` 会原样变成字符串，节点拿到它逐字符迭代、
    `ValueError: could not convert string to float: '['`，**节点起不来**。
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

    rosparams = {e.get('param') for e in node.findall('rosparam')}
    params = {e.get('name') for e in node.findall('param')}
    check("target_mm 走 <rosparam>（YAML，给出真正的 list）",
          'target_mm' in rosparams, f"rosparam={sorted(rosparams)}")
    check("target_mm **不**同时走 <param value=...>（那会给字符串）",
          'target_mm' not in params, f"param={sorted(params)}")

    # D 闸门是**标量**，走 <param> 才对（<rosparam> 是给 list 用的）。
    # 钉住"launch 真的把它透传下去了" —— 只加 <arg> 不加 <param> 的话，
    # 一键脚本传进来会被静默丢掉，节点拿默认值跑，现场看不出来。
    for k in ('d_mm', 'd_tol'):
        check(f"{k} 走 <param value=...>（标量，不是 rosparam）",
              k in params, f"param={sorted(params)}")
        check(f"{k} **不**走 <rosparam>（那是给 list 的）",
              k not in rosparams, f"rosparam={sorted(rosparams)}")
    ro = next((e for e in node.findall('rosparam') if e.get('param') == 'target_mm'), None)
    if ro is not None:
        check("target_mm 的 <rosparam> 开了 subst_value（否则 $(arg) 不展开）",
              (ro.get('subst_value') or '').lower() == 'true', str(ro.attrib))

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

    实测各来源给出什么（不是推理，是跑出来的）：roslaunch `<param value=...>` 无
    type -> 字符串；`<rosparam>` -> list；命令行 `_"..."` -> list；命令行不带方
    括号 -> 字符串；空串 -> None。解析不了时**必须抛错并说清该怎么写**。
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


def test_model_is_a_hard_failure():
    """★ 托盘检测**必须有 OBB 模型**：缺失 / 加载失败直接抛，不静默返回 None。"""
    color, depth = _COLOR, _DEPTH
    try:
        detect_pallet_frame(color, depth, S.K, normal=_TRUTH['nrm'],
                            opts={'backend': 'onnx',
                                  'model_path': '/nonexistent/pallet_obb.onnx'})
    except FileNotFoundError as exc:
        msg = str(exc)
        check("缺模型抛 FileNotFoundError", True)
        check("错误信息里有期望/实际路径", '/nonexistent/pallet_obb.onnx' in msg, msg)
        check("错误信息里有导出办法（export_onnx.py）", 'export_onnx.py' in msg, msg)
    except Exception as exc:                                   # noqa: BLE001
        check("缺模型抛 FileNotFoundError", False,
              f'抛了 {type(exc).__name__}: {exc}')
    else:
        check("缺模型抛 FileNotFoundError", False, '居然没抛，静默返回了 None')
    check("默认模型路径指向 models/pallet_obb.onnx",
          str(DEFAULT_MODEL_PATH).endswith('models/pallet_obb.onnx'),
          str(DEFAULT_MODEL_PATH))


def main():
    print("== 静态检查：ROS 路径零覆盖，只能靠 AST 兜 ==")
    test_no_undefined_module_names()
    test_launch_param_style()
    test_parse_target_mm_accepts_what_ros_actually_gives()

    print("== 模型是硬失败（缺失即抛） ==")
    test_model_is_a_hard_failure()

    print("== build_payload：成功路径（合成场景，已知真值） ==")
    test_valid_payload()

    print("== det 是算出来的，不是写死的 ==")
    test_det_is_computed_not_hardcoded()

    print("== 拒绝路径 ==")
    test_reject_payload()
    test_reject_strings_carry_values_and_thresholds()

    if not _tf_available():
        print("== normal_from_tf：缺一段就返回 None ==")
        print("  SKIP  没有 `tf.transformations`（ROS 没进 PYTHONPATH），"
              "normal_from_tf 的用例全部跳过")
    else:
        print("== normal_from_tf：缺一段就返回 None ==")
        test_normal_from_tf_full_chain()
        test_normal_from_tf_missing_edge_returns_none()
        test_normal_from_tf_uses_the_reverse_edge_too()
        test_normal_from_tf_tolerates_stale_whole_chain()

    print("== 过期 TF：缓存/上一次先验兜底 ==")
    test_normal_for_frame_uses_cached_normal_before_tf()
    test_normal_for_frame_falls_back_to_last_prior_when_tf_missing()

    print("== 法向来源可见 ==")
    test_normal_source_visibility()
    test_normal_tilt_helper()

    print("== TF 监听器自愈（限频重建） ==")
    test_tf_listener_rebuild_is_throttled()

    print("== TF 链 ==")
    test_chain_is_thirteen_segments()

    print("== 节点默认值（静态读源码） ==")
    test_node_obb_defaults()

    print("== 两种前端：obb / pose（静态读源码 + launch） ==")
    test_pose_backend_wiring()

    print("== D 闸门：默认值 / 判据 / 只能一处取值 ==")
    test_d_gate_defaults_and_wiring()

    print("== 统一日志接线（顺序 + 回退） ==")
    test_unified_logging_is_wired_after_init_node()

    print("== 仓库根按特征找，不数层数 ==")
    test_repo_root_is_found_by_feature_not_by_depth()

    print("== 推理后端：唯一实现 + engine/onnx 选择 ==")
    test_inference_backend_single_source_of_truth()

    print("== 图像订阅的 buff_size（rospy 默认 64KB 会拖慢收图） ==")
    test_image_subscribers_use_a_large_buff_size()

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
