# -*- coding: utf-8 -*-
"""`detect` / `refine` 的输出 -> `pallet_detection_msgs/PalletDetection` 的字段。
**纯函数、零状态、不 import ROS。**

分层的意义：业务逻辑（怎么把 frame 拆成 16 个 double、单位怎么换、诊断串里该带
哪些字段）全在这里，能被单元测试直接调；ROS 那一层（订阅 / 出队 / 填消息 / 发布）
只剩搬运，改算法不用动它 —— 与 `box_frame/window.py` 的 `build_payload` 同一分层。

    build_payload(found, diag, k, ...)  ->  dict，键**逐字对应消息字段名**
    format_diag(diag, ...)              ->  逗号分隔的 key=value（格式不进契约）
    format_rejects(diag)                ->  失败槽的拒绝原因（必须写实际值与阈值）

## 两条最容易搞错的地方（都在 `build_payload` 的 docstring 里展开了）

1. **`T_cam_pallet` 的平移单位是「米」，不是毫米。** 填错**不报错**：1000mm 被
   下游读成 1000m，位姿飞到天上，而 `det` 照样 +1、`valid` 照样 true。
2. **`corners_uv` 的顺序是「左下→右下→右上→左上」**，与
   `box_detection_msgs/BoxDetection.corners_uv` 的顺序**不一样**。不是笔误：
   托盘有手性要求（`e1 × e2 = normal`），环序必须跟着 e1/e2 走。

⚠️ **本文件不 import ROS。** 从 TF 链查法向那一组函数
（`normal_from_tf` / `_lookup_edge` / `_ros_time` / `_transform_to_matrix`）
在同目录的 `tf_normal.py` 里，那个文件会 `import rospy` —— 分层要求 ROS 只出现在
`infrastructure/`，`skills/` 下不许出现。本文件只做纯搬运。
"""
from __future__ import annotations

import numpy as np

from .algorithm import COVERAGE_MIN, MASK_AREA_RATIO_MAX, MIN_WOOD_PX
from .algorithm import _corners_mm, _project_px



# 从 `base_link` 走到相机光学系的链，**13 段**。部署时逐段查 TF 拼出
# `T_base_camera`，再把 base_link 的 +z 转进相机系就是台面法向先验。
# 与 maduo 的 `tools/tf_normal_from_bag.py` **共用同一份定义**（那边用来从 bag
# 离线查、这边在线查）。⚠️ 这里**不能**直接 import 那个模块 —— 它顶层
# `import rosbag`，会让本文件**没有 ROS 环境变量就 import 不了**，而本文件是
# 纯函数层、单元测试要能直接调。
CHAIN = [
    ('base_link', 'base_to_joint'),
    ('base_to_joint', 'knee_link'),
    ('knee_link', 'leg_link'),
    ('leg_link', 'waist_link'),
    ('waist_link', 'waist_yaw_link'),
    ('waist_yaw_link', 'zhead_1_link'),
    ('zhead_1_link', 'zhead_2_link'),
    ('zhead_2_link', 'head_camera_base'),
    ('head_camera_base', 'head_camera_depth'),
    ('head_camera_depth', 'camera_link'),
    ('camera_link', 'camera_depth_frame'),
    ('camera_depth_frame', 'camera_color_frame'),
    ('camera_color_frame', 'camera_color_optical_frame'),
]

# 上游自报的法向与本节点实际用的对不上时，`diag` 里要能看出来。
DEFAULT_TARGET_MM = (1200.0, 1000.0)

# ⚠️ **契约要求 `T_cam_pallet` 的平移是「米」**，而 `detect_pallet_frame` 的
# `frame['origin']` 是相机系**毫米**。两者之间只有这一个换算，**别内联到代码里** ——
# 这个数字出现在 `build_payload` 和测试两处，散开写迟早会有一处漏掉。
# 完整的坑说明（为什么填错不会报错、为什么下游不会纠正）见 `build_payload` 的 docstring。
MM_TO_M = 0.001


# --------------------------------------------------------------------------- #
# 纯函数层（**不 import ROS**，单元测试直接调）
# --------------------------------------------------------------------------- #
def _normal_source_tag(diag: dict, normal_source: str | None) -> str:
    """法向来源的标签。`fit` / `cache` / `tf` / `param`；没有就按 `diag` 猜。

    ⚠️ **`fit` 与 `cache` 必须分开**（2026-09-29）：前者是"这一帧真的从深度
    重新拟合了"，后者是"用的是首帧那次拟合的结果"。混成一个标签，就没法回答
    "它到底有没有跟着场景变" —— 而缓存的**前提是相机相对 base 的朝向不变**，
    前提一旦被破坏（动了腰 / 换了工位），症状正是"标签看着没问题、数就是不对"。
    """
    if normal_source:
        return str(normal_source)
    return 'param' if diag.get('_normal') is not None else '?'


def format_diag(diag: dict, normal_source: str | None = None,
                normal=None, refine_diag: dict | None = None,
                refined: bool | None = None) -> str:
    """`diag` dict → 消息里的 `diag` 字符串（逗号分隔 key=value，格式不进契约）。

    带的是**已知失败模式**要用的那几个（选层判据），外加本节点特有的
    `normal_source` / `normal` —— 法向来源不可见就没法排查（见模块 docstring）。

    `refine_diag` / `refined`：跑了 refine 时的落点，写 `refine=ok|none`，
    以及平面细化有没有真的生效（`plane_refined=1|0`）—— 部署必须开
    `plane=True`，这一项就是**它在没在**的证据（见模块 docstring）。
    """
    parts = []

    def _add(key, value, fmt='%.4f'):
        if value is None:
            return
        parts.append(f'{key}={fmt % float(value)}')

    _add('score', diag.get('score'))
    _add('coverage', diag.get('color_coverage'))
    if diag.get('n_observed_edges') is not None:
        parts.append(f"n_edges={int(diag['n_observed_edges'])}")
    _add('deck_h_mm', diag.get('deck_h_rel_mm'), '%.0f')
    _add('mask_area_ratio', diag.get('mask_area_ratio'))
    # ⚠️ **θ 用的是哪一个也进串**（2026-09-30）。现场排查过一次：日志里只能看到
    # `score` / `source`，分不出"这次是图像空间给的 θ、还是回退到了 `_theta_ref`、
    # 还是走了先验" —— `rect_source` / `rect_why` 一趟就答了。
    #
    # ⚠️ **`deg` 只加在真不是 None 的键上。** 老代码无条件 `%+.1f`，
    # 而 `theta_ref_deg` 在 prior 路径下**根本没被赋过值**（`None`）——
    # 一旦走到那条路，格式化就抛 `TypeError`，而它在**发布路径**上，
    # 会把整棵行为树掀翻。宁可这条诊断少写一个数，也不能让发布崩。
    def _add_deg(key):
        v = diag.get(key)
        if v is not None:
            parts.append(f'{key.replace("_deg", "")}={float(v):+.1f}')

    if diag.get('rect_source') is not None:
        parts.append(f"rect={diag['rect_source']}")
    if diag.get('rect_why') is not None:
        parts.append(f"rect_why={diag['rect_why']}")
    _add_deg('theta_ref_deg')
    _add_deg('theta_search_deg')
    if diag.get('dense_measured_mm') is not None:
        m = diag['dense_measured_mm']
        parts.append(f"dense_mm={float(m[0]):.0f}x{float(m[1]):.0f}")
    if diag.get('dense_fill') is not None:
        parts.append(f"dense_fill={float(diag['dense_fill']):.3f}")
    if diag.get('prior_used') is not None:
        parts.append(f"prior_used={int(bool(diag['prior_used']))}")
    if diag.get('prior_reason') is not None:
        parts.append(f"prior_reason={diag['prior_reason']}")
    parts.append(f'normal_source={_normal_source_tag(diag, normal_source)}')
    if normal is not None:
        n = np.asarray(normal, float)
        parts.append('normal=' + ','.join(f'{v:+.4f}' for v in n.reshape(-1)[:3]))
    if refined is not None:
        parts.append('refine=ok' if refined else 'refine=none')
    if refine_diag:
        plane = refine_diag.get('plane') or {}
        if plane:
            parts.append(f"plane_refined={int(bool(plane.get('refined')))}")
            if plane.get('note'):
                parts.append(f"plane_note={plane['note']}")
        if refine_diag.get('plane_offset_mm') is not None:
            parts.append(f"plane_offset_mm={float(refine_diag['plane_offset_mm']):.1f}")
    if diag.get('reject'):
        parts.append(f"reject={diag['reject']}")
    return ','.join(parts)


def format_rejects(diag: dict) -> str:
    """`diag` → 消息里的 `rejects` 字符串。**必须写实际值与阈值。**

    只写"失败"是没用的：同一条链路上有 6 种拒绝（`no_wood` / `no_deck` /
    `mask_too_large` / `no_theta_ref` / `ambiguous_theta` / `too_few_edges` /
    `degenerate`），不写清是哪一条、差多少，排查不了。

    成功（`reject is None`）返回空串 —— 契约说空串 = 没有失败。
    """
    code = diag.get('reject')
    if not code:
        return ''
    bits = [f'reject={code}']
    if code == 'ambiguous_theta' and diag.get('score') is not None:
        bits.append(f"score={float(diag['score']):.4f}<threshold={COVERAGE_MIN:g}")
    elif code == 'mask_too_large' and diag.get('mask_area_ratio') is not None:
        bits.append(f"mask_area_ratio={float(diag['mask_area_ratio']):.4f}"
                    f">threshold={MASK_AREA_RATIO_MAX:g}")
    elif code == 'too_few_edges':
        bits.append(f"n_observed_edges={int(diag.get('n_observed_edges') or 0)}<2")
    elif code in ('no_wood', 'no_deck') and diag.get('n_wood') is not None:
        bits.append(f"n_wood={int(diag['n_wood'])}<{MIN_WOOD_PX}")
    elif code == 'no_deck':
        bits.append(f"deck_h_rel_mm={diag.get('deck_h_rel_mm')}（选不出台面高度层）")
    elif code == 'no_theta_ref':
        bits.append('theta_ref 解不出来（掩码主轴缺失）')
    elif code == 'degenerate':
        bits.append('最优矩形贴到栅格边界，搜索窗开小了')
    # 带上 score（有的话）—— 拒绝时它是"差多少"最直接的量
    if diag.get('score') is not None and code != 'ambiguous_theta':
        bits.append(f"score={float(diag['score']):.4f}")
    if diag.get('source'):
        bits.append(f"source={diag['source']}")
    return ','.join(bits)


def build_payload(found: dict | None, diag: dict, k, latency_ms: float = 0.0,
                  normal_source: str | None = None, normal=None,
                  refine_diag: dict | None = None, refined: bool | None = None) -> dict:
    """`detect_pallet_frame` 的输出 → `PalletDetection` 的字段（纯函数，不 import ROS）。

    返回的 dict 的键**逐字对应消息字段名**，节点那边一个字段一个字段填。
    这样业务逻辑能被单元测试覆盖，ROS 那一层只剩搬运 —— 与
    `box_servo_window.build_payload` 同一分层。

    ⚠️ `det` **自己算**（`np.linalg.det(T[:3, :3])`），不要写死 1.0 ——
    契约里它是**手性判据**（+1 右手 / -1 镜面 / 0 共线），写死就等于废掉这个检查。

    ⚠️⚠️ **`T_cam_pallet` 的平移是「米」，不是毫米**（契约明文，2026-09-23 更新）。
    `detect_pallet_frame` 的 `frame['origin']` 是**相机系毫米**，所以这里**必须乘 0.001**。
    旋转块无量纲不受影响。**这是本模块最容易搞错的一处**：

      * 本消息里**别的**长度量（`size_mm` / `spread_mm`）都是**毫米** ——
        顺手把平移也填成毫米，是这里最自然的错误；
      * 填错**不会报错**：1000mm 被下游读成 1000m，位姿飞到天上，
        而 `det` 照样是 +1、`valid` 照样是 true；
      * 下游 `project_pallet_points` 对**台面系内的点**乘 0.001 换成米，
        而对 `T` 的平移**不做任何换算** —— 所以这个错**不会被下游纠正**。

    ⚠️ `corners_uv` 顺序 **左下→右下→右上→左上**，即 `_corners_mm` 的
    `(0,0)→(W,0)→(W,H)→(0,H)` —— **与 `BoxDetection.corners_uv` 不同**，
    这不是笔误：托盘有手性要求（`e1×e2=normal`），环序必须跟着 e1/e2 走。
    （`corners_uv` 是**像素**，与 `origin` 的单位无关。）

    `refine_diag` / `refined`：跑了 `refine_pallet_frame` 时的落点，写进 `diag`
    （`refine=ok|none`、`plane_refined=1|0`、`plane_offset_mm`）。**没有它们就
    看不出"平面细化到底有没有生效"**，而部署必须开 `plane=True`（见模块 docstring）。

    `found is None`（拒绝）时也**能产出 payload**，`valid=False`、`T_cam_pallet`
    填单位阵、`det=1.0`、`size_mm=[0,0]`、`corners_uv` 空 —— 契约说 `valid=false`
    时这些是**占位值，别用**。`rejects` 里带实际值与阈值。
    """
    latency = float(latency_ms or 0.0)
    diag_str = format_diag(diag, normal_source=normal_source, normal=normal,
                           refine_diag=refine_diag, refined=refined)
    rejects = format_rejects(diag)

    if found is None:
        return {
            'T_cam_pallet': np.eye(4, dtype=np.float64).reshape(-1).tolist(),
            'det': 1.0,
            'valid': False,
            'source': str(diag.get('source') or ''),
            'size_mm': [0.0, 0.0],
            'corners_uv': [],
            'n_used': 0,
            'n_slots': 1,
            'n_failed': 1,
            'spread_mm': 0.0,
            'spread_deg': 0.0,
            'latency_ms': latency,
            'diag': diag_str,
            'rejects': rejects,
        }

    T = np.eye(4, dtype=np.float64)
    T[:3, 0] = np.asarray(found['E1'], np.float64)
    T[:3, 1] = np.asarray(found['E2'], np.float64)
    T[:3, 2] = np.asarray(found['nrm'], np.float64)
    # ⚠️ **毫米 -> 米**：契约要求 `T_cam_pallet` 的平移单位是**米**，
    # 而 `found['origin']` 是相机系**毫米**。见 docstring 的 ⚠️⚠️ 段。
    # 旋转块无量纲，不换算。
    T[:3, 3] = np.asarray(found['origin'], np.float64) * MM_TO_M
    W, H = float(found['W']), float(found['H'])
    px = _project_px(_corners_mm(found, W, H), k)
    return {
        'T_cam_pallet': T.reshape(-1).tolist(),          # 行主序
        'det': float(np.linalg.det(T[:3, :3])),
        'valid': True,
        'source': str(diag.get('source') or ''),
        'size_mm': [W, H],
        'corners_uv': [(float(u), float(v), 0.0) for u, v in px],
        'n_used': 1,            # 不做时间窗的检测器填 1/1/0（窗在 LeTools 侧做）
        'n_slots': 1,
        'n_failed': 0,
        'spread_mm': 0.0,       # 单帧时都是 0.0
        'spread_deg': 0.0,
        'latency_ms': latency,
        'diag': diag_str,
        'rejects': rejects,
    }
