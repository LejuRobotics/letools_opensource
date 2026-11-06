#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""`obb_decoder.py` 的纯函数自检 —— **不需要 ROS、不需要 onnxruntime、不需要模型文件**。

跑法（退出码 0 通过 / 1 失败）：
    cd <仓库根>
    python3 skills/atomic/perception/carton_detect/tests/test_obb_decoder.py

装了 ultralytics 的话会多跑一条**逐帧比对**（拿真模型对真图，与 ultralytics
自己的 `OBBPredictor` 比四点），没装自动跳过。**那条才是真正的判据** ——
手算的断言只能证明「跟我以为的一样」。

⚠️ 2026-10-15 这个模块从 ROS 包（`infrastructure/.../carton_obb/scripts/`）
搬到了 `skills/atomic/perception/carton_detect/`（与 `pallet_detect` 同构）。
下面这行是**模块自己的相对路径**，跟着文件走，与 git 历史无关。
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from obb_decoder import (        # noqa: E402
    DEFAULT_NMS_IOMIN, DEFAULT_NMS_IOU,
    angle_deg_from_corners, decode_obb, d_center_px, d_center_score,
    depth_score, dets_from_rotated_rects, letterbox, nms_rotated,
    order_corners_uv, pick_best,
    quad_from_xywhr, refine_bottom_by_depth, sample_depth_mm, score_candidates,
    to_input_tensor,
)

_FAILED = []


def check(name: str, cond: bool, detail: str = '') -> None:
    if cond:
        print(f'  ✓ {name}')
    else:
        print(f'  ✗ {name}  {detail}')
        _FAILED.append(name)


# --------------------------------------------------------------------------- #
def test_letterbox_square() -> None:
    print('letterbox：方形补边')
    img = np.zeros((480, 640, 3), np.uint8)
    out, r, (left, top) = letterbox(img, (640, 640))
    check('输出是 640x640', bool(out.shape[:2] == (640, 640)), str(out.shape))
    check('640x480 的缩放比是 1.0', abs(r - 1.0) < 1e-9, str(r))
    check('水平不补边', bool(left == 0), str(left))
    check('垂直上下各补 80', top == 80.0, str(top))


def test_letterbox_non_square() -> None:
    print('letterbox：非方图（640x320）等比 + 居中补边')
    img = np.zeros((320, 640, 3), np.uint8)
    out, r, (left, top) = letterbox(img, (640, 640))
    check('输出 640x640', bool(out.shape[:2] == (640, 640)))
    check('缩放比 1.0（宽已经够）', abs(r - 1.0) < 1e-9, str(r))
    check('上下各补 160', top == 160.0, str(top))


def test_quad_from_xywhr_axis_aligned() -> None:
    print('quad_from_xywhr：原始顺序是 p1..p4（p1=右下），契约序要再过 order_corners_uv')
    q = quad_from_xywhr(100.0, 100.0, 40.0, 20.0, 0.0)
    # θ=0、w=40、h=20、中心 (100,100)：p1=(120,110) 右下 → p2=(120,90) 右上
    #                                       → p3=(80,90) 左上 → p4=(80,110) 左下
    raw = np.array([[120, 110], [120, 90], [80, 90], [80, 110]], float)
    check('原始四点逐位相符', bool(np.allclose(q, raw)), f'\n{q}\n{raw}')
    check('p1 是最靠右下的角', bool(np.argmax(q[:, 0] + q[:, 1]) == 0), str(q))
    # ★ 契约序（右下 左下 左上 右上）与原始序**不同**（差一个循环移位）——
    #   这正是 decode_obb 里必须调 order_corners_uv 的原因，不是多余的保险。
    want = np.array([[120, 110], [80, 110], [80, 90], [120, 90]], float)
    check('order_corners_uv 之后 = 契约序',
          bool(np.allclose(order_corners_uv(q), want)), f'\n{order_corners_uv(q)}')
    check('原始序确实不是契约序（否则上面那条判据是白测的）',
          not np.allclose(q, want))


def test_quad_rotated_90() -> None:
    print('quad_from_xywhr：转 90° 时边长与角度一致')
    q = quad_from_xywhr(0.0, 0.0, 40.0, 20.0, math.pi / 2)
    e = [float(np.linalg.norm(q[(i + 1) % 4] - q[i])) for i in range(4)]
    # 从 p1 起走：边 0 = h（短）、边 1 = w（长）、边 2 = h、边 3 = w
    check('边 0/2 = h = 20', abs(e[0] - 20) < 1e-9 and abs(e[2] - 20) < 1e-9, str(e))
    check('边 1/3 = w = 40', abs(e[1] - 40) < 1e-9 and abs(e[3] - 40) < 1e-9, str(e))
    # 转 0° 时同样成立（换角度不改这个规律）
    q0 = quad_from_xywhr(0.0, 0.0, 40.0, 20.0, 0.0)
    e0 = [float(np.linalg.norm(q0[(i + 1) % 4] - q0[i])) for i in range(4)]
    check('转 0° 时规律相同', abs(e0[0] - 20) < 1e-9 and abs(e0[1] - 40) < 1e-9, str(e0))


def test_order_corners_any_input() -> None:
    print('order_corners_uv：任意顺序都归到契约序')
    base = np.array([[120, 110], [80, 110], [80, 90], [120, 90]], float)
    for k in range(4):
        rolled = np.roll(base, k, axis=0)
        check(f'循环移位 {k}', bool(np.allclose(order_corners_uv(rolled), base)))
    check('反序也归回来', bool(np.allclose(order_corners_uv(base[::-1]), base)))


def test_decode_obb_roundtrip() -> None:
    print('decode_obb：640x480 图上的 round-trip（r=1、pad_top=80）')
    # 造一行：中心在 640 空间的 (320, 400)，w=200 h=100，角度 0
    raw = np.array([[320.0, 400.0, 200.0, 100.0, 0.9, 0.0]])
    dets = decode_obb(raw, ratio=1.0, pad=(0.0, 80.0), conf_thr=0.25,
                      orig_shape=(480, 640))
    check('出一个框', len(dets) == 1, str(len(dets)))
    if dets:
        q = dets[0]['corners_uv']
        check('反 letterbox 后中心 v = 320',
              abs(q[:, 1].mean() - 320.0) < 1e-9, str(q[:, 1].mean()))
        check('四角顺序 = 右下 左下 左上 右上',
              bool(q[0, 0] > q[1, 0] and q[0, 1] > q[3, 1]), f'\n{q}')
        check('box_uv 是外接框',
              all(abs(x - y) < 1e-9 for x, y in
                  zip(dets[0]['box_uv'], [220.0, 270.0, 420.0, 370.0])),
              str(dets[0]['box_uv']))


def test_decode_obb_conf_filter() -> None:
    print('decode_obb：conf 阈值与越界过滤')
    raw = np.array([[320.0, 400.0, 100.0, 100.0, 0.10, 0.0],
                    [320.0, 400.0, 100.0, 100.0, 0.90, 0.0]])
    check('低分被滤掉', len(decode_obb(raw, 1.0, (0.0, 80.0), conf_thr=0.25)) == 1)
    far = np.array([[5000.0, 5000.0, 10.0, 10.0, 0.9, 0.0]])
    check('完全跑出图外的被丢',
          len(decode_obb(far, 1.0, (0.0, 80.0), conf_thr=0.25,
                         orig_shape=(480, 640))) == 0)


def test_dets_from_rotated_rects_matches_decode() -> None:
    """★ 公共服务输出 → det：必须与本地 ONNX 解码**逐字段同值**。

    走 `InferCartonObb` 那条路时，服务端给的是原图坐标的 `cv2.RotatedRect`
    （度），本地 `decode_obb` 给的是 letterbox 反算后的四角。两条路只有在这一条
    测试上对得上，「用公共输出」才敢说与「自己推理」效果一模一样。
    """
    print('dets_from_rotated_rects：与 decode_obb 同口径')
    rng = np.random.default_rng(7)
    n = 500
    # 640 空间里的一组随机框（避开边界，保证两边都不过滤）
    cx = rng.uniform(60, 580, n)
    cy = rng.uniform(140, 580, n)
    w = rng.uniform(20, 200, n)
    h = rng.uniform(20, 200, n)
    ang = rng.uniform(-np.pi / 2, np.pi, n)
    conf = rng.uniform(0.26, 1.0, n)
    raw = np.column_stack([cx, cy, w, h, conf, ang])

    pad = (40.0, 50.0)
    ratio = 0.8
    via_raw = decode_obb(raw, ratio, pad, conf_thr=0.25, orig_shape=(480, 640))

    # 服务的响应是**原图坐标**：先按 decode_obb 的反变换把中心/宽高缩回去
    via_svc = dets_from_rotated_rects(
        (cx - pad[0]) / ratio, (cy - pad[1]) / ratio,
        w / ratio, h / ratio, np.degrees(ang), conf,
        orig_shape=(480, 640), conf_thr=0.25)

    check('候选数一致', len(via_raw) == len(via_svc),
          f'{len(via_raw)} vs {len(via_svc)}')
    if len(via_raw) != len(via_svc):
        return
    order_same = all(abs(a['confidence'] - b['confidence']) < 1e-9
                     for a, b in zip(via_raw, via_svc))
    check('按置信度降序、截断口径一致', order_same)
    corner_err = max(float(np.abs(a['corners_uv'] - b['corners_uv']).max())
                     for a, b in zip(via_raw, via_svc))
    check('四角最大差 < 1e-3 px', corner_err < 1e-3, f'{corner_err:.6g}')
    box_err = max(float(np.abs(np.asarray(a['box_uv'])
                               - np.asarray(b['box_uv'])).max())
                  for a, b in zip(via_raw, via_svc))
    check('box_uv 最大差 < 1e-3 px', box_err < 1e-3, f'{box_err:.6g}')
    angle_err = max(abs(float(a['angle_deg']) - float(b['angle_deg']))
                    for a, b in zip(via_raw, via_svc))
    check('angle_deg（弧度）一致', angle_err < 1e-6, f'{angle_err:.6g}')

    # 服务端少发一个字段、或数组不等长 -> 明确报错，不静默错位
    try:
        dets_from_rotated_rects([1.0], [1.0], [1.0], [1.0], [0.0], [],
                                conf_thr=0.25)
        raised = False
    except ValueError:
        raised = True
    check('数组长度不一致直接报错', raised)

    # 越界过滤与 conf 过滤同 decode_obb
    check('低分被滤掉',
          dets_from_rotated_rects([320.0], [240.0], [50.0], [50.0], [0.0],
                                  [0.10], conf_thr=0.25) == [])
    check('完全跑出图外的被丢',
          dets_from_rotated_rects([5000.0], [5000.0], [10.0], [10.0], [0.0],
                                  [0.9], orig_shape=(480, 640)) == [])


def test_nms_rotated() -> None:
    print('nms_rotated：重合框去重、不同框保留')
    a = {'corners_uv': quad_from_xywhr(100, 100, 60, 40, 0.0), 'confidence': 0.9}
    b = {'corners_uv': quad_from_xywhr(102, 101, 60, 40, 0.0), 'confidence': 0.8}
    c = {'corners_uv': quad_from_xywhr(400, 300, 60, 40, 0.0), 'confidence': 0.7}
    out = nms_rotated([a, b, c], 0.45, (480, 640))
    check('两个重合只留一个', len(out) == 2, str(len(out)))
    check('留下的是高分那个', out[0]['confidence'] == 0.9)
    # ★ 同中心、同大小的正方形差 45°：交集是正八边形，面积占单方形 2(√2−1)≈0.8284，
    #   并集 = 2 − 0.8284，所以 **IoU = 1/√2 ≈ 0.7071**。
    #   而**轴对齐 IoU 只有 0.5** —— 两者差 0.2，足以分辨「算的是旋转交集还是 AABB」。
    from obb_decoder import _rot_iou, _mask, _rot_overlaps
    d = quad_from_xywhr(100, 100, 60, 60, math.pi / 4)
    e = quad_from_xywhr(100, 100, 60, 60, 0.0)
    got = _rot_iou(d, e, (480, 640))
    check('同中心转 45° 的旋转 IoU ≈ 1/√2 = 0.7071（AABB 近似会算成 0.5）',
          abs(got - 1.0 / math.sqrt(2.0)) < 0.01, f'{got:.4f}')

    # 掩码缓存版（`_mask` + `_rot_overlaps`，NMS 内循环用的那条）与 `_rot_iou`
    # 必须给出**同一个 IoU** —— 否则"顺手加速"就顺手改了数值。
    m1, m2 = _mask(d, (480, 640)), _mask(e, (480, 640))
    iou2, iomin2 = _rot_overlaps(m1, int(m1.sum()), m2, int(m2.sum()))
    check('掩码缓存版的 IoU 与 _rot_iou 一致',
          abs(iou2 - got) < 1e-12, f'{iou2:.6f} vs {got:.6f}')
    # 这一对（同中心、转 45°、等大）的 IoMin 实测 0.8406，理论 2(√2−1)=0.8284 ——
    # 差在栅格化：两个旋转正方形的像素面积不恰好相等（3613 vs 3721）。
    # ⚠️ 它**贴着 0.85 的边**，但这条不靠 IoMin 兜：IoU 0.707 > 0.45 已经把它压掉了。
    # 两条判据是「或」的关系，这里正好是一个"该由 IoU 管、不该由 IoMin 管"的样本。
    check('同中心转 45° 两等大框：IoMin ≈ 0.828（栅格化实测 0.84，贴着阈值但不归它管）',
          abs(iomin2 - 2 * (math.sqrt(2) - 1)) < 0.02, f'{iomin2:.4f}')
    check('这一对由 IoU 那条压掉（0.707 > 0.45），不依赖 IoMin',
          iou2 > 0.45, f'{iou2:.4f}')


def test_nms_iomin_containment() -> None:
    """★ NMS 的第二条判据：**小框整个套在大框里**（IoU 天生够不着）。

    这正是用户看到的现象：模型在一个箱子上出多个框，有的只框了一小块。
    小框被包住时 `IoU ≈ 小面积/大面积`（这里 ≈0.25），IoU 阈值给多低都放它过去；
    `IoMin = 交/较小面积` 恒 ≈1.0。**抑制按 confidence 降序 → 留下高分那个。**
    """
    big = {'corners_uv': quad_from_xywhr(320, 240, 200, 160, 0.0), 'confidence': 0.55}
    small = {'corners_uv': quad_from_xywhr(320, 240, 100, 80, 0.0), 'confidence': 0.90}
    apart = {'corners_uv': quad_from_xywhr(560, 400, 200, 160, 0.0), 'confidence': 0.50}

    # 先确认这类确实抓不住：IoU ≈ 1/4，远低于 0.45
    from obb_decoder import _mask, _rot_overlaps
    mb, ms = _mask(big['corners_uv'], (480, 640)), _mask(small['corners_uv'], (480, 640))
    iou, iomin = _rot_overlaps(mb, int(mb.sum()), ms, int(ms.sum()))
    check('套框的 IoU ≈ 0.25（现行 IoU-NMS 放它过去）',
          abs(iou - 0.25) < 0.02, f'{iou:.4f}')
    check('套框的 IoMin ≈ 1.0', iomin > 0.98, f'{iomin:.4f}')

    out = nms_rotated([big, small, apart], 0.45, (480, 640))
    check('默认阈值下套着的只留一个', len(out) == 2, str(len(out)))
    check('留下的是**置信度最高**的那个（小框 0.90）',
          out[0]['confidence'] == 0.90, str([d['confidence'] for d in out]))
    check('分开的那个框不受影响', any(d is apart for d in out))

    # 关掉这条（>=1.0）必须回到旧行为 —— 否则"可关"是假的
    old = nms_rotated([big, small, apart], 0.45, (480, 640), iomin_thr=1.01)
    check('iomin_thr >= 1.0 = 关掉，回到只有 IoU 的旧行为',
          len(old) == 3, f'关掉后应留 3 个，实得 {len(old)}')

    # 阈值边界：IoMin 恰好取默认值时不能反过来被杀
    check('默认 nms_iomin 是 0.85', DEFAULT_NMS_IOMIN == 0.85, str(DEFAULT_NMS_IOMIN))
    check('默认 nms_iou 是 0.45', DEFAULT_NMS_IOU == 0.45, str(DEFAULT_NMS_IOU))

    # ★ 擦边的两个**真箱子**不能被误伤：IoMin 会到 0.3~0.46，必须留在阈值之下
    r1 = {'corners_uv': quad_from_xywhr(200, 240, 240, 160, 0.0), 'confidence': 0.8}
    r2 = {'corners_uv': quad_from_xywhr(300, 280, 240, 160, 0.0), 'confidence': 0.7}
    mr1, mr2 = _mask(r1['corners_uv'], (480, 640)), _mask(r2['corners_uv'], (480, 640))
    _, im_edge = _rot_overlaps(mr1, int(mr1.sum()), mr2, int(mr2.sum()))
    check('擦边的两个真箱子：IoMin 应远低于 0.85（实测量级 0.46 封顶）',
          im_edge < 0.85, f'{im_edge:.4f}')
    check('擦边的两个真箱子都留下',
          len(nms_rotated([r1, r2], 0.45, (480, 640))) == 2)


def test_d_center() -> None:
    print('d_center_px / d_center_score：越靠画面下方中心分越高')
    ref = (320.0, 480.0)
    near = quad_from_xywhr(320, 400, 100, 100, 0.0)     # 质心 (320,400)
    far = quad_from_xywhr(600, 100, 100, 100, 0.0)      # 质心 (600,100)
    check('近的 d 更小', d_center_px(near, ref) < d_center_px(far, ref))
    check('近的 d = 80', abs(d_center_px(near, ref) - 80.0) < 1e-9,
          str(d_center_px(near, ref)))
    check('d<=d_near 得 1', d_center_score(50.0, d_near=100, d_far=450) == 1.0)
    check('d>=d_far 得 0', d_center_score(500.0, d_near=100, d_far=450) == 0.0)
    check('中间线性 0.5', abs(d_center_score(275.0, d_near=100, d_far=450) - 0.5) < 1e-9)
    check('参数写反退化成阶跃',
          d_center_score(10.0, d_near=450, d_far=100) == 1.0
          and d_center_score(500.0, d_near=450, d_far=100) == 0.0)


def test_depth() -> None:
    print('sample_depth_mm / depth_score')
    d = np.zeros((480, 640), np.float32)
    d[:] = 1000.0
    d[240:250, 320:330] = 0          # 中心挖一块空洞（0 = 无效）
    z = sample_depth_mm(d, [300.0, 230.0, 340.0, 250.0], half=6, z_near=500, z_far=1500)
    check('空洞被跳过、中位数还是 1000', z == 1000.0, str(z))
    d2 = np.zeros((480, 640), np.float32)     # 全 0 = 全无效
    check('全无效返回 None',
          sample_depth_mm(d2, [300, 230, 340, 250], half=6, z_near=500, z_far=1500) is None)
    # 贴左边界的框：中心 (10,10) 在图内，采样方块会被裁到 [4,17)
    check('贴边裁剪后仍能取到',
          sample_depth_mm(d, [0.0, 0.0, 20.0, 20.0], half=6,
                          z_near=500, z_far=1500) == 1000.0)
    # 中心完全在图外的框 → None（不是 0，也不是抛）
    check('中心在图外返回 None',
          sample_depth_mm(d, [-60.0, -60.0, -40.0, -40.0], half=6,
                          z_near=500, z_far=1500) is None)
    check('depth_score 越近越高', depth_score(600) > depth_score(1400))
    check('depth_score 边界', depth_score(500) == 1.0 and depth_score(1500) == 0.0)


def test_pick_best() -> None:
    print('pick_best：选箱（距离 + 面积 + 深度）')
    ref = (320.0, 480.0)
    hand = {'corners_uv': quad_from_xywhr(320, 400, 200, 150, 0.0),
            'box_uv': [220, 325, 420, 475], 'confidence': 0.97}
    neighbor = {'corners_uv': quad_from_xywhr(600, 200, 300, 250, 0.0),
                'box_uv': [450, 75, 750, 325], 'confidence': 0.98}
    # ⚠️ 邻箱 conf 更高、面积更大 —— 只有距离判据能挑对
    got = pick_best([neighbor, hand], ref, z_list=[None, None])
    check('挑中的是手里那个（不是 conf 最高/面积最大的邻箱）',
          got is hand, '邻箱 conf 0.98 > 手箱 0.97，面积也更大')

    # 同距离同面积的两条，分数必须相等（= conf 没进公式）
    a = {'corners_uv': quad_from_xywhr(320, 400, 100, 100, 0.0),
         'box_uv': [270, 350, 370, 450], 'confidence': 0.05}
    b = {'corners_uv': quad_from_xywhr(320, 400, 100, 100, 0.0),
         'box_uv': [270, 350, 370, 450], 'confidence': 0.99}
    sc = score_candidates([a, b], ref, [None, None])
    check('同几何、conf 天差地别 → 总分相同',
          abs(sc[0][1] - sc[1][1]) < 1e-12, f'{sc[0][1]} vs {sc[1][1]}')

    check('空列表返回 None', pick_best([], ref) is None)

    # ★ 深度缺失的处置是「把深度权重按比例让给另两项」，**归一化到 [0,1]**：
    #   一个在距离与面积上都满分的候选，**无论有没有深度都拿 1.0**。
    #   代价是深度项**只能压低远处的候选，不能额外抬高近处的**。
    only = {'corners_uv': quad_from_xywhr(320, 400, 100, 100, 0.0),
            'box_uv': [270, 350, 370, 450], 'confidence': 0.9}    # d=80 → 距离项满分
    s_none = score_candidates([only], ref, [None])[0][1]
    s_full = score_candidates([only], ref, [500.0])[0][1]         # z_near → 深度满分
    s_far = score_candidates([only], ref, [1500.0])[0][1]         # z_far  → 深度 0
    check('无深度时满分候选 = 1.0', abs(s_none - 1.0) < 1e-9, f'{s_none:.4f}')
    check('有满分深度时同样 = 1.0（深度不额外抬高）', abs(s_full - 1.0) < 1e-9,
          f'{s_full:.4f}')
    check('深度远 → 分数被压低', s_far < s_none - 0.2,
          f'远 {s_far:.4f} vs 无深度 {s_none:.4f}')


def test_angle_deg() -> None:
    print('angle_deg_from_corners：与旧链路 build_payload 同口径')
    # 轴对齐 → 下边水平 → 0°
    q = order_corners_uv(quad_from_xywhr(320, 240, 200, 100, 0.0))
    check('轴对齐 = 0°', abs(angle_deg_from_corners(q)) < 1e-9,
          str(angle_deg_from_corners(q)))
    # 整体转 +10°（图像系 +y 向下，顺时针为正）→ 下边应给 +10°
    q = order_corners_uv(quad_from_xywhr(320, 240, 200, 100, math.radians(10)))
    check('转 +10° → +10°', abs(angle_deg_from_corners(q) - 10.0) < 1e-6,
          str(angle_deg_from_corners(q)))
    q = order_corners_uv(quad_from_xywhr(320, 240, 200, 100, math.radians(-10)))
    check('转 −10° → −10°', abs(angle_deg_from_corners(q) + 10.0) < 1e-6,
          str(angle_deg_from_corners(q)))
    # 角度用**下边**而不是长边：宽扁的框转 90° 后长边变竖，下边仍在下方
    q = order_corners_uv(quad_from_xywhr(320, 240, 200, 100, math.pi / 2))
    check('转 90° 后下边角仍在 (−90,90] 内',
          -90.0 < angle_deg_from_corners(q) <= 90.0,
          str(angle_deg_from_corners(q)))


def test_input_tensor() -> None:
    print('to_input_tensor：BGR→RGB、CHW、/255')
    img = np.zeros((480, 640, 3), np.uint8)
    img[..., 0] = 255        # B 通道满
    x, _, _ = to_input_tensor(img, 640)
    check('形状 (1,3,640,640)', x.shape == (1, 3, 640, 640), str(x.shape))
    check('dtype float32', x.dtype == np.float32)
    # ⚠️ 只在**图内**区域查：补边区是 114/255，`max()` 会把那个 0.447 也算进来。
    inner = x[0, :, 80:560, :]          # 640x480 图在 640 空间里占 v ∈ [80, 560)
    check('BGR→RGB：第 0 个通道是 R（=0），第 2 个是 B（=1）',
          inner[0].max() == 0.0 and abs(inner[2].min() - 1.0) < 1e-6,
          f'R={inner[0].max()} B={inner[2].min()}')
    check('补边区是 114/255',
          abs(x[0, 0, 5, 5] - 114.0 / 255.0) < 1e-6, str(x[0, 0, 5, 5]))


def test_refine_bottom_by_depth() -> None:
    print('refine_bottom_by_depth：按深度下沿收缩被拉长的框')
    # 模拟现场 bag：箱体 580mm、下方背景 830mm，跳变在 y=363
    depth = np.full((480, 640), 580, np.uint16)
    depth[363:, :] = 830
    # 框：中心 (320,260)、宽 350、高 234 → top=143、bottom=377（模型拉长了一点）
    q = order_corners_uv(quad_from_xywhr(320, 260, 350, 234, 0.0))
    new, info = refine_bottom_by_depth(depth, q, step=2, band_px=45)
    check('触发了修正', info['changed'] and new is not None, str(info))
    if new is not None:
        check('y_new 落在深度跳变 363 附近（±2px）', abs(info['y_new'] - 363) <= 2,
              str(info['y_new']))
        check('上边不动', abs(new[:, 1].min() - q[:, 1].min()) < 1e-6,
              f'{new[:, 1].min()} vs {q[:, 1].min()}')
        check('宽度不变', abs((new[:, 0].max() - new[:, 0].min())
                            - (q[:, 0].max() - q[:, 0].min())) < 1e-6)
        check('h 收到 220 附近（±2.5）', abs(info['h_after'] - 220) < 2.5,
              str(info['h_after']))
        check('箱体深度 ≈580', abs(info['zi_mm'] - 580) < 1e-6, str(info['zi_mm']))
        check('背景深度 ≈830', abs(info['zb_mm'] - 830) < 1e-6, str(info['zb_mm']))

    # 没有远背景（箱体下方同深度）→ 不能动
    flat = np.full((480, 640), 580, np.uint16)
    none, info2 = refine_bottom_by_depth(flat, q)
    check('无深度跳变时不改框', none is None and info2['reason'] == 'no_jump',
          str(info2))

    # 只收缩：框本来就比跳变短，搜索区到不了背景 → 不动
    short = order_corners_uv(quad_from_xywhr(320, 220, 350, 154, 0.0))  # bottom=297
    none3, info3 = refine_bottom_by_depth(depth, short)
    check('已经比跳变短时不改框', none3 is None, str(info3))

    # 没有深度图 → 不动
    none4, info4 = refine_bottom_by_depth(None, q)
    check('depth=None 时不改框', none4 is None and info4['reason'] == 'no_depth',
          str(info4))

    # 深度全 0（无效）→ 不动
    bad = np.zeros((480, 640), np.uint16)
    none5, info5 = refine_bottom_by_depth(bad, q)
    check('深度全无效时不动', none5 is None, str(info5))


def test_ultralytics_parity() -> None:
    """装了 ultralytics 才跑的**真判据**：与官方预测器逐帧比四点。"""
    print('与 ultralytics 的 LetterBox 逐像素比对')
    try:
        from ultralytics.data.augment import LetterBox
    except Exception as exc:                     # noqa: BLE001
        print(f'  – 跳过（{type(exc).__name__}）')
        return
    lb = LetterBox((640, 640), auto=False)
    for hw in [(480, 640), (320, 640), (640, 640), (700, 900), (100, 100)]:
        img = np.random.randint(0, 255, (*hw, 3), np.uint8)
        mine, _, _ = letterbox(img, (640, 640))
        theirs = lb(image=img)
        same = mine.shape == theirs.shape and np.array_equal(mine, theirs)
        check(f'{hw} 与 ultralytics 逐像素相同', same,
              f'{mine.shape} vs {theirs.shape}')


if __name__ == '__main__':
    for fn in (test_letterbox_square, test_letterbox_non_square,
               test_quad_from_xywhr_axis_aligned, test_quad_rotated_90,
               test_order_corners_any_input, test_decode_obb_roundtrip,
               test_decode_obb_conf_filter, test_dets_from_rotated_rects_matches_decode,
               test_nms_rotated,
               test_nms_iomin_containment, test_d_center,
               test_depth, test_pick_best, test_angle_deg, test_input_tensor,
               test_refine_bottom_by_depth,
               test_ultralytics_parity):
        fn()
        print()
    if _FAILED:
        print(f'✗ 失败 {len(_FAILED)} 条：{_FAILED}')
        sys.exit(1)
    print('✓ 全部通过')
    sys.exit(0)
