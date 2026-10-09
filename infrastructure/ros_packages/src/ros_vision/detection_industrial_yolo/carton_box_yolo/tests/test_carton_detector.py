#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""纸箱检测的纯函数自检 —— **不需要 ROS、不需要模型、不需要 onnxruntime**。

只测 `carton_detector.py` 里那几件事：letterbox 的几何、坐标反变换、挑框规则、
契约顺序。这几件事错了都是**静默错**（框整体平移、挑到邻箱、顺序给反），
跑真机之前先把它们钉死。

跑法：
    python3 tests/test_carton_detector.py       # 退出码 0 通过 / 1 失败
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scripts"))

from carton_detector import (  # noqa: E402
    box_area_px,
    box_to_polygon_points,
    candidate_score,
    decode,
    depth_score,
    letterbox,
    pick_best,
    sample_depth_mm,
    score_candidates,
    to_input_tensor,
)

_fails: list[str] = []


def check(name: str, cond: bool, detail: str = "") -> None:
    if cond:
        print(f"  PASS  {name}")
    else:
        print(f"  FAIL  {name}  {detail}")
        _fails.append(name)


# --------------------------------------------------------------------------- #
# letterbox 几何
# --------------------------------------------------------------------------- #
def test_letterbox_geometry() -> None:
    """640x480 的图补到 640x640：左右不补、上下各补 80，缩放比 1.0。"""
    img = np.zeros((480, 640, 3), np.uint8)
    out, r, (px, py) = letterbox(img, (640, 640))
    check("输出尺寸 640x640", out.shape == (640, 640, 3), str(out.shape))
    check("缩放比 1.0", abs(r - 1.0) < 1e-9, f"{r}")
    check("左右不补", (px, py) == (0.0, 80.0), f"pad=({px},{py})")
    # 上下补边应是 114 灰
    check("补边是 114 灰", int(out[0, 0, 0]) == 114, str(out[0, 0, 0]))
    check("中心行还是原图（0）", int(out[320, 320, 0]) == 0, str(out[320, 320, 0]))


def test_letterbox_round_trip() -> None:
    """反变换必须把 letterbox 后的点还原回原图坐标。"""
    img = np.zeros((480, 640, 3), np.uint8)
    _, r, (px, py) = letterbox(img, (640, 640))
    # 原图里的 (100, 200) 在 letterbox 图里是 (100*r+px, 200*r+py)
    u_lb, v_lb = 100 * r + px, 200 * r + py
    raw = np.array([[u_lb, v_lb, u_lb + 50, v_lb + 40, 0.9, 0.0]])
    dets = decode(raw, r, (px, py), conf_thr=0.25)
    check("反变换还原 u", len(dets) == 1 and abs(dets[0]["box_uv"][0] - 100) < 1e-6,
          str(dets))
    check("反变换还原 v", len(dets) == 1 and abs(dets[0]["box_uv"][1] - 200) < 1e-6,
          str(dets))


def test_letterbox_non_square() -> None:
    """1280x800 的图（机器人相机的实际分辨率）也要能补对。

    `r = min(640/800, 640/1280) = 0.5` → 缩放后 640x400 → **上下各补 120**、
    左右不补。注意不是"左右补 160"：宽度先到 640 了，补的是高度。
    """
    img = np.zeros((800, 1280, 3), np.uint8)
    out, r, (px, py) = letterbox(img, (640, 640))
    check("1280x800 → 640x640", out.shape == (640, 640, 3), str(out.shape))
    check("缩放比 0.5", abs(r - 0.5) < 1e-9, f"{r}")
    check("左右不补", abs(px) < 1e-9, f"px={px}")
    check("上下各补 120", abs(py - 120) < 1e-9, f"py={py}")


def test_letterbox_matches_ultralytics() -> None:
    """跟 ultralytics 的 `LetterBox` 逐像素比 —— 装了就比，没装就跳过。

    这是**真正的判据**：上面几条手算的断言只能证明"跟我以为的一样"，这条才能
    证明"跟训练时喂进去的一样"。补边差一个像素，框就整体平移一个像素。
    """
    try:
        import cv2
        from ultralytics.data.augment import LetterBox
    except Exception:
        print("  SKIP  ultralytics 没装，跳过（不影响本文件其余用例）")
        return
    rng = np.random.default_rng(0)
    for h, w in ((480, 640), (800, 1280), (480, 640), (300, 900), (1000, 500)):
        img = rng.integers(0, 256, (h, w, 3), dtype=np.uint8)
        mine, r_m, pad_m = letterbox(img, (640, 640))
        ref = LetterBox(new_shape=(640, 640), auto=False, scaleup=True)(image=img)
        check(f"{w}x{h} 逐像素一致", np.array_equal(mine, ref),
              f"最大差 {int(np.abs(mine.astype(int) - ref.astype(int)).max())}")
        check(f"{w}x{h} 缩放比一致",
              abs(r_m - min(640 / h, 640 / w)) < 1e-12, f"{r_m}")


# --------------------------------------------------------------------------- #
# 张量
# --------------------------------------------------------------------------- #
def test_input_tensor_layout() -> None:
    """张量必须是 (1,3,640,640)、float32、值域 [0,1]，且**通道是 RGB 不是 BGR**。"""
    img = np.zeros((480, 640, 3), np.uint8)
    img[:, :, 0] = 255                       # BGR 里第 0 通道 = 蓝
    x, r, _ = to_input_tensor(img)
    check("形状 (1,3,640,640)", x.shape == (1, 3, 640, 640), str(x.shape))
    check("dtype float32", x.dtype == np.float32, str(x.dtype))
    # BGR 图里蓝=255，转成 RGB 之后蓝在**第 2 个**通道。
    # 注意 `to_input_tensor` 会先 letterbox 补边，所以取**图像中心**那个像素
    # （补边区域是 114 灰，取角落会读到补边）。
    center = x[0, :, 320, 320]
    check("通道是 RGB（蓝落在第2通道）",
          center[2] > 0.99 and center[0] < 0.45,
          f"center={center.round(3).tolist()}")
    # 补边区域应为 114/255
    check("补边值 114/255", abs(float(x[0, 0, 0, 0]) - 114 / 255) < 1e-6,
          str(float(x[0, 0, 0, 0])))


# --------------------------------------------------------------------------- #
# 解码
# --------------------------------------------------------------------------- #
def test_decode_filters_low_conf() -> None:
    raw = np.array([[10, 10, 50, 50, 0.9, 0.0],
                    [20, 20, 60, 60, 0.10, 0.0],
                    [30, 30, 70, 70, 0.80, 0.0]])
    dets = decode(raw, 1.0, (0.0, 0.0), conf_thr=0.25)
    check("滤掉低置信度", len(dets) == 2, str(len(dets)))
    check("按置信度降序", dets[0]["confidence"] > dets[1]["confidence"],
          str([d["confidence"] for d in dets]))


def test_decode_drops_out_of_frame() -> None:
    """完全在图外的框要丢掉（那是补边区域里的噪声）。"""
    raw = np.array([[10, 10, 50, 50, 0.9, 0.0],
                    [700, 700, 800, 800, 0.9, 0.0]])
    dets = decode(raw, 1.0, (0.0, 0.0), conf_thr=0.25, orig_shape=(480, 640))
    check("丢掉图外框", len(dets) == 1, str(len(dets)))
    check("留下的是图内那个", abs(dets[0]["box_uv"][0] - 10) < 1e-6, str(dets))


def test_decode_empty() -> None:
    check("空输入返回空", decode(None, 1.0, (0.0, 0.0)) == [])
    check("全零 query 返回空",
          decode(np.zeros((300, 6)), 1.0, (0.0, 0.0)) == [])


def test_decode_no_nms_needed() -> None:
    """yolo26 是端到端头，**不做 NMS** —— 两个高度重叠的框都要原样留下。

    这条是防回归的：哪天有人"顺手"加个 NMS，重叠的两个箱子就会被合并成一个。
    """
    raw = np.array([[10, 10, 100, 100, 0.9, 0.0],
                    [12, 12, 102, 102, 0.8, 0.0]])
    dets = decode(raw, 1.0, (0.0, 0.0), conf_thr=0.25)
    check("重叠框都保留（无 NMS）", len(dets) == 2, str(len(dets)))


# --------------------------------------------------------------------------- #
# 选箱打分：置信度 + 面积 + 中心深度
# --------------------------------------------------------------------------- #
def test_depth_score_monotone() -> None:
    """深度项**越近越高**，两端被 clamp。"""
    check("z_near 得满分", depth_score(500.0) == 1.0, str(depth_score(500.0)))
    check("更近也满分（clamp）", depth_score(100.0) == 1.0, str(depth_score(100.0)))
    check("z_far 得 0", depth_score(1500.0) == 0.0, str(depth_score(1500.0)))
    check("更远也 0（clamp）", depth_score(3000.0) == 0.0, str(depth_score(3000.0)))
    check("中点是 0.5", abs(depth_score(1000.0) - 0.5) < 1e-12,
          str(depth_score(1000.0)))
    # **单调性**：整条线上不能有反转
    zs = [400.0 + 50.0 * i for i in range(25)]
    vals = [depth_score(z) for z in zs]
    check("沿距离单调不增", all(a >= b for a, b in zip(vals, vals[1:])), str(vals))
    check("自定义边界生效",
          depth_score(500.0, z_near=0.0, z_far=1000.0) == 0.5,
          str(depth_score(500.0, z_near=0.0, z_far=1000.0)))
    # 边界写反时退化成阶跃，**不崩也不给 NaN**
    check("边界写反退化成阶跃（近=1）",
          depth_score(100.0, z_near=1500.0, z_far=500.0) == 1.0, "写反应给 1")
    check("边界写反退化成阶跃（远=0）",
          depth_score(2000.0, z_near=1500.0, z_far=500.0) == 0.0, "写反应给 0")


def test_sample_depth_mm_median() -> None:
    """中心块取**中位数**，不是均值、不是中心点。"""
    d = np.full((200, 200), 1000.0)
    box = (80.0, 80.0, 120.0, 120.0)             # 中心 (100,100)
    check("整块同值时取该值", sample_depth_mm(d, box) == 1000.0,
          str(sample_depth_mm(d, box)))

    # 中心点本身是空洞，但周围有值 —— **中位数能兜住，单点取法会返回 None**
    d2 = np.full((200, 200), 1000.0)
    d2[100, 100] = 0.0
    got = sample_depth_mm(d2, box)
    check("中心是空洞也能取到", got == 1000.0, str(got))

    # 少量飞点不影响中位数（若用均值会被拉飞）
    d3 = np.full((200, 200), 1000.0)
    d3[95:106, 95:106] = 65535.0                 # 越界哨兵，应被 z_far 滤掉
    check("越界哨兵被过滤", sample_depth_mm(d3, box) == 1000.0,
          str(sample_depth_mm(d3, box)))

    # 全空洞 → None（不是 0，也不是抛）
    check("全空洞返回 None", sample_depth_mm(np.zeros((200, 200)), box) is None,
          str(sample_depth_mm(np.zeros((200, 200)), box)))
    check("None 深度图返回 None", sample_depth_mm(None, box) is None, "应返回 None")

    # 越界裁剪：框中心在图像角上
    corner = sample_depth_mm(np.full((200, 200), 700.0), (0.0, 0.0, 10.0, 10.0))
    check("越界裁剪后仍取到", corner == 700.0, str(corner))
    # 中心完全在图外 → None（不崩）
    outside = sample_depth_mm(np.full((200, 200), 700.0), (-500.0, -500.0, -400.0, -400.0))
    check("中心在图外返回 None", outside is None, str(outside))
    # half=0 时退化成单点
    d4 = np.full((200, 200), 1000.0)
    d4[100, 100] = 800.0
    check("half=0 退化成中心单点",
          sample_depth_mm(d4, box, half=0) == 800.0,
          str(sample_depth_mm(d4, box, half=0)))


def test_candidate_score_weights_all_matter() -> None:
    """**三个权重各起各的作用** —— 改任何一个都必须能改变选择结果。

    这是"参数真的接上了"的唯一判据：只测公式算得对不够，得测它**能翻转选择**。
    """
    # A：conf 高、框小；B：conf 低、框大。两者深度相同。
    a = {"box_uv": [0, 0, 100, 100], "confidence": 0.9, "class_id": 0}
    b = {"box_uv": [0, 0, 400, 400], "confidence": 0.5, "class_id": 0}
    zs = [1000.0, 1000.0]

    # 权重压在 conf 上 → 挑 A
    got = pick_best([a, b], z_list=zs, w_conf=1.0, w_area=0.0, w_depth=0.0)
    check("w_conf 主导时挑 conf 高的", got is a, "挑错了")
    # 权重压在面积上 → 挑 B
    got = pick_best([a, b], z_list=zs, w_conf=0.0, w_area=1.0, w_depth=0.0)
    check("w_area 主导时挑面积大的", got is b, "挑错了")

    # 权重压在深度上 → 挑近的。conf/面积都偏向 B，只有深度偏向 A
    b_far = {"box_uv": [0, 0, 400, 400], "confidence": 0.5, "class_id": 0}
    got = pick_best([a, b_far], z_list=[600.0, 1400.0],
                    w_conf=0.0, w_area=0.0, w_depth=1.0)
    check("w_depth 主导时挑近的", got is a, "挑错了")

    # 三个权重都给 0 → 全部 0 分，取第一个（**不崩、不除零**）
    got = pick_best([a, b], z_list=zs, w_conf=0.0, w_area=0.0, w_depth=0.0)
    check("权重全 0 时取第一个", got is a, "应取第一个")


def test_candidate_score_depth_missing_degrades() -> None:
    """取不到深度 → **权重让给另两项**（不罚、不丢候选）。"""
    # 两个候选 conf 相同、面积不同，**都没深度** —— 面积项仍要能分出高下
    small = {"box_uv": [0, 0, 100, 100], "confidence": 0.8, "class_id": 0}
    big = {"box_uv": [0, 0, 400, 400], "confidence": 0.8, "class_id": 0}
    got = pick_best([small, big], z_list=[None, None])
    check("都没深度时仍按面积分高下", got is big, "降级后面积项没生效")

    # **不罚**：缺深度的近候选，不该输给有深度的远候选（在 conf/面积相同时）
    near_none = {"box_uv": [0, 0, 200, 200], "confidence": 0.8, "class_id": 0}
    far_some = {"box_uv": [0, 0, 200, 200], "confidence": 0.8, "class_id": 0}
    s_none, _ = candidate_score(0.8, 40000.0, None, max_area_px=40000.0)
    s_far, _ = candidate_score(0.8, 40000.0, 1400.0, max_area_px=40000.0)
    check("缺深度不低于有深度的远候选", s_none >= s_far,
          f"none={s_none} far={s_far}")
    got = pick_best([near_none, far_some], z_list=[None, 1400.0])
    check("缺深度的候选不会被远候选挤掉", got is near_none, "被挤掉了")

    # 分数都落在 [0,1] —— 降级路径的归一化不能把量纲搞坏
    _, parts = candidate_score(1.0, 100.0, None, max_area_px=100.0)
    check("降级后 conf 满分是 1.0", parts['conf'] == 1.0, str(parts))
    check("降级后 area 满分是 1.0", parts['area_norm'] == 1.0, str(parts))
    check("降级后 depth 是 None", parts['depth'] is None, str(parts))


def test_score_candidates_single_candidate() -> None:
    """单候选时面积项恒为 1（同帧相对归一的已知退化）。"""
    one = {"box_uv": [0, 0, 100, 100], "confidence": 0.8, "class_id": 0}
    scored = score_candidates([one], [1000.0])
    check("只有一个候选", len(scored) == 1, str(len(scored)))
    check("单候选面积归一 = 1", scored[0][2]['area_norm'] == 1.0,
          str(scored[0][2]))
    # 分数 = 0.5*0.8 + 0.2*1.0 + 0.3*0.5 = 0.75
    check("单候选总分按公式", abs(scored[0][1] - 0.75) < 1e-12,
          str(scored[0][1]))


def test_pick_best_beats_area_only() -> None:
    """★ **本次改动的核心判据**：构造"面积最大但不是目标"的场景。

    目标在手上（近、conf 高），邻箱更大更远。老逻辑（只看面积）挑错，
    新逻辑挑对 —— 这条测试就是这次改动存在的理由。
    """
    target = {"box_uv": [200, 200, 400, 400], "confidence": 0.92, "class_id": 0}
    neighbor = {"box_uv": [0, 0, 600, 600], "confidence": 0.85, "class_id": 0}
    # 目标在手边 700mm，邻箱在远处 1400mm
    zs = [700.0, 1400.0]

    check("前提：邻箱面积确实更大",
          box_area_px(neighbor["box_uv"]) > box_area_px(target["box_uv"]),
          "构造的场景不成立")

    # 老判据：面积最大 → 挑到邻箱（**错的**）
    old = max([target, neighbor],
              key=lambda d: box_area_px(d["box_uv"]))
    check("老逻辑（纯面积）确实挑错", old is neighbor, "构造的场景不成立")

    # 新判据：综合打分 → 挑到手上的那个
    got = pick_best([target, neighbor], z_list=zs)
    check("新逻辑挑对了（手上的箱子）", got is target, str(got))

    # 逐项核一遍分数，免得"碰巧对了"
    scored = {id(d): (s, p) for d, s, p in
              score_candidates([target, neighbor], zs)}
    st, pt = scored[id(target)]
    sn, pn = scored[id(neighbor)]
    check("目标 conf 更高", pt['conf'] > pn['conf'], f"{pt} vs {pn}")
    check("目标面积归一更小", pt['area_norm'] < pn['area_norm'], f"{pt} vs {pn}")
    check("目标深度项更高", pt['depth'] > pn['depth'], f"{pt} vs {pn}")
    check("目标总分更高", st > sn, f"{st} vs {sn}")


def test_pick_best_filters_label() -> None:
    a = {"box_uv": [0, 0, 100, 100], "confidence": 0.9, "class_id": 1}
    b = {"box_uv": [0, 0, 10, 10], "confidence": 0.9, "class_id": 0}
    check("按类别筛", pick_best([a, b], label=0) is b, "label=0 挑错了")
    check("类别不存在返回 None", pick_best([a], label=7) is None, "应返回 None")
    check("空列表返回 None", pick_best([]) is None, "应返回 None")
    # **先筛类别，再在类别内打分**：类别 1 里只有 a，哪怕 b 面积更大也不该赢
    a2 = {"box_uv": [0, 0, 10, 10], "confidence": 0.5, "class_id": 1}
    b2 = {"box_uv": [0, 0, 400, 400], "confidence": 0.99, "class_id": 0}
    check("先筛类别再打分",
          pick_best([a2, b2], label=1) is a2, "类别筛选被打分绕过了")
    check("z_list 与 dets 按位置对齐",
          pick_best([a2, b2], label=None, z_list=[None, 900.0]) is b2,
          "z_list 错位了")


# --------------------------------------------------------------------------- #
# 话题契约
# --------------------------------------------------------------------------- #
def test_polygon_points_order() -> None:
    """`/box/yolo_box` 的契约：`points[0]`=左上、`points[1]`=右下。"""
    pts = box_to_polygon_points((100.0, 200.0, 300.0, 400.0))
    check("两点", len(pts) == 2, str(len(pts)))
    check("points[0] 是左上", pts[0] == (100.0, 200.0), str(pts[0]))
    check("points[1] 是右下", pts[1] == (300.0, 400.0), str(pts[1]))
    # 输入给反了也要能纠正
    pts2 = box_to_polygon_points((300.0, 400.0, 100.0, 200.0))
    check("输入给反也能纠正", pts2 == [(100.0, 200.0), (300.0, 400.0)], str(pts2))


def main() -> int:
    for fn in (test_letterbox_geometry, test_letterbox_round_trip,
               test_letterbox_non_square, test_letterbox_matches_ultralytics,
               test_input_tensor_layout,
               test_decode_filters_low_conf, test_decode_drops_out_of_frame,
               test_decode_empty, test_decode_no_nms_needed,
               test_depth_score_monotone, test_sample_depth_mm_median,
               test_candidate_score_weights_all_matter,
               test_candidate_score_depth_missing_degrades,
               test_score_candidates_single_candidate,
               test_pick_best_beats_area_only, test_pick_best_filters_label,
               test_polygon_points_order):
        print(f"\n--- {fn.__name__} ---")
        fn()
    print()
    if _fails:
        print(f"FAILED: {len(_fails)} 条 —— {_fails}")
        return 1
    print("全部通过")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
