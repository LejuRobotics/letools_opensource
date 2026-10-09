"""旋转模拟测试：把一帧绕光轴转 5/10/15 度，模拟「箱子不平行于相机」。

场景：箱子在图像里转了角度时，YOLO 只会给出**轴对齐外接框** —— 角度信息被丢掉。
本测试看 `fit_box_frame` 能否从外接框里恢复真实的旋转四角。

规则（操作员 2026-09-20 定，防透题）：
  * **只允许**用 `boxes.json` 里那个人工框推算旋转后的 YOLO 框；
  * **不允许**用任何本次跑出来的中间结果（质心深度、质心位置等）去调算法；
  * 一次跑完，结果如实报告。

跑法（需要 maduo 的数据集，见 §注意事项）：
  cd <仓库根>
  python3 -m skills.atomic.perception.box_frame.tests.test_rotation_sim \
      --sequence <数据集>/5_test --boxes <数据集>/box_out/5_test/boxes.json
  # 6_test 有四角真值文件，走 --gt：
  python3 -m skills.atomic.perception.box_frame.tests.test_rotation_sim \
      --sequence <数据集>/6_test --stem 1789543228.215563 \
      --boxes <数据集>/box_out/6_test/boxes.json \
      --gt    <数据集>/box_out/6_test/corners_gt.json
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

# 数据（test_data / box_out）**不在本仓库**，默认路径按老习惯指到仓库根的上一级
# 的 maduo 目录；跑的时候用 --sequence / --boxes / --gt 显式给也行。
ROOT = Path(__file__).resolve().parents[5]
DATA_ROOT = ROOT.parent / 'maduo'

from ..algorithm import fit_box_frame, load_frame        # noqa: E402

# 5_test 的真值只有下边两个角（操作员 2026-09-20 给的），没有 corners_gt.json。
# 6_test 有四角真值文件，走 --gt。两者都能评。
FALLBACK_TRUE = {
    '1789543127.504033': (np.array([245.8, 449.3]), np.array([426.2, 452.5])),
}
ANGLES = (0.0, 5.0, 10.0, 15.0)
NAMES = ('右下', '左下', '左上', '右上')


def rotate(color, depth, angle_deg, centre=None):
    """绕图像中心旋转 RGB-D。depth 用最近邻以保持 uint16 与 0 的无效语义。"""
    h, w = depth.shape
    if centre is None:
        centre = (w / 2.0, h / 2.0)
    M = cv2.getRotationMatrix2D(centre, angle_deg, 1.0)
    c = cv2.warpAffine(color, M, (w, h), flags=cv2.INTER_LINEAR,
                       borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))
    d = cv2.warpAffine(depth, M, (w, h), flags=cv2.INTER_NEAREST,
                       borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    return c, d, M


def apply_M(M, pts):
    pts = np.asarray(pts, np.float64).reshape(-1, 2)
    return (M[:, :2] @ pts.T).T + M[:, 2]


def bbox_of(pts):
    p = np.asarray(pts, np.float64).reshape(-1, 2)
    return (float(p[:, 0].min()), float(p[:, 1].min()),
            float(p[:, 0].max()), float(p[:, 1].max()))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--sequence', type=Path, default=DATA_ROOT / 'test_data' / '5_test')
    ap.add_argument('--stem', default='1789543127.504033')
    ap.add_argument('--boxes', type=Path, default=None)
    ap.add_argument('--gt', type=Path, default=None)
    ap.add_argument('--out', type=Path, default=None)
    args = ap.parse_args()

    seq = args.sequence
    boxes_path = args.boxes or (DATA_ROOT / 'box_out' / seq.name / 'boxes.json')
    out = args.out or (DATA_ROOT / 'box_out' / seq.name / 'rotation_sim')
    out.mkdir(parents=True, exist_ok=True)

    boxes = json.loads(Path(boxes_path).read_text(encoding='utf-8'))
    box0 = boxes[args.stem]                       # 人工框（唯一允许的输入）
    box0_quad = np.array([[box0[0], box0[1]], [box0[2], box0[1]],
                          [box0[2], box0[3]], [box0[0], box0[3]]], np.float64)

    gt4 = None
    if args.gt is not None:
        gt4 = np.asarray(json.loads(Path(args.gt).read_text(encoding='utf-8'))[args.stem],
                         np.float64)              # 右下 左下 左上 右上
    elif args.stem in FALLBACK_TRUE:
        gt4 = np.asarray(FALLBACK_TRUE[args.stem], np.float64)
    if gt4 is None:
        print(f'{seq.name}/{args.stem}: 没有真值可用，只能出图不能打分')

    color0, depth0, k = load_frame(seq, args.stem)
    print(f'{seq.name} / {args.stem}   图像 {depth0.shape[1]}x{depth0.shape[0]}   '
          f'人工框 {tuple(round(float(v), 1) for v in box0)}')
    rows, tiles = [], []

    for ang in ANGLES:
        if ang == 0.0:
            color, depth = color0.copy(), depth0.copy()
            M = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
        else:
            color, depth, M = rotate(color0, depth0, ang)

        # —— 只由人工框推算旋转后的 YOLO 框（允许的输入）——
        box = bbox_of(apply_M(M, box0_quad))
        gt = apply_M(M, gt4) if gt4 is not None else None

        result, diag = fit_box_frame(color, depth, k, box, target_mm=(530.0, 350.0))
        if result is None:
            rows.append(dict(angle=ang, box=list(box), reject=diag.get('reject')))
            print(f'{ang:5.1f}°  FAILED  reject={diag.get("reject")}')
            tiles.append(color.copy())
            continue

        uv = np.asarray(result['corners_uv'])
        row = dict(angle=ang, box=list(box), corners=uv.tolist(),
                   picked=diag.get('candidate_choice', {}).get('picked'),
                   conf=diag.get('confidence'))
        msg = f'{ang:5.1f}°  '
        if gt is not None:
            # gt 是 4 个角就逐角评；只有 2 个（5_test 的 fallback）就只评下边两角
            full = (len(gt) == 4)
            lb_t, rb_t = (gt[1], gt[0]) if full else (gt[0], gt[1])
            e_lb = float(np.hypot(*(uv[1] - lb_t)))
            e_rb = float(np.hypot(*(uv[0] - rb_t)))
            row['err_lb'], row['err_rb'] = round(e_lb, 1), round(e_rb, 1)
            if full:
                row['err_px'] = [round(float(t), 1) for t in np.hypot(*(uv - gt).T)]
                msg += ' '.join(f'{NAMES[i]} {row["err_px"][i]:5.1f}' for i in range(4))
            else:
                msg += f'左下 {e_lb:5.1f} 右下 {e_rb:5.1f}  (只有下边两角真值)'
            # 黄框基线：直接用 YOLO 框的两个下角当答案
            row['baseline_lb'] = round(float(np.hypot(box[0] - lb_t[0], box[3] - lb_t[1])), 1)
            row['baseline_rb'] = round(float(np.hypot(box[2] - rb_t[0], box[3] - rb_t[1])), 1)
            msg += f'   | 基线 {row["baseline_lb"]:5.1f} / {row["baseline_rb"]:5.1f}'
        a_got = np.degrees(np.arctan2(uv[0, 1] - uv[1, 1], uv[0, 0] - uv[1, 0]))
        row['angle_deg'] = round(float(a_got), 2)
        msg += f'   下边角度 {a_got:+6.2f}°'
        if gt is not None:
            lb_t2, rb_t2 = (gt[1], gt[0]) if len(gt) == 4 else (gt[0], gt[1])
            a_tru = np.degrees(np.arctan2(rb_t2[1] - lb_t2[1], rb_t2[0] - lb_t2[0]))
            row['true_angle_deg'] = round(float(a_tru), 2)
            msg += f' / 真值 {a_tru:+6.2f}° (差 {a_got - a_tru:+5.2f}°)'
        msg += f'   {row["picked"]} {row["conf"]}'
        print(msg)
        rows.append(row)

        tile = color.copy()
        cv2.polylines(tile, [np.round(uv).astype(np.int32)], True, (0, 0, 255), 2)
        # 黄实线 = **算法实际收到的输入框**（旋转后可见区域的轴对齐外接框）
        cv2.rectangle(tile, (int(round(box[0])), int(round(box[1]))),
                      (int(round(box[2])), int(round(box[3]))), (0, 255, 255), 1)
        # 灰细线 = 旋转后的人工框轮廓（对照：说明输入框不是它，是它的外接框）
        cv2.polylines(tile, [np.round(apply_M(M, box0_quad)).astype(np.int32)], True,
                      (130, 130, 130), 1)
        if gt is not None:
            for p in gt:
                cv2.circle(tile, (int(round(p[0])), int(round(p[1]))), 5, (0, 255, 0), -1)
        cv2.putText(tile, f'{ang:.0f}deg', (8, 24), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (255, 255, 255), 2)
        tiles.append(tile)

    (out / 'result.json').write_text(
        json.dumps(rows, ensure_ascii=False, indent=2), encoding='utf-8')
    valid = [r for r in rows if 'err_lb' in r]
    if valid:
        e_lb = np.mean([r['err_lb'] for r in valid])
        e_rb = np.mean([r['err_rb'] for r in valid])
        print()
        print(f'成功 {len(valid)}/{len(rows)} 帧   左下 {e_lb:5.1f}  右下 {e_rb:5.1f}   '
              f'角度差 {np.mean([r["angle_deg"] - r["true_angle_deg"] for r in valid]):+5.2f}°')
        print('基线  ' + f'左下 {np.mean([r["baseline_lb"] for r in valid]):5.1f}'
              f' 右下 {np.mean([r["baseline_rb"] for r in valid]):5.1f}')
        four = [r for r in valid if 'err_px' in r]
        if four:
            e = np.asarray([r['err_px'] for r in four], float)
            print('四角均值  ' + ' '.join(f'{NAMES[i]} {e[:, i].mean():5.1f}'
                                          for i in range(e.shape[1]))
                  + f'   总 {e.mean():5.1f}')
    failed = [r for r in rows if 'reject' in r]
    if failed:
        print('失败帧: ' + ', '.join(f'{r["angle"]:.0f}°({r["reject"]})' for r in failed))
    if tiles:
        h = max(t.shape[0] for t in tiles)
        w = max(t.shape[1] for t in tiles)
        grid = np.full((2 * h + 6, 2 * w + 6, 3), 18, np.uint8)
        for i, t in enumerate(tiles[:4]):
            r, c = divmod(i, 2)
            grid[r * (h + 6):r * (h + 6) + t.shape[0],
                 c * (w + 6):c * (w + 6) + t.shape[1]] = t
        cv2.imwrite(str(out / 'rotation_montage.png'), grid)
        print(f'拼图: {out / "rotation_montage.png"}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
