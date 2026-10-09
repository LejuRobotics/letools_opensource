#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""导出 ONNX 权重 —— 在**有 ultralytics 的环境**里跑一次，产物给机器人用。

为什么要有这一步（而不是直接发 `.pt`）：`.pt` 把网络结构存成 yaml 描述，加载时
由**当前装的 ultralytics** 去实例化，于是同一个权重在不同版本下给出**不同的数**
（实测 8.4.41 vs 8.3.163：conf 差 0.075、框差 2.4px，**不报错**）。导成 ONNX 之后
结构冻结进图里，推理端只认 onnxruntime，这条漂移就消失了。

跑法（在**装了 ultralytics 的开发机**上跑一次，机器人上不需要 ultralytics）：
    OMP_NUM_THREADS=1 python3 export_onnx.py --weights /path/to/best_cartonbb.pt

产物默认落在本包的 `models/` 下，与权重同名 + `.onnx`。**`models/` 不进 git**
（见那个目录的 `.gitignore`），部署时手工拷贝。
"""
from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

MODELS_DIR = Path(__file__).resolve().parent.parent / "models"


def main() -> int:
    ap = argparse.ArgumentParser(description="导出纸箱检测的 ONNX 权重")
    ap.add_argument("--weights", type=Path, required=True,
                    help=".pt 权重（如 best_cartonbb.pt）")
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--opset", type=int, default=12,
                    help="12 是稳妥选择；实测 12/17/19 的结果逐位相同")
    ap.add_argument("--out", type=Path, default=None,
                    help="输出路径，默认 models/<权重名>.onnx")
    ap.add_argument("--verify", action="store_true",
                    help="导出后跟 .pt 比一遍原始输出（需要同一台机器上有 ultralytics）")
    args = ap.parse_args()

    if not args.weights.is_file():
        print(f"找不到权重：{args.weights}", file=sys.stderr)
        return 1

    try:
        from ultralytics import YOLO
    except ImportError as exc:
        print(f"需要 ultralytics 才能导出（{exc}）。\n"
              f"这一步在开发机上做一次即可，机器人上只需要 onnxruntime。",
              file=sys.stderr)
        return 1

    model = YOLO(str(args.weights))
    print(f"加载 {args.weights}（task={model.task}, names={model.names}）")
    produced = model.export(format="onnx", imgsz=args.imgsz,
                            opset=args.opset, dynamic=False)

    dst = args.out or (MODELS_DIR / (args.weights.stem + ".onnx"))
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.move(str(produced), str(dst))
    print(f"写到 {dst}  （{dst.stat().st_size / 1e6:.1f} MB）")

    if args.verify:
        # 同一份输入张量下，PyTorch 与 ONNX 的**原始输出**应当几乎相等。
        # 用 predict() 比是错的 —— 那会把 letterbox 模式的差异也算进来。
        #
        # ⚠️ **只比 conf 过阈值的那些行**。300 行里绝大部分是"没有目标"的 query，
        # 它们的 conf≈0、框坐标是没意义的任意大数；不同后端对这些行的浮点结果
        # 可以差几百像素（实测 621px），但**一个都不会被采用**。拿全部 300 行比
        # 会得到一个吓人且毫无意义的数字。
        import numpy as np
        import onnxruntime as ort
        import cv2
        from carton_detector import to_input_tensor

        sess = ort.InferenceSession(str(dst), providers=["CPUExecutionProvider"])
        img = np.zeros((480, 640, 3), np.uint8)
        img[100:400, 100:500] = 180
        x, _, _ = to_input_tensor(img, args.imgsz)
        import torch
        with torch.no_grad():
            y = model.model(torch.from_numpy(x))
            y = y[0] if isinstance(y, (list, tuple)) else y
            ref = y.cpu().numpy()[0]
        got = sess.run(None, {"images": x})[0][0]
        keep = ref[:, 4] >= 0.25
        if keep.sum() == 0:
            print("⚠️ 这张合成图上没有 conf>=0.25 的检测，只比了 conf 列："
                  f"最大差 {np.abs(ref[:, 4] - got[:, 4]).max():.2e}")
        else:
            d = float(np.abs(ref[keep] - got[keep]).max())
            print(f"conf>=0.25 的 {int(keep.sum())} 个检测：原始输出最大差 {d:.2e}"
                  f"（应 < 1e-3）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
