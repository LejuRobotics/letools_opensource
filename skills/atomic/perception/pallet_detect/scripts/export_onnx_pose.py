#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""在有 ultralytics 的开发机上跑一次，把 YOLO-pose 的 `.pt` 导成 `.onnx`。

机器人上不需要 ultralytics —— `pose_decode.py` 只依赖 numpy / OpenCV，
推理交给 onnxruntime。

用法:
    OMP_NUM_THREADS=1 python3 export_onnx_pose.py \
        --weights best.pt --verify

输出格式（本仓库解码器的唯一约定）：

    输入  (1, 3, 640, 640)  float32  RGB / 255
    输出  (1, 41, 8400)     float32
          每行 41 列 = [cx, cy, w, h, conf,
                        x0, y0, c0, x1, y1, c1, ..., x11, y11, c11]

    关键点坐标是 **640 letterbox 输入图坐标**（像素），不是归一化坐标。
    `pose_decode.decode_pose` 按 `(u - pad_left) / ratio` 还原到原图。

⚠️ **yolo26-pose 必须关掉端到端分支导出**（脚本默认已关，`--end2end` 才开）。
`yolo26-pose.yaml` 里 `end2end: True`，默认导出是 `(1, 300, 42)` ——
多出来的第 6 列是 **class**，后面的 12 组关键点**整体错位一列**；
而 `pose_decode.decode_pose` 只认 `(1, 41, 8400)`，会把 class 列当 kp0.x，
**不报错、直接解出垃圾**。`--verify` 的形状守卫会拦住，但**别不带 `--verify`**。
"""
from __future__ import annotations

import argparse
import shutil
from pathlib import Path

DEFAULT_OUT = (Path(__file__).resolve().parent.parent
               / 'models' / 'pallet_pose.onnx')
N_COLS = 41  # 4 box + 1 conf + 12 * 3 keypoint


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--weights', type=Path, required=True, help='训练出的 .pt')
    ap.add_argument('--out', type=Path, default=DEFAULT_OUT)
    ap.add_argument('--imgsz', type=int, default=640)
    ap.add_argument('--opset', type=int, default=12)
    ap.add_argument('--device', default='0')
    ap.add_argument('--verify', action='store_true',
                    help='导完用 onnxruntime 跑一张随机图，打印形状与样例')
    # 见模块 docstring：yolo26-pose 默认导出 (1,300,42)，第 6 列是 class，
    # 与 pose_decode 的 41 列布局不兼容。**默认必须关**。
    ap.add_argument('--end2end', action='store_true',
                    help='保留端到端分支（输出 (1,300,42)，**当前解码器不兼容**，默认关）')
    a = ap.parse_args()

    import torch
    import ultralytics
    from ultralytics import YOLO

    print(f'ultralytics {ultralytics.__version__} | torch {torch.__version__}')
    model = YOLO(str(a.weights))
    task = getattr(model.model, 'task', None)
    if task != 'pose':
        raise SystemExit(f'这个权重不是 pose 任务（task={task!r}）—— '
                         f'要用 yolo11/yolo26 的 pose 模型定义训出来的 .pt')

    produced = model.export(format='onnx', imgsz=a.imgsz, opset=a.opset,
                            dynamic=False, simplify=True, device=a.device,
                            end2end=bool(a.end2end))
    a.out.parent.mkdir(parents=True, exist_ok=True)
    shutil.move(str(produced), str(a.out))
    print(f'ONNX -> {a.out}（{a.out.stat().st_size / 1e6:.2f} MB）')

    if not a.verify:
        return

    import numpy as np
    import onnxruntime as ort

    sess = ort.InferenceSession(str(a.out), providers=['CPUExecutionProvider'])
    print('input :', [(i.name, i.shape, i.type) for i in sess.get_inputs()])
    print('output:', [(o.name, o.shape, o.type) for o in sess.get_outputs()])
    out_shape = list(sess.get_outputs()[0].shape)
    if out_shape != [1, N_COLS, 8400]:
        raise SystemExit(
            f'输出形状 {out_shape} 不是 (1, {N_COLS}, 8400)；'
            f'`pose_decode.decode_pose` 按 (N, {N_COLS}) 解码，形状不符会静默解错。'
            f'\n   检查导出参数是否被改动（dynamic / imgsz）。')

    x = np.random.rand(1, 3, a.imgsz, a.imgsz).astype(np.float32)
    raw = sess.run(None, {sess.get_inputs()[0].name: x})[0]
    print(f'样例输出形状 {raw.shape}；转置后 (N, 41) 的前 3 行:')
    print(np.round(raw[0].T[:3], 4))
    print('列序: [cx, cy, w, h, conf, x0, y0, c0, ..., x11, y11, c11]')


if __name__ == '__main__':
    main()
