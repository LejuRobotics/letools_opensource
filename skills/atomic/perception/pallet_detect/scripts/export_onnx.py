#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""在**有 ultralytics 的开发机**上跑一次，把 OBB 的 `.pt` 导成 `.onnx`。

机器人上不需要 ultralytics —— `detect.py` 只认 onnxruntime。
结构照抄 `carton_detect/scripts/export_onnx.py`。

用法:
    OMP_NUM_THREADS=1 python3 export_onnx.py --weights best.pt --verify

⚠️ **导出后必须核对输出格式**：`detect._decode` 假定第 6 列是**原始弧度**。
这条是实测定的（见 `detect.py` 模块 docstring），但不是所有 OBB 头都这样 ——
yolo26 的 `OBB26` 会对角度做 sigmoid。`--verify` 会打印输出形状与几行样例，
换模型时**一定带上**。
"""
from __future__ import annotations

import argparse
import shutil
from pathlib import Path

DEFAULT_OUT = Path(__file__).resolve().parent.parent / 'models' / 'pallet_obb.onnx'


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--weights', type=Path, required=True, help='训练出的 .pt')
    ap.add_argument('--out', type=Path, default=DEFAULT_OUT)
    ap.add_argument('--imgsz', type=int, default=640)
    ap.add_argument('--opset', type=int, default=12)
    ap.add_argument('--device', default='0')
    ap.add_argument('--verify', action='store_true',
                    help='导完用 onnxruntime 跑一张随机图，打印输出形状与样例')
    # ⚠️ **yolo26-obb 必须关掉端到端分支导出。**
    #
    # `yolo26-obb.yaml` 里 `end2end: True`，导出的图会带 one2one + topk，
    # 输出是 `(1, 300, 7) = [cx, cy, w, h, score, class, angle]`（已去重、已排序）。
    # 而 `detect._decode`（照抄 carton_obb/obb_decoder.py）**只认 `(1, 6, 8400)`**：
    # 它 `reshape(-1, 6)` 之后会把 300x7 的 2100 个数强行切成 350 个"候选"，
    # conf 列取到的是 w/h 那种几百的量纲 —— **不报错，直接解出 199 个垃圾框**。
    #
    # 关掉之后输出 `(1, 6, 8400)`，与解码器逐字对得上（走它自带的旋转 NMS）。
    # 精度实测：关掉更好（见 models/README.md 的对照表）。
    ap.add_argument('--end2end', action='store_true',
                    help='保留端到端分支（输出 (1,300,7)，**当前解码器不兼容**，默认关）')
    a = ap.parse_args()

    import torch
    from ultralytics import YOLO
    import ultralytics

    print(f'ultralytics {ultralytics.__version__} | torch {torch.__version__}')
    model = YOLO(str(a.weights))
    if getattr(model.model, 'task', None) != 'obb':
        raise SystemExit(f'这个权重不是 OBB 任务（task={model.model.task!r}）—— '
                         f'要用带 -obb 的模型定义训出来的 .pt')

    produced = model.export(format='onnx', imgsz=a.imgsz, opset=a.opset,
                            dynamic=False, simplify=True, device=a.device,
                            end2end=bool(a.end2end))
    a.out.parent.mkdir(parents=True, exist_ok=True)
    shutil.move(str(produced), str(a.out))
    print(f'ONNX -> {a.out}（{a.out.stat().st_size / 1e6:.2f} MB）')

    if a.verify:
        import numpy as np
        import onnxruntime as ort
        sess = ort.InferenceSession(str(a.out), providers=['CPUExecutionProvider'])
        print('input :', [(i.name, i.shape, i.type) for i in sess.get_inputs()])
        print('output:', [(o.name, o.shape, o.type) for o in sess.get_outputs()])
        out_shape = [o.shape for o in sess.get_outputs()][0]
        x = np.random.rand(1, 3, a.imgsz, a.imgsz).astype(np.float32)
        raw = sess.run(None, {sess.get_inputs()[0].name: x})[0]
        if len(out_shape) == 3 and out_shape[1] == 6:
            print(f'样例输出形状 {raw.shape}；转置后前 3 行（[cx, cy, w, h, conf, angle]）:')
            print(np.round(raw[0].T[:3], 4))
            print('\n⚠️ 核对第 6 列：本模块假定它是**原始弧度**（大约在 [-1.6, 1.6] 附近）。'
                  '\n   如果样例里的数落在 [0, 1] 且形状像概率，那是 sigmoid 过的角度，'
                  '\n   解码要改成 (sigmoid(a)-0.25)*pi/2 —— 见 detect.py 的 docstring。')
        else:
            print(f'⚠️⚠️ 输出形状 {out_shape} 不是 (1, 6, 8400) —— '
                  f'`detect._decode` **解不了**这种格式（会静默解出几百个垃圾框）。'
                  f'\n   去掉 --end2end 重新导出。')


if __name__ == '__main__':
    main()
