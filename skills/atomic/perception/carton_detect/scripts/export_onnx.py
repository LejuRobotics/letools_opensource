#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""在**有 ultralytics 的开发机**上跑一次，把 OBB 的 `.pt` 导成 `.onnx`。

机器人上不需要 ultralytics —— 节点只认 onnxruntime。

用法:
    OMP_NUM_THREADS=1 python3 export_onnx.py --weights best.pt --verify

⚠️ **导出后必须核对两件事**（`--verify` 都会打印，**换模型一定带上**）：

1. **输出形状必须是 `(1, 6, 8400)`。**
   `yolo26-obb.yaml` 里 `end2end: True`，直接导会带 one2one + topk，
   输出 `(1, 300, 7) = [cx, cy, w, h, score, class, angle]`（已去重已排序）。
   而 `obb_decoder.decode_obb` **只认 `(1, 6, 8400)`**：它 `reshape(-1, 6)` 会把
   300x7 的 2100 个数强行切成 **350 个"候选"**，第 5 列（本该是 conf）取到的是
   w/h 那种几百的量纲 —— **不报错，直接解出几百个垃圾框**。
   所以本脚本**默认 `end2end=False`**（与 `pallet_detect/scripts/export_onnx.py` 同）。
   好处不只是"能解"：关掉后走 one2many + 解码器自带的旋转 NMS，精度反而更好
   （托盘实测 IoU 中位 0.9300 → 0.9379，p10 0.9125 → 0.9176，多余框 4 → 0）。

2. **第 6 列必须是原始弧度**（大约在 [-1.6, 1.6] 附近）。
   这条是实测定的（见 `obb_decoder` 的模块 docstring）。yolo11 的 `OBB` 头对角度
   做 `(sigmoid(a)-0.25)*pi`；yolo26 的 `OBB26` 不做。若样例里的数落在 [0,1]
   且形状像概率，解码要改。
"""
from __future__ import annotations

import argparse
import shutil
from pathlib import Path

DEFAULT_OUT = Path(__file__).resolve().parent.parent / 'models' / 'carton_obb.onnx'


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--weights', type=Path, required=True, help='训练出的 .pt')
    ap.add_argument('--out', type=Path, default=DEFAULT_OUT)
    ap.add_argument('--imgsz', type=int, default=640)
    ap.add_argument('--opset', type=int, default=12)
    ap.add_argument('--device', default='0')
    ap.add_argument('--verify', action='store_true',
                    help='导完用 onnxruntime 跑一张随机图，打印输出形状与样例')
    # 见模块 docstring 第 1 条：**必须关端到端**，否则输出 (1,300,7)，
    # 解码器会静默解出几百个垃圾框。保留 `--end2end` 只为哪天解码器支持了再开。
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
        out_shape = list(sess.get_outputs()[0].shape)
        x = np.random.rand(1, 3, a.imgsz, a.imgsz).astype(np.float32)
        raw = sess.run(None, {sess.get_inputs()[0].name: x})[0]
        if len(out_shape) == 3 and out_shape[1] == 6:
            print(f'样例输出形状 {raw.shape}；转置后前 3 行（[cx, cy, w, h, conf, angle]）:')
            print(np.round(raw[0].T[:3], 4))
            print('\n⚠️ 核对第 6 列：本模块假定它是**原始弧度**（大约在 [-1.6, 1.6] 附近）。'
                  '\n   如果样例里的数落在 [0, 1] 且形状像概率，那是 sigmoid 过的角度，'
                  '\n   解码要改成 (sigmoid(a)-0.25)*pi/2 —— 见 obb_decoder 的 docstring。')
        else:
            print(f'\n❌ 输出形状 {out_shape} 不是 (1, 6, 8400)。'
                  f'\n   `obb_decoder.decode_obb` **解不了**这种格式（会静默解出几百个垃圾框）。'
                  f'\n   去掉 --end2end 重新导出。')
            raise SystemExit(1)


if __name__ == '__main__':
    main()
