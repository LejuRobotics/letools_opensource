# 权重放这里，**不进 git**

`models/` 在 git 里只有 `.gitignore` 和本文件，权重文件不随仓库提供，如需获取请联系维护者。

## 当前使用的权重

    carton_obb.onnx     yolo26s-obb，640x640 输入
                        输出 (1, 6, 8400)，第 6 列为原始弧度

## 换权重

    # 1) 开发机上导出（要有 ultralytics；机器人上不需要）
    cd <本包>/scripts
    OMP_NUM_THREADS=1 python3 export_onnx.py --weights /path/to/best.pt --verify

    # 2) 拷到机器人上的本目录，传完用 md5sum 核对

⚠️ **`--verify` 要核对两件事**（换模型一定带上）：

1. **输出形状必须 `(1, 6, 8400)`。**
   `yolo26-obb.yaml` 里 `end2end: True`，直接导会得到 `(1, 300, 7)`
   （已去重已排序）。而 `obb_decoder.decode_obb` **只认 `(1, 6, 8400)`** ——
   它 `reshape(-1, 6)` 会把 300x7 切成 **350 个"候选"**，第 5 列取到 w/h 那种
   几百的量纲，**不报错，直接解出几百个垃圾框**。
   脚本**默认 `end2end=False`**，`--verify` 看到形状不对会直接退出码 1。
   关掉还有个好处：走 one2many + 解码器自带旋转 NMS，精度反而更好。

2. **第 6 列必须是原始弧度**（约 [-1.6, 1.6]）。
   yolo11 的 `OBB` 头对角度做 `(sigmoid(a)-0.25)*pi`，yolo26 的 `OBB26` 不做。
   落在 [0,1] 就是 sigmoid 过的，解码要改 —— 见 `obb_decoder` 的 docstring。

顺带：形状是 `(1, 6, 8400)`，**6 在前**，解码前要转置。
