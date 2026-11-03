# 权重放这里，**不进 git**

`models/` 在 git 里只有 `.gitignore` 和本文件 —— 与
`skills/atomic/perception/carton_detect/models/` 同一做法。

## 文件

    pallet_obb.onnx     托盘 OBB 权重（yolo26s-obb 训练导出）

⚠️ **托盘检测必须要有这个模型。** `detect.detect_pallet_frame` 拿不到可用模型时
**直接抛异常**（不静默返回 `None`、不返回空结果）—— 理由：`None` 在下游是合法的
"本帧没检出"，模型缺失混进去后症状是"一直检不到"，与"现场真没有托盘"无法区分。

## 当前用的

    pallet_obb.onnx     yolo26s-obb，2026-10-18 训练（130 epoch，早停）
                        数据 test_data/pallet_obb26（1482 帧，640x640 画布）
                          train 1375 / val 107；val 全是旧场景
                        输出 (1, 6, 8400)、第 6 列**原始弧度**（已核）
                        39.24 MB，md5 753f6b17469c21ba276bafe3ef7e197d
                        val：P=0.999 R=1.0 mAP50=0.995 mAP50-95=0.995
                        机器人上旧版备份为 pallet_obb.onnx.bak-1018

## 上一版

    md5 a92cd130777a2693ebd4817f54b09348   2026-10-17，yolo26s-obb，
                                            1371 帧（train 1264 / val 107）
    md5 49a101d381afd732baca7b0dd8ad556c   2026-10-12，yolo11s-obb，
                                            quad_obb_dataset（601 帧 640x480）

## TensorRT engine（2026-10-20）

`pallet_obb_fp32.engine` / `pallet_pose_fp32.engine` 与本目录的 `.onnx` 并列，
**不进 git**。必须在**目标机**上用**本机 TensorRT 版本**导出（engine 与设备/版本
绑定，换机要重导）：

    cd <本包>/models
    /usr/src/tensorrt/bin/trtexec --onnx=pallet_obb.onnx \
        --saveEngine=pallet_obb_fp32.engine --noTF32 --buildOnly
    /usr/src/tensorrt/bin/trtexec --onnx=pallet_pose.onnx \
        --saveEngine=pallet_pose_fp32.engine --noTF32 --buildOnly

Orin 上约 3~15 分钟（构建阶段是纯 CPU tactic 搜索，`--buildOnly` 跳过推理）。
验证：

    trtexec --loadEngine=pallet_obb_fp32.engine --warmUp=5 --duration=2

**默认用 FP32（`--noTF32`）**：10 帧实测与 ONNX 的角点差中位 ≤0.01px。
同目录也可以放 `*_fp16.engine`，速度略快（POSE 42ms vs 50ms/帧），但 POSE 的
FP16 关键点亚像素差会被四边求交放大（中位 1.4px、最大 22px）—— 要精度就用 FP32，
要极致速度再用 `~engine_path` / `~pose_engine_path` 指到 fp16。

节点的 `~infer_backend`（2026-10-20）：`auto`（默认，文件在就用 engine）/
`engine`（强制）/ `onnx`（强制）。engine 存在但加载失败**直接报错**，不静默退回
ONNX。engine 的输入必须是 `(1,3,640,640)`、输出 `(1,6,8400)`（OBB）或
`(1,41,8400)`（POSE）—— `trt_backend.TensorRTSession` 会校验输入尺寸。

## 换权重
    # 1) 开发机上导出（要有 ultralytics；机器人上不需要）
    cd <本包>/scripts
    OMP_NUM_THREADS=1 python3 export_onnx.py --weights /path/to/best.pt --verify

    # 2) 拷到这台机器。scp 走不通就用管道（本机到机器人只认公钥，scp 会
    #    Permission denied 且**退出码还是 0**，别被它骗了）：
    cat pallet_obb.onnx | ssh <robot> "cat > <临时路径>"   # 传完 md5sum 核对

⚠️ **`--verify` 必须看输出形状。** 正确是 `(1, 6, 8400)`。

⚠️ **yolo26-obb 必须关掉端到端分支导出**（脚本默认已关，`--end2end` 才开）。
`yolo26-obb.yaml` 里 `end2end: True`，直接导会得到 `(1, 300, 7)` 的已去重格式；
而 `detect._decode` 只认 `(1, 6, 8400)` —— 它 `reshape(-1, 6)` 会把 300x7 强行切成
350 个"候选"，conf 列取到 w/h 那种几百的量纲，**不报错，直接解出 199 个垃圾框**。
（关掉后走解码器自带的旋转 NMS，精度反而更好：IoU 中位 0.9300 → 0.9379，
p10 0.9125 → 0.9176，多余框 4 → 0。）
