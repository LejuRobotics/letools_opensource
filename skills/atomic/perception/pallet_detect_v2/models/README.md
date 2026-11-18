# 权重放这里，**不进 git**

`models/` 在 git 里只有 `.gitignore` 和本文件 —— 与
`skills/atomic/perception/carton_detect/models/` 同一做法。

## 文件

    pallet_pose.onnx     托盘 12 关键点 pose 权重

⚠️ **托盘检测必须要有这个模型。** `detect_pose.detect_pallet_frame_pose` 拿不到可用
模型时**直接抛异常**（不静默返回 `None`、不返回空结果）—— 理由：`None` 在下游是合法的
"本帧没检出"，模型缺失混进去后症状是"一直检不到"，与"现场真没有托盘"无法区分。

## 当前用的

    pallet_pose_fp32.engine   共享 v1/v2 的 POSE engine；当前 md5 95775a3a3efba6fa4e652283e899ef19
    pallet_pose.onnx          构建源，不进 git；输出 (1,41,8400)，640×640 letterbox

## TensorRT engine（2026-10-20）

`pallet_pose_fp32.engine` 是指向共享 `pallet_detect/models/` 的相对软链（v1/v2 共用）。必须在**目标机**上
用**本机 TensorRT 版本**导出（engine 与设备/版本绑定，换机要重导）：

    cd <本包>/models
    /usr/src/tensorrt/bin/trtexec --onnx=pallet_pose.onnx \
        --saveEngine=pallet_pose_fp32.engine --noTF32 --buildOnly

Orin 上约 3~15 分钟（构建阶段是纯 CPU tactic 搜索，`--buildOnly` 跳过推理）。
验证：

    trtexec --loadEngine=pallet_pose_fp32.engine --warmUp=5 --duration=2

**默认用 FP32（`--noTF32`）**：10 帧实测与 ONNX 的角点差中位 ≤0.01px。
同目录也可以放 `*_fp16.engine`，速度略快（POSE 42ms vs 50ms/帧），但 FP16 的关键点
亚像素差会被四边求交放大（中位 1.4px、最大 22px）—— 要精度就用 FP32，
要极致速度再用 `~pose_engine_path` 指到 fp16。

节点的 `~infer_backend`：`auto`（默认，文件在就用 engine）/ `engine`（强制）/
`onnx`（强制）。engine 存在但加载失败**直接报错**，不静默退回 ONNX。
engine 的输入必须是 `(1,3,640,640)`、输出 `(1,41,8400)` ——
`trt_backend.TensorRTSession` 会校验输入尺寸。

## 换权重
    # 1) 开发机上导出（要有 ultralytics；机器人上不需要）
    cd <本包>/scripts
    OMP_NUM_THREADS=1 python3 export_onnx_pose.py --weights /path/to/best.pt --verify

    # 2) 拷到这台机器。scp 走不通就用管道（本机到机器人只认公钥，scp 会
    #    Permission denied 且**退出码还是 0**，别被它骗了）：
    cat pallet_pose.onnx | ssh <robot> "cat > <临时路径>"   # 传完 md5sum 核对

⚠️ **`--verify` 必须看输出形状。** 正确是 `(1, 41, 8400)`：4 个框参数 + 1 个 conf +
12 关键点 × 3。形状不对时解码器仍可能切出一堆"候选"，不报错但全是垃圾框。
