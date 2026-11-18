# 托盘 POSE 权重（v1 / v2 共用）

本目录是 v1（`pallet_detect`）和 v2（`pallet_detect_v2`）共用的 POSE 权重目录。
v2 的 `pallet_detect_v2/models/pallet_pose_fp32.engine` 是指向这里的相对软链，
部署时**不要放两份不同的 engine**。

## 当前分支里的文件

```text
pallet_pose_fp32.engine   46,017,261 B   md5 95775a3a3efba6fa4e652283e899ef19
```

- 输出 `(1, 41, 8400)`：4 个框参数 + 1 个 conf + 12 关键点 × 3。
- 在机器人（Orin / TensorRT 8.5）上构建；engine **与设备/TensorRT 版本绑定**。
- 构建源 `pallet_pose.onnx` 不进 git；当前开发机上的 onnx
  `b7c432788e13c9e4ac5dc37003c5f167`（40,619,803 B）只用于重导 engine。

## 重导 engine（必须在目标机）

```bash
cd <本包>/models
/usr/src/tensorrt/bin/trtexec --onnx=pallet_pose.onnx \
  --saveEngine=pallet_pose_fp32.engine --noTF32 --buildOnly
```

Orin 上约 3~15 分钟（`--buildOnly` 跳过推理，构建阶段是纯 CPU tactic 搜索）。
验证：

```bash
trtexec --loadEngine=pallet_pose_fp32.engine --warmUp=5 --duration=2
```

## 推理后端

节点参数 `~infer_backend`：

- `auto`（默认）：engine 在就用 engine，否则 ONNX；
- `engine`：强制 TensorRT；
- `onnx`：强制 ONNX。

engine 存在但加载失败**直接报错**，不静默退回 ONNX。输入必须是
`(1,3,640,640)`，输出必须是 `(1,41,8400)`；`trt_backend.TensorRTSession`
会校验形状。

**默认 FP32（`--noTF32`）**：FP16 的亚像素关键点误差会被四边求交放大，
要精度就用 FP32。

## 换权重

1. 开发机导出（需要 ultralytics）：

   ```bash
   cd <本包>/scripts
   OMP_NUM_THREADS=1 python3 export_onnx_pose.py --weights /path/to/best.pt --verify
   ```

2. 传到目标机（`scp` 到机器人可能静默失败且退出码 0，优先用 ssh 管道/SFTP）：

   ```bash
   cat pallet_pose.onnx | ssh <robot> "cat > /tmp/pallet_pose.onnx"
   # 核对 md5 后在目标机重导 engine
   ```

⚠️ yolo26-obb / pose 导出必须保持正确的输出形状；形状不对时解码器可能仍能切出
“候选”，但全是垃圾框。
