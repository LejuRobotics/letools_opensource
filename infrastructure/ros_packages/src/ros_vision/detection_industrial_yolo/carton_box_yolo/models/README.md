# 模型权重放这里（**不进 git**）

本目录在 git 里只有一个 `.gitignore`（`*` + `!.gitignore`），权重靠**手工拷贝**
分发 —— 与 `module_internal/zhaofeng_feeding/yolo_weights/` 同一做法。

## 需要什么

`best_cartonbb.onnx`（默认名，可在 `config/carton_box_yolo.yaml` 的 `model_path` 改）。

## 怎么得到它

在有 ultralytics 的**开发机**上跑一次导出（机器人上不需要 ultralytics）：

```bash
cd <本包>/scripts
OMP_NUM_THREADS=1 python3 export_onnx.py \
    --weights /path/to/best_cartonbb.pt \
    --verify
```

`--verify` 会把导出的 ONNX 跟 `.pt` 比一遍**同一份输入张量下的原始输出**，
应当 < 1e-3。

然后拷到机器人上本目录：

```bash
scp best_cartonbb.onnx <robot>:<repo>/infrastructure/ros_packages/src/ros_vision/\
detection_industrial_yolo/carton_box_yolo/models/
```

## 为什么发 ONNX 不发 .pt

`.pt` 把网络结构存成 yaml 描述，加载时由**当前装的 ultralytics** 去实例化。
同一个权重在不同版本下给出**不同的数**（实测 8.4.41 vs 8.3.163：conf 差 0.075、
框差 2.4px），而且**不报错**。LeTools 里现成的 pin 是 `ultralytics==8.3.163`
（`third_party/basket_vision` 的 jetpack5 运行时清单），与训练用的 8.4.41 不一致。

导成 ONNX 之后结构冻结进图里，推理端只认 onnxruntime，这条漂移消失。
代价是文件大一点（9.8MB vs 5.4MB）和需要多装一个 onnxruntime。
