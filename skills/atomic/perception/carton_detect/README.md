# 纸箱顶面四角（OBB）：纯函数算法层

图像 → ONNX(OBB) → 旋转框 → 旋转 NMS → 选箱 → 四角。
**本模块只有纯函数**（零状态、**不 `import rospy`、不 import ultralytics / torch**），
推理本身在 ROS 壳里用 onnxruntime 跑一个 `.onnx`。

ROS 壳：`infrastructure/ros_packages/src/ros_vision/carton_obb/`
（收图像 → 调这里 → 发 `/box/detection`）。

- **接口怎么调** → §1
- **怎么快速验一遍** → §2
- **踩过的坑、改动注意事项** → §3（**坑点全在最后一节**）

---

# 1. 接口

| 函数 | 作用 |
|---|---|
| `to_input_tensor(img_bgr, imgsz)` | letterbox 到 `imgsz`，返回 `(x, ratio, pad)` |
| `decode_obb(raw, ratio, pad, *, conf_thr, orig_shape)` | ONNX 输出 → 旋转框列表 |
| `dets_from_rotated_rects(cx, cy, w, h, angle_deg, scores, ...)` | 公共推理服务的输出（原图坐标的 `cv2.RotatedRect`）→ **与 `decode_obb` 同字段**的列表。过滤/排序/截断/越界口径逐条对齐，有逐字段等价性测试 |
| `nms_rotated(dets, iou_thr, size, *, iomin_thr)` | 旋转框去重（**导出图没有 NMS，必须自己去**）。两条判据：IoU 管「错着盖」，IoMin 管「小框整个套在大框里」 |
| `score_candidates(dets, ref_uv, z_list, ...)` | 选箱打分 |
| `pick_best(dets, ref_uv, ...)` | 取分最高的那个（`score_candidates` + `max` 的便捷封装） |
| `sample_depth_mm(depth_mm, box_uv, ...)` | 在框中心采一块深度（选箱的深度项） |
| `angle_deg_from_corners(corners_uv)` | 「下边（左下→右下）」口径的角度，**不是 OBB 自己的角度** |
| `order_corners_uv` / `quad_from_xywhr` / `letterbox` / `d_center_px` / `d_center_score` / `depth_score` / `candidate_score` | 上面这些用到的分解步骤，单独可测 |

模块级默认常量（都可被 ROS 壳的参数覆盖）：`DEFAULT_NMS_IOU`(0.45) /
`DEFAULT_NMS_IOMIN`(0.85) / `DEFAULT_SCORE_W_DCENTER`(0.5) /
`DEFAULT_SCORE_W_AREA`(0.2) / `DEFAULT_SCORE_W_DEPTH`(0.3) /
`DEFAULT_D_NEAR_PX`(100.0) / `DEFAULT_D_FAR_PX`(450.0) /
`DEFAULT_Z_NEAR_MM`(500.0) / `DEFAULT_Z_FAR_MM`(1500.0) / `DEFAULT_DEPTH_PATCH_PX`(15)

`__init__.py` 把这些都重导出了，可以 `from skills.atomic.perception.carton_detect import decode_obb`。

---

# 2. 快速测试

**不需要 ROS、不需要 onnxruntime、不需要模型文件**：

```bash
cd <仓库根>
python3 skills/atomic/perception/carton_detect/tests/test_obb_decoder.py
```

装了 `ultralytics` 的话会多跑一条**逐帧比对**（拿真模型对真图，与 ultralytics
自己的 `OBBPredictor` 比四点），没装自动跳过。**那条才是真正的判据** ——
手算的断言只能证明「跟我以为的一样」。

---

# 3. 注意事项（踩过的坑）

**1. 这一层不许 `import rospy` / `sensor_msgs`，也不许读模型文件。**
分层硬约束：`skills/` 下的东西要能脱离 ROS 单独单测（见
`pallet_detect/__init__.py` 里同一条说明）。所以「从话题收图」「加载 `.onnx`」
「发话题」全在 ROS 壳里，这里只做数学。

**2. 选箱用的是「离画面下方中心的距离」，不是置信度。** 现场反例（2026-10-13）：
val 的 36 个多框帧里有一帧手箱 conf 0.9773、邻箱 0.9642（**分不开**），
而邻箱面积还更大（46147 > 45118）；「离下方中心最近」挑对了。
距离分离度实测：手标双箱 10 帧中位 **192.9px**。
⚠️ `d_near_px` / `d_far_px` 是 **640×480 口径**；换分辨率要跟着改。

**3. 导出的 ONNX 没有 NMS**（8400 个锚点行全在，同一目标出 2~3 行重复框），
必须自己按**旋转 IoU** 去重 —— 用轴对齐近似会把 45° 的 0.707 算成 0.5。

⚠️ **光有 IoU 不够：模型还会出「只框了箱子一角」的小框。** 它整个套在大框里，
`IoU = 交/并 ≈ 小面积/大面积`（实测碎框 IoU **0.25**、IoMin **1.0**），
**IoU 阈值给多低都放它过去**。所以 `nms_rotated` 有第二条判据
`iomin_thr = 交/较小面积`。两条是**或**的关系，各自守各自的形态：

| 判据 | 管的形态 | 管不着的 |
|---|---|---|
| `iou_thr`（0.45） | 两个框**互相错着盖**（同目标的两条预测） | 包含型（IoU 被面积比压死） |
| `iomin_thr`（0.85） | **小框整个套在大框里** | 擦边的两个真箱子、不重叠的 |

抑制始终**按置信度降序**，所以留下的是置信度最高的那个。
`iomin_thr >= 1.0` = 关掉这条（等价于只有 IoU 的旧行为）。

0.85 的依据（`test_data/carton_data` 22 张手标、全部 5574 个候选对）：

| | 条数 | IoMin 最大值 |
|---|---|---|
| 同一条真值箱的两条重复预测（该杀） | 3134 | 1.000 |
| **两条不同真值箱**（杀了就是误伤） | 1565 | **0.460** |

0.50 就能做到误伤 0（比 0.85 多杀 2 对）；取 0.85 留 0.39 的余量。
**别往 0.50 以下压** —— 擦边的真箱子能到 0.46。

⚠️ **「取置信度最高」有理论弱点，但目前够不到。** 那 22 张上有 11 对是
**小框 conf 更高、而大框才对**（image1：小框 0.646/面积 15168 匹配不上任何真值箱，
大框 0.543/面积 59661 对真值 IoU 0.924）。好在这类**在 IoU 那一关就被处理掉了**，
落到 IoMin 这一关的残留集里「高分是小框」是 **0 条**。换模型后要重验这一条。

**4. `angle_deg` 是「下边（左下→右下）」的角度**，不是 OBB 自己的角度：
OBB 给的是「长边相对水平」，箱子转 90° 时两者差一个直角。

**5. `decode_obb` 假定 ONNX 第 6 列是「原始弧度」。** 这条是实测定的；换导出脚本
（比如 yolo26 的 `OBB26` 头）要先核对输出格式，否则角度会整体偏。
`export_onnx.py --verify` 会核对这一条。

**6. ⚠️ 托盘链（`pallet_detect/detect.py`）里有一份「照抄」的实现。**
两边的 OBB 前/后处理现在**靠手工保持一致**。把它们并成一份是**单独一轮**的事 ——
会给已经验到逐位一致的托盘链引入数值风险，必须单独用两边的回归网验证。
**2026-10-15 这一轮只做了搬迁，没碰数值。**

**7. 本模块 2026-10-15 从 ROS 包里搬到这里**（原 `infrastructure/.../carton_obb/scripts/obb_decoder.py`）。
搬的理由是它与 `pallet_detect` 同构：**算法进 `skills/`、ROS 壳进 `infrastructure/`**。
那次之前，它是仓库里唯一一个算法直接住在 ROS 包里的检测链。

**8. 权重与导出脚本也跟着算法走**（2026-10-19 补上那次搬迁漏掉的一环）：

    carton_detect/models/carton_obb.onnx     权重（**不进 git**，见该目录 models/README.md）
    carton_detect/scripts/export_onnx.py     .pt → .onnx，在**有 ultralytics 的开发机**上跑

默认权重路径是本包的 `DEFAULT_MODEL_PATH`（在 `obb_decoder.py` 里），ROS 节点只**取用**、
不自己算 —— 与托盘那条链一致（`pallet_detect.detect.DEFAULT_MODEL_PATH`）。

⚠️ 搬之前权重在 `infrastructure/.../carton_obb/models/`、由节点用
`Path(__file__).parent.parent` 算出来，于是**纯算法包反而依赖 ROS 包的目录结构**才能
拿到权重（想脱离 ROS 复用算法的人撞上才知道）。
