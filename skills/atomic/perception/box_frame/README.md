# 箱子顶面四角（从 YOLO 框恢复旋转角）

输入 **YOLO 给的轴对齐外接框**（框住箱子被遮挡后的可见部分），输出**箱子顶面
完整外轮廓的四个角**（像素坐标，顺序固定 **右下 → 左下 → 左上 → 右上**）。
箱子已知 530×350mm。

**要做的事就是「恢复被 YOLO 丢掉的那个旋转角」** —— 只想要外接矩形的话根本
不需要这个模块。**只出四角，不做伺服控制。**

```text
rosbag / 相机 ──/camera/color/image_raw──→  carton_box_yolo ──/box/yolo_box──┐
              ──/camera/depth/image_raw──────────────────────────┐          │
              ──/camera/color/camera_info──────────────────────┐ │          │
                                                               ↓ ↓          ↓
                                                    box_detection（本模块）
                                                               ↓
                                                       /box/detection
                                                 （四角 + valid + angle_deg）
```

| 文件 | 作用 |
|---|---|
| `algorithm.py` | **纯函数核心**：平面分割、边观测、候选求解、投影排序。零框架依赖、零 ROS |
| `window.py` | 滑动时间窗（**有状态**，但也不 import ROS）：每 3 帧处理一次、窗口 5 帧求平均 |
| `tests/test_fit_box_frame.py` | 合成自检 + 窗口/朝向测试（201 条断言） |
| `tests/test_rotation_sim.py` | 旋转鲁棒性（绕图像中心转 0/5/10/15°） |
| `infrastructure/.../box_detection/` | ROS 节点：把上面两层接到话题上 |
| `infrastructure/.../carton_box_yolo/` | **上游**：YOLO 检测，发 `/box/yolo_box` |
| `infrastructure/.../box_detection_msgs/` | `/box/detection` 的消息定义 |

# 1. 接口

## 算法层

```python
from skills.atomic.perception.box_frame import fit_box_frame

result, diag = fit_box_frame(color, depth, k, box_uv,
                             target_mm=(530.0, 350.0), opts=None)
```

| 参数 | 类型 | 说明 |
|---|---|---|
| `color` | `np.ndarray (H,W,3)` | BGR 彩色图 |
| `depth` | `np.ndarray (H,W)` | **uint16，单位毫米**，0 = 无效 |
| `k` | `CameraIntrinsics` | `fx/fy/cx/cy` |
| `box_uv` | `(u0,v0,u1,v1)` | YOLO 轴对齐框 |
| `target_mm` | `(长, 短)` | 箱子实际尺寸，默认 `(530, 350)` |
| `opts` | `dict` | `orientation`（`'horizontal'` / `'vertical'` / `None`）、`z_range`（**必需**）等 |

返回 `(result, diag)`。**失败时 `result` 为 `None`**，`diag['reject']` 写明原因。

| `result` 字段 | 说明 |
|---|---|
| `corners_uv` | 四角像素坐标，顺序 **右下 → 左下 → 左上 → 右上** |
| `angle_deg` | **下边**（左下→右下）的方向角，范围 `(-90, 90]`。正 = 右边比左边低 |

## 时间窗层

```python
from skills.atomic.perception.box_frame import BoxFrameWindow, build_payload

win = BoxFrameWindow(k, target_mm=(530.0, 350.0), process_every=3, window=5)
win.set_orientation(tape_orientation_deg)     # 可选，每帧或收到服务回包时更新
out = win.push(color, depth, box_uv)          # 非处理槽返回 None
if out is not None:
    payload = build_payload(out, box_uv, latency_ms)   # 直接往消息里填
```

`payload['source']` 有三种：`window`（窗里 ≥2 帧成功，四角是逐点平均）/
`single`（只有 1 帧）/ **`yolo_fallback`（窗全失败，四角 = 原始 YOLO 框）**。

## ROS 节点

见 `infrastructure/ros_packages/src/ros_vision/detection_industrial_yolo/box_detection/`
（参数表与话题契约在该包 `scripts/box_detection_node.py` 的文件头 docstring）。
订阅 `image_color` / `image_depth` / `box_in` / `camera_info`，发 `/box/detection`。
消息定义在 `infrastructure/.../ros_vision/box_detection_msgs/`。

# 2. 快速测试

## 2.1 算法层自检（不需要 ROS / 相机 / 数据）

```bash
cd <仓库根>
python3 skills/atomic/perception/box_frame/tests/test_fit_box_frame.py   # 0 通过 / 1 失败
```

**201 条断言，0 FAIL** 是基线。它合成深度图、自己造真值，不读任何图片。
（用 `python3 -m skills.atomic.perception.box_frame.tests.test_fit_box_frame` 跑也行。）

## 2.2 旋转鲁棒性（需要 maduo 的数据集）

```bash
cd <仓库根>
python3 -m skills.atomic.perception.box_frame.tests.test_rotation_sim \
    --sequence /data/Real_Downloads/maduo/test_data/5_test \
    --boxes    /data/Real_Downloads/maduo/box_out/5_test/boxes.json \
    --out      /tmp/rotation_sim
```

6_test 有四角真值文件，多给 `--stem 1789543228.215563`、`--gt .../6_test/corners_gt.json`。

判据是**成功 4/4 帧**（0/5/10/15° 各一）。

## 2.3 端到端（rosbag 回放）

```bash
# 终端 1
roscore
# 终端 2：回放（循环）
rosbag play -l <你的包>.bag
# 终端 3：上游 YOLO（/box/yolo_box 的生产者）
roslaunch carton_box_yolo carton_box_yolo.launch
# 终端 4：本模块
roslaunch box_detection box_detection.launch
# 终端 5
rostopic echo /box/detection
```

判据是 **`valid: true` 且 `source: "window"`**。`source: "yolo_fallback"` 说明窗
全失败、四角退回原始 YOLO 框，**不能拿去用**。

# 3. 注意事项

**① `flat_plane` 是既定前提。** 操作员要求「忽略俯仰角、不要梯形」，所以输出四角
是**正对相机的矩形**，而真实顶面是上窄下宽的梯形 → **两个上角必然各偏 ±13.8px**。
6_test 右上角那 12~14px 基本全是这一项，不是 `theta` 的锅。

**② theta 的手性必须按 frame0 的 `(E1,E2)` 叉积校正。** 不校正的话同一套 theta
会在**一半帧里转反**，**而输出四角照样是规矩的矩形** —— 不比对真值根本看不出来。

**③ 上下边近乎平行，左右才是两腰。** 真值上 +0.91°/下 +1.53°（差 0.62°），
左 −83.44°/右 +87.18°（差 9.38°）。俯仰绕图像水平轴，平行于该轴的边投影后仍平行。
**早期文档写过反的结论，已更正。**

**④ 一条边被机械手挡掉大半时，要「取可用的那一段」。** 做法是按**手**把观测序列
切段、取段内有效点最多的一段；`frac` 的分母要扣掉手点数。
⚠️ 「外侧」的参照点是**矩形中心**（四角投影均值），**不是 `frame['origin']`** ——
后者是矩形的一个角，拿它当中心会让近一半观测点的外侧指反。
⚠️ **判手时不能「遇到无效深度就停」**：吸盘悬空，旁边就是空洞。

**⑤ 「长边配哪条轴」不再枚举。** `extract_line_segments` 强制 e1 指向点集跨度大的
一维，所以 `u±` 永远是短边对、530 必然沿 e2。`long_along_e1=True` 那一支一次都没
对过，它只是给「整框转 90°」留了条容差大 180mm 的旁路。

**⑥ 候选矩形的朝向由 mask 上/下边界加权融合给出。** 不再取自 `u±` 里 `span_mm`
最大的那条边 —— `span_mm` 是「露出来多长」不是「这条边有多长」，手一挡就变，
9_test 上它偏 −11°，候选整体转 12~18°，长边扫不到深度峰、族 x 判无效、`t_e1`
被 held 就再也纠不回来。**上下两条一起用**（实测夹角 0.36~6.64°），按
`_envelope_line` 的 `score` 加权、**2θ 圆周平均**（直线方向有 180° 歧义，线性平均
会在 ±90° 附近翻转）。**长/短边由点集跨度判**（纯比较无阈值）。
⚠️ 方向符号要用 `_inward_dir` 从角点定向。

**⑦ mask 上边界可能其实是 `_clip_to_box` 的裁剪线，不是真箱沿。**
9_test 247/252 列贴着它，8_test 159/186。`BOX_INBOX_PAD_MM = 15mm` 折算 7.3px，
对「箱子在图像里比 YOLO 框大」的序列不够。操作员 2026-09-22 确认「掩码基本上都是
靠 yolo 框裁剪的，掺杂一点相邻箱子的点，影响不大」。
⚠️ 试过「只裁 u 不裁 v」：9_test **0/5**（框上方共面点撑大点集，尺寸判据全挂）。

**⑧ 朝向先验和数据冲突时直接 reject，不回退。** 操作员裁定：「宁可没有结果走
yolo 框回归这条 fallback，也不能接受错误的结果」。两道闸：候选筛选 + 最终 frame
复核（`frame_long_axis`）。

**⑨ 改完源码必须先 `rm -rf __pycache__` 再跑测试。** 同一秒内改回源码，`.pyc` 会被
当成新鲜的，变异测试会静默跑错版本。

**⑩ 不要 import `pupil_apriltags`。** 同进程里它和 PIL 一起加载会在解释器析构时
abort（实测 2026-09-20，退出码 134）。`algorithm.py` 顶部有这条警告。

**⑪ `OMP_NUM_THREADS=1` 是必须的。** 本项目的矩阵都太小，OpenBLAS 开多线程是纯
开销，实测差 1.6 倍（118ms → 73ms）。

**⑫ 数据（`test_data/` / `box_out/`）不在本仓库。** 合成自检（§2.1）不需要它们；
旋转鲁棒性（§2.2）和任何回归比对需要，路径用参数显式给。

# 4. 相关文档

- 上游：`infrastructure/.../carton_box_yolo/`（YOLO 检测，发 `/box/yolo_box`）
- 下游：`orchestration/nodes/node_box_obs.py`（订阅 `/box/detection` 写
  `latest_box_obs`）、`skills/atomic/perception/pallet_servo/`（消费它算伺服误差）
- 开发过程的完整记录（九个坑的来龙去脉、每轮改了什么、试过什么没用）：
  `maduo/WORKLOG_box_frame_fit.md`（**在 LeTools 之外**）

**迁移自 maduo**（2026-09-23）：本模块原来是 `maduo/fit_box_frame.py` +
`box_servo_window.py` + `box_detection_node.py`。搬过来时把三处跨模块 import
（`ground_detector` / `rgbd_detector` / `refine_pallet_frame`）换成内联 helper，
**算法逻辑一行没动** —— 5_test 五帧四角差 `0.000e+00px`，6_test 最大 `6.3e-07px`
（浮点重结合噪声，亚微米量级）。自检 201 条断言逐条相同。
