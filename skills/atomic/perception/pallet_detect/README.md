# 木托盘台面坐标系检测（无初值）

读「彩色图 + 对齐深度图 + 内参 + **台面法向**」，检出木托盘**台面坐标系**
（`origin` / `E1` / `E2` / `nrm` / `W` / `H`，相机系 mm），给下游伺服当初值。

**只做检测，不做控制律、不做抓取。** 本模块也不碰箱子那条链。

链路两步，可以只用第一步，也可以接起来用：

    algorithm.detect_pallet_frame   法向先验 + 颜色/深度掩码 → 已知尺寸的旋转矩形搜索
    refine.refine_pallet_frame      已知尺寸 + 台面外轮廓的可见边 → 把位姿细化到伺服能用的精度

ROS 壳在 `infrastructure/ros_packages/src/ros_vision/pallet_detection/`
（订阅三路 → 发布 `/pallet/detection`）。

- **接口怎么调** → §1
- **怎么快速验一遍** → §2
- **踩过的坑、部署注意事项** → §3（**全部坑点都在这一节**）

---

# 1. 接口

## 输入

### `detect_pallet_frame(color, depth, k, *, normal, target_mm, long_side_parallel, prior=None, opts=None)`

| 参数 | 类型 | 说明 |
|------|------|------|
| `color` | `np.ndarray (H,W,3) uint8` | BGR 彩色图 |
| `depth` | `np.ndarray (H,W) float32` | 对齐后的深度，**单位 mm** |
| `k` | `CameraIntrinsics` | `fx/fy/cx/cy`（本模块自带这个 dataclass，见 §1 末尾） |
| `normal` | `(3,) array-like` | **台面法向先验，相机系**。必需，见 §3 第 1 条 |
| `target_mm` | `(长, 短)` | 托盘实际尺寸（mm），默认 `(1200.0, 1000.0)` |
| `long_side_parallel` | `bool` | 与画面**近平行**的那条边是不是长边。**默认 `False`（短边平行）**。⚠️ **它只翻转 `W`/`H` 两个数，不碰 `E1`/`E2` 的方向**（E1 恒指画面右）—— 换它不用重新找 x 轴，见 §3 第 7 条 |
| `prior` | `dict \| None` | 上一帧的 `frame`。给了就走先验跟踪路径（单帧快 3~7 倍），见 §3 第 4 条 |
| `opts` | `dict \| None` | 见下 |

`opts` 常用的几个键（其余是搜索参数，默认值都够用）：

| 键 | 默认 | 说明 |
|----|------|------|
| `voxel_mm` | `5.0` | 地面系体素边长（mm） |
| `theta_ref_offset_deg` | `0.0` | 给参考角 θ_ref 加一个偏置（度）。测试用 |
| `prior_min_score` | `0.35` | 先验路径低于它就退回全画布搜索 |
| `crop_search` | `True` | 关掉 = 回到"整张栅格"的老路径（等价性测试用） |
| `snap_edges` | `False` | 拿深度脊线精修矩形的四条边。**实测在主路径上是负收益**（8 帧真值 13px -> 14px），默认关，见 `_snap_rect_to_ridge` 的 docstring |

## 输出

### `(frame | None, diag)`

`frame` 是 `dict`，**与 `pallet_from_ground_quad.ground_frame()` 同构**，
可以直接喂给 `refine.refine_pallet_frame`：

| 键 | 说明 |
|----|------|
| `origin` | 台面系原点，相机系 **mm**（`(3,)`）|
| `E1` / `E2` / `nrm` | 三个单位正交轴（`(3,)`）。**`E1 × E2 = nrm`**，是右手系 |
| `W` / `H` | 台面尺寸（mm）。**是搜索用的 `target_mm`，不是量出来的**。哪个是长边由 `long_side_parallel` 定 —— 见 §3 第 7 条 |

`frame is None` 表示**如实拒绝**，此时看 `diag['reject']`。

`diag` 里的关键字段：

| 键 | 说明 |
|----|------|
| `reject` | `None` = 成功。取值见下 |
| `score` | 最优矩形落在掩码里的覆盖率，与 `COVERAGE_MIN`（0.35）比 |
| `source` | `'color'`（木色分割主导）/ `'depth'`（颜色分不开时的退路）|
| `theta_ref_deg` / `theta_alt_deg` | 参考角与它的备选（相差 90°）。⚠️ **主路径下可能是 `None`** —— 那时 θ 由图像空间给，见 `rect_source` |
| `rect_source` | θ 从哪来：`'image_theta'`（图像空间主路径）/ `'prior'` / `'search'`（回退）|
| `rect_why` | 主路径没接管的原因：`image_mask_small` / `dense_thin` / `dense_fat` / `dense_aspect` / `dense_no_rect` |
| `dense_fill` / `dense_measured_mm` / `dense_angle_deg` | 稠密米制掩码的填充率、`minAreaRect` 量出的尺寸（mm）与长边方向 |
| `long_short_diff_deg` / `long_short_mismatch` | **不依赖真值的长短边哨兵**：帧的长边方向与图像空间量出的方向差多少度；`> 45°` 判 `True`。只在无先验那帧算得出 |
| `lsp_measured_parallel` / `lsp_agrees` | **声明 vs 实测的交叉核对**（不影响输出）：量出的长边是不是与画面近平行、与 `long_side_parallel` 是否一致。只在主路径跑了的帧（无先验）有 |
| `n_observed_edges` | 有几条边可观测（`< 2` 就拒）|
| `mask_area_ratio` | 掩码面积 / 托盘标称面积，与 `MASK_AREA_RATIO_MAX`（3.0）比 |
| `deck_h_rel_mm` / `sat_min` / `n_wood` | 台面高度层、饱和度下限、木色格数 |
| `prior_used` / `prior_reason` | 这一帧实际走了先验还是全搜索、为什么 |
| `timing_ms` | 分段耗时（`deck_mask` / `theta_image` / `snap` / `theta_ref` / `search` / `orient` / `edges` / `total`）|

**拒绝码**（`diag['reject']`，全部互不重叠）：

| 码 | 含义 |
|----|------|
| `no_wood` | 木色格 < `MIN_WOOD_PX`（500）|
| `no_deck` | 选不出台面高度层 |
| `mask_too_large` | `mask_area_ratio > 3.0`（掩码糊成一大片，多半取到地面了）|
| `no_theta_ref` | 掩码主轴解不出来 |
| `ambiguous_theta` | `score < COVERAGE_MIN`（0.35）|
| `too_few_edges` | 可观测的**相邻**边不足两条 |
| `degenerate` | 最优矩形贴到栅格边界（搜索窗开小了）。⚠️ **只在 `rect_source == 'search'` 时判** —— 主路径与先验路径跳过，理由见 §3 第 9 条 |

### `refine_pallet_frame(frame, color, depth, k, *, target_mm, opts=None)`

输入是上面那个 `frame`，输出同样是 `(frame | None, diag)`。`diag` 里：

| 键 | 说明 |
|----|------|
| `reject` | `None` = 成功；`no_edges` = 一条边都采不到 |
| `updated` / `held` | 哪些自由度被更新了、哪些**原样保留初值**（`t_e1` / `t_e2` / `theta`）|
| `rms_px` / `n_observations` | 拟合残差、采样点数 |
| `plane` | 平面细化的落点：`{'refined': bool, 'note': str}` |

> ⚠️ **`opts={'plane': True}` 是部署路径的必需项，而它的默认值是 `False`。**
> 理由见 §3 第 5 条。

### `payload` 里的纯函数（`build_payload` / `format_diag` / `format_rejects`）

把上面两步的输出搬成 `pallet_detection_msgs/PalletDetection` 的字段。
**不 import ROS**，所以能在没有 ROS 的环境里单测。ROS 壳只做搬运。

> `normal_from_tf`（从 TF 链查法向）**不在 `skills/` 里** —— 它要 `import rospy`，
> 按分层约束只能待在 `infrastructure/ros_packages/src/ros_vision/pallet_detection/scripts/tf_normal.py`。

## 节点参数

`roslaunch pallet_detection pallet_detection.launch`，常用参数：

| 名字 | 默认 | 说明 |
|---|---|---|
| `image_color` | `/camera/color/image_raw` | 彩色话题 |
| `image_depth` | `/camera/depth/image_raw` | 深度话题（**16UC1，mm**）|
| `camera_info` | `/camera/color/camera_info` | 内参来源（用它的 K 矩阵）|
| `detection_out` | `/pallet/detection` | 输出话题 |
| `target_mm` | `[1200.0, 1000.0]` | 托盘实际尺寸（长, 短）|
| `long_side_parallel` | `false` | 与画面近平行的边是不是长边 |
| `normal` | `''` | `"x,y,z"`；**空串 = 从 TF 查**（部署默认）|
| `camera_frame` / `base_frame` | `camera_color_optical_frame` / `base_link` | TF 链两端 |
| `refine` / `plane` | `true` / `true` | 检出后是否跑 refine、是否开平面细化 |

## 本模块自带 `CameraIntrinsics`

`algorithm.CameraIntrinsics(fx, fy, cx, cy)` —— 与 maduo 的 `rgbd_detector` 那份
**逐字段同名同序**。本模块**不依赖任何外部仓库**，所以自带一份；构造它不需要 ROS，
`CameraIntrinsics(400.0, 400.0, 320.0, 240.0)` 这样直接给就行。

---

# 2. 快速测试

三个自检脚本，**都不需要 ROS、不需要相机、不需要任何数据**（自己合成场景/真值）：

```bash
cd <仓库根>
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1

# 1) 算法：检测本身（46 条断言，几何真值）
python3 skills/atomic/perception/pallet_detect/tests/test_pallet_detect.py

# 2) 算法：细化（12 条断言，几何真值 + 分级更新）
python3 skills/atomic/perception/pallet_detect/tests/test_pallet_refine.py

# 3) 契约：消息字段、单位、拒绝串、TF 链（ROS 缺依赖时那几条自己 SKIP）
python3 infrastructure/ros_packages/src/ros_vision/pallet_detection/tests/test_pallet_detection_node.py
```

退出码 0 = 通过，1 = 失败。也可以 `pytest skills/atomic/perception/pallet_detect/tests/`。

**跑真机/离线数据**：

```bash
# 只跑检测（需要自己提供法向）
roslaunch pallet_detection pallet_detection.launch \
    normal:="" target_mm:="[1200,1000]" long_side_parallel:=false

# 看结果
rostopic echo /pallet/detection
```

⚠️ 消息包 `pallet_detection_msgs` 要先进 catkin 工作区编译（见 §3 第 10 条）。

---

# 3. 注意事项

**1. 法向是必需先验，本模块不会细化它 —— 而且缺了它不会报错。**

`detect` 只搜平面内 3 个自由度（平移 ×2 + 绕法向转角 ×1），**法向就是投影平面本身**。
法向偏 ε 时真值台面在"高度"坐标里变成一条斜坡，跨度 ≈ `tan(ε) × 781mm`；一旦超过
`DECK_BAND_MM`（40mm），台面就装不进那个高度带、掩码塌成月牙。**实测临界角 ≈ 3~4°**。

⚠️ **给一个完全错误的法向它不会崩、不会报错** —— 5_test 上给"竖直向下"它只是换个
score 继续"如实拒绝"（`source` 从 `color` 变 `depth`）。所以**调用方必须自己保证
法向可信**（TF 或操作员给定）。ROS 壳里 TF 缺失时**不发布**，绝不回退到单位阵。

> 部署路径上这条没有想象中致命：下游 `refine` 的 `_refine_plane` 是 SVD 拟合台面平面，
> **法向 2 + 高度 1 它自己解**，实测能把 5.6° 修到终点 13.2mm / dθ 0.1° ——
> **前提是 `plane=True`**（见第 5 条）。

**2. θ 现在由**图像空间主路径**给（2026-09-30 加）；`long_side_parallel` 只管回退路径。**

原先 θ 只有一个来源：`_theta_ref` 取**栅格掩码的协方差主轴**。现场帧
（`2026-09-30-11-37-17.bag`）实测它给出的两个候选轴是 **34.44° / 124.44°**，
而托盘长边是 **83.49°** —— **两个都不是托盘的边**，`long_side_parallel` 只是
在错答案里二选一，发布出来是整体转 90° 的位姿（三可靠角偏 36/39/108px）。
根因是栅格掩码本身：`_rasterise` 按 5mm 体素撒点，托盘范围内只盖到 **6.3%**，
`close(25)` 填出来的形状是 **1568x1565 的方形**（91% 落在托盘 ROI 之外）。

主路径改从**图像空间**取 θ：图像掩码（开运算 25 + 去小块）**稠密且准**
（回投验证 79.6% 落在真值四边形内），经**单应反查**铺成米制掩码（填充率 70.4%），
再取 `minAreaRect` 的**长边方向**。实测同 8 帧真值：中位 **6px**、θ 抖动 0.4°。

⚠️ **只换 θ，不换搜索** —— 位置仍由原来的 `_search_rect` 定（它对遮挡有韧性）。
用 `minAreaRect` 的矩形直接取代搜索反而更差：合成场景"遮 2 条相邻边"（规格 §5.1
要求输出位姿）会退化成 L 形掩码，主轴转 90°、位姿偏 **184mm**。
所以守卫里有一条"形状不像矩形就回退"（`DENSE_ASPECT_TOL = 0.10`：
像矩形的场景偏 1~6%，L 形偏 15.3%）。哪条路生效看 `diag['rect_source']`。

⚠️ **主路径不读它选象限** —— `minAreaRect` 已经量出哪条边长；它在那条路上只做一次交叉核对，写进 `diag['lsp_agrees']`，**不影响输出**。它真正的职责是**定 `W`/`H` 哪个是长边**，见 §3 第 7 条。

⚠️ 在**回退路径**上它还兼着"θ 落哪个象限"（`_theta_ref` 读它），给错了照样偏 90°
（april_test7 #2 实测 winner θ 偏 **+67.6°**、origin 偏 **452mm**，而 `score`
与"正确"那次一样漂亮、**不报错**）。**默认 `False`（短边平行），换场景要确认。**

**3. `_project_axis` 原先写错（2026-09-30 修），它不影响结论但是错的。**

`p0 = o + 0.0 * vec`，与它自己的 docstring（"两端各 500mm"）不符；而调用方传的
`origin` 是 `np.zeros(3)`（相机光心）—— `p0` 正好落在光心、`p1` 因 `vec` 的 z 分量为负
**落到相机后方**，投影整个翻掉（实测 E1 的方向从"画面右"变成 `[-1.0, 0.0006]`）。
两处都修了（`-500.0 * vec` + `frame_ph['origin']` 改用 `info['origin']`）。

⚠️ **修完之后 θ_ref 一点没变** —— `_theta_ref` 对角度取了 `abs()` 又折到 `[0,90)`，
方向的正负号被吃掉了（实测 `angs` 从 [34.4, 55.6] 变成 [29.07, 51.93]，**顺序不变**）。
**但它本来可能翻掉 `i_par`** —— 之所以没翻是运气。修它是因为它错，不是因为它是病根。

**4. `prior` 是可选的外部状态，给错了不报错，只会退化成全搜索。**

给上一帧的 `frame` 就走先验跟踪路径（θ 与位移都只在上一帧位姿附近搜，单帧快 3~7 倍，
实测 april_test7 2626→563ms）。两条**自动回退**：① 先验结果的 `score` 低于
`PRIOR_MIN_SCORE`；② 最优解**顶到搜索窗边界**（`diag['prior_reason'] == 'prior_drift'`）。

⚠️ **`score` 拦不住先验漂移** —— 掩码是一整块台面，矩形落在台面里任何位置 coverage
都高（实测先验偏 400mm 时 `score` 仍有 0.789，而 `COVERAGE_MIN` 才 0.35）。
回退**只能**靠 `window_saturated` 那条。⚠️ 调用方负责保证 `prior` 是**上一帧、同一相机**
的结果；给一个过时或来自别处的位姿**不会报错**。

**5. `refine` 的 `plane=True` 必须显式传，它的默认是 `False`。**

`refine_pallet_frame` 的 `plane` 默认关（理由可测：april_test7 五帧静止场景里，
用深度重拟合台面平面时**有一帧法向动了 3.28°**，把跨帧一致性顶坏）。
但 `plane=False` 时 **5.6° 的法向偏差会让朝向停在 5.7°**，伺服拿去会歪。

**ROS 壳里 `~plane` 的默认值是 `True`，与 refine 的默认值不同 —— 这是刻意的。**

**6. 两条路线（颜色 / 深度）会静默切换，选错层是已知失败模式。**

`_color_coverage >= WOOD_COVERAGE_MIN`（0.40）走**颜色路线**，否则走**深度路线**
（不做任何颜色过滤）。两条路都如实写进 `diag['source']`，**不静默切换** ——
出问题第一件事就是看 `source`。

⚠️ 深度路线的判据（"格数最多的那一带 = 台面"）**在真实数据上不成立**：
5_test 实测它选出的那一带离真值台面 **−744mm**、与真值矩形 **IoU = 0.000**。
这不是 bug 是结构性局限，`tests/test_pallet_detect.py::test_deck_mask_depth_path`
把它**钉在测试里**。深度路线只在"颜色分不开"时当退路用。

**7. `W` / `H` 是搜索用的已知尺寸，不是量出来的；哪个是长边由 `long_side_parallel` 定。**

按已知尺寸搜索的检测器**必然**报出接近 `target_mm` 的值，这个字段有一定程度的
自我实现，**不能当独立测量用**。消息里的 `size_mm` 同理。

⚠️ **`long_side_parallel` 只翻转 `W`/`H` 这两个数，`E1`/`E2` 的方向一步不动**
（2026-09-30 操作员裁决）。E1 恒指画面右、E2 恒指向上，与这个键无关 ——
所以换键**不用再把 `ref_edges` 的 x/y 上限跟着换算**，操作员调伺服边时
永远知道哪边是 x 轴。

这条是**刻意选的**。之前 `E1`/`W` 的对齐是 `_orient` 决定的：它把"投影更向上"
的那条边定成 `E2`，`E1` 只是 `E2 × nrm` 取到指右 —— 于是**横向那条边是什么，
`E1` 就沿着它**，`W`/`H` 跟着翻。当托盘**长边竖直**时（现场 `2026-09-30-11-37-17`
就是这样，长边 84.4°），`E1` 沿短边、`size_mm` 报 `[1000, 1200]`，而
`ref_edges` 的 `x=1200` 隐含 x 沿长边 —— 两边直接矛盾，启动时整表校验判
`ref_edge_out_of_range`，**改哪个数值都救不了**（矛盾在"x 沿哪条边"，不在数值）。

⚠️ **所以 `ref_edges` 的 `x=<毫米>` 上限 = `W` = `long_side_parallel` 决定的那条。**
现场是 `false`（短边横向）→ 上限 1000，与 board.json 里 `x=1200` / `x=0` 对得上
（那两组用的是 `y=1000` 那条边上的两个点，x 只取 0 与 1200… 见下面那条已知限制）。

**8. 先验路径有一个已知的退化边界：`PRIOR_SHIFT_WIN_MM = 150` / `PRIOR_THETA_HALF_DEG = 5°`。**

操作员给的实际帧间运动上限是 **≤150mm / 5°**（10Hz 伺服正常速度）。真值实测
（april_test7 相邻帧 dt=33.4ms）Δorigin 最大 37.1mm、Δθ 最大 2.41° —— 留了约 4x/2x 余量。
真超出去了**不是给出错答案**，而是退回全画布搜索（那一帧慢，但能自愈）。

**9. `degenerate` 只在「全画布搜索」那条路上判（2026-09-30）。**

它原本的意思是"搜出来的最优矩形顶到了栅格边界 -> 搜索窗开小了"。但现场帧的栅格是
**点云 bbox**，而**图像 v>=476 没有有效深度**（托盘近边压在图像下边界上）—— 点云
到不了托盘近边，**栅格比托盘实际范围小，正确的矩形必然伸出去**（投影四角对真值
5px 已验证）。所以这条判据对**任何正确的解**都会开火。实测：不放宽它，8/8 帧全被拒。
现在只在 `rect_source == 'search'` 时判 —— 先验路径有它自己的两道闸门
（`window_saturated` / `prior_score_low`），主路径的 θ 来自图像、不会跑到错象限。

**10. 部署前必须做的两件事。**

- **消息包要编译**：`pallet_detection_msgs` 放进 catkin 工作区 `src/` 下
  `catkin_make` 再 `source devel/setup.bash`。没编译时节点会给出明确报错。
- **`OMP_NUM_THREADS=1`**：本项目的矩阵都太小，OpenBLAS 开多线程是纯开销，
  实测差 1.6x。launch 文件里已经用 `<env>` 设好了。

**11. 合成自检**验得了 detect、**验不了 refine**。

合成场景的台面色 `BGR(93,145,190)` 灰度 152.5、地面灰 `BGR(150,150,150)` 灰度 150.0，
**只差 2.5**；整幅图梯度幅值 max **3.13**，而 refine 的 `GRAD_MIN = 60.0`。
也就是说合成场景**没有亮度边缘**（台面与地面在彩色上分得开、在灰度上分不开），
而 refine 的边搜索是在灰度上做的。所以 `refine` 在这个场景上必然拒 `no_edges` ——
**是渲染器的局限，不是 refine 的错**。refine 的验证要用真实数据。

**12. 内联了两份 `ground_detector` 的 helper，改一处要改两处。**

`algorithm.py` 与 `refine.py` **各自**内联了一份 `_backproject` / `_project_px` /
`_corners_mm` / `_fit_plane`（同一份代码的两份拷贝），这样两个模块能各自单独 import、
互不依赖 —— 与 `box_frame/algorithm.py` 同一做法，内容与 maduo 的
`ground_detector` **逐位一致**。改其中一份时别忘了另一份。

**13. 改完源码先 `rm -rf __pycache__` 再跑测试。**

同一秒内改回源码时，`.pyc` 会被当成新鲜的，测试会**静默跑错版本**。

---

# 4. 相关文档

- 消息契约：`infrastructure/ros_packages/src/ros_vision/pallet_detection_msgs/msg/PalletDetection.msg`
  （**`T_cam_pallet` 的平移是「米」，`size_mm` 是「毫米」**；`corners_uv` 的顺序是
  左下→右下→右上→左上，**与箱子不同**）
- 消费者：`orchestration/nodes/node_pallet_obs.py`（话题 → 黑板）
- 姊妹模块：`skills/atomic/perception/pallet_servo/`（误差怎么算）、
  `skills/atomic/perception/box_frame/`（同一套分层与内联做法）
- 设计与开发记录在 maduo 仓库（`docs/superpowers/specs/2026-09-22-pallet-detect-from-scratch-design.md`、
  `WORKLOG_3test.md`、`.sdd-detect/progress.md`），**不在本仓库**
