# 木托盘台面坐标系检测（YOLO-POSE）

读「彩色图 + 对齐深度图 + 内参 + **台面法向**」，检出木托盘**台面坐标系**
（`origin` / `E1` / `E2` / `nrm` / `W` / `H`，相机系 mm），给下游伺服当初值。

**只做检测，不做控制律、不做抓取。** 本模块也不碰箱子那条链。

链路一步：

    detect_pose.detect_pallet_frame_pose  YOLO-pose 12 关键点 → 四边 → solve_frame

ROS 壳在 `infrastructure/ros_packages/src/ros_vision/pallet_detection/`
（订阅三路 → 推理会话启动时建一次 → 发布 `/pallet/detection`）。

- **接口怎么调** → §1
- **怎么快速验一遍** → §2
- **踩过的坑、部署注意事项** → §3（**全部坑点都在这一节**）

---

## 前端：POSE（12 关键点）

`detect_pose.detect_pallet_frame_pose`：YOLO-pose 12 关键点 → 每条边 2~4 点加权
TLS 拟合四条直线 → `solve_edges.solve_frame` 解位姿。

`opts['backend']` 选推理后端：`auto`（默认，engine 文件在就用它）/
`engine`（强制）/ `onnx`（强制）。engine 是目标机上预导出的静态文件，启动不编译、
首帧没有 ONNX CUDA EP 的 ~19s 选核；换机 / 换 TensorRT 版本要重导。前后端
**前处理与后处理是同一段代码**（`trt_backend.TensorRTSession` 只替换推理那一步），
所以切后端不改数值口径。导出见 `models/README.md`。

`opts` 额外认 `kconf`（参与边拟合的最低关键点置信度，默认 `0.5`）/
`d_lock`（默认 `'depth'`）/
`fixed_d_mm`（给了数值就**完全不量深度**，call 0 记录之后用）。模型缺失**硬失败**。

---

# 1. 接口

## 输入

### `detect_pallet_frame_pose(color, depth, k, *, normal, target_mm, long_side_parallel, opts=None)`

| 参数 | 类型 | 说明 |
|------|------|------|
| `color` | `np.ndarray (H,W,3) uint8` | BGR 彩色图 |
| `depth` | `np.ndarray (H,W) float32` | 对齐后的深度，**单位 mm**；没有 call 0 记录时用于锁 D |
| `k` | `CameraIntrinsics` | `fx/fy/cx/cy`（本模块自带这个 dataclass，见 §1 末尾） |
| `normal` | `(3,) array-like` | **台面法向先验**，相机系（`z<0` 也行，内部翻到 `z>0`）。必需，见 §3 第 1 条 |
| `target_mm` | `(长, 短)` | 托盘实际尺寸（mm），默认 `(1200.0, 1000.0)`；喂给 `solve_frame` 当已知边长的硬输入 |
| `long_side_parallel` | `bool` | **与画面近平行的托盘边是不是长边**。必需的先验，**定死面内转角种子 θ0**。关键点四边形的宽高比 ≈ 1，"哪条边是长边"在框上不可分，靠残差试两个候选等于让数据投票 —— 给错**明确偏 90°**。现场是短边近平行 → `False` |
| `opts` | `dict \| None` | 见下 |

返回值：**`(frame | None, diag)`**。`frame` 是台面坐标系（相机系 mm），`None` = 本帧没检出
（`diag['reject']` 有值）。`diag` 带 `model` / `conf` / `n_candidates` / `reproj_px` /
`inplane_deg` / `depth_mm` 等落点信息。

`opts` 的键：

| 键 | 默认 | 说明 |
|----|------|------|
| `session` | — | 一个已建好的会话（`onnxruntime.InferenceSession` 或 `trt_backend.TensorRTSession`，**优先级最高**，测试/复用会话时用） |
| `backend` | `auto` | `auto`（engine 文件在就用）/ `engine`（强制）/ `onnx`（强制） |
| `engine_path` | 包内 `models/pallet_pose_fp32.engine` | TensorRT engine 路径 |
| `model_path` | 包内 `models/pallet_pose.onnx` | `.onnx` 路径。与 `session` 都没给且默认路径不存在 → **抛异常** |
| `imgsz` | `640` | 输入边长，**必须与导出时一致** |
| `conf` | `0.25` | 置信度阈值（只用于过滤与 NMS，**不参与选框打分**） |
| `nms_iou` | `0.45` | 候选框去重阈值 |
| `pick` | `bottom_center` | 一帧多个候选时挑哪个：`bottom_center`（离画面下方中心最近）/ `conf` / `area` |
| `kconf` | `0.5` | 参与边拟合的最低关键点置信度 |
| `d_lock` | `'depth'` | D 的锁定口径：`'depth'` / `None` / 数值 |
| `fixed_d_mm` | — | **给了正数就跳过深度测量**，直接用这个 D（call 0 记录路径） |

### `fit_floor_normal(depth, k, prior, *, n_sub=1500, iters=120, tol_mm=15.0, win_deg=25.0, min_inliers=400, seed=0)`

法向**第二级**：从当前帧深度拟合"占画面最大的那个平面"的法向。返回
**`(normal, n_inliers)`**；失败返回 **`(None, 0)`**（调用方**刻意回退 `prior`**，不拒帧）。
`prior` **必需**（夹角窗 + 定符号）。见 §3 第 2 条。

### 算法分层（其余都是内部件，一般不用直接调）

    geom.py            CameraIntrinsics / _project_px / _corners_mm（frame 约定在这里）
    preprocess.py      letterbox / BGR→CHW 张量
    pose_decode.py     12 关键点解码 / NMS / 选框 / 四边拟合 / 边求交
    solve_edges.py     solve_frame（**锁法向 + 四条边所在直线 → 4 自由度初值**）
    frame_geometry.py  共享几何：基、边可用性、深度主峰、规格归一（不解位姿）
    normals.py         fit_floor_normal / _backproject_cloud（法向第二级）
    dsearch.py         平面距离 D 的反解与搜索范围工具
    payload.py         frame → PalletDetection 字段（纯搬运，`build_payload` / `format_*`）

`frame` 的约定（**全项目统一**）：`origin` = 台面矩形 `(0,0)` 角（相机系 mm）；`E1` 在台面内、
投影指向画面右（+u）；`E2` 投影指向画面里更上方（-v）；`nrm = E1 × E2`；四角顺序恒为
`(0,0) → (W,0) → (W,H) → (0,H)`（左下 → 右下 → 右上 → 左上）。

## 输出

`frame` 是 `dict`，键：`origin`(3,) / `E1`(3,) / `E2`(3,) / `nrm`(3,)（都是相机系**毫米**）+
`W` / `H`（mm）。`detect_pallet_frame_pose` 还会塞一个 `_quad_px`（下划线键，
只给诊断出图用，不是契约）。

`payload.build_payload(frame, diag, k, ...)` 把它转成 `PalletDetection` 的字段
（`T_cam_pallet` 行主序 **16 个数、平移单位是米**、`corners_uv` 左下起、`det` 手性判据）。
逐键对应消息字段名，契约见 `pallet_detection_v2_msgs/msg/PalletDetection.msg`。

## 运行前提（不满足会直接报错）

* **托盘检测必须有 POSE 模型**：`opts['session']` → `opts['model_path']` → 包内
  `models/pallet_pose.onnx`，三者都拿不到**立即抛**（`FileNotFoundError` / `RuntimeError`），
  错误信息写清期望路径、实际查找的路径、怎么解决（见 §3 第 1 条）。
* **`normal` 先验必需**：`solve_frame` 把法向锁死，缺了没法解（§3 第 1 / 2 条）。
* 深度单位是**毫米**（`float32`，无效像素 0）；`k` 是**数据集坐标系**的等效内参。
* `OMP_NUM_THREADS=1`（本项目矩阵都太小，多线程是纯开销）。

---

# 2. 快速测试

**不需要 ROS、相机、数据、真模型**：

```bash
cd <仓库根>
OMP_NUM_THREADS=1 python3 -m pytest skills/atomic/perception/pallet_detect/tests -q
```

验：解码/四边拟合/求交、D 反解与闸门、法向合理性、Pose 边线尺寸反算，以及
**模型硬失败**（缺模型抛且报错写清路径与办法）。

**节点契约自检**（`T_cam_pallet` 单位/行主序、`det` 自算、`rejects` 带阈值、
`normal_from_tf` 缺段返回 None、launch 写法、静态 AST 检查）：

```bash
OMP_NUM_THREADS=1 python3 infrastructure/ros_packages/src/ros_vision/pallet_detection/tests/test_pallet_detection_node.py
```

**换模型后**核对输出格式（第 6 列必须是**原始弧度**）：

```bash
cd skills/atomic/perception/pallet_detect/scripts
OMP_NUM_THREADS=1 python3 export_onnx_pose.py --weights best.pt --verify
```

---

# 3. 注意事项（踩过的坑）

**1. 托盘检测必须有 POSE 模型，缺失就抛 —— 不要加静默兜底。** 节点启动时建 session，
拿不到就 `SystemExit`。理由是 `None` 在下游是**合法值**（"本帧没检出"，节点据此不发布、
心跳照常）：模型缺失混进去后症状是"一直检不到"，与"现场真没有托盘"在日志上**完全无法区分**。
**解决**：`detect_pose._resolve_pose_session` 把期望路径、实际查找的路径、导出命令写进异常；
`models/` 只进 `.gitignore` + `README.md`（`*` + 白名单），`.onnx` 不进 git。

**2. 法向是两级：TF 先验 + 深度自检，第二级别跳。** TF 链**通了**，但 `base_link` 的 z
转进相机系后与托盘实际台面法向**有 0~10° 的静态偏差**（用户确认的现场事实）。而
`solve_frame` 把法向**锁死**成先验（自由度 6→4），偏了**没有任何下游步骤会纠正它** ——
那 0~10° 会直接进位姿。`fit_floor_normal` 在先验附近（夹角窗 ±25°）拟合"画面最大的面"来纠正。
**静默错在哪**：不纠正不会报错，只是位姿整体歪着；实测 `test_jia4` 37 帧里 TF 与地面差
12.52°、静态不变（不是抖动），旧路线 `too_few_edges` **37/37 全拒**，换成拟合值 37/37 全过。
**解决**：`~fit_normal`（默认开）做第二级，`~normal_refit` 控制"首帧拟合一次后复用"
（伺服只做平面内平移时法向是常量；**动腰 / 换工位要设 true**，否则拿过期法向静默继续算）。
缓存标签 `fit` / `cache` / `tf` / `tf_stale` / `tf_last` / `tf_bad_fit` / `param`
每次都进日志与 `diag`（`tf_bad_fit` = 拟合没过合理性自检、退回 TF 初值且**没缓存**；
`tf_stale` = 按图像 stamp 拼不齐、整条链退回最新值；`tf_last` = TF 彻底查不到、
用上一次成功的 TF 先验兜底）。

⚠️ **有 call 0 记录时这一节整套不跑**：记录里的 base_link 法向与 D 只换算一次到相机系
（来源标签 `record`），之后每帧直接用 —— 即 `fit_normal` / `normal_refit` / D 闸门
都不生效。这是用户明确要求的：call 0 之后 D 与法向是常量，每帧重算只会把噪声带进位姿。
记录不存在（第一次跑）时才走上面的两级。

**2.1 首帧拟合必须先过合理性自检，不合格不许缓存**（2026-10-22 现场事故）。
TF 初值本身可能歪（那次几乎指向 `-Y`），拟合"与初值只差 0.05°、内点 487"看着很自信，
却是错的；旧代码把它缓存后，D 恒为 **241mm**（ref=990），D 闸门把**每一帧**都拒掉
（`pub=0`，现象与"检测不到托盘"一样），只能重启进程。现在 `fit_floor_normal_checked`
检查两道互相独立的闸：**与相机光轴夹角**（`~normal_max_tilt_deg`，默认 60°）和
**拟合平面距离**与 `d_mm` 的相对差（`~normal_d_check_tol`，默认 35% —— 比发布闸门宽，
因为拟合到的常是地面，地面与台面差 ~15%）。不合格时用**相机光轴**当初值重试一次
（托盘台面必在俯视相机光轴附近，这个先验与 TF 无关）；两次都不合格就**不缓存**，
下一帧继续重试。另有 `~normal_refit_on_d_reject`（默认 true）：D 闸门拒绝时丢弃缓存
法向，形成自愈路径。

**2.2 过期 TF 要宽容，但不能逐段混时刻**（2026-11-03）。TF 的 TCPROS 流坏掉后
订阅线程会死，旧 `Buffer` 不再更新；这时按图像 `stamp` 查 13 段链会失败。处理分三层：

1. **整条链退回最新值**：`normal_from_tf` 先按 `stamp` 拼完整 13 段，任一段缺就
   **整条**按 `Time(0)` 重拼，来源记 `tf_stale` 并打节流 WARN。**绝不能逐段各自回退**
   —— 不同时刻的关节/头部位姿拼进同一个 `T_base_cam`，在机器人运动时是一个静默的
   几何错。
2. **已缓存的拟合法向优先**：`normal_refit=false` 且已有通过合理性自检的
   `_normal_fit` 时，直接返回缓存，**根本不去查 TF**。伺服过程中法向是常量，
   TF 停更不该让整条链一帧都出不来。
3. **上一次 TF 先验兜底**：没有缓存时如果 TF 彻底查不到，用最后一次成功解析的
   `_last_normal_prior` 当初值（来源 `tf_last`），后面的深度拟合与合理性自检一步不少；
   连这个都没有才返回 `None`、不发布。

⚠️ 这三层只放宽"用哪个初值"，**没有放宽任何几何自检**。如果 `normal_refit=true`
（动腰 / 换工位），缓存不参与，但仍会用上一次 TF 先验兜底并重新拟合。

**3. `fit_floor_normal` 拟合的是"画面最大的面"，不一定是台面 —— 这是刻意的。** 现场那帧
最大的是**地面**（9.5 万点 vs 台面 2.2 万点）。按颜色挑台面会重新引入"浅色通路把地面算成
木色"的老问题。地面与台面实测只差 1.14°，而检测的临界角是 5~7°，用最大的面 = 最稳的面。
`prior` 的职责只是**"别锁到墙上去"**（蓝墙占 36.9%、法向差 80°+，靠 ±25° 夹角窗排除），
不是给出答案。

**4. `fit_floor_normal` 固定随机种子（`seed=0`）。** RANSAC 有随机性，不固定种子会让
"这次改动了没有"变成一个统计问题、回归测试钉不住。换实现要保同一帧给同一结果。

**5. 深度只在**退化帧**用；D 一律锁到深度，选峰用"占比 + 先验"。**
框内深度沿法向是**多峰**的（台面 / 地面 / 近处木档叠在一起），全局中位会落在
两峰之间、迭代收紧收敛到**错的那一层**（实测 p90 偏差 39.78% vs 主峰的 4.56%）。

两条 2026-10-19 的改动（真机 881 帧实测）：

* **`D_LOCK_INLIER = 0.0`**：只要量到主峰就锁 D。`peak < 阈值` 时 D 会变成自由变量，
  而深度方向近乎不可观测 —— "该锁没锁"的帧只占 2.3%，却贡献了 **89% 的大跳**
  （max 990.9 → 379.7mm，角点距不变）。**POSE 前端不用这个常量**，它有自己的
  `detect_pose.POSE_D_LOCK_INLIER = 0.30`（阈值降到 0 会让 POSE 角点距退化 26%）。
* **`PEAK_MIN = 0.20` + `D_REF_MM = 890`**：不再取 `argmax`，而是**在占比达标的峰里
  挑最接近 890mm 的那个**。托盘大面积出画时近峰只要以 0.006~0.034 的占比优势胜出
  就会把 D 锁到 657（低 26%）→ 跳 300mm；改完 `>100mm` 从 20 降到 13。
  只有一个峰达标时与旧的 `argmax` **完全等价**。

⚠️ 判据是**峰占比**不是旧的"内点率"（旧版 0.45 与它不可比，沿用会误挡大量帧）。
唯一真相源仍是 `frame_geometry` 的这三个常量。

**6. `_to_spec` 的"先选角、再反推符号"不要改回"先翻符号再重挑 origin"。** 那条路上
矩形会被整体搬走（实测 `11-44-12_00240` 框整个消失）。函数末尾有**不变量断言**守这条
（`_checked=False` 时对等价 frame 校验同一组四角，601 帧偏差 ≤ 7e-13px）——**别关它**。
⚠️ 断言里认 `W`/`H` 必须用**三维长度**（投影会被透视压缩，实测拿投影长会认反边）。

**7. 法向必须翻到 `z>0` 才能当"相机→场景"的参数化。** `nrm` 是台面朝上的外法向（z<0），
不翻会让平面落到相机**背后**：背后的点与前面的点共射线、**投影数值完全一致**，
残差照样小、自检全过 —— 但四角是**镜像**的。`frame_geometry` / `solve_edges` 里
各处都有这一翻，**别删**。

**8. 出画（贴图像边）的边不可信：摘除 + 必要时锁 D。** 关键点四边形的某条边大部分在画面外时，
它的远端角是模型从图外**外推**的（`11-44-12_00240` 右边可见占比 3%）。两个措施：
① 可见占比 < `EDGE_MIN_VIS` 的边从位置约束里**摘掉**；② 剩下的边定不住深度时
（不足 3 条、且没有一对对边），把平面距离 D **锁死**在深度测量值上。
**静默错在哪**：不锁 D 时框会沿"把托盘缩进画面"的方向滑走 —— 边贴得严丝合缝、尺寸先验
也满足，图像里**没有任何证据**能拉回来（`00240` 框缩到 x=635 vs 真值 639）。
判据的单一真相源在 `frame_geometry.usable_edges`（`_vis_frac` 是量测原语）。

**9. 面内转角由 `long_side_parallel` 定种子，不放开 ±180°、也不"两个都试用残差挑"。**
关键点四边形的宽高比 ≈ 1，"哪条边是长边"在框上**不可分**；放开就是 50/50 抽签（实测某包 64 帧
错 90°），而"试两个、用残差挑"等于**把这件事交给数据投票** —— 同一场景不同帧可能投出
不同结果（错配残差只差 12 倍，不是数量级）。`long_side_parallel` 是**用户给的先验、
优先级高于任何数据推断**，它直接给出 θ0；搜索只在 θ0 ± `INPLANE_LIMIT_DEG(25°)` 内做。
⚠️ 给错 = **明确偏 90°**（而不是被"两个都试"兜住）—— 错误应当可见。

**10. `T_cam_pallet` 的平移单位是**米**，不是毫米。** `detect` 的 `frame['origin']` 是**毫米**，
`build_payload` 里**必须**乘 `MM_TO_M = 0.001`。填错**不报错**：1000mm 被下游读成 1000m，
而 `det` 照样 +1、`valid` 照样 true。（`corners_uv` 是**像素**，与这个无关。）

**11. `corners_uv` 的顺序是「左下→右下→右上→左上」，与 `BoxDetection.corners_uv` 不同。**
不是笔误：托盘有手性要求（`e1×e2=normal`），环序必须跟着 e1/e2 走。

**12. 节点脚本里的 import 漏了只有真机上才炸。** `__init__` / `_run` 只在真机跑、单元测试
不构造这个类 —— 所以有一条 **AST 静态检查**钉住"没有解析不到的名字"。改了节点记得跑
`test_pallet_detection_node.py`。

**13. 这个模块是**纯 CPU**的。** 现场（Jetson AGX Orin）实测 `detect` 全链 ~70ms
（其中 **ONNX 推理只占 8.65ms**，其余是解码 / 选框 / `solve_frame` 的小循环）。
**别指望 GPU**：能上 GPU 的只有推理那 8.65ms。**别指望多线程**：单帧是严格串行的
依赖链，Python 有 GIL。**向量化才是正解。**

⚠️ **`TensorrtExecutionProvider` 试过又撤了**（2026-10-15）：稳态是快
（Orin 8.76ms vs CUDA 14.56ms），但**建 session 要 3 分钟以上**（现场实测：
节点起来 65 秒、日志刷 6500 行 TRT 警告、**一帧都没出**），而 ONNX Runtime 的
TRT EP **默认不把引擎落盘**，每起一次节点都要重编。为 5.8ms/帧搭三分钟启动，
不划算。**要换 TRT 必须先开引擎缓存**（`trt_engine_cache_enable` + 可写的
`trt_engine_cache_path`），只加 provider 就是上面那个下场；
`test_pallet_detection_node.py` 里有条用例钉着"两处 provider 列表同序且不含 TRT"。

⚠️ **CUDA 自己的首次推理要 ~19s**（cuDNN 选核），现场首帧日志那条
`本帧 19041ms` 就是它，**不是 bug**；稳态 14.56ms。

⚠️ **心跳带 `detect` 分项**（`最近一帧 70ms（detect 70ms）`）。2026-10-15 之前心跳
停在 detect 返回处，于是 `本帧 51ms` 和实测 1.8Hz **同时成立**，现场排查时被这两个数绕了很久。

⚠️ **心跳每行都带 `法向=… 倾角=…`**（2026-10-22 事故后加）：倾角是法向与相机光轴的
夹角（正常俯视台面 ~15°），超过 `~normal_max_tilt_deg` 标 `!!` 并把法向向量打出来；
`D=… 偏…%(ref=…)` 直接给相对偏差。`pub=0` 且拒绝全是 `d闸` 时另打一条一次性提示 ——
那说明**检测已经成功、是 D 与参考值不符**，先查法向/参考 D，不要先怀疑模型。

⚠️ **自愈（2026-11-02）**：TF 的 TCPROS 流坏掉后订阅线程会死，旧 Buffer 不再更新。
心跳里的 `tf=` 是最后一次成功查 TF 距今多久（空闲时也会主动查一次），持续增长会触发
**每 ≥5 s 重建 Buffer + TransformListener**；工作线程的未捕获异常会被 supervisor 打印
完整 traceback 并在 1 s 后重启，心跳里的 `线程重启=` 不为 0 就是代码异常的信号。

---

# 3b. 哪些边能当量测（`usable_edges`）

`solve_frame` 用四边形的**四条边所在的直线**当量测。麻烦是：**不是每条边都能当量测**。
托盘出画时四边形某条边是从画面外**外推**出来的（图像里没有对应像素），拿它当约束会把
解带歪 —— 现场 `2026-10-10-11-44-12` 整包因此从 7.60px 崩到 70.20px。

判据**只在 `frame_geometry.usable_edges` 定义一次**，`solve_frame` 的 `use_pos` 与
出画判定**走同一个入口**。三道闸门 + 秩保底：

1. `_vis_frac >= EDGE_MIN_VIS(0.5)`；
2. 自己**几乎不在画面里**：`_safe_frac(8px) < 0.15`；
3. 自己**贴边**（`_safe_frac(12px) < 0.30`）**且邻边也出画**；
4. **秩保底**（`_rank_ok`）：摘完必须同时覆盖 `{0,2}` 与 `{1,3}`
   （边序 `0,1,2,3` = 下/右/上/左），不够就按 `_safe_frac(12px)` 从大到小加回来。

`_safe_frac` = 把画面**向内缩 eps 像素**后再量可见占比 = "离边界有安全距离的那一截
有多长"。贴着边界的那一截是画面裁出来的，不是托盘的边。

⚠️ **秩保底是必须的**。三道闸门各自只看单条边，合起来可能把某一侧摘光；只剩 1 条边或
只剩两条**对边**时，面内平移有**一维完全不受约束**，解沿那个方向飘走 —— 现场 bag
`f0224` 实测 69.7px → **620.8px**，加保底后最坏 36.2px。

⚠️ **别把这些量测拿去问"这条线像不像一条边"。** 三条路都试过、都失败（脊线有无/偏移、
四边形自身几何自洽、五候选+图像支持度选择），共同死因：**真边与假边的差不是"有没有脊线"，
而是"这条线是不是托盘的那条边"** —— 那是**归属问题**。托盘上摆着纸箱，箱侧边/地缝/木纹
都是又直又强的真脊线。已落地这套能赢，恰恰因为它问的是"**这条线在不在画面里**"，
有确定答案、不需要归属判断。

---

# 4. 相关文档

- 源方案与实测依据（4 自由度怎么来的、边比角准多少、D 的范围怎么定）：`frame_geometry.py` /
  `dsearch.py` 文件头 docstring。
- 上层（ROS 节点与消息契约）：`infrastructure/ros_packages/src/ros_vision/pallet_detection/`、
  `pallet_detection_v2_msgs/msg/PalletDetection.msg`。
