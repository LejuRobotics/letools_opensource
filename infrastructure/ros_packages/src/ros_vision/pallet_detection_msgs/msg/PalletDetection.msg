# 木托盘台面位姿的检测结果 —— 视觉检测器（非 AprilTag）的输出。
# 默认话题 /pallet/detection，消费端 orchestration/nodes/node_pallet_obs.py。

# `stamp` 的硬要求见文件末尾那段（**输入图像的采集时刻**）。
# `frame_id` 填 `T_cam_pallet` 的**父系**，即那个相机光学帧
# （Orbbec 头部相机是 `camera_color_optical_frame`）。
# 它是给人看/给 TF 排查用的**标注**，下游不靠它做变换 ——
# `T_cam_pallet` 已经把坐标系说死了。
std_msgs/Header header

# 相机系 ← 托盘系，4x4 刚体变换，**行主序**拍平成 16 个 double。
# 读取：T = np.asarray(msg.T_cam_pallet, np.float64).reshape(4, 4)
#
# ⚠️ **平移是米，不是毫米。** 旋转块无量纲，平移 `T[:3, 3]` 的单位是**米**
# （与框架 `Pose6D` 和 `PalletFrame.to_matrix()` 一致）。本消息里别的长度量
# （`size_mm` / `spread_mm`）都是**毫米**，很容易顺手把平移也填成毫米 ——
# 填了会让 1000mm 被读成 1000m。下游 `project_pallet_points` 对**台面系内的点**
# 乘 0.001 把它换成米，而对 `T` 的平移**不做任何换算**。
#
# 用 float64[16] 而不是 geometry_msgs/Pose：**Pose 是有损的**。
# Pose6D 往返走 Rotation.from_matrix，会把 det=-1 的镜面**静默投影**成
# "最近的旋转" —— 参考边会跑到托盘外面去，而下游一路上没有任何报错。
float64[16] T_cam_pallet

# 上面那个矩阵旋转块的行列式。**诊断量，但值得一个独立字段**：它是手性判据，
# 属于"不报错、只是算错"那一类。正常值 ≈ +1（右手系）；≈ -1 = 镜面反射；
# ≈ 0 = e1 与 e2 近乎共线，坐标系根本定不出来。
float32 det

# 这一帧能不能用。**消费端只需看这一个**（与 BoxDetection 同口径）。
#   true  = T_cam_pallet 自洽、det 在 +1 一侧
#   false = 没检出 / 检出但不可信，T_cam_pallet 与 det 都是**占位值，别用**
bool valid

# 这个位姿是怎么来的，自由字符串。建议取值：
#   'color' —— 木色分割主导
#   'depth'  —— 深度主导（颜色分不开时的退路）
#   'refine' —— 有初值，走细化
#   'manual' —— 人工给的（调试用）
string source

# 检测器**实测**的台面尺寸 [W, H]，毫米。
# ⚠️ 这是**诊断量，伺服不用它** —— 伺服的投影基准是配置里的 pallet_size_mm。
# 两者差得多说明检测器内部"把量出来的长度配成已知尺寸"那一步出了问题。
# ⚠️ 也要知道它的局限：按已知尺寸搜索的检测器**必然**报出接近配置的值，
# 这个字段有一定程度的自我实现，不能当独立测量用。
float32[2] size_mm

# 台面四角，像素。顺序固定：**左下 → 右下 → 右上 → 左上**，
# 即从 T_cam_pallet 的 origin 起、沿 e1 绕一圈（(0,0)→(W,0)→(W,H)→(0,H)）。
# z 恒为 0。
#
# ⚠️ **与 BoxDetection.corners_uv 的顺序不一样**（那边是 右下→左下→左上→右上）。
# 不是笔误：托盘这边**有手性要求**（[e1|e2|normal] 要构成旋转矩阵，e1×e2=normal），
# 环序必须跟着 e1/e2 走；箱子那 4 个点只是一份"边的清单"，没有这个约束。
# 为了表面一致去对齐箱子，会让第 0 个角不再是 origin，埋一个看不见的错位。
#
# ⚠️ 本字段是 **T_cam_pallet 投影出来的四角**，不是独立测量 —— 它是给不想自己
# 投影的冗余字段。契约上应当等于"把台面矩形的四个毫米角点
# (0,0)/(W,0)/(W,H)/(0,H) 用内参 K 投到像素"的结果 ——
# 下游 `skills/atomic/perception/pallet_servo/render.py` 的
# `_project_pallet_quad()` 干的正是这件事，可以直接对照。
geometry_msgs/Point32[4] corners_uv

# 参与平均的成功帧数 / 窗口总槽数 / 失败槽数。
# 不做时间窗的检测器填 n_used=1, n_slots=1, n_failed=0（窗在 LeTools 侧做）。
uint8 n_used
uint8 n_slots
uint8 n_failed

# 窗内各帧到平均位姿的最远距离。位置用毫米、朝向用度。
# 只有 1 帧时都是 0.0；不适用时是 -1.0。
float32 spread_mm
float32 spread_deg

# 本次处理耗时（毫秒）
float32 latency_ms

# 自由格式的检测器诊断，逗号分隔的 key=value。**格式不进契约**，给人看。
# 建议带上选层判据那几个（这是已知的失败模式，没它们排查不了）：
#   score=0.83,coverage=0.71,n_edges=3,deck_h_mm=509
string diag

# 窗口里失败槽的拒绝原因，逗号分隔；空串 = 没有失败。
# 取值由检测器定，但**必须写实际值与阈值**，不能只写"失败"。
string rejects

# ⚠️ 关于 header.stamp 的硬要求：
# **header.stamp 必须是"检测器用的那张输入图像的采集时刻"，不是处理完成时刻。**
# 托盘与箱子的检测耗时可能差一个量级（箱子实测 ~60ms），而伺服要把两者**相减**。
# 按"算完的时刻"配对，耗时的差会被当成场景的时间差 —— 要么永远配不上，要么配上一对
# 其实是不同帧的图像，相减出来的误差里混着相机运动，**而三个数照样算得出来**。
# 填错的表现：伺服频繁报 pair_dt 超阈值。
