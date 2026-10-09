# 箱子顶面四角的检测结果 —— `fit_box_frame`（纯函数）经 `BoxFrameWindow`
# （滑动时间窗）之后的输出。默认话题 `/box/detection`。
#
# 消费端只需要关心两件事：
#   * `valid == true`   → 拿 `corners_uv` / `angle_deg` 去做伺服；
#   * `valid == false`  → 四角就是**原始 YOLO 框**（角度没恢复出来），
#                          `source` 与 `rejects` 说明为什么。

std_msgs/Header header

# 四角像素坐标，顺序固定：**右下 → 左下 → 左上 → 右上**
# （与 fit_box_frame.order_corners_uv 一致）。z 恒为 0。
geometry_msgs/Point32[4] corners_uv

# 四角是否可用于伺服
#   true  = source 是 'window' 或 'single'
#   false = source 是 'yolo_fallback'（窗口全失败，退回了输入框）
bool valid

# 'window'        —— 窗里 >=2 帧成功，四角是它们的**逐点平均**
# 'single'        —— 窗里只有 1 帧成功，四角就是那一帧的
# 'yolo_fallback' —— 窗里全失败，四角 = 原始 YOLO 框，valid=false
string source

# 参与平均的成功帧数 / 窗口总槽数 / 失败槽数
uint8 n_used
uint8 n_slots
uint8 n_failed

# 窗内各帧四角到平均值的最远距离（像素）—— 判断这一窗稳不稳。
# 只有 1 帧时是 0.0；兜底时是 -1.0（无意义）。
float32 spread_px

# **下边**（左下 -> 右下）的方向角，单位度。
# 图像右手系：+x 向右、+y 向下；范围 (-90, 90]。正 = 右边比左边低。
# 这就是 YOLO 丢掉、伺服要的那个旋转角。
float32 angle_deg

# 输入框原样透传：[0] = 左上 (u0,v0)，[1] = 右下 (u1,v1)
geometry_msgs/Point32[2] box_uv

# 本次处理耗时（毫秒）；兜底那一次也有值（窗口逻辑本身不耗时）
float32 latency_ms

# 这一窗朝向先验的**处置结果**（口径与 `plane_hint_used` 一致：上游输入只要
# 参与了结果就必须留痕）：
#   'horizontal' / 'vertical' —— 用了上游 /infer_carton_pose 的
#                                tape_orientation_deg（0 / 90），并确实筛掉了候选；
#   ''                        —— 没用上：上游给的是 -1=unknown，或**没接这个服务**。
#                                此时四角是按边支持率自动选的。
#   'conflict'                —— 上游给了朝向，但没有任何候选的长边方向对得上，
#                                已退回自动选。**这是「先验可能错了」的信号**，
#                                消费端应当告警；与 '' 必须区分开。
string orientation

# 窗口里失败槽的 reject 原因，逗号分隔；空串 = 没有失败
string rejects
