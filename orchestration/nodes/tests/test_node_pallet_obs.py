# -*- coding: utf-8 -*-
"""NodePalletObs 单元测试。

不需要 ROS、不需要相机、不需要 `pallet_detection_msgs`：解析那层是**纯函数**，
只按字段名取，所以这里喂的就是普通的假对象。这也是当初把它写成纯函数的理由 ——
CI 的 `verify:opensource` 只跑 `pytest orchestration/nodes/tests/ -m unit`。

最要紧的三条用例：

  * `test_valid_false_is_not_written` —— 占位值不是观测。
  * `test_a_mirrored_matrix_is_not_written` —— `det = -1` 是**镜面**，
    参考边会跑到托盘外面去，而三个数照样算得出来。
  * `test_a_message_without_a_det_field_is_still_checked` —— 没有 `det` 字段时
    **本节点自己算**，不能因为"上游没报"就放行。

运行：
    pytest orchestration/nodes/tests/test_node_pallet_obs.py -m unit -v
"""
import numpy as np
import pytest
import py_trees
from py_trees.common import Status

from orchestration.nodes.node_pallet_obs import (
    NodePalletObs,
    _reject_bucket,
    parse_pallet_message,
)

pytestmark = pytest.mark.unit

# ★ 真实生产者的 `format_rejects()` —— **只读 import**。
# 并发会话在建的 `skills/atomic/perception/pallet_detect/` 是**未跟踪目录**，
# 这里只 import 它的纯函数（不碰 ROS、不改它、不 `git add` 它）。
# import 不到就退回 `None`，用**逐字照抄**的同款格式造夹具 —— 两种都行，
# 但**绝不能自己编一个"看着像"的 `rejects`**：上一轮漏掉这个病，正是因为夹具用的是
# `rejects=""`（假消息），结构上根本看不到「生产者把 `source=` 拼回串里」。
try:
    from skills.atomic.perception.pallet_detect.payload import (
        format_rejects as _producer_format_rejects,
    )
except Exception:                                    # noqa: BLE001
    _producer_format_rejects = None


def producer_rejects(reject="ambiguous_theta", score=0.3, source="color", **extra):
    """真实生产者产出的 `rejects` 串（形状逐字来自 `payload.format_rejects()`）。

    例：`reject=ambiguous_theta,score=0.3000<threshold=0.35,source=color`
    —— 注意两件事：① 按契约它带**实际值与阈值**，所以**逐帧在变**；
    ② 它把 `source=` **又拼了回去**（第三轮把 `source` 从告警文案里删掉，
    代价却白付了 —— 它随 `rejects=` 回到了同一句里）。
    """
    diag = {"reject": reject, "score": score, "source": source}
    diag.update(extra)
    if _producer_format_rejects is not None:
        return _producer_format_rejects(diag)
    # 兜底：**逐字照抄 `format_rejects()` 的整条 if/elif 链**（import 不到时才走到）。
    # ⚠️ 不能只复刻 `ambiguous_theta` 一段：其它 code 会退化成
    # `reject=no_wood,source=color`，而真串是
    # `reject=no_wood,n_wood=120<500,score=0.4000,source=color` ——
    # 那样本文件的绿就取决于未跟踪目录在不在场，CI 的干净 checkout 上会红。
    # 阈值常量逐字来自 `pallet_detect/algorithm.py`：
    # COVERAGE_MIN=0.35 / MASK_AREA_RATIO_MAX=3.0 / MIN_WOOD_PX=500
    # （`{:g}` 渲染 3.0 → `3`、0.35 → `0.35`；`<500` 是 `{MIN_WOOD_PX}` 的 str）。
    code = reject
    if not code:
        return ""
    bits = [f"reject={code}"]
    if code == "ambiguous_theta" and diag.get("score") is not None:
        bits.append(f"score={float(diag['score']):.4f}<threshold=0.35")
    elif code == "mask_too_large" and diag.get("mask_area_ratio") is not None:
        bits.append(f"mask_area_ratio={float(diag['mask_area_ratio']):.4f}"
                    f">threshold=3")
    elif code == "too_few_edges":
        bits.append(f"n_observed_edges={int(diag.get('n_observed_edges') or 0)}<2")
    elif code in ("no_wood", "no_deck") and diag.get("n_wood") is not None:
        bits.append(f"n_wood={int(diag['n_wood'])}<500")
    elif code == "no_deck":
        bits.append(f"deck_h_rel_mm={diag.get('deck_h_rel_mm')}（选不出台面高度层）")
    elif code == "no_theta_ref":
        bits.append("theta_ref 解不出来（掩码主轴缺失）")
    elif code == "degenerate":
        bits.append("最优矩形贴到栅格边界，搜索窗开小了")
    # 带上 score（有的话）—— 拒绝时它是"差多少"最直接的量
    if diag.get("score") is not None and code != "ambiguous_theta":
        bits.append(f"score={float(diag['score']):.4f}")
    if diag.get("source"):
        bits.append(f"source={diag['source']}")
    return ",".join(bits)


def valid_false_frame(i, reject="ambiguous_theta", **extra):
    """**颜色/深度交替 + 持续 `valid=false` + `rejects` 逐帧变**（评审的现场）。

    `score` 每 tick 变一点（真实检测器的分数本来就在抖），`source` 在
    `color` / `depth` 之间交替，且**两者都出现在 `rejects` 串里**。
    """
    source = "color" if i % 2 == 0 else "depth"
    return pallet_detection(
        valid=False, source=source, stamp=float(i),
        rejects=producer_rejects(reject=reject, score=0.30 + i * 1e-4,
                                 source=source, **extra))


# --------------------------------------------------------------------------- #
# 假消息
# --------------------------------------------------------------------------- #
class _Pt:
    def __init__(self, x, y):
        self.x, self.y = x, y


def _header(stamp=1.0):
    from types import SimpleNamespace
    return SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda: stamp))


def straight_down(x_mm=0.0):
    """托盘在相机正下方 1m、台面与像面平行。单位阵旋转 → det = +1。"""
    T = np.eye(4)
    T[2, 3] = 1.0
    T[0, 3] = x_mm / 1000.0
    return T


def mirrored():
    """把 e2 取反 → 旋转块 det = -1，是**镜面反射不是旋转**。"""
    T = straight_down()
    T[:3, 1] = -T[:3, 1]
    return T


def scaled(x=1.0, y=1.0):
    """把 e1/e2 各乘一个系数 → 旋转块 `diag(x, y, 1)`，`det = x·y`。

    合成"**不是正交矩阵**"的位姿用。手性判据判的是**手性**（det 判给 +1 还是 −1），
    所以 `det` 落在 `[0.5, 2.0]` 这一段时（离 −1 更远、离 +1 更近）都该放行 ——
    这正是"离 +1 够不够近"那类阈值式错法会误拒的地方。
    """
    T = straight_down()
    T[0, 0] = x
    T[1, 1] = y
    return T


CORNERS = [(680.0, 800.0), (960.0, 760.0), (980.0, 1080.0), (700.0, 1120.0)]


def pallet_detection(T=None, valid=True, source="color", det=None, stamp=1.0,
                     include_det=True, size_mm=(1200.0, 1000.0), rejects=""):
    """`pallet_detection_msgs/PalletDetection` 的形状。

    `rejects` 默认空串（契约：空串 = 没有失败）。要造真实生产者的串用
    `producer_rejects()` / `valid_false_frame()`。
    """
    from types import SimpleNamespace
    T = straight_down() if T is None else np.asarray(T, float)
    if det is None:
        det = float(np.linalg.det(T[:3, :3]))
    fields = dict(
        header=_header(stamp),
        T_cam_pallet=[float(v) for v in T.reshape(-1)],
        valid=valid, source=source,
        size_mm=[float(v) for v in size_mm],
        corners_uv=[_Pt(u, v) for u, v in CORNERS],
        n_used=1, n_slots=1, n_failed=0,
        spread_mm=0.0, spread_deg=0.0, latency_ms=12.0,
        diag="score=0.83", rejects=rejects)
    if include_det:
        fields["det"] = det
    return SimpleNamespace(**fields)


def make_node(**over):
    params = {"topic": "/pallet/detection", "pallet_key": "latest_pallet"}
    params.update(over)
    node = NodePalletObs("pallet", "pallet", "ns", params)
    # 测试环境没有 ROS master，`initialise()` 会给出 setup_error（那是**对的**
    # 行为）。这里把订阅那一层假装好，专心测黑板与闸门。
    node._subscriber = object()
    node._setup_error = ""
    return node


def feed(node, msg):
    """走真实的回调路径，不直接塞 `_pending`。"""
    node._on_message(msg)


# --------------------------------------------------------------------------- #
# 解析层（纯函数）
# --------------------------------------------------------------------------- #
def test_parses_the_matrix_and_converts_to_pose6d():
    parsed = parse_pallet_message(pallet_detection(stamp=7.5))
    assert parsed is not None
    T = parsed["T_cam_pallet"]
    assert T.shape == (4, 4)
    assert np.allclose(T, straight_down())
    assert parsed["valid"] is True
    assert parsed["has_valid_field"] is True
    assert abs(parsed["stamp"] - 7.5) < 1e-9
    assert abs(parsed["det"] - 1.0) < 1e-9
    assert parsed["mode"] == "T_cam_pallet"


def test_row_major_order_is_respected():
    """行主序 vs 列主序搞错会让位姿整体转置 —— 静默错，必须钉住。"""
    T = np.eye(4)
    T[0, 3] = 0.25          # 只动平移，转置后这个数会跑到 [3, 0]
    parsed = parse_pallet_message(pallet_detection(T=T))
    assert abs(parsed["T_cam_pallet"][0, 3] - 0.25) < 1e-12
    assert abs(parsed["T_cam_pallet"][3, 0]) < 1e-12


def test_the_determinant_is_computed_locally_not_taken_on_trust():
    """★ 上游把 `det` 填错时，本节点算出来的仍是真值。

    手性判据是"不报错、只是算错"那一类，**不能靠上游自觉**。
    """
    parsed = parse_pallet_message(pallet_detection(T=mirrored(), det=+1.0))
    assert parsed["det"] < -0.9, f"本节点该自己算出 -1，实际 {parsed['det']}"
    assert parsed["det_mismatch"] is True, "上游自报的 det 与本地的对不上，要留痕"


def test_a_message_without_a_det_field_is_still_checked():
    """★ 没有 `det` 字段 ≠ 合格。本节点自己算，照样能判手性。"""
    parsed = parse_pallet_message(pallet_detection(T=mirrored(), include_det=False))
    assert parsed is not None
    assert parsed["has_det_field"] is False
    assert parsed["det"] < -0.9


def test_unrecognised_and_bad_messages_return_none_and_never_raise():
    assert parse_pallet_message(None) is None
    from types import SimpleNamespace
    assert parse_pallet_message(SimpleNamespace()) is None
    assert parse_pallet_message(SimpleNamespace(T_cam_pallet=[1.0, 2.0])) is None
    bad = pallet_detection()
    bad.T_cam_pallet = [float("nan")] * 16
    assert parse_pallet_message(bad) is None
    bad2 = pallet_detection()
    bad2.T_cam_pallet = [0.0] * 16          # 全零 → det = 0，定不出坐标系
    assert parse_pallet_message(bad2) is not None, "解析层不该拒，那是闸门 2 的事"


def test_no_valid_field_parses_as_true():
    """★ **没有** `valid` 字段 ≠ `valid=false`，且 `has_valid_field` 要如实为 False。

    与 `test_node_box_obs.py:98-104` 同一个坑（那边注释原话：「光有节点级用例
    根本区分不出」）：节点的闸门写作

        `has_valid_field and require_valid and not valid`

    于是"没有字段"这一路上 `valid` 取什么值**都**走不到闸门里 —— 只测节点行为的话，
    `True if valid is None else bool(valid)` 被改成 `bool(valid)`（→ 默认 False）
    没有任何测试会红。两条断言各钉一行，且**不依赖闸门的短路**：

      * `valid` 默认值变异 → 第一条红
      * `has_valid_field` 变异 → 第二条红
    """
    from types import SimpleNamespace
    msg = pallet_detection(stamp=6.0)
    del msg.valid
    parsed = parse_pallet_message(msg)
    assert parsed is not None
    assert parsed["has_valid_field"] is False, "消息里没有 valid 字段，要如实报 False"
    assert parsed["valid"] is True, "没有 valid 字段 ≠ valid=false，默认必须是 True"
    # 节点层：这一帧该照常写（`test_a_message_without_a_valid_field_is_still_written`
    # 已经钉了行为，这里只补解析层那两条，不重复）


def test_missing_header_stamp_is_written_as_zero_and_warned(caplog):
    """★ 上游没填 `header.stamp` 时**写原始值 0.0**，不编造处理时刻，并告警一次。

    编造一个"看起来像时间"的数（`stamp or time.time()`）会让下游的
    `pair_stamp_zero` 与"缺 stamp"告警**永远不会响**：托盘侧用 tick 时刻、
    箱子侧用图像时刻，箱子那 ~60ms 的检测耗时整个变成 `pair_dt` —— 要么永远
    配不上，要么配上一对**不同帧的图像**，两种都不报错。
    """
    import logging
    from types import SimpleNamespace
    # 解析层：`to_sec()` 返回 0.0 → stamp 就是 0.0，不许被换成墙上时刻
    parsed = parse_pallet_message(pallet_detection(stamp=0.0))
    assert parsed["stamp"] == 0.0, \
        f"缺 stamp 时解析层编造了时间戳：{parsed['stamp']!r}"

    node = make_node()
    with caplog.at_level(logging.WARNING):
        feed(node, pallet_detection(stamp=0.0))
        assert node.update() == Status.RUNNING
    assert getattr(node.global_blackboard, "latest_pallet_stamp") == 0.0, \
        "缺 stamp 时 `_stamp` 必须是原始值 0.0（配对侧据此显式 reject）"
    # 位姿本身照写 —— 缺 stamp 是**上游的问题**，不是这一帧的位姿不能用
    assert getattr(node.global_blackboard, "latest_pallet") is not None
    assert "取不到 / 非正" in caplog.text, caplog.text
    assert "写的是**原始值 0.0**" in caplog.text, caplog.text

    # **只告警一次**（`_warn_once` 口径），不是每 tick 刷屏
    caplog.clear()
    with caplog.at_level(logging.WARNING):
        feed(node, pallet_detection(stamp=0.0))
        assert node.update() == Status.RUNNING
    assert caplog.text == "", f"同一条告警喊了第二遍：{caplog.text}"


def test_a_stretched_matrix_far_from_minus_one_is_still_written():
    """★ 手性判据是「判给两个假设（+1 与 −1）中**更近**的那个」，不是"离 +1 够近"。

    取 `det = 1.6`（e1 被拉长 1.6 倍，**不是正交矩阵**）：它离 +1 有 0.6，
    但离 −1 有 2.6 —— 按"最近假设"必须**放行**。判据判的是**手性**（镜面 or 不是），
    **不是正交性**（点得准不准）：拿容差去要求正交性，正常检测噪声一帧都过不了，
    节点变成永远不写。`algorithm.HANDEDNESS_MARGIN` 那段注释把这条写死了。

    阈值式判据（`abs(det - 1) > 0.5` 判坏）在这个取值上**必然红**：
    0.6 > 0.5，它会把这一帧判成"不能用"。
    """
    T = scaled(x=1.6, y=1.0)                # det = 1.6
    assert abs(np.linalg.det(T[:3, :3]) - 1.6) < 1e-12
    parsed = parse_pallet_message(pallet_detection(T=T))
    assert abs(parsed["det"] - 1.6) < 1e-9, parsed["det"]

    node = make_node()
    feed(node, pallet_detection(T=T, stamp=7.0))
    assert node.update() == Status.RUNNING
    assert getattr(node.global_blackboard, "latest_pallet") is not None, \
        "det=1.6 离 −1 更远、离 +1 更近 —— 按最近假设该放行（判的是手性不是正交性）"
    assert getattr(node.global_blackboard, "latest_pallet_version") == 1


def test_a_real_noisy_frame_is_not_rejected_by_a_strict_threshold():
    """★ 严格阈值式判据（`abs(det - 1) <= 1e-6` 判坏）会把**真实噪声帧**全拒掉。

    `det = 0.99979` 是评审实测的真机噪声帧（相对 +1 差 2.1e-4，远大于 1e-6，
    但离 −1 有 2.0）。按"最近假设"它是右手系，必须放行；按"离 +1 够不够近"
    它会被判"不能用" —— 于是本来能用的位姿一个都写不进去，而节点只会说
    "跳过：行列式…"，看上去像是上游坏了。
    """
    T = scaled(x=0.99979, y=1.0)
    assert abs(np.linalg.det(T[:3, :3]) - 0.99979) < 1e-12

    node = make_node()
    feed(node, pallet_detection(T=T, stamp=8.0))
    assert node.update() == Status.RUNNING
    assert getattr(node.global_blackboard, "latest_pallet") is not None, \
        "det=0.99979 的噪声帧该放行（它离 −1 有 2.0）"
    assert getattr(node.global_blackboard, "latest_pallet_version") == 1


def test_upstream_det_mismatch_warns_but_does_not_block(caplog):
    """上游自报的 `det` 与本地算的差 > `DET_MISMATCH_TOL` → 记一次 WARNING，**不拦**。

    拦的判据是**本节点自己算出来的**那个（闸门 2），上游那个字段只是诊断 ——
    把它当判据就等于又信了上游一次。所以这条必须同时钉住"告警响了"与
    "位姿照写"。

    ⚠️ 断言的是**文案里的具体片段**，不是 `"det" in caplog.text` —— 话题名
    `/pallet/detection` 与 `_topic` 里**本来就有** `det`，那种断言只要分支被走到
    就必过：删掉整段交叉核对（连告警一起删）它照样绿。
    """
    import logging
    node = make_node()
    with caplog.at_level(logging.WARNING):
        feed(node, pallet_detection(det=0.5, stamp=9.0))    # 本地算的是 +1.0
        assert node.update() == Status.RUNNING
    assert "对不上" in caplog.text and "0.500000" in caplog.text, caplog.text
    assert "上游那一步多半有 bug" in caplog.text, caplog.text
    assert getattr(node.global_blackboard, "latest_pallet") is not None, \
        "上游 det 报错不拦写入 —— 判据用的是本地算的那个"
    assert getattr(node.global_blackboard, "latest_pallet_version") == 1


def test_unreadable_det_field_skips_the_crosscheck_with_a_note(caplog):
    """`det` 是垃圾（字符串）时交叉核对**静默跳过** —— 这是**有意的**（判据用本地
    算的），但日志里要留一句，别让"没核对"和"核对过且一致"长得一模一样。

    ⚠️ 同 `test_upstream_det_mismatch_warns_but_does_not_block`：钉的是
    `"取不出数"` 这个**只可能来自这段诊断**的片段。断言 `"det"` 是恒真的
    （话题名里就有），变异成"整段诊断不写"也照样绿。
    """
    import logging
    node = make_node()
    with caplog.at_level(logging.WARNING):
        feed(node, pallet_detection(det="不是数", stamp=10.0))
        assert node.update() == Status.RUNNING
    assert "取不出数" in caplog.text and "交叉核对跳过" in caplog.text, caplog.text
    assert getattr(node.global_blackboard, "latest_pallet") is not None, \
        "上游 det 字段是垃圾不该拒掉这一帧（位姿本身是好的）"


def test_upstream_det_mismatch_is_reported_even_when_the_pose_is_rejected(caplog):
    """★ 镜面帧 + 上游自报 `det=+1.0` → **两条告警都要出现**。

    「上游自报的与本节点算的对不上」是**唯一能指出上游那一步有 bug** 的线索，
    而"镜面"本身往往就是那个 bug 的后果 —— 所以这个场景恰恰是最该看到它的地方。
    交叉核对若排在闸门 2 **之后**，这一帧会被闸门 2 提前 `return`，那条线索
    **永远不会响**（实测只出一条"位姿不能用"）。

    ⚠️ 前移的只是**诊断动作**，闸门顺序语义不变：闸门 2 仍然挡住这一帧、
    仍然一个字都不写。
    """
    import logging
    node = make_node()
    with caplog.at_level(logging.WARNING):
        feed(node, pallet_detection(T=mirrored(), det=+1.0, stamp=12.0))
        assert node.update() == Status.RUNNING
    # 闸门 2 照旧挡下这一帧（一个字都不写）
    assert getattr(node.global_blackboard, "latest_pallet", "缺") is None
    assert getattr(node.global_blackboard, "latest_pallet_version") == 0
    # 两条线索都要在
    assert "位姿不能用" in caplog.text, caplog.text
    assert "对不上" in caplog.text, caplog.text
    assert "上游那一步多半有 bug" in caplog.text, caplog.text


# --------------------------------------------------------------------------- #
# `size_mm` 交叉核对（I1）：**只告警不拦**，基准是本节点自己的 `pallet_size_mm`
# --------------------------------------------------------------------------- #
def test_size_mm_mismatch_warns_once_with_both_values_but_still_writes(caplog):
    """★ 上游 `size_mm` 与配置的 `pallet_size_mm` 差得远 → WARNING，**位姿照写**。

    `size_mm` 从来不是输入（`.msg` 明说伺服的投影基准是**配置值**），所以这条
    与 `det` 那条一样**只留痕、不拦**。它要抓的是"少写一位"这一类**量级错**
    （1200 → 120）：那时参考边整体投到错的位置，而**三个数照样算得出来**、
    现场只有人眼在叠加图上看得出不对。
    """
    import logging
    node = make_node(pallet_size_mm=[1200.0, 1000.0])
    with caplog.at_level(logging.WARNING):
        for i in range(4):
            feed(node, pallet_detection(size_mm=(120.0, 1000.0), stamp=20.0 + i))
            assert node.update() == Status.RUNNING
    assert caplog.text.count("差得远") == 1, (
        f"同一个量级错该只喊一次，实际 {caplog.text.count('差得远')} 条：{caplog.text}")
    # **两个实际值都要点出来** —— 不然操作员不知道差在哪一边
    assert "120" in caplog.text and "1200" in caplog.text, caplog.text
    assert getattr(node.global_blackboard, "latest_pallet") is not None, \
        "size_mm 对不上不许拦写入 —— 判据与几何用的是配置值"


def test_a_noisy_size_mm_does_not_warn(caplog):
    """几个**百分点**的量测噪声不许报警 —— 阈值就是按它定的（3% / 2%）。

    检测器按已知尺寸搜索，报出来的值**必然**接近配置（`.msg` 写着这个字段有
    一定程度的自我实现），所以正常运行只该有几个百分点的抖动。这条与上面那条
    "120 → 1200" 一起夹住阈值：阈值调到噪声以下，这里红；调到噪声与 1× 之间
    的任一值，两条都还得绿。
    """
    import logging
    node = make_node(pallet_size_mm=[1200.0, 1000.0])
    with caplog.at_level(logging.WARNING):
        feed(node, pallet_detection(size_mm=(1240.0, 980.0), stamp=30.0))
        assert node.update() == Status.RUNNING
    assert "差得远" not in caplog.text, caplog.text
    assert getattr(node.global_blackboard, "latest_pallet") is not None


def test_size_check_is_silently_skipped_when_pallet_size_mm_is_not_given(caplog):
    """**没给** `pallet_size_mm` 不算坏参数：整条核对**静默**跳过。

    它是诊断、不是闸门，所以既有场景一行都不用改。**静默**是有意的：没给不是
    错误，不该为它吵一句（与 `parse_pair_param` 的"(None, None) = 没给"同口径）。
    """
    import logging
    node = make_node()
    assert node._size_ref is None
    with caplog.at_level(logging.WARNING):
        feed(node, pallet_detection(size_mm=(120.0, 100.0), stamp=40.0))
        assert node.update() == Status.RUNNING
    assert "差得远" not in caplog.text, caplog.text
    assert getattr(node.global_blackboard, "latest_pallet") is not None


def test_a_bad_pallet_size_mm_is_named_and_only_disables_the_check(caplog):
    """给了个**解释不了**的值 = 坏参数：点名（写实际值）+ 退成"没给"，**不抛**。

    与"没给"必须区分开：没给是静默的，坏参数一定要有人说话 —— 否则操作员以为
    这条诊断在跑，其实它一次都没跑过。
    """
    import logging
    with caplog.at_level(logging.WARNING):
        node = make_node(pallet_size_mm=[1200.0])       # 只给了一个数
    assert "pallet_size_mm" in caplog.text and "1200.0" in caplog.text, caplog.text
    assert node._size_ref is None
    with caplog.at_level(logging.WARNING):
        feed(node, pallet_detection(size_mm=(120.0, 100.0), stamp=50.0))
        assert node.update() == Status.RUNNING
    assert "差得远" not in caplog.text, caplog.text
    assert getattr(node.global_blackboard, "latest_pallet") is not None


def test_size_check_ignores_valid_false_placeholder_frames(caplog):
    """`valid=false` 的帧下面全是**占位值**（`size_mm` 填 0）—— 拿它比会每帧都喊。

    ⚠️ 这条**必须把闸门 1 关掉**（`require_valid="false"`）才有判别力：默认
    配置下闸门 1 会在调用核对**之前**就 `return`，"占位帧不判"那个分支根本
    走不到 —— 那时删掉它测试照样绿。
    """
    import logging
    node = make_node(pallet_size_mm=[1200.0, 1000.0], require_valid="false")
    with caplog.at_level(logging.WARNING):
        feed(node, pallet_detection(valid=False, size_mm=(0.0, 0.0), stamp=60.0))
        assert node.update() == Status.RUNNING
    assert "差得远" not in caplog.text, (
        f"占位帧的 size_mm=0 不是「检测器报了个 0 尺寸」：{caplog.text}")


def test_garbage_det_plus_missing_stamp_warns_each_once_not_every_tick(caplog):
    """★ **两条新告警各有各的槽位** —— 同一条告警不会因为另一条喊了就重喊。

    失败场景（评审实测）：上游每帧报垃圾 `det` **且** `header.stamp` 缺失 ——
    这两个恰好是同一种「上游没按契约填字段」的现场，会**同时发生**。两条告警
    若共用 `_warn_once` 的**单个**槽位，就会**每 tick 互相覆盖**，于是两条都每次
    都响：实测 **6 tick 出 12 条 WARNING**。

    断言**总 WARNING 数 == 2**（不是 12），且两条各自的文案都在 —— 这样
    "共用槽位"（红）与"只喊一条、另一条被吞掉"（也红）两种错法都能抓到。
    """
    import logging
    node = make_node()
    with caplog.at_level(logging.WARNING):
        for _ in range(6):
            feed(node, pallet_detection(det="垃圾", stamp=0.0))
            assert node.update() == Status.RUNNING
    warnings = [r for r in caplog.records if r.levelno == logging.WARNING]
    assert len(warnings) == 2, \
        f"6 tick 该只出 2 条 WARNING（每条告警一次），实际 {len(warnings)} 条：" \
        f"{[r.getMessage() for r in warnings]}"
    assert "取不出数" in caplog.text, caplog.text
    # ⚠️ 收紧（原为 `"header.stamp" in caplog.text`）：这句想钉的是**stamp 告警**
    # 也在，但 `"header.stamp"` 太泛 —— 钉到具体文案上，顺带把"文案里的原始值
    # 写的是什么"一并钉住（这条用例的输入是 `stamp=0.0`，实际值就是 `0.0`）。
    assert "取不到 / 非正" in caplog.text, caplog.text
    assert "写的是**原始值 0.0**" in caplog.text, caplog.text


def test_new_warnings_are_not_respammed_by_the_shared_gate_slot(caplog):
    """★ 两条新告警**各自的槽位**在「别的告警插进来」时也守得住（F1 的完整形态）。

    上一条钉的是"两条新告警**互相**覆盖"（同时共用**同一个** `_warn_once`）。
    这一条钉的是另一半：只要其中**任何一条**落回共用的 `_warn_once` 槽位，
    它就会被「这一帧被闸门挡了」那 4 条**本来就在互相覆盖**的告警挤掉 ——
    现场是一台上游检测器**交替**报两种坏帧（这正是最真实的样子）：

      * 奇数 tick：垃圾 `det` + `header.stamp` 缺失（两条新告警的现场）
      * 偶数 tick：镜面帧（闸门 2 挡下，走共用 `_warn_once` 槽位）

    闸门告警每帧把共用槽位写一遍，于是"寄在共用槽位上"的那条新告警**每两 tick
    重喊一次**（6 tick 出 3 条）。三条告警各自只该响**一次**。
    """
    import logging
    node = make_node()
    with caplog.at_level(logging.WARNING):
        for _ in range(3):
            feed(node, pallet_detection(det="垃圾", stamp=0.0))   # 奇数 tick
            assert node.update() == Status.RUNNING
            feed(node, pallet_detection(T=mirrored(), stamp=1.0))  # 偶数 tick
            assert node.update() == Status.RUNNING

    def count(fragment):
        return sum(1 for r in caplog.records
                   if r.levelno == logging.WARNING and fragment in r.getMessage())

    assert count("取不出数") == 1, \
        f"`det` 取不出数的告警被闸门告警挤掉了槽位，重喊了 {count('取不出数')} 遍"
    assert count("取不到 / 非正") == 1, \
        f"stamp 告警被闸门告警挤掉了槽位，重喊了 {count('取不到 / 非正')} 遍"
    assert count("位姿不能用") == 1, \
        f"闸门 2 的告警自己该只响一次，实际 {count('位姿不能用')} 遍"


def test_stamp_warning_does_not_respam_when_source_alternates(caplog):
    """★ stamp 告警的去重键**不含 `source`** —— 它是个每帧都在变的量。

    检测器交替报 `color` / `depth`（或隔帧 `valid=false`）时，文案里带
    `source` 就等于**没有去重**：槽位被"不同的文案"每 tick 覆盖一次，于是每 tick
    重喊（评审实测 5 tick 5 条）。5 个 tick 走的是**同一个** `header.stamp` 故障，
    操作员该看到的是**一条**。
    """
    import logging
    node = make_node()
    with caplog.at_level(logging.WARNING):
        for i in range(5):
            feed(node, pallet_detection(
                stamp=0.0, source="color" if i % 2 == 0 else "depth"))
            assert node.update() == Status.RUNNING
    warnings = [r for r in caplog.records if r.levelno == logging.WARNING]
    assert len(warnings) == 1, \
        f"同一条 stamp 故障被重喊了 {len(warnings)} 遍：" \
        f"{[r.getMessage() for r in warnings]}"
    assert "取不到 / 非正" in caplog.text, caplog.text


def test_valid_false_warning_does_not_respam_when_source_alternates(caplog):
    """★ `valid=false` 告警的文案**不含 `source`** —— 它是个每帧都在变的量。

    与 `test_stamp_warning_does_not_respam_when_source_alternates` 是**同一个病**
    （第二轮只点了 stamp 告警，漏了这条）：`_warn_once` 的去重键是**整条文案**，
    文案里带 `source={parsed['source'] or '?'}` 时，检测器交替报 `color` /
    `depth` 就等于**没有去重** —— 共用槽位每 tick 被"新文案"覆盖一次，
    **6 tick 出 6 条 WARNING**（评审实测）。

    这条告警要操作员去修的是**上游在发占位值**，`source` 对定位无用；
    **稳定性优先于信息量**（与 F6 同一口径）。

    变异：把 `source` 加回这条文案 → 本条必红（6 条）。
    """
    import logging
    node = make_node()
    with caplog.at_level(logging.WARNING):
        for i in range(6):
            feed(node, pallet_detection(
                valid=False, source="color" if i % 2 == 0 else "depth",
                stamp=float(i)))
            assert node.update() == Status.RUNNING
    warnings = [r for r in caplog.records if r.levelno == logging.WARNING]
    assert len(warnings) == 1, \
        f"同一条 `valid=false` 故障被重喊了 {len(warnings)} 遍：" \
        f"{[r.getMessage() for r in warnings]}"
    assert "valid=false" in caplog.text, caplog.text
    assert getattr(node.global_blackboard, "latest_pallet", "缺") is None, \
        "占位值一个字都不该写"


# --------------------------------------------------------------------------- #
# ★ 修复轮 4：去重键是**分类**，不是文案 —— 真实生产者上那个病
# --------------------------------------------------------------------------- #
def test_valid_false_warning_does_not_respam_on_real_producer_rejects(caplog):
    """★ **核心用例**：夹具用真实生产者的 `format_rejects()` 输出。

    现场（评审核实，第三轮的修复在这里**完全失效**）：「颜色/深度交替 +
    持续 `valid=false` + `rejects` 逐帧变」连跑 6 tick → **WARNING = 6**。

    为什么第三轮没治住：那条路是"把易变量一个个从文案里删掉"。删了 `source`，
    `rejects` 还在 —— 而按 `.msg` 契约 `rejects` **必须**写实际值与阈值
    （`score=0.3000<threshold=0.35`），**不写就排查不了**，所以它逐帧不同；
    生产者的 `format_rejects()` 甚至把 `source=` **又拼了回去**。

    上一轮新增的用例用的是 `rejects=""` 的假消息（见
    `test_valid_false_warning_does_not_respam_when_source_alternates`，它仍然是
    一条**独立**的防线），**结构上看不到这个问题** —— 所以这一条必须用真串。

    断言三条：① 恰好 1 条 WARNING；② `source` 与 `rejects` **都在第一条里**
    （信息量还回来了，不是"首次也喊短文案"）；③ 黑板一个字都没写。
    """
    import logging
    node = make_node()
    with caplog.at_level(logging.WARNING):
        for i in range(6):
            feed(node, valid_false_frame(i))
            assert node.update() == Status.RUNNING
    warnings = [r.getMessage() for r in caplog.records
                if r.levelno == logging.WARNING]
    assert len(warnings) == 1, \
        f"6 tick 该只出 1 条 WARNING（去重键是分类不是文案），实际 " \
        f"{len(warnings)} 条：{warnings}"
    first = warnings[0]
    # ★ 信息量：`rejects` 的**实际值与阈值**要在，且 `source` 也要在
    #   （生产者把它拼在 `rejects` 里）—— 这条断言能把"首次喊就用短文案"变红。
    assert "reject=ambiguous_theta" in first, first
    assert "score=0.3000<threshold=0.35" in first, first
    assert "source=color" in first, first
    assert "valid=false" in first, first
    assert getattr(node.global_blackboard, "latest_pallet", "缺") is None, \
        "占位值一个字都不该写"


def test_valid_false_warning_reminds_every_30_ticks(caplog):
    """★ **周期提醒不能省**：同一分类持续失败时，第 31 tick 必须再响一次。

    只在"分类变化"时喊，会让一个**持续失败**的检测器在操作员眼前**彻底安静**
    —— 那比刷屏更糟（刷屏至少还看得见）。节流口径沿用本节点既有的 30
    （`update()` 里 `self._ticks % 30 == 1`），不新造机制。

    第 1 tick 完整文案 + 第 31 tick 短文案 = **恰好 2 条**。
    """
    import logging
    node = make_node()
    with caplog.at_level(logging.WARNING):
        for i in range(31):
            feed(node, valid_false_frame(i))
            assert node.update() == Status.RUNNING
    warnings = [r.getMessage() for r in caplog.records
                if r.levelno == logging.WARNING]
    assert len(warnings) == 2, \
        f"第 1 tick 完整文案 + 第 31 tick 周期提醒 = 2 条，实际 " \
        f"{len(warnings)} 条：{warnings}"
    # 第二条是**短文案**：只报分类，不带易变量（那正是会让去重失效的东西）
    assert "reject=ambiguous_theta" in warnings[1], warnings[1]
    assert "score=" not in warnings[1], \
        f"周期提醒里带了逐帧在变的量：{warnings[1]}"


def test_valid_false_reminder_does_not_swallow_the_first_full_warning(caplog):
    """★ 周期提醒按**"上次喊这个分类的 tick"**算，不是全局 `_ticks % 30` 相位。

    变异场景：同一个 `valid=false` 故障**恰好从第 30 tick 开始**（前 29 tick
    上游还在正常发观测）。若周期提醒写成 `self._ticks % 30 == 1`，第 30 tick
    **不是**提醒点、第 31 tick 才是 —— 而第 30 tick 的**首次完整文案**已经把它
    按"同一分类 30 tick 内至多一条"挡住了。于是第 31 tick 的相位提醒会把**首次
    完整文案直接吞掉**：操作员只看到一条**短文案**，`rejects` 的实际值与阈值
    **一次都没出现过**（这正是上一轮"代价付了、信息量没换来"的翻版）。

    钉住：首条必须是**完整**文案。
    """
    import logging
    node = make_node()
    with caplog.at_level(logging.WARNING):
        # 前 29 tick：上游正常（有观测、valid=true，走写入路径）
        # ⚠️ stamp 从 1.0 起 —— `stamp=0.0` 会顺带触发 stamp 告警，把 WARNING 数搅浑
        for i in range(29):
            feed(node, pallet_detection(stamp=float(i + 1)))
            assert node.update() == Status.RUNNING
        # 第 30 tick：故障开始
        feed(node, valid_false_frame(30))
        assert node.update() == Status.RUNNING
        # 第 31 tick：同分类仍在失败 —— 不许把首条完整文案挤掉
        feed(node, valid_false_frame(31))
        assert node.update() == Status.RUNNING
    warnings = [r.getMessage() for r in caplog.records
                if r.levelno == logging.WARNING]
    assert len(warnings) == 1, \
        f"同一分类 30 tick 内该只有首条完整文案，实际 {len(warnings)} 条：{warnings}"
    assert "threshold=0.35" in warnings[0], \
        f"首次完整文案被周期提醒吞掉了，`rejects` 的实际值一次都没报出来：" \
        f"{warnings[0]}"


def test_reject_bucket_falls_back_to_the_whole_string():
    """★ `rejects` 里没有 `reject=` 段时**退回整串** —— 那是**有意的**。

    契约要求 `rejects` 写清是**哪一条**拒绝；取不到分类就退化成"每 tick 重喊"。
    那种串不符合契约，操作员**就该被打扰** —— **宁可退化成刷屏，也不要静默**。
    """
    assert _reject_bucket("reject=no_wood,n_wood=120<500,source=color") \
        == "reject=no_wood"
    assert _reject_bucket("reject=degenerate,最优矩形贴到栅格边界") == "reject=degenerate"
    assert _reject_bucket("") == ""
    assert _reject_bucket(None) == ""
    assert _reject_bucket("上游没填 rejects 字段") == "上游没填 rejects 字段"


def test_a_contract_violating_rejects_respams_on_purpose(caplog):
    """★ 兜底路径**每 tick 重喊** —— 这是**有意的**，不是漏网。

    `rejects` 不符合契约（没有 `reject=<code>`，比如上游只回显了一个逐帧在变的
    `score`）时，去重键退回整串 → 每 tick 都算"新分类" → 每 tick 一条 WARNING。
    操作员**就该被打扰**：这种串排查不了，不吵就等于静默。
    """
    import logging
    node = make_node()
    with caplog.at_level(logging.WARNING):
        for i in range(6):
            feed(node, pallet_detection(valid=False, stamp=float(i),
                                        rejects=f"score={0.3 + i:.4f}"))
            assert node.update() == Status.RUNNING
    warnings = [r.getMessage() for r in caplog.records
                if r.levelno == logging.WARNING]
    assert len(warnings) == 6, \
        f"`rejects` 不符合契约（没有 reject=<code>）时**就该每 tick 打扰**，" \
        f"实际 {len(warnings)} 条：{warnings}"


def test_a_new_reject_class_warns_immediately_with_the_full_text(caplog):
    """分类**变化** → 立刻喊完整文案（不是等到下一个 30 tick 的相位）。"""
    import logging
    node = make_node()
    with caplog.at_level(logging.WARNING):
        for i in range(6):
            feed(node, valid_false_frame(i, reject="ambiguous_theta"))
            assert node.update() == Status.RUNNING
        # 第 7 tick：换了一类拒绝 → 立刻完整喊一次
        feed(node, valid_false_frame(6, reject="no_wood", n_wood=120))
        assert node.update() == Status.RUNNING
    warnings = [r.getMessage() for r in caplog.records
                if r.levelno == logging.WARNING]
    assert len(warnings) == 2, f"换了分类该立刻再喊一条完整文案：{warnings}"
    assert "reject=no_wood" in warnings[1], warnings[1]
    assert "n_wood=120<500" in warnings[1], \
        f"新分类的首条必须是完整文案（带实际值与阈值）：{warnings[1]}"


def test_a_negative_stamp_is_written_as_is_not_folded_to_zero(caplog):
    """★ `header.stamp` 是**负数**时写**上游给的那个负数**，不折成 `0.0`。

    口径（模块 docstring「`latest_pallet_stamp` 的取值口径」）：「不编造」的语义是
    **保留上游给的实际值**，负数也是上游给的 —— 折成 `0.0` 反而是本节点替上游
    改了一个数。功能上两种都安全：配对侧 `pair_stamp_zero` 只在**两侧同时** ≤ 0
    时拒，单侧负数会一路走到 `pair_dt`（并如实报出那个负的 dt）。

    ⚠️ 这条钉的是"**没被折**"：`parsed["stamp"]` 与黑板上的 `_stamp` 都必须还是
    `-5.0`。变异成 `stamp = stamp if stamp > 0 else 0.0`（或 `max(stamp, 0.0)`）
    必红。
    """
    import logging
    parsed = parse_pallet_message(pallet_detection(stamp=-5.0))
    assert parsed["stamp"] == -5.0, \
        f"负 stamp 被折掉了：{parsed['stamp']!r}（该原样保留上游给的值）"

    node = make_node()
    with caplog.at_level(logging.WARNING):
        feed(node, pallet_detection(stamp=-5.0))
        assert node.update() == Status.RUNNING
    assert getattr(node.global_blackboard, "latest_pallet_stamp") == -5.0, \
        "黑板上的 `_stamp` 必须是上游给的 −5.0，不是 0.0"
    # 非正就该告警一次（与 stamp 缺失同一个出口）。⚠️ 钉**文案里的具体片段**，
    # 不是 `"header.stamp"` —— 后者在话题名里没有，但只要分支被走到就必过，
    # 钉不住"文案里的原始值写的是什么"。
    assert "取不到 / 非正" in caplog.text, caplog.text
    # ★ 这条告警的**两个** `{stamp!r}` 都要是**上游给的那个数**：一个回显输入、
    # 一个说黑板写的是什么。写死 `0.0`（F1 的变异）在这里必须红 ——
    # `header.stamp = -5.0` 的帧上，日志说"写的是原始值 0.0"就是**在说假话**
    # （黑板上是 `-5.0`）。
    assert "取到 -5.0" in caplog.text, caplog.text
    assert "写的是**原始值 -5.0**" in caplog.text, caplog.text


def test_an_unreadable_valid_field_is_treated_as_absent_and_the_frame_is_written():
    """闸门 1 的 `has_valid_field` 短路 —— **它在这套解析层下不可单独钉**（见下）。

    F3 想要的那条输入是「`parsed["valid"] is False` **且**
    `parsed["has_valid_field"] is False`」（只有短路能救它）。**实测不可构造**：

    解析层写的是 `valid = getattr(msg, "valid", None)` —— 带默认值的 `getattr`
    会把 `__getattr__` / property 抛出的 **`AttributeError` 一并吞掉**，
    所以"属性读不出来"这一路的结果是 `valid is None`，于是
    `has_valid_field=False`、`valid=True`（本用例第一条断言就是钉这个）。
    真实 ROS 消息上更不存在这个状态：`valid` 是普通字段，要么有值、要么没有。

    因此这里钉的是**可达的那一侧**（读不出来的 `valid` 与"没有 `valid` 字段"
    等价、这一帧照常写），并把短路标成**防御性**的：它保证的是
    「`valid` 取不到时 `not parsed["valid"]` 不会把帧拒掉」这条**硬约束**，
    而这条约束靠**解析层的默认值 `True`**（`test_no_valid_field_parses_as_true`
    钉住）就已经成立。改掉解析层的默认值 → 那条测试红；只删短路 → 无测试可红
    （这正是评审 V4 观测到的现象，**不是测试网的洞**，是没有可观测差异）。
    """
    class _UnreadableValid:
        """`valid` 属性读得到（`hasattr` True），但取值抛 `AttributeError`。"""

        def __init__(self, T):
            self.header = _header(13.0)
            self.T_cam_pallet = [float(v) for v in np.asarray(T).reshape(-1)]
            self.source = "color"
            self.diag = ""
            self.rejects = ""

        @property
        def valid(self):
            raise AttributeError("模拟字段名对不上")

    msg = _UnreadableValid(straight_down())
    parsed = parse_pallet_message(msg)
    assert parsed is not None, "解析层不该因为 valid 读不出来就整帧丢掉"
    assert parsed["has_valid_field"] is False
    assert parsed["valid"] is True, \
        "`getattr(..., None)` 吞掉了 AttributeError → 与「没有 valid 字段」等价，" \
        "不能因此把帧拒掉"

    # 与「消息里根本没有 valid 字段」**完全一致**（这就是"不可构造"的证据）
    no_field = pallet_detection(stamp=13.0)
    del no_field.valid
    assert parse_pallet_message(no_field)["valid"] is parsed["valid"] is True
    assert parse_pallet_message(no_field)["has_valid_field"] is False

    node = make_node()
    feed(node, msg)
    assert node.update() == Status.RUNNING
    assert getattr(node.global_blackboard, "latest_pallet") is not None, \
        "`valid` 取不到的帧该照常写 —— 闸门 1 的短路兜的就是这一路"
    assert getattr(node.global_blackboard, "latest_pallet_version") == 1


def test_a_failed_pose_conversion_does_not_swallow_the_frame(monkeypatch):
    """★ 转换失败的那一帧**不许**被标记成"已写过"。

    `_last_written_key` 若在 `matrix_to_pose6d()` **之前**赋值，转换一抛异常这一帧
    就已被记成写过 —— 后续**同 stamp 同矩阵**的帧会被"观测没变"那道闸静默跳过，
    黑板永远停在旧值，而每 tick 都返回 RUNNING、日志里只有一句"没变"。
    """
    import orchestration.nodes.node_pallet_obs as mod
    real = mod.matrix_to_pose6d
    calls = {"n": 0}

    def flaky(T):
        calls["n"] += 1
        if calls["n"] == 1:
            raise RuntimeError("模拟转换失败")
        return real(T)

    monkeypatch.setattr(mod, "matrix_to_pose6d", flaky)
    node = make_node()
    feed(node, pallet_detection(stamp=11.0))
    assert node.update() == Status.RUNNING
    assert getattr(node.global_blackboard, "latest_pallet", "缺") is None
    assert getattr(node.global_blackboard, "latest_pallet_version") == 0

    # 同一条观测再来一次：转换这次成功，**必须写进去**
    assert node.update() == Status.RUNNING
    assert getattr(node.global_blackboard, "latest_pallet") is not None, \
        "转换失败那帧被当成'已写过'了 —— 这一帧被静默吞掉"
    assert getattr(node.global_blackboard, "latest_pallet_version") == 1


# --------------------------------------------------------------------------- #
# 节点：黑板与闸门
# --------------------------------------------------------------------------- #
def test_writes_both_keys():
    node = make_node()
    assert node.update() == Status.RUNNING, "还没收到消息时是 RUNNING，不是 FAILURE"
    assert getattr(node.global_blackboard, "latest_pallet", "缺") is None
    assert getattr(node.global_blackboard, "latest_pallet_version") == 0
    assert getattr(node.global_blackboard, "latest_pallet_stamp") == 0.0

    feed(node, pallet_detection(stamp=1.0))
    assert node.update() == Status.RUNNING
    pose = getattr(node.global_blackboard, "latest_pallet")
    assert pose is not None
    # z 应当是 1 米（Pose6D 的单位是米）
    assert abs(pose.z - 1.0) < 1e-9, pose.to_list()
    # **三个键必须一起写**：只写值不写 version，下游版本门禁会停在 (0,0)；
    # 只写值不写 stamp，配对会静默退回 tick 时刻（两个检测器的耗时差直接变成
    # 配对的时间偏差，而不报错）。
    assert getattr(node.global_blackboard, "latest_pallet_version") == 1
    assert abs(getattr(node.global_blackboard, "latest_pallet_stamp") - 1.0) < 1e-9, \
        "stamp 没写 —— 配对会退回 tick 时刻，而那是静默的"


def test_valid_false_is_not_written():
    """★ 占位值不是观测。"""
    node = make_node()
    feed(node, pallet_detection(valid=False, source="", stamp=2.0))
    assert node.update() == Status.RUNNING
    assert getattr(node.global_blackboard, "latest_pallet", "缺") is None
    assert getattr(node.global_blackboard, "latest_pallet_version") == 0
    assert "valid=false" in node.feedback_message, node.feedback_message


def test_a_mirrored_matrix_is_not_written():
    """★ `det = -1` 是镜面，参考边会跑到托盘外面去。

    `matrix_to_pose6d()` 走 `Rotation.from_matrix`，会把它**静默投影**成
    "最近的旋转" —— 三个数照样算得出来。
    """
    node = make_node()
    feed(node, pallet_detection(T=mirrored(), stamp=3.0))
    assert node.update() == Status.RUNNING
    assert getattr(node.global_blackboard, "latest_pallet", "缺") is None, \
        "镜面的位姿一个字都不该写"
    assert getattr(node.global_blackboard, "latest_pallet_version") == 0
    # 只判**它该判的那个词**：原先写成 `"镜面" in fb or "det" in fb` 是近乎恒真的
    # OR（`feedback_message` 里只要出现 "det" 就过），变异成别的文案也照样绿。
    assert "镜面" in node.feedback_message, node.feedback_message


def test_a_degenerate_matrix_is_not_written():
    """`det ≈ 0` = e1 与 e2 近乎共线，坐标系定不出来。"""
    node = make_node()
    T = np.eye(4)
    T[:3, 0] = [1.0, 0.0, 0.0]
    T[:3, 1] = [1.0, 0.0, 0.0]      # e1 ∥ e2
    feed(node, pallet_detection(T=T, stamp=4.0))
    assert node.update() == Status.RUNNING
    assert getattr(node.global_blackboard, "latest_pallet", "缺") is None


def test_a_message_without_a_valid_field_is_still_written():
    """★ 与 `valid=false` **互为反面**：没有 `valid` 字段 ≠ `valid=false`。

    把"没有这个字段"当成 false，会把本来能用的输入全拒掉，节点变成永远不写。
    """
    from types import SimpleNamespace
    msg = pallet_detection(stamp=5.0)
    del msg.valid
    node = make_node()
    feed(node, msg)
    assert node.update() == Status.RUNNING
    assert getattr(node.global_blackboard, "latest_pallet") is not None
    assert getattr(node.global_blackboard, "latest_pallet_version") == 1


def test_the_same_observation_is_not_rewritten_every_tick():
    """同一条观测只写一次；来了新的才写。

    回调来一条就存一条，而 `update()` 每 tick 都跑 —— 不加这道闸就是
    "40 Hz 相机、10 Hz tick，每 tick 把同一帧重写一遍"：版本号一直涨，
    下游版本门禁每次放行，伺服拿同一个位姿白算。
    """
    node = make_node()
    feed(node, pallet_detection(stamp=1.0))
    for _ in range(5):
        assert node.update() == Status.RUNNING
    assert getattr(node.global_blackboard, "latest_pallet_version") == 1, \
        "同一条观测被重复写了"

    feed(node, pallet_detection(stamp=2.0))
    assert node.update() == Status.RUNNING
    assert getattr(node.global_blackboard, "latest_pallet_version") == 2


def test_unparseable_message_keeps_the_last_good_value():
    node = make_node()
    feed(node, pallet_detection(stamp=1.0))
    assert node.update() == Status.RUNNING
    from types import SimpleNamespace
    feed(node, SimpleNamespace())          # 认不出来的消息
    assert node.update() == Status.RUNNING
    assert getattr(node.global_blackboard, "latest_pallet_version") == 1, \
        "坏消息不该把好值冲掉"


def test_require_handedness_false_still_checks_nothing_else():
    node = make_node(require_handedness="false")
    feed(node, pallet_detection(T=mirrored(), stamp=1.0))
    assert node.update() == Status.RUNNING
    assert getattr(node.global_blackboard, "latest_pallet") is not None, \
        "关掉手性闸后镜面位姿该放行（调试用）"


def test_string_booleans_are_parsed():
    """`bool("false") is True` —— 场景 JSON 的 READ_BOARD 分支真会传字符串。"""
    node = make_node(require_valid="false")
    feed(node, pallet_detection(valid=False, stamp=1.0))
    assert node.update() == Status.RUNNING
    assert getattr(node.global_blackboard, "latest_pallet") is not None


def test_dry_run_subscribes_to_nothing(monkeypatch):
    monkeypatch.setenv("STUDIO_DRY_RUN", "1")
    node = NodePalletObs("pallet", "pallet", "ns", {"pallet_key": "latest_pallet"})
    node.initialise()
    assert node._subscriber is None
    assert node.update() == Status.SUCCESS


def test_dry_run_and_disabled_never_even_try_to_subscribe(monkeypatch):
    """★ `initialise()` 的 dry-run / `enabled=false` 早返回**在试订阅之前**。

    光看 `_subscriber is None` 是区分不出来的：真去 import rospy 失败了，
    `_setup_error` 也会被填上、订阅同样是 None（`update()` 那边 `_setup_error`
    的 FAILURE 分支在 dry-run 的 SUCCESS 分支**之后**，所以照样绿）。所以这里
    钉的是"**根本没去试**"：`_setup_error` 必须是空的。
    """
    monkeypatch.setenv("STUDIO_DRY_RUN", "1")
    node = NodePalletObs("pallet", "pallet", "ns", {"pallet_key": "latest_pallet"})
    node.initialise()                                   # 不许抛
    assert node._subscriber is None
    assert node._setup_error == "", \
        "dry-run 该在建订阅**之前**就返回，这里却去试了订阅（失败原因：" \
        f"{node._setup_error}）"
    assert node.update() == Status.SUCCESS

    monkeypatch.delenv("STUDIO_DRY_RUN", raising=False)
    off = NodePalletObs("pallet", "pallet", "ns",
                        {"pallet_key": "latest_pallet", "enabled": "false"})
    off.initialise()
    assert off._enabled is False and off._subscriber is None
    assert off._setup_error == "", "enabled=false 该在试订阅之前返回"
    assert off.update() == Status.SUCCESS
