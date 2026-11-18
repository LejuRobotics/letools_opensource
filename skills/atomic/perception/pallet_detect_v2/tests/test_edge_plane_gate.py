# -*- coding: utf-8 -*-
"""partial 帧的可见边/平面几何一致性闸门。

正确三边（真托盘投影，缺顶边）必须通过；bag2 模型实际给出的三边必须被拒。
bag2 数字来自 `2026-...` 记录帧：record D=1316.56mm 下
底边反投影 2123mm（托盘短边 1033.9）、B-R 73.0°、R-L 40.2°。
"""
import json

import numpy as np
import pytest

from .. import pose_decode as PD
from ..detect_pose import (_check_edge_plane_consistency,
                           _edge_select_reason, _partial_visible_gate,
                           _put_edge_selection_diag, _select_edge_pair)
from ..geom import CameraIntrinsics

pytestmark = pytest.mark.unit

K = CameraIntrinsics(366.23016357, 366.17419434, 321.08544922, 240.75099182)
NORMAL = np.array([-0.00410537, -0.59423562, -0.80428053])
W, H = 1033.9, 1141.1

TRUE_QUAD = np.array([[155.6, 374.6], [437.9, 374.1],
                      [398.7, 162.3], [211.0, 163.2]], float)
D_TRUE = 1368.8

# bag2 模型第 25 帧实际给出的三条边（顶边缺失）
BAG2_LINES = [
    {'p0': np.array([278.22, 280.20]), 'd': np.array([1.0, -0.0024]),
     'weight': 1.0},
    {'p0': np.array([518.71, 195.00]), 'd': np.array([-0.0213, -0.9998]),
     'weight': 1.0},
    None,
    {'p0': np.array([15.12, 205.43]), 'd': np.array([-0.0895, 0.9960]),
     'weight': 1.0},
]
D_BAG2_RECORD = 1316.5597282105018

# bag2 第 25 帧的关键点/置信度（det.kp_xy / det.kp_conf）
BAG2_KP = np.array([
    [8.4, 280.6], [179.4, 281.1], [358.5, 279.2], [520.5, 280.0],
    [516.7, 102.4], [512.3, 8.6], [508.6, -70.9], [361.8, -68.2],
    [204.9, -68.2], [41.4, -68.8], [33.3, 11.7], [24.3, 103.5],
], float)
BAG2_KP_CONF = np.array([
    0.801, 0.996, 0.992, 0.970, 0.890, 0.002, 0.0, 0.0,
    0.001, 0.002, 0.014, 0.591,
], float)


def _true_lines(drop=(2,)):
    out = []
    for i in range(4):
        if i in drop:
            out.append(None)
            continue
        p0 = TRUE_QUAD[i]
        p1 = TRUE_QUAD[(i + 1) % 4]
        d = p1 - p0
        d /= np.linalg.norm(d)
        out.append({'p0': p0, 'd': d, 'weight': 1.0})
    return out


def _with_visible(visible):
    lines = [None, None, None, None]
    for i in visible:
        lines[i] = {'p0': np.array([10.0 * i, 20.0]), 'd': np.array([1.0, 0.0]),
                    'weight': 1.0}
    return lines


@pytest.mark.parametrize('visible', [[0], [3]])
def test_one_visible_edge_rejected(visible):
    got, supported, reject = _partial_visible_gate(_with_visible(visible), True)
    assert got == visible
    assert supported is False
    assert reject == 'insufficient_visible_edges'


@pytest.mark.parametrize('visible', [[0, 1], [0, 3], [1, 2], [2, 3]])
def test_two_adjacent_edges_are_supported(visible):
    got, supported, reject = _partial_visible_gate(_with_visible(visible), True)
    assert got == visible
    assert supported is True
    assert reject is None


@pytest.mark.parametrize('visible', [[1, 3], [0, 2]])
def test_two_opposite_edges_are_rejected(visible):
    got, supported, reject = _partial_visible_gate(_with_visible(visible), True)
    assert got == visible
    assert supported is False
    assert reject == 'opposite_pair_only'


def test_explicit_adjacent_pair_passes_visible_gate():
    got, supported, reject = _partial_visible_gate(
        _with_visible([0, 1, 3]), True, pair=(0, 3))
    assert got == [0, 1, 3]
    assert supported is True
    assert reject is None


def test_bag2_visible_set_is_supported_and_bottom_right_plus_left():
    got, supported, reject = _partial_visible_gate(_with_visible([0, 1, 3]), True)
    assert got == [0, 1, 3]
    assert supported is True
    assert reject is None


@pytest.mark.parametrize('visible', [[0, 1, 2], [0, 1, 2, 3]])
def test_bottom_right_plus_top_is_supported(visible):
    got, supported, reject = _partial_visible_gate(_with_visible(visible), True)
    assert supported is True
    assert reject is None


def test_unsupported_set_with_D_is_rejected():
    got, supported, reject = _partial_visible_gate(_with_visible([0, 2, 3]), True)
    assert got == [0, 2, 3]
    assert supported is False
    assert reject == 'unsupported_visible_edge_set'


def test_unsupported_set_without_D_is_explicit_skip():
    """free_d / call 0 没有 D 时不能拒；但必须标 supported=False 供调用方记录。"""
    got, supported, reject = _partial_visible_gate(_with_visible([0, 2, 3]), False)
    assert got == [0, 2, 3]
    assert supported is False
    assert reject is None


def test_correct_three_edges_pass():
    check = _check_edge_plane_consistency(
        _true_lines(), K, NORMAL, D_TRUE, (W, H))
    assert check['checked'] is True, check
    assert check['passed'] is True, check
    assert check['ratio'] == pytest.approx(1.0, abs=0.03), check
    assert check['angle_bottom_right_deg'] == pytest.approx(90.0, abs=1.0), check
    assert check['angle_right_left_deg'] == pytest.approx(0.0, abs=1.0), check
    assert check['fails'] == []


def test_selection_rejects_opposite_only():
    """只剩 right+left 两条对边：即使都很可靠也必须拒。"""
    kp = np.tile(np.array([[320.0, 240.0]]), (12, 1))
    conf = np.full(12, 0.95)
    l1 = PD.EdgeLine(p0=np.array([400.0, 200.0]), d=np.array([0.0, 1.0]),
                     n_points=4, residual_px=0.1, mean_conf=0.95,
                     idx=(3, 4, 5, 6))
    l3 = PD.EdgeLine(p0=np.array([100.0, 200.0]), d=np.array([0.0, 1.0]),
                     n_points=4, residual_px=0.1, mean_conf=0.95,
                     idx=(9, 10, 11, 0))
    sel = _select_edge_pair([None, l1, None, l3], kp, conf, 640, 480)
    assert sel['pair'] is None
    assert sel['reason'] == 'opposite_pair_only'
    assert sel['opposite_visible'] is True


def _true_kp_and_conf():
    q = TRUE_QUAD
    kp = np.empty((12, 2), float)
    kp[0], kp[3], kp[6], kp[9] = q[0], q[1], q[2], q[3]
    kp[1] = q[0] + (q[1] - q[0]) / 3.0
    kp[2] = q[0] + 2.0 * (q[1] - q[0]) / 3.0
    kp[4] = q[1] + (q[2] - q[1]) / 3.0
    kp[5] = q[1] + 2.0 * (q[2] - q[1]) / 3.0
    kp[7] = q[2] + (q[3] - q[2]) / 3.0
    kp[8] = q[2] + 2.0 * (q[3] - q[2]) / 3.0
    kp[10] = q[3] + (q[0] - q[3]) / 3.0
    kp[11] = q[3] + 2.0 * (q[0] - q[3]) / 3.0
    conf = np.full(12, 0.95)
    conf[[6, 7, 8, 9]] = 0.0                 # 顶边不可见
    return kp, conf


def test_corrupt_edge_is_dropped_and_other_adjacent_pair_selected():
    """把 right 边弄成贴边 + 低 conf/高残差：应丢它，选 (bottom,left)。"""
    kp, conf = _true_kp_and_conf()
    kp[4] = np.array([635.0, 108.0])         # 距右边框 4px
    conf[4] = 0.10
    lines = PD.fit_edges(kp, conf, kconf=0.5, line_res_px=12.0)
    ln = lines[1]
    lines[1] = PD.EdgeLine(p0=ln.p0, d=ln.d, n_points=2,
                           residual_px=50.0, mean_conf=0.10, idx=(3, 4))
    sel = _select_edge_pair(lines, kp, conf, 640, 480)
    assert set(sel['pair']) == {0, 3}, sel
    assert sel['dropped'] == [1], sel
    scores = {s['edge']: s for s in sel['scores'] if s is not None}
    assert scores[1]['score'] < 0.12, scores
    assert scores[1]['border_dist_px'] < 60.0, scores


def test_selection_diag_fields_are_json_serializable():
    """伺服叠加图直接吃这些顶层字段；必须全是 Python 标量/list/dict。"""
    lines = PD.fit_edges(BAG2_KP, BAG2_KP_CONF, kconf=0.5, line_res_px=12.0)
    sel = _select_edge_pair(lines, BAG2_KP, BAG2_KP_CONF, 640, 480)
    reason = _edge_select_reason(sel, applied=True)
    diag = {}
    _put_edge_selection_diag(diag, BAG2_KP, BAG2_KP_CONF, sel['scores'],
                             sel['pair'], sel['dropped'], reason)
    json.dumps(diag)                            # 不应抛 TypeError

    assert len(diag['keypoints_uv']) == 12
    assert len(diag['keypoints_uv'][0]) == 2
    assert len(diag['keypoints_conf']) == 12
    assert len(diag['edge_score']) == 4
    assert diag['edge_score'][2] is None        # 顶边不可见
    assert diag['edge_score'][0] == pytest.approx(0.619, abs=0.02)
    assert diag['selected_edges'] == [0, 1]
    assert diag['dropped_edges'] == [3]
    assert diag['edge_select_reason'] == 'low_border_conf'
    parts = {p['edge']: p for p in diag['edge_score_parts'] if p}
    for key in ('mean_conf', 'min_conf', 'n_points', 'residual_px',
                'border_dist_px', 'border_score', 'center_score'):
        assert key in parts[0], key



def test_clipped_corner_does_not_kill_a_long_central_edge():
    """2026-11-11 现场：右下角 x=647.75 出画，但底边中点仍在画面中央。

    旧的"任一关键点贴边就把边界分打到 0"会把底边压到 0.05，选不出任何相邻
    边对，整帧报 `no_reliable_edge_pair`。改成按参与点的中位数后，底边仍有
    0.34，可选 (left, bottom)。
    """
    kp = np.array([
        [188.3541, 411.2084], [342.1096, 407.5651],
        [504.1629, 403.2272], [647.7517, 396.6658],
        [624.8962, 218.6094], [601.8564, 65.8905],
        [584.6805, -20.4808], [462.7648, -19.0053],
        [342.5201, -17.8717], [219.5658, -17.9031],
        [211.0282, 75.1711], [200.1555, 232.8722]], float)
    conf = np.array([0.9556, 0.9933, 0.9863, 0.7101, 0.9608, 0.7405,
                     0.00034, 0.00071, 0.00083, 0.00088, 0.8789, 0.9958])
    lines = PD.fit_edges(kp, conf, kconf=0.5, line_res_px=12.0)
    sel = _select_edge_pair(lines, kp, conf, 640, 480)
    scores = {s['edge']: s for s in sel['scores'] if s is not None}
    assert scores[0]['border_dist_px'] > 24.0, scores
    assert scores[0]['score'] >= 0.12, scores
    assert sel['pair'] == [3, 0], sel
    assert sel['dropped'] == [1], sel

def test_opposite_only_select_reason():
    kp = np.tile(np.array([[320.0, 240.0]]), (12, 1))
    conf = np.full(12, 0.95)
    l1 = PD.EdgeLine(p0=np.array([400.0, 200.0]), d=np.array([0.0, 1.0]),
                     n_points=4, residual_px=0.1, mean_conf=0.95,
                     idx=(3, 4, 5, 6))
    l3 = PD.EdgeLine(p0=np.array([100.0, 200.0]), d=np.array([0.0, 1.0]),
                     n_points=4, residual_px=0.1, mean_conf=0.95,
                     idx=(9, 10, 11, 0))
    sel = _select_edge_pair([None, l1, None, l3], kp, conf, 640, 480)
    assert _edge_select_reason(sel, applied=True) == 'opposite_pair_only'


def test_bag2_model_three_edges_are_rejected():
    check = _check_edge_plane_consistency(
        BAG2_LINES, K, NORMAL, D_BAG2_RECORD, (W, H))
    assert check['checked'] is True, check
    assert check['passed'] is False, check
    assert check['ratio'] == pytest.approx(2.05, abs=0.05), check
    assert check['angle_bottom_right_deg'] == pytest.approx(73.04, abs=0.3), check
    assert check['angle_right_left_deg'] == pytest.approx(40.24, abs=0.3), check
    assert set(check['fails']) == {
        'bottom_length', 'bottom_right_angle', 'right_left_parallel'}, check


def test_long_side_parallel_targets_max_edge():
    """`long_side_parallel=True` 时底边是长边，目标长度必须取 max(size_mm)。

    这里用同一组真三边，只验证目标边与 ratio 的分母；若仍死用 min 会在
    长边场景把正确三边误拒或漏判。
    """
    check = _check_edge_plane_consistency(
        _true_lines(), K, NORMAL, D_TRUE, (W, H), long_side_parallel=True)
    assert check['checked'] is True, check
    assert check['target_edge_src'] == 'long_side'
    assert check['target_edge_mm'] == pytest.approx(max(W, H)), check
    assert check['ratio'] == pytest.approx(check['bl_br_mm'] / max(W, H),
                                           abs=1e-3), check


def test_missing_left_checks_bottom_right_pair():
    """缺 left：退化成两条相邻边 (bottom,right)，检查夹角。"""
    lines = _true_lines()
    lines[3] = None
    check = _check_edge_plane_consistency(lines, K, NORMAL, D_TRUE, (W, H))
    assert check['checked'] is True
    assert check['passed'] is True
    assert check['pair'] == [0, 1]
    assert check['angle_pair_deg'] == pytest.approx(90.0, abs=1.0)
    assert check['ratio'] is None
    assert check['checks_run'] == ['edge_pair_angle']


def test_missing_right_checks_bottom_left_pair():
    """缺 right：退化成两条相邻边 (bottom,left)，同样能检查夹角。"""
    lines = _true_lines()
    lines[1] = None
    check = _check_edge_plane_consistency(lines, K, NORMAL, D_TRUE, (W, H))
    assert check['checked'] is True
    assert check['passed'] is True
    assert check['pair'] == [0, 3]
    assert check['angle_pair_deg'] == pytest.approx(90.0, abs=1.0)


def test_explicit_pair_gate_rejects_non_right_angle():
    """两条边投影后夹角必须接近 90°；人为改成 60° 必须拒。"""
    lines = _true_lines()
    lines[1] = {'p0': TRUE_QUAD[1], 'd': np.array([0.5, -0.8660254]),
                'weight': 1.0}
    check = _check_edge_plane_consistency(
        lines, K, NORMAL, D_TRUE, (W, H), pair=(0, 1))
    assert check['checked'] is True
    assert check['passed'] is False
    assert check['fails'] == ['edge_pair_angle']
    assert check['pair_angle_err_deg'] > 15.0


def test_bag2_selection_drops_border_hugging_left_edge():
    """bag2 三边：left 边贴左边框且 conf 最低 -> 丢弃；选 (bottom,right)。"""
    lines = PD.fit_edges(BAG2_KP, BAG2_KP_CONF, kconf=0.5, line_res_px=12.0)
    sel = _select_edge_pair(lines, BAG2_KP, BAG2_KP_CONF, 640, 480)
    assert sel['pair'] == [0, 1], sel
    assert sel['dropped'] == [3], sel
    assert sel['reason'] is None
    scores = {s['edge']: s for s in sel['scores'] if s is not None}
    assert scores[3]['score'] < min(scores[0]['score'], scores[1]['score']), scores
    assert scores[3]['border_dist_px'] < 24.0, scores


def test_bag2_selected_pair_is_rejected_by_pair_angle():
    """bag2 选出的 (bottom,right) 在 record D 下只有 73°，必须拒。"""
    lines = PD.fit_edges(BAG2_KP, BAG2_KP_CONF, kconf=0.5, line_res_px=12.0)
    sel = _select_edge_pair(lines, BAG2_KP, BAG2_KP_CONF, 640, 480)
    check = _check_edge_plane_consistency(
        lines, K, NORMAL, D_BAG2_RECORD, (W, H), pair=sel['pair'])
    assert check['checked'] is True
    assert check['passed'] is False
    assert check['pair'] == [0, 1]
    assert check['angle_pair_deg'] == pytest.approx(73.04, abs=0.5)
    assert check['fails'] == ['edge_pair_angle']
    # 第三条（left）与另两条矛盾，只记录不参与硬判定
    assert check['three_edge_diag']['available'] is True
    assert check['three_edge_contradicts'] is True
