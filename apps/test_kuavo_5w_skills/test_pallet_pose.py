#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""托盘位姿：算法层回归 + 技能层单测（无硬件、无 ROS、无相机）。

分两部分：

  **回归** —— `pallet_pose/algorithm.py` 是从 maduo 项目搬过来的。搬算法最容易
  出的错是静默的：乘法顺序反了、坐标系搞错，输出依然是"看起来合理的位姿"，
  只是偏了。所以这里拿那边导出的**同一份输入**重算一遍，和那边的**同一份输出**
  逐位比对。fixture 由 `maduo/export_regression_fixture.py` 生成，是 april_test7
  标定那 5 帧的 T_cam_pallet / T_cam_tag 以及那边算出来的 T_pallet_tag。

  **技能层** —— 用同一份数据驱动 `PalletPoseSkill`，覆盖两个 tag、一个 tag、
  一个都没有三种情况。

运行：
    python3 apps/test_kuavo_5w_skills/test_pallet_pose.py

退出码 0 通过、1 失败（与仓库其余测试脚本一致）。不需要 source ROS 环境，
因为这个技能本来就不碰硬件。
"""
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from core.common.transform import matrix_to_pose6d        # noqa: E402
from core.domain.pose import Pose6D                      # noqa: E402
from skills.atomic.perception.pallet_pose.algorithm import (  # noqa: E402
    TagObs,
    calibrate_pallet_tag,
    pallet_pose_from_tags,
    smooth_series,
)
from skills.atomic.perception.pallet_pose.skill import (  # noqa: E402
    PalletPoseParams,
    PalletPoseSkill,
)

FIXTURE = Path(__file__).resolve().parent / "pallet_pose_regression.json"
TOL = 1e-9


# --------------------------------------------------------------------------- #
# 算法层回归
# --------------------------------------------------------------------------- #
def load_fixture():
    if not FIXTURE.exists():
        raise SystemExit(
            f"缺少回归数据 {FIXTURE}\n"
            f"在 maduo 项目里生成它：\n"
            f"  python3 export_regression_fixture.py --sequence april_test7 "
            f"--out {FIXTURE}")
    return json.loads(FIXTURE.read_text(encoding="utf-8"))


def observations_from(fixture):
    """把 fixture 的帧展开成 TagObs 列表。"""
    obs = []
    for entry in fixture["frames"]:
        T_cam_pallet = np.array(entry["T_cam_pallet"], np.float64)
        for mid, T_cam_tag in entry["T_cam_tag"].items():
            obs.append(TagObs(tag_id=int(mid),
                              T_sensor_tag=np.array(T_cam_tag, np.float64),
                              T_sensor_pallet=T_cam_pallet,
                              frame=entry["stem"],
                              pallet_size_mm=entry.get("pallet_wh_mm")))
    return obs


def test_calibration_matches_maduo(fixture):
    """标定结果必须和 maduo 侧逐位一致。"""
    result = calibrate_pallet_tag(observations_from(fixture))
    expected = {int(k): np.array(v, np.float64)
                for k, v in fixture["expected_T_pallet_tag_by_marker"].items()}

    assert set(result.by_tag) == set(expected), \
        f"tag 集合不一致: 算出 {sorted(result.by_tag)}，期望 {sorted(expected)}"
    for mid, want in expected.items():
        got = result.by_tag[mid]
        assert np.allclose(got, want, atol=TOL), (
            f"tag {mid} 的 T_pallet_tag 与 maduo 不一致，最大差 "
            f"{np.abs(got - want).max():.3e}")
    print(f"    标定回归通过：{len(expected)} 个 tag，"
          f"{result.n_estimates} 个估计，与 maduo 逐位一致")


def test_self_check_reports_repeatability(fixture):
    """自检要给出重复性和跨 tag 一致性，且数值合理。"""
    check = calibrate_pallet_tag(observations_from(fixture)).self_check
    assert check["n_frames"] == len(fixture["frames"])
    assert check["n_tags"] == len(fixture["expected_T_pallet_tag_by_marker"])
    # april_test7 实测 pooled 位置散布约 17 mm、旋转约 0.9 度
    assert check["pooled_pos_sd_mm"] < 50.0, \
        f"重复性太差: {check['pooled_pos_sd_mm']:.1f} mm"
    assert check["cross_tag"]["n"] > 0, "没有算出跨 tag 一致性"
    print(f"    自检：pooled 位置 sd {check['pooled_pos_sd_mm']:.1f} mm、"
          f"旋转 {check['pooled_rot_deg']:.2f} 度；"
          f"跨 tag 一致性 {check['cross_tag']['mean_mm']:.1f} mm")


def test_calibration_aggregates_pallet_size(fixture):
    """标定要顺带把各帧点出来的台面尺寸聚合出来（中位数 + 散布）。

    伺服的投影需要 W x H，而标定产物原来没有这个键。尺寸**完全由标定时点的
    四个角自动算出**，不引入新的手工输入。
    """
    obs = observations_from(fixture)      # 已经带上 pallet_wh_mm，见下一步
    result = calibrate_pallet_tag(obs)
    assert result.pallet_size_mm is not None, "没算出 pallet_size_mm"
    want_w = sorted(e["pallet_wh_mm"][0] for e in fixture["frames"])
    want_h = sorted(e["pallet_wh_mm"][1] for e in fixture["frames"])
    median_w = want_w[len(want_w) // 2]
    median_h = want_h[len(want_h) // 2]
    assert abs(result.pallet_size_mm[0] - median_w) <= 1, result.pallet_size_mm
    assert abs(result.pallet_size_mm[1] - median_h) <= 1, result.pallet_size_mm
    spread = result.self_check["pallet_size_spread_mm"]
    # **按帧数**，不是按观测数：fixture 是 5 帧 x 2 tag = 10 个观测，但点击只有
    # 5 次。这条断言就是守"聚合按帧去重"这条规则的。
    assert spread["n"] == len(fixture["frames"]), spread
    print(f"    台面尺寸：中位数 {result.pallet_size_mm} mm，"
          f"W 散布 {spread['w_sd_mm']:.1f} mm、H 散布 {spread['h_sd_mm']:.1f} mm")


def test_runtime_roundtrip(fixture):
    """反识别的乘法顺序必须精确可逆（用单帧单 tag 的估计验）。

    标定方向是 `T_pallet_tag = inv(T_sensor_pallet) @ T_sensor_tag`，反识别是
    `T_sensor_tag @ inv(T_pallet_tag)`。拿同一帧同一个 tag 的估计走一个来回，
    必须**精确**回到该帧的 `T_sensor_pallet` —— 这一步只验乘法顺序，容差按
    浮点水平给。

    注意不能用"平均后的 by_tag"来做这个检查：那是多帧多 tag 的平均解，而
    每一帧的估计彼此差着标定的重复性散布（april_test7 上约 17 mm）。用它反推
    只会回到平均解附近，差个几十毫米是**正常**的，不是错。
    """
    worst = 0.0
    for entry in fixture["frames"]:
        T_pallet = np.array(entry["T_cam_pallet"], np.float64)
        for _mid, T_tag in entry["T_cam_tag"].items():
            T_tag = np.array(T_tag, np.float64)
            T_pallet_tag = np.linalg.inv(T_pallet) @ T_tag
            back = pallet_pose_from_tags({0: T_tag}, {0: T_pallet_tag})
            worst = max(worst, float(np.abs(back - T_pallet).max()))
    assert worst < TOL, f"乘法顺序不精确可逆，最大差 {worst:.3e}"
    print(f"    乘法顺序：{len(fixture['frames'])} 帧往返最大残差 {worst:.2e}")


def test_fused_pose_sits_near_the_calibration_frames(fixture):
    """用真实路径（平均后的标定）反推，结果应落在各帧答案的散布之内。

    这条量的是"标定平均"本身引入的偏差：它不该比标定的重复性散布大一个量级，
    否则说明平均这一步有问题。april_test7 的 pooled 位置 sd 约 17 mm，所以这里
    按 100 mm 放宽 —— 只拦"大得离谱"，不假装能比自检更精确。
    """
    result = calibrate_pallet_tag(observations_from(fixture))
    worst = 0.0
    for entry in fixture["frames"]:
        tag_poses = {int(m): np.array(T, np.float64)
                     for m, T in entry["T_cam_tag"].items()}
        got = pallet_pose_from_tags(tag_poses, result.by_tag)
        want = np.array(entry["T_cam_pallet"], np.float64)
        worst = max(worst, float(np.linalg.norm(got[:3, 3] - want[:3, 3]) * 1000.0))
    assert worst < 100.0, \
        f"平均标定反推偏离标定帧 {worst:.1f} mm，超过重复性散布一个量级"
    print(f"    平均标定反推：偏离标定帧最大 {worst:.1f} mm "
          f"(重复性 sd {result.self_check['pooled_pos_sd_mm']:.1f} mm)")


def test_smooth_series_keeps_rigid_and_none():
    """时间窗：输出仍是合法刚体变换，且输入里的 None 原样跳过。"""
    rng = np.random.RandomState(0)
    poses = []
    for i in range(30):
        T = np.eye(4)
        T[:3, 3] = [0.001 * i, 0.0, 0.5]
        poses.append(T)
    poses[10] = None

    for mode, width, order in (("trailing", 15, 0), ("savgol", 15, 1)):
        out = smooth_series(poses, mode, width, order)
        assert len(out) == len(poses)
        for T in out:
            if T is None:
                continue
            R = T[:3, :3]
            assert np.allclose(R.T @ R, np.eye(3), atol=1e-9), \
                f"{mode}: 输出不是正交阵"
            assert abs(np.linalg.det(R) - 1.0) < 1e-9, f"{mode}: 输出不是旋转"
        # 平滑后的轨迹噪声应当远小于输入的阶跃（这里输入本就平滑，只查形状）
    # None 只可能落在没有可用邻居的窗口上；这里有邻居，所以应当被填上
    assert smooth_series(poses, "savgol", 15, 1)[10] is not None
    print("    时间窗：输出保持刚体，None 处理正确")


# --------------------------------------------------------------------------- #
# 技能层
# --------------------------------------------------------------------------- #
def test_skill_two_tags_one_tag_none(fixture):
    """技能层三种情况：两个 tag / 一个 tag / 一个都没有。"""
    expected = {int(k): np.array(v, np.float64)
                for k, v in fixture["expected_T_pallet_tag_by_marker"].items()}
    entry = fixture["frames"][0]
    table = {mid: T.tolist() for mid, T in expected.items()}
    # 位姿要从矩阵转，才和标定时的输入是同一个东西（平移单独搬会丢掉姿态）
    poses = {int(m): matrix_to_pose6d(np.array(T, np.float64))
             for m, T in entry["T_cam_tag"].items()}

    # 两个 tag
    skill = PalletPoseSkill()
    init = skill.initialize(PalletPoseParams(tag_poses=poses, pallet_tag=table))
    assert init.success, init.message
    result = skill.execute()
    assert result.success, result.message
    assert result.data["tag_ids"] == sorted(poses), result.data
    two_tags = np.array(result.data["pose"].to_list()[:3])

    # 一个 tag
    one = {sorted(poses)[0]: poses[sorted(poses)[0]]}
    skill = PalletPoseSkill()
    assert skill.initialize(PalletPoseParams(tag_poses=one, pallet_tag=table)).success
    result = skill.execute()
    assert result.success, result.message
    assert result.data["tag_ids"] == [sorted(poses)[0]]
    one_tag = np.array(result.data["pose"].to_list()[:3])

    # 一个都没有
    skill = PalletPoseSkill()
    assert skill.initialize(PalletPoseParams(tag_poses={}, pallet_tag=table)).success
    result = skill.execute()
    assert not result.success, "没有 tag 时不该成功"
    assert "没有" in result.message

    print(f"    技能层：两 tag {np.round(two_tags, 3)}，"
          f"单 tag {np.round(one_tag, 3)}，无 tag 正确失败")


def test_skill_rejects_missing_calibration():
    """没给标定结果时，initialize 就该失败并提示去跑标定工具。"""
    skill = PalletPoseSkill()
    result = skill.initialize(PalletPoseParams(tag_poses={1: Pose6D()}, pallet_tag={}))
    assert not result.success
    assert "pallet_calibrate" in result.message
    print("    技能层：缺少标定时给出可操作的提示")


# --------------------------------------------------------------------------- #
# runner
# --------------------------------------------------------------------------- #
def main() -> int:
    fixture = load_fixture()
    print(f"回归数据 {FIXTURE.name}：{fixture['sequence']}，"
          f"{len(fixture['frames'])} 帧，dictionary={fixture['dictionary']}，"
          f"marker_size={fixture['marker_size_m']} m")
    print()

    cases = [
        ("标定与 maduo 逐位一致", lambda: test_calibration_matches_maduo(fixture)),
        ("标定自检", lambda: test_self_check_reports_repeatability(fixture)),
        ("标定聚合台面尺寸", lambda: test_calibration_aggregates_pallet_size(fixture)),
        ("乘法顺序精确可逆", lambda: test_runtime_roundtrip(fixture)),
        ("平均标定反推", lambda: test_fused_pose_sits_near_the_calibration_frames(fixture)),
        ("时间窗", lambda: test_smooth_series_keeps_rigid_and_none()),
        ("技能层三种 tag 情况", lambda: test_skill_two_tags_one_tag_none(fixture)),
        ("技能层缺标定", lambda: test_skill_rejects_missing_calibration()),
    ]
    failed = 0
    for title, fn in cases:
        try:
            fn()
        except AssertionError as exc:
            failed += 1
            print(f"  FAIL  {title}: {exc}")
        except Exception as exc:                       # noqa: BLE001
            failed += 1
            print(f"  ERROR {title}: {type(exc).__name__}: {exc}")

    print()
    if failed:
        print(f"FAILED: {failed}/{len(cases)}")
        return 1
    print(f"OK: {len(cases)}/{len(cases)} 全部通过")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
