#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""托盘位姿技能：由 tag 位姿反推木托盘位姿。

输入是若干 tag 的位姿（来自黑板 `latest_tag_<id>`，由 NodePercep 写入），
输出是一个托盘位姿。数学全在 `.algorithm` 里，本文件只做三件事：把框架的
`Pose6D` 换成 4×4 矩阵、可选地过一遍时间窗、把结果包成 `Result`。

**本技能不接 `hardware` 参数**，因为它没有任何硬件调用 —— 输入是已经算好的
tag 位姿，输出是一个纯计算的结果。这样它不依赖 ROS、不依赖相机、也不依赖
`get_shared_hardware()`，可以直接构造出来测。

时间窗与延迟
------------
`window=0`（默认）不平滑，每帧独立输出，无延迟。开窗时用与 maduo 完全相同的
`smooth_series`，但要注意两种窗口的**在线代价**不同：

  - `mode="trailing"` —— 因果窗，只看过去，无延迟，但输出会**滞后**于真实
    运动。这套采集里运动/噪声比只有 0.03~0.05，滞后很小，所以在线上是安全
    的选择。
  - `mode="savgol"` —— 居中窗，**需要未来帧**，因此输出会滞后 `window//2`
    帧。这个配置（w=15、阶=1）是操作员在 maduo 里逐帧看图选定的，精度最好，
    代价是 7 帧的延迟。30 fps 下约 0.23 秒，通常可以接受；对延迟敏感的场景
    用 `trailing`。

无论哪种窗都压不掉**慢变**的系统误差 —— 这套系统里占大头的是随视角缓慢变化
的那种，只有换观测手段才能动它，这一点在 maduo 的消融里已经量化过。

运行与测试
----------
    # 技能层单测：不需要硬件、不需要 ROS
    python3 apps/test_kuavo_5w_skills/test_pallet_pose.py

    # 编排层节点单测：CI 会跑这条
    pytest orchestration/nodes/tests/test_node_pallet_pose.py -m unit -v

    # 手工试一次（会打印算出来的托盘位姿）
    python3 -c "
    from skills.atomic.perception.pallet_pose.skill import PalletPoseSkill, PalletPoseParams
    from core.domain.pose import Pose6D
    import numpy as np
    T = np.eye(4); T[0, 3] = 0.5
    s = PalletPoseSkill()
    s.initialize(PalletPoseParams(tag_poses={1: Pose6D(x=0, y=0, z=1)},
                                  pallet_tag={1: T.tolist()}))
    r = s.execute(); print(r.success, r.message, r.data['pose'].to_list())
    "
"""
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import numpy as np

from core.common.logger import get_logger
from core.common.transform import matrix_to_pose6d, pose6d_to_matrix
from core.domain.pose import Pose6D
from core.domain.result import Result
from core.domain.skill_params import SkillParams
from orchestration.utils.manifest_decorators import define_manifest
from skills.base.skill_base import SkillBase

from .algorithm import DEFAULT_WINDOW, pallet_pose_from_tags, smooth_series

logger = get_logger(__name__)


@dataclass
class PalletPoseParams(SkillParams):
    """一次托盘位姿估计的输入。

    tag_poses:  本帧可用的 tag 位姿，`{tag_id: Pose6D}`。参考系随意 ——
                `T_pallet_tag` 是两个物体间的相对位姿，进来什么系出去什么系。
    pallet_tag: 标定产出的 `{tag_id: 4×4}`（pallet ← tag），来自
                `config/pallet_tag.yaml`。没标过的 tag 会被跳过。
    """

    skill_name: str = "pallet_pose"
    tag_poses: Dict[int, Pose6D] = field(default_factory=dict)
    pallet_tag: Dict[int, Any] = field(default_factory=dict)
    timeout: float = 5.0


@define_manifest(
    label="托盘位姿（二维码反识别）",
    category=["skill", "perception", "pallet"],
    tree_type="studio_smoke",
    description="由若干 AprilTag 的位姿反推木托盘位姿；两个 tag 各自推一个再融合",
    params=[
        {"name": "tag_ids", "type": "intArr", "default": "[]",
         "description": "参与反识别的 tag id，空表示全部"},
        {"name": "window", "type": "int", "default": "0",
         "description": "时间窗宽度，0 表示不平滑"},
    ],
    inputs=[{"name": "latest_tag_<id>", "type": "object",
             "description": "NodePercep 写入的 TagDetection"}],
    outputs=[{"name": "latest_pallet_<name>", "type": "object",
              "description": "反推出来的托盘 Pose6D"}],
)
class PalletPoseSkill(SkillBase):
    """由 tag 位姿反推托盘位姿（纯计算，无硬件依赖）。"""

    def __init__(self, window: int = 0, smooth_mode: str = "trailing"):
        super().__init__(name="pallet_pose")
        self.params: Optional[PalletPoseParams] = None
        self._window = int(window)
        self._mode = smooth_mode
        self._history: List[Optional[np.ndarray]] = []
        self._done = False

    def on_initialize(self, params: PalletPoseParams) -> Result:
        if not isinstance(params, PalletPoseParams):
            return Result.fail("Invalid parameters for PalletPoseSkill")
        if not params.pallet_tag:
            return Result.fail(
                "pallet_tag 为空 —— 还没有标定过。先跑 "
                "apps/test_camera_internal/pallet_calibration/pallet_calibrate.py，"
                "生成 config/pallet_tag.yaml")
        self.params = params
        self._done = False
        return Result.ok()

    def on_execute(self) -> Result:
        if self._done:
            return Result.ok("PalletPoseSkill already finished")

        # 标定结果可能是从 YAML 读进来的嵌套 list，统一成 ndarray
        table = {int(mid): np.asarray(T, np.float64)
                 for mid, T in self.params.pallet_tag.items()}
        tag_mats = {int(mid): pose6d_to_matrix(p)
                    for mid, p in self.params.tag_poses.items()
                    if int(mid) in table}
        if not tag_mats:
            return Result.fail(
                "本帧没有任何已标定的 tag 可见 —— 无法反推托盘位姿")

        pose = pallet_pose_from_tags(tag_mats, table)
        if pose is None:
            return Result.fail("托盘位姿解算失败（tag 位姿退化）")

        used = sorted(tag_mats)
        if self._window > 0:
            # 有窗时把这一帧接到历史上再平滑；返回的是"当前能确定的那一帧"，
            # 居中窗下就是滞后 window//2 帧的位置。
            self._history.append(pose)
            keep = self._window + 2
            if len(self._history) > keep:
                del self._history[:-keep]
            series = smooth_series(self._history, self._mode, self._window, 1)
            pose = series[-1] if series and series[-1] is not None else pose

        self._done = True
        return Result.ok(
            f"pallet pose from {len(used)} tag(s): {used}",
            data={"pose": matrix_to_pose6d(pose),
                  "tag_ids": used,
                  "smoothed": self._window > 0},
        )

    def on_is_finished(self) -> bool:
        return self._done
