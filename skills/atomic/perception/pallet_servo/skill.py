#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""托盘伺服误差技能：把框架的 `Pose6D` 换成矩阵，包成 `Result`。

数学全在 `.algorithm` 里，本文件只做四件事：

  1. **启动时**把配置错误判掉（`ref_edges`、台面尺寸、K、`T_cam_base` 的形状）
     —— 配错了每帧都会失败，早失败一句好过每帧失败一句（设计文档 §7）。
  2. 把 `Pose6D` 与 `BoxObservation` 换成 `.algorithm` 要的东西。
  3. 调 `servo_error`，**不做任何几何加工**——技能层自己算一笔的话，节点和
     回归测试就没有唯一的真相了。
  4. 把结果包成 `Result`；退化时 `Result.message` 就是 `str(Reject)`，
     里面带着实际值与阈值。

**本技能不接 `hardware` 参数**，也不自己查 TF：`T_cam_base` 由节点解析好传
进来（TF 那部分要 ROS，放在这里会让本技能没法脱离环境测试）。这样它可以
直接构造出来测。

运行与测试
----------
    # 技能层单测：不需要硬件、不需要 ROS
    python3 apps/test_kuavo_5w_skills/test_pallet_servo.py

    # 编排层节点单测：CI 会跑这条
    pytest orchestration/nodes/tests/test_node_pallet_servo.py -m unit -v
"""
import time
from dataclasses import dataclass
from typing import Any, Optional

import numpy as np

from core.common.logger import get_logger
from core.common.transform import pose6d_to_matrix
from core.domain.pose import Pose6D
from core.domain.result import Result
from core.domain.skill_params import SkillParams
from orchestration.utils.manifest_decorators import define_manifest
from skills.base.skill_base import SkillBase

from .algorithm import (
    BoxObservation,
    Reject,
    parse_pair_param,
    parse_ref_edges,
    servo_error,
)

logger = get_logger(__name__)


@dataclass
class PalletServoParams(SkillParams):
    """一次伺服误差计算的输入。

    pallet_pose:    托盘位姿（`Pose6D`，**约定按 base_link 解释**，见 README）。
    box_obs:        箱子观测。
    pallet_size_mm: 台面尺寸 `(W, H)`，毫米。来自标定产物的 `pallet_size_mm`。
    ref_edges:      两条参考边，第 1 个是底边、第 2 个是右边。
    K / D:          相机内参与畸变系数。`use_distortion=False` 时 D 被忽略。
    T_cam_base:     4×4，相机系 ← base 系。缺省单位阵（离线模拟就是单位阵）。
    image_size:     `(w, h)`，给了就顺带查"整条边跑到图外"；形状不对在
                    `on_initialize` 里就被拒（不是每帧失败一次）。
    frame_no:       这一帧的编号，**由调用方给**（节点传它自己那本账上的号）。
                    0 = 没给，技能自己数。节点每帧都调 `initialize`，技能自己数
                    会被反复清零 —— 旧的写法让技能每一条 WARNING 都写"第 1 帧"，
                    而节点那边写的是"第 N 帧"：同一个事件两个号。
    """

    skill_name: str = "pallet_servo"
    pallet_pose: Optional[Pose6D] = None
    box_obs: Optional[BoxObservation] = None
    pallet_size_mm: Any = None
    ref_edges: Any = ("y=0", "x=W")
    K: Any = None
    D: Any = None
    T_cam_base: Any = None
    image_size: Any = None
    use_distortion: bool = True
    frame_no: int = 0
    timeout: float = 5.0


def _matrix(value, shape, name: str):
    """把参数里给的嵌套 list 拧成 ndarray 并校验形状。失败返回错误说明。"""
    try:
        arr = np.asarray(value, np.float64)
    except (TypeError, ValueError) as exc:
        return None, f"{name} 不是数值矩阵：{exc}"
    if arr.shape != shape:
        return None, f"{name} 形状必须是 {shape}，实际 {arr.shape}"
    if not np.all(np.isfinite(arr)):
        return None, f"{name} 里有 NaN/inf"
    return arr, None


@define_manifest(
    label="托盘伺服误差",
    category=["skill", "perception", "pallet", "servo"],
    tree_type="studio_smoke",
    description="由托盘位姿与箱子观测算出图像空间的两个带符号垂距与角度差",
    params=[
        {"name": "ref_edges", "type": "strArr", "default": '["y=0", "x=W"]',
         "description": "两条参考边：第 1 个是底边、第 2 个是右边"},
        {"name": "pallet_size_mm", "type": "floatArr", "default": "[]",
         "description": "台面尺寸 [W, H] 毫米；空表示从标定产物读"},
        {"name": "K", "type": "floatArr", "default": "[]",
         "description": "相机内参 3x3"},
        {"name": "D", "type": "floatArr", "default": "[]",
         "description": "畸变系数，空表示无畸变"},
        {"name": "use_distortion", "type": "bool", "default": "true",
         "description": "投影要不要带畸变；必须与算像素的那一侧一致"},
    ],
    inputs=[{"name": "latest_pallet", "type": "object",
             "description": "托盘 Pose6D"},
            {"name": "latest_box_obs", "type": "object",
             "description": "箱子 BoxObservation"}],
    outputs=[{"name": "latest_servo_error", "type": "object",
              "description": "ServoError：三个量 + 诊断量"}],
)
class PalletServoSkill(SkillBase):
    """由托盘位姿与箱子观测算三个伺服误差量（纯计算，无硬件依赖）。"""

    def __init__(self, log_every_n: int = 30):
        super().__init__(name="pallet_servo")
        self.params: Optional[PalletServoParams] = None
        self._log_every_n = max(1, int(log_every_n))
        self._frame_no = 0
        self._done = False
        self._logged_init = False

    # ---------------------------------------------------------------- 启动
    def on_initialize(self, params: PalletServoParams) -> Result:
        if not isinstance(params, PalletServoParams):
            return Result.fail("Invalid parameters for PalletServoSkill")

        # 传尺寸是为了让绝对毫米写法的越界（`y=1000` 而台面只有 800）在
        # **技能初始化**时就死。尺寸此刻还没过下面的 `_check_size`，但
        # `parse_ref_edges` 自带 `size_mm_bad` 兜底 —— 畸形值在这里也是
        # `Reject` 不是异常，顺序无所谓。
        parsed = parse_ref_edges(params.ref_edges, params.pallet_size_mm)
        if isinstance(parsed, Reject):
            return Result.fail(f"配置错误 —— {parsed}")

        size, err = self._check_size(params.pallet_size_mm)
        if err:
            return Result.fail(f"配置错误 —— {err}")

        # `image_size` 是唯一一个"元组形状"的参数，也在这里查掉：不查的话坏值
        # 活到 `on_execute` 里（`ref_edge_px` 取 `image_size[1]` 抛 IndexError），
        # 被 `SkillBase.execute` 的宽接接住 → **当成"这一帧被拒"**：每帧一条
        # 看着像几何退化的 WARNING，外带每帧一份 dump。配置错误该在启动时说。
        image_size, err = parse_pair_param(params.image_size, "image_size")
        if err:
            return Result.fail(f"配置错误 —— {err}")

        K, err = _matrix(params.K, (3, 3), "K")
        if err:
            return Result.fail(f"配置错误 —— {err}")

        T_cam_base = np.eye(4)
        if params.T_cam_base is not None:
            T_cam_base, err = _matrix(params.T_cam_base, (4, 4), "T_cam_base")
            if err:
                return Result.fail(f"配置错误 —— {err}")

        D = None
        if params.use_distortion and params.D is not None:
            try:
                D = np.asarray(params.D, np.float64).reshape(-1)
            except (TypeError, ValueError) as exc:
                return Result.fail(f"配置错误 —— D 不是数值：{exc}")
            if D.size == 0:
                D = None

        self.params = params
        self._size = size
        self._K = K
        self._D = D
        self._T_cam_base = T_cam_base
        self._image_size = image_size
        self._done = False
        # `self._frame_no` **故意不在这里清零**：本技能每帧都要 `initialize`
        # （每帧的位姿与观测都不一样），而节点传进来的 `frame_no` 才是权威编号
        # （见 `on_execute`）；没有调用方给号时才退回到自己数，那时更不该清零
        # —— 清零会让技能层自己的节流日志（每 `log_every_n` 帧一条 DEBUG）
        # 永远打在"第 1 帧"上，等于没打。

        # 设计文档 §6.2「初始化 INFO」：把这一层决定口径的东西全打出来，
        # 现场出问题时第一眼看的就是它 —— 尤其是 K 与 ref_edges 有没有和
        # 离线点点时用的一致。
        #
        # **只打第一次**：`initialize` 是每帧都要调的，每次都打就是刷屏，
        # 把有用的日志冲掉（框架的 `Skill [...] initialized.` 每帧一条，
        # 那是框架的事，这里自己别再添一条）。
        if not self._logged_init:
            self._logged_init = True
            logger.info(
                "PalletServoSkill 初始化：pallet_size_mm=%s ref_edges=%s "
                "use_distortion=%s D=%s K=%s image_size=%s",
                size, parsed, params.use_distortion,
                None if D is None else np.round(D, 6).tolist(),
                np.round(K, 3).tolist(), image_size)
        return Result.ok()

    @staticmethod
    def _check_size(value):
        """台面尺寸必须是两个正数。

        用 `parse_pair_param` 而不是 `float(value[0]), float(value[1])`：后者对
        **字符串**不报错而是静默算错（`"12"` → `(1.0, 2.0)`，还正好通过"都为正"），
        对多一个元素的列表则无声忽略多出来的那个。
        """
        if value is None:
            # **参数那条排第一**（与 `NodePalletServo.initialise()` 同一份文案口径）：
            # 新链路走 `pallet_size_mm` rosparam，**不需要** apriltag 标定 ——
            # 旧文案指路的 `config/pallet_tag.yaml` 是旧产物、仓库里没有它。
            return None, ("没有 pallet_size_mm —— **在节点参数里给 [W, H]**"
                          "（毫米，例如 [1200, 1000]）即可。**新链路不需要 "
                          "apriltag 标定**：`config/pallet_tag.yaml` 是旧产物、"
                          "已废弃")
        size, err = parse_pair_param(value, "pallet_size_mm")
        if err:
            return None, err
        if size is None:
            # 空列表 / 空串：`parse_pair_param` 把"没给"和"给了个空"当同一件事，
            # 但在这里两者都不是"没给"—— 空尺寸投影不出东西，必须报出来
            return None, (f"pallet_size_mm 必须是 [W, H] 两个数，实际 {value!r}"
                          f"（空值）")
        return size, None

    # ---------------------------------------------------------------- 每帧
    def on_execute(self) -> Result:
        if self._done:
            return Result.ok("PalletServoSkill already finished")
        if self.params.pallet_pose is None:
            return Result.fail("这一帧没有托盘位姿（黑板上的 latest_pallet 是空的）")
        if self.params.box_obs is None:
            return Result.fail("这一帧没有箱子观测（黑板上的 latest_box_obs 是空的）")

        T_base_pallet = pose6d_to_matrix(self.params.pallet_pose)
        T_cam_pallet = self._T_cam_base @ T_base_pallet

        t0 = time.perf_counter()
        out = servo_error(T_cam_pallet, self._size, self.params.ref_edges,
                          self.params.box_obs, self._K, self._D,
                          self._image_size)
        elapsed_ms = (time.perf_counter() - t0) * 1000.0

        # 帧号：**调用方给的优先**。节点每帧都调 `initialize`，技能自己数会被
        # 反复清零 —— 旧的写法让这里的每一条 WARNING 都写"第 1 帧"，而节点那条
        # 写的是"第 N 帧"：同一个事件两个号，两条都落盘。没给（0）时自己数。
        given = self._given_frame_no()
        self._frame_no = given if given > 0 else self._frame_no + 1
        if isinstance(out, Reject):
            # 拒绝帧：逐帧 WARNING，且必须带实际值与阈值（设计文档 §6.2）
            logger.warning("托盘伺服本帧被拒（第 %d 帧）：%s", self._frame_no, out)
            return Result.fail(str(out))
        if out.warn:
            logger.warning("托盘伺服本帧有告警（第 %d 帧）：%s",
                           self._frame_no, "; ".join(out.warn))
        if self._frame_no % self._log_every_n == 0:
            logger.debug("托盘伺服第 %d 帧：%s（T_cam_base 用时 %.3f ms）",
                         self._frame_no, out.to_log_line(), elapsed_ms)

        self._done = True
        return Result.ok(out.to_log_line(), data={"error": out})

    def _given_frame_no(self) -> int:
        """`params.frame_no` 转 int；非数字当"没给"（0）—— **不抛**。

        这个值只影响日志行里那个编号，为它把整帧变成异常（进而被当成"这一帧
        被拒"）不值得。
        """
        try:
            return int(self.params.frame_no)
        except (TypeError, ValueError):
            return 0

    def on_is_finished(self) -> bool:
        return self._done
