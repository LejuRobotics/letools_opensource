# -*- coding: utf-8 -*-
"""托盘位姿的**因果**卡尔曼滤波（不 import ROS，可直接单测）。

放在 `NodePalletObs` 的 ROS 回调里，按**检测器原始帧率**吃观测；它只输出当前
采样时刻的状态，不做固定延迟的平滑，因此不会像 N 帧滑窗均值那样额外压半窗。

状态（12 维误差状态）：

    [p(3)  v(3)  θ(3)  ω(3)]

* `p` 是相机系平移（米），`v` 是平移速度；
* `R` 是名义旋转（相机系 ← 托盘系），`θ` 是作用在 **R 左侧**的小角度误差
  （`R_true = Exp(θ) · R`），`ω` 是相机系角速度。
* 预测是常速度模型，没有加速度项；过程噪声按“未知加速度 / 角加速度”给出。

## 为什么在检测回调里滤，而不在伺服窗里

伺服节点在约 10 Hz 的 tick 上 `resolve()` 成对观测；把 5 帧平滑放到那里，
5 帧 ≈ 0.5 s。检测回调通常是 20~30 Hz，5 帧 ≈ 0.17~0.25 s，且每一帧都参与
滤波、只把最新结果交给黑板。这样既有 5 帧的抑制能力，又尽量少滞后。

## 跳变处理

检测器现在会在两个局部极小值之间来回跳。单纯调小过程噪声只能把跳变“拖慢”，
还会在真运动时滞后。这里加了三道闸：

1. 单帧跳变超过 `max_pos_jump_m` / `max_rot_jump_deg` 时**不采纳**这一帧，
   只按常速度模型外推并返回预测值；
2. 连续 `window` 帧都超阈值，说明目标真的换了状态（不是单帧毛刺），
   这时**重置**到当前观测，避免永远追不上；
3. 时间戳回退或间隔超过 `max_gap_s` 时重置（检测器重启、bag 回跳、丢帧）。

`window` 同时是“连续多少帧异常才重置”的默认值；默认 5 帧。
"""
from __future__ import annotations

import math

import numpy as np
from scipy.spatial.transform import Rotation

# 观测/过程噪声默认值。位置单位 m，角度单位 rad。
DEFAULT_SIGMA_POS_M = 0.020
DEFAULT_SIGMA_ROT_DEG = 2.0
DEFAULT_ACCEL_MPS2 = 3.0
DEFAULT_ANG_ACCEL_RAD_S2 = 20.0
DEFAULT_MAX_POS_JUMP_M = 0.60
DEFAULT_MAX_ROT_JUMP_DEG = 25.0
DEFAULT_MAX_GAP_S = 0.50
DEFAULT_WINDOW = 5

_EPS = 1e-9


def _as_rotation(T: np.ndarray) -> Rotation:
    """4×4 的旋转块 → `Rotation`；非正交/退化时抛 `ValueError`。"""
    R = np.asarray(T, np.float64)[:3, :3]
    if not np.all(np.isfinite(R)):
        raise ValueError("旋转块不是有限数")
    try:
        return Rotation.from_matrix(R)
    except ValueError as exc:
        raise ValueError(f"旋转块不能转成旋转：{exc}") from exc


def _make_T(R: Rotation, p: np.ndarray) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R.as_matrix()
    T[:3, 3] = np.asarray(p, np.float64)
    return T


class PoseKalmanFilter:
    """常速度误差状态卡尔曼滤波。

    `update(T, stamp)` 返回 4×4 的滤波后位姿。第一次观测原样返回；之后遇到
    单帧跳变时返回预测值（并在连续 `window` 帧后重置到新观测）。

    线程模型：调用方应在**单一线程**里喂观测（`NodePalletObs._on_message`
    的 ROS 回调就是单线程）；本类自己不加锁。
    """

    def __init__(self, *, window: int = DEFAULT_WINDOW,
                 sigma_pos_m: float = DEFAULT_SIGMA_POS_M,
                 sigma_rot_deg: float = DEFAULT_SIGMA_ROT_DEG,
                 accel_mps2: float = DEFAULT_ACCEL_MPS2,
                 ang_accel_rad_s2: float = DEFAULT_ANG_ACCEL_RAD_S2,
                 max_pos_jump_m: float = DEFAULT_MAX_POS_JUMP_M,
                 max_rot_jump_deg: float = DEFAULT_MAX_ROT_JUMP_DEG,
                 max_gap_s: float = DEFAULT_MAX_GAP_S) -> None:
        self._window = max(1, int(window))
        self._sigma_p = max(float(sigma_pos_m), 1e-6)
        self._sigma_rot = math.radians(max(float(sigma_rot_deg), 1e-6))
        # `window` 越大越平滑：把过程噪声按 5/window 缩放。5 是默认工作点；
        # window=1 时接近“只做跳变闸、少平滑”，window=10 时更信模型。
        win_scale = DEFAULT_WINDOW / float(self._window)
        self._accel = max(float(accel_mps2) * win_scale, 1e-6)
        self._ang_accel = max(float(ang_accel_rad_s2) * win_scale, 1e-6)
        self._max_pos_jump = max(float(max_pos_jump_m), 1e-6)
        self._max_rot_jump = math.radians(max(float(max_rot_jump_deg), 1e-6))
        self._max_gap = max(float(max_gap_s), 1e-6)

        self._p = np.zeros(3)
        self._v = np.zeros(3)
        self._R = Rotation.identity()
        self._w = np.zeros(3)
        self._P = np.zeros((12, 12))
        self._stamp = None
        self._bad = 0
        self._last_T = None
        self._initialized = False

    # ------------------------------------------------------------------ 状态
    @property
    def initialized(self) -> bool:
        return self._initialized

    @property
    def bad_frames(self) -> int:
        """当前连续被跳变闸挡下的帧数（诊断用）。"""
        return self._bad

    def reset(self) -> None:
        """清空状态；下一帧作为新目标的第一帧原样输出。"""
        self._p = np.zeros(3)
        self._v = np.zeros(3)
        self._R = Rotation.identity()
        self._w = np.zeros(3)
        self._P = np.zeros((12, 12))
        self._stamp = None
        self._bad = 0
        self._last_T = None
        self._initialized = False

    def _init_measurement(self, T: np.ndarray, stamp: float) -> np.ndarray:
        T = np.asarray(T, np.float64)
        self._p = T[:3, 3].copy()
        self._v = np.zeros(3)
        self._R = _as_rotation(T)
        self._w = np.zeros(3)
        # 初值协方差：位置/姿态来自测量；速度/角速度完全不知道。
        self._P = np.diag([
            self._sigma_p ** 2, self._sigma_p ** 2, self._sigma_p ** 2,
            0.50 ** 2, 0.50 ** 2, 0.50 ** 2,
            self._sigma_rot ** 2, self._sigma_rot ** 2, self._sigma_rot ** 2,
            2.0 ** 2, 2.0 ** 2, 2.0 ** 2,
        ]).astype(np.float64)
        self._stamp = float(stamp)
        self._bad = 0
        self._initialized = True
        out = _make_T(self._R, self._p)
        self._last_T = out
        return out

    def _predict(self, dt: float) -> np.ndarray:
        # 名义状态：常速度。
        self._p = self._p + self._v * dt
        self._R = Rotation.from_rotvec(self._w * dt) * self._R
        # 误差状态：δp' = δp + δv dt，δθ' = δθ + δω dt。
        F = np.eye(12)
        F[0:3, 3:6] = np.eye(3) * dt
        F[6:9, 9:12] = np.eye(3) * dt
        # 连续白噪声加速度模型；比离散“每帧独立”更稳定。
        qp = (self._accel * dt * dt / 2.0) ** 2
        qv = (self._accel * dt) ** 2
        qr = (self._ang_accel * dt * dt / 2.0) ** 2
        qw = (self._ang_accel * dt) ** 2
        Q = np.diag([qp] * 3 + [qv] * 3 + [qr] * 3 + [qw] * 3)
        self._P = F @ self._P @ F.T + Q
        return np.zeros(6)

    def update(self, T: np.ndarray, stamp: float) -> np.ndarray:
        """喂一帧观测，返回滤波后的 4×4 位姿。

        非有限矩阵抛 `ValueError`；时间戳回退/大间隔时自动重置。
        跳变帧不会抛异常，按类文档的三道闸处理。
        """
        T = np.asarray(T, np.float64)
        if T.shape != (4, 4) or not np.all(np.isfinite(T)):
            raise ValueError("位姿必须是有限的 4×4 矩阵")
        stamp = float(stamp)
        if not math.isfinite(stamp):
            raise ValueError("stamp 不是有限数")

        if not self._initialized or self._stamp is None:
            return self._init_measurement(T, stamp)

        dt = stamp - self._stamp
        if dt < -_EPS:
            # 小幅回退多半是 ROS 回调乱序：忽略这一帧，不要因此重置；
            # 大幅回退（bag 循环 / 换时间源）才重新捕获。
            if -dt > self._max_gap:
                return self._init_measurement(T, stamp)
            assert self._last_T is not None
            return self._last_T.copy()
        if abs(dt) <= _EPS:
            # 同 stamp 的重复消息不重复更新。
            assert self._last_T is not None
            return self._last_T.copy()
        if dt > self._max_gap:
            return self._init_measurement(T, stamp)

        R_meas = _as_rotation(T)
        p_meas = T[:3, 3]

        # ---- 预测 ------------------------------------------------------------
        self._predict(dt)

        # ---- 新息 ------------------------------------------------------------
        dp = p_meas - self._p
        dR = R_meas * self._R.inv()
        dtheta = dR.as_rotvec()
        pos_jump = float(np.linalg.norm(dp))
        rot_jump = float(np.linalg.norm(dtheta))
        if (pos_jump > self._max_pos_jump
                or rot_jump > self._max_rot_jump):
            self._bad += 1
            self._stamp = stamp
            if self._bad >= self._window:
                # 连续 window 帧都异常：目标真的换了状态，重新捕获。
                return self._init_measurement(T, stamp)
            out = _make_T(self._R, self._p)
            self._last_T = out
            return out
        self._bad = 0

        # ---- 更新 ------------------------------------------------------------
        H = np.zeros((6, 12))
        H[0:3, 0:3] = np.eye(3)
        H[3:6, 6:9] = np.eye(3)
        Rn = np.diag([self._sigma_p ** 2] * 3
                     + [self._sigma_rot ** 2] * 3).astype(np.float64)
        S = H @ self._P @ H.T + Rn
        K = np.linalg.solve(S, H @ self._P).T          # P H^T S^{-1}
        dx = K @ np.concatenate([dp, dtheta])
        self._p = self._p + dx[0:3]
        self._v = self._v + dx[3:6]
        self._R = Rotation.from_rotvec(dx[6:9]) * self._R
        self._w = self._w + dx[9:12]
        IKH = np.eye(12) - K @ H
        self._P = IKH @ self._P @ IKH.T + K @ Rn @ K.T

        self._stamp = stamp
        out = _make_T(self._R, self._p)
        self._last_T = out
        return out
