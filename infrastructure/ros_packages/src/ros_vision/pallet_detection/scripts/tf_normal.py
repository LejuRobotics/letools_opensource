# -*- coding: utf-8 -*-
"""从 TF 链查台面法向先验。**这一层会 `import rospy`，所以放在基础设施层。**

`pallet_detect` 的算法与搬运层（`skills/atomic/perception/pallet_detect/`）**不许
import ROS** —— 那是分层硬约束。但"从 TF 查法向"这件事本质上要 ROS，所以它落在
这里，紧挨着调用它的节点。

    normal_from_tf(buf, target_frame, source_frame, stamp, missing)  ->  (3,) | None

⚠️ **`CHAIN` 与 maduo 的 `tools/tf_normal_from_bag.py` 共用同一份定义**
（那条链 13 段，那边用来从 bag 离线查、这边在线查）。改链时**两边都要改**。

⚠️ **缺任何一段都返回 `None`，绝不静默返回单位阵** —— 单位阵 = 法向竖直向下，
而检测**不会崩、不会报错**，只是换个 score 继续算。调用方据此**不发布**并给出
明确报错。理由见 `pallet_detect/README.md` §3 第 1 条。
"""
from __future__ import annotations

import numpy as np

from skills.atomic.perception.pallet_detect.payload import CHAIN  # noqa: F401


def normal_from_tf(buf, target_frame: str, source_frame: str = 'base_link',
                   stamp=None, missing: list | None = None) -> np.ndarray | None:
    """从 TF 查 `source_frame` 的 +z 在 `target_frame`（相机系）里的方向（= 台面法向先验）。

    逐段查 `tools/tf_normal_from_bag.CHAIN`（13 段），拼出 `T_source_target`。

    ⚠️ **缺任何一段都返回 None**，不要静默返回单位阵 —— 与
    `tools/tf_normal_from_bag.py` 同一条规矩（那条链 13 段，缺一段就拼不出来）。
    调用方（节点）据此**不发布**并给出明确报错，而不是拿个瞎猜的法向继续算 ——
    detect 单独对法向极其敏感，错 5° 就是 ~180mm（见模块 docstring；
    但 refine 的平面拟合能修回来，前提是 `plane=True`）。

    ⚠️ 相机系是**光学系**（+x 右 +y 下 +z 前），所以：
        up_in_camera = T_base_camera[:3, :3].T @ [0, 0, 1]

    `missing`：给一个 list 时，缺的那一段 `(parent, child)` 会被 append 进去
    （调用方要拿它写日志/`rejects`）。**这是唯一的副作用，不影响返回值。**
    """
    # ⚠️ 这里**不** `import tf2_ros`：本函数只用 `buf` 的 `lookup_transform` 接口，
    # 不依赖 tf2_ros 这个模块本身。加上那个 import 会让它**没有 ROS 就 import 不了**，
    # 而它是纯函数、单元测试要用假 buffer 直接调（`test_pallet_detection_node.py`）。
    T = np.eye(4, dtype=np.float64)
    for parent, child in CHAIN:
        edge = _lookup_edge(buf, parent, child, stamp)
        if edge is None:
            if missing is not None:
                missing.append((parent, child))
            return None
        T = T @ edge
    up = T[:3, :3].T @ np.array([0.0, 0.0, 1.0])
    norm = float(np.linalg.norm(up))
    if not np.isfinite(norm) or norm < 1e-9:
        if missing is not None:
            missing.append(('<normal 退化>', f'|up|={norm}'))
        return None
    return up / norm


def camera_origin_from_tf(buf, target_frame: str, source_frame: str = 'base_link',
                          stamp=None, missing: list | None = None):
    """`source_frame`（相机光学系）原点在 `target_frame`（base_link）里的位置，**米**。

    返回 `(3,) | None`。**与 `normal_from_tf` 走同一条 `CHAIN`**，只是取的是平移
    而不是旋转 —— 两者拼的是同一个 `T_base_cam`，所以这里再走一遍只是为了避免
    把两个返回值捆成一个接口（调用方经常只要法向）。

    ⚠️ **用来把"沿法向的相对高度"换算成"base_link 的绝对高度"**：
    某个点在相机系是 `P`（米），它沿法向的高度是 `h = P·n`，而它在 base_link 的
    绝对 z 是 `(T_base_cam @ P).z`。两者只差一个**常数**：

        K = t_base_cam.z + (origin_placeholder · n)

    其中 `origin_placeholder` 是 `detect_pallet_frame` 里那个占位平面原点。
    调用方把 `K` 算出来，就能把 `deck_h` 约束在"托盘应该在的绝对高度带"里
    （现场实测：托盘 z ≈ +43mm、地面 ≈ −107mm、**手里抱着的纸箱 ≈ +600mm**，
    所以一条 `[-40, +130]` 的带就能把纸箱整个排除掉）。

    ⚠️ 缺任何一段都返回 `None`，与 `normal_from_tf` 同一条规矩。
    """
    T = np.eye(4, dtype=np.float64)
    for parent, child in CHAIN:
        edge = _lookup_edge(buf, parent, child, stamp)
        if edge is None:
            if missing is not None:
                missing.append((parent, child))
            return None
        T = T @ edge
    t = np.asarray(T[:3, 3], dtype=np.float64)
    if not np.all(np.isfinite(t)):
        if missing is not None:
            missing.append(('<origin 退化>', f't={t.tolist()}'))
        return None
    return t


def _lookup_edge(buf, parent: str, child: str, stamp):
    """查一段 `T_parent_child`（4x4）。查不到返回 None。

    先按 `stamp` 查（动态 TF 必须按时刻查，否则拿到的是"现在"的那一段，
    与图像帧对不上）；查不到再退回"最新值"（`Time(0)` —— tf2 的约定，见
    `Buffer.lookup_transform` 的 docstring：*0 will get the latest*）。
    仍查不到再试一次反方向（TF 里只登记了一个方向是常事）—— 与
    `tf_normal_from_bag.lookup` 同一条规矩：**缺边返回 None，不静默造一个**。
    """
    candidates = ((parent, child), (child, parent))
    for stamp_try in ([stamp, 0.0] if stamp is not None else [0.0]):
        for target, source in candidates:
            try:
                tr = buf.lookup_transform(target, source, _ros_time(stamp_try))
            except Exception:                        # noqa: BLE001
                continue
            E = _transform_to_matrix(tr)
            if E is None:
                continue
            return E if (target, source) == (parent, child) else np.linalg.inv(E)
    return None


def _ros_time(sec: float):
    """秒 → `rospy.Time`（不 import rospy 也能构造时给 0 = 最新）。"""
    import rospy
    return rospy.Time.from_sec(float(sec))


def _transform_to_matrix(tr) -> np.ndarray | None:
    """`geometry_msgs/TransformStamped` → 4x4。取不出来返回 None。"""
    try:
        from tf.transformations import quaternion_matrix
    except ImportError:                              # pragma: no cover
        return None
    q = tr.transform.rotation
    p = tr.transform.translation
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
    T[:3, 3] = [p.x, p.y, p.z]
    return T
