# -*- coding: utf-8 -*-
"""NodeChassisServoPhase：读视觉偏差角 → 判决先转后平的两阶段 → 写黑板 servo_phase。

拍板来源
--------
底盘伺服原来的双模逻辑是「按平移误差大小在位置/速度两模式间滞回切换」，两条通道
同时被激活（一条发 dx/dy/dtheta，一条发 vx/vy/wz）。真机跑下来发现旋转这一路始终
无效：`wz` 没有死区补偿，而真实偏差角很小，算出来的 `wz` 整段落进底盘角速度死区，
「转了跟没转一样」，平移也被这一路拖着。

本节点把控制律改成分阶段的**串行**流程：

    阶段 1（rotate，转角度）：只发 `wz`，`vx`/`vy` 严格置零
        —— 先把 |theta_rad| 收敛到阈值以内，底盘朝向对准
    阶段 2（translate，平移）：只发 `vx`/`vy`，`wz` 严格置零
        —— 再把两路像素平移偏差收敛到阈值以内

先对准朝向再平移，是因为平移的像素偏差是在**当前朝向**下测的：朝向没对好就平移，
每次平移都会改变观测视角，误差不单调、收敛判据来回抖。

单向闩锁（不可回退）
--------------------
进阶段 2 后**不再回退**，即使 `theta` 后来漂出阈值。理由：两阶段各有自己的收敛
判据（阶段 1 看 theta、阶段 2 看像素），来回切会让两个判据都难以稳定满足；而且
`wz` 死区未实测前，反复回退大概率只是空转。代价是角度漂了不会自纠正 —— 此时
judge 因 `theta` 超阈值不判收敛，一路跑到 exec 的 `velocity_timeout_s` 判 FAILURE，
日志里能看到超时而非静默错收。真机若确认有漂移，再加回差（对照 `NodeChassisServoMode`
的 `enter_px`/`exit_px` 滞回）即可。

阈值与 judge 共用
-----------------
`theta_threshold_rad` 必须与 judge 的 `servo_stop_theta_rad` 取同一个值（推荐在
场景里配成同一个数）：阶段 1 交棒那一刻 `|theta|` 就已经小于 judge 的角度阈值，
阶段 2 才有希望收口。若本节点阈值比 judge 松，会出现「阶段机认为转好了、judge
认为角度没收敛」的死区，平移对准了也收不了口。

节拍口径（源节点）
------------------
观测缺失/无效一律返回 `RUNNING`（**不是** FAILURE）：源节点等不到数据就该等，判
FAILURE 会让 `Parallel(success_on_one)` 误以为失败而收口，把还在途的底盘动作掐断
（同 node_chassis_servo_judge 的注释）。观测无效时维持现状 —— 零速兜底归 exec。

连续帧计数按**观测版本**推进，不按 tick
--------------------------------------
`converge_frames` 数的是**连续多少条不同的观测**满足阈值（读 `{obs_key}_version`，
与 calc / judge 同口径）。按 tick 数是不行的：主循环 ~50Hz 而真机视觉 ~10Hz，同一条
观测会被 tick 三五次，「连续 2 帧」实际只覆盖 40ms、连一条观测的寿命都不到，防抖
形同虚设。版本未更新时本节点直接返回，**不重复计数**。

权限由本节点自己管（重要）
--------------------------
本节点**兼管速度控制权限**，取代 `NodeChassisServoMode` + `NodeEnableVelControl`
那套两处配置的写法：

- `initialise()`：先发零速，再 `enable_vel_control_jibot(True)` 开门
- `terminate()`：先发零速，再 `enable_vel_control_jibot(False)` 还原

为什么不让场景去配 `NodeEnableVelControl.enable=true`：那是**第二个开关**，
改了阶段机却忘了改权限就重现 2026-10-15 21:23 那次故障（模式说 velocity、权限
始终 False，`/cmd_vel` 没人转发，底盘一动不动）；而且落地的权限状态依赖
`restore_on_stop` 与启动时读到的原状态，不唯一。开-关两条路径都在本节点内，
不存在「只知道一半」的中间态。

黑板契约
--------
读 latest_servo_error   AlignObservation 或 dev 的 ServoError（鸭子类型判别）
写 servo_phase          str，当前阶段："rotate" | "translate"
"""

import os
import time

import py_trees
from py_trees.common import Status

from core.domain.servo_types import is_observation_like, observation_field
from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")

# 阶段取值（对应 calc 的 phase 参数）
PHASE_ROTATE = "rotate"
PHASE_TRANSLATE = "translate"
_VALID_PHASES = (PHASE_ROTATE, PHASE_TRANSLATE)

OBS_KEY_DEFAULT = "latest_servo_error"
PHASE_KEY_DEFAULT = "servo_phase"


def _read_blackboard(client, key, default=None):
    """读黑板键；py_trees Client.__getattr__ 抛 KeyError，须显式 try（同 node_chassis_servo_judge）。"""
    try:
        return getattr(client, key)
    except KeyError:
        return default


def _read_version(client, key) -> int:
    """读 `key_version`；从未写过 / 不是数字都当 0（同 node_chassis_servo_calc）。"""
    try:
        return int(_read_blackboard(client, key, 0))
    except (TypeError, ValueError):
        return 0


def _parse_bool(val, default: bool) -> bool:
    """解析布尔参数；ParamsWrapper 可能把 bool 存成字符串 'false'/'true'。

    Python 的 ``bool("false")`` 是 ``True``（非空字符串都为真），故不能直接
    ``bool(...)``；须按字符串内容判断，与工厂 ``_convert_type`` 同口径
    （同 node_enable_vel_control）。
    """
    if isinstance(val, bool):
        return val
    if isinstance(val, str):
        s = val.strip().lower()
        if s in ("true", "1", "yes"):
            return True
        if s in ("false", "0", "no", ""):
            return False
    return bool(val) if val is not None else default


@define_manifest(
    label="底盘伺服阶段选择",
    category=["skill", "chassis", "servo"],
    tree_type="studio_smoke",
    description="读偏差角 → 判决先转后平的两阶段 → 写 servo_phase + 开关速度权限（硬件）",
    params=[
        {"name": "phase_key", "type": "string", "default": "servo_phase",
         "description": "写阶段的黑板键名（\"rotate\"|\"translate\"）"},
        {"name": "obs_key", "type": "string", "default": "latest_servo_error",
         "description": "读观测量的黑板键名（须与 calc / judge 的 obs_key 一致）"},
        {"name": "initial_phase", "type": "string", "default": "rotate",
         "description": "起始阶段：rotate=先转角度（默认），translate=直接平移"},
        {"name": "theta_threshold_rad", "type": "float", "default": "0.09",
         "description": "|偏差角| < 它 判阶段 1 完成；须与 judge 的 servo_stop_theta_rad 取同值"},
        {"name": "converge_frames", "type": "int", "default": "2",
         "description": "连续多少条**不同的观测**满足阈值才交棒（按观测版本计数，防噪声里一闪而过就提前切）"},
        {"name": "own_vel_control", "type": "bool", "default": "true",
         "description": "是否由本节点开关速度权限：true=initialise 开门(True)、terminate 还原(False)；"
                        "false=只做阶段判决，权限交给别的节点"},
        {"name": "settle_s", "type": "float", "default": "0.6",
         "description": "权限切换后等待(s)，等底盘侧生效（与 NodeChassisServoMode 同口径）"},
    ],
    inputs=[{"name": "latest_servo_error", "type": "object",
             "description": "AlignObservation/ServoError：两路像素偏差 + 偏差角 + valid"}],
    outputs=[{"name": "servo_phase", "type": "string",
              "description": "当前阶段：rotate | translate"}],
)
class NodeChassisServoPhase(BaseAction):
    """底盘伺服阶段选择节点（纯计算）：偏差角 → 先转后平的两阶段判决。"""

    # "这个键从未被写过"的哨兵（不能拿 None 代替：None 是合法的「写过但为空」）
    _MISSING = object()

    def __init__(self, name, label, namespace, params):
        super(NodeChassisServoPhase, self).__init__(name, label, namespace, params)
        self._phase_key = str(self.params.get("phase_key", PHASE_KEY_DEFAULT))
        self._obs_key = str(self.params.get("obs_key", OBS_KEY_DEFAULT))
        # 参数从 ParamsWrapper 读出来是字符串，显式 float/int 转换
        self._theta_threshold = float(self.params.get("theta_threshold_rad", 0.09))
        self._converge_frames = int(self.params.get("converge_frames", 2))
        self._initial_phase = self._parse_phase(
            self.params.get("initial_phase", PHASE_ROTATE))
        self._own_vel_control = _parse_bool(
            self.params.get("own_vel_control", True), True)
        self._settle_s = float(self.params.get("settle_s", 0.6))

        self._phase = None            # 本节点认定的当前阶段（None=首次 tick 时按黑板/初值初始化）
        self._theta_frames = 0        # 连续满足角度阈值的帧数
        self._version_seen = None     # 已计入连续帧计数的观测版本号

        self.global_blackboard.register_key(
            key=self._obs_key, access=py_trees.common.Access.READ)
        self.global_blackboard.register_key(
            key=f"{self._obs_key}_version", access=py_trees.common.Access.READ)
        self.global_blackboard.register_key(
            key=self._phase_key, access=py_trees.common.Access.WRITE)

    def _parse_phase(self, raw) -> str:
        """把参数收敛到合法阶段；拼错就退回 rotate 并告警（不静默用错阶段）。"""
        val = str(raw).strip().lower() if raw is not None else ""
        if val in _VALID_PHASES:
            return val
        if val:
            from core.common.logger import get_logger
            get_logger(__name__).warning(
                "[chassis_servo_phase] initial_phase=%r 非法（期望 rotate|translate），"
                "退回 %s", raw, PHASE_ROTATE)
        return PHASE_ROTATE

    def initialise(self):
        self._phase = None
        self._theta_frames = 0
        self._version_seen = None
        # 开门：先发零速（切换期间底盘不可控），再切权限。切不动就维持现状 ——
        # 阶段判决照跑，但下游 `/cmd_vel` 没人转发，日志里会看到本节点的反馈。
        if self._own_vel_control and not _DRY_RUN:
            self._send_zero_velocity()
            hw = get_shared_hardware()
            if not hasattr(hw, "enable_vel_control_jibot"):
                self.feedback_message = (
                    "当前硬件无 enable_vel_control_jibot（非 jibot？），速度权限未开")
            else:
                result = hw.enable_vel_control_jibot(True)
                if result.success and self._settle_s > 0:
                    time.sleep(self._settle_s)

    def update(self):
        if _DRY_RUN:
            self.feedback_message = "dry-run chassis_servo_phase"
            return Status.RUNNING

        # 首次 tick：黑板已写过就信黑板（重启后能接着跑），否则用 initial_phase
        if self._phase is None:
            raw = _read_blackboard(self.global_blackboard, self._phase_key, self._MISSING)
            if raw is self._MISSING or raw is None:
                self._phase = self._initial_phase
            else:
                self._phase = (raw if raw in _VALID_PHASES else self._initial_phase)
            self.global_blackboard.set(self._phase_key, self._phase)

        # 已到阶段 2：单向闩锁，不再回退（见模块 docstring）
        if self._phase == PHASE_TRANSLATE:
            self.feedback_message = f"阶段 {PHASE_TRANSLATE}（角度已对准，不再回退）"
            return Status.RUNNING

        # 版本门禁：连续帧计数要按**新的观测**算，不是按 tick。主循环 ~50Hz、真机视觉
        # ~10Hz，同一条观测会被 tick 到三五次；按 tick 计数的话"连续 2 帧"实际只覆盖
        # 40ms、一条观测的寿命都不到，噪声帧照样会被数进去 —— 防抖形同虚设。
        # 与 calc / judge 同口径读 `{obs_key}_version`。
        newest_version = _read_version(self.global_blackboard, f"{self._obs_key}_version")
        if newest_version == self._version_seen:
            return Status.RUNNING

        theta = self._read_theta()
        if theta is None:
            # 观测缺失 / 无效：维持现状，返回 RUNNING（源节点等数据，不判 FAILURE）。
            # 不更新 _version_seen —— 否则这一版观测会被永久跳过。
            return Status.RUNNING
        self._version_seen = newest_version

        if abs(theta) < self._theta_threshold:
            self._theta_frames += 1
        else:
            self._theta_frames = 0

        if self._theta_frames >= self._converge_frames:
            self._phase = PHASE_TRANSLATE
            self.global_blackboard.set(self._phase_key, PHASE_TRANSLATE)
            self.feedback_message = (
                f"角度已收敛（|theta|={abs(theta):.4f} < {self._theta_threshold}，"
                f"连续 {self._theta_frames} 帧）→ 交棒 {PHASE_TRANSLATE}")
            return Status.RUNNING

        self.feedback_message = (
            f"维持 {PHASE_ROTATE}：|theta|={abs(theta):.4f}"
            f"（阈值 {self._theta_threshold}，连续 {self._theta_frames}/{self._converge_frames} 帧）")
        return Status.RUNNING

    def _read_theta(self):
        """读观测并取偏差角；不可用返回 None。"""
        obs = _read_blackboard(self.global_blackboard, self._obs_key, self._MISSING)
        if obs is self._MISSING or obs is None:
            self.feedback_message = "等观测（黑板上还没有 AlignObservation）"
            return None
        if not is_observation_like(obs):
            self.feedback_message = (
                f"{self._obs_key} 不是观测量（缺 e_bottom_px/e_right_px/theta_rad）")
            return None
        if not observation_field(obs, "valid", True):
            self.feedback_message = (
                f"观测无效：{observation_field(obs, 'reject', '') or 'unknown'}，"
                f"阶段维持 {self._phase}（零速兜底归 exec）")
            return None
        return float(observation_field(obs, "theta_rad", 0.0))

    def _send_zero_velocity(self):
        """零速兜底：best-effort，硬件不支持或抛异常都不阻断开/还原权限流程。"""
        try:
            hw = get_shared_hardware()
            if hasattr(hw, "send_base_velocity_tick"):
                hw.send_base_velocity_tick(0.0, 0.0, 0.0)
        except Exception:  # noqa: BLE001
            # 零速是兜底（best-effort），发不出去不该让权限切换崩掉。但它是**安全
            # 动作**，静默吞掉会让「车没停住」这件事无从查起，故记 error + 栈。
            from core.common.logger import get_logger
            get_logger(__name__).error(
                "chassis_servo_phase: 零速兜底发送失败", exc_info=True)

    def terminate(self, new_status):
        """树终止钩子：发零速 + 还原速度权限。

        还原是**无条件**的（不看 initialise 是否成功开过）：权限是全局状态，
        本节点既然承担了开门职责，就必须保证任何退出路径都把门关回去，否则
        底盘会停在「人形控制器接管」态，下一次 base_move 类动作全部失效。
        """
        if _DRY_RUN or not self._own_vel_control:
            return
        self._send_zero_velocity()
        try:
            hw = get_shared_hardware()
            if hasattr(hw, "enable_vel_control_jibot"):
                hw.enable_vel_control_jibot(False)
        except Exception:  # noqa: BLE001
            # 还原是 best-effort：树正在收场，不该为一个恢复失败抛出去
            pass
