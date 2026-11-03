# -*- coding: utf-8 -*-
"""NodeChassisServoMode：读视觉误差 → 双阈值滞回判决 → 切底盘速度权限 → 写模式键。

飞书 §3.3 / DESIGN §4.6 的「模式选择」薄封装：底盘伺服有位置模式（导航栈离散目标点，
单步最小 0.08 m）与速度模式（持续 `/cmd_vel`，可做厘米以下微动）两条通路，两者互斥
（速度权限 `/enable_vel_control` 是硬门控）。本节点按视觉误差在两模式间自动切换。

判决规则（双阈值滞回，DESIGN §3.2）
-----------------------------------
    位置模式               │        速度模式
    ───────────────────────┼───────────────────────►  误差变大
    e_px: 0 ── enter ──────┼────── exit ──────────►
                (小)       │       (大)

    当前 position 且 e_px < enter_px  → 切 velocity
    当前 velocity 且 e_px > exit_px   → 回 position
    其余（含 enter_px < e_px < exit_px 的滞回带）→ 维持现状，不调权限服务

**不变式：`exit_px` 必须严格大于 `enter_px`**，两者之间是死区。滞回带内的抖动静默
不切换——每次切换都要打一次权限服务（内部 `rospy.sleep(0.5)` + 状态回读，0.5~1 s
底盘不可控，DESIGN §3.3），边界横跳的代价极高。若把 `exit_px` 配成 ≤ `enter_px`，
死区消失、退化成单阈值，两侧条件可能同时成立或以抖动速率反复成立，本文件在
`initialise()` 里对这种配置给出告警但仍按配置执行（不改写用户的现场标定值）。

切换动作的顺序（DESIGN §4.6 第 4 步）
-------------------------------------
    0. 先查 `servo_exec_idle`    位置动作在途（False）就**不切**，等到达再切
                                 （硬门控会掐断在途任务，见 `update()` 里的注释）
    1. 先发零速          速度模式退回位置模式前必须停车（速度是「持续的」，
                         不发零速它就按最后速度一直跑，DESIGN §4.5）
    2. enable_vel_control_jibot(target)   切权限，内部自带 sleep + 回读
    3. sleep(settle_s)   再等权限切换在底盘侧生效
    4. 写黑板 servo_mode  最后才对外宣告新模式，避免下游读到「模式已切但权限未落地」
    5. 切换计数 +1；达 max_mode_switch 后**锁定**当前模式，不再调权限服务

节拍口径（源节点）
------------------
观测缺失/无效一律返回 `RUNNING`（**不是** FAILURE）：源节点等不到数据就该等，判 FAILURE
会让 Parallel(success_on_one) 或 Sequence 误以为失败而收口/终止，把还在途的底盘动作
掐断（同 node_chassis_servo_judge 的注释）。观测无效另有一层理由——按 DESIGN §4.5，
「观测看门狗超时立即发零速」是 exec 层的职责，本节点只做模式判决，观测无效时维持
现状即可，零速兜底在 exec。

黑板契约
--------
读 latest_servo_error  AlignObservation 或 dev 的 ServoError（鸭子类型判别）
读 servo_mode          str，当前模式："position" | "velocity"，默认 "position"
读 servo_exec_idle     bool，False = 位置动作在途，此时不切权限（暂缓到到达）
写 servo_mode          str，判决后的模式

权限还原的职责划分（与 node_enable_vel_control 的协调，DESIGN §4.6「冲突处理」）
------------------------------------------------------------------------------
现有 `NodeEnableVelControl.restore_on_stop=True` 会在树停止时把权限还原成
initialise() 记录的「原状态」，与本节点抢权限。新场景里让 `NodeEnableVelControl`
只做**初始置位**（`restore_on_stop=false`，场景配置在 ticket 06 落），**还原职责
交给本节点的 terminate()**。这样还原路径唯一、不会两处互相覆盖（谁后跑谁赢）。
本节点不读也不改 `NodeEnableVelControl` 的任何状态，协调完全靠场景参数，故无需
依赖它在树中的位置。

安全（DESIGN §4.5 / §4.5.1）
----------------------------
`use_vel_control_` 的源码默认值是 **True**（门槛本就敞开），零速兜底是唯一防线：
- 每次切换前先发零速
- terminate() 无条件发零速 + `enable_vel_control_jibot(False)` 还原

硬件缺失（非 jibot 底盘没有 `enable_vel_control_jibot`）时按「服务不存在」处理：
不切换、发 feedback、保持 RUNNING，不阻断树（同 node_enable_vel_control 的处置）。
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

# 模式取值（对应 ServoCommand.mode 与速度权限的映射）
MODE_POSITION = "position"
MODE_VELOCITY = "velocity"

OBS_KEY_DEFAULT = "latest_servo_error"


def _read_blackboard(client, key, default=None):
    """读黑板键；py_trees Client.__getattr__ 抛 KeyError，须显式 try（同 node_chassis_servo_judge）。"""
    try:
        return getattr(client, key)
    except KeyError:
        return default


def _read_idle(client, key) -> bool:
    """读节拍门禁键；从未写过（执行节点还没构造）当作空闲，别把树卡死。"""
    return bool(_read_blackboard(client, key, True))


@define_manifest(
    label="底盘伺服模式选择",
    category=["skill", "chassis", "servo"],
    tree_type="studio_smoke",
    description="读视觉误差 → 双阈值滞回判决 → 切速度权限 → 写 servo_mode（硬件）",
    params=[
        {"name": "mode_key", "type": "string", "default": "servo_mode",
         "description": "写模式的黑板键名（\"position\"|\"velocity\"）"},
        {"name": "obs_key", "type": "string", "default": "latest_servo_error",
         "description": "读观测量的黑板键名（须与 calc / judge 的 obs_key 一致）"},
        {"name": "enter_px", "type": "float", "default": "12.0",
         "description": "误差 < 它 进速度模式"},
        {"name": "exit_px", "type": "float", "default": "25.0",
         "description": "误差 > 它 退回位置模式（不变式：须 > enter_px）"},
        {"name": "max_mode_switch", "type": "int", "default": "2",
         "description": "切换次数上限，达到后锁定当前模式不再调权限服务"},
        {"name": "settle_s", "type": "float", "default": "0.6",
         "description": "权限切换后等待(s)，等底盘侧生效再宣告新模式"},
        {"name": "idle_key", "type": "string", "default": "servo_exec_idle",
         "description": "读节拍门禁的黑板键名（False=底盘在途，本节点暂缓切权限）"},
    ],
    inputs=[{"name": "latest_servo_error", "type": "object",
             "description": "AlignObservation/ServoError：两像素偏差 + 偏差角 + valid"},
            {"name": "servo_mode", "type": "string",
             "description": "当前模式：position | velocity"},
            {"name": "servo_exec_idle", "type": "bool",
             "description": "False=位置动作在途，禁止切权限（硬门控会掐断在途任务）"}],
    outputs=[{"name": "servo_mode", "type": "string",
              "description": "判决后的模式：position | velocity"}],
)
class NodeChassisServoMode(BaseAction):
    """底盘伺服模式选择节点（硬件）：误差滞回判决 + 权限切换。"""

    # "这个键从未被写过"的哨兵（不能拿 None 代替：None 是合法的「写过但为空」）
    _MISSING = object()

    def __init__(self, name, label, namespace, params):
        super(NodeChassisServoMode, self).__init__(name, label, namespace, params)
        self._mode_key = str(self.params.get("mode_key", "servo_mode"))
        self._obs_key = str(self.params.get("obs_key", OBS_KEY_DEFAULT))
        # enter_px / exit_px / ... 从 ParamsWrapper 读出来是字符串，显式 float/int 转换
        self._enter_px = float(self.params.get("enter_px", 12.0))
        self._exit_px = float(self.params.get("exit_px", 25.0))
        self._max_switch = int(self.params.get("max_mode_switch", 2))
        self._settle_s = float(self.params.get("settle_s", 0.6))
        self._idle_key = str(self.params.get("idle_key", "servo_exec_idle"))

        self._switch_count = 0        # 已发生的模式切换次数（达上限即锁定）
        self._mode = None             # 本节点认定的当前模式（None=首次 tick 时按黑板初始化）

        self.global_blackboard.register_key(
            key=self._obs_key, access=py_trees.common.Access.READ)
        self.global_blackboard.register_key(
            key=self._mode_key, access=py_trees.common.Access.WRITE)
        self.global_blackboard.register_key(
            key=self._idle_key, access=py_trees.common.Access.READ)

    def initialise(self):
        self._switch_count = 0
        self._mode = None

        if self._exit_px <= self._enter_px:
            # 滞回带为空 → 退化成单阈值，两侧可能同时成立；不擅自改写现场标定值，只告警
            from core.common.logger import get_logger
            get_logger(__name__).warning(
                "[chassis_servo_mode] 配置告警：exit_px(%.3f) 必须 > enter_px(%.3f)，"
                "否则滞回带为空、模式会抖", self._exit_px, self._enter_px)

    def update(self):
        if _DRY_RUN:
            self.feedback_message = "dry-run chassis_servo_mode"
            return Status.RUNNING

        # 读当前模式（默认 position）：首次 tick 从黑板取；一旦本节点切过就信自己的状态
        if self._mode is None:
            raw = _read_blackboard(self.global_blackboard, self._mode_key, MODE_POSITION)
            self._mode = MODE_VELOCITY if raw == MODE_VELOCITY else MODE_POSITION

        e_px = self._read_error_px()
        if e_px is None:
            # 观测缺失 / 无效：维持现状，返回 RUNNING（源节点等数据，不判 FAILURE）
            return Status.RUNNING

        # 滞回判决（exit_px > enter_px 是不变式，中间是死区）
        target = self._mode
        if self._mode == MODE_POSITION and e_px < self._enter_px:
            target = MODE_VELOCITY
        elif self._mode == MODE_VELOCITY and e_px > self._exit_px:
            target = MODE_POSITION

        if target == self._mode:
            self.feedback_message = (
                f"维持 {self._mode}：e_px={e_px:.1f}"
                f"（enter={self._enter_px}, exit={self._exit_px}）")
            return Status.RUNNING

        # 次数已用尽 → 锁定当前模式，不再调权限服务
        if self._switch_count >= self._max_switch:
            self.feedback_message = (
                f"模式已锁定 {self._mode}（切换上限 {self._max_switch}），忽略 → {target}")
            return Status.RUNNING

        # 位置动作在途时**不许切权限**：`enable_vel_control=True` 是硬门控，会把在途的
        # jibot 导航任务从导航栈手里抢走，任务既不 arrived 也不 interrupted，卡在
        # `moving` 直到 exec 的 arrival_timeout（默认 20s）才放行 —— 期间 exec 停在
        # `_poll_arrival()` 不读新指令、judge 被本键挡着不判决，底盘静止而终端刷屏
        # （2026-10-15 真机 17:53:29~17:53:49 实测）。等这一步走完（exec 清在途、写
        # idle=True）再切。DESIGN §4.6 的时序本就是「位置模式走完两步再交棒」。
        if not _read_idle(self.global_blackboard, self._idle_key):
            self.feedback_message = (
                f"位置动作在途，暂缓切 {target}（e_px={e_px:.1f}），等到达")
            return Status.RUNNING

        self._switch_to(target, e_px)
        return Status.RUNNING

    def _read_error_px(self):
        """读观测并取误差 e_px = max(|e_bottom_px|, |e_right_px|)；不可用返回 None。"""
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
                f"模式维持 {self._mode}（零速兜底归 exec）")
            return None
        return max(abs(float(observation_field(obs, "e_bottom_px", 0.0))),
                   abs(float(observation_field(obs, "e_right_px", 0.0))))

    def _switch_to(self, target, e_px):
        """执行一次模式切换：零速 → 切权限 → 等待 → 写模式 → 计数+1。"""
        hw = get_shared_hardware()

        if not hasattr(hw, "enable_vel_control_jibot"):
            self.feedback_message = (
                "当前硬件无 enable_vel_control_jibot（非 jibot？），不切换模式")
            return

        enable = (target == MODE_VELOCITY)

        # 1) 先发零速：切换期间底盘不可控，停车是唯一安全状态
        self._send_zero_velocity()
        # 2) 切权限（内部自带 sleep(0.5) + 状态回读）
        result = hw.enable_vel_control_jibot(enable)
        if not result.success:
            self.feedback_message = f"enable_vel_control({enable}) 失败：{result.message}"
            return
        # 3) 等权限在底盘侧生效，再对外宣告新模式
        if self._settle_s > 0:
            time.sleep(self._settle_s)
        self._mode = target
        self.global_blackboard.set(self._mode_key, target)

        self._switch_count += 1
        self.feedback_message = (
            f"切到 {target}（e_px={e_px:.1f}），第 {self._switch_count} 次切换"
            + (f"，已达切换上限 {self._max_switch}，锁定"
               if self._switch_count >= self._max_switch else ""))

    def _send_zero_velocity(self):
        """零速兜底：best-effort，硬件不支持或抛异常都不阻断切换/终止流程。"""
        try:
            hw = get_shared_hardware()
            if hasattr(hw, "send_base_velocity_tick"):
                hw.send_base_velocity_tick(0.0, 0.0, 0.0)
        except Exception:  # noqa: BLE001
            # 零速是兜底（best-effort），发不出去不该让模式判决崩掉。但它是**安全
            # 动作**，静默吞掉会让「车没停住」这件事无从查起，故记 error + 栈。
            from core.common.logger import get_logger
            get_logger(__name__).error(
                "chassis_servo_mode: 零速兜底发送失败", exc_info=True)

    def terminate(self, new_status):
        """树终止钩子：发零速 + 还原速度权限（本节点承担还原职责，见模块 docstring）。"""
        if _DRY_RUN:
            return
        self._send_zero_velocity()
        try:
            hw = get_shared_hardware()
            if hasattr(hw, "enable_vel_control_jibot"):
                hw.enable_vel_control_jibot(False)
        except Exception:  # noqa: BLE001
            # 还原是 best-effort：树正在收场，不该为一个恢复失败抛出去
            pass
