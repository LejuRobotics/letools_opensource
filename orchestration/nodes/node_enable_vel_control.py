# -*- coding: utf-8 -*-
"""NodeEnableVelControl：树启动时切换底盘速度控制权限，停止时恢复。

真机底盘控制（base_move_relative_jibot）前必须先把底盘速度控制权限切到
**jibot 底盘导航**（/enable_vel_control=False），否则 /move_base/base_move
会被拒绝或无效——enable=True 是人形机器人接管底盘，enable=False 才是 jibot
底盘导航模块接管。本节点在 initialise() 里切换一次，update() 保持 RUNNING
（源节点口径，不收口），terminate() 恢复到切换前的状态。

来自 AAEON 下位机 kuavo-ros-control，非 Jarvis 底盘服务，**可能并非所有环境都有**
（见 chassis_mixin.py 顶部注释）。服务不存在时不阻断树（返 RUNNING + feedback），
因为没有它也能跑的底盘不应被本节点卡死。

黑板契约：不读写黑板（纯硬件副作用节点）。
"""
import os

import py_trees
from py_trees.common import Status

from core.common.logger import get_logger
from orchestration.nodes.base_node import BaseAction
from orchestration.shared_hardware import get_shared_hardware
from orchestration.utils.manifest_decorators import define_manifest

logger = get_logger(__name__)

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


def _parse_bool_param(val, default: bool) -> bool:
    """解析布尔参数；ParamsWrapper 可能把 bool 存成字符串 'false'/'true'。

    Python 的 ``bool("false")`` 是 ``True``（非空字符串都为真），故不能直接
    ``bool(...)``；须按字符串内容判断，与工厂 ``_convert_type`` 同口径。
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


def _parse_optional_bool_param(val):
    """解析可选布尔参数：None / 空串表示「未设置」，返回 None。

    与 ``_parse_bool_param`` 同口径解析 true/false 字符串，只是多一个「未设置」态，
    用于 ``restore_to`` 这类「不填就走默认逻辑」的兜底参数。
    """
    if val is None:
        return None
    if isinstance(val, bool):
        return val
    if isinstance(val, str):
        s = val.strip().lower()
        if s == "":
            return None
        if s in ("true", "1", "yes"):
            return True
        if s in ("false", "0", "no"):
            return False
        return None
    return bool(val)


@define_manifest(
    label="速度控制权限切换",
    category=["skill", "chassis", "servo"],
    tree_type="studio_smoke",
    description="树启动时 enable_vel_control(False) 切到 jibot 底盘导航，停止时恢复",
    params=[
        {"name": "enable", "type": "bool", "default": "False",
         "description": "False=jibot 底盘导航接管（base_move 可用），"
                        "True=人形机器人接管底盘"},
        {"name": "restore_on_stop", "type": "bool", "default": "True",
         "description": "树停止时是否恢复切换前的状态"},
        {"name": "restore_to", "type": "bool",
         "description": "收尾强制恢复到该值，不依赖原状态读取是否成功（兜底用）；"
                        "不填=恢复到读到的原值"},
    ],
)
class NodeEnableVelControl(BaseAction):
    """底盘速度控制权限切换（硬件）：启动切、停止恢复。"""

    def __init__(self, name, label, namespace, params):
        super(NodeEnableVelControl, self).__init__(name, label, namespace, params)
        self._enable = _parse_bool_param(self.params.get("enable", False), False)
        self._restore = _parse_bool_param(self.params.get("restore_on_stop", True), True)
        self._restore_to = _parse_optional_bool_param(self.params.get("restore_to"))
        self._orig_state = None      # 切换前原状态，terminate 时恢复
        self._switched = False       # initialise 里是否切换成功
        self._service_missing = False

    def initialise(self):
        self._orig_state = None
        self._switched = False
        self._service_missing = False

        if _DRY_RUN:
            self.feedback_message = "dry-run enable_vel_control"
            return

        hw = get_shared_hardware()
        # enable_vel_control_jibot 是 LejuWheeledArmHardware 的方法（jibot mixin），
        # 不是 IHardware 抽象方法；非 jibot 底盘没有它时不阻断。
        if not hasattr(hw, "enable_vel_control_jibot"):
            self._service_missing = True
            self.feedback_message = "当前硬件无 enable_vel_control_jibot（非 jibot？），跳过"
            return

        # 记录原状态供 terminate 恢复；restore_to 已给定时无需读取。
        # 读不到不能静默——没有原值 terminate 就恢复不了，权限会永久停在 self._enable。
        state_unknown = False
        if self._restore and self._restore_to is None \
                and hasattr(hw, "get_vel_control_state_jibot"):
            state_res = hw.get_vel_control_state_jibot(timeout=3.0)
            state = state_res.data.get("state") if state_res.success and state_res.data else None
            if state is None:
                state_unknown = True
                logger.warning(
                    "[NodeEnableVelControl] 读取底盘速度控制权限原状态失败：%s；"
                    "收尾将无法自动恢复，权限可能停在 %s，请人工确认"
                    "（可用 restore_to 兜底）",
                    state_res.message, self._enable)
            else:
                self._orig_state = state

        result = hw.enable_vel_control_jibot(self._enable)
        if result.success:
            self._switched = True
            if state_unknown:
                self.feedback_message = (
                    f"enable_vel_control({self._enable}) 成功，"
                    f"原状态未知（收尾不恢复，需人工确认）")
            else:
                self.feedback_message = (
                    f"enable_vel_control({self._enable}) 成功，原状态={self._orig_state}")
        else:
            # 切换失败是硬错误：底盘控制不可用，整树应停（Parallel 下 FAILURE 终止树）。
            self.feedback_message = f"enable_vel_control 失败：{result.message}"

    def update(self):
        if _DRY_RUN or self._service_missing:
            return Status.RUNNING
        if not self._switched:
            return Status.FAILURE
        # 切换成功后保持 RUNNING：源节点口径，不触发 success_on_one 收口。
        # Parallel 在节点保持 RUNNING 时不会重进 initialise()，故只切一次。
        return Status.RUNNING

    def terminate(self, new_status):
        if not (self._restore and self._switched):
            return

        # restore_to 优先：显式配置的目标值，不依赖原状态是否读到
        target = self._restore_to if self._restore_to is not None else self._orig_state
        if target is None:
            # 原状态没读到（initialise 里已 WARN）：不能猜，只能出声提醒
            logger.warning(
                "[NodeEnableVelControl] 原状态未知，跳过恢复；底盘速度控制权限可能停在 "
                "%s，请人工确认（下次可设 restore_to=true|false 兜底）", self._enable)
            return

        try:
            hw = get_shared_hardware()
            if hasattr(hw, "enable_vel_control_jibot"):
                res = hw.enable_vel_control_jibot(target)
                if getattr(res, "success", False):
                    logger.info("[NodeEnableVelControl] 已恢复底盘速度控制权限为 %s", target)
                else:
                    logger.warning(
                        "[NodeEnableVelControl] 恢复底盘速度控制权限为 %s 失败：%s；请人工确认",
                        target, getattr(res, "message", ""))
        except Exception as e:                     # noqa: BLE001
            # 恢复是 best-effort：树正在收场，不该为一个恢复失败抛出去，但要留痕。
            logger.warning(
                "[NodeEnableVelControl] 恢复底盘速度控制权限为 %s 异常：%s；请人工确认",
                target, e)
