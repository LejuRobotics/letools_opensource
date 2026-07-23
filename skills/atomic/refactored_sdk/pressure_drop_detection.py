# -*- coding: utf-8 -*-
"""Atomic skill: pressure_drop_detection — 气压掉落检测。

持续监控夹爪气压传感器，检测到纸箱掉落时暂停流程、打印告警，等待人工确认后继续。

封装接口：
- `adapters.vacuum_control.read_pressure_kpa(sensor_id)` — 读取气压值 (kPa)
- `adapters.vacuum_485.blow()` — 破真空（ROS service /relay/channel_1_on）
- `adapters.vacuum_485.power_off()` — 断电全关（ROS service /relay/all_off）

判定条件：当左臂或右臂气压 > pressure_threshold 时判定为掉落。
气压为负值（真空吸力），正常搬运时气压很低（如 -50 kPa）；吸力消失后气压回升接近 0，
当气压 > 阈值（即吸力不足）时触发掉落警报。

默认值唯一来源：PressureDropDetectionParams dataclass 字段。
运行时调参：board.json 黑板（工厂通过 from_node_params 读取）。
"""

import os
import time
from dataclasses import dataclass
from typing import Optional

from adapters.vacuum_control import read_pressure_kpa

from core.common.logger import get_logger
from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.interfaces.i_hardware import IHardware
from orchestration.utils.manifest_decorators import define_manifest
from skills.base.skill_base import SkillBase

logger = get_logger(__name__)


@dataclass
class PressureDropDetectionParams(SkillParams):
    """气压掉落检测参数 — 代码默认值唯一定义处。

    改代码默认值只需修改此处；改运行时参数只需改 board.json。

    pressure_threshold:  掉落判定阈值 (kPa)，气压 > 此值时触发
    check_interval:      检测间隔（秒），每隔此时间检测一次
    enable:              是否启用检测
    left_sensor_id:      左臂气压传感器 ID
    right_sensor_id:     右臂气压传感器 ID
    auto_stop_chassis:   掉落时是否自动停止底盘/导航（调用 /enable_vel_control）
    """

    skill_name: str = "pressure_drop_detection"
    pressure_threshold: float = -15.0
    check_interval: float = 0.8
    enable: bool = True
    left_sensor_id: int = 2
    right_sensor_id: int = 1
    auto_stop_chassis: bool = True
    timeout: float = 999999.0  # 监控类技能，默认不超时

    @classmethod
    def from_node_params(cls, node_params: dict) -> "PressureDropDetectionParams":
        """从工厂 node_params 字典构造参数，缺失项用 dataclass 默认值兜底。

        工厂调用的唯一入口，替代散落在工厂里的逐参数 try/except 解析。
        """
        defaults = cls()

        def _extract(key, default, cast):
            raw = (node_params or {}).get(key)
            if isinstance(raw, dict) and "value" in raw:
                raw = raw["value"]
            if raw is None:
                return default
            try:
                return cast(raw)
            except (TypeError, ValueError):
                return default

        def _to_bool(v):
            if isinstance(v, bool):
                return v
            return str(v).strip().lower() in ("true", "1", "yes")

        return cls(
            pressure_threshold=_extract("pressure_threshold", defaults.pressure_threshold, float),
            check_interval=_extract("check_interval", defaults.check_interval, float),
            enable=_extract("enable", defaults.enable, _to_bool),
            left_sensor_id=_extract("left_sensor_id", defaults.left_sensor_id, int),
            right_sensor_id=_extract("right_sensor_id", defaults.right_sensor_id, int),
            auto_stop_chassis=_extract("auto_stop_chassis", defaults.auto_stop_chassis, _to_bool),
        )


@define_manifest(
    label="气压掉落检测",
    category=["safety", "vacuum", "perception"],
    tree_type="studio_smoke",
    description="持续监控夹爪气压，检测到掉落时暂停流程并等待人工确认后继续",
    params=[
        {"name": "pressure_threshold", "type": "float", "default": -15.0,
         "description": "掉落判定阈值 (kPa)，气压 > 此值时触发"},
        {"name": "check_interval", "type": "float", "default": 0.8,
         "description": "检测间隔（秒）"},
        {"name": "enable", "type": "bool", "default": True,
         "description": "是否启用检测"},
        {"name": "left_sensor_id", "type": "int", "default": 2,
         "description": "左臂气压传感器 ID"},
        {"name": "right_sensor_id", "type": "int", "default": 1,
         "description": "右臂气压传感器 ID"},
        {"name": "auto_stop_chassis", "type": "bool", "default": True,
         "description": "掉落时是否自动停止底盘/导航（调用 /enable_vel_control）"},
    ],
    inputs=[],
    outputs=[],
)
class PressureDropDetectionSkill(SkillBase):
    """气压掉落检测技能 —— 持续监控型，由外部循环反复调用 execute()。

    生命周期：
    - initialize()  → 重置状态，准备监控
    - execute()     → 每次调用读取气压并判定，正常返回 ok，掉落返回 fail
    - is_finished() → True 表示已检测到掉落（本轮监控结束）
    """

    def __init__(self, hardware: IHardware):
        super().__init__(name="pressure_drop_detection")
        self.hardware = hardware  # 保留以匹配 Skill 构造签名；气压读取走自己的 Modbus 串口
        self.params: Optional[PressureDropDetectionParams] = None
        self._alarm_triggered = False
        self._last_check_time = 0.0
        self._done = False
        self._result: Optional[Result] = None

    def on_initialize(self, params: PressureDropDetectionParams) -> Result:
        if not isinstance(params, PressureDropDetectionParams):
            return Result.fail("Invalid parameters for PressureDropDetectionSkill")
        self.params = params
        self._alarm_triggered = False
        self._last_check_time = 0.0
        self._done = False
        self._result = None
        logger.info(
            "[pressure_drop] 初始化: threshold=%.1f kPa, interval=%.2fs, "
            "enable=%s, left_id=%d, right_id=%d, auto_stop_chassis=%s",
            params.pressure_threshold, params.check_interval,
            params.enable, params.left_sensor_id, params.right_sensor_id,
            params.auto_stop_chassis,
        )
        return Result.ok()

    def on_execute(self) -> Result:
        if self._done:
            return self._result or Result.ok("monitoring finished")

        p = self.params
        if not p.enable or self._alarm_triggered:
            return Result.ok("monitoring active (disabled or already triggered)")

        # 检测间隔控制
        now = time.time()
        if (now - self._last_check_time) < p.check_interval:
            return Result.ok("monitoring active (interval skip)")
        self._last_check_time = now

        # 读取气压
        left_pressure = read_pressure_kpa(p.left_sensor_id)
        right_pressure = read_pressure_kpa(p.right_sensor_id)

        if left_pressure is None or right_pressure is None:
            logger.warning(
                "[pressure_drop] 气压读取失败 - 左臂: %s, 右臂: %s",
                left_pressure, right_pressure,
            )
            return Result.ok("monitoring active (read error)")

        logger.debug(
            "[pressure_drop] 左臂: %.2f kPa, 右臂: %.2f kPa, 阈值: %.1f kPa",
            left_pressure, right_pressure, p.pressure_threshold,
        )

        # 掉落判定
        if left_pressure > p.pressure_threshold or right_pressure > p.pressure_threshold:
            self._alarm_triggered = True
            self._done = True

            logger.error(
                "[pressure_drop] *** 掉落警报！*** 左臂: %.2f kPa, 右臂: %.2f kPa, "
                "阈值: %.1f kPa", left_pressure, right_pressure, p.pressure_threshold,
            )

            # 可选：自动停止底盘
            if p.auto_stop_chassis:
                self._stop_chassis()

            # 打印告警，暂停流程等待人工确认
            print("\n" + "=" * 60)
            print(f"*** 掉落警报！***")
            print(f"  左臂: {left_pressure:.2f} kPa, 右臂: {right_pressure:.2f} kPa, "
                  f"阈值: {p.pressure_threshold} kPa")
            print("请检查纸箱是否异常，确认后按 Enter 继续流程...")
            print("=" * 60)

            _DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")
            if not _DRY_RUN:
                try:
                    input()
                except EOFError:
                    pass

            # 人工确认后，恢复导航控制（交还底盘给导航模块）
            if p.auto_stop_chassis:
                self._restore_chassis()

            self._result = Result.ok("掉落警报已确认，继续流程")
            return self._result

        return Result.ok("monitoring active")

    def _stop_chassis(self):
        """掉落时停止底盘/导航运动（调用 /enable_vel_control(True)）。"""
        try:
            fn = getattr(self.hardware, "enable_vel_control_jibot", None)
            if fn is not None:
                result = fn(True)
                logger.info("[pressure_drop] 停止底盘: %s (%s)",
                            "成功" if result.success else "失败", result.message or "")
            else:
                logger.warning("[pressure_drop] 硬件无 enable_vel_control_jibot 方法，跳过停底盘")
        except Exception as e:
            logger.error("[pressure_drop] 停止底盘异常: %s", e, exc_info=True)

    def _restore_chassis(self):
        """人工确认后恢复导航控制（调用 /enable_vel_control(False)）。"""
        try:
            fn = getattr(self.hardware, "enable_vel_control_jibot", None)
            if fn is not None:
                result = fn(False)
                logger.info("[pressure_drop] 恢复导航控制: %s (%s)",
                            "成功" if result.success else "失败", result.message or "")
        except Exception as e:
            logger.error("[pressure_drop] 恢复导航控制异常: %s", e, exc_info=True)

    def reset_after_alarm(self):
        """掉落警报处理完毕后重置：保持 _alarm_triggered=True 不再检测，
        但 _done=False 使 is_finished() 返回 False，让 guard 继续放行子节点。
        """
        self._done = False

    def on_cancel(self) -> Result:
        self._done = True
        self._result = Result.ok("pressure_drop cancelled")
        logger.info("[pressure_drop] 监控已取消")
        return self._result

    def on_is_finished(self) -> bool:
        return self._done
