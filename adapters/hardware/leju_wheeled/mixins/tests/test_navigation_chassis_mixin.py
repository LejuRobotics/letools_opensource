"""通用导航底盘路由单元测试。"""

import sys
from pathlib import Path
from unittest.mock import MagicMock

if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).resolve().parents[5]))

from core.domain.result import Result
from adapters.hardware.leju_wheeled.mixins.navigation_chassis_mixin import (
    NavigationChassisMixin,
)


class _RoutingHardware(NavigationChassisMixin):
    def __init__(self, backend):
        self.config = {"navigation_chassis_backend": backend}
        self.base_move_relative_jiateng = MagicMock(
            return_value=Result.ok(data={"task_id": "jt-1"})
        )
        self.base_move_relative_jibot = MagicMock(
            return_value=Result.ok(data={"task_id": "jb-1"})
        )


def test_jiateng_backend_is_selected_from_hardware_config():
    hardware = _RoutingHardware("jiateng")
    result = hardware.move_chassis_relative(0.1, 0.0, 0.2)
    assert result.success
    assert result.data["task_id"] == "jt-1"
    hardware.base_move_relative_jiateng.assert_called_once_with(
        x=0.1,
        y=0.0,
        theta=0.2,
        options=None,
    )
    hardware.base_move_relative_jibot.assert_not_called()


def test_jibot_backend_is_selected_from_hardware_config():
    hardware = _RoutingHardware("jibot")
    result = hardware.move_chassis_relative(-0.2, 0.1, 0.0)
    assert result.success
    assert result.data["task_id"] == "jb-1"
    hardware.base_move_relative_jibot.assert_called_once()
    hardware.base_move_relative_jiateng.assert_not_called()


def test_missing_backend_returns_clear_failure():
    hardware = _RoutingHardware("")
    result = hardware.move_chassis_relative(0.1, 0.0, 0.0)
    assert not result.success
    assert "navigation_chassis_backend" in result.message
    hardware.base_move_relative_jiateng.assert_not_called()
    hardware.base_move_relative_jibot.assert_not_called()


if __name__ == "__main__":
    tests = (
        test_jiateng_backend_is_selected_from_hardware_config,
        test_jibot_backend_is_selected_from_hardware_config,
        test_missing_backend_returns_clear_failure,
    )
    for test in tests:
        test()
        print(f"PASS {test.__name__}")
    print(f"NavigationChassisMixin: {len(tests)} tests passed")
