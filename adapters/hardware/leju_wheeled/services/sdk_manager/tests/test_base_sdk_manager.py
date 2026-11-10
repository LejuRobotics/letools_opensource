"""
BaseSDKManager 单元测试

测试内容:
1. 初始化和关闭
2. MPC 模式管理（手动）
3. MPC 模式管理（自动 - 上下文管理器）
4. 错误处理
"""

import os
import sys

# 将项目根目录加入 sys.path，支持直接 python3 运行与 pytest 运行
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..', '..')))

import pytest
from unittest.mock import patch, call
import time

from adapters.hardware.leju_wheeled.services.sdk_manager import base_sdk_manager as base_sdk_module
from adapters.hardware.leju_wheeled.services.sdk_manager.base_sdk_manager import BaseSDKManager
from core.domain.result import Result


@pytest.fixture(autouse=True)
def sdk_mocks(monkeypatch):
    """把 KuavoSDK 与 RobotSDK 都换成 Mock，并复位进程级单例。

    两个都要换：代码里 robot_sdk 取自 RobotSDK() 的模块级单例
    （_robot_sdk_instance），只 patch KuavoSDK 拦不住它——那样每个用例都会构造
    真实 SDK，既慢又不确定，而 mock 上的断言永远为假。

    单例还必须复位：_robot_sdk_instance 一旦被上个用例填过，后面的用例会直接
    复用那个对象，用例之间互相污染。
    """
    monkeypatch.setattr(base_sdk_module, "_kuavo_sdk_init_done", False)
    monkeypatch.setattr(base_sdk_module, "_robot_sdk_instance", None)

    with patch.object(base_sdk_module, "KuavoSDK") as kuavo_sdk_cls, \
            patch.object(base_sdk_module, "RobotSDK") as robot_sdk_cls:
        yield kuavo_sdk_cls, robot_sdk_cls


def _robot_sdk(sdk_mocks):
    """取出 fixture 里的 RobotSDK Mock 实例——代码中 control.* 都调在它身上。"""
    _, robot_sdk_cls = sdk_mocks
    return robot_sdk_cls.return_value


class TestBaseSDKManagerInit:
    """测试初始化和关闭"""

    def test_init_creates_instance(self):
        """测试创建实例"""
        manager = BaseSDKManager()
        assert manager is not None
        assert manager.robot_sdk is None
        assert manager._initialized is False

    def test_initialize_success(self, sdk_mocks):
        """测试初始化成功"""
        kuavo_sdk_cls, robot_sdk_cls = sdk_mocks

        manager = BaseSDKManager()
        result = manager.initialize()

        assert result.success is True
        assert manager.is_initialized() is True
        # robot_sdk 来自 RobotSDK() 单例，不是 KuavoSDK 实例
        assert manager.robot_sdk is robot_sdk_cls.return_value
        # KuavoSDK.Init 是类方法，调用点在类上而非实例上
        kuavo_sdk_cls.Init.assert_called_once()

    def test_initialize_failure(self, sdk_mocks):
        """测试初始化失败"""
        kuavo_sdk_cls, _ = sdk_mocks
        kuavo_sdk_cls.Init.side_effect = Exception("初始化失败")

        manager = BaseSDKManager()
        result = manager.initialize()

        assert result.success is False
        assert "初始化失败" in result.message
        assert manager.is_initialized() is False

    def test_shutdown_success(self, sdk_mocks):
        """测试关闭成功。

        注意 shutdown() 不会关闭 RobotSDK：robot_sdk 是进程级单例，所有管理器
        共用同一份，单个管理器关闭时把它关掉会连累其它管理器。
        """
        manager = BaseSDKManager()
        manager.initialize()
        result = manager.shutdown()

        assert result.success is True
        assert manager.is_initialized() is False

    def test_shutdown_without_init(self):
        """测试未初始化时关闭"""
        manager = BaseSDKManager()
        result = manager.shutdown()

        # 应该返回成功（无操作）
        assert result.success is True


class TestBaseSDKManagerMPCMode:
    """测试 MPC 模式管理（手动）"""

    def test_set_external_control_mode(self, sdk_mocks):
        """测试设置外部控制模式"""
        robot_sdk = _robot_sdk(sdk_mocks)

        manager = BaseSDKManager()
        manager.initialize()

        # 调用方法
        manager.set_external_control_mode()

        # 验证调用
        robot_sdk.control.set_external_control_arm_mode.assert_called_once()

    def test_set_mpc_mode(self, sdk_mocks):
        """测试设置 MPC 模式"""
        from kuavo_humanoid_sdk.interfaces.data_types import (
            KuavoManipulationMpcCtrlMode,
            KuavoManipulationMpcControlFlow
        )
        robot_sdk = _robot_sdk(sdk_mocks)

        manager = BaseSDKManager()
        manager.initialize()

        # 调用方法
        manager.set_mpc_mode(
            mode=KuavoManipulationMpcCtrlMode.ArmOnly,
            flow=KuavoManipulationMpcControlFlow.ThroughFullBodyMpc
        )

        # 验证调用
        robot_sdk.control.set_manipulation_mpc_mode.assert_called_once_with(
            KuavoManipulationMpcCtrlMode.ArmOnly
        )
        robot_sdk.control.set_manipulation_mpc_control_flow.assert_called_once_with(
            KuavoManipulationMpcControlFlow.ThroughFullBodyMpc
        )

    def test_restore_mpc_mode(self, sdk_mocks):
        """测试恢复 MPC 模式"""
        from kuavo_humanoid_sdk.interfaces.data_types import (
            KuavoManipulationMpcCtrlMode,
            KuavoManipulationMpcControlFlow
        )
        robot_sdk = _robot_sdk(sdk_mocks)

        manager = BaseSDKManager()
        manager.initialize()

        # 调用方法
        manager.restore_mpc_mode()

        # 验证调用
        robot_sdk.control.set_manipulation_mpc_mode.assert_called_once_with(
            KuavoManipulationMpcCtrlMode.NoControl
        )
        robot_sdk.control.set_manipulation_mpc_control_flow.assert_called_once_with(
            KuavoManipulationMpcControlFlow.ThroughFullBodyMpc
        )

    def test_mpc_mode_sequence(self, sdk_mocks):
        """测试完整的 MPC 模式设置和恢复流程"""
        from kuavo_humanoid_sdk.interfaces.data_types import (
            KuavoManipulationMpcCtrlMode,
            KuavoManipulationMpcControlFlow
        )
        robot_sdk = _robot_sdk(sdk_mocks)
        # 必须显式给回 bool：若返回 MagicMock，代码里的 `if not success:` 求值会在
        # mock_calls 中插入 `().__bool__()` 记录，把下面要断言的调用顺序打断。
        robot_sdk.control.set_external_control_arm_mode.return_value = True
        robot_sdk.control.set_manipulation_mpc_mode.return_value = True
        robot_sdk.control.set_manipulation_mpc_control_flow.return_value = True

        manager = BaseSDKManager()
        manager.initialize()

        # 设置模式
        manager.set_external_control_mode()
        manager.set_mpc_mode(
            mode=KuavoManipulationMpcCtrlMode.ArmOnly,
            flow=KuavoManipulationMpcControlFlow.ThroughFullBodyMpc
        )

        # 恢复模式
        manager.restore_mpc_mode()

        # 验证调用顺序
        calls = [
            call.control.set_external_control_arm_mode(),
            call.control.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.ArmOnly),
            call.control.set_manipulation_mpc_control_flow(
                KuavoManipulationMpcControlFlow.ThroughFullBodyMpc
            ),
            call.control.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.NoControl),
            call.control.set_manipulation_mpc_control_flow(
                KuavoManipulationMpcControlFlow.ThroughFullBodyMpc
            ),
        ]
        robot_sdk.assert_has_calls(calls, any_order=False)


class TestBaseSDKManagerAutoMode:
    """测试 MPC 模式管理（自动 - 上下文管理器）"""

    def test_mpc_mode_context_enter(self, sdk_mocks):
        """测试进入上下文管理器"""
        from kuavo_humanoid_sdk.interfaces.data_types import (
            KuavoManipulationMpcCtrlMode,
            KuavoManipulationMpcControlFlow
        )
        robot_sdk = _robot_sdk(sdk_mocks)

        manager = BaseSDKManager()
        manager.initialize()

        # 进入上下文
        # wait_time=0 跳过设置/恢复后的等待：等待不是本组用例的被测对象，
        # 用默认的 0.5s 会让每个上下文用例白睡 1 秒
        with manager.mpc_mode_context(
            mode=KuavoManipulationMpcCtrlMode.ArmOnly,
            flow=KuavoManipulationMpcControlFlow.ThroughFullBodyMpc,
            wait_time=0,
        ):
            # 验证已设置模式
            robot_sdk.control.set_external_control_arm_mode.assert_called()
            robot_sdk.control.set_manipulation_mpc_mode.assert_called()

    def test_mpc_mode_context_exit(self, sdk_mocks):
        """测试退出上下文管理器"""
        from kuavo_humanoid_sdk.interfaces.data_types import (
            KuavoManipulationMpcCtrlMode,
            KuavoManipulationMpcControlFlow
        )
        robot_sdk = _robot_sdk(sdk_mocks)

        manager = BaseSDKManager()
        manager.initialize()

        # 退出上下文
        with manager.mpc_mode_context(
            mode=KuavoManipulationMpcCtrlMode.ArmOnly,
            flow=KuavoManipulationMpcControlFlow.ThroughFullBodyMpc,
            wait_time=0,
        ):
            pass

        # 验证已恢复模式
        robot_sdk.control.set_manipulation_mpc_mode.assert_called_with(
            KuavoManipulationMpcCtrlMode.NoControl
        )

    def test_mpc_mode_context_exception(self, sdk_mocks):
        """测试上下文管理器中的异常处理"""
        from kuavo_humanoid_sdk.interfaces.data_types import (
            KuavoManipulationMpcCtrlMode,
            KuavoManipulationMpcControlFlow
        )
        robot_sdk = _robot_sdk(sdk_mocks)

        manager = BaseSDKManager()
        manager.initialize()

        # 在上下文中抛出异常
        with pytest.raises(ValueError):
            with manager.mpc_mode_context(
                mode=KuavoManipulationMpcCtrlMode.ArmOnly,
                flow=KuavoManipulationMpcControlFlow.ThroughFullBodyMpc,
                wait_time=0,
            ):
                raise ValueError("测试异常")

        # 验证即使有异常，模式也被恢复
        robot_sdk.control.set_manipulation_mpc_mode.assert_called_with(
            KuavoManipulationMpcCtrlMode.NoControl
        )


class TestBaseSDKManagerErrorHandling:
    """测试错误处理"""

    def test_is_initialized_before_init(self):
        """测试初始化前检查状态"""
        manager = BaseSDKManager()
        assert manager.is_initialized() is False

    def test_is_initialized_after_init(self, sdk_mocks):
        """测试初始化后检查状态"""
        manager = BaseSDKManager()
        manager.initialize()
        assert manager.is_initialized() is True

    def test_is_initialized_after_shutdown(self, sdk_mocks):
        """测试关闭后检查状态"""
        manager = BaseSDKManager()
        manager.initialize()
        manager.shutdown()
        assert manager.is_initialized() is False

    def test_double_initialize(self, sdk_mocks):
        """测试重复初始化"""
        manager = BaseSDKManager()
        result1 = manager.initialize()
        result2 = manager.initialize()

        # 第二次初始化应该返回失败或警告
        assert result1.success is True
        # 注意：根据实际实现，第二次可能成功或失败


class TestRobotSDKSingleton:
    """多个 SDK 管理器共用同一份 RobotSDK。

    RobotSDK() 会建 ROS 连接并 sleep(2)，每个管理器各建一份既拖慢启动，
    又让上游 SDK 的构造期告警（"当前为轮臂模型"、KuavoRobotArm 废弃警告）
    按管理器数量重复打印。改用进程级单例后只构造一次。
    """

    def test_two_managers_share_one_robot_sdk(self, sdk_mocks):
        """两个管理器初始化后拿到同一个 RobotSDK 对象。"""
        _, robot_sdk_cls = sdk_mocks

        first = BaseSDKManager()
        second = BaseSDKManager()

        assert first.initialize().success is True
        assert second.initialize().success is True

        assert first.robot_sdk is second.robot_sdk
        robot_sdk_cls.assert_called_once()

    def test_kuavo_sdk_init_called_once_for_two_managers(self, sdk_mocks):
        """KuavoSDK.Init 也仍然只调用一次。"""
        kuavo_sdk_cls, _ = sdk_mocks

        first = BaseSDKManager()
        second = BaseSDKManager()

        first.initialize()
        second.initialize()

        assert kuavo_sdk_cls.Init.call_count == 1


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
