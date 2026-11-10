"""
LowLevelSDKManager 单元测试

测试内容:
1. 底层 API 直接访问
2. MPC 模式管理（手动）
3. 各种控制方法
4. 单位转换验证
"""

import os
import sys

# 将项目根目录加入 sys.path，支持直接 python3 运行与 pytest 运行
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..', '..')))

import pytest
from unittest.mock import patch
import numpy as np

from adapters.hardware.leju_wheeled.services.sdk_manager import base_sdk_manager as base_sdk_module
from adapters.hardware.leju_wheeled.services.sdk_manager.low_level_sdk_manager import LowLevelSDKManager
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


class TestLowLevelSDKManagerInit:
    """测试初始化"""

    def test_init_creates_instance(self):
        """测试创建实例"""
        manager = LowLevelSDKManager()
        assert manager is not None
        assert hasattr(manager, 'robot_sdk')

    def test_initialize_success(self, sdk_mocks):
        """测试初始化成功"""
        manager = LowLevelSDKManager()
        result = manager.initialize()

        assert result.success is True
        assert manager.is_initialized() is True
        assert manager.robot_sdk is not None


class TestLowLevelSDKManagerArmControl:
    """测试手臂控制"""

    def test_control_arm_joint_positions(self, sdk_mocks):
        """测试手臂关节位置控制"""
        robot_sdk = _robot_sdk(sdk_mocks)
        robot_sdk.control.control_arm_joint_positions.return_value = True

        manager = LowLevelSDKManager()
        manager.initialize()

        # 传入弧度
        joint_angles_rad = [0.0] * 14

        result = manager.control_arm_joint_positions(joint_angles_rad)

        assert result.success is True
        robot_sdk.control.control_arm_joint_positions.assert_called_once_with(
            joint_positions=joint_angles_rad
        )

    def test_control_robot_end_effector_pose(self, sdk_mocks):
        """测试手臂末端位姿控制"""
        from kuavo_humanoid_sdk.interfaces.data_types import KuavoPose

        robot_sdk = _robot_sdk(sdk_mocks)
        robot_sdk.control.control_robot_end_effector_pose.return_value = True

        manager = LowLevelSDKManager()
        manager.initialize()

        # KuavoPose 的 position / orientation 是必填构造参数
        left_pose = KuavoPose(
            position=(0.5, 0.3, 0.3),
            orientation=(0.0, 0.707, 0.0, 0.707)
        )
        right_pose = KuavoPose(
            position=(0.5, -0.3, 0.3),
            orientation=(0.0, 0.707, 0.0, 0.707)
        )

        result = manager.control_robot_end_effector_pose(left_pose, right_pose)

        assert result.success is True
        robot_sdk.control.control_robot_end_effector_pose.assert_called_once()


class TestLowLevelSDKManagerHeadControl:
    """测试头部控制"""

    def test_control_head(self, sdk_mocks):
        """测试头部控制"""
        robot_sdk = _robot_sdk(sdk_mocks)
        robot_sdk.control.control_head.return_value = True

        manager = LowLevelSDKManager()
        manager.initialize()

        # 传入弧度
        yaw = np.deg2rad(30.0)
        pitch = np.deg2rad(-10.0)

        result = manager.control_head(yaw=yaw, pitch=pitch)

        assert result.success is True
        robot_sdk.control.control_head.assert_called_once_with(
            yaw=yaw, pitch=pitch
        )


class TestLowLevelSDKManagerBaseControl:
    """测试底盘控制"""

    def test_control_base_position(self, sdk_mocks):
        """测试底盘位置控制（世界系）"""
        robot_sdk = _robot_sdk(sdk_mocks)
        robot_sdk.control.control_command_pose_world.return_value = True

        manager = LowLevelSDKManager()
        manager.initialize()

        result = manager.control_base_position(target_pos=(0.5, 0.0, 0.0))

        assert result.success is True
        # 底层 API 收 (x, y, z, yaw)，z 固定为 0
        robot_sdk.control.control_command_pose_world.assert_called_once_with(
            0.5, 0.0, 0.0, 0.0
        )

    def test_control_base_velocity(self, sdk_mocks):
        """测试底盘速度控制"""
        robot_sdk = _robot_sdk(sdk_mocks)
        robot_sdk.control.walk.return_value = True

        manager = LowLevelSDKManager()
        manager.initialize()

        result = manager.control_base_velocity(vel_cmd=(0.3, 0.0, 0.0))

        assert result.success is True
        # 优先走 SDK walk()
        robot_sdk.control.walk.assert_called_once_with(0.3, 0.0, 0.0)


class TestLowLevelSDKManagerTorsoControl:
    """测试躯干控制"""

    def test_control_torso_6dof(self, sdk_mocks):
        """测试躯干 6DOF 控制"""
        robot_sdk = _robot_sdk(sdk_mocks)
        robot_sdk.control.control_torso_pose.return_value = True

        manager = LowLevelSDKManager()
        manager.initialize()

        # 传入弧度
        roll = 0.0
        pitch = np.deg2rad(-5.0)
        yaw = 0.0

        result = manager.control_torso_6dof(
            torso_pose=(0.0, 0.0, 0.8, roll, pitch, yaw)
        )

        assert result.success is True
        robot_sdk.control.control_torso_pose.assert_called_once_with(
            x=0.0, y=0.0, z=0.8, roll=roll, pitch=pitch, yaw=yaw
        )


class TestLowLevelSDKManagerLegControl:
    """测试下肢控制"""

    def test_control_wheel_lower_joint(self, sdk_mocks):
        """测试下肢关节控制"""
        robot_sdk = _robot_sdk(sdk_mocks)
        robot_sdk.control.control_wheel_lower_joint.return_value = True

        manager = LowLevelSDKManager()
        manager.initialize()

        # 下肢接口收的是角度（度），不是弧度
        joint_angles_deg = [14.90, -32.01, 18.03, -90.0]

        result = manager.control_wheel_lower_joint(joint_traj=joint_angles_deg)

        assert result.success is True
        robot_sdk.control.control_wheel_lower_joint.assert_called_once_with(
            joint_angles_deg
        )


class TestLowLevelSDKManagerUnitConversion:
    """测试单位转换"""

    def test_degree_to_radian_conversion(self, sdk_mocks):
        """测试度到弧度的转换"""
        robot_sdk = _robot_sdk(sdk_mocks)
        robot_sdk.control.control_head.return_value = True

        manager = LowLevelSDKManager()
        manager.initialize()

        # 用户传入度，需要手动转换为弧度
        yaw_deg = 45.0
        pitch_deg = -10.0

        yaw_rad = np.deg2rad(yaw_deg)
        pitch_rad = np.deg2rad(pitch_deg)

        result = manager.control_head(yaw=yaw_rad, pitch=pitch_rad)

        assert result.success is True

        # 验证传递给 API 的是弧度
        call_args = robot_sdk.control.control_head.call_args
        assert abs(call_args[1]['yaw'] - yaw_rad) < 1e-6
        assert abs(call_args[1]['pitch'] - pitch_rad) < 1e-6


class TestLowLevelSDKManagerMPCMode:
    """测试 MPC 模式管理"""

    def test_manual_mpc_mode_management(self, sdk_mocks):
        """测试手动 MPC 模式管理"""
        from kuavo_humanoid_sdk.interfaces.data_types import (
            KuavoManipulationMpcCtrlMode,
            KuavoManipulationMpcControlFlow
        )
        robot_sdk = _robot_sdk(sdk_mocks)
        robot_sdk.control.control_arm_joint_positions.return_value = True

        manager = LowLevelSDKManager()
        manager.initialize()

        # 手动设置模式
        manager.set_external_control_mode()
        manager.set_mpc_mode(
            mode=KuavoManipulationMpcCtrlMode.ArmOnly,
            flow=KuavoManipulationMpcControlFlow.ThroughFullBodyMpc
        )

        # 执行控制
        joint_angles_rad = [0.0] * 14
        result = manager.control_arm_joint_positions(joint_angles_rad)

        assert result.success is True

        # 手动恢复模式
        manager.restore_mpc_mode()

        # 验证调用顺序
        assert robot_sdk.control.set_external_control_arm_mode.called
        assert robot_sdk.control.set_manipulation_mpc_mode.called
        assert robot_sdk.control.control_arm_joint_positions.called
        assert robot_sdk.control.set_manipulation_mpc_mode.call_count == 2  # 设置 + 恢复


class TestLowLevelSDKManagerErrorHandling:
    """测试错误处理"""

    def test_not_initialized(self, sdk_mocks):
        """测试未初始化时调用"""
        manager = LowLevelSDKManager()

        joint_angles_rad = [0.0] * 14

        # 未初始化时返回失败的 Result，而不是抛异常
        result = manager.control_arm_joint_positions(joint_angles_rad)

        assert result.success is False
        assert "未初始化" in result.message

    def test_api_failure(self, sdk_mocks):
        """测试 API 调用失败"""
        robot_sdk = _robot_sdk(sdk_mocks)
        robot_sdk.control.control_arm_joint_positions.return_value = False

        manager = LowLevelSDKManager()
        manager.initialize()

        joint_angles_rad = [0.0] * 14

        result = manager.control_arm_joint_positions(joint_angles_rad)

        assert result.success is False
        assert "失败" in result.message

    def test_exception_handling(self, sdk_mocks):
        """测试异常处理"""
        robot_sdk = _robot_sdk(sdk_mocks)
        robot_sdk.control.control_arm_joint_positions.side_effect = Exception("通信错误")

        manager = LowLevelSDKManager()
        manager.initialize()

        joint_angles_rad = [0.0] * 14

        # 异常被转换成失败的 Result，而不是向上抛出
        result = manager.control_arm_joint_positions(joint_angles_rad)

        assert result.success is False
        assert "通信错误" in result.message


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
