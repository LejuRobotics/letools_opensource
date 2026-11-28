"""
ArmSDKManager 单元测试

测试内容:
1. 自动 MPC 模式管理
2. 手动 MPC 模式管理
3. 末端轨迹控制
4. 关节轨迹控制
5. 手臂归位
"""

import os
import sys

# 将项目根目录加入 sys.path，支持直接 python3 运行与 pytest 运行
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..', '..')))

import pytest
from unittest.mock import Mock, patch, MagicMock
import time

from adapters.hardware.leju_wheeled.services.sdk_manager.arm_sdk_manager import ArmSDKManager
from core.domain.result import Result
from kuavo_humanoid_sdk.interfaces.data_types import (
    KuavoManipulationMpcCtrlMode,
    KuavoManipulationMpcControlFlow,
)


class TestArmSDKManagerInit:
    """测试初始化"""

    def test_init_creates_instance(self):
        """测试创建实例"""
        manager = ArmSDKManager()
        assert manager is not None

    @patch('adapters.hardware.leju_wheeled.services.sdk_manager.base_sdk_manager.KuavoSDK')
    def test_initialize_success(self, mock_sdk_class):
        """测试初始化成功"""
        mock_sdk = Mock()
        mock_sdk_class.return_value = mock_sdk
        
        manager = ArmSDKManager()
        result = manager.initialize()
        
        assert result.success is True
        assert manager.is_initialized() is True


class TestArmSDKManagerAutoMode:
    """测试自动 MPC 模式管理"""


    @patch('adapters.hardware.leju_wheeled.services.sdk_manager.arm_sdk_manager.ArmAPI')
    @patch('adapters.hardware.leju_wheeled.services.sdk_manager.base_sdk_manager.KuavoSDK')
    def test_move_joint_traj_auto(self, mock_sdk_class, mock_arm_api_class):
        """测试关节轨迹控制（自动模式）"""
        mock_sdk = Mock()
        mock_sdk_class.return_value = mock_sdk
        
        mock_arm_api = Mock()
        mock_arm_api.move_joint_traj.return_value = True
        mock_arm_api_class.return_value = mock_arm_api
        
        manager = ArmSDKManager()
        manager.initialize()
        
        left_traj = [[0.0] * 7, [0.1] * 7]
        right_traj = [[0.0] * 7, [0.1] * 7]
        
        result = manager.move_joint_traj_auto(
            left_traj=left_traj,
            right_traj=right_traj,
            total_time=3.0,
            back_default=False
        )
        
        assert result.success is True
        mock_arm_api.move_joint_traj.assert_called_once()

    @patch('adapters.hardware.leju_wheeled.services.sdk_manager.arm_sdk_manager.ArmAPI')
    @patch('adapters.hardware.leju_wheeled.services.sdk_manager.base_sdk_manager.KuavoSDK')
    def test_arm_reset_auto(self, mock_sdk_class, mock_arm_api_class):
        """测试手臂归位（自动模式）"""
        mock_sdk = Mock()
        mock_sdk_class.return_value = mock_sdk
        
        mock_arm_api = Mock()
        mock_arm_api.arm_reset.return_value = True
        mock_arm_api_class.return_value = mock_arm_api
        
        manager = ArmSDKManager()
        manager.initialize()
        
        result = manager.arm_reset(back_default=False)
        
        assert result.success is True
        mock_arm_api.arm_reset.assert_called_once()


class TestArmSDKManagerManualMode:
    """测试手动 MPC 模式管理"""

    @patch('adapters.hardware.leju_wheeled.services.sdk_manager.arm_sdk_manager.ArmAPI')
    @patch('adapters.hardware.leju_wheeled.services.sdk_manager.base_sdk_manager.KuavoSDK')
    def test_move_eef_traj_manual(self, mock_sdk_class, mock_arm_api_class):
        """测试末端轨迹控制（手动模式）"""
        mock_sdk = Mock()
        mock_sdk_class.return_value = mock_sdk
        
        mock_arm_api = Mock()
        mock_arm_api.move_eef_traj_wheel_mpc.return_value = True
        mock_arm_api_class.return_value = mock_arm_api
        
        manager = ArmSDKManager()
        manager.initialize()
        
        # 手动设置模式
        manager.set_external_control_mode()
        manager.set_mpc_mode(
            mode=KuavoManipulationMpcCtrlMode.ArmOnly,
            flow=KuavoManipulationMpcControlFlow.ThroughFullBodyMpc
        )
        time.sleep(0.1)  # 模拟等待
        
        left_traj = [[0.5, 0.3, 0.3, 0.0, 0.707, 0.0, 0.707]]
        right_traj = [[0.5, -0.3, 0.3, 0.0, 0.707, 0.0, 0.707]]
        
        result = manager.move_eef_traj_manual(
            left_traj=left_traj,
            right_traj=right_traj,
            total_time=3.0,
            frame='world'
        )
        
        assert result.success is True
        
        # 手动恢复模式
        manager.restore_mpc_mode()
        
        # 验证调用
        assert mock_sdk.control.set_external_control_arm_mode.called
        assert mock_arm_api.move_eef_traj_wheel_mpc.called
        assert mock_sdk.control.set_manipulation_mpc_mode.called

    @patch('adapters.hardware.leju_wheeled.services.sdk_manager.arm_sdk_manager.ArmAPI')
    @patch('adapters.hardware.leju_wheeled.services.sdk_manager.base_sdk_manager.KuavoSDK')
    def test_move_joint_traj_manual(self, mock_sdk_class, mock_arm_api_class):
        """测试关节轨迹控制（手动模式）"""
        mock_sdk = Mock()
        mock_sdk_class.return_value = mock_sdk
        
        mock_arm_api = Mock()
        mock_arm_api.move_joint_traj.return_value = True
        mock_arm_api_class.return_value = mock_arm_api
        
        manager = ArmSDKManager()
        manager.initialize()
        
        # 手动设置模式
        manager.set_external_control_mode()
        manager.set_mpc_mode(
            mode=KuavoManipulationMpcCtrlMode.ArmOnly,
            flow=KuavoManipulationMpcControlFlow.ThroughFullBodyMpc
        )
        
        left_traj = [[0.0] * 7]
        right_traj = [[0.0] * 7]
        
        result = manager.move_joint_traj_manual(
            left_traj=left_traj,
            right_traj=right_traj,
            total_time=2.0
        )
        
        assert result.success is True
        
        # 手动恢复模式
        manager.restore_mpc_mode()


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
