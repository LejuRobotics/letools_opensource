"""
TimedCmdManager 单元测试

测试内容:
1. 7 种控制模式
2. 单位转换（度 → 弧度）
3. 参数验证
4. 错误处理
"""

import os
import sys

# 将项目根目录加入 sys.path，支持直接 python3 运行与 pytest 运行
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..', '..')))

import pytest
from unittest.mock import Mock, patch, MagicMock
import numpy as np

from adapters.hardware.leju_wheeled.services.sdk_manager.timed_cmd_manager import TimedCmdManager
from core.domain.result import Result


class TestTimedCmdManagerInit:
    """测试初始化"""

    def test_init_creates_instance(self):
        """测试创建实例"""
        manager = TimedCmdManager()
        assert manager is not None
        assert hasattr(manager, 'timed_cmd_api')

    @patch('adapters.hardware.leju_wheeled.services.sdk_manager.timed_cmd_manager.TimedCmdAPI')
    @patch('adapters.hardware.leju_wheeled.services.sdk_manager.base_sdk_manager.KuavoSDK')
    def test_initialize_success(self, mock_sdk_class, mock_api_class):
        """测试初始化成功"""
        mock_sdk = Mock()
        mock_sdk_class.return_value = mock_sdk
        
        mock_api = Mock()
        mock_api_class.return_value = mock_api
        
        manager = TimedCmdManager()
        result = manager.initialize()
        
        assert result.success is True
        assert manager.is_initialized() is True


class TestTimedCmdManagerChassis:
    """测试底盘控制"""

    @patch('adapters.hardware.leju_wheeled.services.sdk_manager.timed_cmd_manager.TimedCmdAPI')
    @patch('adapters.hardware.leju_wheeled.services.sdk_manager.base_sdk_manager.KuavoSDK')
    def test_send_chassis_world_success(self, mock_sdk_class, mock_api_class):
        """测试底盘世界系控制成功"""
        mock_sdk = Mock()
        mock_sdk_class.return_value = mock_sdk
        
        mock_api = Mock()
        mock_api.send_timed_cmd.return_value = (True, 3.0)
        mock_api_class.return_value = mock_api
        
        manager = TimedCmdManager()
        manager.initialize()
        
        result = manager.send_chassis_world(
            x=0.5, y=0.0, yaw=0.0, desire_time=3.0
        )
        
        assert result.success is True
        assert result.data['actual_time'] == 3.0
        mock_api.send_timed_cmd.assert_called_once_with(
            cmd_type='chassis_world',
            cmd_vec=[0.5, 0.0, 0.0],
            desire_time=3.0
        )

    @patch('adapters.hardware.leju_wheeled.services.sdk_manager.timed_cmd_manager.TimedCmdAPI')
    @patch('adapters.hardware.leju_wheeled.services.sdk_manager.base_sdk_manager.KuavoSDK')
    def test_send_chassis_local_success(self, mock_sdk_class, mock_api_class):
        """测试底盘局部系控制成功"""
        mock_sdk = Mock()
        mock_sdk_class.return_value = mock_sdk
        
        mock_api = Mock()
        mock_api.send_timed_cmd.return_value = (True, 2.5)
        mock_api_class.return_value = mock_api
        
        manager = TimedCmdManager()
        manager.initialize()
        
        result = manager.send_chassis_local(
            x=0.3, y=0.2, yaw=0.5, desire_time=2.5
        )
        
        assert result.success is True
        mock_api.send_timed_cmd.assert_called_once_with(
            cmd_type='chassis_local',
            cmd_vec=[0.3, 0.2, 0.5],
            desire_time=2.5
        )

    @patch('adapters.hardware.leju_wheeled.services.sdk_manager.timed_cmd_manager.TimedCmdAPI')
    @patch('adapters.hardware.leju_wheeled.services.sdk_manager.base_sdk_manager.KuavoSDK')
    def test_send_chassis_world_failure(self, mock_sdk_class, mock_api_class):
        """测试底盘控制失败"""
        mock_sdk = Mock()
        mock_sdk_class.return_value = mock_sdk
        
        mock_api = Mock()
        mock_api.send_timed_cmd.return_value = (False, 0.0)
        mock_api_class.return_value = mock_api
        
        manager = TimedCmdManager()
        manager.initialize()
        
        result = manager.send_chassis_world(
            x=0.5, y=0.0, yaw=0.0, desire_time=3.0
        )
        
        assert result.success is False
        assert "底盘世界系控制失败" in result.message


class TestTimedCmdManagerTorso:
    """测试躯干控制"""

    @patch('adapters.hardware.leju_wheeled.services.sdk_manager.timed_cmd_manager.TimedCmdAPI')
    @patch('adapters.hardware.leju_wheeled.services.sdk_manager.base_sdk_manager.KuavoSDK')
    def test_send_torso_pose(self, mock_sdk_class, mock_api_class):
        """测试躯干位姿控制"""
        mock_sdk = Mock()
        mock_sdk_class.return_value = mock_sdk
        
        mock_api = Mock()
        mock_api.send_timed_cmd.return_value = (True, 2.0)
        mock_api_class.return_value = mock_api
        
        manager = TimedCmdManager()
        manager.initialize()
        
        result = manager.send_torso_pose(
            x=0.0, z=0.8, yaw=0.0, pitch=-0.2, desire_time=2.0
        )
        
        assert result.success is True
        mock_api.send_timed_cmd.assert_called_once()


class TestTimedCmdManagerLegJoint:
    """测试下肢关节控制"""

    @patch('adapters.hardware.leju_wheeled.services.sdk_manager.timed_cmd_manager.TimedCmdAPI')
    @patch('adapters.hardware.leju_wheeled.services.sdk_manager.base_sdk_manager.KuavoSDK')
    def test_send_leg_joint_with_degrees(self, mock_sdk_class, mock_api_class):
        """测试下肢关节控制（自动转换度→弧度）"""
        mock_sdk = Mock()
        mock_sdk_class.return_value = mock_sdk
        
        mock_api = Mock()
        mock_api.send_timed_cmd.return_value = (True, 1.5)
        mock_api_class.return_value = mock_api
        
        manager = TimedCmdManager()
        manager.initialize()
        
        # 传入角度（度）
        joint_angles_deg = [14.90, -32.01, 18.03, -90.0]
        
        result = manager.send_leg_joint(
            joint_angles_deg=joint_angles_deg,
            desire_time=1.5
        )
        
        assert result.success is True
        
        # 验证是否转换为弧度
        call_args = mock_api.send_timed_cmd.call_args
        cmd_vec = call_args[1]['cmd_vec']
        
        # 检查是否为弧度（应该小于 2π）
        for angle in cmd_vec:
            assert abs(angle) <= np.pi * 2
        
        # 验证具体值
        expected_rad = [np.deg2rad(a) for a in joint_angles_deg]
        for actual, expected in zip(cmd_vec, expected_rad):
            assert abs(actual - expected) < 1e-6


class TestTimedCmdManagerArmJoint:
    """测试手臂关节控制"""

    @patch('adapters.hardware.leju_wheeled.services.sdk_manager.timed_cmd_manager.TimedCmdAPI')
    @patch('adapters.hardware.leju_wheeled.services.sdk_manager.base_sdk_manager.KuavoSDK')
    def test_send_arm_joint_with_degrees(self, mock_sdk_class, mock_api_class):
        """测试手臂关节控制（一次批量调用，planner 8+9，isSync=True，自动转换度→弧度）"""
        mock_api = Mock()
        mock_api.send_timed_cmd.return_value = (True, 2.0)
        mock_api_class.return_value = mock_api

        manager = TimedCmdManager()
        # 绕开 initialize()（真实 KuavoSDK.Init 会等 ROS 服务超时），直接置就绪状态
        manager._initialized = True
        manager.robot_sdk = Mock()
        manager.timed_cmd_api = mock_api

        # 传入 14 个关节角度（度）[左臂7, 右臂7]
        joint_angles_deg = [10.0] * 7 + [20.0] * 7

        # 捕获 send_multi_commands 的调用参数（send_arm_joint 应委托给它）
        with patch.object(manager, 'send_multi_commands',
                          return_value=Result.ok(data={'actual_time': 2.0})) as mock_multi:
            result = manager.send_arm_joint(
                joint_angles_deg=joint_angles_deg,
                desire_time=2.0
            )

        assert result.success is True
        assert result.data == {'actual_time': 2.0}
        # 不再走 SDK 串行拆分路径
        mock_api.send_timed_cmd.assert_not_called()
        # 一次批量调用：左臂 planner 8 + 右臂 planner 9，同步执行
        mock_multi.assert_called_once()
        commands, kwargs = mock_multi.call_args[0], mock_multi.call_args[1]
        assert kwargs.get('is_sync') is True
        assert len(commands[0]) == 2
        # 左臂 7 关节（度→弧度）
        assert commands[0][0]['planner_index'] == 8
        assert commands[0][0]['desire_time'] == 2.0
        expected_left = [np.deg2rad(a) for a in joint_angles_deg[:7]]
        for actual, expected in zip(commands[0][0]['cmd_vec'], expected_left):
            assert abs(actual - expected) < 1e-6
        # 右臂 7 关节（度→弧度）
        assert commands[0][1]['planner_index'] == 9
        assert commands[0][1]['desire_time'] == 2.0
        expected_right = [np.deg2rad(a) for a in joint_angles_deg[7:]]
        for actual, expected in zip(commands[0][1]['cmd_vec'], expected_right):
            assert abs(actual - expected) < 1e-6


class TestTimedCmdManagerArmEE:
    """测试手臂末端控制"""

    @patch('adapters.hardware.leju_wheeled.services.sdk_manager.timed_cmd_manager.TimedCmdAPI')
    @patch('adapters.hardware.leju_wheeled.services.sdk_manager.base_sdk_manager.KuavoSDK')
    def test_send_arm_ee_world(self, mock_sdk_class, mock_api_class):
        """测试手臂末端世界系控制（一次批量调用，planner 4+5，isSync=True）"""
        mock_api = Mock()
        mock_api.send_timed_cmd.return_value = (True, 3.0)
        mock_api_class.return_value = mock_api

        manager = TimedCmdManager()
        # 绕开 initialize()（真实 KuavoSDK.Init 会等 ROS 服务超时），直接置就绪状态
        manager._initialized = True
        manager.robot_sdk = Mock()
        manager.timed_cmd_api = mock_api

        left_pose = [0.5, 0.3, 0.3, 0.0, 0.1, 0.0]
        right_pose = [0.5, -0.3, 0.3, 0.0, 0.1, 0.0]

        # 捕获 send_multi_commands 的调用参数（send_arm_ee_world 应委托给它）
        with patch.object(manager, 'send_multi_commands',
                          return_value=Result.ok(data={'actual_time': 3.0})) as mock_multi:
            result = manager.send_arm_ee_world(
                left_pose=left_pose,
                right_pose=right_pose,
                desire_time=3.0
            )

        assert result.success is True
        assert result.data == {'actual_time': 3.0}
        # 不再走 SDK 串行拆分路径
        mock_api.send_timed_cmd.assert_not_called()
        # 一次批量调用：左臂 planner 4 + 右臂 planner 5，同步执行
        mock_multi.assert_called_once()
        commands, kwargs = mock_multi.call_args[0], mock_multi.call_args[1]
        assert kwargs.get('is_sync') is True
        assert len(commands[0]) == 2
        assert commands[0][0] == {'planner_index': 4, 'desire_time': 3.0, 'cmd_vec': left_pose}
        assert commands[0][1] == {'planner_index': 5, 'desire_time': 3.0, 'cmd_vec': right_pose}

    @patch('adapters.hardware.leju_wheeled.services.sdk_manager.timed_cmd_manager.TimedCmdAPI')
    @patch('adapters.hardware.leju_wheeled.services.sdk_manager.base_sdk_manager.KuavoSDK')
    def test_send_arm_ee_local(self, mock_sdk_class, mock_api_class):
        """测试手臂末端局部系控制（一次批量调用，planner 6+7，isSync=True）"""
        mock_api = Mock()
        mock_api.send_timed_cmd.return_value = (True, 3.0)
        mock_api_class.return_value = mock_api

        manager = TimedCmdManager()
        # 绕开 initialize()（真实 KuavoSDK.Init 会等 ROS 服务超时），直接置就绪状态
        manager._initialized = True
        manager.robot_sdk = Mock()
        manager.timed_cmd_api = mock_api

        left_pose = [0.5, 0.3, 0.3, 0.0, 0.1, 0.0]
        right_pose = [0.5, -0.3, 0.3, 0.0, 0.1, 0.0]

        # 捕获 send_multi_commands 的调用参数（send_arm_ee_local 应委托给它）
        with patch.object(manager, 'send_multi_commands',
                          return_value=Result.ok(data={'actual_time': 3.0})) as mock_multi:
            result = manager.send_arm_ee_local(
                left_pose=left_pose,
                right_pose=right_pose,
                desire_time=3.0
            )

        assert result.success is True
        assert result.data == {'actual_time': 3.0}
        # 不再走 SDK 串行拆分路径
        mock_api.send_timed_cmd.assert_not_called()
        # 一次批量调用：左臂 planner 6 + 右臂 planner 7，同步执行
        mock_multi.assert_called_once()
        commands, kwargs = mock_multi.call_args[0], mock_multi.call_args[1]
        assert kwargs.get('is_sync') is True
        assert len(commands[0]) == 2
        assert commands[0][0] == {'planner_index': 6, 'desire_time': 3.0, 'cmd_vec': left_pose}
        assert commands[0][1] == {'planner_index': 7, 'desire_time': 3.0, 'cmd_vec': right_pose}


class TestTimedCmdManagerArmForce:
    """测试手臂力控"""

    @patch('adapters.hardware.leju_wheeled.services.sdk_manager.timed_cmd_manager.TimedCmdAPI')
    @patch('adapters.hardware.leju_wheeled.services.sdk_manager.base_sdk_manager.KuavoSDK')
    def test_send_arm_force(self, mock_sdk_class, mock_api_class):
        """测试手臂力控"""
        mock_sdk = Mock()
        mock_sdk_class.return_value = mock_sdk
        
        mock_api = Mock()
        mock_api.send_timed_cmd.return_value = (True, 2.0)
        mock_api_class.return_value = mock_api
        
        manager = TimedCmdManager()
        manager.initialize()
        
        force = [10.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # [fx, fy, fz, tx, ty, tz]
        
        result = manager.send_arm_force(
            force=force,
            desire_time=2.0
        )
        
        assert result.success is True
        mock_api.send_timed_cmd.assert_called_once_with(
            cmd_type='arm_force',
            cmd_vec=force,
            desire_time=2.0
        )


class TestTimedCmdManagerErrorHandling:
    """测试错误处理"""

    @patch('adapters.hardware.leju_wheeled.services.sdk_manager.timed_cmd_manager.TimedCmdAPI')
    @patch('adapters.hardware.leju_wheeled.services.sdk_manager.base_sdk_manager.KuavoSDK')
    def test_not_initialized(self, mock_sdk_class, mock_api_class):
        """测试未初始化时调用"""
        manager = TimedCmdManager()
        
        result = manager.send_chassis_world(
            x=0.5, y=0.0, yaw=0.0, desire_time=3.0
        )
        
        assert result.success is False
        assert "未初始化" in result.message

    @patch('adapters.hardware.leju_wheeled.services.sdk_manager.timed_cmd_manager.TimedCmdAPI')
    @patch('adapters.hardware.leju_wheeled.services.sdk_manager.base_sdk_manager.KuavoSDK')
    def test_exception_handling(self, mock_sdk_class, mock_api_class):
        """测试异常处理"""
        mock_sdk = Mock()
        mock_sdk_class.return_value = mock_sdk
        
        mock_api = Mock()
        mock_api.send_timed_cmd.side_effect = Exception("通信错误")
        mock_api_class.return_value = mock_api
        
        manager = TimedCmdManager()
        manager.initialize()
        
        result = manager.send_chassis_world(
            x=0.5, y=0.0, yaw=0.0, desire_time=3.0
        )
        
        assert result.success is False
        assert "通信错误" in result.message


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
