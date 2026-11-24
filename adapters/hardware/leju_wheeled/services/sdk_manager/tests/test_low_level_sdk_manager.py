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
from unittest.mock import patch, call
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


@pytest.fixture(autouse=True)
def close_test_managers(monkeypatch):
    """测试结束前释放Manager引用，避免析构日志写入pytest已关闭的捕获流。"""
    managers = []
    original_init = LowLevelSDKManager.__init__

    def tracked_init(manager):
        original_init(manager)
        managers.append(manager)

    monkeypatch.setattr(LowLevelSDKManager, "__init__", tracked_init)
    yield
    for manager in managers:
        if manager.is_initialized():
            manager.shutdown()


def _robot_sdk(sdk_mocks):
    """取出 fixture 里的 RobotSDK Mock 实例——代码中 control.* 都调在它身上。"""
    _, robot_sdk_cls = sdk_mocks
    return robot_sdk_cls.return_value


class TestTimedArmAPI:
    def test_lazy_native_api_reuses_robot_sdk_and_is_released(self, sdk_mocks):
        from unittest.mock import Mock
        from types import SimpleNamespace
        import sys
        manager = LowLevelSDKManager()
        manager.initialize()
        factory = Mock()
        factory.return_value.move_joint_traj.return_value = None
        with patch.dict(sys.modules, {'kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.api': SimpleNamespace(ArmAPI=factory)}):
            assert manager.set_arm_joint_traj([[0.1]*14]*2, 1.0).success
            assert manager.set_arm_joint_traj([[0.1]*14]*2, 1.0).success
        factory.assert_called_once_with(_robot_sdk(sdk_mocks))
        manager.shutdown()
        factory.return_value._pool.shutdown.assert_called_once_with(wait=True)

    def test_joint_calls_native_arm_api_and_none_is_completion(self, sdk_mocks):
        from unittest.mock import Mock
        manager = LowLevelSDKManager()
        manager.initialize()
        manager._arm_api = Mock()
        manager._arm_api.move_joint_traj.return_value = None
        frames = [[0.1]*14, [-0.2]*14]
        result = manager.set_arm_joint_traj(frames, 1.0)
        assert result.success and result.data is None
        from kuavo_humanoid_sdk.interfaces.data_types import KuavoManipulationMpcCtrlMode
        manager._arm_api.move_joint_traj.assert_called_once_with(
            joint_traj=frames, asynchronous=False, total_time=1.0,
            mpc_ctrl_mode=KuavoManipulationMpcCtrlMode.ArmOnly)

    def test_pose_native_flags_and_quaternion_sign_alignment(self, sdk_mocks):
        from unittest.mock import Mock
        from kuavo_humanoid_sdk.interfaces.data_types import KuavoPose, KuavoManipulationMpcFrame, KuavoManipulationMpcCtrlMode
        manager = LowLevelSDKManager()
        manager.initialize()
        manager._arm_api = Mock()
        manager._arm_api.move_eef_traj_wheel_mpc.return_value = None
        poses = [KuavoPose((1,0,0), (0,0,0,1)), KuavoPose((2,0,0), (0,0,0,-1))]
        result = manager.set_arm_pose_traj(poses, poses, 1.0, direct_to_wbc=False, back_default=True)
        assert result.success
        args = manager._arm_api.move_eef_traj_wheel_mpc.call_args.kwargs
        assert args['frame'] == KuavoManipulationMpcFrame.LocalFrame
        assert args['mpc_ctrl_mode'] == KuavoManipulationMpcCtrlMode.ArmOnly
        assert args['asynchronous'] is False and args['direct_to_wbc'] is False and args['back_default'] is True
        assert args['left_traj'][1].orientation == (0,0,0,1)
        assert poses[1].orientation == (0,0,0,-1)  # 不修改输入。

    def test_sdk_exception_becomes_failure_and_pool_released(self, sdk_mocks):
        from unittest.mock import Mock
        manager = LowLevelSDKManager()
        manager.initialize()
        api = manager._arm_api = Mock()
        api.move_joint_traj.side_effect = RuntimeError('native failure')
        assert not manager.set_arm_joint_traj([[0.1]*14]*2, 1.0).success
        assert manager.shutdown().success
        api._pool.shutdown.assert_called_once_with(wait=True)
        assert manager._arm_api is None

    def test_uninitialized_or_invalid_input_never_creates_api(self, sdk_mocks):
        manager = LowLevelSDKManager()
        assert not manager.set_arm_joint_traj([[0.1]*14]*2, 1.0).success
        manager.initialize()
        assert not manager.set_arm_joint_traj([[0.1]*14], 1.0).success
        assert not manager.set_arm_pose_traj([], [], 1.0).success
        assert manager._arm_api is None


class TestArmKinematics:
    def test_fk_given_joint_does_not_read_feedback(self, sdk_mocks):
        from types import SimpleNamespace
        sdk = _robot_sdk(sdk_mocks)
        sdk.control.arm_fk.return_value = (SimpleNamespace(position=(1, 0, 0), orientation=(0, 0, 0, 1)),) * 2
        sdk.tools.get_tf_transform.return_value = SimpleNamespace(position=(0, 2, 0), orientation=(0, 0, np.sin(np.pi/4), np.cos(np.pi/4)))
        manager = LowLevelSDKManager()
        manager.initialize()
        result = manager.get_arm_fk_result([0.1] * 14, frame="world", fk_frame="torso")
        assert result.success
        assert result.data["left_pose"][:3] == pytest.approx([0, 3, 0])
        sdk.control.arm_fk.assert_called_once_with([0.1] * 14)
        sdk.state.arm_joint_state.assert_not_called()
        sdk.tools.get_tf_transform.assert_called_once_with(target_frame="odom", source_frame="torso", return_type="pose_quaternion")

    def test_ik_transforms_targets_into_model_and_returns_radians(self, sdk_mocks):
        from types import SimpleNamespace
        sdk = _robot_sdk(sdk_mocks)
        sdk.control.arm_ik.return_value = [0.2] * 14
        sdk.tools.get_tf_transform.return_value = SimpleNamespace(position=(0, -2, 0), orientation=(0, 0, 0, 1))
        pose = SimpleNamespace(position=(1, 2, 3), orientation=(0, 0, 0, 1))
        manager = LowLevelSDKManager()
        manager.initialize()
        result = manager.get_arm_ik_result(pose, pose, frame="world", ik_frame="torso", joint=[0.1] * 14)
        assert result.data == [0.2] * 14
        kwargs = sdk.control.arm_ik.call_args.kwargs
        assert kwargs["left_pose"].position == (1, 0, 3)
        assert kwargs["arm_q0"] == [0.1] * 14
        assert kwargs["params"] is not None  # SDK下层不能接收params=None。
        sdk.tools.get_tf_transform.assert_called_once_with(target_frame="torso", source_frame="odom", return_type="pose_quaternion")
        sdk.control.control_arm_joint_positions.assert_not_called()
        sdk.control.set_external_control_arm_mode.assert_not_called()

    @pytest.mark.parametrize("solution", [None, [0.0] * 13, [np.nan] * 14])
    def test_invalid_ik_solution_fails(self, sdk_mocks, solution):
        from types import SimpleNamespace
        sdk = _robot_sdk(sdk_mocks)
        sdk.control.arm_ik.return_value = solution
        pose = SimpleNamespace(position=(1, 0, 0), orientation=(0, 0, 0, 1))
        manager = LowLevelSDKManager()
        manager.initialize()
        assert not manager.get_arm_ik_result(pose, pose, ik_frame="base_link").success

    def test_missing_model_frame_or_transform_never_calls_solver(self, sdk_mocks):
        from types import SimpleNamespace
        sdk = _robot_sdk(sdk_mocks)
        manager = LowLevelSDKManager()
        manager.initialize()
        assert not manager.get_arm_fk_result([0.1]*14).success
        pose = SimpleNamespace(position=(1, 0, 0), orientation=(0, 0, 0, 1))
        assert not manager.get_arm_ik_result(pose, pose).success
        sdk.tools.get_tf_transform.return_value = None
        assert not manager.get_arm_ik_result(pose, pose, ik_frame="torso").success
        sdk.control.arm_ik.assert_not_called()
        sdk.control.arm_fk.assert_not_called()


class TestArmControlMode:
    @pytest.mark.parametrize("mode,method", [(0, "set_fixed_arm_mode"), (1, "set_auto_swing_arm_mode"), (2, "set_external_control_arm_mode")])
    def test_set_mode_calls_only_corresponding_sdk_method(self, sdk_mocks, mode, method):
        sdk = _robot_sdk(sdk_mocks)
        getattr(sdk.control, method).return_value = True
        manager = LowLevelSDKManager()
        assert manager.initialize().success
        result = manager.set_arm_control_mode(mode)
        assert result.success and result.data is None
        assert sdk.control.mock_calls == [getattr(call, method)()]
        sdk.state.arm_control_mode.assert_not_called()

    @pytest.mark.parametrize("returned", [False, None])
    def test_set_mode_failure(self, sdk_mocks, returned):
        sdk = _robot_sdk(sdk_mocks)
        sdk.control.set_external_control_arm_mode.return_value = returned
        manager = LowLevelSDKManager()
        manager.initialize()
        assert not manager.set_arm_control_mode(2).success

    def test_set_mode_exception_and_missing_initialization(self, sdk_mocks):
        sdk = _robot_sdk(sdk_mocks)
        manager = LowLevelSDKManager()
        assert not manager.set_arm_control_mode(2).success
        assert not sdk.control.mock_calls
        manager.initialize()
        sdk.control.set_external_control_arm_mode.side_effect = RuntimeError("collision")
        result = manager.set_arm_control_mode(2)
        assert not result.success and "collision" in result.message

    @pytest.mark.parametrize("mode", [None, True, "2", 2.0, -1, 3])
    def test_set_mode_invalid_input_does_not_call_sdk(self, sdk_mocks, mode):
        sdk = _robot_sdk(sdk_mocks)
        manager = LowLevelSDKManager()
        manager.initialize()
        assert not manager.set_arm_control_mode(mode).success
        assert not sdk.control.mock_calls

    def test_set_mode_full_hardware_skill_chain(self, sdk_mocks):
        from adapters.hardware.leju_wheeled.hardware import LejuWheeledArmHardware
        from skills.atomic.refactored_sdk.set_arm_control_mode import SetArmControlModeSkill, SetArmControlModeParams
        sdk = _robot_sdk(sdk_mocks)
        sdk.control.set_fixed_arm_mode.return_value = True
        manager = LowLevelSDKManager()
        manager.initialize()
        hardware = object.__new__(LejuWheeledArmHardware)
        hardware._low_level_sdk_manager = manager
        skill = SetArmControlModeSkill(hardware)
        assert skill.initialize(SetArmControlModeParams(control_mode=0)).success
        assert skill.execute().success
        sdk.control.set_fixed_arm_mode.assert_called_once_with()

    @pytest.mark.parametrize("value,name", [(0, "ArmFixed"), (1, "AutoSwing"), (2, "ExternalControl")])
    def test_reads_sdk_enum_without_setting_mode(self, sdk_mocks, value, name):
        from kuavo_humanoid_sdk.interfaces.data_types import KuavoArmCtrlMode
        sdk = _robot_sdk(sdk_mocks)
        sdk.state.arm_control_mode.return_value = KuavoArmCtrlMode(value)
        manager = LowLevelSDKManager()
        assert manager.initialize().success
        result = manager.get_arm_control_mode()
        assert result.success
        assert result.data == {"mode": value, "name": name}
        sdk.state.arm_control_mode.assert_called_once_with()
        assert not sdk.control.mock_calls

    @pytest.mark.parametrize("value", [None, 0, True, "ExternalControl", -1])
    def test_rejects_missing_or_invalid_sdk_mode(self, sdk_mocks, value):
        sdk = _robot_sdk(sdk_mocks)
        sdk.state.arm_control_mode.return_value = value
        manager = LowLevelSDKManager()
        manager.initialize()
        assert not manager.get_arm_control_mode().success

    def test_sdk_exception_and_uninitialized(self, sdk_mocks):
        sdk = _robot_sdk(sdk_mocks)
        manager = LowLevelSDKManager()
        assert not manager.get_arm_control_mode().success
        sdk.state.arm_control_mode.assert_not_called()
        manager.initialize()
        sdk.state.arm_control_mode.side_effect = RuntimeError("query failed")
        result = manager.get_arm_control_mode()
        assert not result.success and "query failed" in result.message


class TestArmFeedbackPose:
    def test_joint_read_uses_state(self, sdk_mocks):
        sdk = _robot_sdk(sdk_mocks)
        sdk.state.arm_joint_state.return_value.position = [0.1] * 14
        manager = LowLevelSDKManager()
        manager.initialize()
        assert manager.get_arm_joint().data == [0.1] * 14
        sdk.state.arm_joint_state.assert_called_once()

    @pytest.mark.parametrize("position", [None, [0.0] * 13, [np.nan] * 14])
    def test_invalid_joint_feedback(self, sdk_mocks, position):
        sdk = _robot_sdk(sdk_mocks)
        sdk.state.arm_joint_state.return_value.position = position
        manager = LowLevelSDKManager()
        manager.initialize()
        assert not manager.get_arm_joint().success

    def test_tf_reads_both_links_in_requested_frame(self, sdk_mocks):
        from types import SimpleNamespace
        sdk = _robot_sdk(sdk_mocks)
        sdk.tools.get_link_pose.return_value = SimpleNamespace(position=(0.1, 0.2, 0.3), orientation=(0, 0, 0, 1))
        manager = LowLevelSDKManager()
        manager.initialize()
        result = manager.get_arm_pose()
        assert result.success and result.data["frame"] == "local"
        assert sdk.tools.get_link_pose.call_args_list[0].kwargs == {"link_name": "zarm_l7_end_effector", "reference_frame": "base_link"}
        assert sdk.tools.get_link_pose.call_args_list[1].kwargs == {"link_name": "zarm_r7_end_effector", "reference_frame": "base_link"}
        sdk.control.arm_fk.assert_not_called()

    def test_fk_transforms_from_explicit_model_frame(self, sdk_mocks):
        from types import SimpleNamespace
        sdk = _robot_sdk(sdk_mocks)
        sdk.state.arm_joint_state.return_value.position = [0.1] * 14
        sdk.control.arm_fk.return_value = (SimpleNamespace(position=(1, 0, 0), orientation=(0, 0, 0, 1)),) * 2
        # 非平凡旋转+平移，验证变换方向，不只验证单位阵。
        sdk.tools.get_tf_transform.return_value = SimpleNamespace(position=(0, 2, 0), orientation=(0, 0, np.sin(np.pi/4), np.cos(np.pi/4)))
        manager = LowLevelSDKManager()
        manager.initialize()
        result = manager.get_arm_pose(frame="world", source="fk", fk_frame="torso")
        assert result.success
        assert result.data["left_pose"][:3] == pytest.approx([0, 3, 0])
        sdk.control.arm_fk.assert_called_once_with([0.1] * 14)
        sdk.tools.get_tf_transform.assert_called_once_with(target_frame="odom", source_frame="torso", return_type="pose_quaternion")

    def test_fk_same_frame_does_not_require_tf(self, sdk_mocks):
        from types import SimpleNamespace
        sdk = _robot_sdk(sdk_mocks)
        sdk.state.arm_joint_state.return_value.position = [0.1] * 14
        sdk.control.arm_fk.return_value = (SimpleNamespace(position=(1, 0, 0), orientation=(0, 0, 0, 1)),) * 2
        manager = LowLevelSDKManager()
        manager.initialize()
        assert manager.get_arm_pose(source="fk", fk_frame="base_link").success
        sdk.tools.get_tf_transform.assert_not_called()

    def test_missing_source_does_not_fallback(self, sdk_mocks):
        sdk = _robot_sdk(sdk_mocks)
        sdk.tools.get_link_pose.return_value = None
        manager = LowLevelSDKManager()
        manager.initialize()
        assert not manager.get_arm_pose().success
        sdk.control.arm_fk.assert_not_called()
        assert not manager.get_arm_pose(source="fk").success
        assert not manager.get_arm_pose(source="wrong").success

    def test_fk_failure_or_missing_transform_returns_failure(self, sdk_mocks):
        from types import SimpleNamespace
        sdk = _robot_sdk(sdk_mocks)
        sdk.state.arm_joint_state.return_value.position = [0.1] * 14
        sdk.control.arm_fk.return_value = (None, None)
        manager = LowLevelSDKManager()
        manager.initialize()
        assert not manager.get_arm_pose(source="fk", fk_frame="base_link").success
        sdk.control.arm_fk.return_value = (SimpleNamespace(position=(1, 0, 0), orientation=(0, 0, 0, 1)),) * 2
        sdk.tools.get_tf_transform.return_value = None
        assert not manager.get_arm_pose(frame="world", source="fk", fk_frame="torso").success

    def test_feedback_methods_require_initialized_sdk(self):
        manager = LowLevelSDKManager()
        assert not manager.get_arm_joint().success
        assert not manager.get_arm_pose().success

    def test_invalid_quaternion_is_rejected(self, sdk_mocks):
        from types import SimpleNamespace
        sdk = _robot_sdk(sdk_mocks)
        sdk.tools.get_link_pose.return_value = SimpleNamespace(position=(0, 0, 0), orientation=(0, 0, 0, 0))
        manager = LowLevelSDKManager()
        manager.initialize()
        assert not manager.get_arm_pose().success




class TestHeadPose:
    def test_set_direct_sdk_call(self, sdk_mocks):
        sdk = _robot_sdk(sdk_mocks)
        sdk.control.control_head.return_value = True
        manager = LowLevelSDKManager()
        assert manager.initialize().success
        with patch.object(manager, "control_head", side_effect=AssertionError("旧入口不应调用")):
            assert manager.set_head_pose(0.1, -0.2).success
        sdk.control.control_head.assert_called_once_with(yaw=0.1, pitch=-0.2)

    def test_set_failure(self, sdk_mocks):
        sdk = _robot_sdk(sdk_mocks)
        sdk.control.control_head.return_value = False
        manager = LowLevelSDKManager()
        manager.initialize()
        assert not manager.set_head_pose(0.1, -0.2).success

    def test_get_feedback(self, sdk_mocks):
        sdk = _robot_sdk(sdk_mocks)
        sdk.state.head_joint_state.return_value.position = [0.1, -0.2]
        manager = LowLevelSDKManager()
        manager.initialize()
        assert manager.get_head_pose().data == {"yaw": 0.1, "pitch": -0.2}

    @pytest.mark.parametrize("position", [None, [], [0.0], [np.nan, 0.0], [0.0, np.inf]])
    def test_invalid_feedback(self, sdk_mocks, position):
        sdk = _robot_sdk(sdk_mocks)
        sdk.state.head_joint_state.return_value.position = position
        manager = LowLevelSDKManager()
        manager.initialize()
        assert not manager.get_head_pose().success

    def test_not_initialized(self):
        manager = LowLevelSDKManager()
        assert not manager.set_head_pose(0.0, 0.0).success
        assert not manager.get_head_pose().success


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
