from abc import ABC, abstractmethod
from typing import List, Dict, Optional, Union
from ..domain.result import Result
from ..domain.pose import Pose6D
from ..domain.enums import FrameType, MPCControlMode, ArmSide
from ..domain.end_effector import GripperCommand, HandFingerCommand

class IHardware(ABC):
    """
    乐聚轮臂机器人硬件抽象层接口。
    屏蔽底层 ROS 话题发布或 SDK 调用的差异，为上层 Skill 提供统一控制入口。
    """

    # --- 1. 连接管理 ---
    @abstractmethod
    def initialize(self) -> Result:
        """初始化硬件连接（如启动 ROS Node 或连接 SDK）"""
        pass

    @abstractmethod
    def shutdown(self) -> Result:
        """断开连接并释放资源"""
        pass

    @property
    @abstractmethod
    def is_connected(self) -> bool:
        """检查当前连接状态"""
        pass

    # --- 2. 底盘与躯干控制 ---
    @abstractmethod
    def send_base_velocity(self, vx: float, vy: float, vyaw: float, frame: FrameType = FrameType.LOCAL) -> Result:
        """底盘速度控制。支持本体坐标系和世界坐标系。

        .. note:: 角度单位由适配器的 ``angle_unit`` 配置决定，默认为度（deg）。Pose6D 对象内部始终使用弧度。
        """
        pass

    @abstractmethod
    def send_base_pose(self, x: float, y: float, yaw: float, frame: FrameType = FrameType.WORLD) -> Result:
        """底盘位置控制。

        .. note:: 角度单位由适配器的 ``angle_unit`` 配置决定，默认为度（deg）。Pose6D 对象内部始终使用弧度。
        """
        pass

    @abstractmethod
    def send_torso_pose(self, pose: Pose6D) -> Result:
        """躯干相对基座的位姿控制 (x, z, pitch, yaw)。"""
        pass

    @abstractmethod
    def get_torso_initial_pose(self) -> Result:
        """获取躯干初始位姿。

        Returns:
            Result: 成功时 data 包含 {'position': [x,y,z], 'euler': [yaw,pitch,roll]}
        """
        pass

    # --- 3. 手臂运动控制 ---
    @abstractmethod
    def send_ee_pose(self, side: ArmSide, pose: Pose6D, frame: FrameType = FrameType.WORLD) -> Result:
        """单臂手部笛卡尔位姿控制。
        
        :param side: 手臂侧 (LEFT/RIGHT)
        :param pose: 目标位姿
        :param frame: 坐标系类型
        :return: Result
        """
        pass

    @abstractmethod
    def send_both_ee_poses(self, left_pose: Pose6D, right_pose: Pose6D, frame: FrameType = FrameType.WORLD) -> Result:
        """双臂手部笛卡尔位姿控制（同时控制）。
        
        :param left_pose: 左手目标位姿
        :param right_pose: 右手目标位姿
        :param frame: 坐标系类型 (WORLD/LOCAL)
        :return: Result
        """
        pass

    @abstractmethod
    def send_arm_joint_trajectory(self, positions: List[float], time_sec: float = 0.0) -> Result:
        """手臂关节轨迹控制 (14个自由度)。

        .. note:: 角度单位由适配器的 ``angle_unit`` 配置决定，默认为度（deg）。Pose6D 对象内部始终使用弧度。
        """
        pass

    @abstractmethod
    def send_leg_joint_command(self, positions: List[float]) -> Result:
        """腿部关节控制 (4个自由度)。

        .. note:: 角度单位由适配器的 ``angle_unit`` 配置决定，默认为度（deg）。Pose6D 对象内部始终使用弧度。
        """
        pass

    # --- 3.5 时序指令控制（高级功能）---
    @abstractmethod
    def send_timed_base_pose(self, x: float, y: float, yaw: float, 
                             desire_time: float, frame: FrameType = FrameType.WORLD) -> Result:
        """
        发送带时间参数的底盘位置指令。

        :param x: X轴目标位置（米）
        :param y: Y轴目标位置（米）
        :param yaw: 偏航角目标
        :param desire_time: 期望执行时间（秒）
        :param frame: 坐标系类型（WORLD=0, LOCAL=1）
        :return: Result，包含实际执行时间

        .. note:: 角度单位由适配器的 ``angle_unit`` 配置决定，默认为度（deg）。Pose6D 对象内部始终使用弧度。
        """
        pass

    @abstractmethod
    def send_timed_torso_pose(self, x: float, z: float, yaw: float, pitch: float,
                              desire_time: float) -> Result:
        """
        发送带时间参数的躯干位姿指令。

        :param x: X轴位移（米）
        :param z: Z轴位移（米）
        :param yaw: 偏航角
        :param pitch: 俯仰角
        :param desire_time: 期望执行时间（秒）
        :return: Result，包含实际执行时间

        .. note:: 角度单位由适配器的 ``angle_unit`` 配置决定，默认为度（deg）。Pose6D 对象内部始终使用弧度。
        """
        pass

    @abstractmethod
    def send_timed_leg_joint(self, joint_angles: List[float], 
                             desire_time: float) -> Result:
        """
        发送带时间参数的腿部关节指令。

        :param joint_angles: 关节角度列表 [j1, j2, j3, j4]
        :param desire_time: 期望执行时间（秒）
        :return: Result，包含实际执行时间

        .. note:: 角度单位由适配器的 ``angle_unit`` 配置决定，默认为度（deg）。Pose6D 对象内部始终使用弧度。
        """
        pass

    @abstractmethod
    def send_timed_left_arm_joint(self, joint_angles: List[float], 
                                  desire_time: float) -> Result:
        """
        发送带时间参数的左臂关节指令。

        :param joint_angles: 左臂7个关节角度
                           顺序: [肩俯仰, 肩侧摆, 肩偏航, 肘俯仰, 腕偏航, 腕俯仰, 腕滚转]
        :param desire_time: 期望执行时间（秒）
        :return: Result，包含实际执行时间

        .. note:: 角度单位由适配器的 ``angle_unit`` 配置决定，默认为度（deg）。Pose6D 对象内部始终使用弧度。
        """
        pass

    @abstractmethod
    def send_timed_right_arm_joint(self, joint_angles: List[float], 
                                   desire_time: float) -> Result:
        """
        发送带时间参数的右臂关节指令。

        :param joint_angles: 右臂7个关节角度
                           顺序: [肩俯仰, 肩侧摆, 肩偏航, 肘俯仰, 腕偏航, 腕俯仰, 腕滚转]
        :param desire_time: 期望执行时间（秒）
        :return: Result，包含实际执行时间

        .. note:: 角度单位由适配器的 ``angle_unit`` 配置决定，默认为度（deg）。Pose6D 对象内部始终使用弧度。
        """
        pass

    @abstractmethod
    def send_timed_multi_commands(self, commands: List[dict], 
                                  is_sync: bool = False) -> Result:
        """
        发送多条定时指令到移动机械臂（并发控制）。

        :param commands: 定时指令列表，每个元素为字典格式：
            {
                'planner_index': int,    # 规划器索引 (0-9)
                'desire_time': float,    # 期望执行时间（秒）
                'cmd_vec': List[float]   # 命令向量列表
            }
        :param is_sync: 多个规划器是否做时间同步
                        True: 同步模式，所有指令同时完成
                        False: 异步模式，各指令按各自时间执行
        :return: Result，包含实际执行时间和详细信息

        规划器索引说明：
            0: 底盘世界系位置运动 (3维: x, y, yaw)
            1: 底盘局部系位置运动 (3维: x, y, yaw)
            2: 躯干笛卡尔局部系运动 (4维: x, z, yaw, pitch)
            3: 下肢关节运动 (4维: joint1-4)
            4: 左臂笛卡尔世界系运动 (6维: x,y,z,yaw,pitch,roll)
            5: 右臂笛卡尔世界系运动 (6维: x,y,z,yaw,pitch,roll)
            6: 左臂笛卡尔局部系运动 (6维: x,y,z,yaw,pitch,roll)
            7: 右臂笛卡尔局部系运动 (6维: x,y,z,yaw,pitch,roll)
            8: 左臂上肢关节运动 (7维: joint1-7)
            9: 右臂上肢关节运动 (7维: joint1-7)

        .. note:: 角度单位由适配器的 ``angle_unit`` 配置决定，默认为度（deg）。Pose6D 对象内部始终使用弧度。
        """
        pass

    @abstractmethod
    def set_ruckig_planner_params(self, planner_index: int,
                                  is_sync: bool,
                                  velocity_max: List[float],
                                  acceleration_max: List[float],
                                  jerk_max: List[float],
                                  velocity_min: List[float] = None,
                                  acceleration_min: List[float] = None) -> Result:
        """
        设置Ruckig规划器参数（速度/加速度/急动度限制）。
        
        :param planner_index: 规划器索引 (0-9)
        :param is_sync: 是否同步模式
        :param velocity_max: 最大速度列表（需与规划器自由度匹配）
        :param acceleration_max: 最大加速度列表（需与规划器自由度匹配）
        :param jerk_max: 最大急动度列表（需与规划器自由度匹配）
        :param velocity_min: 最小速度列表（可选，默认为 -velocity_max）
        :param acceleration_min: 最小加速度列表（可选，默认为 -acceleration_max）
        :return: Result，包含设置结果和详细消息
        
        规划器索引说明：
            0: 底盘位置运动 (3维)
            1: 底盘速度运动 (3维)
            2: 躯干笛卡尔运动 (4维: x, z, yaw, pitch)
            3: 下肢关节运动 (4维)
            4: 左臂笛卡尔运动 (6维)
            5: 右臂笛卡尔运动 (6维)
            6: 左臂关节运动 (7维)
            7: 右臂关节运动 (7维)
            8: 左臂上肢关节运动 (7维) - 与时序指令相同
            9: 右臂上肢关节运动 (7维) - 与时序指令相同
        """
        pass

    @abstractmethod
    def set_offline_trajectory(self, trajectories: List[dict]) -> Result:
        """
        设置多条离线定时轨迹到移动机械臂（预定义复杂轨迹）。
        
        :param trajectories: 离线轨迹列表，每个元素为字典格式：
            {
                'planner_index': int,       # 规划器索引 (0:左臂笛卡尔, 1:右臂笛卡尔, 2:躯干)
                'frame': int,               # 坐标系 (0:世界系, 1:局部系)
                'timed_traj': List[dict]    # 定时轨迹点列表
            }
            其中 timed_traj 的每个点为：
            {
                'desire_time': float,       # 期望执行时间(秒), 第一帧必须为0
                'cmd_vec': List[float]      # 命令向量，左/右臂6维, 躯干4维
            }
        :return: Result，包含设置结果和详细消息
        
        注意事项：
        - 第一帧时间必须为0
        - 时间必须严格递增
        - 命令向量维度必须与规划器匹配
        - 设置后需调用 enable_offline_trajectory() 启动执行
        """
        pass

    @abstractmethod
    def enable_offline_trajectory(self, enable: bool) -> Result:
        """
        启用或禁用离线轨迹功能。
        
        :param enable: True启用离线轨迹，False禁用离线轨迹
        :return: Result，包含设置结果和详细消息
        """
        pass

    @abstractmethod
    def check_ik_accessibility(self, is_left: bool,
                               is_local: bool,
                               is_whole_body: bool,
                               pose_desired: List[float],
                               total_time_desired: float = 1.0,
                               max_attempts: int = 5,
                               linear_error_max: float = 0.005,
                               angular_error_max: float = 0.05) -> Result:
        """
        检查移动机械臂的目标位姿是否可达（IK逆运动学求解）。
        
        :param is_left: 是否为左臂 (True: 左臂, False: 右臂)
        :param is_local: 是否使用局部坐标系 (True: 局部, False: 世界)
        :param is_whole_body: 是否使用全身运动 (True: 全身, False: 仅手臂)
        :param pose_desired: 期望位姿 [x, y, z, roll, pitch, yaw] (6维向量)
        :param total_time_desired: 期望求解总时间（秒）
        :param max_attempts: 最大求解尝试次数
        :param linear_error_max: 最大允许线位移误差（米）
        :param angular_error_max: 最大允许角位移误差（弧度）
        :return: Result，包含可达性检查结果和IK解
                 data字段包含:
                 - success: 是否可达（精确IK解满足误差要求）
                 - best_linear_error: 最优解的线位移误差
                 - best_angular_error: 最优解的角位移误差
                 - q_best: 最优关节角度（精确IK解）
                 - pos_priority_access: 位置优先零空间解是否满足要求
                 - pos_priority_linear_error: 位置优先解的线位移误差
                 - pos_priority_angular_error: 位置优先解的角位移误差
                 - q_pos_priority_best: 位置优先解的关节角度
        """
        pass

    # --- 4. 模式与服务调用 ---
    @abstractmethod
    def set_mpc_mode(self, mode: MPCControlMode) -> Result:
        """切换移动操作机器人的 MPC 控制模式。"""
        pass

    @abstractmethod
    def enable_quick_mode(self, enable: bool) -> Result:
        """启用或禁用手臂/下肢快速模式（绕过 MPC 直接控制电机）。"""
        pass

    # --- 5. 末端执行器控制 ---
    @abstractmethod
    def control_end_effector(self, side: ArmSide, cmd: Union[GripperCommand, HandFingerCommand]) -> Result:
        """统一控制末端执行器（夹爪或灵巧手）。"""
        pass

    # --- 5.5 头部控制 ---
    @abstractmethod
    def control_head(self, yaw: float, pitch: float) -> Result:
        """
        控制头部姿态

        :param yaw: 偏航角
        :param pitch: 俯仰角
        :return: Result

        .. note:: 角度单位由适配器的 ``angle_unit`` 配置决定，默认为度（deg）。Pose6D 对象内部始终使用弧度。
        """
        pass

    # --- 5.6 手臂管理 ---
    @abstractmethod
    def arm_reset(self) -> Result:
        """
        手臂归位到初始姿态
        
        :return: Result
        """
        pass

    # --- 5.7 末端期望力控制 ---
    @abstractmethod
    def set_ee_force(self, side: ArmSide,
                     force_kg: tuple = (0.0, 0.0, 0.0),
                     torque: tuple = (0.0, 0.0, 0.0)) -> Result:
        """
        设置末端期望力（3D 力 + 3D 力矩）

        通过 ROS 话题 /desired_ee_force/{left,right} 发布 WrenchStamped。
        对齐源脚本 LBForceController.set_desired_ee_force()。
        力的方向以机器人本体为参考坐标系。

        :param side: 手臂侧 (LEFT / RIGHT / BOTH)
        :param force_kg: 3D 力向量 (fx, fy, fz)，单位 kg，内部 ×9.8 转 N
        :param torque: 3D 力矩向量 (tx, ty, tz)，单位 Nm
        :return: Result
        """
        pass

    @abstractmethod
    def set_ee_force_both(self,
                          left_force_kg: tuple = (0.0, 0.0, 0.0),
                          right_force_kg: tuple = (0.0, 0.0, 0.0),
                          left_torque: tuple = (0.0, 0.0, 0.0),
                          right_torque: tuple = (0.0, 0.0, 0.0)) -> Result:
        """
        分别设置左右手末端期望力

        :param left_force_kg: 左手 3D 力向量 (fx, fy, fz)，单位 kg
        :param right_force_kg: 右手 3D 力向量 (fx, fy, fz)，单位 kg
        :param left_torque: 左手 3D 力矩向量 (tx, ty, tz)，单位 Nm
        :param right_torque: 右手 3D 力矩向量 (tx, ty, tz)，单位 Nm
        :return: Result
        """
        pass

    @abstractmethod
    def clear_ee_force(self, side: ArmSide = None) -> Result:
        """
        清除末端期望力（设为零）

        对齐源脚本 LBForceController.clear_desired_ee_force()。

        :param side: 手臂侧 (LEFT / RIGHT / BOTH)，None 表示双手
        :return: Result
        """
        pass

    @abstractmethod
    def set_external_wrench(self, side: ArmSide,
                            force_n: tuple = (0.0, 0.0, 0.0),
                            torque: tuple = (0.0, 0.0, 0.0)) -> Result:
        """
        设置仿真外力

        通过 ROS 话题 /external_wrench/{left_hand,right_hand} 发布 Wrench。
        对齐源脚本 LBForceController.set_external_wrench()。

        :param side: 手臂侧 (LEFT / RIGHT / BOTH)
        :param force_n: 3D 力向量 (fx, fy, fz)，单位 N
        :param torque: 3D 力矩向量 (tx, ty, tz)，单位 Nm
        :return: Result
        """
        pass

    @abstractmethod
    def clear_external_wrench(self, side: ArmSide = None) -> Result:
        """
        清除仿真外力

        :param side: 手臂侧 (LEFT / RIGHT / BOTH)，None 表示双手
        :return: Result
        """
        pass

    @abstractmethod
    def enable_force_empty_detect(self, enable: bool) -> Result:
        """
        启用或禁用挥空检测

        通过 ROS 话题 /enable_force_empty_detact 发布 Bool（latch）。
        对齐源脚本 LBForceController.enable_force_empty_detact()。

        :param enable: True=启用, False=禁用
        :return: Result
        """
        pass

    @abstractmethod
    def set_contact_force_params(self, transition_time: float,
                                 interpolation_speed: float) -> Result:
        """
        设置接触力插值参数

        通过 ROS 服务 /set_contact_force_params 配置。
        对齐源脚本 LBForceController.set_contact_force_params()。

        :param transition_time: 过渡时间（秒）
        :param interpolation_speed: 插值速度（N/s）
        :return: Result
        """
        pass

    # --- 6. 状态反馈 ---
    @abstractmethod
    def get_reach_time(self, topic_type: str) -> Optional[float]:
        """
        获取指令预计到达时间 (秒)。
        
        :param topic_type: 话题类型
            - 'cmd_pose': 底盘位置
            - 'torso_pose': 躯干位姿
            - 'arm_joint': 手臂关节
            - 'leg_joint': 腿部关节
            - 'arm_ee': 手臂末端
        :return: 预计到达时间（秒），如果未收到则返回 None
        """
        pass
    
    @abstractmethod
    def get_mpc_observation(self) -> Optional[Dict]:
        """获取MPC观测状态"""
        pass
    
    @abstractmethod
    def get_mpc_control_mode(self) -> Optional[int]:
        """获取当前MPC控制模式"""
        pass
    
    @abstractmethod
    def get_body_acceleration(self) -> Optional[Dict]:
        """获取本体加速度"""
        pass
    
    @abstractmethod
    def get_joint_torque(self) -> Optional[Dict]:
        """获取关节力矩"""
        pass
    
    @abstractmethod
    def get_ee_poses(self) -> Optional[Dict]:
        """获取末端执行器位姿"""
        pass
    