# -*- coding: utf-8 -*-
"""NodeTagToArmGoal:算手臂关节轨迹(spec § 3.1 L3 决策)。

initialise() 内部按 keypoints_source="pick"/"lift" 调 generate_*_keypoints。
update() 用 tag.pose + keypoints → 逆运动学 → 写黑板 joint_traj 或 eef_traj。

实现策略:
- EEF 控制:直接从 SDK Pose 转为 Pose6D(纯格式转换,不依赖 IK 服务),写入
  left_arm_eef_traj / right_arm_eef_traj,由下游 NodeWheelArm → send_both_ee_poses
  处理。此路径与原始 case_wheel_pick_and_place.py 一致,稳定可靠。
- Joint 控制:尝试调 check_ik_accessibility 将 keypoint 位姿→7 关节角度,
  写入 left_arm_joint_traj / right_arm_joint_traj。如 IK 不可用,回退写 EEF 轨迹。

注:keypoints 在 BASE 系(局部坐标系),send_both_ee_poses 应使用 LOCAL frame。
"""
import os
import warnings

import py_trees
from py_trees.common import Status

from orchestration.nodes.base_node import BaseAction
from orchestration.nodes.utils.keypoints import (
    generate_pick_keypoints, generate_lift_keypoints,
)


def _is_dry_run() -> bool:
    return os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


class NodeTagToArmGoal(BaseAction):
    """调 keypoints + 写手臂轨迹。

    update() 从黑板读 tag + walk_goal，算出 tag 在 BASE 系的实际高度，
    将 keypoints 的 Z 整体平移到 tag 高度后再输出轨迹。
    """

    def __init__(self, name, label, namespace, params):
        super(NodeTagToArmGoal, self).__init__(name, label, namespace, params)
        self._tag_id = int(self.params.get("tag_id", 0))
        self._control_type = self.params.get("control_type", "joint")
        self._keypoints_source = self.params.get("keypoints_source", "pick")
        # 注册黑板 key(读 tag, 写 traj)
        for k in [
            "left_arm_joint_traj", "right_arm_joint_traj",
            "left_arm_eef_traj", "right_arm_eef_traj",
            f"latest_tag_{self._tag_id}", f"latest_tag_{self._tag_id}_version",
        ]:
            access = py_trees.common.Access.WRITE if "traj" in k else py_trees.common.Access.READ
            self.global_blackboard.register_key(key=k, access=access)
        self._keypoints = None          # (left_poses, right_poses), SDK Pose 列表
        self._tag_version_seen = -1

    def initialise(self):
        if self._keypoints_source == "pick":
            self._keypoints = generate_pick_keypoints(
                float(self.params.get("box_width", 0.0)),
                float(self.params.get("box_behind_tag", 0.0)),
                float(self.params.get("box_beneath_tag", 0.0)),
                float(self.params.get("box_left_tag", 0.0)),
                float(self.params.get("hand_pitch_degree", 0.0)),
            )
        else:  # "lift"
            self._keypoints = generate_lift_keypoints(
                float(self.params.get("box_width", 0.0)),
                float(self.params.get("box_behind_tag", 0.0)),
                float(self.params.get("box_beneath_tag", 0.0)),
                float(self.params.get("box_left_tag", 0.0)),
                float(self.params.get("z_lift", 0.2)),
            )
        self._tag_version_seen = -1

    # ------------------------------------------------------------------
    # update
    # ------------------------------------------------------------------

    def update(self):
        if _is_dry_run():
            return Status.SUCCESS

        # 检查 tag 版本:有新版本才生成轨迹
        version = getattr(
            self.global_blackboard, f"latest_tag_{self._tag_id}_version", None
        )
        if version is None or version == self._tag_version_seen:
            return Status.RUNNING
        self._tag_version_seen = version

        # 按 tag 在 BASE 系中的实际高度平移 keypoints
        # lift 不做 tag 锚定：lift = pick 终点 + z_lift（相对抬升）
        if self._keypoints_source == "lift":
            adjusted = self._make_lift_from_pick_end()
        else:
            adjusted = self._adjust_keypoints_to_tag()

        # ---- Joint 控制:尝试 IK ----
        if self._control_type == "joint":
            result = self._try_write_joint_traj(adjusted)
            if result == Status.SUCCESS:
                return Status.SUCCESS
            return Status.FAILURE

        # ---- EEF 控制:直接转换 ----
        return self._write_eef_traj(adjusted)

    # ------------------------------------------------------------------
    # Tag 高度 → keypoints Z 调整
    # ------------------------------------------------------------------

    def _get_tag_height_for_arm(self):
        """算 tag 的高度参考值，用于调整手臂目标 Z。

        用 tag.pose_in_world.z（= virtual_tag_odom_z - stand_in_tag_pos_z），
        避免 walk_goal 抵消 virtual_tag_odom_z 的变化。
        virtual_tag_pose_in_odom[2] 改变时，手臂高度随之改变。
        """
        tag = getattr(self.global_blackboard, f"latest_tag_{self._tag_id}", None)
        if tag is None:
            return None
        return tag.pose_in_world.z

    def _adjust_keypoints_to_tag(self):
        """以目标帧 Z 为锚点，将所有 keypoints 平移到 tag 高度。

        若 tag 在 BASE 系中的高度超过 0.8m，先通过 send_torso_pose
        抬升躯干，使手臂目标高度降至 0.7m，避免超出可达工作空间。

        dz = tag_z_in_base - 目标帧原始 Z，所有帧统一加 dz。
        """
        tag = getattr(self.global_blackboard, f"latest_tag_{self._tag_id}", None)
        if tag is None:
            return self._keypoints
        tag_z = tag.pose_in_world.z
        print(f"[TAG_POS] tag_id={self._tag_id} "
              f"x={tag.pose_in_world.x:.3f} y={tag.pose_in_world.y:.3f} z={tag_z:.3f} "
              f"roll={tag.pose_in_world.roll:.3f} pitch={tag.pose_in_world.pitch:.3f} yaw={tag.pose_in_world.yaw:.3f} "
              f"frame={tag.frame_id}")
        # 清零 tag 角度，避免真机检测的姿态角影响手臂目标
        tag.pose_in_world.roll = 0.0
        tag.pose_in_world.pitch = 0.0
        tag.pose_in_world.yaw = 0.0

        from kuavo_humanoid_sdk.kuavo_strategy_pytree.common.data_type import Pose

        HEIGHT_THRESHOLD = 0.8   # tag 超过此高度时抬升躯干（米）
        ARM_TARGET_Z = 0.7       # 抬升后手臂目标在 BASE 系中的高度（米）

        left_kp, right_kp = self._keypoints
        target_idx = self._target_frame_idx()
        anchor_z = float(left_kp[target_idx].pos[2])

        # 若 tag 高度超过阈值，抬升躯干使手臂目标高度降至 ARM_TARGET_Z
        torso_msg = ""
        if tag_z > HEIGHT_THRESHOLD:
            lift_z = tag_z - ARM_TARGET_Z
            try:
                from orchestration.shared_hardware import get_shared_hardware
                from core.domain.pose import Pose6D
                hw = get_shared_hardware()
                # 降低躯干规划器速度，防止抬升过猛
                hw.set_ruckig_planner_params(
                    planner_index=2, is_sync=False,
                    velocity_max=[0.1, 0.1, 0.2, 0.2],
                    acceleration_max=[0.08, 0.08, 0.15, 0.15],
                    jerk_max=[0.8, 0.8, 1.5, 1.5],
                )
                pose_result = hw.get_torso_initial_pose()
                if pose_result.success:
                    init = pose_result.data  # {'position': [x,y,z], 'euler': [yaw,pitch,roll]}
                    init_x = init['position'][0]
                    init_z = init['position'][2]
                    new_z = init_z + lift_z
                    # 参考 test_torso_pose.py 的 send_torso_pose(pose) 用法
                    torso_pose = Pose6D(
                        x=init_x + 0.0, y=0.0, z=new_z,
                        yaw=0.0, pitch=0.0, roll=0.0,
                    )
                    hw.send_torso_pose(torso_pose)
                    torso_msg = (
                        f"torso_lifted: init_z={init_z:.3f}→{new_z:.3f} "
                        f"(+{lift_z:.3f}m)"
                    )
                else:
                    torso_msg = (
                        f"torso_lift_failed: get_torso_initial_pose={pose_result.message}"
                    )
            except Exception as e:
                torso_msg = f"torso_lift_error: {e}"
        effective_tag_z = tag_z

        PICK_Z_OFFSET = 0.2  # 帧3收臂比 tag 低 0.2m，不举太高
        dz = effective_tag_z - anchor_z - PICK_Z_OFFSET

        new_left, new_right = [], []
        for lk, rk in zip(left_kp, right_kp):
            new_left.append(Pose(
                pos=(float(lk.pos[0]), float(lk.pos[1]), float(lk.pos[2]) + dz),
                quat=tuple(float(v) for v in lk.quat),
                frame=lk.frame,
            ))
            new_right.append(Pose(
                pos=(float(rk.pos[0]), float(rk.pos[1]), float(rk.pos[2]) + dz),
                quat=tuple(float(v) for v in rk.quat),
                frame=rk.frame,
            ))

        parts = [f"tag_z={tag_z:.3f}"]
        if torso_msg:
            parts.append(torso_msg)
        parts.append(f"effective_z={effective_tag_z:.3f} anchor={anchor_z:.3f} dz={dz:.3f}")
        self.feedback_message = " | ".join(parts)
        return (new_left, new_right)

    def _make_lift_from_pick_end(self):
        """lift = pick 终点 + z_lift（相对抬升，不锚定 tag）。

        从黑板读 pick 阶段的 eef_traj 终点，Z 加上 z_lift 后作为 lift 目标帧。
        读不到则回退用原始 lift keypoints + tag 锚定。
        """
        from kuavo_humanoid_sdk.kuavo_strategy_pytree.common.data_type import Pose

        pick_end_left = self._read_eef_traj_end("left_arm_eef_traj")
        pick_end_right = self._read_eef_traj_end("right_arm_eef_traj")
        z_lift = float(self.params.get("z_lift", 0.2))

        if pick_end_left is not None and pick_end_right is not None:
            # 躯干已扛高度，arm 只需在 BASE 系做与 tag=0.7m 时完全一样的上抬
            # 从 Pose6D 转回 SDK Pose
            left_pos = (pick_end_left.x, pick_end_left.y, pick_end_left.z + z_lift)
            right_pos = (pick_end_right.x, pick_end_right.y, pick_end_right.z + z_lift)
            left_quat = pick_end_left.to_quaternion()
            right_quat = pick_end_right.to_quaternion()
            new_left = [Pose(pos=left_pos, quat=left_quat, frame="base_link")]
            new_right = [Pose(pos=right_pos, quat=right_quat, frame="base_link")]
            self.feedback_message = (
                f"lift from pick_end: z={pick_end_left.z:.2f}→{pick_end_left.z + z_lift:.2f}"
            )
            return (new_left, new_right)

        # 回退：用原始 keypoints + tag 锚定
        return self._adjust_keypoints_to_tag()

    def _read_eef_traj_end(self, traj_key):
        """读黑板上游写入的 eef_traj，返回最后一帧 Pose6D。"""
        traj = getattr(self.global_blackboard, traj_key, None)
        if traj and isinstance(traj, (list, tuple)) and len(traj) > 0:
            return traj[-1]
        return None

    # ------------------------------------------------------------------
    # EEF 轨迹(直接 SDK Pose → Pose6D,不依赖 IK)
    # ------------------------------------------------------------------

    def _write_eef_traj(self, keypoints):
        """输出 2 帧 EEF 轨迹(起点→终点)，WBC 自插值。

        pick: 帧0 → 帧3
        lift: 目标 Z - z_lift → 目标 Z
        """
        from core.domain.pose import Pose6D

        left_keypoints, right_keypoints = keypoints
        target_idx = self._target_frame_idx()

        if self._keypoints_source == "pick" and len(left_keypoints) >= 4:
            start_idx = 0
            target_idx = 3
        elif target_idx > 0:
            start_idx = target_idx - 1
        else:
            start_idx = None

        if start_idx is not None and self._keypoints_source != "pick":
            left_start = _sdk_pose_to_pose6d(left_keypoints[start_idx])
            right_start = _sdk_pose_to_pose6d(right_keypoints[start_idx])
        elif self._keypoints_source == "pick":
            # 起点用手臂复位位置（BASE 系），对齐 test_arm_eef_base.py INITIAL
            import math
            left_start = Pose6D(x=0.2, y=0.25, z=1.1,
                                roll=0.0, pitch=-math.pi/2, yaw=0.0)
            right_start = Pose6D(x=0.2, y=-0.25, z=1.1,
                                 roll=0.0, pitch=-math.pi/2, yaw=0.0)
        else:
            left_tgt = _sdk_pose_to_pose6d(left_keypoints[target_idx])
            right_tgt = _sdk_pose_to_pose6d(right_keypoints[target_idx])
            z_offset = float(self.params.get("z_lift", 0.2))
            left_start = Pose6D(
                x=left_tgt.x, y=left_tgt.y, z=left_tgt.z - z_offset,
                roll=left_tgt.roll, pitch=left_tgt.pitch, yaw=left_tgt.yaw,
            )
            right_start = Pose6D(
                x=right_tgt.x, y=right_tgt.y, z=right_tgt.z - z_offset,
                roll=right_tgt.roll, pitch=right_tgt.pitch, yaw=right_tgt.yaw,
            )

        left_tgt = _sdk_pose_to_pose6d(left_keypoints[target_idx])
        right_tgt = _sdk_pose_to_pose6d(right_keypoints[target_idx])

        setattr(self.global_blackboard, "left_arm_eef_traj", [left_start, left_tgt])
        setattr(self.global_blackboard, "right_arm_eef_traj", [right_start, right_tgt])
        self.feedback_message = (
            f"wrote eef traj (2 pts): "
            f"left=({left_start.x:.2f},{left_start.y:.2f},{left_start.z:.2f})"
            f"→({left_tgt.x:.2f},{left_tgt.y:.2f},{left_tgt.z:.2f}), "
            f"right=({right_start.x:.2f},{right_start.y:.2f},{right_start.z:.2f})"
            f"→({right_tgt.x:.2f},{right_tgt.y:.2f},{right_tgt.z:.2f})"
        )
        return Status.SUCCESS

    # ------------------------------------------------------------------
    # Joint 轨迹(通过 check_ik_accessibility 服务求解)
    # ------------------------------------------------------------------

    def _try_write_joint_traj(self, keypoints):
        """通过 IK 服务将目标帧 keypoint → 7 关节角度,写入 joint_traj。

        成功返回 SUCCESS,失败返回 FAILURE。
        """
        from orchestration.shared_hardware import get_shared_hardware

        try:
            hw = get_shared_hardware()
        except Exception as e:
            self.feedback_message = f"hw unavailable: {e}"
            return Status.FAILURE

        left_keypoints, right_keypoints = keypoints
        frame_idx = self._target_frame_idx()

        lk = left_keypoints[frame_idx]
        rk = right_keypoints[frame_idx]

        l_joints = _pose_to_joints(hw, lk, is_left=True)
        r_joints = _pose_to_joints(hw, rk, is_left=False)

        if l_joints is None or r_joints is None:
            self.feedback_message = (
                f"IK failed frame {frame_idx}: "
                f"left={'OK' if l_joints else 'FAIL'}, "
                f"right={'OK' if r_joints else 'FAIL'}"
            )
            return Status.FAILURE

        setattr(self.global_blackboard, "left_arm_joint_traj", l_joints)
        setattr(self.global_blackboard, "right_arm_joint_traj", r_joints)
        self.feedback_message = (
            f"wrote joint traj frame {frame_idx}: "
            f"left={[round(a, 1) for a in l_joints]}, "
            f"right={[round(a, 1) for a in r_joints]}"
        )
        return Status.SUCCESS

    def _target_frame_idx(self):
        """pick → 收臂帧(index 3), lift → 唯一帧(index 0)。"""
        left_kp, _ = self._keypoints
        idx = 3 if self._keypoints_source == "pick" else 0
        return min(idx, len(left_kp) - 1)


# ----------------------------------------------------------------------
# 模块级工具函数
# ----------------------------------------------------------------------

def _sdk_pose_to_pose6d(sdk_pose):
    """SDK Pose(BASE 系, pos/quat 为 np.array) → Pose6D。

    避开 scipy as_euler 在 pitch=±90° 时的万向锁警告,输出确定。
    """
    from core.domain.pose import Pose6D
    from scipy.spatial.transform import Rotation as R

    pos = sdk_pose.pos
    quat = sdk_pose.quat

    # 万向锁警告可安全忽略:对 keypoints.py 中的 euler=(0, ±90°, 0),
    # as_euler('xyz') 输出即为 (0, ±π/2, 0),第三个角=0
    with warnings.catch_warnings():
        warnings.filterwarnings("ignore", message="Gimbal lock")
        euler = R.from_quat(quat).as_euler('xyz', degrees=False)

    return Pose6D(
        x=float(pos[0]), y=float(pos[1]), z=float(pos[2]),
        roll=float(euler[0]), pitch=float(euler[1]), yaw=float(euler[2]),
    )


def _pose_to_joints(hw, sdk_pose, is_left):
    """单个 SDK Pose → 7 关节角度(度),通过 IK 服务求解。

    sdk_pose: kuavo_humanoid_sdk Pose(pos/quat 为 np.array, frame=BASE)。
    失败返回 None。
    """
    from scipy.spatial.transform import Rotation as R

    pos = sdk_pose.pos
    quat = sdk_pose.quat

    with warnings.catch_warnings():
        warnings.filterwarnings("ignore", message="Gimbal lock")
        euler = R.from_quat(quat).as_euler('xyz', degrees=False)

    pose_desired = [
        float(pos[0]), float(pos[1]), float(pos[2]),
        float(euler[0]), float(euler[1]), float(euler[2]),
    ]

    try:
        result = hw.check_ik_accessibility(
            is_left=is_left,
            is_local=True,          # keypoints 在 BASE 系(局部)
            is_whole_body=False,     # 仅手臂
            pose_desired=pose_desired,
            total_time_desired=1.0,
            max_attempts=5,
            linear_error_max=0.005,
            angular_error_max=0.05,
        )
    except Exception:
        return None

    if not result.success or result.data is None:
        return None

    data = result.data
    # 优先精确解 q_best,其次位置优先解 q_pos_priority_best
    for key in ("q_best", "q_pos_priority_best"):
        q = data.get(key)
        if q and isinstance(q, (list, tuple)) and len(q) == 7:
            return [float(v) for v in q]

    return None
