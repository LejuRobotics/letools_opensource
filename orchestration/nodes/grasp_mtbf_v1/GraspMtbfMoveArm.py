# -*- coding: utf-8 -*-
"""GraspMtbfMoveArm：按 ArmPoseAndWrench 关键点序列移动双臂（世界系）。

- 从黑板读 ArmPoseAndWrench（CalcArmPoseMove 输出，tag 系关键点）
- tag 来自黑板 latest_tag_<tag_id>（NodePercep 输出，odom 系）；
- 搬运箱流程（夹板末端）：在并拢关键点闭合夹爪，在放置关键点张开夹爪
"""

import math
import os
import threading
import time

import py_trees
from py_trees.common import Status

from core.common.transform import pose6d_to_matrix, transform_pose
from core.domain.end_effector import GripperCommand
from core.domain.enums import ArmSide
from core.domain.result import Result
from core.domain.pose import Pose6D
from orchestration.nodes.base_node import BaseAction
from .scene_io import get_scene_io
from orchestration.utils.manifest_decorators import define_manifest

_DRY_RUN = os.environ.get("STUDIO_DRY_RUN", "").lower() in ("1", "true", "yes")


def _make_odom_to_base():
    """构造 odom → base_link 的 4x4 变换（经 tf2），失败返回 None。"""
    try:
        import rospy
        import tf2_ros
        import numpy as np
        from scipy.spatial.transform import Rotation as R
        if not hasattr(_make_odom_to_base, "_buffer"):
            _make_odom_to_base._buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
            _make_odom_to_base._listener = tf2_ros.TransformListener(_make_odom_to_base._buffer)
        for base_frame in ("base_link", "base_link_lb", "base_footprint"):
            try:
                t = _make_odom_to_base._buffer.lookup_transform(
                    base_frame, "odom", rospy.Time(0), rospy.Duration(0.3))
                tr = t.transform
                m = np.eye(4)
                m[:3, 3] = [tr.translation.x, tr.translation.y, tr.translation.z]
                m[:3, :3] = R.from_quat([tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w]).as_matrix()
                return m
            except Exception:
                continue
        return None
    except Exception:
        return None


@define_manifest(
    label="移动机械臂到目标点",
    category=["motion", "arm"],
    tree_type="grasp_mtbf_v1",
    description="从黑板读 ArmPoseAndWrench，tag系关键点经 latest_tag_<tag_id> 变换到世界系后走 TimedCmd planner 6/7 下发，按 actualTime 等待后执行关键点夹爪动作",
    params=[
        {"name": "tag_id", "type": "int", "default": "0",
         "description": "黑板 latest_tag_<tag_id> 来源（NodePercep）；<=0 时按 base 系下发"},
        {"name": "total_time", "type": "float", "default": "3.0", "description": "末端轨迹总时间(s)"},
        {"name": "gripper_close_indices", "type": "string", "default": "",
         "description": "逗号分隔的关键点索引，轨迹执行完成后闭合夹爪（如 '1,2'）"},
        {"name": "gripper_open_indices", "type": "string", "default": "",
         "description": "逗号分隔的关键点索引，轨迹执行完成后张开夹爪（如 '0,1'）"},
        {"name": "gripper_position", "type": "float", "default": "100", "description": "夹爪闭合行程[0,100]"},
        {"name": "gripper_effort", "type": "float", "default": "1.0", "description": "夹爪力矩(A)"},
        {"name": "debug_break", "type": "bool", "default": "false", "description": "调试：轨迹下发前暂停等待 Enter"},
        {"name": "service_timeout", "type": "float", "default": "6.0", "description": "单次服务响应超时(s)，超时后停止流程"},
        {"name": "motion_timeout", "type": "float", "default": "60.0", "description": "单个关键点允许的最大执行时长(s)"},
        {"name": "settle_time", "type": "float", "default": "0.3", "description": "每个关键点规划结束后的额外等待(s)"},
        {"name": "cmd_interval", "type": "float", "default": "0.5", "description": "相邻关键点指令间隔秒数（避免运控 main loop busy）"},
        {"name": "prep_only", "type": "bool", "default": "false",
         "description": "只执行预抓取准备位（BASE 系写死位姿），不走 Tag 关键点轨迹"},
        {"name": "prep_time", "type": "float", "default": "2.0", "description": "预抓取准备位执行时长(s)"},
        {"name": "prep_left_pose", "type": "json", "default": "[0.4,0.35,0.13,0,-90,0]",
         "description": "prep_only 左臂 BASE 位姿 [x,y,z,yaw,pitch,roll]（米/度）"},
        {"name": "prep_right_pose", "type": "json", "default": "[0.4,-0.35,0.13,0,-90,0]",
         "description": "prep_only 右臂 BASE 位姿 [x,y,z,yaw,pitch,roll]（米/度）"},
        {"name": "max_keypoints", "type": "int", "default": "0",
         "description": "最多执行前 N 个关键点；0 表示全部。预伸时用 1（只走抓取预张点，不夹爪）"},
        {"name": "max_reach_xy", "type": "float", "default": "0",
         "description": "base系水平伸距上限(m)；>0 时按计算方向等比缩放 xy，保留 z/姿态，避免超臂展抽动"},
    ],
    inputs=[
        {"name": "arm_pose_and_wrench", "type": "object", "required": True, "default_key": "ArmPoseAndWrench",
         "description": "CalcArmPoseMove 输出 [(left_kps, right_kps), (wrench)]"},
        {"name": "latest_tag", "type": "object", "required": False, "default_key": "latest_tag_<tag_id>",
         "description": "NodePercep 写入的 TagDetection（odom 系）"},
    ],
    outputs=[
        {"name": "arm_move_result", "type": "bool", "default_key": "ArmMoveResult", "description": "移动是否成功"},
    ],
)
class GraspMtbfMoveArm(BaseAction):
    # An unknown outcome must not be bypassed by Retry or another arm node.
    # Confirm controller state before restarting the process.
    _uncertain_motion = threading.Event()

    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._done = False
        self._success = False
        self._fault = None
        self._pending = None
        self._hardware = None
        self._steps_iter = None
        self._motion_started = False
        self._phase = "idle"

    @staticmethod
    def _parse_indices(raw):
        s = str(raw or "").strip()
        if not s:
            return set()
        return {int(x.strip()) for x in s.split(",") if x.strip()}

    @staticmethod
    def _quat_point_to_cmd_deg(p):
        """[x,y,z,qx,qy,qz,qw] → TimedCmd [x,y,z,yaw,pitch,roll]（度）。"""
        from scipy.spatial.transform import Rotation as R
        r = R.from_quat([p[3], p[4], p[5], p[6]])
        roll, pitch, yaw = r.as_euler("xyz")
        return [p[0], p[1], p[2], math.degrees(yaw), math.degrees(pitch), math.degrees(roll)]

    @staticmethod
    def _parse_pose6(raw, default):
        """解析 [x,y,z,yaw,pitch,roll]（yaw/pitch/roll 为度）。"""
        import ast
        parsed = default if raw is None else raw
        if isinstance(parsed, str):
            parsed = ast.literal_eval(parsed)
        if not isinstance(parsed, (list, tuple)) or len(parsed) != 6:
            raise ValueError("手臂准备位必须是6个有限数")
        values = [float(value) for value in parsed]
        if not all(math.isfinite(value) for value in values):
            raise ValueError("手臂准备位必须是6个有限数")
        return values

    def _gripper_steps(self, close):
        position = float(self.params.get("gripper_position", 100.0)) if close else 0.0
        effort = float(self.params.get("gripper_effort", 1.0))
        if not math.isfinite(position) or not 0 <= position <= 100 or not math.isfinite(effort):
            raise ValueError("夹爪参数无效")
        cmd = GripperCommand(position=position, velocity=50.0, effort=effort)
        for side in (ArmSide.LEFT, ArmSide.RIGHT):
            yield ("gripper", (side, cmd), 0.0)

    def initialise(self):
        if self._fault or self._uncertain_motion.is_set():
            self._fail(self._fault or "之前的运动结果不确定，请确认控制器状态后重启流程")
            return
        if self._pending is not None or self._phase not in ("idle", "success"):
            self._fail("上一次动作尚未结束，禁止重复启动")
            return
        self._done = False
        self._success = False
        self._motion_started = False
        try:
            self._service_timeout = self._positive_param("service_timeout", 6.0)
            self._motion_timeout = self._positive_param("motion_timeout", 60.0)
            self._settle_time = self._positive_param("settle_time", 0.3, allow_zero=True)
            self._cmd_interval = self._positive_param("cmd_interval", 0.5, allow_zero=True)
            self._tag_id = int(self.params.get("tag_id", 0) or 0)
            self._prep_only = str(self.params.get("prep_only", "false")).lower() in ("true", "1", "yes")
            self.global_blackboard.register_key(key="ArmMoveResult", access=py_trees.common.Access.WRITE)
            if not self._prep_only:
                self.global_blackboard.register_key(key="ArmPoseAndWrench", access=py_trees.common.Access.READ)
            if self._tag_id > -1:
                self.global_blackboard.register_key(key=f"latest_tag_{self._tag_id}", access=py_trees.common.Access.READ)
            self.global_blackboard.set("ArmMoveResult", False)
            if _DRY_RUN:
                self._succeed()
                return
            self._steps_iter = self._steps()
            self._phase = "next"
        except Exception as exc:
            self._fail(f"手臂节点初始化失败: {exc}")

    def _positive_param(self, key, default, allow_zero=False):
        value = float(self.params.get(key, default))
        if not math.isfinite(value) or value < 0 or (value == 0 and not allow_zero):
            raise ValueError(f"{key} 必须是{'非负' if allow_zero else '正'}有限数")
        return value

    def _steps(self):
        tag_key = f"latest_tag_{self._tag_id}"
        hw = self._hardware = get_scene_io()
        # prep_only 模式：只执行写死 BASE 预抓取位，不读关键点、不走 Tag 轨迹
        if self._prep_only:
            lp = self._parse_pose6(
                self.params.get("prep_left_pose"), [0.4, 0.35, 0.13, 0.0, -90.0, 0.0])
            rp = self._parse_pose6(
                self.params.get("prep_right_pose"), [0.4, -0.35, 0.13, 0.0, -90.0, 0.0])
            # TimedCmd: [x,y,z,yaw,pitch,roll]（度）
            left_cmd = [lp[0], lp[1], lp[2], lp[3], lp[4], lp[5]]
            right_cmd = [rp[0], rp[1], rp[2], rp[3], rp[4], rp[5]]
            print(
                f"[GraspMtbfMoveArm] 预抓取准备位(prep_only BASE): "
                f"L=({left_cmd[0]:.3f},{left_cmd[1]:.3f},{left_cmd[2]:.3f}) "
                f"R=({right_cmd[0]:.3f},{right_cmd[1]:.3f},{right_cmd[2]:.3f}) "
                f"pitch={left_cmd[4]:.1f}°",
                flush=True,
            )
            yield ("motion", [
                {"planner_index": 6, "desire_time": float(self.params.get("prep_time", 2.0)), "cmd_vec": left_cmd},
                {"planner_index": 7, "desire_time": float(self.params.get("prep_time", 2.0)), "cmd_vec": right_cmd},
            ], 0.0)
            self.feedback_message = "预抓取准备位规划时间等待完成"
            return

        data = self.global_blackboard.get("ArmPoseAndWrench")
        if not data:
            raise ValueError("ArmPoseAndWrench 为空")

        (left_kps, right_kps) = data[0]
        if not left_kps or len(left_kps) != len(right_kps):
            raise ValueError("双臂关键点不能为空且数量必须一致")

        # tag 来自 NodePercep 写入的 latest_tag_<id>（TagDetection, odom 系）。
        tag_matrix = None
        if self._tag_id > -1:
            tag = getattr(self.global_blackboard, tag_key, None)
            if tag is not None and getattr(tag, "pose_in_world", None) is not None:
                t = tag.pose_in_world
                fixed = Pose6D(x=t.x, y=t.y, z=t.z, roll=math.pi / 2, pitch=0.0, yaw=t.yaw)
                tag_matrix = pose6d_to_matrix(fixed)
                print(f"[GraspMtbfMoveArm] tag {self._tag_id}: odom=({t.x:.3f},{t.y:.3f},{t.z:.3f},yaw={t.yaw:.3f})", flush=True)
            else:
                self.feedback_message = f"latest_tag_{self._tag_id} 未就绪（tag 未检测到或未写入黑板）"
                print(f"[GraspMtbfMoveArm] ❌ {self.feedback_message}", flush=True)
                raise ValueError(self.feedback_message)

        # TAG 系关键点 × tag(odom) → odom → base_link(tf) → BASE 系。
        # 统一转 [x,y,z,qx,qy,qz,qw]。
        odom_to_base = _make_odom_to_base() if tag_matrix is not None else None
        if tag_matrix is not None and odom_to_base is None:
            self.feedback_message = "odom→base_link TF 查询失败"
            raise ValueError(self.feedback_message)

        def _to_traj_point(lk):
            p = Pose6D(x=lk[0], y=lk[1], z=lk[2],
                       roll=math.radians(lk[3]), pitch=math.radians(lk[4]), yaw=math.radians(lk[5]))
            if tag_matrix is not None:
                # TAG 系 → odom → base
                p = transform_pose(p, tag_matrix)
                p = transform_pose(p, odom_to_base)
            qx, qy, qz, qw = p.to_quaternion()
            return [p.x, p.y, p.z, qx, qy, qz, qw]

        left_traj, right_traj = [], []
        for lk, rk in zip(left_kps, right_kps):
            left_traj.append(_to_traj_point(lk))
            right_traj.append(_to_traj_point(rk))

        max_kps = int(self.params.get("max_keypoints", 0) or 0)
        if max_kps > 0:
            left_traj = left_traj[:max_kps]
            right_traj = right_traj[:max_kps]
            print(f"[GraspMtbfMoveArm] max_keypoints={max_kps}，仅执行前 {len(left_traj)} 点", flush=True)

        max_reach_xy = float(self.params.get("max_reach_xy", 0) or 0)
        if max_reach_xy > 0:
            for side, traj in (("L", left_traj), ("R", right_traj)):
                for i, p in enumerate(traj):
                    xy = math.hypot(p[0], p[1])
                    if xy > max_reach_xy:
                        scale = max_reach_xy / xy
                        old = (p[0], p[1], p[2])
                        p[0] *= scale
                        p[1] *= scale
                        print(
                            f"[GraspMtbfMoveArm] {side}{i} 水平伸距 {xy:.3f}m → "
                            f"{max_reach_xy:.3f}m，pos {old} → ({p[0]:.3f},{p[1]:.3f},{p[2]:.3f})",
                            flush=True,
                        )

        # 仅在「完整轨迹恰好 1 点」时复制；max_keypoints=1 的预伸只发一次，避免连发抽动
        if len(left_traj) == 1 and max_kps <= 0:
            left_traj.insert(0, list(left_traj[0]))
            right_traj.insert(0, list(right_traj[0]))

        total_time = float(self.params.get("total_time", 3.0))

        # 调试断点（轨迹下发前，确认当前手臂/躯干状态）
        if str(self.params.get("debug_break", "false")).lower() in ("true", "1", "yes"):
            print(f"[DEBUG-BREAK] 即将下发 {len(left_traj)} 点轨迹 (frame=base_link):")
            for i, p in enumerate(left_traj):
                print(f"  L{i}: pos=({p[0]:.3f},{p[1]:.3f},{p[2]:.3f}) quat=({p[3]:.3f},{p[4]:.3f},{p[5]:.3f},{p[6]:.3f})")
            # 打印当前手臂实际位姿，看和轨迹起点差距
            try:
                sm = getattr(hw, "state_manager", None)
                if sm is not None:
                    poses = sm.get_state("ee_poses")
                    if poses and len(poses) >= 2:
                        lp0 = poses[0]["position"]
                        rp0 = poses[1]["position"]
                        print(f"[DEBUG-BREAK] 当前手臂位姿(eePoses): L=({lp0['x']:.3f},{lp0['y']:.3f},{lp0['z']:.3f}) R=({rp0['x']:.3f},{rp0['y']:.3f},{rp0['z']:.3f})")
            except Exception as e:
                print(f"[DEBUG-BREAK] 读 eePoses 失败: {e}")
            print("[DEBUG-BREAK] 按 Enter 继续 ...")
            yield ("prompt", None, 0.0)

        # is_sync only synchronizes planner durations; the node waits actualTime.
        close_indices = self._parse_indices(self.params.get("gripper_close_indices"))
        open_indices = self._parse_indices(self.params.get("gripper_open_indices"))
        for i, (lp, rp) in enumerate(zip(left_traj, right_traj)):
            # [x, y, z, qx, qy, qz, qw] → TimedCmd [x, y, z, yaw, pitch, roll]（度）
            def _quat_to_euler_deg(p):
                from scipy.spatial.transform import Rotation as R
                r = R.from_quat([p[3], p[4], p[5], p[6]])
                roll, pitch, yaw = r.as_euler("xyz")
                import math as _m
                return [p[0], p[1], p[2], _m.degrees(yaw), _m.degrees(pitch), _m.degrees(roll)]

            lv = _quat_to_euler_deg(lp)
            rv = _quat_to_euler_deg(rp)

            print(f"[GraspMtbfMoveArm] kp{i}(局部系) 下发值:")
            print(f"  L planner=6 cmd_vec=(x={lv[0]:.3f},y={lv[1]:.3f},z={lv[2]:.3f},yaw={lv[3]:.2f},pitch={lv[4]:.2f},roll={lv[5]:.2f})", flush=True)
            print(f"  R planner=7 cmd_vec=(x={rv[0]:.3f},y={rv[1]:.3f},z={rv[2]:.3f},yaw={rv[3]:.2f},pitch={rv[4]:.2f},roll={rv[5]:.2f})", flush=True)

            commands = [
                {"planner_index": 6, "desire_time": total_time, "cmd_vec": lv},  # 左臂末端局部系
                {"planner_index": 7, "desire_time": total_time, "cmd_vec": rv},  # 右臂末端局部系
            ]
            print(f"[GraspMtbfMoveArm] kp{i}: 调 send_timed_multi_commands(planner 6/7, {total_time}s)", flush=True)
            yield ("motion", commands, 0.0)
            if i in close_indices:
                yield from self._gripper_steps(close=True)
            if i in open_indices:
                yield from self._gripper_steps(close=False)
            if i < len(left_traj) - 1:
                yield ("delay", None, self._cmd_interval)
        self.feedback_message = f"全部 {len(left_traj)} 个关键点规划时间等待完成"

    def _start_call(self, kind, payload):
        # Each worker performs exactly one call. It can never issue the next
        # waypoint/gripper command after cancellation or a late response.
        flight = {"done": threading.Event(), "cancel": threading.Event()}
        self._pending = flight
        self._call_kind = kind
        self._deadline = float("inf") if kind == "prompt" else time.monotonic() + self._service_timeout
        call_deadline = self._deadline
        if kind != "prompt":
            self._motion_started = True

        def invoke():
            try:
                if flight["cancel"].is_set() or self._uncertain_motion.is_set():
                    raise RuntimeError("动作已取消，禁止补发请求")
                if time.monotonic() >= call_deadline:
                    raise TimeoutError("调用启动已超时，禁止补发请求")
                if kind == "motion":
                    flight["result"] = self._hardware.send_timed_multi_commands(
                        payload, is_sync=True, cancel_event=flight["cancel"])
                elif kind == "gripper":
                    flight["result"] = self._hardware.control_end_effector(*payload)
                else:
                    try:
                        input()
                    except EOFError:
                        pass
                    flight["result"] = Result.ok()
            except BaseException as exc:
                flight["error"] = exc
            finally:
                flight["finished_at"] = time.monotonic()
                flight["done"].set()

        self._phase = "call"
        threading.Thread(target=invoke, name="grasp_arm_command", daemon=True).start()

    def _succeed(self):
        self.global_blackboard.set("ArmMoveResult", True)
        self._phase = "success"
        self._success = self._done = True

    def _fail(self, reason):
        first_failure = self._fault is None
        if self._pending is not None:
            self._pending["cancel"].set()
        self._fault = self._fault or reason
        self.feedback_message = self._fault
        self._phase = "failed"
        self._success = False
        self._done = True
        try:
            self.global_blackboard.set("ArmMoveResult", False)
        except Exception:
            pass
        if first_failure and self._motion_started:
            self._uncertain_motion.set()
            hw = self._hardware
            def stop():
                try:
                    result = hw.set_motion_control_enabled(False)
                    if not result.success:
                        print(f"[GraspMtbfMoveArm] 停止请求失败: {result.message}", flush=True)
                except Exception as exc:
                    print(f"[GraspMtbfMoveArm] 停止请求异常: {exc}", flush=True)
            threading.Thread(target=stop, name="grasp_arm_stop", daemon=True).start()

    def update(self):
        if self._done:
            return Status.SUCCESS if self._success else Status.FAILURE
        try:
            if self._uncertain_motion.is_set():
                raise RuntimeError("之前的运动结果不确定，请确认控制器状态后重启流程")
            if self._phase == "call":
                flight = self._pending
                if not flight["done"].is_set():
                    if time.monotonic() >= self._deadline:
                        raise TimeoutError("控制服务响应超时；请求可能仍在服务端执行，停止后续动作")
                    return Status.RUNNING
                if flight["finished_at"] >= self._deadline:
                    raise TimeoutError("控制服务响应迟到，停止后续动作")
                self._pending = None
                if "error" in flight:
                    raise RuntimeError(f"控制服务异常: {flight['error']}")
                result = flight["result"]
                if result is None or not result.success:
                    raise RuntimeError(f"控制请求失败: {getattr(result, 'message', '缺少响应')}")
                if self._call_kind == "motion":
                    actual_time = float(result.data["actual_time"])
                    if not math.isfinite(actual_time) or not 0 <= actual_time <= self._motion_timeout:
                        raise ValueError(f"控制器返回无效执行时长: {actual_time}")
                    duration = max(actual_time, self._requested_time)
                    self._finish_at = flight["finished_at"] + duration + self._settle_time
                    self.feedback_message = f"等待轨迹规划时长 {duration:.2f}s（不等同实测到位）"
                    self._phase = "wait"
                else:
                    self._phase = "next"
            if self._phase == "wait":
                if time.monotonic() < self._finish_at:
                    return Status.RUNNING
                self._phase = "next"
            if self._phase == "next":
                try:
                    kind, payload, duration = next(self._steps_iter)
                except StopIteration:
                    self._succeed()
                    return Status.SUCCESS
                if kind == "delay":
                    self._finish_at = time.monotonic() + duration
                    self._phase = "wait"
                else:
                    if kind == "motion":
                        times = [float(cmd["desire_time"]) for cmd in payload]
                        if any(not math.isfinite(t) or not 0 < t <= self._motion_timeout for t in times):
                            raise ValueError("期望执行时间必须大于0且不超过 motion_timeout")
                        if any(len(cmd["cmd_vec"]) != 6 or not all(math.isfinite(float(v)) for v in cmd["cmd_vec"]) for cmd in payload):
                            raise ValueError("手臂目标必须包含6个有限数")
                        self._requested_time = max(times)
                    self._start_call(kind, payload)
            return Status.RUNNING
        except Exception as exc:
            self._fail(f"手臂动作失败: {exc}")
            return Status.FAILURE

    def terminate(self, new_status):
        if new_status != Status.SUCCESS and not self._done:
            self._fail("手臂动作被中断；确认控制器状态后再重新运行")
