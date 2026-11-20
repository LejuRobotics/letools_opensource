#!/usr/bin/env python3
"""Offline head regression tests: fake clients only; never create hardware/Node."""
from __future__ import annotations

import ast
import importlib.util
import math
from pathlib import Path
import sys
import time
import types
import unittest
from unittest.mock import Mock, call, patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from ros2_mixin_contracts import BODY_OUTCOMES, BODY_SIGNATURES


# ---------------------------------------------------------------------------
# Python 3.8 (CI / ROS 1 Noetic) 兼容层
#   * ast.unparse 是 3.9+ 才有的接口
#   * ast.dump 在 3.8 会多打印 annotation/type_comment/kind 这些 None 字段
#   * float() 的异常文案在 3.10 才改成 "a real number"
# ---------------------------------------------------------------------------
_NONE_FIELD_JUNK = (", annotation=None", ", type_comment=None", ", kind=None",
                     ", vararg=None", ", kwarg=None")


class _MiniUnparser:
    """仅覆盖本文件用到的表达式；未知节点退回 ast.dump 文本。"""

    _BIN = {ast.Add: "+", ast.Sub: "-", ast.Mult: "*", ast.Div: "/",
            ast.FloorDiv: "//", ast.Mod: "%", ast.Pow: "**"}
    _CMP = {ast.Eq: "==", ast.NotEq: "!=", ast.Lt: "<", ast.LtE: "<=",
            ast.Gt: ">", ast.GtE: ">=", ast.Is: "is", ast.IsNot: "is not",
            ast.In: "in", ast.NotIn: "not in"}
    _UNARY = {ast.USub: "-", ast.UAdd: "+", ast.Not: "not ", ast.Invert: "~"}

    def visit(self, node):
        method = getattr(self, "_visit_" + type(node).__name__, None)
        return ast.dump(node) if method is None else method(node)

    def _visit_Module(self, node):
        return ast.dump(node)

    def _visit_Expr(self, node):
        return self.visit(node.value)

    def _visit_Name(self, node):
        return node.id

    def _visit_Attribute(self, node):
        return self.visit(node.value) + "." + node.attr

    def _visit_Constant(self, node):
        return repr(node.value)

    def _visit_keyword(self, node):
        prefix = "" if node.arg is None else node.arg + "="
        return prefix + self.visit(node.value)

    def _visit_Call(self, node):
        parts = [self.visit(a) for a in node.args]
        parts += [self.visit(k) for k in node.keywords]
        return self.visit(node.func) + "(" + ", ".join(parts) + ")"

    def _visit_List(self, node):
        return "[" + ", ".join(self.visit(e) for e in node.elts) + "]"

    def _visit_Tuple(self, node):
        body = ", ".join(self.visit(e) for e in node.elts)
        return "(" + body + ("," if len(node.elts) == 1 else "") + ")"

    def _visit_Set(self, node):
        return "{" + ", ".join(self.visit(e) for e in node.elts) + "}"

    def _visit_Dict(self, node):
        return "{" + ", ".join(self.visit(k) + ": " + self.visit(v)
                               for k, v in zip(node.keys, node.values)) + "}"

    def _visit_UnaryOp(self, node):
        return self._UNARY.get(type(node.op), "?") + self.visit(node.operand)

    def _visit_BinOp(self, node):
        return (self.visit(node.left) + " " + self._BIN.get(type(node.op), "?")
                + " " + self.visit(node.right))

    def _visit_Subscript(self, node):
        return self.visit(node.value) + "[" + self.visit(node.slice) + "]"

    def _visit_Index(self, node):
        return self.visit(node.value)

    def _visit_Compare(self, node):
        parts = [self.visit(node.left)]
        for op, comparator in zip(node.ops, node.comparators):
            parts.append(self._CMP.get(type(op), "?"))
            parts.append(self.visit(comparator))
        return " ".join(parts)


def _unparse(node):
    fn = getattr(ast, "unparse", None)
    return fn(node) if fn is not None else _MiniUnparser().visit(node)


def _canon_dump(node):
    # Python 3.8 把 Subscript.slice 包成 ast.Index(...)，3.9+ 已取消该包装。
    index_type = getattr(ast, "Index", None)
    if index_type is not None:
        for child in ast.walk(node):
            if isinstance(child, ast.Subscript) and isinstance(child.slice, index_type):
                child.slice = child.slice.value
    text = ast.dump(node)
    for junk in _NONE_FIELD_JUNK:
        text = text.replace(junk, "")
    return text


def _norm_msg(value):
    return value.replace("a real number", "a number") if isinstance(value, str) else value


def _norm_result(value):
    tag, payload = value
    if isinstance(payload, dict):
        payload = dict(payload)
        payload["message"] = _norm_msg(payload.get("message"))
    else:
        payload = _norm_msg(payload)
    return (tag, payload)


def load(name, relative):
    spec = importlib.util.spec_from_file_location(name, ROOT / relative)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


# Direct-file imports avoid hardware package __init__ and all hardware constructors.
head = load("offline_head_mixin", "adapters/hardware_ros2/mixins/head_mixin.py")
# Load core dependencies before patch.dict restores sys.modules, preserving type
# identity between the compatibility result, actual hardware MRO, and assertions.
from core.domain.chassis_options import MoveToTargetOptions
from core.interfaces.i_hardware import IHardware

# Never import ROS: forbidden stubs make these tests independent of ROS installation.
def forbidden(*args, **kwargs):
    raise AssertionError("ROS/Node/client construction or DDS operation forbidden")


ros_stub = types.ModuleType("rclpy")
for name in ("init", "shutdown", "ok", "spin_once", "spin_until_future_complete"):
    setattr(ros_stub, name, forbidden)
with patch.dict(sys.modules, {"rclpy": ros_stub}):
    compat = load("offline_head_compat", "adapters/hardware_ros2/ihardware_compat.py")
# Import the actual hardware class and real mixins, but stub all dependency
# constructors. A synthetic package avoids the public package's runtime imports.
package = types.ModuleType("offline_ros2_hardware")
package.__path__ = [str(ROOT / "adapters" / "hardware_ros2")]
stubs = {"rclpy": ros_stub, package.__name__: package,
         package.__name__ + ".ihardware_compat": compat}
for module_name, class_name in (
        ("arm_client", "ArmClient"), ("controller_client", "ControllerClient"),
        ("locomotion_client", "LocomotionClient"),
        ("multibody_client", "MultibodyClient"), ("torso_client", "TorsoClient"),
        ("jibot_chassis_client", "JibotChassisClient"),
        ("head_client", "HeadClient")):
    stub = types.ModuleType(module_name)
    setattr(stub, class_name, type(class_name, (), {"__init__": forbidden}))
    if module_name == "locomotion_client":
        stub.FRAME_WORLD, stub.FRAME_LOCAL = 0, 1
    stubs[module_name] = stub
runtime_stub = types.ModuleType(package.__name__ + ".runtime")
runtime_stub.Ros2Runtime = type("Ros2Runtime", (), {"__init__": forbidden})
stubs[runtime_stub.__name__] = runtime_stub
for mixin_module, mixin_name in (("legacy_chassis_mixin", "LegacyChassisMixin"),
                                 ("state_feedback_rt_mixin", "StateFeedbackRtMixin")):
    mixin_stub = types.ModuleType(package.__name__ + "." + mixin_module)
    setattr(mixin_stub, mixin_name, type(mixin_name, (), {}))
    stubs[mixin_stub.__name__] = mixin_stub
with patch.dict(sys.modules, stubs):
    hardware = load(package.__name__ + ".hardware", "adapters/hardware_ros2/hardware.py")


class Host(compat.IHardwareCompatMixin, head.HeadMixin):
    def __init__(self, client):
        self._head_client = client
        self._head_limits_rad = None


def fake():
    return Mock(frames=7, measured_frames=11,
                **{"wait_ready.return_value": True,
                   "get_limits.return_value": ([-math.pi/2, -math.pi/4],
                                                [math.pi/2, math.pi/4]),
                   "send.return_value": True,
                   "read_current.return_value": [math.pi/6, -math.pi/9],
                   "read_measured.return_value": [-math.pi/12, math.pi/18],
                   "wait_for_new_frame.return_value": [math.pi/6, -math.pi/9]})


class HeadTests(unittest.TestCase):
    def setUp(self):
        self.client = fake()
        self.hw = Host(self.client)

    def check_result(self, result, success, message, data=None, core=False):
        cls = compat.CoreResult if core else head.Result
        self.assertIs(type(result), cls)
        self.assertEqual((result.success, result.message, result.data, result.error_code),
                         (success, message, data, None))

    def test_ready_results_timeout_and_counters(self):
        self.check_result(self.hw.wait_head_ready(0.75), True, "head client ready")
        self.client.wait_ready.assert_called_once_with(0.75)
        self.client.wait_ready.return_value = False
        self.check_result(self.hw.wait_head_ready(), False, "head client not ready")
        self.assertEqual((self.hw.head_frames, self.hw.head_measured_frames), (7, 11))
        self.client.frames = 8
        self.client.measured_frames = 12
        self.assertEqual((self.hw.head_frames, self.hw.head_measured_frames), (8, 12))

    def test_limits_conversion_shape_and_cache_survives_unavailable(self):
        result = self.hw.read_head_limits_deg()
        self.check_result(result, True, "head limits", ([-90.0, -45.0], [90.0, 45.0]))
        self.assertIsInstance(result.data, tuple)
        self.assertTrue(all(isinstance(x, list) for x in result.data))
        self.client.get_limits.return_value = None
        self.check_result(self.hw.read_head_limits_deg(), False, "get_head_limits unavailable")
        self.assertFalse(self.hw.send_head_target(91, 0).success)
        self.client.send.assert_not_called()
        other = Host(fake())
        self.assertTrue(other.send_head_target(91, 0).success)  # no cache cross-talk

    def test_inclusive_boundaries_and_conversion(self):
        self.hw.read_head_limits_deg()
        for yaw, pitch in [(-90, -45), (90, 45), (0, 0), (30, -20)]:
            with self.subTest(yaw=yaw, pitch=pitch):
                self.check_result(self.hw.send_head_target(yaw, pitch), True,
                                  f"head -> yaw={yaw:.2f} deg, pitch={pitch:.2f} deg")
                self.client.send.assert_called_with(math.radians(yaw), math.radians(pitch))

    def test_outside_each_boundary_never_sends(self):
        self.hw.read_head_limits_deg()
        for yaw, pitch, message in [
            (-90.0001, 0, "yaw -90.0001 deg out of [-90.00, 90.00]"),
            (90.0001, 0, "yaw 90.0001 deg out of [-90.00, 90.00]"),
            (0, -45.0001, "pitch -45.0001 deg out of [-45.00, 45.00]"),
            (0, 45.0001, "pitch 45.0001 deg out of [-45.00, 45.00]")]:
            self.check_result(self.hw.send_head_target(yaw, pitch), False, message)
        self.client.send.assert_not_called()

    def test_nonfinite_and_send_failure(self):
        for value in (math.nan, math.inf, -math.inf):
            for yaw, pitch in [(value, 0), (0, value)]:
                self.check_result(self.hw.send_head_target(yaw, pitch), False,
                                  "yaw/pitch must be finite")
        self.client.send.assert_not_called()
        self.client.send.return_value = False
        self.check_result(self.hw.send_head_target(0, 0), False,
                          "cmd_head publish failed / no subscriber")

    def test_target_measured_and_wait_feedback_are_separate(self):
        current = self.hw.read_head_current_deg()
        measured = self.hw.read_head_measured_deg()
        frame = self.hw.wait_head_frame(7, 0.25)
        self.assertIs(type(current), head.Result)
        self.assertIs(type(measured), head.Result)
        self.assertIs(type(frame), head.Result)
        self.assertEqual(current.message, "current")
        self.assertEqual(measured.message, "measured head positions")
        self.assertEqual(frame.message, "new frame")
        for got, want in zip(current.data, [30, -20]): self.assertAlmostEqual(got, want)
        for got, want in zip(measured.data, [-15, 10]): self.assertAlmostEqual(got, want)
        self.assertEqual(frame.data, current.data)
        self.client.wait_for_new_frame.assert_called_once_with(7, 0.25)
        self.client.wait_measured.assert_not_called()

    def test_missing_feedback_results(self):
        self.client.read_current.return_value = None
        self.client.wait_for_new_frame.return_value = None
        self.client.read_measured.return_value = None
        self.check_result(self.hw.read_head_current_deg(), False, "no current_head_cmd frame")
        self.check_result(self.hw.wait_head_frame(7), False, "no new current_head_cmd frame")
        self.check_result(self.hw.read_head_measured_deg(), False,
                          "no measured head positions (lowstate slots 29/30)")
        self.client.wait_measured.assert_called_once_with(2.0)
        self.assertEqual(self.client.read_measured.call_count, 2)

    def test_first_measured_frame_retry(self):
        self.client.read_measured.side_effect = [None, [math.pi/2, -math.pi/4]]
        self.check_result(self.hw.read_head_measured_deg(), True,
                          "measured head positions", [90.0, -45.0])
        self.client.wait_measured.assert_called_once_with(2.0)

    def test_compat_delegates_public_path_and_preserves_core_shape(self):
        self.hw.send_head_target = Mock(return_value=head.Result.fail(
            "injected", error_code="HEAD_ERROR", data={"detail": 1}))
        result = self.hw.control_head(yaw=12, pitch=-8)
        self.hw.send_head_target.assert_called_once_with(12, -8)
        self.assertIs(type(result), compat.CoreResult)
        self.assertEqual((result.success, result.message, result.data, result.error_code),
                         (False, "injected", {"detail": 1}, "HEAD_ERROR"))
        already_core = compat.CoreResult.ok("core", [1, 2])
        self.hw.send_head_target.return_value = already_core
        self.assertIs(self.hw.control_head(0, 0), already_core)

    def test_compat_real_fake_client_path(self):
        self.check_result(self.hw.control_head(0, 0), True,
                          "head -> yaw=0.00 deg, pitch=0.00 deg", core=True)
        self.check_result(self.hw.control_head(math.nan, 0), False,
                          "yaw/pitch must be finite", core=True)
        self.hw.read_head_limits_deg()
        self.assertFalse(self.hw.control_head(91, 0).success)
        self.client.send.return_value = False
        self.check_result(self.hw.control_head(0, 0), False,
                          "cmd_head publish failed / no subscriber", core=True)

    def test_head_standalone_result_and_client_contracts(self):
        cases = [
            ("wait_head_ready", (), True, "head client ready", None),
            ("read_head_limits_deg", (), True, "head limits", ([-90.0, -45.0], [90.0, 45.0])),
            ("read_head_current_deg", (), True, "current", [30.0, -20.0]),
            ("read_head_measured_deg", (), True, "measured head positions", [-15.0, 10.0]),
            ("wait_head_frame", (7, 0.5), True, "new frame", [30.0, -20.0]),
            ("send_head_target", (90, -45), True, "head -> yaw=90.00 deg, pitch=-45.00 deg", None),
            ("send_head_target", (91, 0), False, "yaw 91 deg out of [-90.00, 90.00]", None),
            ("send_head_target", (0, math.inf), False, "yaw/pitch must be finite", None),
        ]
        for public, args, success, message, data in cases:
            with self.subTest(public=public, args=args):
                result = getattr(self.hw, public)(*args)
                self.assertIs(type(result), compat.CoreResult)
                self.assertEqual((result.success, result.message, result.error_code), (success, message, None))
                if public in ("read_head_current_deg", "read_head_measured_deg", "wait_head_frame"):
                    for actual, expected in zip(result.data, data): self.assertAlmostEqual(actual, expected)
                else: self.assertEqual(result.data, data)
        self.client.wait_for_new_frame.assert_called_once_with(7, 0.5)
        self.client.send.assert_called_once_with(math.pi/2, -math.pi/4)

    def test_existing_type_and_driver_exceptions_are_not_silently_swallowed(self):
        for args in [(None, 0), (0, "bad")]:
            with self.subTest(args=args), self.assertRaises(TypeError):
                self.hw.send_head_target(*args)
        self.client.send.assert_not_called()
        self.client.send.side_effect = RuntimeError("driver failed")
        with self.assertRaisesRegex(RuntimeError, "driver failed"):
            self.hw.control_head(0, 0)

    def test_actual_hardware_mro_facade_and_head_namespace(self):
        self.assertEqual(hardware.Ros2WheeledHardware.__name__, "Ros2WheeledHardware")
        for cls in (hardware.Ros2WheeledHardware,):
            with self.subTest(cls=cls.__name__):
                self.assertTrue(issubclass(cls, IHardware))
                self.assertIs(cls.control_head, compat.IHardwareCompatMixin.control_head)
                self.assertFalse(cls.__abstractmethods__)
                hw = object.__new__(cls)  # never invoke hardware/client/runtime init
                hw._head_client = fake()
                hw._head_limits_rad = None
                hw.client = object()  # unrelated adapter/client names stay untouched
                hw._limits_rad = object()
                unrelated = (hw.client, hw._limits_rad)
                self.assertTrue(hw.read_head_limits_deg().success)
                result = hw.control_head(yaw=30, pitch=-20)
                self.assertIs(type(result), compat.CoreResult)
                self.assertTrue(result.success)
                hw._head_client.send.assert_called_once_with(math.radians(30), math.radians(-20))
                self.assertEqual((hw.client, hw._limits_rad), unrelated)
                self.assertFalse(hw.control_head(91, 0).success)
                self.assertEqual(hw._head_client.send.call_count, 1)

    def test_actual_hardware_close_idempotence_and_failure_cleanup(self):
        names = ("_controller_client", "_locomotion_client", "_jibot_client",
                 "_multibody_client", "_torso_client", "_head_client",
                 "_arm_client", "_runtime")
        for failure in (None, "_head_client"):
            with self.subTest(failure=failure):
                hw = object.__new__(hardware.Ros2WheeledHardware)
                hw._closed = False
                hw._controller_closed = False
                hw._arm_saved_modes = []
                hw.camera = None
                hw.perception = None
                for body in ("arm", "torso", "locomotion", "multibody"):
                    setattr(hw, "_" + body + "_closed", False)
                calls = []
                for name in names:
                    def close(name=name):
                        calls.append(name)
                        if name == failure:
                            raise RuntimeError("fake close failed")
                    setattr(hw, name, Mock(close=Mock(side_effect=close)))
                if failure:
                    with self.assertRaisesRegex(RuntimeError, "fake close failed"):
                        hw.close()
                else:
                    hw.close()
                self.assertEqual(calls, list(names))
                hw.close()
                self.assertEqual(calls, list(names))
                for name in names:
                    getattr(hw, name).close.assert_called_once_with()

    def test_hardware_wiring_canonical_exports_and_close_order_static(self):
        tree = ast.parse((ROOT / "adapters/hardware_ros2/hardware.py").read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
        init = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "__init__")
        assignments = {_unparse(n.targets[0]): _unparse(n.value)
                       for n in init.body if isinstance(n, ast.Assign)}
        self.assertEqual(assignments["self._head_client"], "HeadClient(node=self._runtime.node)")
        self.assertEqual(assignments["self._head_limits_rad"], "None")
        self.assertNotIn("self._head", assignments)
        self.assertNotIn("HeadAdapter", _unparse(tree))
        close = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "close")
        calls = [_unparse(n.func) for n in ast.walk(close)
                 if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                 and (n.func.attr == "close" or n.func.attr.endswith("_close"))]
        self.assertEqual(calls, ["self._controller_close", "self._locomotion_close",
                                "self._jibot_close", "self._multibody_close",
                                "self._torso_close",
                                "self._head_client.close", "self._arm_close", "self._runtime.close"])
        self.assertEqual([n.name for n in tree.body if isinstance(n, ast.ClassDef)],
                         ["Ros2WheeledHardware"])
        self.assertFalse(any(isinstance(n, ast.Assign) and isinstance(n.value, ast.Name)
                             and n.value.id == "Ros2WheeledHardware" for n in tree.body))
        exports = ast.parse((ROOT / "adapters/hardware_ros2/__init__.py").read_text())
        names = next(ast.literal_eval(n.value) for n in exports.body
                     if isinstance(n, ast.Assign) and _unparse(n.targets[0]) == "__all__")
        self.assertEqual(names, ["Ros2Runtime", "Ros2WheeledHardware"])


# The same import guards cover all bodies and the actual unified hardware MRO.
BODY_CLASSES = {"arm": hardware.ArmMixin, "torso": hardware.TorsoMixin,
                "locomotion": hardware.LocomotionMixin,
                "multibody": hardware.MultibodyMixin}
def body_fake(success=True):
    reply = types.SimpleNamespace(success=success, message="fake reply", value=0,
                                  actual_time=2.5, vector=[0.1] * 14,
                                  torso=[0.2] * 4, linear_error=0.001,
                                  poses={side: {"pos": [0.1, 0.2, 0.3],
                                                "quat": [0.0, 0.0, 0.0, 1.0]}
                                         for side in ("left", "right")},
                                  limits=types.SimpleNamespace(
                                      linear_x_min=-1, linear_x_max=1,
                                      linear_y_min=-2, linear_y_max=2,
                                      angular_z_min=-3, angular_z_max=3))
    c = Mock(reach_frames=9, reach_value=1.5)
    bool_methods = ("wait_ready", "wait_for_consumer", "service_available",
                    "reach_topic_available", "send_joint_positions", "send_joint_trajectory",
                    "cancel_joint_trajectory", "send_ee_poses", "send_ee_trajectory",
                    "cancel_ee_trajectory", "send_torso_pose", "send_leg_joints",
                    "publish_velocity_odom", "publish_velocity_base", "publish_pose_odom",
                    "publish_pose_base", "set_focus_z", "set_focus_ee")
    for name in bool_methods:
        getattr(c, name).return_value = success
    for name in ("send_timed_joint_target", "send_timed_ee_target", "get_arm_mode",
                 "get_wheel_mode", "get_quick_mode", "get_reach_error",
                 "check_accessibility", "set_async_exec", "execute_arm_action",
                 "interrupt_arm_traj", "solve_fk", "solve_ik", "solve_fk_with_frame",
                 "publish_ee_poses_ik", "publish_ee_poses_advance_ik",
                 "send_timed_torso_target", "send_timed_leg_target", "reset_torso",
                 "send_timed_target", "get_velocity_limits", "reset_vel_ruckig",
                 "set_offline_traj", "enable_offline_traj", "wait_for_reach_time"):
        getattr(c, name).return_value = reply
    c.get_torso_init_pose.return_value = types.SimpleNamespace(
        success=success, message="fake reply", vector=[0.1, 0.2, math.pi/2, -math.pi/4])
    c.get_controller.return_value = types.SimpleNamespace(
        current="wheel_mpc", switching=False, pending="") if success else None
    c.read_arm_positions.return_value = [0.1] * 14 if success else None
    c.read_model_positions.return_value = [0.2] * 18 if success else None
    c.read_joint_torques.return_value = list(range(35)) if success else None
    c.read_arm_control_mode.return_value = 2 if success else None
    c.arm_control_mode_frames.return_value = 3
    # Setters return requested readback values, exactly as a mode service does.
    for name in ("set_arm_mode", "set_wheel_mode", "set_quick_mode"):
        getattr(c, name).side_effect = lambda *a, **k: types.SimpleNamespace(
            success=success, message="fake reply", value=a[-2])
    return c


def body_host(body, client):
    host = object.__new__(hardware.Ros2WheeledHardware)
    setattr(host, "_" + body + "_client", client)
    setattr(host, "_" + body + "_closed", False)
    host._arm_saved_modes = []
    return host


Q7 = [10, -20, 30, -40, 50, -60, 70]
Q14 = Q7 + [-v for v in Q7]
POSE6 = [0.1, 0.2, 0.3, 90, -45, 30]
POSE7 = [0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0]
CASES = {
    "arm": [
        ("wait_ready", (0.25,), {}), ("wait_for_consumer", ("cmd_joints", 0.2), {}),
        ("read_arm_positions_deg", (), {}), ("read_model_positions_deg", (), {}),
        ("read_joint_torques", (), {}), ("controller", (), {}), ("get_modes", (), {}),
        ("service_available", ("ik", 0.1), {}),
        ("get_arm_control_mode", (), {}), ("get_arm_control_mode_topic", (), {}),
        ("set_arm_quick_mode", (3,), {}), ("set_arm_control_mode", (2,), {}),
        ("set_wheel_control_mode", (1,), {}), ("prepare_modes", (), {}),
        ("restore_modes", (), {}), ("send_arm_joint_positions", (Q14,), {}),
        ("send_arm_joint_trajectory", ([0, 2], [Q14, Q14]), {}),
        ("cancel_arm_joint_trajectory", (), {}), ("send_ee_poses", (POSE7, POSE7, 0), {}),
        ("send_ee_trajectory", ([0, 2], [POSE7, POSE7], [POSE7, POSE7], 1), {}),
        ("cancel_ee_trajectory", (), {}),
        ("send_timed_joint_target", ("left", Q7, 2, 0.2), {}),
        ("send_timed_ee_target", ("right", POSE6, 3, 0, 0.2), {}),
        ("wait_for_reach_time", ("left", "joint", 0, 0.1, 2.5), {}),
        ("reach_topic_available", ("right", "ee"), {}),
        ("reach_value", ("left", "joint"), {}),
        ("get_reach_error", ("left",), {}),
        ("check_accessibility", ("right", POSE6), {"frame": 0, "is_whole_body": True}),
        ("set_async_exec", (True,), {}), ("execute_arm_action", ("fake",), {}),
        ("interrupt_arm_traj", (), {}), ("solve_fk", (Q14,), {}),
        ("solve_fk_with_frame", (Q14, "base", "ee", 1, 0.1), {}),
        ("solve_ik", ("left", POSE6), {"frame": 0, "other_pose_zyx_deg": POSE6,
                                          "arm_positions_deg": Q14, "multi": True,
                                          "constraint_mode": 1}),
        ("send_ee_poses_ik", (POSE6, POSE6, 0, 2), {}),
        ("send_ee_poses_advance_ik", (POSE6, POSE6, 1, Q14, 2), {}),
    ],
    "torso": [
        ("read_torso_init_pose_deg", (0.2,), {}),
        ("send_torso_pose_timed", ([0.1, 0.2, 90, -45], 2, 0.2), {}),
        ("send_leg_joints_timed", (Q7[:4], 2, 0.2), {}),
        ("send_torso_pose_frame", ([0.1, 0.2, 90, -45],), {}),
        ("send_leg_joints_frame", (Q7[:4],), {}), ("reset_torso", (0.2,), {}),
        ("reach_topic_available", ("torso",), {}),
        ("reach_frames", ("leg",), {}), ("reach_value", ("torso",), {}),
        ("wait_for_reach_time", ("leg", 0, 0.2, 2.5), {}),
    ],
    "locomotion": [
        ("send_base_velocity", (0.1, 0.2, 90), {"frame": "world", "duration": 0}),
        ("send_base_velocity_tick", (0.1, 0.2, 0.3), {}),
        ("send_base_pose", (0.1, 0.2, 90), {"frame": "local"}),
        ("send_base_pose_timed", (0.1, 0.2, 90), {"frame": "world", "desire_time": 2}),
        ("get_velocity_limits", (0.2,), {}), ("reset_vel_ruckig", (0.2,), {}),
        ("reach_topic_available", (), {}), ("reach_frames", None, {}),
        ("reach_value", None, {}), ("wait_for_reach_time", (0, 0.2, 2.5), {}),
    ],
    "multibody": [
        ("send_timed_target", ([{"planner_index": i, "desire_time": 2,
                                  "cmd_vec": [10] * n}
                                 for i, n in enumerate([3, 3, 4, 4, 6, 6, 6, 6, 7, 7])], False), {}),
        ("set_offline_traj", ([{"planner_index": i, "frame": 1,
                                "points": [{"desire_time": 0, "cmd_vec": [10] * n},
                                           {"desire_time": 2, "cmd_vec": [20] * n}]}
                               for i, n in enumerate([6, 6, 4, 7])],), {}),
        ("set_focus_z", (True,), {}), ("set_focus_ee", (False,), {}),
        ("enable_offline_traj", (True,), {}),
    ],
}


class BodyContractTests(unittest.TestCase):
    def contract(self, body, name, args, kwargs, success=True, exception=None):
        client = body_fake(success)
        host = body_host(body, client)
        if body in ("arm", "torso"):
            client.wait_for_reach_time.return_value = None
            client.reach_frames = Mock(return_value=9)
            client.reach_value = Mock(return_value=1.5)
        if exception:
            getattr(client, exception[0]).side_effect = exception[1]
        key = repr((body, name, args, kwargs, success, None if exception is None else
                    (exception[0], type(exception[1]).__name__, str(exception[1]))))
        expected, calls = BODY_OUTCOMES[key]
        try:
            value = getattr(host, "_" + body + "_" + name)
            value = value if args is None else value(*args, **kwargs)
            if isinstance(value, head.Result):
                self.assertIs(type(value), compat.CoreResult)
                outcome = ("Result", vars(value))
            else: outcome = (type(value).__name__, value)
        except Exception as exc:
            outcome = (type(exc).__name__, str(exc))
        self.assertEqual(_norm_result(outcome), _norm_result(expected))
        self.assertEqual([repr(call) for call in client.mock_calls], calls)
        return host, client

    def test_all_substantive_method_signature_contracts(self):
        for body in BODY_CLASSES:
            tree = ast.parse((ROOT / ("adapters/hardware_ros2/mixins/" + body + "_mixin.py")).read_text())
            cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
            actual = {n.name: (_canon_dump(n.args), _canon_dump(n.returns) if n.returns else None,
                              tuple(_canon_dump(d) for d in n.decorator_list))
                      for n in cls.body if isinstance(n, ast.FunctionDef) and n.name.startswith("_" + body + "_")}
            self.assertEqual(actual, BODY_SIGNATURES[body])

    def test_ee_poses_accept_zyx_degrees_and_convert(self):
        h = body_host("arm", body_fake())
        result = h.send_ee_poses([0.5, 0.0, 1.0, 90.0, 0.0, 0.0],
                                 [0.5, 0.0, 1.0, 0.0, 0.0, 0.0])
        self.assertTrue(result.success)
        left, right, frame = h._arm_client.send_ee_poses.call_args.args
        self.assertEqual(len(left), 7)
        self.assertAlmostEqual(left[5], math.sin(math.pi / 4), places=6)
        self.assertAlmostEqual(left[6], math.cos(math.pi / 4), places=6)
        self.assertEqual(right, [0.5, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
        self.assertEqual(frame, 1)

        h._arm_client.send_ee_poses.reset_mock()
        self.assertTrue(h.send_ee_poses([0.1, 0.2, 0.3, 0, 0, 0, 1],
                                        [0.1, 0.2, 0.3, 0, 0, 0, 1]).success)
        self.assertEqual(h._arm_client.send_ee_poses.call_args.args[0],
                         [0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0])

        bad = h.send_ee_poses([0.1, 0.2, 0.3], [0.1, 0.2, 0.3])
        self.assertFalse(bad.success)
        self.assertEqual(bad.error_code, "INVALID_ARGUMENT")

        h._arm_client.send_ee_trajectory.reset_mock()
        trajectory = h.send_ee_traj([0.0, 1.0],
                                    [[0.5, 0.0, 1.0, 90.0, 0.0, 0.0]] * 2,
                                    [[0.5, 0.0, 1.0, 0.0, 0.0, 0.0]] * 2)
        self.assertTrue(trajectory.success)
        times, left_traj, _right_traj, frame2 =             h._arm_client.send_ee_trajectory.call_args.args
        self.assertEqual(times, [0.0, 1.0])
        self.assertEqual(len(left_traj[0]), 7)
        self.assertEqual(frame2, 1)

    def test_public_wrapper_api_and_routing_coverage(self):
        """Every old public mixin wrapper is exercised on the actual hardware MRO."""
        for body, cls in BODY_CLASSES.items():
            for name, method in cls.__dict__.items():
                if name.startswith("_"): continue
                fn = method.fget if isinstance(method, property) else method
                import inspect
                import textwrap
                node = ast.parse(textwrap.dedent(inspect.getsource(fn)))
                refs = [n for n in ast.walk(node) if isinstance(n, ast.Attribute)
                        and isinstance(n.value, ast.Name) and n.value.id == "self"
                        and n.attr.startswith("_" + body + "_")]
                self.assertTrue(refs, body + "." + name)
                host = body_host(body, body_fake())
                if body == "arm" and name == "arm_ik_result_frames":
                    host._arm_client.ik_result_frames = 9
                    self.assertEqual(host.arm_ik_result_frames, 9)
                    self.assertEqual(host._arm_client.mock_calls, [])
                    continue
                if body == "arm" and name == "read_arm_ik_result_deg":
                    host._arm_client.read_ik_result.return_value = [math.pi / 2] * 14
                    result = host.read_arm_ik_result_deg()
                    self.assertTrue(result.success)
                    self.assertEqual(result.data, [90.0] * 14)
                    host._arm_client.read_ik_result.assert_called_once_with()
                    continue
                if isinstance(method, property):
                    self.assertEqual(getattr(host, name), getattr(host, refs[0].attr))
                    self.assertEqual(getattr(host, name), 9 if "frames" in name else 1.5)
                    continue
                for ref in refs:
                    setattr(host, ref.attr, Mock(return_value=head.Result.ok("routed", list(range(18)))))
                values = {"side": "left", "kind": "joint", "topic": "fake",
                          "pose_zyx_deg": POSE6, "left_zyx_deg": POSE6,
                          "right_zyx_deg": POSE6, "positions_deg": Q14,
                          "joints_deg": Q7, "times": [0, 2], "left": POSE7,
                          "right": POSE7, "commands": [], "trajectories": [],
                          "pose_deg": [0, 0, 0, 0], "action_name": "fake"}
                if name == "send_ee_traj":
                    values["left"] = [POSE7, POSE7]
                    values["right"] = [POSE7, POSE7]
                args = [values.get(p.name, 0.1) for p in inspect.signature(fn).parameters.values()
                        if p.name != "self" and p.default is inspect.Parameter.empty
                        and p.kind not in (p.VAR_POSITIONAL, p.VAR_KEYWORD)]
                with self.subTest(body=body, wrapper=name):
                    fn(host, *args)
                    self.assertTrue(any(getattr(host, ref.attr).called for ref in refs))

    def test_arm_conversion_and_7_plus_7_aliases(self):
        h = body_host("arm", body_fake())
        h._multibody_client = body_fake()
        h.send_two_arm_joint_command(Q7, Q7, frame=5)
        h._arm_client.send_joint_positions.assert_called_once_with([math.radians(v) for v in Q7 + Q7])
        h.send_arm_joint_timed("right", Q7, 1.25)
        h._arm_client.send_timed_joint_target.assert_called_once_with(
            "right", [math.radians(v) for v in Q7], 1.25, 5.0)
        h.send_arm_joint_timed(Q14, 1.5)
        commands, sync = h._multibody_client.send_timed_target.call_args.args
        self.assertTrue(sync)
        self.assertEqual([c["planner_index"] for c in commands], [8, 9])
        self.assertEqual(commands[0]["cmd_vec"], [math.radians(v) for v in Q7])
        h.send_two_arm_hand_pose(POSE6, POSE6, frame=2)
        left, right, frame = h._arm_client.send_ee_poses.call_args.args
        self.assertEqual(frame, 1)
        self.assertEqual(left[:3], POSE6[:3])
        self.assertEqual(left, right)
        self.assertAlmostEqual(sum(v*v for v in left[3:]), 1)
        result = h.get_arm_joint_positions()
        self.assertIs(type(result), compat.CoreResult)
        for v in result.data: self.assertAlmostEqual(v, 0.1)
        h.send_timed_left_arm_ee_world(POSE6, 1.0)
        self.assertEqual(h._arm_client.send_timed_ee_target.call_args.args,
                         ("left", POSE6[:3] + [math.radians(v) for v in POSE6[3:]], 1.0, 0, 5.0))

    def test_multibody_all_mappings_and_distinct_planner3(self):
        h = body_host("multibody", body_fake())
        for idx, size in enumerate([3, 3, 4, 4, 6, 6, 6, 6, 7, 7]):
            result = h.send_multi_body_timed([{"planner_index": idx, "desire_time": 2,
                                             "cmd_vec": [90] * size}])
            self.assertTrue(result.success)
            converted = h._multibody_client.send_timed_target.call_args.args[0][0]["cmd_vec"]
            angle_start = {0: 2, 1: 2, 2: 2, 3: 0, 4: 3, 5: 3, 6: 3, 7: 3, 8: 0, 9: 0}[idx]
            self.assertEqual(converted, [90.0] * angle_start + [math.pi/2] * (size-angle_start))
        offline = [{"planner_index": 3, "points": [{"desire_time": 0, "cmd_vec": Q7}]}]
        self.assertTrue(h.set_multi_body_offline_traj(offline).success)
        self.assertEqual(h._multibody_client.set_offline_traj.call_args.args[0][0]["points"][0]["cmd_vec"],
                         [math.radians(v) for v in Q7])
        self.assertFalse(h.send_multi_body_timed([{"planner_index": 3, "desire_time": 2, "cmd_vec": Q7}]).success)
        self.assertFalse(h.set_multi_body_offline_traj([
            {"planner_index": 3, "points": [{"desire_time": 0, "cmd_vec": Q7[:4]}]}]).success)
        self.assertEqual(offline[0]["points"][0]["cmd_vec"], Q7)  # never mutate caller inputs

    def test_base_frames_yaw_and_public_core_facade(self):
        from core.domain.enums import FrameType
        h = body_host("locomotion", body_fake())
        for frame, world in [("world", True), ("odom", True), (0, True), (FrameType.WORLD, True),
                             ("local", False), ("base", False), ("body", False), (1, False), (FrameType.LOCAL, False)]:
            with self.subTest(frame=frame):
                r = h.send_base_pose(0.1, 0.2, 90, frame=frame)
                self.assertIs(type(r), compat.CoreResult)
                publish = h._locomotion_client.publish_pose_odom if world else h._locomotion_client.publish_pose_base
                publish.assert_called_with(0.1, 0.2, math.pi/2)
                h.send_base_pose_timed(0.1, 0.2, 90, frame=frame)
                self.assertEqual(h._locomotion_client.send_timed_target.call_args.args,
                                 (0 if world else 1, 3.0, [0.1, 0.2, math.pi/2]))
                h._locomotion_send_base_velocity(0.1, 0.2, 90, frame=frame, duration=0)
                publish = h._locomotion_client.publish_velocity_odom if world else h._locomotion_client.publish_velocity_base
                publish.assert_called_with(0.1, 0.2, math.pi/2)
        h.send_base_velocity_tick(0.1, 0.2, 0.3)
        h._locomotion_client.publish_velocity_base.assert_called_with(0.1, 0.2, 0.3)
        self.assertEqual(h.send_base_pose(0, 0, 0, "invalid").error_code, "INVALID_ARGUMENT")
        # The facade keeps the legacy default duration and zeroing behavior; fake clock only.
        clock = hardware.LocomotionMixin._locomotion_send_base_velocity.__globals__["time"]
        with patch.object(clock, "monotonic", side_effect=[0, 0, 6]), patch.object(clock, "sleep"):
            self.assertIs(type(h.send_base_velocity(0.1, 0.2, 90)), compat.CoreResult)
        self.assertEqual(h._locomotion_client.publish_velocity_base.call_args.args, (0.0, 0.0, 0.0))

    def test_compat_skill_torso_focus_trajectory_and_reach_routes(self):
        h = body_host("arm", body_fake())
        h._torso_client, h._multibody_client, h._locomotion_client = body_fake(), body_fake(), body_fake()
        h._arm_client.reach_value = Mock(return_value=1.5)
        h._torso_client.reach_value = Mock(return_value=2.5)
        from core.domain.pose import Pose6D
        h.send_torso_pose(Pose6D(0.1, 0, 0.2, math.pi/2, -math.pi/4, 0))
        h._torso_client.send_torso_pose.assert_called_with([0.1, 0.2, math.pi/2, -math.pi/4])
        h.send_timed_torso_pose(0.1, 0.2, 90, -45, 1.0)
        h._torso_client.send_timed_torso_target.assert_called_with([0.1, 0.2, math.pi/2, -math.pi/4], 1.0, 5.0)
        h.send_leg_joint_positions(["a"] * 4, Q7[:4])
        h._torso_client.send_leg_joints.assert_called_with([math.radians(v) for v in Q7[:4]])
        h.reset_torso_to_initial()
        h._torso_client.reset_torso.assert_called_with(True, 5.0)
        for name in ("set_focus_z", "set_focus_ee"):
            self.assertIs(type(getattr(h, name)(True)), compat.CoreResult)
            getattr(h._multibody_client, name).assert_called_with(True)
        h.set_arm_quick_mode(3)
        h._arm_client.set_quick_mode.assert_called_with("wheeled", 3, 5.0)
        h.send_joint_trajectory(["a"] * 14, [0, 2], [Q14, Q14])
        h._arm_client.send_joint_trajectory.assert_called_with([0.0, 2.0], [[math.radians(v) for v in Q14]] * 2)
        for key, value in [("cmd_pose", 1.5), ("torso_pose", 2.5), ("leg_joint", 2.5),
                           ("arm_joint", 1.5), ("arm_ee", 1.5), ("right_ee", 1.5)]:
            self.assertEqual(h.get_reach_time(key), value)
        r = h.set_offline_trajectory([{"planner_index": 3, "timed_traj": [
            {"desire_time": 0, "cmd_vec": Q7}]}])
        self.assertIs(type(r), compat.CoreResult)
        self.assertTrue(r.success)

    def test_compat_publish_arm_joint_setpoint_nonblocking_frame_route(self):
        h = body_host("arm", body_fake())
        r = h.publish_arm_joint_setpoint(Q14)
        self.assertIs(type(r), compat.CoreResult)
        self.assertEqual(r.message,
                         "one arm joint frame published; not a motion completion acknowledgement")
        h._arm_client.send_joint_positions.assert_called_once_with([math.radians(v) for v in Q14])
        bad = h.publish_arm_joint_setpoint([0.0] * 13)
        self.assertIs(type(bad), compat.CoreResult)
        self.assertEqual((bad.success, bad.message), (False, "expect 14 joint angles, got 13"))
        h._arm_client.send_joint_positions.assert_called_once()  # wrong length never reaches the client

    def test_actual_mro_ik_core_and_arm_wrapper_are_separate(self):
        h = body_host("arm", body_fake())
        request = compat.IkSolveRequest(left_pose=POSE7, right_pose=POSE7)
        r = h.solve_ik(request)
        self.assertIs(type(r), compat.CoreResult)
        self.assertTrue(r.success, r.message)
        self.assertIsInstance(r.data, compat.IkSolution)
        for v in r.data.q_arm: self.assertAlmostEqual(v, 0.1)
        r = h.solve_arm_ik("left", POSE6)
        self.assertIs(type(r), head.Result)
        self.assertTrue(r.success)
        self.assertFalse(h.solve_ik(None).success)

    def test_modes_restore_lifo_failures_and_instance_isolation(self):
        h = body_host("arm", body_fake())
        other = body_host("arm", body_fake())
        self.assertTrue(h.prepare_arm_modes(quick_source=3).success)
        self.assertEqual([s[0] for s in h._arm_saved_modes], ["wheel", "arm", "wheel_quick"])
        self.assertEqual(other._arm_saved_modes, [])
        self.assertFalse(h.prepare_arm_modes().success)
        h._arm_client.reset_mock()
        self.assertTrue(h.restore_arm_modes().success)
        self.assertEqual([c[0] for c in h._arm_client.mock_calls],
                         ["set_quick_mode", "set_arm_mode", "set_wheel_mode"])
        self.assertEqual(h._arm_saved_modes, [])
        setters = [Mock(return_value=types.SimpleNamespace(success=True)),
                   Mock(side_effect=RuntimeError("restore exception")),
                   Mock(return_value=types.SimpleNamespace(success=False, message="rejected"))]
        h._arm_saved_modes = [(str(i), setter, i) for i, setter in enumerate(setters)]
        result = h.restore_arm_modes()
        self.assertFalse(result.success)
        self.assertIn("restore exception", result.message)
        self.assertIn("rejected", result.message)
        for i, setter in enumerate(setters): setter.assert_called_once_with(i, 5.0)
        self.assertEqual(h._arm_saved_modes, [])
        h._arm_saved_modes = [("wheel", setters[1], 0)]
        h._arm_close()
        h._arm_client.node.get_logger.return_value.error.assert_called_once()
        h._arm_client.close.assert_called_once()

    def test_prepare_failure_rolls_back_contract(self):
        for cause, message in [
            (types.SimpleNamespace(success=False, message="rejected", value=None), "cannot set arm=2: rejected"),
            (RuntimeError("mode failed"), "prepare arm modes failed: mode failed"),
        ]:
            h = body_host("arm", body_fake())
            if isinstance(cause, Exception): h._arm_client.set_arm_mode.side_effect = cause
            else:
                h._arm_client.set_arm_mode.side_effect = None
                h._arm_client.set_arm_mode.return_value = cause
            self.assertEqual(vars(h.prepare_arm_modes()),
                             {"success": False, "message": message, "data": None, "error_code": None})
            self.assertEqual([repr(c) for c in h._arm_client.mock_calls], [
                "call.get_controller(5.0)", "call.get_wheel_mode(5.0)", "call.set_wheel_mode(1, 5.0)",
                "call.get_arm_mode(5.0)", "call.set_arm_mode(2, 5.0)", "call.set_wheel_mode(0, 5.0)"])
            self.assertEqual(h._arm_saved_modes, [])

    def test_each_close_failure_and_restoration_cleanup_order(self):
        names = ("_controller_client", "_locomotion_client", "_jibot_client",
                 "_multibody_client", "_torso_client", "_head_client",
                 "_arm_client", "_runtime")
        for failure in names + ("restore", "logger"):
            h = object.__new__(hardware.Ros2WheeledHardware)
            h._closed = False
            h._controller_closed = False
            h._arm_saved_modes = []
            h.camera = None
            h.perception = None
            calls = []
            for body in BODY_CLASSES: setattr(h, "_" + body + "_closed", False)
            for name in names:
                def close(name=name):
                    calls.append(name)
                    if name == failure: raise RuntimeError(name)
                setattr(h, name, Mock(close=Mock(side_effect=close)))
            def restore():
                calls.append("restore")
                if failure == "restore": raise RuntimeError("restore")
                return head.Result.fail("restore rejected") if failure == "logger" else head.Result.ok()
            h._arm_restore_modes = restore
            if failure == "logger":
                h._arm_client.node.get_logger.return_value.error.side_effect = RuntimeError("logger")
            with self.subTest(failure=failure), self.assertRaises(RuntimeError): h.close()
            self.assertEqual(calls, list(names[:6]) + ["restore"] + list(names[6:]))
            h.close()
            self.assertEqual(len(calls), 9)

    def test_no_body_adapter_imports_nodes_or_shared_helper_names(self):
        for relative in ["adapters/hardware_ros2/hardware.py",
                         "adapters/hardware_ros2/ihardware_compat.py"] + [
                             "adapters/hardware_ros2/mixins/" + b + "_mixin.py"
                             for b in BODY_CLASSES] + ["adapters/hardware_ros2/mixins/skill_compat_mixin.py"]:
            tree = ast.parse((ROOT / relative).read_text())
            for n in ast.walk(tree):
                if isinstance(n, ast.ImportFrom):
                    self.assertNotIn(n.module, {b + "_adapter" for b in BODY_CLASSES})
                if isinstance(n, ast.Attribute) and isinstance(n.value, ast.Name) and n.value.id == "self":
                    self.assertNotIn(n.attr, {"client", "_arm", "_torso", "_multibody", "_locomotion"})
                if isinstance(n, ast.Call) and isinstance(n.func, ast.Name):
                    self.assertNotIn(n.func.id, {"Node", "ArmAdapter", "TorsoAdapter",
                                               "LocomotionAdapter", "MultibodyAdapter"})
        tree = ast.parse((ROOT / "adapters/hardware_ros2/hardware.py").read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
        init = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "__init__")
        assignments = {_unparse(n.targets[0]): _unparse(n.value)
                       for n in init.body if isinstance(n, ast.Assign)}
        for body in BODY_CLASSES:
            self.assertEqual(assignments["self._" + body + "_client"],
                             body.title() + "Client(node=self._runtime.node)")
        self.assertEqual(assignments["self._arm_saved_modes"], "[]")
        self.assertEqual(assignments["self._controller_client"], "ControllerClient(node=self._runtime.node)")
        self.assertEqual(assignments["self._controller_closed"], "False")


# Independently reported runtime contract cases include success and failure paths.
for _body, _cases in CASES.items():
    for _name, _args, _kwargs in _cases:
        for _success in (True, False):
            def _test(self, body=_body, name=_name, args=_args, kwargs=_kwargs, success=_success):
                self.contract(body, name, args, kwargs, success)
            setattr(BodyContractTests, "test_contract_" + _body + "_" + _name +
                    ("_success" if _success else "_failure"), _test)

for _body, _name, _args, _driver in [
    ("arm", "send_arm_joint_positions", (Q14,), "send_joint_positions"),
    ("arm", "solve_ik", ("left", POSE6), "solve_ik"),
    ("arm", "get_reach_error", ("left",), "get_reach_error"),
    ("torso", "send_torso_pose_frame", ([0, 0, 90, 0],), "send_torso_pose"),
    ("locomotion", "send_base_pose", (0, 0, 90), "publish_pose_odom"),
    ("multibody", "send_timed_target", ([{"planner_index": 3, "desire_time": 2, "cmd_vec": [0]*4}],), "send_timed_target")]:
    for _exc_type in (ValueError, TypeError, RuntimeError):
        def _test(self, body=_body, name=_name, args=_args, driver=_driver, exc_type=_exc_type):
            self.contract(body, name, args, {}, exception=(driver, exc_type("injected")))
        setattr(BodyContractTests, "test_exception_" + _body + "_" + _name + "_" + _exc_type.__name__, _test)


PUBLIC_SIGNATURES = {'adapters/hardware_ros2/ihardware_compat.py': {'initialize': ("arguments(posonlyargs=[], args=[arg(arg='self')], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='CoreResult', ctx=Load())", ()), 'shutdown': ("arguments(posonlyargs=[], args=[arg(arg='self')], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='CoreResult', ctx=Load())", ()), 'is_connected': ("arguments(posonlyargs=[], args=[arg(arg='self')], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='bool', ctx=Load())", ("Name(id='property', ctx=Load())",)), 'move_chassis_relative': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='x', annotation=Name(id='float', ctx=Load())), arg(arg='y', annotation=Name(id='float', ctx=Load())), arg(arg='theta', annotation=Name(id='float', ctx=Load())), arg(arg='options')], kwonlyargs=[], kw_defaults=[], defaults=[Constant(value=None)])", "Name(id='CoreResult', ctx=Load())", ()), 'move_chassis_to_target': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='x', annotation=Name(id='float', ctx=Load())), arg(arg='y', annotation=Name(id='float', ctx=Load())), arg(arg='theta', annotation=Name(id='float', ctx=Load())), arg(arg='options')], kwonlyargs=[], kw_defaults=[], defaults=[Constant(value=None)])", "Name(id='CoreResult', ctx=Load())", ()), 'check_chassis_arrived': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='task_id', annotation=Name(id='str', ctx=Load())), arg(arg='blocking', annotation=Name(id='bool', ctx=Load())), arg(arg='timeout', annotation=Name(id='float', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[Constant(value=True), Constant(value=20.0)])", "Name(id='CoreResult', ctx=Load())", ()), 'set_chassis_external_control': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='enable', annotation=Name(id='bool', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='CoreResult', ctx=Load())", ()), 'get_chassis_external_control_state': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='timeout', annotation=Name(id='float', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[Constant(value=3.0)])", "Name(id='CoreResult', ctx=Load())", ()), 'send_base_velocity_timed': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='vx', annotation=Name(id='float', ctx=Load())), arg(arg='vy', annotation=Name(id='float', ctx=Load())), arg(arg='vyaw', annotation=Name(id='float', ctx=Load())), arg(arg='frame', annotation=Name(id='FrameType', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[Attribute(value=Name(id='FrameType', ctx=Load()), attr='LOCAL', ctx=Load())])", "Name(id='CoreResult', ctx=Load())", ()), 'send_base_velocity': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='vx', annotation=Name(id='float', ctx=Load())), arg(arg='vy', annotation=Name(id='float', ctx=Load())), arg(arg='vyaw', annotation=Name(id='float', ctx=Load())), arg(arg='frame', annotation=Name(id='FrameType', ctx=Load())), arg(arg='duration', annotation=Subscript(value=Name(id='Optional', ctx=Load()), slice=Name(id='float', ctx=Load()), ctx=Load())), arg(arg='rate_hz', annotation=Subscript(value=Name(id='Optional', ctx=Load()), slice=Name(id='float', ctx=Load()), ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[Attribute(value=Name(id='FrameType', ctx=Load()), attr='LOCAL', ctx=Load()), Constant(value=None), Constant(value=None)])", "Name(id='CoreResult', ctx=Load())", ()), 'send_base_pose': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='x', annotation=Name(id='float', ctx=Load())), arg(arg='y', annotation=Name(id='float', ctx=Load())), arg(arg='yaw', annotation=Name(id='float', ctx=Load())), arg(arg='frame', annotation=Name(id='FrameType', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[Attribute(value=Name(id='FrameType', ctx=Load()), attr='WORLD', ctx=Load())])", "Name(id='CoreResult', ctx=Load())", ()), 'send_timed_base_pose': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='x', annotation=Name(id='float', ctx=Load())), arg(arg='y', annotation=Name(id='float', ctx=Load())), arg(arg='yaw', annotation=Name(id='float', ctx=Load())), arg(arg='desire_time', annotation=Name(id='float', ctx=Load())), arg(arg='frame', annotation=Name(id='FrameType', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[Attribute(value=Name(id='FrameType', ctx=Load()), attr='WORLD', ctx=Load())])", "Name(id='CoreResult', ctx=Load())", ()), 'send_torso_pose': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='pose', annotation=Name(id='Pose6D', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='CoreResult', ctx=Load())", ()), 'get_torso_initial_pose': ("arguments(posonlyargs=[], args=[arg(arg='self')], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='CoreResult', ctx=Load())", ()), 'send_timed_torso_pose': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='x', annotation=Name(id='float', ctx=Load())), arg(arg='z', annotation=Name(id='float', ctx=Load())), arg(arg='yaw', annotation=Name(id='float', ctx=Load())), arg(arg='pitch', annotation=Name(id='float', ctx=Load())), arg(arg='desire_time', annotation=Name(id='float', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='CoreResult', ctx=Load())", ()), 'send_timed_leg_joint': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='joint_angles', annotation=Subscript(value=Name(id='List', ctx=Load()), slice=Name(id='float', ctx=Load()), ctx=Load())), arg(arg='desire_time', annotation=Name(id='float', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='CoreResult', ctx=Load())", ()), 'send_leg_joint_command': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='positions', annotation=Subscript(value=Name(id='List', ctx=Load()), slice=Name(id='float', ctx=Load()), ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='CoreResult', ctx=Load())", ()), 'set_focus_z': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='focus_z', annotation=Name(id='bool', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[Constant(value=True)])", "Name(id='CoreResult', ctx=Load())", ()), 'set_focus_ee': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='focus_ee', annotation=Name(id='bool', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[Constant(value=True)])", "Name(id='CoreResult', ctx=Load())", ()), 'send_ee_pose': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='side', annotation=Name(id='ArmSide', ctx=Load())), arg(arg='pose', annotation=Name(id='Pose6D', ctx=Load())), arg(arg='frame', annotation=Name(id='FrameType', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[Attribute(value=Name(id='FrameType', ctx=Load()), attr='WORLD', ctx=Load())])", "Name(id='CoreResult', ctx=Load())", ()), 'send_both_ee_poses': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='left_pose', annotation=Name(id='Pose6D', ctx=Load())), arg(arg='right_pose', annotation=Name(id='Pose6D', ctx=Load())), arg(arg='frame', annotation=Name(id='FrameType', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[Attribute(value=Name(id='FrameType', ctx=Load()), attr='WORLD', ctx=Load())])", "Name(id='CoreResult', ctx=Load())", ()), 'send_arm_joint_trajectory': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='positions', annotation=Subscript(value=Name(id='List', ctx=Load()), slice=Name(id='float', ctx=Load()), ctx=Load())), arg(arg='time_sec', annotation=Name(id='float', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[Constant(value=0.0)])", "Name(id='CoreResult', ctx=Load())", ()), 'publish_arm_joint_setpoint': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='positions_deg', annotation=Subscript(value=Name(id='List', ctx=Load()), slice=Name(id='float', ctx=Load()), ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='CoreResult', ctx=Load())", ()), 'send_timed_left_arm_joint': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='joint_angles', annotation=Subscript(value=Name(id='List', ctx=Load()), slice=Name(id='float', ctx=Load()), ctx=Load())), arg(arg='desire_time', annotation=Name(id='float', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='CoreResult', ctx=Load())", ()), 'send_timed_right_arm_joint': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='joint_angles', annotation=Subscript(value=Name(id='List', ctx=Load()), slice=Name(id='float', ctx=Load()), ctx=Load())), arg(arg='desire_time', annotation=Name(id='float', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='CoreResult', ctx=Load())", ()), 'send_timed_multi_commands': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='commands', annotation=Subscript(value=Name(id='List', ctx=Load()), slice=Name(id='dict', ctx=Load()), ctx=Load())), arg(arg='is_sync', annotation=Name(id='bool', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[Constant(value=False)])", "Name(id='CoreResult', ctx=Load())", ()), 'send_timed_left_arm_ee_world': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='pose', annotation=Subscript(value=Name(id='List', ctx=Load()), slice=Name(id='float', ctx=Load()), ctx=Load())), arg(arg='desire_time', annotation=Name(id='float', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[Constant(value=3.0)])", "Name(id='CoreResult', ctx=Load())", ()), 'send_timed_right_arm_ee_world': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='pose', annotation=Subscript(value=Name(id='List', ctx=Load()), slice=Name(id='float', ctx=Load()), ctx=Load())), arg(arg='desire_time', annotation=Name(id='float', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[Constant(value=3.0)])", "Name(id='CoreResult', ctx=Load())", ()), 'send_timed_left_arm_ee_local': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='pose', annotation=Subscript(value=Name(id='List', ctx=Load()), slice=Name(id='float', ctx=Load()), ctx=Load())), arg(arg='desire_time', annotation=Name(id='float', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[Constant(value=3.0)])", "Name(id='CoreResult', ctx=Load())", ()), 'send_timed_right_arm_ee_local': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='pose', annotation=Subscript(value=Name(id='List', ctx=Load()), slice=Name(id='float', ctx=Load()), ctx=Load())), arg(arg='desire_time', annotation=Name(id='float', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[Constant(value=3.0)])", "Name(id='CoreResult', ctx=Load())", ()), 'set_ruckig_planner_params': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='planner_index', annotation=Name(id='int', ctx=Load())), arg(arg='is_sync', annotation=Name(id='bool', ctx=Load())), arg(arg='velocity_max', annotation=Subscript(value=Name(id='List', ctx=Load()), slice=Name(id='float', ctx=Load()), ctx=Load())), arg(arg='acceleration_max', annotation=Subscript(value=Name(id='List', ctx=Load()), slice=Name(id='float', ctx=Load()), ctx=Load())), arg(arg='jerk_max', annotation=Subscript(value=Name(id='List', ctx=Load()), slice=Name(id='float', ctx=Load()), ctx=Load())), arg(arg='velocity_min', annotation=Subscript(value=Name(id='List', ctx=Load()), slice=Name(id='float', ctx=Load()), ctx=Load())), arg(arg='acceleration_min', annotation=Subscript(value=Name(id='List', ctx=Load()), slice=Name(id='float', ctx=Load()), ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[Constant(value=None), Constant(value=None)])", "Name(id='CoreResult', ctx=Load())", ()), 'set_offline_trajectory': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='trajectories', annotation=Subscript(value=Name(id='List', ctx=Load()), slice=Name(id='dict', ctx=Load()), ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='CoreResult', ctx=Load())", ()), 'enable_offline_trajectory': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='enable', annotation=Name(id='bool', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='CoreResult', ctx=Load())", ()), 'check_ik_accessibility': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='is_left', annotation=Name(id='bool', ctx=Load())), arg(arg='is_local', annotation=Name(id='bool', ctx=Load())), arg(arg='is_whole_body', annotation=Name(id='bool', ctx=Load())), arg(arg='pose_desired', annotation=Subscript(value=Name(id='List', ctx=Load()), slice=Name(id='float', ctx=Load()), ctx=Load())), arg(arg='total_time_desired', annotation=Name(id='float', ctx=Load())), arg(arg='max_attempts', annotation=Name(id='int', ctx=Load())), arg(arg='linear_error_max', annotation=Name(id='float', ctx=Load())), arg(arg='angular_error_max', annotation=Name(id='float', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[Constant(value=1.0), Constant(value=5), Constant(value=0.005), Constant(value=0.05)])", "Name(id='CoreResult', ctx=Load())", ()), 'get_arm_joint_positions': ("arguments(posonlyargs=[], args=[arg(arg='self')], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='CoreResult', ctx=Load())", ()), 'solve_ik': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='request', annotation=Name(id='IkSolveRequest', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='CoreResult', ctx=Load())", ()), 'get_ee_poses': ("arguments(posonlyargs=[], args=[arg(arg='self')], kwonlyargs=[], kw_defaults=[], defaults=[])", "Subscript(value=Name(id='Optional', ctx=Load()), slice=Name(id='list', ctx=Load()), ctx=Load())", ()), 'get_ee_pose_reach_error': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='is_left', annotation=Name(id='bool', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='CoreResult', ctx=Load())", ()), 'set_ee_force': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='side', annotation=Name(id='ArmSide', ctx=Load())), arg(arg='force_kg', annotation=Name(id='tuple', ctx=Load())), arg(arg='torque', annotation=Name(id='tuple', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[Tuple(elts=[Constant(value=0.0), Constant(value=0.0), Constant(value=0.0)], ctx=Load()), Tuple(elts=[Constant(value=0.0), Constant(value=0.0), Constant(value=0.0)], ctx=Load())])", "Name(id='CoreResult', ctx=Load())", ()), 'set_ee_force_both': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='left_force_kg', annotation=Name(id='tuple', ctx=Load())), arg(arg='right_force_kg', annotation=Name(id='tuple', ctx=Load())), arg(arg='left_torque', annotation=Name(id='tuple', ctx=Load())), arg(arg='right_torque', annotation=Name(id='tuple', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[Tuple(elts=[Constant(value=0.0), Constant(value=0.0), Constant(value=0.0)], ctx=Load()), Tuple(elts=[Constant(value=0.0), Constant(value=0.0), Constant(value=0.0)], ctx=Load()), Tuple(elts=[Constant(value=0.0), Constant(value=0.0), Constant(value=0.0)], ctx=Load()), Tuple(elts=[Constant(value=0.0), Constant(value=0.0), Constant(value=0.0)], ctx=Load())])", "Name(id='CoreResult', ctx=Load())", ()), 'clear_ee_force': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='side', annotation=Name(id='ArmSide', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[Constant(value=None)])", "Name(id='CoreResult', ctx=Load())", ()), 'set_external_wrench': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='side', annotation=Name(id='ArmSide', ctx=Load())), arg(arg='force_n', annotation=Name(id='tuple', ctx=Load())), arg(arg='torque', annotation=Name(id='tuple', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[Tuple(elts=[Constant(value=0.0), Constant(value=0.0), Constant(value=0.0)], ctx=Load()), Tuple(elts=[Constant(value=0.0), Constant(value=0.0), Constant(value=0.0)], ctx=Load())])", "Name(id='CoreResult', ctx=Load())", ()), 'clear_external_wrench': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='side', annotation=Name(id='ArmSide', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[Constant(value=None)])", "Name(id='CoreResult', ctx=Load())", ()), 'enable_force_empty_detect': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='enable', annotation=Name(id='bool', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='CoreResult', ctx=Load())", ()), 'set_contact_force_params': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='transition_time', annotation=Name(id='float', ctx=Load())), arg(arg='interpolation_speed', annotation=Name(id='float', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='CoreResult', ctx=Load())", ()), 'control_end_effector': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='side', annotation=Name(id='ArmSide', ctx=Load())), arg(arg='cmd')], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='CoreResult', ctx=Load())", ()), 'set_mpc_mode': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='mode', annotation=Name(id='MPCControlMode', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='CoreResult', ctx=Load())", ()), 'enable_quick_mode': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='enable', annotation=Name(id='bool', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='CoreResult', ctx=Load())", ()), 'control_head': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='yaw', annotation=Name(id='float', ctx=Load())), arg(arg='pitch', annotation=Name(id='float', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='CoreResult', ctx=Load())", ()), 'arm_reset': ("arguments(posonlyargs=[], args=[arg(arg='self')], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='CoreResult', ctx=Load())", ()), 'get_reach_time': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='topic_type', annotation=Name(id='str', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[])", "Subscript(value=Name(id='Optional', ctx=Load()), slice=Name(id='float', ctx=Load()), ctx=Load())", ()), 'get_mpc_observation': ("arguments(posonlyargs=[], args=[arg(arg='self')], kwonlyargs=[], kw_defaults=[], defaults=[])", "Subscript(value=Name(id='Optional', ctx=Load()), slice=Name(id='Dict', ctx=Load()), ctx=Load())", ()), 'get_mpc_control_mode': ("arguments(posonlyargs=[], args=[arg(arg='self')], kwonlyargs=[], kw_defaults=[], defaults=[])", "Subscript(value=Name(id='Optional', ctx=Load()), slice=Name(id='int', ctx=Load()), ctx=Load())", ()), 'get_body_acceleration': ("arguments(posonlyargs=[], args=[arg(arg='self')], kwonlyargs=[], kw_defaults=[], defaults=[])", "Subscript(value=Name(id='Optional', ctx=Load()), slice=Name(id='Dict', ctx=Load()), ctx=Load())", ()), 'get_joint_torque': ("arguments(posonlyargs=[], args=[arg(arg='self')], kwonlyargs=[], kw_defaults=[], defaults=[])", "Subscript(value=Name(id='Optional', ctx=Load()), slice=Name(id='Dict', ctx=Load()), ctx=Load())", ()), 'get_transform': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='target_frame', annotation=Name(id='str', ctx=Load())), arg(arg='source_frame', annotation=Name(id='str', ctx=Load())), arg(arg='timeout', annotation=Name(id='float', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[Constant(value=0.2)])", "Name(id='CoreResult', ctx=Load())", ()), 'check_joint_path': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='start_q14', annotation=Subscript(value=Name(id='List', ctx=Load()), slice=Name(id='float', ctx=Load()), ctx=Load())), arg(arg='goal_q14', annotation=Subscript(value=Name(id='List', ctx=Load()), slice=Name(id='float', ctx=Load()), ctx=Load())), arg(arg='sampling_deg', annotation=Name(id='float', ctx=Load())), arg(arg='timeout', annotation=Name(id='float', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[Constant(value=3.0), Constant(value=5.0)])", "Name(id='CoreResult', ctx=Load())", ())}, 'adapters/hardware_ros2/mixins/head_mixin.py': {'head_frames': ("arguments(posonlyargs=[], args=[arg(arg='self')], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='int', ctx=Load())", ("Name(id='property', ctx=Load())",)), 'wait_head_ready': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='timeout', annotation=Name(id='float', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[Constant(value=10.0)])", "Name(id='Result', ctx=Load())", ()), 'read_head_limits_deg': ("arguments(posonlyargs=[], args=[arg(arg='self')], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='Result', ctx=Load())", ()), 'send_head_target': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='yaw_deg', annotation=Name(id='float', ctx=Load())), arg(arg='pitch_deg', annotation=Name(id='float', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='Result', ctx=Load())", ()), 'wait_head_frame': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='previous_frames', annotation=Name(id='int', ctx=Load())), arg(arg='timeout', annotation=Name(id='float', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[Constant(value=2.0)])", "Name(id='Result', ctx=Load())", ()), 'head_measured_frames': ("arguments(posonlyargs=[], args=[arg(arg='self')], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='int', ctx=Load())", ("Name(id='property', ctx=Load())",)), 'read_head_measured_deg': ("arguments(posonlyargs=[], args=[arg(arg='self')], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='Result', ctx=Load())", ()), 'read_head_current_deg': ("arguments(posonlyargs=[], args=[arg(arg='self')], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='Result', ctx=Load())", ()), 'control_head_sdk': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='yaw', annotation=Name(id='float', ctx=Load())), arg(arg='pitch', annotation=Name(id='float', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='Result', ctx=Load())", ())}, 'adapters/hardware_ros2/mixins/arm_mixin.py': {'wait_arm_ready': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='timeout', annotation=Name(id='float', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[Constant(value=10.0)])", "Name(id='Result', ctx=Load())", ()), 'read_arm_deg': ("arguments(posonlyargs=[], args=[arg(arg='self')], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='Result', ctx=Load())", ()), 'read_torso_deg': ("arguments(posonlyargs=[], args=[arg(arg='self')], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='Result', ctx=Load())", ()), 'read_model_deg': ("arguments(posonlyargs=[], args=[arg(arg='self')], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='Result', ctx=Load())", ()), 'solve_arm_fk': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='arm_positions_deg', annotation=Subscript(value=Name(id='Optional', ctx=Load()), slice=Subscript(value=Name(id='Sequence', ctx=Load()), slice=Name(id='float', ctx=Load()), ctx=Load()), ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[Constant(value=None)])", "Name(id='Result', ctx=Load())", ()), 'solve_arm_ik': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='side', annotation=Name(id='str', ctx=Load())), arg(arg='pose_zyx_deg', annotation=Subscript(value=Name(id='Sequence', ctx=Load()), slice=Name(id='float', ctx=Load()), ctx=Load())), arg(arg='frame', annotation=Name(id='int', ctx=Load()))], kwonlyargs=[], kw_defaults=[], kwarg=arg(arg='kwargs'), defaults=[Constant(value=1)])", "Name(id='Result', ctx=Load())", ()), 'check_arm_reachability': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='side', annotation=Name(id='str', ctx=Load())), arg(arg='pose_zyx_deg', annotation=Subscript(value=Name(id='Sequence', ctx=Load()), slice=Name(id='float', ctx=Load()), ctx=Load())), arg(arg='frame', annotation=Name(id='int', ctx=Load()))], kwonlyargs=[], kw_defaults=[], kwarg=arg(arg='kwargs'), defaults=[Constant(value=1)])", "Name(id='Result', ctx=Load())", ()), 'solve_arm_fk_with_frame': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='arm_positions_deg', annotation=Subscript(value=Name(id='Optional', ctx=Load()), slice=Subscript(value=Name(id='Sequence', ctx=Load()), slice=Name(id='float', ctx=Load()), ctx=Load()), ctx=Load())), arg(arg='base_frame', annotation=Name(id='str', ctx=Load())), arg(arg='end_effector_frame', annotation=Name(id='str', ctx=Load())), arg(arg='hand_side', annotation=Name(id='int', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[Constant(value=None), Constant(value=''), Constant(value=''), Constant(value=0)])", "Name(id='Result', ctx=Load())", ()), 'solve_arm_ik_multi_refer': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='side', annotation=Name(id='str', ctx=Load())), arg(arg='pose_zyx_deg', annotation=Subscript(value=Name(id='Sequence', ctx=Load()), slice=Name(id='float', ctx=Load()), ctx=Load())), arg(arg='frame', annotation=Name(id='int', ctx=Load()))], kwonlyargs=[], kw_defaults=[], kwarg=arg(arg='kwargs'), defaults=[Constant(value=1)])", "Name(id='Result', ctx=Load())", ()), 'send_ee_poses_ik': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='left_zyx_deg', annotation=Subscript(value=Name(id='Sequence', ctx=Load()), slice=Name(id='float', ctx=Load()), ctx=Load())), arg(arg='right_zyx_deg', annotation=Subscript(value=Name(id='Sequence', ctx=Load()), slice=Name(id='float', ctx=Load()), ctx=Load())), arg(arg='frame', annotation=Name(id='int', ctx=Load())), arg(arg='repeats', annotation=Name(id='int', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[Constant(value=0), Constant(value=1)])", "Name(id='Result', ctx=Load())", ()), 'send_ee_poses_advance_ik': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='left_zyx_deg', annotation=Subscript(value=Name(id='Sequence', ctx=Load()), slice=Name(id='float', ctx=Load()), ctx=Load())), arg(arg='right_zyx_deg', annotation=Subscript(value=Name(id='Sequence', ctx=Load()), slice=Name(id='float', ctx=Load()), ctx=Load())), arg(arg='frame', annotation=Name(id='int', ctx=Load())), arg(arg='arm_positions_deg', annotation=Subscript(value=Name(id='Optional', ctx=Load()), slice=Subscript(value=Name(id='Sequence', ctx=Load()), slice=Name(id='float', ctx=Load()), ctx=Load()), ctx=Load())), arg(arg='repeats', annotation=Name(id='int', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[Constant(value=0), Constant(value=None), Constant(value=1)])", "Name(id='Result', ctx=Load())", ()), 'send_arm_joint_timed': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='side', annotation=Name(id='str', ctx=Load())), arg(arg='joints_deg', annotation=Subscript(value=Name(id='Sequence', ctx=Load()), slice=Name(id='float', ctx=Load()), ctx=Load())), arg(arg='desire_time', annotation=Name(id='float', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[Constant(value=2.0)])", "Name(id='Result', ctx=Load())", ()), 'send_arm_ee_timed': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='side', annotation=Name(id='str', ctx=Load())), arg(arg='pose_zyx_deg', annotation=Subscript(value=Name(id='Sequence', ctx=Load()), slice=Name(id='float', ctx=Load()), ctx=Load())), arg(arg='desire_time', annotation=Name(id='float', ctx=Load())), arg(arg='frame', annotation=Name(id='int', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[Constant(value=3.0), Constant(value=1)])", "Name(id='Result', ctx=Load())", ()), 'send_arm_action': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='action_name', annotation=Name(id='str', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='Result', ctx=Load())", ()), 'interrupt_arm_action': ("arguments(posonlyargs=[], args=[arg(arg='self')], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='Result', ctx=Load())", ()), 'send_arm_joint_frame': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='positions_deg', annotation=Subscript(value=Name(id='Sequence', ctx=Load()), slice=Name(id='float', ctx=Load()), ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='Result', ctx=Load())", ()), 'send_arm_joint_traj': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='times', annotation=Subscript(value=Name(id='Sequence', ctx=Load()), slice=Name(id='float', ctx=Load()), ctx=Load())), arg(arg='positions_deg', annotation=Subscript(value=Name(id='Sequence', ctx=Load()), slice=Subscript(value=Name(id='Sequence', ctx=Load()), slice=Name(id='float', ctx=Load()), ctx=Load()), ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='Result', ctx=Load())", ()), 'send_ee_poses': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='left', annotation=Subscript(value=Name(id='Sequence', ctx=Load()), slice=Name(id='float', ctx=Load()), ctx=Load())), arg(arg='right', annotation=Subscript(value=Name(id='Sequence', ctx=Load()), slice=Name(id='float', ctx=Load()), ctx=Load())), arg(arg='frame', annotation=Name(id='int', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[Constant(value=1)])", "Name(id='Result', ctx=Load())", ()), 'send_ee_traj': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='times', annotation=Subscript(value=Name(id='Sequence', ctx=Load()), slice=Name(id='float', ctx=Load()), ctx=Load())), arg(arg='left', annotation=Subscript(value=Name(id='Sequence', ctx=Load()), slice=Subscript(value=Name(id='Sequence', ctx=Load()), slice=Name(id='float', ctx=Load()), ctx=Load()), ctx=Load())), arg(arg='right', annotation=Subscript(value=Name(id='Sequence', ctx=Load()), slice=Subscript(value=Name(id='Sequence', ctx=Load()), slice=Name(id='float', ctx=Load()), ctx=Load()), ctx=Load())), arg(arg='frame', annotation=Name(id='int', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[Constant(value=1)])", "Name(id='Result', ctx=Load())", ()), 'cancel_arm_joint_traj': ("arguments(posonlyargs=[], args=[arg(arg='self')], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='Result', ctx=Load())", ()), 'cancel_ee_traj': ("arguments(posonlyargs=[], args=[arg(arg='self')], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='Result', ctx=Load())", ()), 'get_arm_reach_error': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='side', annotation=Name(id='str', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='Result', ctx=Load())", ()), 'set_async_exec': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='enabled', annotation=Name(id='bool', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='Result', ctx=Load())", ()), 'controller': ("arguments(posonlyargs=[], args=[arg(arg='self')], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='Result', ctx=Load())", ()), 'get_modes': ("arguments(posonlyargs=[], args=[arg(arg='self')], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='Result', ctx=Load())", ()), 'get_arm_control_mode': ("arguments(posonlyargs=[], args=[arg(arg='self')], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='Result', ctx=Load())", ()), 'wait_for_consumer': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='topic', annotation=Name(id='str', ctx=Load())), arg(arg='timeout', annotation=Name(id='float', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[Constant(value=5.0)])", "Name(id='Result', ctx=Load())", ()), 'reach_topic_available': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='side', annotation=Name(id='str', ctx=Load())), arg(arg='kind', annotation=Name(id='str', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='Result', ctx=Load())", ()), 'wait_for_reach_time': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='side', annotation=Name(id='str', ctx=Load())), arg(arg='kind', annotation=Name(id='str', ctx=Load())), arg(arg='previous_frames', annotation=Name(id='int', ctx=Load())), arg(arg='timeout', annotation=Name(id='float', ctx=Load())), arg(arg='fallback_actual_time')], kwonlyargs=[], kw_defaults=[], defaults=[Constant(value=None)])", "Name(id='Result', ctx=Load())", ()), 'set_arm_control_mode': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='mode', annotation=Name(id='int', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='Result', ctx=Load())", ()), 'set_wheel_control_mode': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='mode', annotation=Name(id='int', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='Result', ctx=Load())", ()), 'read_arm_control_mode_topic': ("arguments(posonlyargs=[], args=[arg(arg='self')], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='Result', ctx=Load())", ()), 'prepare_arm_modes': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='quick_source', annotation=Subscript(value=Name(id='Optional', ctx=Load()), slice=Name(id='int', ctx=Load()), ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[Constant(value=None)])", "Name(id='Result', ctx=Load())", ()), 'restore_arm_modes': ("arguments(posonlyargs=[], args=[arg(arg='self')], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='Result', ctx=Load())", ())}, 'adapters/hardware_ros2/mixins/torso_mixin.py': {'read_torso_init_pose_deg': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='timeout', annotation=Name(id='float', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[Constant(value=5.0)])", "Name(id='Result', ctx=Load())", ()), 'send_torso_pose_timed': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='pose_deg', annotation=Subscript(value=Name(id='Sequence', ctx=Load()), slice=Name(id='float', ctx=Load()), ctx=Load())), arg(arg='desire_time', annotation=Name(id='float', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[Constant(value=2.0)])", "Name(id='Result', ctx=Load())", ()), 'send_leg_joints_timed': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='joints_deg', annotation=Subscript(value=Name(id='Sequence', ctx=Load()), slice=Name(id='float', ctx=Load()), ctx=Load())), arg(arg='desire_time', annotation=Name(id='float', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[Constant(value=2.0)])", "Name(id='Result', ctx=Load())", ()), 'send_torso_pose_frame': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='pose_deg', annotation=Subscript(value=Name(id='Sequence', ctx=Load()), slice=Name(id='float', ctx=Load()), ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='Result', ctx=Load())", ()), 'send_leg_joints_frame': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='joints_deg', annotation=Subscript(value=Name(id='Sequence', ctx=Load()), slice=Name(id='float', ctx=Load()), ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='Result', ctx=Load())", ()), 'reset_torso': ("arguments(posonlyargs=[], args=[arg(arg='self')], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='Result', ctx=Load())", ()), 'wait_torso_reach_time': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='kind', annotation=Name(id='str', ctx=Load())), arg(arg='previous_frames', annotation=Name(id='int', ctx=Load())), arg(arg='timeout', annotation=Name(id='float', ctx=Load())), arg(arg='fallback_actual_time', annotation=Subscript(value=Name(id='Optional', ctx=Load()), slice=Name(id='float', ctx=Load()), ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[Constant(value=3.0), Constant(value=None)])", "Name(id='Result', ctx=Load())", ()), 'torso_reach_topic_available': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='kind', annotation=Name(id='str', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='Result', ctx=Load())", ()), 'torso_reach_frames': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='kind', annotation=Name(id='str', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='int', ctx=Load())", ())}, 'adapters/hardware_ros2/mixins/locomotion_mixin.py': {'send_base_velocity': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='vx', annotation=Name(id='float', ctx=Load())), arg(arg='vy', annotation=Name(id='float', ctx=Load())), arg(arg='vyaw_deg', annotation=Name(id='float', ctx=Load())), arg(arg='frame'), arg(arg='duration', annotation=Name(id='float', ctx=Load())), arg(arg='rate_hz', annotation=Name(id='float', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[Constant(value='local'), Constant(value=5.0), Constant(value=50.0)])", "Name(id='Result', ctx=Load())", ()), 'send_base_velocity_tick': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='vx', annotation=Name(id='float', ctx=Load())), arg(arg='vy', annotation=Name(id='float', ctx=Load())), arg(arg='vyaw_rad', annotation=Name(id='float', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='Result', ctx=Load())", ()), 'send_base_pose': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='x', annotation=Name(id='float', ctx=Load())), arg(arg='y', annotation=Name(id='float', ctx=Load())), arg(arg='yaw_deg', annotation=Name(id='float', ctx=Load())), arg(arg='frame')], kwonlyargs=[], kw_defaults=[], defaults=[Constant(value='world')])", "Name(id='Result', ctx=Load())", ()), 'send_base_pose_timed': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='x', annotation=Name(id='float', ctx=Load())), arg(arg='y', annotation=Name(id='float', ctx=Load())), arg(arg='yaw_deg', annotation=Name(id='float', ctx=Load())), arg(arg='frame'), arg(arg='desire_time', annotation=Name(id='float', ctx=Load())), arg(arg='timeout', annotation=Name(id='float', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[Constant(value='world'), Constant(value=3.0), Constant(value=15.0)])", "Name(id='Result', ctx=Load())", ()), 'get_velocity_limits': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='timeout', annotation=Name(id='float', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[Constant(value=5.0)])", "Name(id='Result', ctx=Load())", ()), 'reset_vel_ruckig': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='timeout', annotation=Name(id='float', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[Constant(value=5.0)])", "Name(id='Result', ctx=Load())", ()), 'wait_base_reach_time': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='previous_frames', annotation=Name(id='int', ctx=Load())), arg(arg='timeout', annotation=Name(id='float', ctx=Load())), arg(arg='fallback_actual_time', annotation=Subscript(value=Name(id='Optional', ctx=Load()), slice=Name(id='float', ctx=Load()), ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[Constant(value=3.0), Constant(value=None)])", "Name(id='Result', ctx=Load())", ()), 'base_reach_topic_available': ("arguments(posonlyargs=[], args=[arg(arg='self')], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='bool', ctx=Load())", ()), 'base_reach_frames': ("arguments(posonlyargs=[], args=[arg(arg='self')], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='int', ctx=Load())", ("Name(id='property', ctx=Load())",)), 'base_reach_value': ("arguments(posonlyargs=[], args=[arg(arg='self')], kwonlyargs=[], kw_defaults=[], defaults=[])", "Subscript(value=Name(id='Optional', ctx=Load()), slice=Name(id='float', ctx=Load()), ctx=Load())", ("Name(id='property', ctx=Load())",))}, 'adapters/hardware_ros2/mixins/multibody_mixin.py': {'send_multi_body_timed': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='commands', annotation=Subscript(value=Name(id='Sequence', ctx=Load()), slice=Name(id='dict', ctx=Load()), ctx=Load())), arg(arg='is_sync', annotation=Name(id='bool', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[Constant(value=True)])", "Name(id='Result', ctx=Load())", ()), 'set_multi_body_offline_traj': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='trajectories', annotation=Subscript(value=Name(id='Sequence', ctx=Load()), slice=Name(id='dict', ctx=Load()), ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='Result', ctx=Load())", ()), 'enable_multi_body_offline_traj': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='enabled', annotation=Name(id='bool', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='Result', ctx=Load())", ()), 'set_focus_z': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='enable', annotation=Name(id='bool', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='Result', ctx=Load())", ()), 'set_focus_ee': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='enable', annotation=Name(id='bool', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='Result', ctx=Load())", ())}, 'adapters/hardware_ros2/mixins/skill_compat_mixin.py': {'set_arm_quick_mode': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='quick_mode', annotation=Name(id='int', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='Result', ctx=Load())", ()), 'send_arm_joint_timed': ("arguments(posonlyargs=[], args=[arg(arg='self')], vararg=arg(arg='args'), kwonlyargs=[], kw_defaults=[], kwarg=arg(arg='kwargs'), defaults=[])", "Name(id='Result', ctx=Load())", ()), 'send_arm_joint_traj_sdk': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='joint_traj'), arg(arg='total_time', annotation=Name(id='float', ctx=Load()))], vararg=arg(arg='args'), kwonlyargs=[], kw_defaults=[], kwarg=arg(arg='kwargs'), defaults=[Constant(value=None), Constant(value=3.0)])", "Name(id='Result', ctx=Load())", ()), 'send_two_arm_hand_pose': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='left_target'), arg(arg='right_target'), arg(arg='frame'), arg(arg='use_custom_ik_param', annotation=Name(id='bool', ctx=Load())), arg(arg='ik_param')], kwonlyargs=[], kw_defaults=[], defaults=[Constant(value=1), Constant(value=True), Constant(value=None)])", "Name(id='Result', ctx=Load())", ()), 'send_joint_trajectory': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='joint_names'), arg(arg='times'), arg(arg='joint_angles_list')], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='Result', ctx=Load())", ()), 'send_two_arm_joint_command': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='left_joints'), arg(arg='right_joints'), arg(arg='frame'), arg(arg='use_custom_ik_param', annotation=Name(id='bool', ctx=Load()))], kwonlyargs=[], kw_defaults=[], defaults=[Constant(value=5), Constant(value=False)])", "Name(id='Result', ctx=Load())", ()), 'send_leg_joint_positions': ("arguments(posonlyargs=[], args=[arg(arg='self'), arg(arg='joint_names'), arg(arg='positions')], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='Result', ctx=Load())", ()), 'reset_torso_to_initial': ("arguments(posonlyargs=[], args=[arg(arg='self')], kwonlyargs=[], kw_defaults=[], defaults=[])", "Name(id='Result', ctx=Load())", ())}}

# Explicit additive signatures for read-only IK-result observation only.
PUBLIC_SIGNATURES["adapters/hardware_ros2/mixins/arm_mixin.py"].update({
    "arm_ik_result_frames": (
        "arguments(posonlyargs=[], args=[arg(arg='self')], kwonlyargs=[], kw_defaults=[], defaults=[])",
        "Name(id='int', ctx=Load())", ("Name(id='property', ctx=Load())",)),
    "read_arm_ik_result_deg": (
        "arguments(posonlyargs=[], args=[arg(arg='self')], kwonlyargs=[], kw_defaults=[], defaults=[])",
        "Name(id='Result', ctx=Load())", ()),
})

def signature_regression(self):
    count = 0
    for relative, expected in PUBLIC_SIGNATURES.items():
        tree = ast.parse((ROOT / relative).read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
        got = {n.name: (_canon_dump(n.args), _canon_dump(n.returns) if n.returns else None,
                       tuple(_canon_dump(d) for d in n.decorator_list))
               for n in cls.body if isinstance(n, ast.FunctionDef) and not n.name.startswith("_")}
        self.assertEqual(got, expected, relative)
        count += len(expected)
    self.assertEqual(count, 134)
BodyContractTests.test_original_public_signatures_defaults_and_properties = signature_regression


def invalid_input_regression(self):
    cases = [
        ("arm", "send_arm_joint_positions", (["bad"] * 14,), {}),
        ("arm", "send_timed_joint_target", ("left", [None] * 7), {}),
        ("arm", "solve_ik", ("left", [0] * 5), {}),
        ("arm", "solve_ik", ("left", POSE6), {"other_pose_zyx_deg": [0] * 5}),
        ("arm", "check_accessibility", ("left", [0] * 5), {}),
        ("torso", "send_torso_pose_frame", ([0] * 3,), {}),
        ("torso", "send_torso_pose_timed", ([None] * 4,), {}),
        ("locomotion", "send_base_pose", (0, 0, 0), {"frame": "invalid"}),
        ("locomotion", "send_base_pose_timed", (0, 0, "bad"), {}),
        ("multibody", "send_timed_target", ([{}],), {}),
        ("multibody", "send_timed_target", ([{"planner_index": 3, "desire_time": 2,
                                              "cmd_vec": [math.inf] * 4}],), {}),
        ("multibody", "set_offline_traj", ([{"planner_index": 4, "points": []}],), {}),
    ]
    for body, name, args, kwargs in cases:
        with self.subTest(body=body, method=name): self.contract(body, name, args, kwargs)
BodyContractTests.test_invalid_inputs_preserve_fixed_results = invalid_input_regression


def core_failure_regression(self):
    h = body_host("arm", body_fake(False))
    h._torso_client, h._multibody_client, h._locomotion_client = body_fake(False), body_fake(False), body_fake(False)
    for method, args in [("send_base_pose", (0, 0, 0)), ("send_timed_base_pose", (0, 0, 0, 2)),
                         ("set_focus_z", (True,)), ("set_focus_ee", (True,)),
                         ("send_timed_torso_pose", (0, 0, 0, 0, 2)),
                         ("send_timed_leg_joint", ([0]*4, 2)),
                         ("get_torso_initial_pose", ()), ("get_arm_joint_positions", ()),
                         ("send_arm_joint_trajectory", ([0]*14, 2)),
                         ("send_timed_left_arm_joint", ([0]*7, 2)),
                         ("enable_offline_trajectory", (True,))]:
        with self.subTest(method=method):
            result = getattr(h, method)(*args)
            self.assertIs(type(result), compat.CoreResult)
            self.assertFalse(result.success)
    h._arm_solve_fk = Mock(return_value=head.Result.fail("fake", error_code="SERVICE_ERROR", data={"detail": 1}))
    from core.domain.enums import ArmSide
    from core.domain.pose import Pose6D
    r = h.send_ee_pose(ArmSide.LEFT, Pose6D(0,0,0,0,0,0))
    self.assertEqual(vars(r), vars(h._arm_solve_fk.return_value))
BodyContractTests.test_core_facade_failure_result_preservation = core_failure_regression


def public_package_import_regression(self):
    stubs = {"rclpy": ros_stub}
    node_stub = types.ModuleType("rclpy.node")
    node_stub.Node = type("Node", (), {"__init__": forbidden})
    stubs["rclpy.node"] = node_stub
    for body, title in (("arm", "Arm"), ("head", "Head"), ("torso", "Torso"),
                        ("locomotion", "Locomotion"), ("multibody", "Multibody"),
                        ("controller", "Controller"), ("jibot_chassis", "JibotChassis")):
        stub = types.ModuleType(body + "_client")
        setattr(stub, title + "Client", type(title + "Client", (), {"__init__": forbidden}))
        if body == "locomotion": stub.FRAME_WORLD, stub.FRAME_LOCAL = 0, 1
        stubs[stub.__name__] = stub
    # hardware.py 是以 offline_public_ros2.hardware 名字加载的，其相对导入
    # （.legacy_chassis_mixin / .state_feedback_rt_mixin）需要同名子模块可解析。
    pkg = sys.modules.get("offline_public_ros2")
    for mixin_module, mixin_name in (("legacy_chassis_mixin", "LegacyChassisMixin"),
                                     ("state_feedback_rt_mixin", "StateFeedbackRtMixin")):
        full = "offline_public_ros2." + mixin_module
        stub = types.ModuleType(full)
        stub.__path__ = [str(ROOT / "adapters" / "hardware_ros2")]
        setattr(stub, mixin_name, type(mixin_name, (), {}))
        sys.modules[full] = stub
        if pkg is not None and hasattr(pkg, "__path__"):
            pass
    # Real public package/runtime/controller client imports, all constructors guarded.
    with patch.dict(sys.modules, stubs):
        public = load("offline_public_ros2", "adapters/hardware_ros2/__init__.py")
    self.assertEqual([name for name in vars(public) if name.endswith("Hardware")],
                     ["Ros2WheeledHardware"])
    self.assertFalse(public.Ros2WheeledHardware.__abstractmethods__)
    self.assertEqual(set(public.__all__), {"Ros2Runtime", "Ros2WheeledHardware"})
BodyContractTests.test_public_package_import_with_forbidden_ros_constructors = public_package_import_regression


def shutdown_context_regression(self):
    h = object.__new__(hardware.Ros2WheeledHardware)
    h.close = Mock()
    self.assertIs(h.__enter__(), h)
    h.__exit__(None, None, None)
    h.close.assert_called_once_with()
    h.close.reset_mock()
    result = h.shutdown()
    self.assertIs(type(result), compat.CoreResult)
    self.assertTrue(result.success)
    h.close.assert_called_once_with()
    h.close.side_effect = RuntimeError("cleanup")
    with self.assertRaisesRegex(RuntimeError, "cleanup"): h.shutdown()
BodyContractTests.test_shutdown_and_context_manager_preserve_exceptions = shutdown_context_regression



class CanonicalResultTests(unittest.TestCase):
    def test_exact_result_factory_contract(self):
        from core.domain.result import Result
        import inspect
        self.assertEqual([p.default for p in inspect.signature(Result.ok).parameters.values()], ["Success", None])
        self.assertEqual(list(inspect.signature(Result.fail).parameters), ["msg", "error_code", "data"])
        data = {"detail": [1, 2]}
        for got, expected in [
            (Result(True), (True, "", None, None)),
            (Result.ok(), (True, "Success", None, None)),
            (Result.fail(), (False, "Failed", None, None)),
            (Result.ok("done", data), (True, "done", data, None)),
            (Result.fail("bad", "REMOTE", data), (False, "bad", data, "REMOTE")),
            (Result.fail("bad", data=data, error_code="ARG"), (False, "bad", data, "ARG")),
        ]:
            self.assertIs(type(got), Result)
            self.assertEqual((got.success, got.message, got.data, got.error_code), expected)
        self.assertIs(Result.ok(data=data).data, data)
        self.assertIs(Result.fail(data=data).data, data)

    def test_all_mixins_import_single_core_result_and_no_adapter_search_path(self):
        from core.domain.result import Result
        for cls in [hardware.ArmMixin, hardware.HeadMixin, hardware.TorsoMixin,
                    hardware.MultibodyMixin, hardware.LocomotionMixin,
                    hardware.ControllerMixin, hardware.SkillCompatMixin]:
            method = next(v for v in cls.__dict__.values() if callable(v) and hasattr(v, "__globals__"))
            self.assertIs(method.__globals__["Result"], Result)
        exports = ast.parse((ROOT / "adapters/hardware_ros2/__init__.py").read_text())
        self.assertNotIn("_SDK_DIR", _unparse(exports))
        # 相机/感知适配器是感知链路契约保留的文件（同名同接口对齐 ROS 1），
        # 除此之外仍禁止任何 *_adapter 相对导入。
        allowed_adapter_imports = {"camera_adapter", "perception_adapter"}
        for path in (ROOT / "adapters/hardware_ros2").rglob("*.py"):
            for node in ast.walk(ast.parse(path.read_text())):
                if isinstance(node, ast.ImportFrom):
                    self.assertNotEqual(node.module, "result")
                    if (node.module or "") in allowed_adapter_imports:
                        continue
                    self.assertFalse((node.module or "").endswith("_adapter"))
                if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) and isinstance(node.func.value, ast.Name):
                    if node.func.value.id in ("Result", "CoreResult") and node.func.attr == "fail":
                        self.assertLessEqual(len(node.args), 1, str(path))


class ControllerContractTests(unittest.TestCase):
    def host(self):
        host = object.__new__(hardware.Ros2WheeledHardware)
        host._controller_client = Mock()
        host._controller_closed = False
        return host

    def assert_result(self, result, success, message, data=None, code=None):
        self.assertIs(type(result), compat.CoreResult)
        self.assertEqual(vars(result), {"success": success, "message": message,
                                       "data": data, "error_code": code})

    def test_queries_exact_data_and_timeout_contracts(self):
        for public, driver, message, data in [
            ("get_controller_status", "get_status", "controller status", {"current": "wheel_mpc", "switching": False, "pending": ""}),
            ("list_controllers", "list_controllers", "controller list", [{"name": "wheel_mpc", "type": "MPC"}]),
        ]:
            for success in (True, False):
                h = self.host()
                getattr(h._controller_client, driver).return_value = types.SimpleNamespace(success=success, message="reply", data=data)
                result = getattr(h, public)(0.75)
                self.assert_result(result, success, message if success else "reply", data if success else None)
                if success: self.assertIs(result.data, data)
                getattr(h._controller_client, driver).assert_called_once_with(0.75)

    def test_explicit_service_results_fallbacks_and_defaults(self):
        for name, args, defaults, fallback in [
            ("switch_controller", ("wheel_mpc",), ("wheel_mpc", 10.0), "controller switched to wheel_mpc"),
            ("load_controller", ("wheel_mpc",), ("wheel_mpc", "", 30.0), "controller load requested: wheel_mpc"),
            ("confirm_startup", (), (5.0,), "startup confirmed"),
        ]:
            for success in (True, False):
                for message in ("", "remote_ex=2: rejected"):
                    h = self.host()
                    getattr(h._controller_client, name).return_value = types.SimpleNamespace(success=success, message=message, data={"discard": True})
                    self.assert_result(getattr(h, name)(*args), success, (message or fallback) if success else message)
                    getattr(h._controller_client, name).assert_called_once_with(*defaults)
                    self.assertEqual(len(h._controller_client.mock_calls), 1)
        h = self.host()
        h._controller_client.load_controller.return_value = types.SimpleNamespace(success=True, message="loaded")
        self.assert_result(h.load_controller("custom", config_path="fake.yaml", timeout=0.2), True, "loaded")
        h._controller_client.load_controller.assert_called_once_with("custom", "fake.yaml", 0.2)

    def test_events_missing_empty_and_updated_frames(self):
        for name in ("event_switch", "event_ready", "startup_state"):
            for value, frames in [(None, 0), ("", 1), ("ready", 7)]:
                h = self.host()
                getattr(h._controller_client, name).return_value = value
                getattr(h._controller_client, name + "_frames").return_value = frames
                self.assert_result(getattr(h, name)(), value is not None,
                                   name + " state" if value is not None else name + " state has no frame yet",
                                   {"value": value, "frames": frames} if value is not None else {"frames": frames})
                self.assertEqual([repr(c) for c in h._controller_client.mock_calls],
                                 ["call." + name + "()", "call." + name + "_frames()"])

    def test_wait_reply_and_invalid_kind_contracts(self):
        for kind in ("switch", "ready", "startup"):
            for success in (True, False):
                h = self.host()
                data = {"value": "active", "frames": 9}
                h._controller_client.wait_for_state.return_value = types.SimpleNamespace(success=success, message="no fresh frame", data=data)
                self.assert_result(h.wait_for_controller_state(kind, 7, 0.25), success,
                                   kind + " state" if success else "no fresh frame", data if success else None)
                h._controller_client.wait_for_state.assert_called_once_with(kind, 7, 0.25)
        h = self.host()
        h._controller_client.wait_for_state.side_effect = ValueError("kind must be switch/ready/startup")
        self.assert_result(h.wait_for_controller_state("bad", 0), False,
                           "kind must be switch/ready/startup", code="INVALID_ARGUMENT")
        h._controller_client.wait_for_state.assert_called_once_with("bad", 0, 5.0)

    def test_driver_exceptions_propagate_except_invalid_wait(self):
        for public, driver, args in [
            ("get_controller_status", "get_status", ()), ("list_controllers", "list_controllers", ()),
            ("switch_controller", "switch_controller", ("fake",)), ("load_controller", "load_controller", ("fake",)),
            ("confirm_startup", "confirm_startup", ()), ("event_switch", "event_switch", ()),
            ("event_ready", "event_ready", ()), ("startup_state", "startup_state", ()),
            ("wait_for_controller_state", "wait_for_state", ("switch", 0)),
        ]:
            h = self.host()
            getattr(h._controller_client, driver).side_effect = RuntimeError("driver exception")
            with self.subTest(public=public), self.assertRaisesRegex(RuntimeError, "driver exception"):
                getattr(h, public)(*args)

    def test_controller_close_idempotence_and_failure_flag(self):
        for error in (None, RuntimeError("close")):
            h = self.host()
            h._controller_client.close.side_effect = error
            if error:
                with self.assertRaisesRegex(RuntimeError, "close"): h._controller_close()
            else: h._controller_close()
            self.assertTrue(h._controller_closed)
            h._controller_close()
            h._controller_client.close.assert_called_once_with()

    def test_defaults_and_return_annotations(self):
        import inspect
        expected = {"get_controller_status": [5.0], "list_controllers": [5.0],
                    "switch_controller": [10.0], "load_controller": ["", 30.0],
                    "confirm_startup": [5.0], "event_switch": [], "event_ready": [],
                    "startup_state": [], "wait_for_controller_state": [5.0]}
        for name, defaults in expected.items():
            sig = inspect.signature(getattr(hardware.ControllerMixin, name))
            self.assertEqual([p.default for p in sig.parameters.values() if p.default is not p.empty], defaults)
            self.assertEqual(sig.return_annotation, "Result")


# Import-only AST execution of the existing driver. Remove imports, not methods;
# every ROS constructor/action is forbidden unless explicitly replaced by a local fake.
def controller_driver_offline():
    import threading
    import time
    from dataclasses import dataclass
    from typing import Dict, Optional, Tuple
    tree = ast.parse((ROOT / "drivers/ros2_new_sdk/controller_client.py").read_text())
    tree.body = [n for n in tree.body if not isinstance(n, (ast.Import, ast.ImportFrom))
                 and not (isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id == "LATCHED_QOS" for t in n.targets))]
    module = types.ModuleType("offline_controller_driver_contract")
    sys.modules[module.__name__] = module
    module.__dict__.update(threading=threading, time=time, dataclass=dataclass, Dict=Dict,
                           Optional=Optional, Tuple=Tuple, rclpy=ros_stub, Node=forbidden,
                           LATCHED_QOS=object(), StringStamped=object)
    for name in ("ControlStatus", "ControlGetList", "ControlSwitch", "ControlLoad", "Trigger"):
        module.__dict__[name] = types.SimpleNamespace(Request=types.SimpleNamespace)
    tree.body.insert(0, ast.ImportFrom(module="__future__", names=[ast.alias(name="annotations")], level=0))
    exec(compile(ast.fix_missing_locations(tree), "offline-controller-contract", "exec"), module.__dict__)
    return module


class ControllerDriverOfflineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls): cls.driver = controller_driver_offline()

    def client(self): return object.__new__(self.driver.ControllerClient)

    def test_reply_remote_exception_missing_and_failure_fields(self):
        for response, expected in [
            (None, (False, "service unavailable or timed out")),
            (types.SimpleNamespace(success=True, message="ok"), (True, "ok")),
            (types.SimpleNamespace(success=False, message="rejected"), (False, "rejected")),
            (types.SimpleNamespace(success=True, message="rejected", remote_ex=2), (False, "remote_ex=2: rejected")),
        ]:
            reply = self.driver.ControllerClient._reply(response)
            self.assertEqual((reply.success, reply.message, reply.data), (*expected, None))

    def test_requests_and_query_shapes_no_node(self):
        c = self.client()
        for field in ("status", "list", "switch", "load", "confirm"): setattr(c, "_" + field + "_cli", object())
        c._call = Mock(return_value=types.SimpleNamespace(success=True, message="ok"))
        c.switch_controller(42, 0.2)
        args = c._call.call_args.args
        self.assertIs(args[0], c._switch_cli)
        self.assertEqual((vars(args[1]), args[2]), ({"controller_name": "42"}, 0.2))
        c.load_controller(42, 7, 0.3)
        args = c._call.call_args.args
        self.assertIs(args[0], c._load_cli)
        self.assertEqual((vars(args[1]), args[2]), ({"name": "42", "config_path": "7"}, 0.3))
        c.confirm_startup(0.4)
        self.assertIs(c._call.call_args.args[0], c._confirm_cli)
        self.assertEqual(c._call.call_args.args[2], 0.4)
        c._call.return_value = types.SimpleNamespace(current_controller="wheel_mpc", switching=True, pending_controller="mpc")
        self.assertEqual(c.get_status(0.5).data, {"current": "wheel_mpc", "switching": True, "pending": "mpc"})
        c._call.return_value = None
        self.assertEqual(c.get_status().message, "get_status unavailable or timed out")
        c._call.return_value = types.SimpleNamespace(success=True, message="", controllers=[types.SimpleNamespace(name="wheel_mpc", type="MPC")])
        reply = c.list_controllers(0.6)
        self.assertEqual((reply.success, reply.message, reply.data), (True, "controller list", [{"name": "wheel_mpc", "type": "MPC"}]))
        c._call.return_value = types.SimpleNamespace(success=True, message="bad", remote_ex=1)
        self.assertFalse(c.list_controllers().success)

    def test_state_callback_counters_and_fresh_frame_wait(self):
        import threading
        c = self.client()
        c._lock = threading.Lock()
        c._state = {key: (0, None) for key in self.driver.STRING_TOPICS}
        for kind, getter in [("switch", "event_switch"), ("ready", "event_ready"), ("startup", "startup_state")]:
            self.assertIsNone(getattr(c, getter)())
            c._on_string(kind)(types.SimpleNamespace(data="active"))
            c._on_string(kind)(types.SimpleNamespace(data=""))
            self.assertEqual(getattr(c, getter)(), "")
            self.assertEqual(getattr(c, getter + "_frames")(), 2)
            c.spin_once = Mock()
            with patch.object(self.driver.time, "monotonic", side_effect=[0, 0]):
                reply = c.wait_for_state(kind, 1, 0.2)
            self.assertEqual((reply.success, reply.data), (True, {"value": "", "frames": 2}))
            c.spin_once.assert_not_called()
            with patch.object(self.driver.time, "monotonic", side_effect=[0, 0, 1]):
                reply = c.wait_for_state(kind, 2, 0.2)
            self.assertFalse(reply.success)
            c.spin_once.assert_called_once_with(0.02)
        with self.assertRaisesRegex(ValueError, "kind must be"): c.wait_for_state("invalid", 0)

    def test_service_wait_future_cancellation_and_exception(self):
        c = self.client()
        c.node = object()
        request = object()
        service = Mock()
        service.wait_for_service.return_value = False
        self.assertIsNone(c._call(service, request, 0.2))
        service.call_async.assert_not_called()
        service.wait_for_service.assert_called_once_with(timeout_sec=0.2)
        future = Mock()
        service.wait_for_service.return_value = True
        service.call_async.return_value = future
        with patch.object(self.driver.rclpy, "spin_until_future_complete") as spin:
            future.done.return_value = False
            self.assertIsNone(c._call(service, request, 0.3))
            future.cancel.assert_called_once_with()
            spin.assert_called_once_with(c.node, future, timeout_sec=0.3)
            future.done.return_value = True
            future.result.side_effect = RuntimeError("fake failure")
            self.assertIsNone(c._call(service, request, 0.4))
            future.result.side_effect = None
            future.result.return_value = request
            self.assertIs(c._call(service, request, 0.5), request)

    def test_shared_node_ownership_close_and_owned_cleanup_finally(self):
        for owns_node, owns_context, failure in [(False, False, False), (True, False, False), (True, True, False), (True, True, True)]:
            c = self.client()
            c._owns_node, c._owns_context = owns_node, owns_context
            c.node = Mock()
            if failure: c.node.destroy_node.side_effect = RuntimeError("destroy")
            with patch.object(self.driver.rclpy, "shutdown") as shutdown:
                if failure:
                    with self.assertRaisesRegex(RuntimeError, "destroy"): c.close()
                else: c.close()
                self.assertEqual(c.node.destroy_node.call_count, int(owns_node))
                self.assertEqual(shutdown.call_count, int(owns_node and owns_context))

    def test_real_ros_and_client_constructors_remain_forbidden(self):
        with self.assertRaises(AssertionError): self.driver.ControllerClient()
        with self.assertRaises(AssertionError): hardware.Ros2WheeledHardware()
        for name in ("init", "ok", "spin_once", "spin_until_future_complete", "shutdown"):
            with self.assertRaises(AssertionError): getattr(ros_stub, name)()

# Execute driver definitions only, with every ROS action forbidden. No constructor
# or live Node is used when exercising the new read-only IK-result cache.
def arm_driver_offline():
    import threading
    import time
    from dataclasses import dataclass
    from typing import Dict, List, Optional, Sequence, Tuple
    tree = ast.parse((ROOT / "drivers/ros2_new_sdk/arm_client.py").read_text())
    qos_names = {"LATEST_QOS", "BEST_EFFORT_QOS", "IK_PUB_QOS", "IK_RESULT_QOS"}
    tree.body = [n for n in tree.body if not isinstance(n, (ast.Import, ast.ImportFrom))
                 and not (isinstance(n, ast.Assign) and any(
                     isinstance(t, ast.Name) and t.id in qos_names for t in n.targets))]
    module = types.ModuleType("offline_arm_driver_observation")
    sys.modules[module.__name__] = module
    module.__dict__.update(threading=threading, time=time, math=math, dataclass=dataclass,
                           Dict=Dict, List=List, Optional=Optional, Sequence=Sequence,
                           Tuple=Tuple, rclpy=ros_stub, Node=forbidden)
    for name in qos_names:
        module.__dict__[name] = object()
    for name in ("Float32", "CmdArmJoints", "CmdArmJointsTraj", "CmdEePoses",
                 "CmdEePosesIK", "CmdEePosesTraj", "Float64Array", "LowState",
                 "AccessIkSolve", "ChangeArmCtrlMode", "ChangeLbQuickMode",
                 "ChangeTorsoCtrlMode", "ControlStatus", "EePoseReachError",
                 "ExecuteArmAction", "SetArmQuickMode", "SetBool", "SolveEePoses",
                 "SolveFk", "SolveFkReferFrame", "Trigger", "WheelSetTimedTarget"):
        module.__dict__[name] = forbidden
    tree.body.insert(0, ast.ImportFrom(module="__future__",
                                     names=[ast.alias(name="annotations")], level=0))
    exec(compile(ast.fix_missing_locations(tree), "offline-arm-observation", "exec"),
         module.__dict__)
    return module


class ArmIkObservationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.driver = arm_driver_offline()

    def client(self):
        import threading
        c = object.__new__(self.driver.ArmClient)
        c._lock = threading.Lock()
        c._ik_result_positions = None
        c._ik_result_frames = 0
        return c

    def test_locked_result_cache_defensive_copies_and_frame_counts(self):
        import threading
        class CountingLock:
            def __init__(self):
                self.lock, self.entries = threading.Lock(), 0
            def __enter__(self):
                self.lock.acquire()
                self.entries += 1
            def __exit__(self, *args):
                self.lock.release()
        c = self.client()
        c._lock = CountingLock()
        self.assertIsNone(c.read_ik_result())
        self.assertEqual((c.ik_result_frames, c._lock.entries), (0, 2))
        left, right = [0.1] * 7, [-0.2] * 7
        msg = types.SimpleNamespace(
            left_joints=types.SimpleNamespace(joint_angles=left),
            right_joints=types.SimpleNamespace(joint_angles=right))
        c._on_ik_result(msg)
        self.assertEqual(c._lock.entries, 3)
        self.assertEqual(c.ik_result_frames, 1)
        self.assertEqual(c._lock.entries, 4)
        got = c.read_ik_result()
        self.assertEqual((got, c._lock.entries), ([0.1] * 7 + [-0.2] * 7, 5))
        got[0], left[0], right[0] = 99.0, 0.3, -0.4
        self.assertEqual(c.read_ik_result(), [0.1] * 7 + [-0.2] * 7)
        c._on_ik_result(msg)
        self.assertEqual(c.ik_result_frames, 2)
        self.assertEqual(c.read_ik_result(), [0.3] + [0.1] * 6 + [-0.4] + [-0.2] * 6)

    def test_existing_endpoint_qos_and_readonly_subscription_wiring(self):
        tree = ast.parse((ROOT / "drivers/ros2_new_sdk/arm_client.py").read_text())
        topic = next(n.value for n in tree.body if isinstance(n, ast.Assign)
                     and _unparse(n.targets[0]) == "IK_RESULT_TOPIC")
        self.assertEqual(ast.literal_eval(topic), "/planning/kinematics/ik_result_arm_joints")
        qos = next(n.value for n in tree.body if isinstance(n, ast.Assign)
                   and _unparse(n.targets[0]) == "IK_RESULT_QOS")
        self.assertEqual(_unparse(qos.func), "QoSProfile")
        self.assertEqual({kw.arg: _unparse(kw.value) for kw in qos.keywords}, {
            "history": "HistoryPolicy.KEEP_LAST", "depth": "10",
            "reliability": "ReliabilityPolicy.RELIABLE",
            "durability": "DurabilityPolicy.VOLATILE"})
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "ArmClient")
        init = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "__init__")
        sub = next(n.value for n in init.body if isinstance(n, ast.Assign)
                   and _unparse(n.targets[0]) == "self._ik_result_sub")
        self.assertEqual(_unparse(sub),
                         "self.node.create_subscription(CmdArmJoints, IK_RESULT_TOPIC, self._on_ik_result, IK_RESULT_QOS)")
        with self.assertRaises(AssertionError):
            self.driver.ArmClient()  # forbidden context guard, never a real constructor

    def test_mixin_readonly_forwarding_units_absence_and_exceptions(self):
        c = self.client()
        hw = body_host("arm", c)
        self.assertEqual(hw.arm_ik_result_frames, 0)
        missing = hw.read_arm_ik_result_deg()
        self.assertEqual((type(missing), missing.success, missing.message, missing.data),
                         (head.Result, False, "no IK result arm joints", None))
        c._ik_result_positions = [math.pi / 2] * 7 + [-math.pi / 4] * 7
        c._ik_result_frames = 5
        self.assertEqual(hw.arm_ik_result_frames, 5)
        result = hw.read_arm_ik_result_deg()
        self.assertEqual((type(result), result.success, result.message, result.data),
                         (head.Result, True, "IK result arm joints (deg)", [90.0] * 7 + [-45.0] * 7))
        result.data[0] = 999.0
        self.assertEqual(hw.read_arm_ik_result_deg().data[0], 90.0)
        self.assertEqual(c.ik_result_frames, 5)
        c.read_ik_result = Mock(side_effect=RuntimeError("fake observation failed"))
        with self.assertRaisesRegex(RuntimeError, "fake observation failed"):
            hw.read_arm_ik_result_deg()


def mixin_app_offline(filename, hw=None):
    """Load an app with guarded fake Hardware and in-memory reporting only."""
    records = []
    common = types.ModuleType("mixin_test_common")
    class Context:
        def __enter__(self): return hw
        def __exit__(self, *args): return False
    common.Ros2WheeledHardware = (forbidden if hw is None
                                  else Mock(return_value=Context()))
    common.record = lambda name, status, detail="": records.append((name, status, detail))
    def show(name, result, detail=""):
        common.record(name, "PASS" if result.success else "FAIL", result.message or detail)
        return bool(result.success)
    common.show = show
    common.skip = lambda name, detail: common.record(name, "SKIP", detail)
    common.report = lambda *args: int(any(status == "FAIL" for _, status, _ in records))
    common.set_log_dir, common.start_segment = Mock(), Mock()
    common.dist = lambda a, b: math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))
    common.quat_to_zyx_deg = lambda quat: [0.0, 0.0, 0.0]  # test input is identity
    with patch.dict(sys.modules, {"mixin_test_common": common, "rclpy": ros_stub}):
        app = load("offline_app_" + Path(filename).stem, "apps/ros2_new_sdk/" + filename)
    return app, records, common


class Ros2MixinAppOfflineTests(unittest.TestCase):
    def test_canonical_inventory_and_apps_use_only_hardware_mixin_methods(self):
        folder = ROOT / "apps/ros2_new_sdk"
        expected = {"mixin_test_common.py", "test_mixin_head.py"}
        expected.update("test_mixin_" + name + ".py" for name in
                        ("readonly", "joint", "ee", "misc", "torso", "multibody",
                         "arm_ik", "base", "chassis", "chassis_readonly",
                         "chassis_cmdvel", "camera"))
        self.assertEqual({p.name for p in folder.glob("*.py")}, expected)
        for path in sorted(folder.glob("*.py")):
            with self.subTest(file=path.name):
                tree = ast.parse(path.read_text())
                for n in ast.walk(tree):
                    if isinstance(n, (ast.Import, ast.ImportFrom)):
                        modules = ([n.module] if isinstance(n, ast.ImportFrom)
                                   else [alias.name for alias in n.names])
                        self.assertFalse(any(module and module.split(".")[0] in {"rclpy", "leju"}
                                             for module in modules))
                        self.assertFalse(any(alias.name.endswith("Client") for alias in n.names))
                    if isinstance(n, ast.Attribute) and isinstance(n.value, ast.Name) and n.value.id == "hw":
                        self.assertFalse(n.attr.startswith("_"))
                        self.assertNotEqual(n.attr, "node")
                    if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute):
                        self.assertNotIn(n.func.attr, {"publish", "call_async", "create_publisher",
                                                     "create_subscription", "create_client", "destroy_subscription"})

    def test_head_cli_motion_plan_and_execute_gate_preserve_full_coverage(self):
        tree = ast.parse((ROOT / "apps/ros2_new_sdk/test_mixin_head.py").read_text())
        main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "main")
        flags = {ast.literal_eval(n.args[0]): {kw.arg: _unparse(kw.value) for kw in n.keywords}
                 for n in ast.walk(main) if isinstance(n, ast.Call)
                 and isinstance(n.func, ast.Attribute) and n.func.attr == "add_argument"}
        self.assertEqual(set(flags), {"--execute", "--yaw-deg", "--pitch-deg", "--settle-sec", "--log-dir"})
        self.assertEqual(flags["--execute"]["action"], "'store_true'")
        self.assertEqual({name: flags[name]["default"] for name in
                          ("--yaw-deg", "--pitch-deg", "--settle-sec")},
                         {"--yaw-deg": "12.0", "--pitch-deg": "8.0", "--settle-sec": "3.0"})
        motions = [_unparse(n) for n in ast.walk(main) if isinstance(n, ast.Call)
                   and isinstance(n.func, ast.Name) and n.func.id == "move"]
        self.assertEqual(motions, [
            "move(hw, 'control_head(+yaw,+pitch)', yaw, pitch, args.settle_sec, via_compat=True)",
            "move(hw, 'send_head_target(-yaw,-pitch)', -yaw, -pitch, args.settle_sec)",
            "move(hw, 'send_head_target(0,0)', 0.0, 0.0, args.settle_sec)"])
        comparisons = [_unparse(n) for n in ast.walk(main) if isinstance(n, ast.Compare)]
        self.assertIn("hw.head_frames - frames0 >= 3", comparisons)
        self.assertIn("hw.head_measured_frames - measured0 > 0", comparisons)
        hw = Mock(spec=hardware.Ros2WheeledHardware, head_frames=7, head_measured_frames=11)
        for name in ("wait_head_ready", "read_head_current_deg", "read_head_measured_deg"):
            getattr(hw, name).return_value = head.Result.ok()
        hw.read_head_limits_deg.return_value = head.Result.ok(data=([-90, -45], [90, 45]))
        app, records, _ = mixin_app_offline("test_mixin_head.py", hw)
        with patch.object(sys, "argv", ["test_mixin_head.py"]):
            self.assertEqual(app.main(), 0)
        hw.control_head.assert_not_called()
        hw.send_head_target.assert_not_called()
        self.assertEqual({name for name, status, _ in records if status == "SKIP"},
                         {"control_head", "send_head_target(-)", "send_head_target(0)",
                          "frames_increment", "measured_frames_increment"})

    def test_head_readback_and_measured_boundaries_keep_original_tolerances(self):
        app, records, _ = mixin_app_offline("test_mixin_head.py")
        for via_compat in (False, True):
            for readback_error, measured_error, target_status, track_status in [
                    (0.0, 0.0, "PASS", "PASS"), (0.5, 0.0, "FAIL", "PASS"),
                    (0.0, 2.0, "PASS", "FAIL"), (0.499, 1.999, "PASS", "PASS")]:
                with self.subTest(compat=via_compat, readback=readback_error, measured=measured_error):
                    records.clear()
                    hw = Mock(spec=hardware.Ros2WheeledHardware, head_frames=7)
                    hw.control_head.return_value = hw.send_head_target.return_value = head.Result.ok()
                    hw.wait_head_frame.return_value = head.Result.ok(data=[12 + readback_error, -8])
                    hw.read_head_measured_deg.return_value = head.Result.ok(data=[12 + measured_error, -8])
                    app.move(hw, "head", 12.0, 8.0, 3.0, via_compat=via_compat)
                    sender, other = (hw.control_head, hw.send_head_target) if via_compat else (hw.send_head_target, hw.control_head)
                    sender.assert_called_once_with(12.0, 8.0)
                    other.assert_not_called()
                    hw.wait_head_frame.assert_called_once_with(7, 3.0)
                    hw.spin_for.assert_called_once_with(3.0)
                    statuses = {name: status for name, status, _ in records}
                    self.assertEqual(statuses["head.target_readback"], target_status)
                    self.assertEqual(statuses["head.track"], track_status)

    def test_s7_requires_success_and_fresh_frames_with_guarded_fake_hardware(self):
        from core.domain.ik import IkSolution
        for execute, success, fresh in [(False, True, True), (True, True, True),
                                         (True, True, False), (True, False, True)]:
            with self.subTest(execute=execute, success=success, fresh=fresh):
                hw = Mock(spec=hardware.Ros2WheeledHardware, arm_ik_result_frames=7)
                hw.wait_arm_ready.return_value = head.Result.ok()
                poses = {side: {"pos": [0.1, 0.2, 0.3], "quat": [0.0, 0.0, 0.0, 1.0]}
                         for side in ("left", "right")}
                hw.solve_arm_fk.return_value = hw.solve_arm_fk_with_frame.return_value = head.Result.ok(data=poses)
                hw.read_arm_deg.return_value = head.Result.ok(data=[0.0] * 14)
                for name in ("solve_arm_ik", "solve_arm_ik_multi_refer"):
                    getattr(hw, name).return_value = head.Result.ok(data={"q_arm_deg": [0.0] * 14})
                def sent(*args, **kwargs):
                    if fresh:
                        hw.arm_ik_result_frames += 1
                    return head.Result.ok("fake publication") if success else head.Result.fail("fake rejected")
                hw.send_ee_poses_ik.side_effect = hw.send_ee_poses_advance_ik.side_effect = sent
                hw.solve_ik.return_value = head.Result.ok(data=IkSolution(
                    q_arm=[0.0] * 14, left_hand_pose=[0.1, 0.2, 0.3, 0, 0, 0, 1],
                    right_hand_pose=[0.1, 0.2, 0.3, 0, 0, 0, 1], elapsed_ms=0.5))
                hw.get_joint_torque.return_value = {"arm_torques": [0.1] * 14}
                hw.get_reach_time.return_value = None
                app, records, _ = mixin_app_offline("test_mixin_arm_ik.py", hw)
                argv = ["test_mixin_arm_ik.py"] + (["--execute"] if execute else [])
                with patch.object(sys, "argv", argv):
                    self.assertEqual(app.main(), int(execute and not (success and fresh)))
                hw.arm_reset.assert_not_called()
                if not execute:
                    hw.send_ee_poses_ik.assert_not_called()
                    hw.send_ee_poses_advance_ik.assert_not_called()
                    continue
                expected_target = [0.1, 0.2, 0.32, 0.0, 0.0, 0.0]
                for method in (hw.send_ee_poses_ik, hw.send_ee_poses_advance_ik):
                    method.assert_called_once_with(expected_target, expected_target, repeats=5)
                statuses = {name: status for name, status, _ in records}
                for name in ("cmd_ee_poses_ik", "cmd_ee_poses_advance_ik"):
                    self.assertEqual(statuses[name], "PASS" if success and fresh else "FAIL")
                hw.spin_for.assert_any_call(0.1)


class JibotChassisMixinTests(unittest.TestCase):
    """base_move/check_arrived/控制权/流式速度/motion gate 的离线契约。"""

    def make(self, handover=True, flag=True, base_ok=True):
        client = Mock()
        client.read_vel_control_flag.return_value = flag
        client.wait_vel_control_flag.return_value = flag
        client.set_vel_control.return_value = types.SimpleNamespace(success=True, message="ok")
        client.base_move.return_value = types.SimpleNamespace(
            success=base_ok, message="accepted" if base_ok else "rejected", task_id="move_task_1")
        client.move_to_target.return_value = types.SimpleNamespace(
            success=True, message="accepted", task_id="goto_1")
        client.check_arrived.return_value = types.SimpleNamespace(
            success=True, message="", arrived=False, status=1)
        client.publish_base_cmd_vel.return_value = True
        client.last_cmd_vel_status.return_value = None
        host = hardware.JibotChassisMixin()
        host._jibot_client = client
        host.config = {"chassis_nav_auto_handover": handover}
        return host, client

    def test_base_move_maps_options_and_suspends_control(self):
        host, client = self.make()
        options = MoveToTargetOptions(
            avoid_enabled=True, avoid_distance=0.7, linear_velocity=0.2,
            angular_velocity=0.3, position_threshold=0.01, angle_threshold=0.02,
            allow_rotation=False)
        result = host.base_move_relative_jibot(0.1, 0.0, 0.0, options)
        self.assertTrue(result.success)
        self.assertEqual(result.data["task_id"], "move_task_1")
        client.set_vel_control.assert_called_once_with(False)
        client.base_move.assert_called_once_with(
            0.1, 0.0, 0.0, True, 0.7, 0.2, 0.3, 0.01, 0.02, False)

    def test_defaults_follow_ros1_options(self):
        host, client = self.make(handover=False)
        result = host.base_move_to_target_jibot(1.0, 2.0, 0.5)
        self.assertTrue(result.success)
        client.set_vel_control.assert_not_called()
        client.move_to_target.assert_called_once_with(
            1.0, 2.0, 0.5, False, 0.5, 0.15, 0.25, 0.08, 0.1, True)

    def test_invalid_arguments_never_reach_client(self):
        host, client = self.make()
        result = host.base_move_relative_jibot(float("nan"), 0.0, 0.0)
        self.assertFalse(result.success)
        self.assertEqual(result.error_code, "INVALID_ARGUMENT")
        client.base_move.assert_not_called()
        client.set_vel_control.assert_not_called()

    def test_rejected_move_restores_control(self):
        host, client = self.make(base_ok=False)
        result = host.base_move_relative_jibot(0.1, 0.0, 0.0)
        self.assertFalse(result.success)
        self.assertEqual([c.args for c in client.set_vel_control.call_args_list],
                         [(False,), (True,)])

    def test_check_arrived_terminal_restores_control(self):
        host, client = self.make()
        host.base_move_relative_jibot(0.1, 0.0, 0.0)
        client.check_arrived.return_value = types.SimpleNamespace(
            success=True, message="", arrived=True, status=2)
        result = host.check_arrived_jibot("move_task_1")
        self.assertTrue(result.success)
        self.assertTrue(result.data["arrived"])
        self.assertTrue(result.data["vel_control_restored"])
        self.assertEqual([c.args for c in client.set_vel_control.call_args_list],
                         [(False,), (True,)])

    def test_vel_control_state_shape_wait_and_fallbacks(self):
        host, client = self.make(flag=False)
        result = host.get_vel_control_state_jibot()
        self.assertTrue(result.success)
        self.assertEqual(result.data, {"state": False, "source": "topic"})
        client.wait_vel_control_flag.assert_called_once_with(3.0)

        client.wait_vel_control_flag.return_value = None
        result = host.get_vel_control_state_jibot(1.5)
        self.assertEqual(result.data, {"state": False, "source": "last_set"})
        client.wait_vel_control_flag.assert_called_with(1.5)

        fresh, fresh_client = self.make()
        fresh_client.wait_vel_control_flag.return_value = None
        result = fresh.get_vel_control_state_jibot()
        self.assertEqual(result.data, {"state": True, "source": "default"})

        self.assertTrue(host.enable_vel_control_jibot(True).success)
        result = host.get_vel_control_state_jibot()
        self.assertEqual(result.data, {"state": True, "source": "last_set"})

    def test_map_pose_degrees_and_radians(self):
        host, client = self.make()
        client.read_map_pose.return_value = {
            "x": 7.0, "y": 1.3, "qx": 0.0, "qy": 0.0,
            "qz": 0.0, "qw": 1.0, "frame_id": "map", "age_s": 0.2}
        result = host.get_map_pose(max_age_s=1.0)
        self.assertTrue(result.success)
        self.assertAlmostEqual(result.data["x"], 7.0)
        self.assertAlmostEqual(result.data["yaw_deg"], 0.0)
        self.assertAlmostEqual(result.data["yaw_rad"], 0.0)
        client.read_map_pose.assert_called_once_with(1.0)
        client.read_map_pose.return_value = {
            "x": 0.0, "y": 0.0, "qx": 0.0, "qy": 0.0,
            "qz": 0.70710678, "qw": 0.70710678, "frame_id": "", "age_s": 0.1}
        self.assertAlmostEqual(host.get_map_pose().data["yaw_deg"], 90.0, places=4)

    def test_map_pose_missing_reports_localization(self):
        host, client = self.make()
        client.read_map_pose.return_value = None
        result = host.get_map_pose(max_age_s=2.0)
        self.assertFalse(result.success)
        self.assertIn("amcl_pose", result.message)
        client.read_map_pose.assert_called_once_with(2.0)

    def test_stream_lifecycle(self):
        host, client = self.make()
        self.assertTrue(host.start_cmd_vel_stream(rate_hz=200.0).success)
        self.assertTrue(host.start_cmd_vel_stream(rate_hz=200.0).success)  # 幂等
        self.assertTrue(host.update_cmd_vel_setpoint(0.1, 0.0, 0.0).success)
        time.sleep(0.05)
        self.assertTrue(host.stop_cmd_vel_stream(zero_hold_s=0.0).success)
        self.assertIsNone(host._jibot_stream_thread)
        self.assertGreaterEqual(client.publish_base_cmd_vel.call_count, 2)

    def test_motion_gate_defaults_and_status(self):
        host, client = self.make()
        self.assertTrue(host.get_motion_gate()["cmd_vel_effective"])
        client.last_cmd_vel_status.return_value = {
            "cmd_vel_effective": False, "reason": 2,
            "description": "chassis rejected", "age_s": 0.5}
        gate = host.get_motion_gate()
        self.assertFalse(gate["cmd_vel_effective"])
        self.assertEqual((gate["reason"], gate["description"], gate["effective_age_s"]),
                         (2, "chassis rejected", 0.5))

    def test_close_stops_stream_and_client(self):
        host, client = self.make()
        host.start_cmd_vel_stream(rate_hz=100.0)
        host._jibot_close()
        client.close.assert_called_once()
        self.assertIsNone(host._jibot_stream_thread)
        host._jibot_close()  # 幂等
        client.close.assert_called_once()




# ---------------------------------------------------------------------------
# 相机适配器：腕部双相机（16bit 深度 mm）+ 多设备订阅（离线，fake node）
# ---------------------------------------------------------------------------
try:
    from adapters.hardware_ros2.camera_adapter import CameraAdapter as _CameraAdapterProbe
    from sensor_msgs.msg import CameraInfo as _RosCameraInfo
    from sensor_msgs.msg import Image as _RosImage
    _CAMERA_ADAPTER_OK = True
except Exception:  # pragma: no cover - 无 ROS 2 (rclpy) 的环境整体跳过
    _CAMERA_ADAPTER_OK = False


class _FakeCameraNode:
    """记录订阅话题；时钟固定，供 _frame_time 的设备时基回退路径使用。"""

    def __init__(self):
        self.topics = []

    def create_subscription(self, msg_type, topic, callback, qos):
        self.topics.append(topic)
        return ("sub", topic)

    def destroy_subscription(self, sub):
        pass

    class _Clock:
        class _Now:
            nanoseconds = 1_700_000_000_000_000_000

        @staticmethod
        def now():
            return _FakeCameraNode._Clock._Now()

    def get_clock(self):
        return _FakeCameraNode._Clock()


def _make_image(width, height, encoding, stamp_sec, frame_id):
    msg = _RosImage()
    msg.height = height
    msg.width = width
    msg.encoding = encoding
    msg.header.frame_id = frame_id
    msg.header.stamp.sec = int(stamp_sec)
    msg.header.stamp.nanosec = int(round((stamp_sec - int(stamp_sec)) * 1e9))
    if encoding == "rgb8":
        msg.step = width * 3
        msg.data = bytes(bytearray([200] * (width * height * 3)))
    elif encoding == "16uc1":
        msg.step = width * 2
        raw = [1000 + i for i in range(width * height)]
        msg.data = bytes(np.uint16(raw).tobytes())
    else:
        raise ValueError(encoding)
    return msg


@unittest.skipUnless(_CAMERA_ADAPTER_OK, "rclpy/sensor_msgs unavailable (no ROS 2)")
class CameraAdapterWristTests(unittest.TestCase):
    """腕部相机：三路设备订阅、深度 16bit 毫米、别名、开关与硬件时间戳。"""

    def _adapter(self, config=None):
        from adapters.hardware_ros2.camera_adapter import CameraAdapter
        node = _FakeCameraNode()
        adapter = CameraAdapter(node=node)
        result = adapter.initialize(config or {})
        self.assertTrue(result.success, result.message)
        return adapter, node

    def test_three_devices_subscribed_with_default_topics(self):
        adapter, node = self._adapter({})
        self.assertEqual(len(node.topics), 9)
        for slot in ("head_camera", "wrist_left_camera", "wrist_right_camera"):
            for suffix in ("color_image_raw", "color_info", "depth_image_raw"):
                self.assertIn(f"/sensor/{slot}/{suffix}", node.topics)

    def test_wrist_depth_stays_uint16_mm_and_head_float32_m(self):
        adapter, _ = self._adapter({})
        adapter._on_depth("wrist_left_camera",
                          _make_image(848, 480, "16uc1", 1795162865.5, "wrist_left_camera_optical_link"))
        adapter._on_depth("head",
                          _make_image(848, 480, "16uc1", 1795162865.5, "head_camera_optical_link"))
        wrist = adapter.get_depth_data("wrist_left_camera")
        head = adapter.get_depth_data("head")
        self.assertEqual(wrist.depth_image.dtype, np.uint16)
        self.assertEqual(int(wrist.depth_image[0, 0]), 1000)
        self.assertAlmostEqual(wrist.scale, 0.001)
        self.assertEqual(head.depth_image.dtype, np.float32)
        self.assertAlmostEqual(float(head.depth_image[0, 0]), 1.0)
        self.assertAlmostEqual(head.scale, 1.0)

    def test_wrist_frame_carries_mm_depth_and_hw_timestamp(self):
        adapter, _ = self._adapter({})
        stamp = 1795162867.343643136
        adapter._on_color("wrist_left_camera",
                          _make_image(848, 480, "rgb8", stamp, "wrist_left_camera_optical_link"))
        adapter._on_depth("wrist_left_camera",
                          _make_image(848, 480, "16uc1", stamp, "wrist_left_camera_optical_link"))
        frame = adapter.get_camera_frame("wrist_left_camera")
        self.assertEqual(frame.color_image.shape, (480, 848, 3))
        self.assertEqual(frame.depth_image.dtype, np.uint16)
        self.assertEqual(frame.depth_frame_id, "wrist_left_camera_optical_link")
        self.assertAlmostEqual(frame.timestamp, stamp)

    def test_aliases_and_default_device(self):
        adapter, _ = self._adapter({})
        adapter._on_color("wrist_left_camera",
                          _make_image(8, 4, "rgb8", 1795162865.0, "l"))
        adapter._on_color("head",
                          _make_image(8, 4, "rgb8", 1795162865.0, "h"))
        via_alias = adapter.get_camera_frame("wrist_left")
        via_full = adapter.get_camera_frame("wrist_left_camera")
        self.assertEqual(via_alias.color_frame_id, via_full.color_frame_id)
        self.assertEqual(adapter.get_camera_frame().color_frame_id, "h")
        self.assertFalse(adapter.is_connected("wrist_right_camera"))
        # 新鲜度依赖真实时钟，这里只验证别名路径一致（不比较绝对结果）
        self.assertEqual(adapter.is_connected("wrist_left"),
                         adapter.is_connected("wrist_left_camera"))

    def test_enable_flags_control_subscriptions(self):
        from adapters.hardware_ros2.camera_adapter import CameraAdapter
        node = _FakeCameraNode()
        adapter = CameraAdapter(node=node)
        result = adapter.initialize({"enable_wrist_camera": False})
        self.assertTrue(result.success)
        self.assertEqual(len(node.topics), 3)
        self.assertFalse(any("wrist" in t for t in node.topics))

        node2 = _FakeCameraNode()
        adapter2 = CameraAdapter(node=node2)
        result2 = adapter2.initialize({"has_left_wrist": False})
        self.assertTrue(result2.success)
        self.assertTrue(any("wrist_right_camera" in t for t in node2.topics))
        self.assertFalse(any("wrist_left_camera" in t for t in node2.topics))

        node3 = _FakeCameraNode()
        adapter3 = CameraAdapter(node=node3)
        result3 = adapter3.initialize({"enable_head": False})
        self.assertFalse(result3.success)
        self.assertEqual(result3.error_code, "CAMERA_NOT_ENABLED")

    def test_per_slot_topic_override(self):
        adapter, node = self._adapter({
            "wrist_left_camera_color_image_topic": "/cam_l/color"})
        self.assertIn("/cam_l/color", node.topics)
        self.assertIn("/sensor/wrist_left_camera/color_info", node.topics)

    def test_wrist_info_reports_realsense(self):
        adapter, _ = self._adapter({})
        info = _RosCameraInfo()
        info.header.frame_id = "wrist_left_camera_optical_link"
        info.width, info.height = 848, 480
        info.k = [611.0, 0.0, 420.0, 0.0, 611.0, 240.0, 0.0, 0.0, 1.0]
        adapter._on_info("wrist_left_camera", info)
        got = adapter.get_camera_info("wrist_left_camera")
        self.assertEqual(got.camera_type, "realsense")
        self.assertAlmostEqual(got.intrinsics.fx, 611.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
