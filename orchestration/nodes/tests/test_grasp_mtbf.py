# -*- coding: utf-8 -*-
"""grasp_mtbf 关键点及整树离线回归测试。"""

import copy
import json
import os
from pathlib import Path
import subprocess
import sys
import textwrap

import pytest
from py_trees.blackboard import Blackboard, Client
from py_trees.common import Access, Status

from orchestration.nodes.grasp_mtbf.compute_key_points import ComputeKeyPoints

pytestmark = pytest.mark.unit

_ROOT = Path(__file__).resolve().parents[3]
_SCENARIO = _ROOT / "orchestration/scenarios/grasp_mtbf"
_STAGES = {
    "pick": ["pick_align", "pick_approach", "pick", "pick_lift"],
    "place": ["place_above", "place", "place_release", "place_retreat"],
}
_EXPECTED_XYZ = {
    "pick_align": [0.77, 0.44, 1.10],
    "pick_approach": [0.92, 0.44, 1.10],
    "pick": [0.92, 0.29, 1.10],
    "pick_lift": [0.77, 0.29, 1.25],
    "place_above": [0.67, 0.29, 1.25],
    "place": [0.82, 0.29, 1.10],
    "place_release": [0.82, 0.44, 1.10],
    "place_retreat": [0.67, 0.44, 1.10],
}


@pytest.fixture
def board(monkeypatch):
    monkeypatch.setattr(Blackboard, "storage", {})
    client = Client(name="grasp_mtbf_test", namespace="/")
    data = json.loads((_SCENARIO / "board.json").read_text(encoding="utf-8"))
    for group in data.values():
        for entry in group:
            client.register_key(key=entry["key"], access=Access.WRITE)
            client.set(entry["key"], copy.deepcopy(entry["value"]))
    return client


def _node(board, phase, runtime=False):
    params = {
        "phase": phase,
        "tag_xyz": board.get(f"virtual_tag_{phase}")[:3],
        "position_offset": board.get(f"{phase}_position_offset"),
        "box_size": board.get("box_size"),
    }
    if runtime:
        params["tag_xyz__board_key"] = "test_tag_xyz"
    return ComputeKeyPoints("compute", "compute", "test", params)


@pytest.mark.parametrize("phase", ["pick", "place"])
def test_stage_coordinates_shared_torso_and_individual_eulers(board, phase):
    # 每组姿态刻意不同，确保不会被统一姿态或其他阶段的姿态覆盖。
    eulers = {}
    for number, stage in enumerate(_STAGES[phase]):
        commands = copy.deepcopy(board.get(stage))
        commands[1]["cmd_vec"][3:] = [number, -90.0, -12.0 - number]
        commands[2]["cmd_vec"][3:] = [-number, -90.0, 12.0 + number]
        board.set(stage, commands)
        eulers[stage] = {cmd["planner_index"]: cmd["cmd_vec"][3:] for cmd in commands[1:]}

    node = _node(board, phase)
    node.initialise()
    assert node.update() == Status.SUCCESS, node.feedback_message
    assert list(node._commands) == _STAGES[phase]
    torso = [0.28, 1.05, 0.0, 0.0] if phase == "pick" else [0.23, 1.05, 0.0, 0.0]
    for stage in _STAGES[phase]:
        commands = {cmd["planner_index"]: cmd["cmd_vec"] for cmd in board.get(stage)}
        assert set(commands) == {2, 6, 7}
        assert commands[2] == pytest.approx(torso)
        x, y, z = _EXPECTED_XYZ[stage]
        assert commands[6][:3] == pytest.approx([x, y, z])
        assert commands[7][:3] == pytest.approx([x, -y, z])
        assert commands[6][3:] == eulers[stage][6]
        assert commands[7][3:] == eulers[stage][7]


@pytest.mark.parametrize("phase", ["pick", "place"])
def test_runtime_tag_is_read_again_and_y_is_centered_on_tag(board, phase):
    node = _node(board, phase, runtime=True)
    node.initialise()
    assert node.update() == Status.FAILURE
    assert "test_tag_xyz 未就绪" in node.feedback_message

    board.register_key(key="test_tag_xyz", access=Access.WRITE)
    board.set("test_tag_xyz", board.get(f"virtual_tag_{phase}")[:3])
    node.initialise()
    assert node.update() == Status.SUCCESS
    original = copy.deepcopy(board.get(phase))

    tag = board.get("test_tag_xyz")
    board.set("test_tag_xyz", [tag[0] + 0.10, tag[1] + 0.20, tag[2] + 0.30])
    node.initialise()
    assert node.update() == Status.SUCCESS
    for old, new in zip(original, board.get(phase)):
        delta = [0.05, 0.15, 0.0, 0.0] if old["planner_index"] == 2 else [0.10, 0.20, 0.30, 0.0, 0.0, 0.0]
        assert new["cmd_vec"] == pytest.approx([value + change for value, change in zip(old["cmd_vec"], delta)])


@pytest.mark.parametrize("size", [[0.0, 0.58, 0.26], [0.34, -0.58, 0.26], [0.34, float("nan"), 0.26]])
def test_invalid_size_does_not_overwrite_commands(board, size):
    node = _node(board, "pick")
    original = {stage: copy.deepcopy(board.get(stage)) for stage in _STAGES["pick"]}
    node.params["box_size"] = size
    node.initialise()
    assert node.update() == Status.FAILURE
    for stage, commands in original.items():
        assert board.get(stage) == commands


@pytest.mark.parametrize("repeat_count", [1, 3])
def test_complete_tree_dry_run_uses_all_eight_updated_command_groups(repeat_count):
    # 独立进程避免公共节点导入时缓存的 dry-run 标志影响其他测试。
    script = textwrap.dedent("""
        import copy
        import json
        import os
        from pathlib import Path
        from unittest.mock import patch

        from py_trees.blackboard import Client
        from py_trees.common import Status
        from apps.test_upper_init.run_behavior_tree_json import _load_board_into_blackboard
        import orchestration.engine.py_trees_compat
        from orchestration.engine.behavior_tree_factory import BehaviorTreeFactory
        from skills.atomic.refactored_sdk.multi_timed_command import MultiTimedCommandSkill

        scenario = Path('orchestration/scenarios/grasp_mtbf')
        board = Client(name='tree_test', namespace='/')
        _load_board_into_blackboard(board, str(scenario / 'board.json'))
        repeat_count = int(os.environ['TEST_REPEAT_COUNT'])
        board.set('repeat_count', repeat_count)
        subtrees = json.loads((scenario / 'py_tree_child.json').read_text())
        stages = {
            'pick': ['pick_align', 'pick_approach', 'pick', 'pick_lift'],
            'place': ['place_above', 'place', 'place_release', 'place_retreat'],
        }
        for phase, keys in stages.items():
            children = subtrees[f'{phase}_one.json']['tree']['childs']
            moves = [node for node in children if node['name'] == 'MultiTimedCommandMove']
            assert [node['params']['commands']['board_key'] for node in moves] == keys
        assert board.get('fallback_to_virtual') is False
        assert board.get('require_fresh_tag') is True

        submitted = []
        original = MultiTimedCommandSkill.on_initialize
        def record(skill, params):
            result = original(skill, params)
            submitted.append(copy.deepcopy(skill.commands))
            return result

        with patch('orchestration.shared_hardware.get_shared_hardware', side_effect=AssertionError('unexpected hardware access')), \
             patch.object(MultiTimedCommandSkill, 'on_initialize', record):
            factory = BehaviorTreeFactory(board, enable_parallel_loading=False,
                subtree_json_path=str(scenario / 'py_tree_child.json'))
            tree = factory.load_tree_from_json(str(scenario / 'py_tree.json'))
            assert tree.root.__class__.__name__ == 'RepeatUntil'
            assert tree.root.max_iterations == repeat_count
            assert tree.root.check_before_iteration
            assert tree.root.wait_for_child_completion
            for number in range(repeat_count):
                tree.tick()
                assert tree.root.iterations == number + 1
                expected = Status.SUCCESS if number + 1 == repeat_count else Status.RUNNING
                assert tree.root.status == expected

        assert tree.root.status == Status.SUCCESS, tree.root.status
        leaves = [node for node in tree.root.iterate() if not node.children]
        moves = [node for node in leaves if node.__class__.__name__ == 'MultiTimedCommandMove']
        assert len(moves) == 11
        assert len(submitted) == 11 * repeat_count
        for node, commands in zip(moves * repeat_count, submitted):
            key = node.params['commands__board_key']
            assert node.status == Status.SUCCESS, (node.label, node.feedback_message)
            assert not node._error, node._error
            expected = board.get(key)
            assert [c['cmd_vec'] for c in commands] == [c['cmd_vec'] for c in expected], key
            assert all(c['desire_time'] == board.get(f'{key}_duration') for c in commands), key
        actual_stages = [node.params['commands__board_key'] for node in moves
            if node.params['commands__board_key'] in stages['pick'] + stages['place']]
        assert actual_stages == stages['pick'] + stages['place'], actual_stages
        assert all(node.status == Status.SUCCESS for node in leaves
            if node.__class__.__name__ == 'ComputeKeyPoints')
        print('Verified: root SUCCESS; eight stages, runtime commands, durations, and perception policy')
    """)
    env = dict(os.environ, STUDIO_DRY_RUN="1", TEST_REPEAT_COUNT=str(repeat_count))
    result = subprocess.run([sys.executable, "-c", script], cwd=_ROOT, env=env,
                            capture_output=True, text=True, timeout=60)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "Verified: root SUCCESS" in result.stdout


@pytest.mark.parametrize("phase", ["pick", "place"])
def test_detection_timeout_stops_tree_before_phase_motion(board, monkeypatch, phase):
    from unittest.mock import MagicMock, patch
    from py_trees.blackboard import Blackboard
    from core.domain.perception import TagDetection
    from core.domain.pose import Pose6D
    import orchestration.engine.py_trees_compat
    from orchestration.engine.behavior_tree_factory import BehaviorTreeFactory
    from orchestration.nodes.chassis_navigation_move import ChassisNavigationMove
    from orchestration.nodes.multi_timed_command_move import MultiTimedCommandMove
    from orchestration.nodes.node_wait_for_blackboard import NodeWaitForBlackboard

    monkeypatch.delenv("STUDIO_DRY_RUN", raising=False)
    board.set("repeat_count", 3)
    hardware = MagicMock()
    hardware.perception.get_tag_detections.return_value = []
    factory = BehaviorTreeFactory(board, enable_parallel_loading=False,
                                  subtree_json_path=str(_SCENARIO / "py_tree_child.json"))
    tree = factory.load_tree_from_json(str(_SCENARIO / "py_tree.json"))
    waits = {node.label: node for node in tree.root.iterate()
             if isinstance(node, NodeWaitForBlackboard)}
    for wait in waits.values():
        assert wait._fallback_to_virtual is False
        assert wait._require_fresh_tag is True
        assert wait.parent.children[0] is wait

    def skip_motion(node):
        node._error = ""

    with patch.object(MultiTimedCommandMove, "initialise", skip_motion), \
         patch.object(MultiTimedCommandMove, "update", return_value=Status.SUCCESS), \
         patch.object(ChassisNavigationMove, "initialise"), \
         patch.object(ChassisNavigationMove, "update", return_value=Status.SUCCESS), \
         patch("orchestration.nodes.node_percep.get_shared_hardware", return_value=hardware), \
         patch("orchestration.shared_hardware.get_shared_hardware", side_effect=AssertionError("unexpected real hardware access")), \
         patch("orchestration.nodes.node_wait_for_blackboard._tag_time", return_value=100.0):
        tree.tick()
        assert tree.root.status == Status.RUNNING
        if phase == "place":
            # 让抓取检测成功，以实际主树顺序进入放置检测。
            wait = waits["wait_for_pick_tag"]
            bb = wait.global_blackboard
            bb.register_key(key="latest_tag_1_version", access=Access.WRITE)
            bb.set("latest_tag_1", TagDetection(1, Pose6D(x=0.7, y=0.0, z=1.1), timestamp=100.0))
            bb.set("latest_tag_1_version", wait._start_version + 1)
            tree.tick()
            assert waits["wait_for_place_tag"].status == Status.RUNNING

        wait = waits[f"wait_for_{phase}_tag"]
        with patch("orchestration.nodes.node_wait_for_blackboard.time.monotonic",
                   return_value=wait._start_time + 5.0), \
             patch.object(wait, "_inject_virtual_tag") as inject:
            tree.tick()
            inject.assert_not_called()
        assert tree.root.status == Status.FAILURE
        assert wait.status == Status.FAILURE
        assert "virtual fallback disabled" in wait.feedback_message
        for node in tree.root.iterate():
            label = getattr(node, "label", node.name)
            if label in [f"compute_{phase}_key_points", f"{phase}_one"]:
                assert node.status == Status.INVALID
            if isinstance(node, MultiTimedCommandMove) and node.params.get("commands__board_key") in _STAGES[phase]:
                assert node.status == Status.INVALID
        assert f"/latest_tag_{1 if phase == 'pick' else 2}_xyz" not in Blackboard.storage
        assert tree.root.iterations == 0


@pytest.mark.parametrize("stop_at", ["none", "before", "during", "between", "failure_second"])
def test_complete_rounds_refresh_tags_and_obey_stop_boundaries(board, monkeypatch, stop_at):
    from unittest.mock import MagicMock
    from core.domain.perception import TagDetection
    from core.domain.pose import Pose6D
    from core.domain.result import Result
    import orchestration.engine.py_trees_compat
    from orchestration.engine.behavior_tree_factory import BehaviorTreeFactory
    import orchestration.nodes.multi_timed_command_move as motion
    from orchestration.nodes.chassis_navigation_move import ChassisNavigationMove
    from orchestration.nodes.node_wait_for_blackboard import NodeWaitForBlackboard

    monkeypatch.delenv("STUDIO_DRY_RUN", raising=False)
    monkeypatch.setattr(motion, "_DRY_RUN", False)
    board.set("repeat_count", 3)
    board.set("mtbf_stop_requested", stop_at == "before")
    hardware = MagicMock()
    hardware.set_arm_control_mode.return_value = Result.ok()
    hardware.set_focus_ee.return_value = Result.ok()
    hardware.set_focus_z.return_value = Result.ok()
    monkeypatch.setattr(motion, "get_shared_hardware", lambda: hardware)
    monkeypatch.setattr("orchestration.nodes.node_percep.get_shared_hardware", lambda: hardware)
    monkeypatch.setattr("orchestration.shared_hardware.get_shared_hardware",
                        lambda: pytest.fail("unexpected real hardware access"))
    monkeypatch.setattr("skills.atomic.refactored_sdk.multi_timed_command.time.sleep", lambda _: None)
    clock = {"now": 100.0}
    monkeypatch.setattr("orchestration.nodes.node_wait_for_blackboard._tag_time", lambda: clock["now"])
    monkeypatch.setattr("orchestration.nodes.node_wait_for_blackboard.time.monotonic", lambda: clock["now"])

    navigation = []
    monkeypatch.setattr(ChassisNavigationMove, "initialise", lambda _: None)

    def navigate(node):
        navigation.append(node.params["target_key"])
        return Status.SUCCESS

    monkeypatch.setattr(ChassisNavigationMove, "update", navigate)
    factory = BehaviorTreeFactory(board, enable_parallel_loading=False,
                                  subtree_json_path=str(_SCENARIO / "py_tree_child.json"))
    tree = factory.load_tree_from_json(str(_SCENARIO / "py_tree.json"))
    waits = {phase: next(node for node in tree.root.iterate()
                        if isinstance(node, NodeWaitForBlackboard) and node.label == f"wait_for_{phase}_tag")
             for phase in _STAGES}
    attempts = {}
    cached = {}

    def detect():
        phase = next((phase for phase, wait in waits.items() if wait.status == Status.RUNNING), None)
        if phase is None:
            return []
        round_number = tree.root.iterations + 1
        key = (round_number, phase)
        attempts[key] = attempts.get(key, 0) + 1
        if stop_at == "during":
            board.set("mtbf_stop_requested", True)
        if attempts[key] == 1:
            return [cached[phase]] if phase in cached else []
        xyz = board.get(f"virtual_tag_{phase}")[:3]
        xyz = [xyz[0] + (round_number - 1) * 0.1, xyz[1], xyz[2]]
        tag = TagDetection(1 if phase == "pick" else 2, Pose6D(x=xyz[0], y=xyz[1], z=xyz[2]),
                           timestamp=clock["now"])
        cached[phase] = tag
        return [tag]

    hardware.perception.get_tag_detections.side_effect = detect
    initialized = []
    original = motion.MultiTimedCommandMove.initialise

    def record(node):
        original(node)
        assert not node._error, node.feedback_message
        initialized.append((node.params["commands__board_key"], copy.deepcopy(node._skill.commands)))

    monkeypatch.setattr(motion.MultiTimedCommandMove, "initialise", record)

    def send(**kwargs):
        if stop_at == "failure_second" and hardware.send_timed_multi_commands.call_count == 12:
            return Result.fail("second round motion failed")
        return Result.ok()

    hardware.send_timed_multi_commands.side_effect = send
    for _ in range(200):
        clock["now"] += 1.0
        tree.tick()
        if tree.root.status in (Status.SUCCESS, Status.FAILURE):
            break
        if stop_at == "between" and tree.root.iterations == 1:
            board.set("mtbf_stop_requested", True)
    else:
        pytest.fail("loop did not terminate within tick budget")

    expected_rounds = 3 if stop_at == "none" else (0 if stop_at == "before" else 1)
    assert tree.root.iterations == expected_rounds
    assert tree.root.status == (Status.FAILURE if stop_at == "failure_second" else Status.SUCCESS)
    expected_keys = ["initial", "pick_observe"] + _STAGES["pick"] + _STAGES["place"] + ["initial"]
    actual_keys = [key for key, _ in initialized]
    assert actual_keys == expected_keys * expected_rounds + (["initial"] if stop_at == "failure_second" else [])
    assert navigation == ["pick_point", "place_point"] * expected_rounds
    assert hardware.send_timed_multi_commands.call_count == len(initialized)
    assert hardware.set_arm_control_mode.call_count == len(initialized)
    assert all(call.args == (2,) for call in hardware.set_arm_control_mode.call_args_list)
    for number in range(expected_rounds):
        for key, commands in initialized[number * 11:(number + 1) * 11]:
            if key in _STAGES["pick"] + _STAGES["place"]:
                xyz = _EXPECTED_XYZ[key]
                by_planner = {command["planner_index"]: command["cmd_vec"] for command in commands}
                assert by_planner[6][:3] == pytest.approx([xyz[0] + number * 0.1, xyz[1], xyz[2]])
        for phase in _STAGES:
            assert attempts[(number + 1, phase)] == 2  # 首次无目标/旧缓存被拒绝，再采集新帧。

