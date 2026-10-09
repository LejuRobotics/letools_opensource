# -*- coding: utf-8 -*-
"""Tests for mapping JiBot arrival results to behavior-tree statuses."""

import unittest
from unittest.mock import MagicMock, patch

from py_trees.common import Status

from core.domain.result import Result
from orchestration.nodes.base_node import BaseAction
from orchestration.nodes.grasp_mtbf_v1.GraspMtbfCheckArrived import GraspMtbfCheckArrived

def _run_node(adapter_result):
    hardware = MagicMock()
    hardware.check_arrived_jibot.return_value = adapter_result
    hardware.enable_vel_control_jibot.return_value = Result.ok()
    params = {
        "task_id": "goto_task_test",
        "task_id_key": "",
        "blocking": True,
        "timeout": 120.0,
    }
    with patch(
        "orchestration.nodes.grasp_mtbf_v1.GraspMtbfCheckArrived.get_scene_io",
        return_value=hardware,
    ):
        with patch.object(
            BaseAction,
            "attach_blackboard_client",
            create=True,
            return_value=MagicMock(),
        ):
            node = GraspMtbfCheckArrived("check", "check", "", params)
        node.initialise()
        status = node.update()
    return node, status, hardware


class GraspMtbfCheckArrivedTest(unittest.TestCase):
    def test_arrived_true_returns_success(self):
        node, status, hardware = _run_node(
            Result.ok(
                "check_arrived: arrived",
                data={"arrived": True, "status": 2, "message": "arrived"},
            )
        )

        self.assertEqual(status, Status.SUCCESS)
        self.assertEqual(node.feedback_message, "arrived")
        hardware.enable_vel_control_jibot.assert_not_called()

    def test_not_arrived_keeps_polling_without_blocking_tree(self):
        node, status, hardware = _run_node(
            Result.ok(
                "check_arrived: timeout",
                data={"arrived": False, "status": 0, "message": "timeout"},
            )
        )

        self.assertEqual(status, Status.RUNNING)
        self.assertIn("等待导航到达", node.feedback_message)
        hardware.check_arrived_jibot.assert_called_once_with(
            task_id="goto_task_test", blocking=False, timeout=0.0
        )
        hardware.enable_vel_control_jibot.assert_not_called()

    def test_service_failure_returns_failure(self):
        node, status, hardware = _run_node(Result.fail("service unavailable"))

        self.assertEqual(status, Status.FAILURE)
        self.assertIn("service unavailable", node.feedback_message)
        hardware.enable_vel_control_jibot.assert_called_once_with(True)

    def test_interrupted_task_returns_failure_and_restores_control(self):
        node, status, hardware = _run_node(
            Result.ok(
                data={
                    "arrived": False,
                    "status": 0,
                    "message": "interrupted",
                }
            )
        )

        self.assertEqual(status, Status.FAILURE)
        self.assertIn("interrupted", node.feedback_message)
        hardware.enable_vel_control_jibot.assert_called_once_with(True)


if __name__ == "__main__":
    unittest.main()


import pytest
pytestmark = pytest.mark.unit
