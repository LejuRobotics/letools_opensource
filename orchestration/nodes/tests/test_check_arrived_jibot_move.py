# -*- coding: utf-8 -*-
"""Tests for mapping JiBot arrival results to behavior-tree statuses."""

import unittest
from unittest.mock import MagicMock, patch

from py_trees.common import Status

from core.domain.result import Result
from orchestration.nodes.check_arrived_jibot_move import CheckArrivedJibotMove

def _run_node(adapter_result):
    hardware = MagicMock()
    hardware.check_arrived_jibot.return_value = adapter_result
    params = {
        "task_id": "goto_task_test",
        "task_id_key": "",
        "blocking": True,
        "timeout": 120.0,
    }
    with patch(
        "orchestration.nodes.check_arrived_jibot_move.get_shared_hardware",
        return_value=hardware,
    ):
        node = CheckArrivedJibotMove("check", "check", "", params)
        node.initialise()
        status = node.update()
    return node, status


class CheckArrivedJibotMoveTest(unittest.TestCase):
    def test_arrived_true_returns_success(self):
        node, status = _run_node(
            Result.ok(
                "check_arrived: arrived",
                data={"arrived": True, "status": 2, "message": "arrived"},
            )
        )

        self.assertEqual(status, Status.SUCCESS)
        self.assertEqual(node.feedback_message, "arrived")

    def test_timeout_result_returns_failure(self):
        node, status = _run_node(
            Result.ok(
                "check_arrived: timeout",
                data={"arrived": False, "status": 0, "message": "timeout"},
            )
        )

        self.assertEqual(status, Status.FAILURE)
        self.assertIn("timeout", node.feedback_message)

    def test_service_failure_returns_failure(self):
        node, status = _run_node(Result.fail("service unavailable"))

        self.assertEqual(status, Status.FAILURE)
        self.assertEqual(node.feedback_message, "service unavailable")


if __name__ == "__main__":
    unittest.main()
