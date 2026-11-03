import json
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from skills.atomic.refactored_sdk.inference_control_skill import (
    _build_inference_env,
    _load_scenario_runtime_config,
    _sdk_pythonpath_entries,
)


class InferenceSdkPathTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix="inference_sdk_paths_")
        self.root = Path(self.tmp.name)
        self.kuavo = self.root / "kuavo"
        self.sdk = self.kuavo / "src/kuavo_humanoid_sdk"
        package = self.sdk / "kuavo_humanoid_sdk"
        package.mkdir(parents=True)
        (package / "__init__.py").write_text("")
        (package / "msg").mkdir()
        (self.kuavo / "src/kuavo_common/python").mkdir(parents=True)
        self.config = self.root / "config.json"
        self.config.write_text(
            json.dumps(
                {
                    "paths": {
                        "kuavo_root": str(self.kuavo),
                        "learning_python": sys.executable,
                    }
                }
            ),
            encoding="utf-8",
        )

    def tearDown(self):
        self.tmp.cleanup()

    def test_configured_kuavo_sdk_is_added_before_existing_paths(self):
        with patch.dict(
            os.environ,
            {"RING_SIM_CONFIG": str(self.config), "PYTHONPATH": ""},
        ):
            loaded = _load_scenario_runtime_config()
            entries = _sdk_pythonpath_entries(loaded)

        self.assertEqual(entries[0], str(self.sdk))
        self.assertIn(str(self.sdk / "kuavo_humanoid_sdk/msg"), entries)
        self.assertIn(str(self.kuavo / "src/kuavo_common/python"), entries)

    def test_missing_sdk_is_left_for_subprocess_error(self):
        for path in sorted(self.sdk.rglob("*"), reverse=True):
            path.unlink() if path.is_file() else path.rmdir()
        self.sdk.rmdir()

        with patch.dict(
            os.environ,
            {"RING_SIM_CONFIG": str(self.config), "PYTHONPATH": ""},
        ):
            loaded = _load_scenario_runtime_config()
            env = _build_inference_env(loaded, [])

        self.assertNotIn(str(self.sdk), env["PYTHONPATH"].split(os.pathsep))


if __name__ == "__main__":
    unittest.main()
