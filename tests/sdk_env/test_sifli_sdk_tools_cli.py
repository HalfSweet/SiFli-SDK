from __future__ import annotations

import io
import os
import sys
import unittest
from contextlib import redirect_stderr


ROOT = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "tools"))

import sifli_sdk_tools  # noqa: E402


class LegacyCommandDisableTests(unittest.TestCase):
    def test_legacy_env_subcommands_fail_with_migration_message(self) -> None:
        legacy_actions = [
            "export",
            "install-python-env",
            "get-install-python-env",
            "check-python-dependencies",
        ]
        for action in legacy_actions:
            with self.subTest(action=action):
                err = io.StringIO()
                with redirect_stderr(err):
                    with self.assertRaises(SystemExit) as cm:
                        sifli_sdk_tools.main([action])
                self.assertEqual(cm.exception.code, 1)
                output = err.getvalue()
                self.assertIn("no longer supported", output)
                self.assertIn("./install.sh", output)


if __name__ == "__main__":
    unittest.main()
